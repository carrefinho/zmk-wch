/*
 * Copyright (c) 2025 Bootlin
 * Copyright (c) 2026 carrefinho
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * device-next (udc_api) driver for the CH32X03x USBFS device controller.
 *
 * Derived from udc_wch.c, the V20x/V30x driver. Transfer and EP0 handling
 * follow it; the register block differs enough to need its own file:
 *  - Base 0x40023400, IRQ 45/46, RCC gate on AHB bit 12.
 *  - Each endpoint has one control byte, the low byte of UEPn_CTRL_H, holding
 *    both its TX and RX bits (see WCH_TXCTRL below). The V20x/V30x block has
 *    two.
 *  - EP5-7 share one mode register, R8_UEP567_MOD.
 *  - There is no UEP4_DMA, and the LEN/CTRL/DMA registers have a gap after
 *    EP4, so every per-endpoint register is looked up by switch, never by
 *    index arithmetic.
 *  - The DMA registers keep only the low address bits and the SIE adds
 *    0x20000000 back, so all 20 KB of SRAM is reachable. Buffers must be
 *    4-byte aligned (RM 18.2.2.2).
 *
 * One direction per endpoint index: with both RX_EN and TX_EN set on an
 * index, the SIE moves that index's TX buffer to UEPn_DMA+64. Transfers here
 * are armed zero-copy from the net_buf, which only works with TX at +0, so odd
 * indices are offered as IN only and even ones as OUT only, as in the WCH
 * device examples. That leaves 4 IN and 2 OUT endpoints, enough for HID plus
 * CDC-ACM.
 *
 * EP4 is not offered. With no DMA register it shares EP0's buffer
 * (UEP0_DMA+64 OUT, +128 IN), which would need a bounce buffer. Its
 * udc_ep_config is still registered so the index space stays contiguous.
 *
 * EP0 rules carried over from udc_wch.c:
 *  1. IN and OUT share one DMA pointer. Defer all EP0 OUT arming while an EP0
 *     IN is queued; the IN completion re-posts the deferred event.
 *  2. A SETUP can land at any time, and the SIE writes 8 bytes wherever the
 *     EP0 DMA points. Never point it at a zero-length buffer's NULL; park it
 *     on the setup buffer for every zero-length stage.
 *  3. Clear DEV_ADDR in the bus-reset ISR itself, or the first SETUP to
 *     address 0 can be lost.
 *  4. On an IN completion with an empty queue, leave the data toggle alone.
 *
 * Data toggles are driven manually; AUTO_TOG is not used.
 *
 * Not verified on hardware yet: the 5V supply path (wch,vdd-5v), suspend and
 * resume, remote wakeup (host_wakeup is a stub) and ISO transfers.
 */

#include "udc_common.h"

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/clock/ch32v20x_30x-clocks.h>

#include <hal_ch32fun.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(udc_wch_x03x, CONFIG_UDC_DRIVER_LOG_LEVEL);

/*
 * X035-local register constants. The ch32fun header (ch32x03xhw.h) provides
 * most bit names; the INT_ST token field is not named there, so define it.
 * INT_ST bits[5:4] = token PID: 00 OUT, 10 IN, 11 SETUP.
 */
#define WCH_UIS_TOKEN_MASK  0x30U
#define WCH_UIS_TOKEN_OUT   0x00U
#define WCH_UIS_TOKEN_IN    0x20U
#define WCH_UIS_TOKEN_SETUP 0x30U
#define WCH_UIS_ENDP_MASK   0x0FU

/*
 * Endpoint mode register bits, defined here on purpose: ch32fun's ch32x03xhw.h
 * carries the V20x/V30x RB_UEPn_* names, which agree for UEP4_1_MOD and
 * UEP2_3_MOD but are wrong for endpoints 5-7. The V20x/V30x block splits those
 * over R8_UEP5_6_MOD (0x0E) and R8_UEP7_MOD (0x0F); the X03x packs all three
 * into R8_UEP567_MOD at 0x0E with a different bit assignment (RM 18.2.1.10).
 * RB_UEP5_TX_EN would enable endpoint 6 here.
 */
#define WCH_UEP1_TX_EN 0x40U /* R8_UEP4_1_MOD (0x0C) */
#define WCH_UEP1_RX_EN 0x80U
#define WCH_UEP4_TX_EN 0x04U
#define WCH_UEP4_RX_EN 0x08U
#define WCH_UEP2_TX_EN 0x04U /* R8_UEP2_3_MOD (0x0D) */
#define WCH_UEP2_RX_EN 0x08U
#define WCH_UEP3_TX_EN 0x40U
#define WCH_UEP3_RX_EN 0x80U
#define WCH_UEP5_TX_EN 0x01U /* R8_UEP567_MOD (0x0E), X035 packing */
#define WCH_UEP5_RX_EN 0x02U
#define WCH_UEP6_TX_EN 0x04U
#define WCH_UEP6_RX_EN 0x08U
#define WCH_UEP7_TX_EN 0x10U
#define WCH_UEP7_RX_EN 0x20U

/*
 * Odd endpoint indices are IN, even ones OUT, so RX_EN and TX_EN are never set
 * on the same index and the TX buffer stays at UEPn_DMA+0 (see the top of the
 * file).
 */
#define WCH_EP_IDX_IS_IN(idx) (((idx) & 1U) != 0U)

/*
 * Both directions live in the low byte of UEPn_CTRL_H:
 *   T_RES[1:0]  R_RES[3:2]  AUTO_TOG bit4  T_TOG bit6  R_TOG bit7
 * The TX and RX accessors deliberately return the same byte, so a whole-byte
 * write for one direction clobbers the other. Plain assignments go through the
 * masked setters below; read-modify-writes that only touch one direction's
 * bits are already safe. (The same packing is used by the CH32V103 and CH5xx
 * USB blocks.)
 */
#define WCH_TXCTRL(ctrlh_ptr) ((volatile uint8_t *)(ctrlh_ptr) + 0)
#define WCH_RXCTRL(ctrlh_ptr) ((volatile uint8_t *)(ctrlh_ptr) + 0)

#define WCH_UEP_T_SET(reg, val)                                                 \
	((reg) = (uint8_t)(((reg) & ~(USBFS_UEP_T_RES_MASK | USBFS_UEP_T_TOG)) | (val)))
#define WCH_UEP_R_SET(reg, val)                                                 \
	((reg) = (uint8_t)(((reg) & ~(USBFS_UEP_R_RES_MASK | USBFS_UEP_R_TOG)) | (val)))

struct udc_wch_x03x_config {
	USBFS_TypeDef *regs;
	size_t num_of_eps;
	struct udc_ep_config *ep_cfg_in;
	struct udc_ep_config *ep_cfg_out;
	void (*make_thread)(const struct device *dev);
	int speed_idx;
	const struct device *clock_dev;
	uint8_t clock_id_usbfs;
	uint8_t clock_id_afio;
	uint8_t clock_id_gpioc;
	bool vdd_5v;
	void (*irq_enable_func)(const struct device *dev);
};

struct udc_wch_x03x_data {
	struct k_thread thread_data;
	uint32_t setup[2];
};

enum udc_wch_x03x_event_type {
	UDC_WCH_X03X_EVT_XFER,
};

struct udc_wch_x03x_evt {
	enum udc_wch_x03x_event_type type;
	uint8_t ep;
};

K_MSGQ_DEFINE(drv_msgq_x03x, sizeof(struct udc_wch_x03x_evt),
	      CONFIG_UDC_WCH_X03X_MAX_QMESSAGES, sizeof(uint32_t));

/*
 * Per-endpoint register accessors. The gap after EP4 and the missing
 * UEP4_DMA rule out index arithmetic.
 */

/* DMA base pointer for endpoint idx, or NULL if the endpoint has no DMA
 * register (EP4 shares EP0's region; anything else is out of range).
 */
static volatile uint32_t *wch_ep_dma(USBFS_TypeDef *regs, uint8_t idx)
{
	switch (idx) {
	case 0:
		return &regs->UEP0_DMA;
	case 1:
		return &regs->UEP1_DMA;
	case 2:
		return &regs->UEP2_DMA;
	case 3:
		return &regs->UEP3_DMA;
	case 5:
		return &regs->UEP5_DMA;
	case 6:
		return &regs->UEP6_DMA;
	case 7:
		return &regs->UEP7_DMA;
	default:
		return NULL; /* EP4 shares EP0's region and is not offered */
	}
}

static volatile uint16_t *wch_ep_txlen(USBFS_TypeDef *regs, uint8_t idx)
{
	switch (idx) {
	case 0:
		return &regs->UEP0_TX_LEN;
	case 1:
		return &regs->UEP1_TX_LEN;
	case 2:
		return &regs->UEP2_TX_LEN;
	case 3:
		return &regs->UEP3_TX_LEN;
	case 4:
		return &regs->UEP4_TX_LEN;
	case 5:
		return &regs->UEP5_TX_LEN;
	case 6:
		return &regs->UEP6_TX_LEN;
	case 7:
		return &regs->UEP7_TX_LEN;
	default:
		return NULL;
	}
}

/* UEPn_CTRL_H; both directions use its low byte (see WCH_TXCTRL). */
static volatile uint16_t *wch_ep_ctrlh(USBFS_TypeDef *regs, uint8_t idx)
{
	switch (idx) {
	case 0:
		return &regs->UEP0_CTRL_H;
	case 1:
		return &regs->UEP1_CTRL_H;
	case 2:
		return &regs->UEP2_CTRL_H;
	case 3:
		return &regs->UEP3_CTRL_H;
	case 4:
		return &regs->UEP4_CTRL_H;
	case 5:
		return &regs->UEP5_CTRL_H;
	case 6:
		return &regs->UEP6_CTRL_H;
	case 7:
		return &regs->UEP7_CTRL_H;
	default:
		return NULL;
	}
}

/* Resolve the mode register + enable bits for a data endpoint (1..7, not 0/4).
 * Endpoints 5-7 all live in R8_UEP567_MOD; the 0x0F byte after it is reserved.
 */
static int wch_ep_mode(USBFS_TypeDef *regs, uint8_t idx,
		       volatile uint8_t **mode_reg, uint8_t *tx_en, uint8_t *rx_en)
{
	switch (idx) {
	case 1:
		*mode_reg = &regs->UEP4_1_MOD;
		*tx_en = WCH_UEP1_TX_EN;
		*rx_en = WCH_UEP1_RX_EN;
		break;
	case 2:
		*mode_reg = &regs->UEP2_3_MOD;
		*tx_en = WCH_UEP2_TX_EN;
		*rx_en = WCH_UEP2_RX_EN;
		break;
	case 3:
		*mode_reg = &regs->UEP2_3_MOD;
		*tx_en = WCH_UEP3_TX_EN;
		*rx_en = WCH_UEP3_RX_EN;
		break;
	case 5:
		*mode_reg = &regs->UEP567_MOD;
		*tx_en = WCH_UEP5_TX_EN;
		*rx_en = WCH_UEP5_RX_EN;
		break;
	case 6:
		*mode_reg = &regs->UEP567_MOD;
		*tx_en = WCH_UEP6_TX_EN;
		*rx_en = WCH_UEP6_RX_EN;
		break;
	case 7:
		*mode_reg = &regs->UEP567_MOD;
		*tx_en = WCH_UEP7_TX_EN;
		*rx_en = WCH_UEP7_RX_EN;
		break;
	default:
		LOG_ERR("ep idx %u has no mode register", idx);
		return -ENOTSUP;
	}

	return 0;
}

/*
 * ---------------------------------------------------------------------------
 * Transfer machinery (ported policy from udc_wch.c)
 * ---------------------------------------------------------------------------
 */

static void udc_wch_x03x_set_status_buffer(const struct device *dev)
{
	const struct udc_wch_x03x_config *config = dev->config;
	struct udc_wch_x03x_data *priv = udc_get_private(dev);

	config->regs->UEP0_DMA = (uint32_t)(uintptr_t)&priv->setup;
}

static void udc_wch_x03x_handle_setup(const struct device *dev)
{
	struct udc_wch_x03x_data *priv = udc_get_private(dev);
	const struct udc_wch_x03x_config *config = dev->config;
	USBFS_TypeDef *regs = config->regs;

	/* NAK both directions of EP0 until the stack arms the data/status
	 * stage; also clears any lingering STALL if the host is fast.
	 */
	WCH_UEP_T_SET(*WCH_TXCTRL(&regs->UEP0_CTRL_H), USBFS_UEP_T_TOG | USBFS_UEP_T_RES_NAK);
	WCH_UEP_R_SET(*WCH_RXCTRL(&regs->UEP0_CTRL_H), USBFS_UEP_R_TOG | USBFS_UEP_R_RES_NAK);

	struct udc_ep_config *cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_IN);

	cfg->stat.halted = false;
	cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
	cfg->stat.halted = false;

	LOG_HEXDUMP_DBG(priv->setup, 8, "SETUP");
	udc_setup_received(dev, priv->setup);
}

static void udc_wch_x03x_xfer_next(const struct device *dev, const uint8_t ep)
{
	struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, ep);
	const struct udc_wch_x03x_config *config = dev->config;
	USBFS_TypeDef *regs = config->regs;
	const uint8_t idx = USB_EP_GET_IDX(ep);
	struct net_buf *buf;
	int len;

	buf = udc_buf_peek(ep_cfg);
	if (buf == NULL) {
		return;
	}

	if (ep == USB_CONTROL_EP_IN) {
		struct udc_wch_x03x_data *priv = udc_get_private(dev);

		len = MIN(ep_cfg->mps, buf->len);

		/* EP0 rule 2: never arm the shared EP0 DMA at a NULL/zero-length
		 * buffer -- park it on the setup buffer so a SETUP arriving in
		 * the window lands where it belongs.
		 */
		if (len > 0) {
			regs->UEP0_DMA = (uint32_t)(uintptr_t)buf->data;
		} else {
			regs->UEP0_DMA = (uint32_t)(uintptr_t)&priv->setup;
		}
		regs->UEP0_TX_LEN = len;
		WCH_UEP_T_SET(*WCH_TXCTRL(&regs->UEP0_CTRL_H),
			      USBFS_UEP_T_TOG | USBFS_UEP_T_RES_ACK);

		buf->data += len;
		buf->len -= len;
	} else if (ep == USB_CONTROL_EP_OUT) {
		struct udc_wch_x03x_data *priv = udc_get_private(dev);
		struct udc_ep_config *in_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_IN);
		struct udc_buf_info *bi = udc_get_buf_info(buf);

		/* EP0 rule 1: EP0 DMA is shared; defer OUT arming while an EP0 IN
		 * is queued. EP0 IN completion re-posts this event. Host NAKs
		 * meanwhile and retries.
		 */
		if (udc_buf_peek(in_cfg) != NULL) {
			return;
		}

		if (bi->setup) {
			/* Next SETUP always DATA0, lands in the private buffer. */
			regs->UEP0_DMA = (uint32_t)(uintptr_t)&priv->setup;
			WCH_UEP_R_SET(*WCH_RXCTRL(&regs->UEP0_CTRL_H), USBFS_UEP_R_RES_ACK);
		} else if (bi->status) {
			/* Status OUT ZLP: no memory written, keep DMA parked on
			 * the setup buffer. Status stage is always DATA1.
			 */
			regs->UEP0_DMA = (uint32_t)(uintptr_t)&priv->setup;
			WCH_UEP_R_SET(*WCH_RXCTRL(&regs->UEP0_CTRL_H),
				      USBFS_UEP_R_TOG | USBFS_UEP_R_RES_ACK);
		} else {
			/* Data OUT stage, first packet DATA1 (TOG set on SETUP). */
			volatile uint8_t *rx = WCH_RXCTRL(&regs->UEP0_CTRL_H);

			regs->UEP0_DMA = (uint32_t)(uintptr_t)buf->data;
			*rx = (*rx & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_ACK;
		}
	} else if (USB_EP_GET_DIR(ep) == USB_EP_DIR_IN) {
		volatile uint32_t *dma = wch_ep_dma(regs, idx);
		volatile uint16_t *txlen = wch_ep_txlen(regs, idx);
		volatile uint8_t *txctrl = WCH_TXCTRL(wch_ep_ctrlh(regs, idx));

		if (dma == NULL || txlen == NULL) {
			return;
		}

		len = MIN(ep_cfg->mps, buf->len);
		if (len > 0) {
			*dma = (uint32_t)(uintptr_t)buf->data;
		}
		*txlen = len;
		*txctrl = (*txctrl & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_ACK;
		LOG_DBG("arm ep%02x len %d", ep, len);
		buf->data += len;
		buf->len -= len;
	} else {
		volatile uint32_t *dma = wch_ep_dma(regs, idx);
		volatile uint8_t *rxctrl = WCH_RXCTRL(wch_ep_ctrlh(regs, idx));

		if (dma == NULL) {
			return;
		}

		*dma = (uint32_t)(uintptr_t)buf->data;
		*rxctrl = (*rxctrl & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_ACK;
	}
}

static ALWAYS_INLINE void wch_x03x_thread_handler(void *const arg)
{
	const struct device *dev = (const struct device *)arg;

	LOG_DBG("Driver %p thread started", dev);
	while (true) {
		struct udc_wch_x03x_evt evt;
		int ret = k_msgq_get(&drv_msgq_x03x, &evt, K_FOREVER);

		if (ret != 0) {
			continue;
		}

		switch (evt.type) {
		case UDC_WCH_X03X_EVT_XFER:
			udc_wch_x03x_xfer_next(dev, evt.ep);
			break;
		}
	}
}

/* Returns non-zero when a further packet was armed in-ISR (mirrors udc_wch:
 * the caller must then re-ack the transfer flag and return without clobbering).
 */
static int udc_wch_x03x_xfer_in(const struct device *dev)
{
	const struct udc_wch_x03x_config *config = dev->config;
	USBFS_TypeDef *regs = config->regs;
	uint8_t int_status = regs->INT_ST;
	uint8_t ep_idx = int_status & WCH_UIS_ENDP_MASK;
	uint8_t ep = ep_idx | USB_EP_DIR_IN;
	struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, ep);
	struct net_buf *buf;
	int len;

	buf = udc_buf_peek(ep_cfg);

	if (ep == USB_CONTROL_EP_IN) {
		*WCH_TXCTRL(&regs->UEP0_CTRL_H) ^= USBFS_UEP_T_TOG;

		if (unlikely(buf == NULL)) {
			LOG_DBG("ep 0x%02x queue empty", USB_CONTROL_EP_IN);
			return 0;
		}

		if (buf->len != 0) {
			len = MIN(ep_cfg->mps, buf->len);
			regs->UEP0_DMA = (uint32_t)(uintptr_t)buf->data;
			regs->UEP0_TX_LEN = len;
			buf->data += len;
			buf->len -= len;
			return 1;
		}

		if (udc_ep_buf_has_zlp(buf)) {
			struct udc_wch_x03x_data *priv = udc_get_private(dev);

			udc_ep_buf_clear_zlp(buf);
			/* EP0 rule 2: ZLP reads no memory, park DMA on setup buf. */
			regs->UEP0_DMA = (uint32_t)(uintptr_t)&priv->setup;
			regs->UEP0_TX_LEN = 0;
			return 1;
		}

		buf = udc_buf_get(ep_cfg);

		{
			volatile uint8_t *tx = WCH_TXCTRL(&regs->UEP0_CTRL_H);

			*tx = (*tx & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
		}

		/* Give the shared EP0 DMA back to SETUP reception. */
		{
			struct udc_wch_x03x_data *priv = udc_get_private(dev);

			regs->UEP0_DMA = (uint32_t)(uintptr_t)&priv->setup;
		}

		/* EP0 rule 1: process the EP0 OUT arming deferred while this IN
		 * owned the shared DMA.
		 */
		if (udc_buf_peek(udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT)) != NULL) {
			struct udc_wch_x03x_evt evt = {
				.type = UDC_WCH_X03X_EVT_XFER,
				.ep = USB_CONTROL_EP_OUT,
			};

			k_msgq_put(&drv_msgq_x03x, &evt, K_NO_WAIT);
		}

		udc_submit_ep_event(dev, buf, 0);
	} else {
		uint8_t idx = ep_idx;
		volatile uint8_t *txctrl = WCH_TXCTRL(wch_ep_ctrlh(regs, idx));
		volatile uint16_t *txlen = wch_ep_txlen(regs, idx);
		volatile uint32_t *dma = wch_ep_dma(regs, idx);

		if (unlikely(buf == NULL)) {
			/* EP0 rule 4: spurious completion, do not touch toggle. */
			LOG_WRN("ep 0x%02x IN completion, empty queue", ep);
			return 0;
		}

		*txctrl ^= USBFS_UEP_T_TOG;

		if (buf->len != 0) {
			len = MIN(ep_cfg->mps, buf->len);
			if (dma != NULL) {
				*dma = (uint32_t)(uintptr_t)buf->data;
			}
			*txlen = len;
			buf->data += len;
			buf->len -= len;
			return 1;
		}

		if (udc_ep_buf_has_zlp(buf)) {
			udc_ep_buf_clear_zlp(buf);
			*txlen = 0;
			return 1;
		}

		buf = udc_buf_get(ep_cfg);
		*txctrl = (*txctrl & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
		udc_submit_ep_event(dev, buf, 0);
	}

	return 0;
}

static void udc_wch_x03x_xfer_out(const struct device *dev)
{
	const struct udc_wch_x03x_config *config = dev->config;
	USBFS_TypeDef *regs = config->regs;
	uint8_t int_status = regs->INT_ST;
	uint8_t ep = int_status & WCH_UIS_ENDP_MASK;
	struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, ep);
	struct net_buf *buf;
	int len;

	buf = udc_buf_get(ep_cfg);
	LOG_DBG("OUT ep%02x buf %p rxlen %d", ep, buf, (int)regs->RX_LEN);
	if (buf == NULL) {
		udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
		return;
	}

	if (ep == USB_CONTROL_EP_OUT) {
		struct udc_wch_x03x_data *priv = udc_get_private(dev);

		len = regs->RX_LEN;
		net_buf_add(buf, len);
		udc_submit_ep_event(dev, buf, 0);

		/* Rearm EP0 RX for the next SETUP (always DATA0). */
		regs->UEP0_DMA = (uint32_t)(uintptr_t)&priv->setup;
		WCH_UEP_R_SET(*WCH_RXCTRL(&regs->UEP0_CTRL_H), USBFS_UEP_R_RES_ACK);
	} else {
		volatile uint8_t *rxctrl = WCH_RXCTRL(wch_ep_ctrlh(regs, ep));

		*rxctrl ^= USBFS_UEP_R_TOG;
		len = regs->RX_LEN;
		net_buf_add(buf, len);
		udc_submit_ep_event(dev, buf, 0);
	}
}

static void udc_wch_x03x_isr_handler(const struct device *dev)
{
	const struct udc_wch_x03x_config *config = dev->config;
	USBFS_TypeDef *regs = config->regs;
	uint8_t int_flag, int_status;
	int ret;

	int_flag = regs->INT_FG;

	if (int_flag & USBFS_UIE_TRANSFER) {
		int_status = regs->INT_ST;
		if ((int_status & WCH_UIS_TOKEN_MASK) == WCH_UIS_TOKEN_OUT) {
			udc_wch_x03x_xfer_out(dev);
		} else if ((int_status & WCH_UIS_TOKEN_MASK) == WCH_UIS_TOKEN_IN) {
			ret = udc_wch_x03x_xfer_in(dev);
			if (ret) {
				regs->INT_FG = USBFS_UIE_TRANSFER;
				return;
			}
		} else if ((int_status & WCH_UIS_TOKEN_MASK) == WCH_UIS_TOKEN_SETUP) {
			udc_wch_x03x_handle_setup(dev);
		}
		regs->INT_FG = USBFS_UIE_TRANSFER;
	}

	if (int_flag & USBFS_UIE_BUS_RST) {
		if (regs->MIS_ST & USBFS_UMS_BUS_RESET) {
			/* EP0 rule 3: clear DEV_ADDR here, in the ISR. */
			regs->DEV_ADDR = regs->DEV_ADDR & USBFS_UDA_GP_BIT;

			udc_submit_event(dev, UDC_EVT_RESET, 0);

			udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT);
			udc_ep_disable_internal(dev, USB_CONTROL_EP_IN);
			udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT,
					       USB_EP_TYPE_CONTROL, 64, 0);
			udc_ep_enable_internal(dev, USB_CONTROL_EP_IN,
					       USB_EP_TYPE_CONTROL, 64, 0);
		}
		regs->INT_FG = USBFS_UIE_BUS_RST;
	}

	if (int_flag & USBFS_UIE_SUSPEND) {
		if (regs->MIS_ST & USBFS_UMS_SUSPEND) {
			udc_submit_event(dev, UDC_EVT_SUSPEND, 0);
		} else {
			udc_submit_event(dev, UDC_EVT_RESUME, 0);
		}
		regs->INT_FG = USBFS_UIE_SUSPEND;
	}
}

/*
 * ---------------------------------------------------------------------------
 * udc_api surface
 * ---------------------------------------------------------------------------
 */

static int udc_wch_x03x_ep_enqueue(const struct device *dev, struct udc_ep_config *const cfg,
				   struct net_buf *buf)
{
	struct udc_wch_x03x_evt evt;

	udc_buf_put(cfg, buf);

	if (cfg->stat.halted) {
		LOG_DBG("ep 0x%02x halted", cfg->addr);
		return 0;
	}

	evt.ep = cfg->addr;
	evt.type = UDC_WCH_X03X_EVT_XFER;
	k_msgq_put(&drv_msgq_x03x, &evt, K_NO_WAIT);

	return 0;
}

static int udc_wch_x03x_ep_dequeue(const struct device *dev, struct udc_ep_config *const cfg)
{
	unsigned int lock_key = irq_lock();

	udc_ep_cancel_queued(dev, cfg);
	irq_unlock(lock_key);

	return 0;
}

static int udc_wch_x03x_ep_enable(const struct device *dev, struct udc_ep_config *const cfg)
{
	const struct udc_wch_x03x_config *config = dev->config;
	USBFS_TypeDef *regs = config->regs;
	const uint8_t ep = cfg->addr;
	const uint8_t idx = USB_EP_GET_IDX(ep);
	volatile uint8_t *mode_reg;
	uint8_t tx_en, rx_en;
	int ret;

	LOG_DBG("Enable ep 0x%02x", ep);

	if (ep == USB_CONTROL_EP_IN) {
		WCH_UEP_T_SET(*WCH_TXCTRL(&regs->UEP0_CTRL_H), USBFS_UEP_T_RES_NAK);
		udc_wch_x03x_set_status_buffer(dev);
		return 0;
	}
	if (ep == USB_CONTROL_EP_OUT) {
		WCH_UEP_R_SET(*WCH_RXCTRL(&regs->UEP0_CTRL_H), USBFS_UEP_R_RES_ACK);
		return 0;
	}

	ret = wch_ep_mode(regs, idx, &mode_reg, &tx_en, &rx_en);
	if (ret) {
		return ret;
	}

	/* Enabling both directions on one index would move the transmit
	 * buffer to UEPn_DMA+64 and break every IN transfer on it. The caps
	 * advertised in preinit already prevent the stack from asking, so
	 * this is only a backstop against a future caps change.
	 */
	if (USB_EP_DIR_IS_IN(ep) != WCH_EP_IDX_IS_IN(idx)) {
		LOG_ERR("ep 0x%02x direction not available on index %u", ep, idx);
		return -ENOTSUP;
	}

	if (USB_EP_DIR_IS_IN(ep)) {
		*mode_reg |= tx_en;
		WCH_UEP_T_SET(*WCH_TXCTRL(wch_ep_ctrlh(regs, idx)), USBFS_UEP_T_RES_NAK);
	} else {
		*mode_reg |= rx_en;
		WCH_UEP_R_SET(*WCH_RXCTRL(wch_ep_ctrlh(regs, idx)), USBFS_UEP_R_RES_NAK);
	}

	return 0;
}

static int udc_wch_x03x_ep_disable(const struct device *dev, struct udc_ep_config *const cfg)
{
	const struct udc_wch_x03x_config *config = dev->config;
	USBFS_TypeDef *regs = config->regs;
	const uint8_t idx = USB_EP_GET_IDX(cfg->addr);
	volatile uint8_t *mode_reg;
	uint8_t tx_en, rx_en;
	int ret;

	LOG_DBG("Disable ep 0x%02x", cfg->addr);

	if (idx == 0) {
		return 0;
	}

	ret = wch_ep_mode(regs, idx, &mode_reg, &tx_en, &rx_en);
	if (ret) {
		return ret;
	}

	if (USB_EP_DIR_IS_IN(cfg->addr)) {
		*mode_reg &= ~tx_en;
		WCH_UEP_T_SET(*WCH_TXCTRL(wch_ep_ctrlh(regs, idx)), USBFS_UEP_T_RES_NAK);
	} else {
		*mode_reg &= ~rx_en;
		WCH_UEP_R_SET(*WCH_RXCTRL(wch_ep_ctrlh(regs, idx)), USBFS_UEP_R_RES_NAK);
	}

	return 0;
}

static int udc_wch_x03x_ep_set_halt(const struct device *dev, struct udc_ep_config *const cfg)
{
	const struct udc_wch_x03x_config *config = dev->config;
	USBFS_TypeDef *regs = config->regs;
	const uint8_t idx = USB_EP_GET_IDX(cfg->addr);

	LOG_DBG("Set halt ep 0x%02x", cfg->addr);

	if (cfg->addr & USB_EP_DIR_IN) {
		WCH_UEP_T_SET(*WCH_TXCTRL(wch_ep_ctrlh(regs, idx)), USBFS_UEP_T_RES_STALL);
	} else {
		WCH_UEP_R_SET(*WCH_RXCTRL(wch_ep_ctrlh(regs, idx)), USBFS_UEP_R_RES_STALL);
	}

	cfg->stat.halted = true;

	return 0;
}

static int udc_wch_x03x_ep_clear_halt(const struct device *dev, struct udc_ep_config *const cfg)
{
	LOG_DBG("Clear halt ep 0x%02x", cfg->addr);
	cfg->stat.halted = false;

	if (udc_buf_peek(cfg) != NULL) {
		struct udc_wch_x03x_evt evt = {
			.type = UDC_WCH_X03X_EVT_XFER,
			.ep = cfg->addr,
		};

		k_msgq_put(&drv_msgq_x03x, &evt, K_NO_WAIT);
	}

	return 0;
}

static int udc_wch_x03x_set_address(const struct device *dev, const uint8_t addr)
{
	const struct udc_wch_x03x_config *config = dev->config;
	USBFS_TypeDef *regs = config->regs;

	LOG_DBG("Set address %u", addr);
	regs->DEV_ADDR = (regs->DEV_ADDR & USBFS_UDA_GP_BIT) | addr;

	return 0;
}

static int udc_wch_x03x_host_wakeup(const struct device *dev)
{
	LOG_DBG("Remote wakeup from %p", dev);
	/* TODO: drive resume signaling via UDEV_CTRL. */
	return -ENOTSUP;
}

static enum udc_bus_speed udc_wch_x03x_device_speed(const struct device *dev)
{
	ARG_UNUSED(dev);
	return UDC_BUS_SPEED_FS;
}

/* The D-/D+ pads (PC16/PC17) must be configured as inputs before USB_IOEN
 * hands them to the PHY: D- floating, D+ with its pull-up, as the vendor
 * USBFS_GPIO_Init does.
 */
static void udc_wch_x03x_pads_enable(const struct udc_wch_x03x_config *config)
{
	GPIO_TypeDef *gpioc = (GPIO_TypeDef *)GPIOC_BASE;
	uint32_t cfgxr;

	clock_control_on(config->clock_dev,
			 (clock_control_subsys_t)(uintptr_t)config->clock_id_gpioc);

	cfgxr = gpioc->CFGXR;
	/* pin 16 -> nibble 0 (D-, floating), pin 17 -> nibble 1 (D+, pull) */
	cfgxr &= ~((0x0FU << 0) | (0x0FU << 4));
	cfgxr |= (GPIO_CFGLR_IN_FLOAT << 0) | (GPIO_CFGLR_IN_PUPD << 4);
	gpioc->CFGXR = cfgxr;

	/* Select pull-up (not pull-down) on pin 17: BSXR low half sets. */
	gpioc->BSXR = BIT(17 - 16);
}

/* Program the USB PHY analog controls (AFIO->CTLR). */
static void udc_wch_x03x_phy_enable(const struct udc_wch_x03x_config *config)
{
	if (config->vdd_5v) {
		/* 5V VDD: no internal 3.3V PHY LDO, 10K pull-up on D+. */
		AFIO->CTLR = (AFIO->CTLR & ~(UDP_PUE_MASK | UDM_PUE_MASK | USB_PHY_V33)) |
			     UDP_PUE_10K | USB_IOEN;
	} else {
		/* 3.3V VDD (default): enable PHY LDO, 1.5K pull-up on D+. */
		AFIO->CTLR = (AFIO->CTLR & ~(UDP_PUE_MASK | UDM_PUE_MASK)) |
			     USB_PHY_V33 | UDP_PUE_1K5 | USB_IOEN;
	}
}

static void udc_wch_x03x_phy_disable(void)
{
	AFIO->CTLR = AFIO->CTLR & ~(UDP_PUE_MASK | UDM_PUE_MASK | USB_IOEN);
}

static int udc_wch_x03x_enable(const struct device *dev)
{
	const struct udc_wch_x03x_config *config = dev->config;
	USBFS_TypeDef *regs = config->regs;

	LOG_DBG("Enable device %p", dev);

	/* DEV_PU_EN (0x20) is undocumented but required. */
	regs->BASE_CTRL = USBFS_UC_DEV_PU_EN | USBFS_UC_INT_BUSY | USBFS_UC_DMA_EN;
	regs->UDEV_CTRL = RB_UD_PD_DIS | RB_UD_PORT_EN;

	udc_wch_x03x_pads_enable(config);
	udc_wch_x03x_phy_enable(config);

	return 0;
}

static int udc_wch_x03x_disable(const struct device *dev)
{
	const struct udc_wch_x03x_config *config = dev->config;
	USBFS_TypeDef *regs = config->regs;

	LOG_DBG("Disable device %p", dev);

	udc_wch_x03x_phy_disable();
	regs->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;

	return 0;
}

static int udc_wch_x03x_init(const struct device *dev)
{
	const struct udc_wch_x03x_config *config = dev->config;
	USBFS_TypeDef *regs = config->regs;

	/* Gate on the USBFS (AHB bit 12) and AFIO (APB2 bit 0) clocks. The
	 * AFIO clock must be live before AFIO->CTLR is written in enable().
	 */
	clock_control_on(config->clock_dev,
			 (clock_control_subsys_t)(uintptr_t)config->clock_id_afio);
	clock_control_on(config->clock_dev,
			 (clock_control_subsys_t)(uintptr_t)config->clock_id_usbfs);

	/* Reset the SIE, then release; 10 us is enough to settle. */
	regs->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
	k_usleep(10);
	regs->BASE_CTRL = 0;

	regs->DEV_ADDR = 0;
	regs->INT_FG = 0xFF;
	regs->INT_EN = USBFS_UIE_SUSPEND | USBFS_UIE_BUS_RST | USBFS_UIE_TRANSFER;

	if (udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT, USB_EP_TYPE_CONTROL, 64, 0)) {
		LOG_ERR("Failed to enable control OUT endpoint");
		return -EIO;
	}
	if (udc_ep_enable_internal(dev, USB_CONTROL_EP_IN, USB_EP_TYPE_CONTROL, 64, 0)) {
		LOG_ERR("Failed to enable control IN endpoint");
		return -EIO;
	}

	config->irq_enable_func(dev);

	return 0;
}

static int udc_wch_x03x_shutdown(const struct device *dev)
{
	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT)) {
		LOG_ERR("Failed to disable control OUT endpoint");
		return -EIO;
	}
	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_IN)) {
		LOG_ERR("Failed to disable control IN endpoint");
		return -EIO;
	}

	return 0;
}

static int udc_wch_x03x_driver_preinit(const struct device *dev)
{
	const struct udc_wch_x03x_config *config = dev->config;
	struct udc_data *data = dev->data;
	int err;

	k_mutex_init(&data->mutex);

	data->caps.rwup = true;
	data->caps.mps0 = UDC_MPS0_64;

	for (int i = 0; i < config->num_of_eps; i++) {
		config->ep_cfg_out[i].addr = USB_EP_DIR_OUT | i;

		if (i == 0) {
			config->ep_cfg_out[i].caps.control = 1;
			config->ep_cfg_out[i].caps.out = 1;
			config->ep_cfg_out[i].caps.mps = 64;
		} else if (i == 4 || WCH_EP_IDX_IS_IN(i)) {
			/* EP4 has no DMA register, and odd indices are
			 * reserved for IN. Caps stay cleared so the stack's
			 * endpoint allocator skips this one.
			 */
		} else {
			config->ep_cfg_out[i].caps.bulk = 1;
			config->ep_cfg_out[i].caps.interrupt = 1;
			config->ep_cfg_out[i].caps.iso = 1;
			config->ep_cfg_out[i].caps.out = 1;
			config->ep_cfg_out[i].caps.mps = 64;
		}

		err = udc_register_ep(dev, &config->ep_cfg_out[i]);
		if (err != 0) {
			LOG_ERR("Failed to register OUT endpoint %d", i);
			return err;
		}
	}

	for (int i = 0; i < config->num_of_eps; i++) {
		config->ep_cfg_in[i].addr = USB_EP_DIR_IN | i;

		if (i == 0) {
			config->ep_cfg_in[i].caps.control = 1;
			config->ep_cfg_in[i].caps.in = 1;
			config->ep_cfg_in[i].caps.mps = 64;
		} else if (i == 4 || !WCH_EP_IDX_IS_IN(i)) {
			/* EP4 has no DMA register, and even indices are
			 * reserved for OUT.
			 */
		} else {
			config->ep_cfg_in[i].caps.bulk = 1;
			config->ep_cfg_in[i].caps.interrupt = 1;
			config->ep_cfg_in[i].caps.iso = 1;
			config->ep_cfg_in[i].caps.in = 1;
			config->ep_cfg_in[i].caps.mps = 64;
		}

		err = udc_register_ep(dev, &config->ep_cfg_in[i]);
		if (err != 0) {
			LOG_ERR("Failed to register IN endpoint %d", i);
			return err;
		}
	}

	config->make_thread(dev);
	LOG_INF("Device %p (max. speed %d)", dev, config->speed_idx);

	return 0;
}

static void udc_wch_x03x_lock(const struct device *dev)
{
	udc_lock_internal(dev, K_FOREVER);
}

static void udc_wch_x03x_unlock(const struct device *dev)
{
	udc_unlock_internal(dev);
}

static const struct udc_api udc_wch_x03x_api = {
	.lock = udc_wch_x03x_lock,
	.unlock = udc_wch_x03x_unlock,
	.device_speed = udc_wch_x03x_device_speed,
	.init = udc_wch_x03x_init,
	.enable = udc_wch_x03x_enable,
	.disable = udc_wch_x03x_disable,
	.shutdown = udc_wch_x03x_shutdown,
	.set_address = udc_wch_x03x_set_address,
	.host_wakeup = udc_wch_x03x_host_wakeup,
	.ep_enable = udc_wch_x03x_ep_enable,
	.ep_disable = udc_wch_x03x_ep_disable,
	.ep_set_halt = udc_wch_x03x_ep_set_halt,
	.ep_clear_halt = udc_wch_x03x_ep_clear_halt,
	.ep_enqueue = udc_wch_x03x_ep_enqueue,
	.ep_dequeue = udc_wch_x03x_ep_dequeue,
};

#define DT_DRV_COMPAT wch_ch32x035_usbfs

#define UDC_WCH_X03X_DEVICE_DEFINE(n)                                                              \
	K_THREAD_STACK_DEFINE(udc_wch_x03x_stack_##n, CONFIG_UDC_WCH_X03X_STACK_SIZE);             \
                                                                                                   \
	static void udc_wch_x03x_thread_##n(void *dev, void *arg1, void *arg2)                     \
	{                                                                                          \
		wch_x03x_thread_handler(dev);                                                       \
	}                                                                                          \
                                                                                                   \
	static void udc_wch_x03x_make_thread_##n(const struct device *dev)                         \
	{                                                                                          \
		struct udc_wch_x03x_data *priv = udc_get_private(dev);                              \
                                                                                                   \
		k_thread_create(&priv->thread_data, udc_wch_x03x_stack_##n,                         \
				K_THREAD_STACK_SIZEOF(udc_wch_x03x_stack_##n),                     \
				udc_wch_x03x_thread_##n, (void *)dev, NULL, NULL,                  \
				K_PRIO_COOP(CONFIG_UDC_WCH_X03X_THREAD_PRIORITY), K_ESSENTIAL,     \
				K_NO_WAIT);                                                        \
		k_thread_name_set(&priv->thread_data, dev->name);                                  \
	}                                                                                          \
                                                                                                   \
	static struct udc_ep_config ep_cfg_out_##n[DT_INST_PROP(n, num_bidir_endpoints)];          \
	static struct udc_ep_config ep_cfg_in_##n[DT_INST_PROP(n, num_bidir_endpoints)];           \
                                                                                                   \
	static void udc_wch_x03x_irq_enable_func_##n(const struct device *dev)                     \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),                             \
			    udc_wch_x03x_isr_handler, DEVICE_DT_INST_GET(n), 0);                   \
		irq_enable(DT_INST_IRQN(n));                                                        \
	}                                                                                          \
                                                                                                   \
	static const struct udc_wch_x03x_config udc_wch_x03x_config_##n = {                        \
		.regs = (USBFS_TypeDef *)DT_INST_REG_ADDR(n),                                      \
		.num_of_eps = DT_INST_PROP(n, num_bidir_endpoints),                                \
		.ep_cfg_in = ep_cfg_in_##n,                                                        \
		.ep_cfg_out = ep_cfg_out_##n,                                                      \
		.make_thread = udc_wch_x03x_make_thread_##n,                                       \
		.speed_idx = DT_ENUM_IDX(DT_DRV_INST(n), maximum_speed),                           \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_id_usbfs = DT_INST_CLOCKS_CELL_BY_IDX(n, 0, id),                            \
		.clock_id_afio = DT_INST_CLOCKS_CELL_BY_IDX(n, 1, id),                             \
		.clock_id_gpioc = CH32V20X_V30X_CLOCK_IOPC,                                       \
		.vdd_5v = DT_INST_PROP(n, wch_vdd_5v),                                             \
		.irq_enable_func = udc_wch_x03x_irq_enable_func_##n,                               \
	};                                                                                         \
                                                                                                   \
	static struct udc_wch_x03x_data udc_priv_##n = {};                                         \
                                                                                                   \
	static struct udc_data udc_data_##n = {                                                    \
		.mutex = Z_MUTEX_INITIALIZER(udc_data_##n.mutex),                                  \
		.priv = &udc_priv_##n,                                                             \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, udc_wch_x03x_driver_preinit, NULL, &udc_data_##n,                 \
			      &udc_wch_x03x_config_##n, POST_KERNEL,                              \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &udc_wch_x03x_api);

DT_INST_FOREACH_STATUS_OKAY(UDC_WCH_X03X_DEVICE_DEFINE)
