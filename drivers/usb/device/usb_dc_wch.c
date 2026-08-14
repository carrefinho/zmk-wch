/*
 * Copyright (c) 2023 Nicholas Winans
 * Copyright (c) 2026 carrefinho
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Legacy USB device-controller (usb_dc) driver for the WCH full-speed device
 * controllers. One file covers both register blocks:
 *
 *   V20x/V30x  CH32V203 / V303 / V307   "wch,usbfs"           base 0x50000000
 *   X03x       CH32X033 / X035          "wch,ch32x035-usbfs"  base 0x40023400
 *
 * A SoC has exactly one of them, so the block is picked at compile time from
 * devicetree and only the register accessors differ. The device-next drivers
 * needed a file each (udc_wch.c, udc_wch_x03x.c) because they arm transfers
 * zero-copy from a net_buf, which pushes the layout differences into the
 * transfer logic.
 *
 * The X03x path is adapted from the Apache-2.0 zephyrboards driver
 * usb_dc_ch32_usbfs.c by Nicholas Winans.
 *
 * Buffer model
 * ------------
 * The legacy usb_dc API hands the driver a pointer that is only valid for the
 * duration of usb_dc_ep_write(), and expects usb_dc_ep_read() to copy out of
 * driver-owned storage. So every endpoint gets a driver-owned DMA buffer and
 * transfers are memcpy'd in and out. The SIE derives the transmit and receive
 * buffer positions from the single UEPn_DMA pointer:
 *
 *     RX_EN only        : receive  buffer at UEPn_DMA + 0
 *     TX_EN only        : transmit buffer at UEPn_DMA + 0
 *     RX_EN and TX_EN   : receive at UEPn_DMA + 0, transmit at UEPn_DMA + 64
 *
 * Each data endpoint gets a 128-byte buffer with OUT at +0 and IN at +64,
 * which is exactly the hardware layout, and both directions can be used on the
 * same endpoint index.
 *
 * EP0
 * ---
 * EP0 IN and OUT share one UEP0_DMA pointer, and a SETUP can arrive at any
 * time: the SIE writes 8 bytes wherever UEP0_DMA points. Here UEP0_DMA is set
 * once in usb_dc_attach() and never moves. It always points at the driver's
 * ep0 buffer, and the SETUP packet is copied out in the ISR as soon as it
 * lands, so the same buffer can serve the data stage.
 *
 * Data toggles
 * ------------
 * Data endpoints use the hardware AUTO_TOG. EP0 cannot: the control transfer
 * toggle sequence (SETUP=DATA0, first IN/OUT data=DATA1, then alternating,
 * status=DATA1) is driven by the stack, not the SIE, so ep0_tog is tracked in
 * software and applied on every EP0 IN. It is reset to DATA1 on each SETUP.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/dt-bindings/clock/ch32v20x_30x-clocks.h>

#include <hal_ch32fun.h>

#include <zephyr/logging/log.h>

#if DT_HAS_COMPAT_STATUS_OKAY(wch_ch32x035_usbfs)
#define DT_DRV_COMPAT       wch_ch32x035_usbfs
#define WCH_USB_X03X        1
#elif DT_HAS_COMPAT_STATUS_OKAY(wch_usbfs)
#define DT_DRV_COMPAT       wch_usbfs
#define WCH_USB_X03X        0
#else
#error "No enabled WCH USBFS node in devicetree"
#endif

LOG_MODULE_REGISTER(usb_dc_wch, CONFIG_USB_DRIVER_LOG_LEVEL);

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) != 1
#error "usb_dc_wch supports exactly one instance"
#endif

#if WCH_USB_X03X
typedef USBFS_TypeDef wch_usb_regs_t;
#else
typedef USBOTG_FS_TypeDef wch_usb_regs_t;
#endif

#define WCH_USB_REGS ((wch_usb_regs_t *)DT_INST_REG_ADDR(0))

/*
 * The controllers have 8 endpoint indices. The devicetree num-bidir-endpoints
 * property says 16 on the V30x node (it counts directions), so clamp.
 */
#define WCH_USB_NUM_EP MIN(DT_INST_PROP(0, num_bidir_endpoints), 8)

#define WCH_USB_MPS_MAX  64
#define WCH_EP_BUF_SIZE  (2 * WCH_USB_MPS_MAX)
#define WCH_EP_TX_OFFSET WCH_USB_MPS_MAX

/*
 * The X03x block has no UEP4_DMA: endpoint 4 shares EP0's buffer region
 * (UEP0_DMA+64 OUT, +128 IN). Rather than alias EP0's buffer, EP4 is not
 * offered to the stack. EP1-3 and EP5-7 all own a DMA register, which is more
 * endpoints than a HID + CDC-ACM composite needs.
 */
#if WCH_USB_X03X
#define WCH_EP_IS_USABLE(idx) ((idx) != 4U)
#else
#define WCH_EP_IS_USABLE(idx) (true)
#endif

/* INT_ST token field: bits [5:4], 00 OUT, 10 IN, 11 SETUP. */
#define WCH_UIS_TOKEN_MASK  0x30U
#define WCH_UIS_TOKEN_OUT   0x00U
#define WCH_UIS_TOKEN_IN    0x20U
#define WCH_UIS_TOKEN_SETUP 0x30U
#define WCH_UIS_ENDP_MASK   0x0FU

/*
 * Vendor-header naming drift, part one: ch32v20xhw.h spells every bit of this
 * register block USBOTG_* and has no USBFS_* names at all. ch32v30xhw.h
 * defines both spellings; its USBFS_* set belongs to the separate standalone
 * FS device, but the values agree with USBOTG_* for every field this driver
 * touches. The V30x and X03x headers already define these names, so the
 * #ifndef makes this block apply to the V20x only.
 */
#ifndef USBFS_UEP_T_RES_MASK
/* USB_CTRL */
#define USBFS_UC_CLR_ALL     USBOTG_UC_CLR_ALL
#define USBFS_UC_RESET_SIE   USBOTG_UC_RESET_SIE
#define USBFS_UC_DEV_PU_EN   USBOTG_UC_DEV_PU_EN
#define USBFS_UC_DMA_EN      USBOTG_UC_DMA_EN
#define USBFS_UC_INT_BUSY    USBOTG_UC_INT_BUSY
/* UDEV_CTRL / MIS_ST */
#define USBFS_UD_PD_DIS      USBOTG_UD_PD_DIS
#define USBFS_UD_PORT_EN     USBOTG_UD_PORT_EN
#define USBFS_UMS_SUSPEND    USBOTG_UMS_SUSPEND
/* INT_EN / INT_FG (shared bit layout) */
#define USBFS_UIE_BUS_RST    USBOTG_UIE_BUS_RST
#define USBFS_UIE_TRANSFER   USBOTG_UIE_TRANSFER
#define USBFS_UIE_SUSPEND    USBOTG_UIE_SUSPEND
/* UEPn_TX_CTRL / UEPn_RX_CTRL */
#define USBFS_UEP_T_RES_MASK  USBOTG_UEP_T_RES_MASK
#define USBFS_UEP_T_RES_ACK   USBOTG_UEP_T_RES_ACK
#define USBFS_UEP_T_RES_NAK   USBOTG_UEP_T_RES_NAK
#define USBFS_UEP_T_RES_NONE  USBOTG_UEP_T_RES_NONE
#define USBFS_UEP_T_RES_STALL USBOTG_UEP_T_RES_STALL
#define USBFS_UEP_T_TOG       USBOTG_UEP_T_TOG
#define USBFS_UEP_T_AUTO_TOG  USBOTG_UEP_T_AUTO_TOG
#define USBFS_UEP_R_RES_MASK  USBOTG_UEP_R_RES_MASK
#define USBFS_UEP_R_RES_ACK   USBOTG_UEP_R_RES_ACK
#define USBFS_UEP_R_RES_NAK   USBOTG_UEP_R_RES_NAK
#define USBFS_UEP_R_RES_NONE  USBOTG_UEP_R_RES_NONE
#define USBFS_UEP_R_RES_STALL USBOTG_UEP_R_RES_STALL
#define USBFS_UEP_R_TOG       USBOTG_UEP_R_TOG
#define USBFS_UEP_R_AUTO_TOG  USBOTG_UEP_R_AUTO_TOG
/* UEP4_1_MOD / UEP2_3_MOD / UEP5_6_MOD / UEP7_MOD */
#define USBFS_UEP1_TX_EN USBOTG_UEP1_TX_EN
#define USBFS_UEP1_RX_EN USBOTG_UEP1_RX_EN
#define USBFS_UEP2_TX_EN USBOTG_UEP2_TX_EN
#define USBFS_UEP2_RX_EN USBOTG_UEP2_RX_EN
#define USBFS_UEP3_TX_EN USBOTG_UEP3_TX_EN
#define USBFS_UEP3_RX_EN USBOTG_UEP3_RX_EN
#define USBFS_UEP4_TX_EN USBOTG_UEP4_TX_EN
#define USBFS_UEP4_RX_EN USBOTG_UEP4_RX_EN
#define USBFS_UEP5_TX_EN USBOTG_UEP5_TX_EN
#define USBFS_UEP5_RX_EN USBOTG_UEP5_RX_EN
#define USBFS_UEP6_TX_EN USBOTG_UEP6_TX_EN
#define USBFS_UEP6_RX_EN USBOTG_UEP6_RX_EN
#define USBFS_UEP7_TX_EN USBOTG_UEP7_TX_EN
#define USBFS_UEP7_RX_EN USBOTG_UEP7_RX_EN
#endif

/*
 * Part two: ch32v30xhw.h spells the UDEV_CTRL bits USBFS_UD_*; ch32x03xhw.h
 * only has the RB_UD_* forms. (The INT_FG flag bits differ the same way --
 * USBFS_UIF_* vs RB_UIF_* -- but USBFS_UIE_* exists in both headers with
 * identical values, 0x01/0x02/0x04, and INT_FG and INT_EN share a bit layout,
 * so this driver uses the UIE names for both registers and needs no shim
 * there.)
 */
#ifndef USBFS_UD_PD_DIS
#define USBFS_UD_PD_DIS  RB_UD_PD_DIS
#define USBFS_UD_PORT_EN RB_UD_PORT_EN
#endif

#if WCH_USB_X03X
/*
 * Endpoint mode-register bits for the X03x. ch32fun's ch32x03xhw.h carries the
 * V20x/V30x RB_UEPn_* names, which agree for UEP4_1_MOD and UEP2_3_MOD but are
 * wrong for endpoints 5-7. The V20x/V30x block splits those over
 * R8_UEP5_6_MOD and R8_UEP7_MOD; the X03x packs all three into R8_UEP567_MOD
 * at 0x0E with a different bit assignment (RM 18.2.1.10). RB_UEP5_TX_EN would
 * enable endpoint 6 here.
 */
#define WCH_UEP1_TX_EN 0x40U /* UEP4_1_MOD */
#define WCH_UEP1_RX_EN 0x80U
#define WCH_UEP2_TX_EN 0x04U /* UEP2_3_MOD */
#define WCH_UEP2_RX_EN 0x08U
#define WCH_UEP3_TX_EN 0x40U
#define WCH_UEP3_RX_EN 0x80U
#define WCH_UEP5_TX_EN 0x01U /* UEP567_MOD */
#define WCH_UEP5_RX_EN 0x02U
#define WCH_UEP6_TX_EN 0x04U
#define WCH_UEP6_RX_EN 0x08U
#define WCH_UEP7_TX_EN 0x10U
#define WCH_UEP7_RX_EN 0x20U
#endif /* WCH_USB_X03X */

#define USB_OUT_IDX 0
#define USB_IN_IDX  1

struct wch_ep_state {
	usb_dc_ep_callback cb;
	uint16_t mps;
	uint8_t rx_len;
	uint8_t rx_idx;
	bool isochronous;
	bool enabled;
};

struct wch_usb_data {
	struct wch_ep_state ep_state[WCH_USB_NUM_EP][2];
	usb_dc_status_callback status_cb;
	uint8_t addr;
	bool should_set_address;
	bool setup_available;
	bool ep0_tog;
	bool attached;

	/*
	 * EP0 uses one 64-byte buffer for both directions (the SIE does not
	 * apply the +64 transmit offset to the control endpoint). setup_rx is
	 * the copy-out taken in the ISR so ep0 can be reused for the IN data
	 * stage while the stack still has the SETUP pending.
	 */
	uint8_t ep0[WCH_USB_MPS_MAX] __aligned(4);
	uint8_t setup_rx[WCH_USB_MPS_MAX] __aligned(4);
	uint8_t ep_buf[WCH_USB_NUM_EP][WCH_EP_BUF_SIZE] __aligned(4);
};

static struct wch_usb_data dev_data;

struct wch_usb_msg {
	bool ep_event;
	uint32_t type;
	uint8_t ep;
};

K_MSGQ_DEFINE(wch_usb_msgq, sizeof(struct wch_usb_msg), CONFIG_USB_DC_WCH_MAX_QMESSAGES, 4);
K_THREAD_STACK_DEFINE(wch_usb_thread_stack, CONFIG_USB_DC_WCH_STACK_SIZE);
static struct k_thread wch_usb_thread;

/* ------------------------------------------------------------------ */
/* Per-endpoint register accessors                                      */
/* ------------------------------------------------------------------ */

/*
 * The V20x/V30x block is a regular array: UEP0_DMA..UEP7_DMA are contiguous
 * u32, and each endpoint has {u16 TX_LEN, u8 TX_CTRL, u8 RX_CTRL} at stride 4.
 *
 * The X03x block is not. There is no UEP4_DMA, UEP0..UEP3 DMA are contiguous
 * with a gap before UEP5..UEP7, and UEP0..UEP4 LEN/CTRL are contiguous with a
 * 32-byte gap before UEP5..UEP7. So on the X03x every register is resolved by
 * an explicit switch, never by pointer arithmetic.
 */

static volatile uint32_t *wch_ep_dma(uint8_t idx)
{
	wch_usb_regs_t *regs = WCH_USB_REGS;

#if WCH_USB_X03X
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
		return NULL;
	}
#else
	return &regs->UEP0_DMA + idx;
#endif
}

static volatile uint16_t *wch_ep_tx_len(uint8_t idx)
{
	wch_usb_regs_t *regs = WCH_USB_REGS;

#if WCH_USB_X03X
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
#else
	return &regs->UEP0_TX_LEN + 2 * idx;
#endif
}

/*
 * Control-register access. The V20x/V30x block has two bytes per endpoint;
 * the X03x packs both directions into the low byte of UEPn_CTRL_H:
 *
 *     T_RES[1:0]  R_RES[3:2]  AUTO_TOG bit4  T_TOG bit6  R_TOG bit7
 *
 * so on the X03x the TX and RX accessors deliberately return the same byte.
 * Every write in this driver is a read-modify-write that touches only one
 * direction's RES/TOG bits, which is safe on both blocks; a whole-byte
 * assignment would clobber the other direction on the X03x.
 */

static volatile uint8_t *wch_ep_tx_ctrl(uint8_t idx)
{
	wch_usb_regs_t *regs = WCH_USB_REGS;

#if WCH_USB_X03X
	switch (idx) {
	case 0:
		return (volatile uint8_t *)&regs->UEP0_CTRL_H;
	case 1:
		return (volatile uint8_t *)&regs->UEP1_CTRL_H;
	case 2:
		return (volatile uint8_t *)&regs->UEP2_CTRL_H;
	case 3:
		return (volatile uint8_t *)&regs->UEP3_CTRL_H;
	case 4:
		return (volatile uint8_t *)&regs->UEP4_CTRL_H;
	case 5:
		return (volatile uint8_t *)&regs->UEP5_CTRL_H;
	case 6:
		return (volatile uint8_t *)&regs->UEP6_CTRL_H;
	case 7:
		return (volatile uint8_t *)&regs->UEP7_CTRL_H;
	default:
		return NULL;
	}
#else
	return &regs->UEP0_TX_CTRL + 4 * idx;
#endif
}

static volatile uint8_t *wch_ep_rx_ctrl(uint8_t idx)
{
#if WCH_USB_X03X
	/* Same byte as TX -- see the comment above. */
	return wch_ep_tx_ctrl(idx);
#else
	wch_usb_regs_t *regs = WCH_USB_REGS;

	return &regs->UEP0_RX_CTRL + 4 * idx;
#endif
}

static void wch_ep_set_tx_res(uint8_t idx, uint8_t res)
{
	volatile uint8_t *reg = wch_ep_tx_ctrl(idx);

	*reg = (*reg & ~USBFS_UEP_T_RES_MASK) | res;
}

static void wch_ep_set_rx_res(uint8_t idx, uint8_t res)
{
	volatile uint8_t *reg = wch_ep_rx_ctrl(idx);

	*reg = (*reg & ~USBFS_UEP_R_RES_MASK) | res;
}

static uint8_t *wch_ep_buffer(uint8_t idx, bool is_in)
{
	if (idx == 0) {
		return dev_data.ep0;
	}

	return &dev_data.ep_buf[idx][is_in ? WCH_EP_TX_OFFSET : 0];
}

/* ------------------------------------------------------------------ */
/* Interrupt service routine                                            */
/* ------------------------------------------------------------------ */

static void wch_usb_isr(const struct device *dev)
{
	wch_usb_regs_t *regs = WCH_USB_REGS;
	struct wch_usb_msg msg = {.ep = 0, .ep_event = false};
	uint8_t status = regs->INT_FG;

	ARG_UNUSED(dev);

	if (status & USBFS_UIE_TRANSFER) {
		uint8_t int_st = regs->INT_ST;
		uint8_t idx = int_st & WCH_UIS_ENDP_MASK;
		uint8_t token = int_st & WCH_UIS_TOKEN_MASK;

		switch (token) {
		case WCH_UIS_TOKEN_OUT:
			wch_ep_set_rx_res(idx, USBFS_UEP_R_RES_NAK);
			if (idx == 0) {
				/*
				 * Some hosts send the next packet before we
				 * respond; take a copy so the EP0 DMA buffer is
				 * free to be reused immediately.
				 */
				memcpy(dev_data.setup_rx, dev_data.ep0, WCH_USB_MPS_MAX);
			}
			dev_data.ep_state[idx][USB_OUT_IDX].rx_len = regs->RX_LEN;
			dev_data.ep_state[idx][USB_OUT_IDX].rx_idx = 0;
			msg.ep = idx | USB_EP_DIR_OUT;
			msg.type = USB_DC_EP_DATA_OUT;
			msg.ep_event = true;
			k_msgq_put(&wch_usb_msgq, &msg, K_NO_WAIT);
			break;

		case WCH_UIS_TOKEN_IN:
			wch_ep_set_tx_res(idx, USBFS_UEP_T_RES_NAK);
			/*
			 * SET_ADDRESS takes effect only after the status stage
			 * completes, i.e. on this IN completion.
			 */
			if (dev_data.should_set_address) {
				regs->DEV_ADDR = dev_data.addr;
				dev_data.should_set_address = false;
			}
			msg.ep = idx | USB_EP_DIR_IN;
			msg.type = USB_DC_EP_DATA_IN;
			msg.ep_event = true;
			k_msgq_put(&wch_usb_msgq, &msg, K_NO_WAIT);
			break;

		case WCH_UIS_TOKEN_SETUP:
			/* Clear any stall as well, in case the host is fast. */
			wch_ep_set_tx_res(0, USBFS_UEP_T_RES_NAK);
			wch_ep_set_rx_res(0, USBFS_UEP_R_RES_NAK);
			memcpy(dev_data.setup_rx, dev_data.ep0, WCH_USB_MPS_MAX);
			/* Data stage after SETUP is always DATA1. */
			dev_data.ep0_tog = true;
			dev_data.setup_available = true;
			msg.ep = USB_CONTROL_EP_OUT;
			msg.type = USB_DC_EP_SETUP;
			msg.ep_event = true;
			k_msgq_put(&wch_usb_msgq, &msg, K_NO_WAIT);
			break;

		default:
			break;
		}

		regs->INT_FG = USBFS_UIE_TRANSFER;
	} else if (status & USBFS_UIE_BUS_RST) {
		/*
		 * Clear the device address inside the ISR, not after a thread
		 * hop: the controller does not do it in hardware, and the
		 * first SETUP addressed to 0 after the reset would be lost.
		 */
		regs->DEV_ADDR = 0;
		dev_data.should_set_address = false;
		dev_data.ep0_tog = true;
		dev_data.ep_state[0][USB_OUT_IDX].mps = WCH_USB_MPS_MAX;
		dev_data.ep_state[0][USB_IN_IDX].mps = WCH_USB_MPS_MAX;

		wch_ep_set_tx_res(0, USBFS_UEP_T_RES_NAK);
		wch_ep_set_rx_res(0, USBFS_UEP_R_RES_ACK);

		msg.type = USB_DC_RESET;
		k_msgq_put(&wch_usb_msgq, &msg, K_NO_WAIT);
		regs->INT_FG = USBFS_UIE_BUS_RST;
	} else if (status & USBFS_UIE_SUSPEND) {
		/* MIS_ST SUSPEND distinguishes suspend from resume. */
		msg.type = (regs->MIS_ST & USBFS_UMS_SUSPEND) ? USB_DC_SUSPEND : USB_DC_RESUME;
		k_msgq_put(&wch_usb_msgq, &msg, K_NO_WAIT);
		regs->INT_FG = USBFS_UIE_SUSPEND;
	} else {
		regs->INT_FG = status;
	}
}

/* ------------------------------------------------------------------ */
/* usb_dc API                                                           */
/* ------------------------------------------------------------------ */

static void wch_usb_pads_enable(void)
{
#if WCH_USB_X03X
	GPIO_TypeDef *gpioc = (GPIO_TypeDef *)GPIOC_BASE;
	uint32_t cfgxr;

	/*
	 * PC16/PC17 are the D-/D+ pads and must be configured as inputs before
	 * USB_IOEN is asserted. They live in the CFGXR nibbles for pins 16-23.
	 */
	cfgxr = gpioc->CFGXR;
	cfgxr &= ~((0x0FU << 0) | (0x0FU << 4));
	cfgxr |= (GPIO_CFGLR_IN_FLOAT << 0) | (GPIO_CFGLR_IN_PUPD << 4);
	gpioc->CFGXR = cfgxr;

	/* Select pull-up (not pull-down) on PC17: BSXR low half sets. */
	gpioc->BSXR = BIT(17 - 16);

	if (IS_ENABLED(CONFIG_USB_DC_WCH_VDD_5V)) {
		/* 5V VDD: no internal 3.3V PHY LDO, 10K pull-up on D+. */
		AFIO->CTLR = (AFIO->CTLR & ~(UDP_PUE_MASK | UDM_PUE_MASK | USB_PHY_V33)) |
			     UDP_PUE_10K | USB_IOEN;
	} else {
		/* 3.3V VDD (default): enable PHY LDO, 1.5K pull-up on D+. */
		AFIO->CTLR = (AFIO->CTLR & ~(UDP_PUE_MASK | UDM_PUE_MASK)) | USB_PHY_V33 |
			     UDP_PUE_1K5 | USB_IOEN;
	}
#endif
}

static void wch_usb_ep_mod_init(void)
{
	wch_usb_regs_t *regs = WCH_USB_REGS;

	/*
	 * Enable both directions on every usable endpoint. With RX_EN and
	 * TX_EN both set the SIE places RX at UEPn_DMA+0 and TX at +64, which
	 * is exactly how wch_ep_buffer() lays out ep_buf[].
	 */
#if WCH_USB_X03X
	regs->UEP4_1_MOD = WCH_UEP1_TX_EN | WCH_UEP1_RX_EN;
	regs->UEP2_3_MOD = WCH_UEP2_TX_EN | WCH_UEP2_RX_EN | WCH_UEP3_TX_EN | WCH_UEP3_RX_EN;
	regs->UEP567_MOD = WCH_UEP5_TX_EN | WCH_UEP5_RX_EN | WCH_UEP6_TX_EN | WCH_UEP6_RX_EN |
			   WCH_UEP7_TX_EN | WCH_UEP7_RX_EN;
#else
	regs->UEP4_1_MOD = USBFS_UEP1_TX_EN | USBFS_UEP1_RX_EN | USBFS_UEP4_TX_EN |
			   USBFS_UEP4_RX_EN;
	regs->UEP2_3_MOD = USBFS_UEP2_TX_EN | USBFS_UEP2_RX_EN | USBFS_UEP3_TX_EN |
			   USBFS_UEP3_RX_EN;
	regs->UEP5_6_MOD = USBFS_UEP5_TX_EN | USBFS_UEP5_RX_EN | USBFS_UEP6_TX_EN |
			   USBFS_UEP6_RX_EN;
	regs->UEP7_MOD = USBFS_UEP7_TX_EN | USBFS_UEP7_RX_EN;
#endif
}

int usb_dc_attach(void)
{
	wch_usb_regs_t *regs = WCH_USB_REGS;

	if (dev_data.attached) {
		return 0;
	}

	/* Reset the SIE and clear all FIFOs, then bring it up. */
	regs->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
	k_busy_wait(10);
	/* DEV_PU_EN is undocumented but required for the device to appear. */
	regs->BASE_CTRL = USBFS_UC_DEV_PU_EN | USBFS_UC_INT_BUSY | USBFS_UC_DMA_EN;
	regs->UDEV_CTRL = USBFS_UD_PD_DIS | USBFS_UD_PORT_EN;
	regs->DEV_ADDR = 0;
	regs->INT_FG = 0xFF;
	regs->INT_EN = USBFS_UIE_SUSPEND | USBFS_UIE_TRANSFER | USBFS_UIE_BUS_RST;

	/*
	 * EP0's DMA pointer is programmed once here and never moves again --
	 * this is what makes the whole class of EP0 aliasing bugs impossible.
	 */
	*wch_ep_dma(0) = (uint32_t)dev_data.ep0;
	*wch_ep_tx_len(0) = 0;
	wch_ep_set_tx_res(0, USBFS_UEP_T_RES_NAK);
	wch_ep_set_rx_res(0, USBFS_UEP_R_RES_ACK);
	dev_data.ep_state[0][USB_OUT_IDX].mps = WCH_USB_MPS_MAX;
	dev_data.ep_state[0][USB_IN_IDX].mps = WCH_USB_MPS_MAX;

	wch_usb_ep_mod_init();

	for (uint8_t idx = 1; idx < WCH_USB_NUM_EP; idx++) {
		volatile uint8_t *ctrl;

		if (!WCH_EP_IS_USABLE(idx)) {
			continue;
		}

		*wch_ep_dma(idx) = (uint32_t)dev_data.ep_buf[idx];
		*wch_ep_tx_len(idx) = 0;

		/*
		 * Data endpoints use the hardware auto-toggle; NAK both
		 * directions until the class driver configures them.
		 */
		ctrl = wch_ep_tx_ctrl(idx);
		*ctrl = USBFS_UEP_T_AUTO_TOG | USBFS_UEP_T_RES_NAK | USBFS_UEP_R_RES_NAK;
#if !WCH_USB_X03X
		*wch_ep_rx_ctrl(idx) = USBFS_UEP_R_AUTO_TOG | USBFS_UEP_R_RES_NAK;
#endif
	}

	wch_usb_pads_enable();

	dev_data.attached = true;

	return 0;
}

int usb_dc_detach(void)
{
	wch_usb_regs_t *regs = WCH_USB_REGS;

#if WCH_USB_X03X
	/* Drop the D+ pull-up so the host sees a disconnect. */
	AFIO->CTLR &= ~UDP_PUE_MASK;
#else
	regs->UDEV_CTRL &= ~USBFS_UD_PORT_EN;
#endif
	regs->INT_EN = 0;
	regs->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;

	dev_data.attached = false;

	return 0;
}

int usb_dc_reset(void)
{
	usb_dc_detach();

	return usb_dc_attach();
}

int usb_dc_set_address(const uint8_t addr)
{
	/*
	 * The address must not take effect until the status stage of the
	 * SET_ADDRESS transfer has completed, so latch it and program
	 * DEV_ADDR on the next EP0 IN completion.
	 */
	dev_data.addr = addr;
	dev_data.should_set_address = true;

	return 0;
}

int usb_dc_ep_check_cap(const struct usb_dc_ep_cfg_data *const cfg)
{
	uint8_t idx = USB_EP_GET_IDX(cfg->ep_addr);

	if ((cfg->ep_type == USB_DC_EP_CONTROL) != (idx == 0)) {
		return -ENOTSUP;
	}

	if (idx >= WCH_USB_NUM_EP || !WCH_EP_IS_USABLE(idx)) {
		return -ENOTSUP;
	}

	if (cfg->ep_mps > WCH_USB_MPS_MAX) {
		return -ENOTSUP;
	}

	return 0;
}

int usb_dc_ep_configure(const struct usb_dc_ep_cfg_data *const ep_cfg)
{
	uint8_t idx = USB_EP_GET_IDX(ep_cfg->ep_addr);
	bool is_in = USB_EP_GET_DIR(ep_cfg->ep_addr) == USB_EP_DIR_IN;
	struct wch_ep_state *state;

	if (idx >= WCH_USB_NUM_EP || !WCH_EP_IS_USABLE(idx)) {
		return -EINVAL;
	}

	state = &dev_data.ep_state[idx][is_in];
	state->isochronous = ep_cfg->ep_type == USB_DC_EP_ISOCHRONOUS;
	state->mps = MIN(ep_cfg->ep_mps, WCH_USB_MPS_MAX);

	if (idx == 0) {
		return 0;
	}

	if (is_in) {
		volatile uint8_t *reg = wch_ep_tx_ctrl(idx);

		*wch_ep_tx_len(idx) = 0;
		*reg = (*reg & ~(USBFS_UEP_T_TOG | USBFS_UEP_T_RES_MASK)) | USBFS_UEP_T_RES_NAK;
	} else {
		volatile uint8_t *reg = wch_ep_rx_ctrl(idx);
		uint8_t res = state->isochronous ? USBFS_UEP_R_RES_NONE : USBFS_UEP_R_RES_ACK;

		*reg = (*reg & ~(USBFS_UEP_R_TOG | USBFS_UEP_R_RES_MASK)) | res;
	}

	return 0;
}

int usb_dc_ep_enable(const uint8_t ep)
{
	uint8_t idx = USB_EP_GET_IDX(ep);
	bool is_in = USB_EP_GET_DIR(ep) == USB_EP_DIR_IN;

	if (idx >= WCH_USB_NUM_EP || !WCH_EP_IS_USABLE(idx)) {
		return -EINVAL;
	}

	dev_data.ep_state[idx][is_in].enabled = true;

	/* OUT endpoints must be armed to receive as soon as they are enabled. */
	if (!is_in && idx != 0) {
		struct wch_ep_state *state = &dev_data.ep_state[idx][USB_OUT_IDX];

		wch_ep_set_rx_res(idx, state->isochronous ? USBFS_UEP_R_RES_NONE
							  : USBFS_UEP_R_RES_ACK);
	}

	return 0;
}

int usb_dc_ep_disable(const uint8_t ep)
{
	uint8_t idx = USB_EP_GET_IDX(ep);
	bool is_in = USB_EP_GET_DIR(ep) == USB_EP_DIR_IN;

	if (idx >= WCH_USB_NUM_EP || !WCH_EP_IS_USABLE(idx)) {
		return -EINVAL;
	}

	dev_data.ep_state[idx][is_in].enabled = false;

	if (is_in) {
		wch_ep_set_tx_res(idx, USBFS_UEP_T_RES_NAK);
	} else {
		wch_ep_set_rx_res(idx, USBFS_UEP_R_RES_NAK);
	}

	return 0;
}

int usb_dc_ep_set_stall(const uint8_t ep)
{
	uint8_t idx = USB_EP_GET_IDX(ep);

	if (idx >= WCH_USB_NUM_EP) {
		return -EINVAL;
	}

	if (USB_EP_DIR_IS_IN(ep)) {
		wch_ep_set_tx_res(idx, USBFS_UEP_T_RES_STALL);
	} else {
		wch_ep_set_rx_res(idx, USBFS_UEP_R_RES_STALL);
	}

	return 0;
}

int usb_dc_ep_clear_stall(const uint8_t ep)
{
	uint8_t idx = USB_EP_GET_IDX(ep);

	if (idx >= WCH_USB_NUM_EP) {
		return -EINVAL;
	}

	/* Clearing a stall also resets the data toggle to DATA0. */
	if (USB_EP_DIR_IS_IN(ep)) {
		volatile uint8_t *reg = wch_ep_tx_ctrl(idx);

		*reg = (*reg & ~(USBFS_UEP_T_TOG | USBFS_UEP_T_RES_MASK)) | USBFS_UEP_T_RES_NAK;
	} else {
		volatile uint8_t *reg = wch_ep_rx_ctrl(idx);

		*reg = (*reg & ~(USBFS_UEP_R_TOG | USBFS_UEP_R_RES_MASK)) | USBFS_UEP_R_RES_ACK;
	}

	return 0;
}

int usb_dc_ep_is_stalled(const uint8_t ep, uint8_t *const stalled)
{
	uint8_t idx = USB_EP_GET_IDX(ep);

	if (idx >= WCH_USB_NUM_EP || stalled == NULL) {
		return -EINVAL;
	}

	if (USB_EP_DIR_IS_IN(ep)) {
		*stalled = (*wch_ep_tx_ctrl(idx) & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_STALL;
	} else {
		*stalled = (*wch_ep_rx_ctrl(idx) & USBFS_UEP_R_RES_MASK) == USBFS_UEP_R_RES_STALL;
	}

	return 0;
}

int usb_dc_ep_halt(const uint8_t ep)
{
	return usb_dc_ep_set_stall(ep);
}

int usb_dc_ep_flush(const uint8_t ep)
{
	uint8_t idx = USB_EP_GET_IDX(ep);

	if (idx >= WCH_USB_NUM_EP) {
		return -EINVAL;
	}

	if (USB_EP_DIR_IS_IN(ep)) {
		*wch_ep_tx_len(idx) = 0;
	} else {
		dev_data.ep_state[idx][USB_OUT_IDX].rx_len = 0;
		dev_data.ep_state[idx][USB_OUT_IDX].rx_idx = 0;
	}

	return 0;
}

int usb_dc_ep_write(const uint8_t ep, const uint8_t *const data, const uint32_t data_len,
		    uint32_t *const ret_bytes)
{
	uint8_t idx = USB_EP_GET_IDX(ep);
	uint32_t write_cnt;

	if (USB_EP_DIR_IS_OUT(ep)) {
		return -EINVAL;
	}

	if (idx >= WCH_USB_NUM_EP || !WCH_EP_IS_USABLE(idx)) {
		return -EINVAL;
	}

	write_cnt = MIN(data_len, dev_data.ep_state[idx][USB_IN_IDX].mps);

	if (ret_bytes == NULL && data_len != write_cnt) {
		LOG_ERR("ep 0x%02x: cannot write %u bytes in one call", ep, data_len);
		return -ENOTSUP;
	}

	if (write_cnt != 0) {
		memcpy(wch_ep_buffer(idx, true), data, write_cnt);
	}

	*wch_ep_tx_len(idx) = write_cnt;

	if (idx == 0) {
		/*
		 * EP0 cannot use AUTO_TOG: the control transfer toggle
		 * sequence is driven by the stack, so apply it by hand.
		 */
		volatile uint8_t *reg = wch_ep_tx_ctrl(0);

		*reg = (*reg & ~(USBFS_UEP_T_RES_MASK | USBFS_UEP_T_TOG)) | USBFS_UEP_T_RES_ACK |
		       (dev_data.ep0_tog ? USBFS_UEP_T_TOG : 0);
		dev_data.ep0_tog = !dev_data.ep0_tog;
	} else {
		wch_ep_set_tx_res(idx, dev_data.ep_state[idx][USB_IN_IDX].isochronous
					       ? USBFS_UEP_T_RES_NONE
					       : USBFS_UEP_T_RES_ACK);
	}

	if (ret_bytes != NULL) {
		*ret_bytes = write_cnt;
	}

	return 0;
}

int usb_dc_ep_read_wait(uint8_t ep, uint8_t *data, uint32_t max_data_len, uint32_t *read_bytes)
{
	uint8_t idx = USB_EP_GET_IDX(ep);
	struct wch_ep_state *state;
	uint32_t read_cnt;
	const uint8_t *src;

	if (USB_EP_DIR_IS_IN(ep)) {
		return -EINVAL;
	}

	if (idx >= WCH_USB_NUM_EP || !WCH_EP_IS_USABLE(idx)) {
		return -EINVAL;
	}

	state = &dev_data.ep_state[idx][USB_OUT_IDX];

	if (idx == 0 && dev_data.setup_available) {
		read_cnt = sizeof(struct usb_setup_packet);
	} else {
		read_cnt = state->rx_len - state->rx_idx;
	}

	/* Query form: report how many bytes are available. */
	if (data == NULL && max_data_len == 0) {
		if (read_bytes != NULL) {
			*read_bytes = read_cnt;
		}
		return 0;
	}

	read_cnt = MIN(read_cnt, max_data_len);

	if (idx == 0 && dev_data.setup_available) {
		if (read_cnt != 0) {
			dev_data.setup_available = false;
		}
		memcpy(data, dev_data.setup_rx, read_cnt);
	} else {
		src = (idx == 0) ? dev_data.setup_rx : wch_ep_buffer(idx, false);
		memcpy(data, src + state->rx_idx, read_cnt);
		state->rx_idx += read_cnt;
	}

	if (read_bytes != NULL) {
		*read_bytes = read_cnt;
	}

	return 0;
}

int usb_dc_ep_read_continue(uint8_t ep)
{
	uint8_t idx = USB_EP_GET_IDX(ep);
	struct wch_ep_state *state;

	if (USB_EP_DIR_IS_IN(ep)) {
		return -EINVAL;
	}

	if (idx >= WCH_USB_NUM_EP || !WCH_EP_IS_USABLE(idx)) {
		return -EINVAL;
	}

	state = &dev_data.ep_state[idx][USB_OUT_IDX];

	/* Only re-arm once the buffer has actually been drained. */
	if (state->rx_idx < state->rx_len) {
		return 0;
	}

	state->rx_len = 0;
	state->rx_idx = 0;
	wch_ep_set_rx_res(idx, state->isochronous ? USBFS_UEP_R_RES_NONE : USBFS_UEP_R_RES_ACK);

	return 0;
}

int usb_dc_ep_read(const uint8_t ep, uint8_t *const data, const uint32_t max_data_len,
		   uint32_t *const read_bytes)
{
	int ret = usb_dc_ep_read_wait(ep, data, max_data_len, read_bytes);

	if (ret != 0) {
		return ret;
	}

	return usb_dc_ep_read_continue(ep);
}

int usb_dc_ep_set_callback(const uint8_t ep, const usb_dc_ep_callback cb)
{
	uint8_t idx = USB_EP_GET_IDX(ep);
	bool is_in = USB_EP_GET_DIR(ep) == USB_EP_DIR_IN;

	if (idx >= WCH_USB_NUM_EP) {
		return -EINVAL;
	}

	dev_data.ep_state[idx][is_in].cb = cb;

	return 0;
}

void usb_dc_set_status_callback(const usb_dc_status_callback cb)
{
	dev_data.status_cb = cb;
}

int usb_dc_ep_mps(const uint8_t ep)
{
	uint8_t idx = USB_EP_GET_IDX(ep);
	bool is_in = USB_EP_GET_DIR(ep) == USB_EP_DIR_IN;

	if (idx >= WCH_USB_NUM_EP) {
		return -EINVAL;
	}

	return dev_data.ep_state[idx][is_in].mps;
}

int usb_dc_wakeup_request(void)
{
	return -ENOTSUP;
}

/* ------------------------------------------------------------------ */
/* Deferred callback thread                                             */
/* ------------------------------------------------------------------ */

/*
 * The stack's callbacks re-enter the driver (usb_dc_ep_read, usb_dc_ep_write)
 * and may block, so they must not run in ISR context. Same approach as
 * usb_dc_rpi_pico.c.
 */
static void wch_usb_thread_main(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		struct wch_usb_msg msg;

		k_msgq_get(&wch_usb_msgq, &msg, K_FOREVER);

		if (msg.ep_event) {
			uint8_t idx = USB_EP_GET_IDX(msg.ep);
			bool is_in = USB_EP_GET_DIR(msg.ep) == USB_EP_DIR_IN;

			if (dev_data.ep_state[idx][is_in].cb != NULL) {
				dev_data.ep_state[idx][is_in].cb(msg.ep, msg.type);
			}
		} else if (dev_data.status_cb != NULL) {
			dev_data.status_cb(msg.type, NULL);
		}
	}
}

/*
 * Devicetree accessors need literal indices, so unroll the clocks list. All
 * entries share one controller (&rcc) on both blocks.
 */
#define WCH_USB_CLOCK_ID(i, _) DT_INST_CLOCKS_CELL_BY_IDX(0, i, id)

static const uint8_t wch_usb_clock_ids[] = {
	LISTIFY(DT_INST_NUM_CLOCKS(0), WCH_USB_CLOCK_ID, (,)),
#if WCH_USB_X03X
	/* The D-/D+ pads live on GPIOC, which needs its own gate. */
	CH32V20X_V30X_CLOCK_IOPC,
#endif
};

static int usb_dc_wch_init(void)
{
	const struct device *clk = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(0));

	if (!device_is_ready(clk)) {
		return -ENODEV;
	}

	for (size_t i = 0; i < ARRAY_SIZE(wch_usb_clock_ids); i++) {
		int ret = clock_control_on(
			clk, (clock_control_subsys_t)(uintptr_t)wch_usb_clock_ids[i]);

		if (ret < 0 && ret != -EALREADY) {
			return ret;
		}
	}

	IRQ_CONNECT(DT_INST_IRQN(0), 0, wch_usb_isr, NULL, 0);
	irq_enable(DT_INST_IRQN(0));

	k_thread_create(&wch_usb_thread, wch_usb_thread_stack, CONFIG_USB_DC_WCH_STACK_SIZE,
			wch_usb_thread_main, NULL, NULL, NULL,
			K_PRIO_COOP(CONFIG_USB_DC_WCH_THREAD_PRIORITY), 0, K_NO_WAIT);
	k_thread_name_set(&wch_usb_thread, "usb_dc_wch");

	return 0;
}

SYS_INIT(usb_dc_wch_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
