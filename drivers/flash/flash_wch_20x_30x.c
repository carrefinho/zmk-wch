/*
 * Copyright (c) 2026 carrefinho
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/devicetree.h>

/*
 * One flash family is ever enabled per build. Select the active controller
 * compatible so the single-instance DT_INST_* accessors below bind to it.
 * (devicetree.h must be included first so DT_HAS_COMPAT_STATUS_OKAY resolves.)
 * The CH32X03x (X035) FPEC register block and geometry match the V20x/30x --
 * the fast page is 256 bytes on both (the vendor SDK's FLASH_ROM_WRITE loops
 * 64 *words*, adr += 256, with 256-byte address/length alignment), so nothing
 * below is page-size dependent -- but the fast-program buffer is loaded by a
 * different sequence on each. See FLASH_WCH_CTLR_BUF_LOAD below.
 */
#if DT_HAS_COMPAT_STATUS_OKAY(wch_ch32x03x_flash_controller)
#define DT_DRV_COMPAT wch_ch32x03x_flash_controller
#define FLASH_WCH_X03X 1
#else
#define DT_DRV_COMPAT wch_ch32v20x_30x_flash_controller
#define FLASH_WCH_X03X 0
#endif

#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#ifdef CONFIG_PM
#include <zephyr/pm/pm.h>
#endif

#include <hal_ch32fun.h>

/* The X035 header omits the program-error flag, but its FPEC STATR layout is
 * byte-identical to the V20x/30x (PGERR = bit 2). Provide a fallback so the
 * error mask below is the same on every family.
 */
#ifndef FLASH_STATR_PGERR
#define FLASH_STATR_PGERR ((uint32_t)0x04)
#endif

LOG_MODULE_REGISTER(flash_wch, CONFIG_FLASH_LOG_LEVEL);

#define SOC_NV_FLASH_NODE DT_INST(0, soc_nv_flash)
#define SOC_NV_FLASH_ADDR DT_REG_ADDR(SOC_NV_FLASH_NODE)
#define SOC_NV_FLASH_SIZE DT_REG_SIZE(SOC_NV_FLASH_NODE)

/* This driver uses the WCH "fast" mode exclusively: 256-byte page erase and
 * 256-byte buffered page program. Erased cells on this flash do NOT read back
 * 0xFF but a fixed 0xE339 halfword pattern (zephyrproject-rtos/zephyr#100490),
 * and programming a cell twice without an erase in between is electrically
 * unreliable (hardware-verified: scattered failed-to-clear bits). Neither NVS
 * nor ZMS can work with those semantics directly, so this driver emulates a
 * conventional 0xFF-erased flash: every write is a read-modify-write of whole
 * 256-byte pages (fast-erase + fast-program, so cells are always freshly
 * erased when programmed), and erase is an RMW that programs 0xFF.
 */
#define FLASH_WCH_PAGE_SIZE  256U
#define FLASH_WCH_PAGE_WORDS (FLASH_WCH_PAGE_SIZE / sizeof(uint32_t))

/* Fast-mode CTLR bits (not all are in the ch32fun v20x headers) */
#define FLASH_WCH_CTLR_PAGE_PG BIT(16)
#define FLASH_WCH_CTLR_PAGE_ER BIT(17)
#define FLASH_WCH_CTLR_FLOCK   BIT(15)

/* The two families load the fast-program buffer differently, and the bits do
 * not line up. Hardware-verified on a CH32X035C8T6: driving the V20x/30x
 * sequence on an X03x programs nothing at all -- the page keeps its previous
 * contents, STATR reports no error, and only the driver's own read-back
 * catches it.
 *
 *   V20x/30x: writing a word to the page address loads the buffer implicitly;
 *             the program is started with PG_STRT (bit 21) and the address
 *             comes from those writes, so FLASH->ADDR is not used.
 *   X03x:     the buffer must be reset with BUF_RST (bit 19) and each word
 *             explicitly committed with BUF_LOAD (bit 18); the program is
 *             then started the same way an erase is, with FLASH->ADDR plus
 *             the ordinary STRT (bit 6). PG_STRT does not exist.
 *
 * Both match their vendor SDK's FLASH_ROM_WRITE()/FLASH_ProgramPage_Fast().
 */
#if FLASH_WCH_X03X
#define FLASH_WCH_CTLR_BUF_LOAD BIT(18)
#define FLASH_WCH_CTLR_BUF_RST  BIT(19)
#else
#define FLASH_WCH_CTLR_PG_STRT BIT(21)
#endif

/* On the V20x/30x bit 22 is RSENACT (RM 32.4, write-only, self-clearing): the
 * actuator that completes an exit from enhanced read mode. The X03x has no
 * such bit -- its read buffer is reset by BUF_RST above, which is part of the
 * program sequence rather than something to poke on lock.
 */
#define FLASH_WCH_CTLR_RSENACT BIT(22)
/* Read acceleration, V20x/30x only (the X03x RM lists both as reserved).
 * ENHANCE_MOD is enhanced read mode; SCKMOD selects the flash access clock
 * (0 = SYSCLK/2, 1 = SYSCLK) and is the SCKMOD referred to in
 * flash_wch_clock_slow() below. Both are clear out of reset, and RM 32.3
 * notes the hardware also exits enhanced read mode on any system reset.
 */
#define FLASH_WCH_CTLR_ENHANCE BIT(24)
#define FLASH_WCH_CTLR_SCKMOD  BIT(25)

/* STATR bit 7: enhanced read mode has actually taken effect. RM 32.3 says to
 * set ENHANCE_MOD and then read this back.
 */
#define FLASH_WCH_STATR_EHMODS BIT(7)

/* SCKMOD=1 runs the flash access clock at the full system clock, and RM 32.4
 * states it "cannot be more than 60 MHz". Only claim it when that holds.
 */
#define FLASH_WCH_SCKMOD_IN_SPEC (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC <= MHZ(60))

#if defined(CONFIG_SOC_FLASH_WCH_SCKMOD_FULL_SPEED)
#define FLASH_WCH_CTLR_ACCEL (FLASH_WCH_CTLR_ENHANCE | FLASH_WCH_CTLR_SCKMOD)
#elif FLASH_WCH_SCKMOD_IN_SPEC
#define FLASH_WCH_CTLR_ACCEL (FLASH_WCH_CTLR_ENHANCE | FLASH_WCH_CTLR_SCKMOD)
#else
#define FLASH_WCH_CTLR_ACCEL (FLASH_WCH_CTLR_ENHANCE)
#endif

/* STATR write-1-to-clear flags. WR_BSY (the fast-program buffer is accepting
 * a word) is V20x/30x only; on the X03x bit 1 is reserved and BUF_LOAD's own
 * BSY covers the same wait.
 */
#if FLASH_WCH_X03X
#define FLASH_WCH_STATR_WR_BSY 0U
#else
#define FLASH_WCH_STATR_WR_BSY BIT(1)
#endif
#define FLASH_WCH_STATR_ERR    (FLASH_STATR_PGERR | FLASH_STATR_WRPRTERR)

#define FLASH_WCH_ERASE_TIMEOUT_MS 500

struct flash_wch_data {
	struct k_sem mutex;
	/* RMW staging buffer, word-aligned for the fast-program loop */
	uint32_t page_buf[FLASH_WCH_PAGE_WORDS];
};

static struct flash_wch_data flash_data;

static const struct flash_parameters flash_wch_parameters = {
	/* RMW makes any offset/length writable */
	.write_block_size = 1,
	.erase_value = 0xff,
};

#ifdef CONFIG_FLASH_PAGE_LAYOUT
static const struct flash_pages_layout flash_wch_layout[] = {
	{
		.pages_size = FLASH_WCH_PAGE_SIZE,
		.pages_count = SOC_NV_FLASH_SIZE / FLASH_WCH_PAGE_SIZE,
	},
};
#endif

/* Per RM chapter 32: with SYSCLK above 120 MHz, HCLK must be halved for the
 * duration of any flash program/erase so the flash access clock (HCLK/2 with
 * the default SCKMOD) stays below 60 MHz; programming at full speed corrupts
 * individual bits (hardware-verified). Returns the saved CFGR0 to pass to
 * flash_wch_clock_restore().
 */
static uint32_t flash_wch_clock_slow(void)
{
	uint32_t cfgr0 = RCC->CFGR0;

	if ((cfgr0 & RCC_HPRE) == RCC_HPRE_DIV1 &&
	    CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC > MHZ(120)) {
		RCC->CFGR0 = (cfgr0 & ~RCC_HPRE) | RCC_HPRE_DIV2;
	}

	return cfgr0;
}

static void flash_wch_clock_restore(uint32_t cfgr0)
{
	RCC->CFGR0 = cfgr0;
}

/* Enhanced read mode is a read-path-only optimisation. RM 32.3 requires it to
 * be off for any program/erase ("otherwise the erasing and programming
 * operations will fail" -- hardware-confirmed: writing with it on returns
 * -EIO on verify, and once left state stale enough to fault the next
 * instruction fetch from zero-wait flash). So it is enabled at init and
 * cleared across every operation, and the exit follows the documented order:
 * clear ENHANCE_MOD first, then set RSENACT.
 *
 * Measured on a CH32V203C8T6 at 144 MHz reading the non-zero-wait region
 * above the marketed 64K, with SCKMOD also set: sequential 2148 -> 1010
 * us/4K, pointer-chase 12642 -> 5975 us, instruction fetch 751 -> 347 ms.
 * Enhanced read mode alone (SCKMOD clear, the in-spec configuration above
 * 60 MHz) is worth rather less: 2148 -> 1807 us and 751 -> 681 ms.
 * Reads inside the zero-wait region are unaffected either way.
 */
static void flash_wch_accel_set(bool enable)
{
	if (IS_ENABLED(CONFIG_SOC_FLASH_WCH_READ_ACCELERATION)) {
		unsigned int key = irq_lock();

		if (enable) {
			FLASH->CTLR |= FLASH_WCH_CTLR_ACCEL;
		} else {
			FLASH->CTLR &= ~FLASH_WCH_CTLR_ACCEL;
			FLASH->CTLR |= FLASH_WCH_CTLR_RSENACT;
		}

		irq_unlock(key);
	}
}

static void flash_wch_unlock(void)
{
	FLASH->KEYR = FLASH_KEY1;
	FLASH->KEYR = FLASH_KEY2;
	FLASH->MODEKEYR = FLASH_KEY1;
	FLASH->MODEKEYR = FLASH_KEY2;
}

static void flash_wch_lock(void)
{
	/* Bit 22 is RSENACT on the V20x/30x -- the actuator that completes an
	 * exit from enhanced read mode. Setting it here would tear down the
	 * mode we just enabled, so only poke it when acceleration is off, which
	 * matches the vendor disable path; the accel disable path sets it itself,
	 * in the documented order. The bit is reserved on the X03x, whose read
	 * buffer is reset by BUF_RST inside the program sequence instead.
	 */
	if (!FLASH_WCH_X03X && !IS_ENABLED(CONFIG_SOC_FLASH_WCH_READ_ACCELERATION)) {
		FLASH->CTLR |= FLASH_WCH_CTLR_RSENACT;
	}
	FLASH->CTLR |= FLASH_WCH_CTLR_FLOCK;
	FLASH->CTLR |= FLASH_CTLR_LOCK;
}

/* RM 32.3 note 2: enhanced read mode must be exited before entering stop mode,
 * "otherwise it may cause an abnormal stop mode". The hardware clears it by
 * itself on reset (note 3) but not on a stop-mode entry, so hook system PM and
 * re-enter on resume. Without CONFIG_PM nothing puts the SoC into stop mode
 * and this compiles out.
 */
#if defined(CONFIG_PM) && defined(CONFIG_SOC_FLASH_WCH_READ_ACCELERATION)
static void flash_wch_accel_pm(bool enable)
{
	flash_wch_unlock();
	flash_wch_accel_set(enable);
	flash_wch_lock();
}

static void flash_wch_pm_entry(enum pm_state state)
{
	ARG_UNUSED(state);

	flash_wch_accel_pm(false);
}

static void flash_wch_pm_exit(enum pm_state state)
{
	ARG_UNUSED(state);

	flash_wch_accel_pm(true);
}

static struct pm_notifier flash_wch_pm_notifier = {
	.state_entry = flash_wch_pm_entry,
	.state_exit = flash_wch_pm_exit,
};

static void flash_wch_pm_init(void)
{
	pm_notifier_register(&flash_wch_pm_notifier);
}
#else
static void flash_wch_pm_init(void)
{
}
#endif

static int flash_wch_wait_idle(int32_t timeout_ms)
{
	const int64_t expired = k_uptime_get() + timeout_ms;

	while (FLASH->STATR & FLASH_STATR_BSY) {
		if (k_uptime_get() > expired) {
			return -ETIMEDOUT;
		}
	}

	return 0;
}

/* Check and clear status; STATR error/EOP bits are write-1-to-clear. */
static int flash_wch_check_status(void)
{
	uint32_t statr = FLASH->STATR;

	FLASH->STATR = statr & (FLASH_WCH_STATR_ERR | FLASH_STATR_EOP);

	if (statr & FLASH_WCH_STATR_ERR) {
		LOG_ERR("operation failed, STATR 0x%08x", statr);
		return -EIO;
	}

	return 0;
}

/* Fast-mode erase + program of one 256-byte page, vendor SDK discipline
 * (FLASH_ErasePage_Fast / FLASH_ProgramPage_Fast). Runs with interrupts
 * locked and makes no outside calls: instruction fetches from flash while an
 * operation is in flight corrupt it. `buf` is the full page payload.
 */
static void flash_wch_page_update(uint32_t page_addr, const uint32_t *buf)
{
	volatile uint32_t *dst = (uint32_t *)page_addr;
	unsigned int key = irq_lock();

	FLASH->STATR = FLASH_STATR_EOP | FLASH_WCH_STATR_ERR;

	FLASH->CTLR |= FLASH_WCH_CTLR_PAGE_ER;
	FLASH->ADDR = page_addr;
	FLASH->CTLR |= FLASH_CTLR_STRT;
	while (FLASH->STATR & FLASH_STATR_BSY) {
	}
	FLASH->CTLR &= ~FLASH_WCH_CTLR_PAGE_ER;

	FLASH->CTLR |= FLASH_WCH_CTLR_PAGE_PG;
#if FLASH_WCH_X03X
	FLASH->CTLR |= FLASH_WCH_CTLR_BUF_RST;
#endif
	while (FLASH->STATR & (FLASH_STATR_BSY | FLASH_WCH_STATR_WR_BSY)) {
	}

	for (size_t i = 0; i < FLASH_WCH_PAGE_WORDS; i++) {
		dst[i] = buf[i];
#if FLASH_WCH_X03X
		FLASH->CTLR |= FLASH_WCH_CTLR_BUF_LOAD;
		while (FLASH->STATR & FLASH_STATR_BSY) {
		}
#else
		while (FLASH->STATR & FLASH_WCH_STATR_WR_BSY) {
		}
#endif
	}

#if FLASH_WCH_X03X
	FLASH->ADDR = page_addr;
	FLASH->CTLR |= FLASH_CTLR_STRT;
#else
	FLASH->CTLR |= FLASH_WCH_CTLR_PG_STRT;
#endif
	while (FLASH->STATR & FLASH_STATR_BSY) {
	}
	FLASH->CTLR &= ~FLASH_WCH_CTLR_PAGE_PG;

	irq_unlock(key);
}

static int flash_wch_read(const struct device *dev, off_t offset, void *data,
			  size_t len)
{
	if ((offset < 0) || (offset > (off_t)SOC_NV_FLASH_SIZE) ||
	    (len > SOC_NV_FLASH_SIZE - offset)) {
		return -EINVAL;
	}

	if (len == 0U) {
		return 0;
	}

	memcpy(data, (uint8_t *)SOC_NV_FLASH_ADDR + offset, len);

	return 0;
}

/* Update the range [offset, offset+len) page by page; data == NULL erases
 * (fills with the physical erased pattern). Caller holds the mutex.
 */
static int flash_wch_update_range(struct flash_wch_data *dev_data, off_t offset,
				  const uint8_t *data, size_t len)
{
	int ret = 0;
	uint32_t cfgr0;

	cfgr0 = flash_wch_clock_slow();
	flash_wch_unlock();
	flash_wch_accel_set(false);

	while (len > 0U) {
		uint32_t page_addr = (SOC_NV_FLASH_ADDR + offset) & ~(FLASH_WCH_PAGE_SIZE - 1);
		size_t page_off = (SOC_NV_FLASH_ADDR + offset) - page_addr;
		size_t chunk = MIN(len, FLASH_WCH_PAGE_SIZE - page_off);

		ret = flash_wch_wait_idle(FLASH_WCH_ERASE_TIMEOUT_MS);
		if (ret < 0) {
			break;
		}

		memcpy(dev_data->page_buf, (void *)page_addr, FLASH_WCH_PAGE_SIZE);
		if (data != NULL) {
			memcpy((uint8_t *)dev_data->page_buf + page_off, data, chunk);
		} else {
			memset((uint8_t *)dev_data->page_buf + page_off, 0xff, chunk);
		}

		flash_wch_page_update(page_addr, dev_data->page_buf);

		ret = flash_wch_check_status();
		if (ret < 0) {
			break;
		}

		if (memcmp((void *)page_addr, dev_data->page_buf, FLASH_WCH_PAGE_SIZE) != 0) {
			LOG_ERR("verify failed at 0x%08x", page_addr);
			ret = -EIO;
			break;
		}

		offset += chunk;
		if (data != NULL) {
			data += chunk;
		}
		len -= chunk;
	}

	/* Re-enable before locking: CTLR is not writable once LOCK is set. */
	flash_wch_accel_set(true);
	flash_wch_lock();
	flash_wch_clock_restore(cfgr0);

	return ret;
}

static int flash_wch_write(const struct device *dev, off_t offset,
			   const void *data, size_t len)
{
	struct flash_wch_data *dev_data = dev->data;
	int ret;

	if ((offset < 0) || (offset > (off_t)SOC_NV_FLASH_SIZE) ||
	    (len > SOC_NV_FLASH_SIZE - offset)) {
		return -EINVAL;
	}

	if (len == 0U) {
		return 0;
	}

	k_sem_take(&dev_data->mutex, K_FOREVER);
	ret = flash_wch_update_range(dev_data, offset, data, len);
	k_sem_give(&dev_data->mutex);

	return ret;
}

static int flash_wch_erase(const struct device *dev, off_t offset, size_t size)
{
	struct flash_wch_data *dev_data = dev->data;
	int ret;

	if ((offset < 0) || (offset > (off_t)SOC_NV_FLASH_SIZE) ||
	    (size > SOC_NV_FLASH_SIZE - offset)) {
		return -EINVAL;
	}

	if (size == 0U) {
		return 0;
	}

	if ((offset % FLASH_WCH_PAGE_SIZE) || (size % FLASH_WCH_PAGE_SIZE)) {
		return -EINVAL;
	}

	k_sem_take(&dev_data->mutex, K_FOREVER);
	ret = flash_wch_update_range(dev_data, offset, NULL, size);
	k_sem_give(&dev_data->mutex);

	return ret;
}

static const struct flash_parameters *
flash_wch_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_wch_parameters;
}

#ifdef CONFIG_FLASH_PAGE_LAYOUT
static void flash_wch_pages_layout(const struct device *dev,
				   const struct flash_pages_layout **layout,
				   size_t *layout_size)
{
	ARG_UNUSED(dev);

	*layout = flash_wch_layout;
	*layout_size = ARRAY_SIZE(flash_wch_layout);
}
#endif

static DEVICE_API(flash, flash_wch_driver_api) = {
	.read = flash_wch_read,
	.write = flash_wch_write,
	.erase = flash_wch_erase,
	.get_parameters = flash_wch_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_wch_pages_layout,
#endif
};

static int flash_wch_init(const struct device *dev)
{
	struct flash_wch_data *data = dev->data;

	k_sem_init(&data->mutex, 1, 1);

	flash_wch_unlock();
	flash_wch_accel_set(true);
	flash_wch_lock();

	if (IS_ENABLED(CONFIG_SOC_FLASH_WCH_READ_ACCELERATION)) {
		/* RM 32.3 says EHMODS reads back 1 once the mode has taken
		 * effect. Measured on a CH32V203C8T6: it reads 0 whenever the
		 * FPEC is unlocked (confirmed by polling 1000 times and again
		 * after 10 ms), but reads 1 once the controller is locked
		 * again -- so it reflects the mode being live, not merely the
		 * bit being accepted, and this log line samples it right after
		 * flash_wch_lock() where it should be valid. Informational
		 * only: the mode works regardless, so do not gate on it.
		 */
		LOG_DBG("enhanced read mode: CTLR 0x%08x, EHMODS %d",
			FLASH->CTLR,
			(FLASH->STATR & FLASH_WCH_STATR_EHMODS) ? 1 : 0);
	}

	flash_wch_pm_init();

	return 0;
}

DEVICE_DT_INST_DEFINE(0, flash_wch_init, NULL, &flash_data, NULL, POST_KERNEL,
		      CONFIG_FLASH_INIT_PRIORITY, &flash_wch_driver_api);
