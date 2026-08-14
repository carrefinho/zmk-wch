/*
 * Copyright (c) 2026 carrefinho
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Reset and factory-bootloader entry for CH32V/CH32X parts.
 *
 * arch/riscv/core/reboot.c only provides an empty __weak sys_arch_reboot(), so
 * without this file sys_reboot() does nothing on these SoCs, and ZMK's
 * &sys_reset and &bootloader behaviors silently no-op.
 *
 * Bootloader entry (X03x only): FLASH_STATR bit 14 makes the next reset boot
 * the BOOT area, the factory USB ISP, even with a valid application in main
 * flash. The bit is write-protected until the two keys are written to
 * BOOT_MODEKEYR. This follows WCH's SystemReset_StartMode(Start_Mode_BOOT),
 * which also unlocks main flash via KEYR first, and ch32fun's
 * funRebootToBootloader(). The V20x/V30x headers name the key register
 * MODEKEYR and the sequence is unverified there, so other parts get a plain
 * reset for every type.
 *
 * The reset is a PFIC SYSRESET from the running core; a debugger-initiated
 * reset doesn't work on these parts (the core just resumes).
 */

#include <zephyr/kernel.h>

#include <hal_ch32fun.h>

/*
 * ZMK's &bootloader passes RST_UF2 (dt-bindings/zmk/reset.h). The name is an
 * Adafruit nRF52 bootloader artifact; here it just means "reboot into whatever
 * this board flashes from", i.e. the WCH ISP.
 */
#define REBOOT_TYPE_BOOTLOADER 0x57

/* FLASH_STATR bit 14: 1 = next reset boots the BOOT area, 0 = user flash. */
#define FLASH_STATR_START_MODE_BOOT ((uint32_t)(1U << 14))

/* PFIC_SCTLR bit 31: SYSRESET. */
#define PFIC_SCTLR_SYSRESET ((uint32_t)(1U << 31))

void sys_arch_reboot(int type)
{
#if defined(CONFIG_SOC_CH32X035)
	if (type == REBOOT_TYPE_BOOTLOADER) {
		FLASH->KEYR = FLASH_KEY1;
		FLASH->KEYR = FLASH_KEY2;

		FLASH->BOOT_MODEKEYR = FLASH_KEY1;
		FLASH->BOOT_MODEKEYR = FLASH_KEY2;

		FLASH->STATR |= FLASH_STATR_START_MODE_BOOT;
	}
#else
	ARG_UNUSED(type);
#endif

	RCC->RSTSCKR |= RSTSCKR_RMVF_Set;
	PFIC->SCTLR = PFIC_SCTLR_SYSRESET;

	/* The reset is not instantaneous; do not let the caller continue. */
	while (1) {
	}
}
