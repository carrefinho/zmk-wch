/*
 * Copyright (c) 2024 MASSDRIVER EI (massdriver.space)
 * Copyright (c) 2026 carrefinho
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_x03x_afio

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/pinctrl/ch32x035-pinctrl.h>

#include <hal_ch32fun.h>

/*
 * CH32X035 AFIO pin controller.
 *
 * Modelled on the wch,20x_30x-afio driver but for the X035, which has a single
 * PCFR1 remap register (no PCFR2). GPIO configuration uses the classic WCH
 * 4-bits-per-pin CFGLR/CFGHR/CFGXR scheme, covering pins 0-23 (pins 16-23 in
 * CFGXR with set/reset via BSXR).
 */

static GPIO_TypeDef *const wch_afio_pinctrl_regs[] = {
	(GPIO_TypeDef *)DT_REG_ADDR(DT_NODELABEL(gpioa)),
	(GPIO_TypeDef *)DT_REG_ADDR(DT_NODELABEL(gpiob)),
	(GPIO_TypeDef *)DT_REG_ADDR(DT_NODELABEL(gpioc)),
};

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	int i;

	for (i = 0; i < pin_cnt; i++, pins++) {
		uint8_t port = FIELD_GET(CH32X035_PINCTRL_PORT_MASK, pins->config);
		uint8_t pin = FIELD_GET(CH32X035_PINCTRL_PIN_MASK, pins->config);
		uint8_t bit0 = FIELD_GET(CH32X035_PINCTRL_BASE_MASK, pins->config);
		uint8_t remap = FIELD_GET(CH32X035_PINCTRL_RM_MASK, pins->config);
		GPIO_TypeDef *regs = wch_afio_pinctrl_regs[port];
		uint8_t cfg = 0;

		if (pins->output_high || pins->output_low) {
			cfg |= (pins->slew_rate + 1);
			if (pins->drive_open_drain) {
				cfg |= BIT(2);
			}
			/* Select the alternate function. */
			cfg |= BIT(3);
		} else {
			if (pins->bias_pull_up || pins->bias_pull_down) {
				cfg |= BIT(3);
			}
		}

		if (pin < 8) {
			regs->CFGLR = (regs->CFGLR & ~(0x0F << (pin * 4))) | (cfg << (pin * 4));
		} else if (pin < 16) {
			regs->CFGHR = (regs->CFGHR & ~(0x0F << ((pin - 8) * 4))) |
				      (cfg << ((pin - 8) * 4));
		} else {
			/* Pins 16-23 configure via CFGXR, same 4-bits-per-pin scheme
			 * as CFGLR/CFGHR (needed for PC16/PC17 = USB DM/DP and any
			 * high-index GPIO).
			 */
			regs->CFGXR = (regs->CFGXR & ~(0x0F << ((pin - 16) * 4))) |
				      (cfg << ((pin - 16) * 4));
		}

		/* Drive the initial output level / pull direction. Pins 0-15 use
		 * BSHR (low half sets, BCR clears); pins 16-23 use BSXR (bit
		 * [pin-16] sets, bit [pin] clears, per the GPIO_BSXR_BS16/BR16
		 * layout).
		 */
		if (pin < 16) {
			if (pins->output_high) {
				regs->BSHR = BIT(pin);
			} else if (pins->output_low) {
				regs->BCR = BIT(pin);
			} else {
				if (pins->bias_pull_up) {
					regs->BSHR = BIT(pin);
				}
				if (pins->bias_pull_down) {
					regs->BCR = BIT(pin);
				}
			}
		} else {
			if (pins->output_high || pins->bias_pull_up) {
				regs->BSXR = BIT(pin - 16);
			} else if (pins->output_low || pins->bias_pull_down) {
				regs->BSXR = BIT(pin);
			}
		}

		if (remap != 0) {
			AFIO->PCFR1 |= (uint32_t)remap << bit0;
		}
	}

	return 0;
}

static int pinctrl_clock_init(void)
{
	const struct device *clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(0));
	uint8_t clock_id = DT_INST_CLOCKS_CELL(0, id);

	return clock_control_on(clock_dev, (clock_control_subsys_t *)(uintptr_t)clock_id);
}

SYS_INIT(pinctrl_clock_init, PRE_KERNEL_1, 0);
