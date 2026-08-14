/*
 * Copyright (c) 2024 Dhiru Kholia
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * NOTE: See this table for IC family reference,
 * in conjunction with Page 5 of the reference manual:
 * https://www.wch-ic.com/products/productsCenter/mcuInterface?categoryId=70
 */

#ifndef _CH32FUN_H
#define _CH32FUN_H

/*
 * The WCH vendor headers (notably ch32x03xhw.h for the CH32X035) define a
 * value-like `BIT_MASK ((uint8_t)0x7F)` that collides with Zephyr's
 * function-like `BIT_MASK(n)`. The latter is used pervasively by cbprintf /
 * logging, so letting the vendor definition win breaks any WCH driver that
 * logs. Preserve Zephyr's definition across the vendor include (and undef
 * first to silence the redefinition warning).
 */
#include <zephyr/sys/util_macro.h>
#pragma push_macro("BIT_MASK")
#undef BIT_MASK

#if defined(CONFIG_SOC_SERIES_QINGKE_V2A)
#define CH32V003 1
#include <ch32fun.h>
#endif /* defined(CONFIG_SOC_SERIES_QINGKE_V2A) */

#if defined(CONFIG_SOC_SERIES_QINGKE_V2C)
#define CH32V00x 1
#include <ch32fun.h>
#endif /* defined(CONFIG_SOC_SERIES_QINGKE_V2C) */

#if defined(CONFIG_SOC_SERIES_QINGKE_V4B)
#define CH32V20x    1
#define CH32V20x_D6 1
#include <ch32fun.h>
#endif /* defined(CONFIG_SOC_SERIES_QINGKE_V4B) */

#if defined(CONFIG_SOC_SERIES_QINGKE_V4C)
#if defined(CONFIG_SOC_CH32X035)
/* CH32X035 shares the QingKe V4C core but is its own peripheral family
 * (HSI-only 48 MHz, no PLL/HSE, PCFR1-only AFIO) -> ch32x03xhw.h.
 */
#define CH32X03x 1
#else
#define CH32V20x 1
#if defined(CONFIG_SOC_CH32V208)
#define CH32V20x_D8W 1
#endif
#endif
#include <ch32fun.h>
#endif /* defined(CONFIG_SOC_SERIES_QINGKE_V4C) */

#if defined(CONFIG_SOC_SERIES_QINGKE_V4F)
#define CH32V30x 1
#if defined(CONFIG_SOC_CH32V303)
#define CH32V30x_D8 1
#elif defined(CONFIG_SOC_CH32V307)
#define CH32V30x_D8C 1
#endif
#include <ch32fun.h>
#endif /* defined(CONFIG_SOC_SERIES_QINGKE_V4F) */

/* Restore Zephyr's function-like BIT_MASK(n) clobbered by the vendor header. */
#pragma pop_macro("BIT_MASK")

#endif
