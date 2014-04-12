/* linux/arch/arm/mach-exynos/include/mach/gpio-exynos4.h
 *
 * Copyright (c) 2010-2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS4 - GPIO common lib support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_GPIO_EXYNOS4_H
#define __ASM_ARCH_GPIO_EXYNOS4_H __FILE__

#define gpio_get_value	__gpio_get_value
#define gpio_set_value	__gpio_set_value
#define gpio_cansleep	__gpio_cansleep
#define gpio_to_irq	__gpio_to_irq

/* Practically, GPIO banks up to GPZ are the configurable gpio banks */

/* Common GPIO bank sizes */
#define EXYNOS4_GPIO_A0_NR	(8)
#define EXYNOS4_GPIO_A1_NR	(6)
#define EXYNOS4_GPIO_B_NR	(8)
#define EXYNOS4_GPIO_C0_NR	(5)
#define EXYNOS4_GPIO_C1_NR	(5)
#define EXYNOS4_GPIO_D0_NR	(4)
#define EXYNOS4_GPIO_D1_NR	(4)
#define EXYNOS4_GPIO_F0_NR	(8)
#define EXYNOS4_GPIO_F1_NR	(8)
#define EXYNOS4_GPIO_F2_NR	(8)
#define EXYNOS4_GPIO_F3_NR	(6)
#define EXYNOS4_GPIO_K0_NR	(7)
#define EXYNOS4_GPIO_K1_NR	(7)
#define EXYNOS4_GPIO_K2_NR	(7)
#define EXYNOS4_GPIO_K3_NR	(7)
#define EXYNOS4_GPIO_L0_NR	(8)
#define EXYNOS4_GPIO_L1_NR	(3)
#define EXYNOS4_GPIO_L2_NR	(8)
#define EXYNOS4_GPIO_Y0_NR	(6)
#define EXYNOS4_GPIO_Y1_NR	(4)
#define EXYNOS4_GPIO_Y2_NR	(6)
#define EXYNOS4_GPIO_Y3_NR	(8)
#define EXYNOS4_GPIO_Y4_NR	(8)
#define EXYNOS4_GPIO_Y5_NR	(8)
#define EXYNOS4_GPIO_Y6_NR	(8)
#define EXYNOS4_GPIO_X0_NR	(8)
#define EXYNOS4_GPIO_X1_NR	(8)
#define EXYNOS4_GPIO_X2_NR	(8)
#define EXYNOS4_GPIO_X3_NR	(8)
#define EXYNOS4_GPIO_Z_NR	(7)

/* Only EXYNOS4210 GPIO bank sizes */
#define EXYNOS4210_GPIO_E0_NR	(5)
#define EXYNOS4210_GPIO_E1_NR	(8)
#define EXYNOS4210_GPIO_E2_NR	(6)
#define EXYNOS4210_GPIO_E3_NR	(8)
#define EXYNOS4210_GPIO_E4_NR	(8)
#define EXYNOS4210_GPIO_J0_NR	(8)
#define EXYNOS4210_GPIO_J1_NR	(5)

/* Only EXYNOS4212 GPIO bank sizes */
#define EXYNOS4212_GPIO_J0_NR	(8)
#define EXYNOS4212_GPIO_J1_NR	(5)
#define EXYNOS4212_GPIO_M0_NR	(8)
#define EXYNOS4212_GPIO_M1_NR	(7)
#define EXYNOS4212_GPIO_M2_NR	(5)
#define EXYNOS4212_GPIO_M3_NR	(8)
#define EXYNOS4212_GPIO_M4_NR	(8)
#define EXYNOS4212_GPIO_V0_NR	(8)
#define EXYNOS4212_GPIO_V1_NR	(8)
#define EXYNOS4212_GPIO_V2_NR	(8)
#define EXYNOS4212_GPIO_V3_NR	(8)
#define EXYNOS4212_GPIO_V4_NR	(2)

/* GPIO bank numbers */

#define EXYNOS4_GPIO_NEXT(__gpio) \
	((__gpio##_START) + (__gpio##_NR) + CONFIG_S3C_GPIO_SPACE + 1)

enum exynos4_gpio_number {
	EXYNOS4_GPIO_A0_START		= 0,
	EXYNOS4_GPIO_A1_START		= EXYNOS4_GPIO_NEXT(EXYNOS4_GPIO_A0),
	EXYNOS4_GPIO_B_START		= EXYNOS4_GPIO_NEXT(EXYNOS4_GPIO_A1),
	EXYNOS4_GPIO_C0_START		= EXYNOS4_GPIO_NEXT(EXYNOS4_GPIO_B),
	EXYNOS4_GPIO_C1_START		= EXYNOS4_GPIO_NEXT(EXYNOS4_GPIO_C0),
	EXYNOS4_GPIO_D0_START		= EXYNOS4_GPIO_NEXT(EXYNOS4_GPIO_C1),
	EXYNOS4_GPIO_D1_START		= EXYNOS4_GPIO_NEXT(EXYNOS4_GPIO_D0),
	EXYNOS4_GPIO_F0_START		= EXYNOS4_GPIO_NEXT(EXYNOS4_GPIO_D1),
	EXYNOS4_GPIO_F1_START		= EXYNOS4_GPIO_NEXT(EXYNOS4_GPIO_F0),
	EXYNOS4_GPIO_F2_START		= EXYNOS4_GPIO_NEXT(EXYNOS4_GPIO_F1),
	EXYNOS4_GPIO_F3_START		= EXYNOS4_GPIO_NEXT(EXYNOS4_GPIO_F2),
	EXYNOS4_GPIO_K0_START		= EXYNOS4_GPIO_NEXT(EXYNOS4_GPIO_F3),
	EXYNOS4_GPIO_K1_START		= EXYNOS4_GPIO_NEXT(EXYNOS4_GPIO_K0),
	EXYNOS4_GPIO_K2_START		= EXYNOS4_GPIO_NEXT(EXYNOS4_GPIO_K1),
	EXYNOS4_GPIO_K3_START		= EXYNOS4_GPIO_NEXT(EXYNOS4_GPIO_K2),
	EXYNOS4_GPIO_L0_START		= EXYNOS4_GPIO_NEXT(EXYNOS4_GPIO_K3),
	EXYNOS4_GPIO_L1_START		= EXYNOS4_GPIO_NEXT(EXYNOS4_GPIO_L0),
	EXYNOS4_GPIO_L2_START		= EXYNOS4_GPIO_NEXT(EXYNOS4_GPIO_L1),
	EXYNOS4_GPIO_Y0_START		= EXYNOS4_GPIO_NEXT(EXYNOS4_GPIO_L2),
	EXYNOS4_GPIO_Y1_START		= EXYNOS4_GPIO_NEXT(EXYNOS4_GPIO_Y0),
	EXYNOS4_GPIO_Y2_START		= EXYNOS4_GPIO_NEXT(EXYNOS4_GPIO_Y1),
	EXYNOS4_GPIO_Y3_START		= EXYNOS4_GPIO_NEXT(EXYNOS4_GPIO_Y2),
	EXYNOS4_GPIO_Y4_START		= EXYNOS4_GPIO_NEXT(EXYNOS4_GPIO_Y3),
	EXYNOS4_GPIO_Y5_START		= EXYNOS4_GPIO_NEXT(EXYNOS4_GPIO_Y4),
	EXYNOS4_GPIO_Y6_START		= EXYNOS4_GPIO_NEXT(EXYNOS4_GPIO_Y5),
	EXYNOS4_GPIO_X0_START		= EXYNOS4_GPIO_NEXT(EXYNOS4_GPIO_Y6),
	EXYNOS4_GPIO_X1_START		= EXYNOS4_GPIO_NEXT(EXYNOS4_GPIO_X0),
	EXYNOS4_GPIO_X2_START		= EXYNOS4_GPIO_NEXT(EXYNOS4_GPIO_X1),
	EXYNOS4_GPIO_X3_START		= EXYNOS4_GPIO_NEXT(EXYNOS4_GPIO_X2),
	EXYNOS4_GPIO_Z_START		= EXYNOS4_GPIO_NEXT(EXYNOS4_GPIO_X3),
};

enum exynos4210_gpio_number {
	EXYNOS4210_GPIO_E0_START	= EXYNOS4_GPIO_NEXT(EXYNOS4_GPIO_Z),
	EXYNOS4210_GPIO_E1_START	= EXYNOS4_GPIO_NEXT(EXYNOS4210_GPIO_E0),
	EXYNOS4210_GPIO_E2_START	= EXYNOS4_GPIO_NEXT(EXYNOS4210_GPIO_E1),
	EXYNOS4210_GPIO_E3_START	= EXYNOS4_GPIO_NEXT(EXYNOS4210_GPIO_E2),
	EXYNOS4210_GPIO_E4_START	= EXYNOS4_GPIO_NEXT(EXYNOS4210_GPIO_E3),
	EXYNOS4210_GPIO_J0_START	= EXYNOS4_GPIO_NEXT(EXYNOS4210_GPIO_E4),
	EXYNOS4210_GPIO_J1_START	= EXYNOS4_GPIO_NEXT(EXYNOS4210_GPIO_J0),
};

enum exynos4212_gpio_number {
	EXYNOS4212_GPIO_J0_START	= EXYNOS4_GPIO_NEXT(EXYNOS4_GPIO_Z),
	EXYNOS4212_GPIO_J1_START	= EXYNOS4_GPIO_NEXT(EXYNOS4212_GPIO_J0),
	EXYNOS4212_GPIO_M0_START	= EXYNOS4_GPIO_NEXT(EXYNOS4212_GPIO_J1),
	EXYNOS4212_GPIO_M1_START	= EXYNOS4_GPIO_NEXT(EXYNOS4212_GPIO_M0),
	EXYNOS4212_GPIO_M2_START	= EXYNOS4_GPIO_NEXT(EXYNOS4212_GPIO_M1),
	EXYNOS4212_GPIO_M3_START	= EXYNOS4_GPIO_NEXT(EXYNOS4212_GPIO_M2),
	EXYNOS4212_GPIO_M4_START	= EXYNOS4_GPIO_NEXT(EXYNOS4212_GPIO_M3),
	EXYNOS4212_GPIO_V0_START	= EXYNOS4_GPIO_NEXT(EXYNOS4212_GPIO_M4),
	EXYNOS4212_GPIO_V1_START	= EXYNOS4_GPIO_NEXT(EXYNOS4212_GPIO_V0),
	EXYNOS4212_GPIO_V2_START	= EXYNOS4_GPIO_NEXT(EXYNOS4212_GPIO_V1),
	EXYNOS4212_GPIO_V3_START	= EXYNOS4_GPIO_NEXT(EXYNOS4212_GPIO_V2),
	EXYNOS4212_GPIO_V4_START	= EXYNOS4_GPIO_NEXT(EXYNOS4212_GPIO_V3),
};

/* EXYNOS4 GPIO number definitions */
#define EXYNOS4_GPA0(_nr)	(EXYNOS4_GPIO_A0_START + (_nr))
#define EXYNOS4_GPA1(_nr)	(EXYNOS4_GPIO_A1_START + (_nr))
#define EXYNOS4_GPB(_nr)	(EXYNOS4_GPIO_B_START + (_nr))
#define EXYNOS4_GPC0(_nr)	(EXYNOS4_GPIO_C0_START + (_nr))
#define EXYNOS4_GPC1(_nr)	(EXYNOS4_GPIO_C1_START + (_nr))
#define EXYNOS4_GPD0(_nr)	(EXYNOS4_GPIO_D0_START + (_nr))
#define EXYNOS4_GPD1(_nr)	(EXYNOS4_GPIO_D1_START + (_nr))
#define EXYNOS4_GPF0(_nr)	(EXYNOS4_GPIO_F0_START + (_nr))
#define EXYNOS4_GPF1(_nr)	(EXYNOS4_GPIO_F1_START + (_nr))
#define EXYNOS4_GPF2(_nr)	(EXYNOS4_GPIO_F2_START + (_nr))
#define EXYNOS4_GPF3(_nr)	(EXYNOS4_GPIO_F3_START + (_nr))
#define EXYNOS4_GPK0(_nr)	(EXYNOS4_GPIO_K0_START + (_nr))
#define EXYNOS4_GPK1(_nr)	(EXYNOS4_GPIO_K1_START + (_nr))
#define EXYNOS4_GPK2(_nr)	(EXYNOS4_GPIO_K2_START + (_nr))
#define EXYNOS4_GPK3(_nr)	(EXYNOS4_GPIO_K3_START + (_nr))
#define EXYNOS4_GPL0(_nr)	(EXYNOS4_GPIO_L0_START + (_nr))
#define EXYNOS4_GPL1(_nr)	(EXYNOS4_GPIO_L1_START + (_nr))
#define EXYNOS4_GPL2(_nr)	(EXYNOS4_GPIO_L2_START + (_nr))
#define EXYNOS4_GPY0(_nr)	(EXYNOS4_GPIO_Y0_START + (_nr))
#define EXYNOS4_GPY1(_nr)	(EXYNOS4_GPIO_Y1_START + (_nr))
#define EXYNOS4_GPY2(_nr)	(EXYNOS4_GPIO_Y2_START + (_nr))
#define EXYNOS4_GPY3(_nr)	(EXYNOS4_GPIO_Y3_START + (_nr))
#define EXYNOS4_GPY4(_nr)	(EXYNOS4_GPIO_Y4_START + (_nr))
#define EXYNOS4_GPY5(_nr)	(EXYNOS4_GPIO_Y5_START + (_nr))
#define EXYNOS4_GPY6(_nr)	(EXYNOS4_GPIO_Y6_START + (_nr))
#define EXYNOS4_GPX0(_nr)	(EXYNOS4_GPIO_X0_START + (_nr))
#define EXYNOS4_GPX1(_nr)	(EXYNOS4_GPIO_X1_START + (_nr))
#define EXYNOS4_GPX2(_nr)	(EXYNOS4_GPIO_X2_START + (_nr))
#define EXYNOS4_GPX3(_nr)	(EXYNOS4_GPIO_X3_START + (_nr))
#define EXYNOS4_GPZ(_nr)	(EXYNOS4_GPIO_Z_START + (_nr))

#define EXYNOS4210_GPE0(_nr)	(EXYNOS4210_GPIO_E0_START + (_nr))
#define EXYNOS4210_GPE1(_nr)	(EXYNOS4210_GPIO_E1_START + (_nr))
#define EXYNOS4210_GPE2(_nr)	(EXYNOS4210_GPIO_E2_START + (_nr))
#define EXYNOS4210_GPE3(_nr)	(EXYNOS4210_GPIO_E3_START + (_nr))
#define EXYNOS4210_GPE4(_nr)	(EXYNOS4210_GPIO_E4_START + (_nr))
#define EXYNOS4210_GPJ0(_nr)	(EXYNOS4210_GPIO_J0_START + (_nr))
#define EXYNOS4210_GPJ1(_nr)	(EXYNOS4210_GPIO_J1_START + (_nr))

#define EXYNOS4212_GPJ0(_nr)	(EXYNOS4212_GPIO_J0_START + (_nr))
#define EXYNOS4212_GPJ1(_nr)	(EXYNOS4212_GPIO_J1_START + (_nr))
#define EXYNOS4212_GPM0(_nr)	(EXYNOS4212_GPIO_M0_START + (_nr))
#define EXYNOS4212_GPM1(_nr)	(EXYNOS4212_GPIO_M1_START + (_nr))
#define EXYNOS4212_GPM2(_nr)	(EXYNOS4212_GPIO_M2_START + (_nr))
#define EXYNOS4212_GPM3(_nr)	(EXYNOS4212_GPIO_M3_START + (_nr))
#define EXYNOS4212_GPM4(_nr)	(EXYNOS4212_GPIO_M4_START + (_nr))
#define EXYNOS4212_GPV0(_nr)	(EXYNOS4212_GPIO_V0_START + (_nr))
#define EXYNOS4212_GPV1(_nr)	(EXYNOS4212_GPIO_V1_START + (_nr))
#define EXYNOS4212_GPV2(_nr)	(EXYNOS4212_GPIO_V2_START + (_nr))
#define EXYNOS4212_GPV3(_nr)	(EXYNOS4212_GPIO_V3_START + (_nr))
#define EXYNOS4212_GPV4(_nr)	(EXYNOS4212_GPIO_V4_START + (_nr))

/* the end of the EXYNOS4 specific gpios */
#define EXYNOS4210_GPIO_END	(EXYNOS4212_GPV4(EXYNOS4212_GPIO_V4_NR) + 1)
#define EXYNOS4212_GPIO_END	(EXYNOS4210_GPJ1(EXYNOS4210_GPIO_J1_NR) + 1)

#define EXYNOS4XXX_GPIO_END	(EXYNOS4212_GPIO_END > EXYNOS4210_GPIO_END ? \
				 EXYNOS4212_GPIO_END : EXYNOS4210_GPIO_END)
#define EXYNOS4_GPIO_END	EXYNOS4XXX_GPIO_END

/* define the number of gpios we need to the one after the GPZ() range */
#define ARCH_NR_GPIOS		(EXYNOS4XXX_GPIO_END +			\
				CONFIG_SAMSUNG_GPIO_EXTRA)

#include <asm-generic/gpio.h>
#if defined(CONFIG_MACH_SMDK4X12) 
#include "gpio-rp4x12.h"
#endif

#endif /* __ASM_ARCH_GPIO_EXYNOS4_H */
