/* SPDX-License-Identifier: GPL-2.0+ */

#ifndef __LINUX_MFD_MICROCHIP_DPLL_H
#define __LINUX_MFD_MICROCHIP_DPLL_H

struct microchip_dpll_ddata {
	struct device *dev;
	struct regmap *regmap;
	struct mutex lock;
	u16 page;
};
#endif /*  __LINUX_MFD_MICROCHIP_DPLL_H */
