/*
 * linux/sound/soc/pxa/pxa3xx-ac97.h
 * Base on linux/sound/soc/pxa/pxa2xx-ac97.h
 *
 * Copyright (C) 2007 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#ifndef _PXA3XX_AC97_H
#define _PXA3XX_AC97_H

/* pxa2xx DAI ID's */
#define PXA3XX_DAI_AC97_HIFI	0
#define PXA3XX_DAI_AC97_AUX		1
#define PXA3XX_DAI_AC97_MIC		2

extern struct snd_soc_dai pxa_ac97_dai[3];

/* platform data */
extern struct snd_ac97_bus_ops pxa3xx_ac97_ops;

extern struct clk *ac97_clk;

extern void enable_oscc_pout(void);
extern void disable_oscc_pout(void);

#endif
