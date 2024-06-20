// SPDX-License-Identifier: GPL-2.0
//
// Copyright (c) 2024 BayLibre, SAS.
// Author: Jerome Brunet <jbrunet@baylibre.com>

#ifndef _MESON_AXG_SPDIFIN_UTILS_H
#define _MESON_AXG_SPDIFIN_UTILS_H

#include <sound/soc.h>

struct clk;
struct regmap;

#define SPDIFIN_MODE_NUM 7

int snd_soc_dpcm_be_stop(struct snd_pcm_substream *substream,
			 snd_pcm_state_t state);

struct meson_spdifin_chsts_control {
	unsigned int sel_reg;
	unsigned int sel_mask;
	unsigned int stat_reg;
};

unsigned int meson_spdifin_get_rate(struct regmap *map,
				    unsigned int offset,
				    const unsigned int *modes);
void meson_spdifin_sample_mode_config(struct regmap *map,
				      struct clk *ref_clk,
				      unsigned int offset,
				      const unsigned int *modes);

int meson_spdifin_iec958_info(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_info *uinfo);
int meson_spdifin_get_chsts_mask(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol);
int meson_spdifin_get_chsts(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol);

#define MESON_SPDIFIN_IEC958_MASK(xname)					\
	{								\
		.access = SNDRV_CTL_ELEM_ACCESS_READ,			\
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,			\
		.name = SNDRV_CTL_NAME_IEC958(xname, CAPTURE, MASK),	\
		.info = meson_spdifin_iec958_info,			\
		.get = meson_spdifin_get_chsts_mask,			\
	}

#define MESON_SPDIFIN_IEC958_STATUS(xname, xsel_reg, xsel_mask,		\
				    xstat_reg)				\
	{								\
		.access = (SNDRV_CTL_ELEM_ACCESS_READ |			\
			   SNDRV_CTL_ELEM_ACCESS_VOLATILE),		\
			.iface = SNDRV_CTL_ELEM_IFACE_PCM,		\
		.name =	SNDRV_CTL_NAME_IEC958(xname, CAPTURE, NONE),	\
		.info = meson_spdifin_iec958_info,			\
		.get = meson_spdifin_get_chsts,				\
		.private_value = (unsigned long)&(struct meson_spdifin_chsts_control)	\
			{.sel_reg = xsel_reg, .sel_mask = xsel_mask,	\
		 	.stat_reg = xstat_reg }				\
	}

#endif /* _MESON_AXG_SPDIFIN_UTILS_H */
