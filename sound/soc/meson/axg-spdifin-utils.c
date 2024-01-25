// SPDX-License-Identifier: GPL-2.0
//
// Copyright (c) 2024 BayLibre, SAS.
// Author: Jerome Brunet <jbrunet@baylibre.com>

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <sound/soc.h>

#include "axg-spdifin-utils.h"

/*
 * the ARC path of the ARC/eARC IP is very similar to the SPDIF input HW
 * It is very likely the HW was copied and quickly adapted for ARC.
 * As a consequence, several tasks, like setting timers, thresholds or
 * reading the channel status word is done in the same way, with similar
 * register organisation
 *
 * This is a factorisation of the code for both IPs
 */

#define SPDIFIN_MODE			GENMASK(30, 28)
#define SPDIFIN_MINW			GENMASK(27, 18)
#define SPDIFIN_MAXW			GENMASK(17, 8)

#define SPDIFIN_BASE_OFF		0x0
#define  SPDIFIN_DET_BASE_TIMER		GENMASK(19, 0)
#define SPDIFIN_THRES_OFF		0x4
#define  SPDIFIN_THRES_PER_REG		3
#define  SPDIFIN_THRES_WIDTH		10
#define SPDIFIN_TIMER_OFF		0xc
#define  SPDIFIN_TIMER_PER_REG		4
#define  SPDIFIN_TIMER_WIDTH		8

unsigned int meson_spdifin_get_rate(struct regmap *map,
				    unsigned int offset,
				    const unsigned int *modes)
{
	unsigned int stat, mode;

	regmap_read(map, offset, &stat);
	mode = FIELD_GET(SPDIFIN_MODE, stat);

	/*
	 * If max width is zero or if min width is the maximum value,
	 * we are not capturing anything.
	 * Also sometimes, when the capture is on but there is no data,
	 * mode is SPDIFIN_MODE_NUM, but not always ...
	 */
	if ((stat & SPDIFIN_MINW) == SPDIFIN_MINW ||
	    !(stat & SPDIFIN_MAXW) ||
	    (mode >= SPDIFIN_MODE_NUM))
		return 0;

	return modes[mode];
}
EXPORT_SYMBOL_GPL(meson_spdifin_get_rate);

static void meson_spdifin_write_mode_param(struct regmap *map,
					   unsigned int base_reg,					 
					   int mode,
					   unsigned int val,
					   unsigned int num_per_reg,
					   unsigned int width)
{
	uint64_t offset = mode;
	unsigned int reg, shift, rem;

	rem = do_div(offset, num_per_reg);

	reg = offset * regmap_get_reg_stride(map) + base_reg;
	shift = width * (num_per_reg - 1 - rem);

	regmap_update_bits(map, reg, GENMASK(width - 1, 0) << shift,
			   val << shift);
}

static unsigned int meson_spdifin_mode_timer(unsigned int sample_rate,
					     unsigned int reference_rate)
{
	/*
	 * Number of period of the reference clock during a period of the
	 * input signal reference clock
	 */
	return reference_rate / (128 * sample_rate);
}

static void meson_spdifin_write_timer(struct regmap *map, unsigned int offset,
				      int mode, unsigned int val)
{
	meson_spdifin_write_mode_param(map, offset + SPDIFIN_TIMER_OFF,
				       mode, val, SPDIFIN_TIMER_PER_REG,
				       SPDIFIN_TIMER_WIDTH);
}

static void meson_spdifin_write_threshold(struct regmap *map, unsigned int offset,
					  int mode, unsigned int val)
{
	meson_spdifin_write_mode_param(map, offset + SPDIFIN_THRES_OFF,
				       mode, val, SPDIFIN_THRES_PER_REG,
				       SPDIFIN_THRES_WIDTH);
}

void meson_spdifin_sample_mode_config(struct regmap *map,
				      struct clk *ref_clk,
				      unsigned int offset,
				      const unsigned int *modes)
{
	unsigned int rate, t_next = 0;
	int i;

	rate = clk_get_rate(ref_clk);

	/* HW will update mode every 1ms */
	regmap_update_bits(map, offset + SPDIFIN_BASE_OFF,
			   SPDIFIN_DET_BASE_TIMER,
			   FIELD_PREP(SPDIFIN_DET_BASE_TIMER, rate / 1000));

	for (i = SPDIFIN_MODE_NUM - 1; i >= 0; i--) {
		unsigned int t;

		/* Calculate the timer */
		t = meson_spdifin_mode_timer(modes[i], rate);

		/* Set the timer value */
		meson_spdifin_write_timer(map, offset, i, t);

		/* Set the threshold value, skipping the last mode */
		if (t_next != 0)
			meson_spdifin_write_threshold(map, offset, i, 3 * (t + t_next));

		/* Save the current timer for the next threshold calculation */
		t_next = t;
	}
}
EXPORT_SYMBOL_GPL(meson_spdifin_sample_mode_config);

int meson_spdifin_iec958_info(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_IEC958;
	uinfo->count = 1;

	return 0;
}
EXPORT_SYMBOL_GPL(meson_spdifin_iec958_info);

int meson_spdifin_get_chsts_mask(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	int i;

	for (i = 0; i < 24; i++)
		ucontrol->value.iec958.status[i] = 0xff;

	return 0;
}
EXPORT_SYMBOL_GPL(meson_spdifin_get_chsts_mask);

int meson_spdifin_get_chsts(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct meson_spdifin_chsts_control *chstsc =
		(struct meson_spdifin_chsts_control *)kcontrol->private_value;
	int i, j;

	for (i = 0; i < 6; i++) {
		unsigned int val;

		snd_soc_component_update_bits(component, chstsc->sel_reg,
					      chstsc->sel_mask,
					      i << __ffs(chstsc->sel_mask));
		val = snd_soc_component_read(component, chstsc->stat_reg);

		for (j = 0; j < 4; j++) {
			unsigned int offset = i * 4 + j;

			ucontrol->value.iec958.status[offset] =
				(val >> (j * 8)) & 0xff;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(meson_spdifin_get_chsts);

MODULE_DESCRIPTION("Amlogic SPDIF Input Utils");
MODULE_AUTHOR("Jerome Brunet <jbrunet@baylibre.com>");
MODULE_LICENSE("GPL v2");
