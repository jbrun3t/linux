// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESS es9038q2m stub driver
 *
 *  This driver is a stub for an externaly controlled ess es9038q2m
 *
 * Author:      Jerome Brunet <jbrunet@baylibre.com>
 * Copyright:   (C) 2023 Baylibre SAS
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <linux/of.h>

#define DRV_NAME "es9038q2m"

#define STUB_RATES	SNDRV_PCM_RATE_8000_768000
#define STUB_FORMATS	(SNDRV_PCM_FMTBIT_S8 |		\
			 SNDRV_PCM_FMTBIT_U8 |		\
			 SNDRV_PCM_FMTBIT_S16_LE |	\
			 SNDRV_PCM_FMTBIT_U16_LE |	\
			 SNDRV_PCM_FMTBIT_S24_LE |	\
			 SNDRV_PCM_FMTBIT_S24_3LE |	\
			 SNDRV_PCM_FMTBIT_U24_LE |	\
			 SNDRV_PCM_FMTBIT_S32_LE |	\
			 SNDRV_PCM_FMTBIT_U32_LE)

static const struct snd_soc_dapm_widget es9038q2m_widgets[] = {
	SND_SOC_DAPM_OUTPUT("DACL"),
	SND_SOC_DAPM_OUTPUT("DACR"),
};

static const struct snd_soc_dapm_route es9038q2m_routes[] = {
	{ "DACL", NULL, "Playback" },
	{ "DACR", NULL, "Playback" },
};

static struct snd_soc_component_driver soc_codec_es9038q2m = {
	.dapm_widgets		= es9038q2m_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(es9038q2m_widgets),
	.dapm_routes		= es9038q2m_routes,
	.num_dapm_routes	= ARRAY_SIZE(es9038q2m_routes),
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

static struct snd_soc_dai_driver es9038q2m_stub_dai = {
	.name		= "hifi",
	.playback 	= {
		.stream_name	= "Playback",
		.channels_min	= 1,
		.channels_max	= 2,
		.rates		= STUB_RATES,
		.formats	= STUB_FORMATS,
	},
};

static int es9038q2m_probe(struct platform_device *pdev)
{
	return devm_snd_soc_register_component(&pdev->dev,
			&soc_codec_es9038q2m,
			&es9038q2m_stub_dai, 1);
}

#ifdef CONFIG_OF
static const struct of_device_id es9038q2m_dt_ids[] = {
	{ .compatible = "ess,es9038q2m", },
	{ }
};
MODULE_DEVICE_TABLE(of, es9038q2m_dt_ids);
#endif

static struct platform_driver es9038q2m_driver = {
	.probe		= es9038q2m_probe,
	.driver		= {
		.name	= DRV_NAME,
		.of_match_table = of_match_ptr(es9038q2m_dt_ids),
	},
};

module_platform_driver(es9038q2m_driver);

MODULE_AUTHOR("Jerome Brunet <jbrunet@baylibre.com>");
MODULE_DESCRIPTION("ESS es9038q2m stub driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
