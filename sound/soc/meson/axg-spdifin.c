// SPDX-License-Identifier: (GPL-2.0 OR MIT)
//
// Copyright (c) 2018 BayLibre, SAS.
// Author: Jerome Brunet <jbrunet@baylibre.com>

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <sound/jack.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <sound/pcm_params.h>

#include "axg-spdifin-utils.h"

#define SPDIFIN_CTRL0			0x00
#define  SPDIFIN_CTRL0_EN		BIT(31)
#define  SPDIFIN_CTRL0_RST_OUT		BIT(29)
#define  SPDIFIN_CTRL0_RST_IN		BIT(28)
#define  SPDIFIN_CTRL0_AXG_IRQ_CLR	BIT(26)
#define  SPDIFIN_CTRL0_WIDTH_SEL	BIT(24)
#define  SPDIFIN_CTRL0_STATUS_CH_SHIFT	11
#define  SPDIFIN_CTRL0_STATUS_SEL	GENMASK(10, 8)
#define  SPDIFIN_CTRL0_SRC_SEL		GENMASK(5, 4)
#define  SPDIFIN_CTRL0_CHK_VALID	BIT(3)
#define SPDIFIN_CTRL1			0x04
#define  SPDIFIN_CTRL1_BASE_TIMER	GENMASK(19, 0)
#define  SPDIFIN_CTRL1_IRQ_MASK		GENMASK(27, 20)
#define SPDIFIN_CTRL2			0x08
#define SPDIFIN_CTRL3			0x0c
#define SPDIFIN_CTRL4			0x10
#define SPDIFIN_CTRL5			0x14
#define SPDIFIN_CTRL6			0x18
#define  SPDIFIN_CTRL6_G12_IRQ_CLR	GENMASK(23, 16)
#define SPDIFIN_STAT0			0x1c
#define  SPDIFIN_STAT0_IRQ		GENMASK(7, 0)
#define  SPDIFIN_IRQ_VALID_CHANGED	BIT(6)
#define  SPDIFIN_IRQ_CSW_CHANGED	BIT(3)
#define  SPDIFIN_IRQ_MODE_CHANGED	BIT(2)
#define  SPDIFIN_IRQ_PARITY_ERR		BIT(1)
#define  SPDIFIN_IRQ_OVERFLOW		BIT(0)
#define SPDIFIN_STAT1			0x20
#define SPDIFIN_STAT2			0x24
#define SPDIFIN_MUTE_VAL		0x28

#define SPDIFIN_IRQS (SPDIFIN_IRQ_VALID_CHANGED | \
		      SPDIFIN_IRQ_CSW_CHANGED |	  \
		      SPDIFIN_IRQ_MODE_CHANGED |  \
		      SPDIFIN_IRQ_PARITY_ERR |	  \
		      SPDIFIN_IRQ_OVERFLOW)

#define SPDIFIN_MODE_NUM		7

struct axg_spdifin_cfg {
	const unsigned int *mode_rates;
	unsigned int ref_rate;
	irq_handler_t isr;
};

struct axg_spdifin {
	const struct axg_spdifin_cfg *conf;
	struct regmap *map;
	struct clk *refclk;
	struct clk *pclk;
	struct mutex lock;
	struct snd_pcm_substream *substream;
	struct snd_soc_jack jack;
	unsigned int rate;
	int irq;
};

static irqreturn_t axg_spdifin_isr(int irq, void *dev_id)
{
	struct snd_soc_dai *dai = dev_id;
	struct axg_spdifin *priv = snd_soc_dai_get_drvdata(dai);

	/* Clear all IRQs with a single bit on axg */
	regmap_update_bits(priv->map, SPDIFIN_CTRL0,
			   SPDIFIN_CTRL0_AXG_IRQ_CLR,
			   SPDIFIN_CTRL0_AXG_IRQ_CLR);
	regmap_update_bits(priv->map, SPDIFIN_CTRL0,
			   SPDIFIN_CTRL0_AXG_IRQ_CLR,
			   0);	

	return IRQ_WAKE_THREAD;
}

static irqreturn_t g12_spdifin_isr(int irq, void *dev_id)
{
	struct snd_soc_dai *dai = dev_id;
	struct axg_spdifin *priv = snd_soc_dai_get_drvdata(dai);
	unsigned int mask;

	/* From G12 on, the each IRQs are cleared individually */
	regmap_read(priv->map, SPDIFIN_STAT0, &mask);
	mask = FIELD_GET(SPDIFIN_STAT0_IRQ, mask);

	regmap_update_bits(priv->map, SPDIFIN_CTRL6,
			   SPDIFIN_CTRL6_G12_IRQ_CLR,
			   FIELD_PREP(SPDIFIN_CTRL6_G12_IRQ_CLR,
				      mask));

	return IRQ_WAKE_THREAD;
}

static irqreturn_t axg_spdifin_isr_thread(int irq, void *dev_id)
{
	struct snd_soc_dai *dai = dev_id;
	struct axg_spdifin *priv = snd_soc_dai_get_drvdata(dai);
	unsigned int rate;
	bool valid_change;

	mutex_lock(&priv->lock);
	rate = meson_spdifin_get_rate(priv->map, SPDIFIN_STAT0,
				    priv->conf->mode_rates);

	/* Check if the rate validity changed */
	valid_change = !!priv->rate != !!rate;
	
	/*
	 * If the stream rate changes while capturing, capture must
	 * restart to reset the constraints on the stream
	 */
	if ((priv->rate != rate) && priv->substream) {
		snd_soc_dpcm_be_stop(priv->substream, SNDRV_PCM_STATE_DISCONNECTED);
		priv->substream = NULL;
	}

	priv->rate = rate;
	mutex_unlock(&priv->lock);

	if (valid_change)
		snd_soc_jack_report(&priv->jack,
				    rate ? SND_JACK_LINEIN : 0,
				    SND_JACK_LINEIN);

	return IRQ_HANDLED;
}

static int axg_spdifin_dai_prepare(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	struct axg_spdifin *priv = snd_soc_dai_get_drvdata(dai);

	/* Apply both reset */
	regmap_update_bits(priv->map, SPDIFIN_CTRL0,
			   SPDIFIN_CTRL0_RST_OUT |
			   SPDIFIN_CTRL0_RST_IN,
			   0);

	/* Clear out reset before in reset */
	regmap_update_bits(priv->map, SPDIFIN_CTRL0,
			   SPDIFIN_CTRL0_RST_OUT, SPDIFIN_CTRL0_RST_OUT);
	regmap_update_bits(priv->map, SPDIFIN_CTRL0,
			   SPDIFIN_CTRL0_RST_IN,  SPDIFIN_CTRL0_RST_IN);

	return 0;
}

static int axg_spdifin_dai_startup(struct snd_pcm_substream *substream,
				   struct snd_soc_dai *dai)
{
	struct axg_spdifin *priv = snd_soc_dai_get_drvdata(dai);
	int ret = 0;

        guard(mutex)(&priv->lock);

	/* Only allow start if a rate is locked from ARC or eARC */
	if (!priv->rate)
	        return -EIO;

	/* Save substream to stop from IRQ if necessary */
	priv->substream = substream;

	/* Only support the rate provided by the source */
	ret = snd_pcm_hw_constraint_single(substream->runtime,
					   SNDRV_PCM_HW_PARAM_RATE,
					   priv->rate);
	if (ret < 0)
		dev_err(dai->dev, "can't set SPDIF input rate constraint\n");

	return ret;
}

static void axg_spdifin_dai_shutdown(struct snd_pcm_substream *substream,
				     struct snd_soc_dai *dai)
{
	struct axg_spdifin *priv = snd_soc_dai_get_drvdata(dai);

	guard(mutex)(&priv->lock);
	priv->substream = NULL;
}


static int axg_spdifin_sample_mode_config(struct snd_soc_dai *dai,
					  struct axg_spdifin *priv)
{
	int ret;

	/* Set spdif input reference clock */
	ret = clk_set_rate(priv->refclk, priv->conf->ref_rate);
	if (ret) {
		dev_err(dai->dev, "reference clock rate set failed\n");
		return ret;
	}

	/* Threshold based on the maximum width between two edges */
	regmap_update_bits(priv->map, SPDIFIN_CTRL0,
			   SPDIFIN_CTRL0_WIDTH_SEL, 0);

	meson_spdifin_sample_mode_config(priv->map, priv->refclk,
					 SPDIFIN_CTRL1,
					 priv->conf->mode_rates);

	return 0;
}

static int axg_spdifin_dai_probe(struct snd_soc_dai *dai)
{
	struct axg_spdifin *priv = snd_soc_dai_get_drvdata(dai);
	int ret;

	ret = request_threaded_irq(priv->irq, priv->conf->isr,
				   axg_spdifin_isr_thread,
				   IRQF_ONESHOT, "spdifin", dai);
	if (ret) {
		dev_err(dai->dev, "failed to install isr");
		return ret;
	}

	ret = clk_prepare_enable(priv->pclk);
	if (ret) {
		dev_err(dai->dev, "failed to enable pclk\n");
		goto irq_err;
	}

	ret = axg_spdifin_sample_mode_config(dai, priv);
	if (ret) {
		dev_err(dai->dev, "mode configuration failed\n");
		goto pclk_err;
	}

	ret = clk_prepare_enable(priv->refclk);
	if (ret) {
		dev_err(dai->dev,
			"failed to enable spdifin reference clock\n");
		goto pclk_err;
	}

	/* Initially report JACK is disconnected */
	snd_soc_jack_report(&priv->jack, 0, SND_JACK_LINEIN);

	/* Enable IRQs */
	regmap_update_bits(priv->map, SPDIFIN_CTRL1,
			   SPDIFIN_CTRL1_IRQ_MASK,
			   FIELD_PREP(SPDIFIN_CTRL1_IRQ_MASK,
				      SPDIFIN_IRQS));

	regmap_update_bits(priv->map, SPDIFIN_CTRL0, SPDIFIN_CTRL0_EN,
			   SPDIFIN_CTRL0_EN);

	return 0;

pclk_err:
	clk_disable_unprepare(priv->pclk);
irq_err:
	free_irq(priv->irq, dai);
	return ret;
}

static int axg_spdifin_dai_remove(struct snd_soc_dai *dai)
{
	struct axg_spdifin *priv = snd_soc_dai_get_drvdata(dai);

	regmap_update_bits(priv->map, SPDIFIN_CTRL0, SPDIFIN_CTRL0_EN, 0);
	clk_disable_unprepare(priv->refclk);
	clk_disable_unprepare(priv->pclk);
	return 0;
}

static int axg_spdifin_component_probe(struct snd_soc_component *component)
{
	struct axg_spdifin *priv = snd_soc_component_get_drvdata(component);

	/* Insert SPDIF input jack in the card */
	return snd_soc_card_jack_new(component->card, "SPDIF Input Jack",
				     SND_JACK_LINEIN, &priv->jack);
}

static const struct snd_soc_dai_ops axg_spdifin_ops = {
	.probe		= axg_spdifin_dai_probe,
	.remove		= axg_spdifin_dai_remove,
	.startup	= axg_spdifin_dai_startup,
	.shutdown	= axg_spdifin_dai_shutdown,
	.prepare	= axg_spdifin_dai_prepare,
};

static const char * const spdifin_chsts_src_texts[] = {
	"A", "B",
};

static SOC_ENUM_SINGLE_DECL(axg_spdifin_chsts_src_enum, SPDIFIN_CTRL0,
			    SPDIFIN_CTRL0_STATUS_CH_SHIFT,
			    spdifin_chsts_src_texts);

static int axg_spdifin_rate_lock_info(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 192000;

	return 0;
}

static int axg_spdifin_rate_lock_get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_kcontrol_chip(kcontrol);
	struct axg_spdifin *priv = snd_soc_component_get_drvdata(c);

	ucontrol->value.integer.value[0] =
		meson_spdifin_get_rate(priv->map, SPDIFIN_STAT0,
				       priv->conf->mode_rates);

	return 0;
}

#define AXG_SPDIFIN_LOCK_RATE(xname)				\
	{							\
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,		\
		.access = (SNDRV_CTL_ELEM_ACCESS_READ |		\
			   SNDRV_CTL_ELEM_ACCESS_VOLATILE),	\
		.get = axg_spdifin_rate_lock_get,		\
		.info = axg_spdifin_rate_lock_info,		\
		.name = xname,					\
	}

static const struct snd_kcontrol_new axg_spdifin_controls[] = {
	AXG_SPDIFIN_LOCK_RATE("Capture Rate Lock"),
	SOC_DOUBLE("Capture Switch", SPDIFIN_CTRL0, 7, 6, 1, 1),
	SOC_ENUM(SNDRV_CTL_NAME_IEC958("", CAPTURE, NONE) "Src",
		 axg_spdifin_chsts_src_enum),
	MESON_SPDIFIN_IEC958_MASK(""),
	MESON_SPDIFIN_IEC958_STATUS("", SPDIFIN_CTRL0,
				    SPDIFIN_CTRL0_STATUS_SEL,
				    SPDIFIN_STAT1),
};

static const struct snd_soc_component_driver axg_spdifin_component_drv = {
	.probe			= axg_spdifin_component_probe,
	.controls		= axg_spdifin_controls,
	.num_controls		= ARRAY_SIZE(axg_spdifin_controls),
	.legacy_dai_naming	= 1,
};

static const struct regmap_config axg_spdifin_regmap_cfg = {
	.reg_bits	= 32,
	.val_bits	= 32,
	.reg_stride	= 4,
	.max_register	= SPDIFIN_MUTE_VAL,
};

static const unsigned int axg_spdifin_mode_rates[SPDIFIN_MODE_NUM] = {
	32000, 44100, 48000, 88200, 96000, 176400, 192000,
};

static const struct axg_spdifin_cfg axg_cfg = {
	.mode_rates = axg_spdifin_mode_rates,
	.ref_rate = 333333333,
	.isr = axg_spdifin_isr,
};

static const struct axg_spdifin_cfg g12_cfg = {
	.mode_rates = axg_spdifin_mode_rates,
	.ref_rate = 333333333,
	.isr = g12_spdifin_isr,
};

static const struct of_device_id axg_spdifin_of_match[] = {
	{
		.compatible = "amlogic,axg-spdifin",
		.data = &axg_cfg,
	}, {
		.compatible = "amlogic,g12-spdifin",
		.data = &g12_cfg,
	}, {
		.compatible = "amlogic,sm1-spdifin",
		.data = &g12_cfg,
	}, {}
};
MODULE_DEVICE_TABLE(of, axg_spdifin_of_match);

static struct snd_soc_dai_driver *
axg_spdifin_get_dai_drv(struct device *dev, struct axg_spdifin *priv)
{
	struct snd_soc_dai_driver *drv;
	int i;

	drv = devm_kzalloc(dev, sizeof(*drv), GFP_KERNEL);
	if (!drv)
		return ERR_PTR(-ENOMEM);

	drv->name = "SPDIF Input";
	drv->ops = &axg_spdifin_ops;
	drv->capture.stream_name = "Capture";
	drv->capture.channels_min = 1;
	drv->capture.channels_max = 2;
	drv->capture.formats = SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_LE;

	for (i = 0; i < SPDIFIN_MODE_NUM; i++) {
		unsigned int rb =
			snd_pcm_rate_to_rate_bit(priv->conf->mode_rates[i]);

		if (rb == SNDRV_PCM_RATE_KNOT)
			return ERR_PTR(-EINVAL);

		drv->capture.rates |= rb;
	}

	return drv;
}

static int axg_spdifin_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct axg_spdifin *priv;
	struct snd_soc_dai_driver *dai_drv;
	void __iomem *regs;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	platform_set_drvdata(pdev, priv);

	priv->conf = of_device_get_match_data(dev);
	if (!priv->conf) {
		dev_err(dev, "failed to match device\n");
		return -ENODEV;
	}

	regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	priv->map = devm_regmap_init_mmio(dev, regs, &axg_spdifin_regmap_cfg);
	if (IS_ERR(priv->map)) {
		dev_err(dev, "failed to init regmap: %ld\n",
			PTR_ERR(priv->map));
		return PTR_ERR(priv->map);
	}

	priv->pclk = devm_clk_get(dev, "pclk");
	if (IS_ERR(priv->pclk))
		return dev_err_probe(dev, PTR_ERR(priv->pclk), "failed to get pclk\n");

	priv->refclk = devm_clk_get(dev, "refclk");
	if (IS_ERR(priv->refclk))
		return dev_err_probe(dev, PTR_ERR(priv->refclk), "failed to get mclk\n");

	priv->irq = of_irq_get(dev->of_node, 0);
	if (priv->irq <= 0) {
		dev_err(dev, "failed to get irq: %d\n", priv->irq);
		return priv->irq;
	}

	mutex_init(&priv->lock);

	dai_drv = axg_spdifin_get_dai_drv(dev, priv);
	if (IS_ERR(dai_drv)) {
		dev_err(dev, "failed to get dai driver: %ld\n",
			PTR_ERR(dai_drv));
		return PTR_ERR(dai_drv);
	}

	return devm_snd_soc_register_component(dev, &axg_spdifin_component_drv,
					       dai_drv, 1);
}

static struct platform_driver axg_spdifin_pdrv = {
	.probe = axg_spdifin_probe,
	.driver = {
		.name = "axg-spdifin",
		.of_match_table = axg_spdifin_of_match,
	},
};
module_platform_driver(axg_spdifin_pdrv);

MODULE_DESCRIPTION("Amlogic AXG SPDIF Input driver");
MODULE_AUTHOR("Jerome Brunet <jbrunet@baylibre.com>");
MODULE_LICENSE("GPL v2");
