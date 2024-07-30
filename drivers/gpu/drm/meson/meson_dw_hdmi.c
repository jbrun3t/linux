// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2016 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>

#include <drm/bridge/dw_hdmi.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_device.h>
#include <drm/drm_edid.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_print.h>

#include <linux/videodev2.h>

#include "meson_drv.h"
#include "meson_dw_hdmi.h"
#include "meson_registers.h"

#define DRIVER_NAME "meson-dw-hdmi"
#define DRIVER_DESC "Amlogic Meson HDMI-TX DRM driver"

/**
 * DOC: HDMI Output
 *
 * HDMI Output is composed of :
 *
 * - A Synopsys DesignWare HDMI Controller IP
 * - A TOP control block controlling the Clocks and PHY
 * - A custom HDMI PHY in order convert video to TMDS signal
 *
 * .. code::
 *
 *    ___________________________________
 *   |            HDMI TOP               |<= HPD
 *   |___________________________________|
 *   |                  |                |
 *   |  Synopsys HDMI   |   HDMI PHY     |=> TMDS
 *   |    Controller    |________________|
 *   |___________________________________|<=> DDC
 *
 *
 * The HDMI TOP block only supports HPD sensing.
 * The Synopsys HDMI Controller interrupt is routed
 * through the TOP Block interrupt.
 * Communication to the TOP Block and the Synopsys
 * HDMI Controller is done a pair of addr+read/write
 * registers.
 * The HDMI PHY is configured by registers in the
 * HHI register block.
 *
 * Pixel data arrives in 4:4:4 format from the VENC
 * block and the VPU HDMI mux selects either the ENCI
 * encoder for the 576i or 480i formats or the ENCP
 * encoder for all the other formats including
 * interlaced HD formats.
 * The VENC uses a DVI encoder on top of the ENCI
 * or ENCP encoders to generate DVI timings for the
 * HDMI controller.
 *
 * GXBB, GXL and GXM embeds the Synopsys DesignWare
 * HDMI TX IP version 2.01a with HDCP and I2C & S/PDIF
 * audio source interfaces.
 *
 * We handle the following features :
 *
 * - HPD Rise & Fall interrupt
 * - HDMI Controller Interrupt
 * - HDMI PHY Init for 480i to 1080p60
 * - VENC & HDMI Clock setup for 480i to 1080p60
 * - VENC Mode setup for 480i to 1080p60
 *
 * What is missing :
 *
 * - PHY, Clock and Mode setup for 2k && 4k modes
 * - SDDC Scrambling mode for HDMI 2.0a
 * - HDCP Setup
 * - CEC Management
 */

/* Indirect channel definition for GX */
#define HDMITX_TOP_REGS		0x0
#define HDMITX_DWC_REGS		0x10

#define GX_ADDR_OFFSET		0x0
#define GX_DATA_OFFSET		0x4
#define GX_CTRL_OFFSET		0x8
#define  GX_CTRL_APB3_ERRFAIL	BIT(15)

/*
 * NOTE: G12 use direct addressing:
 * Ideally it should receive one memory region for each of the top
 * and dwc register regions but fixing this would require to change
 * the DT bindings. Doing so is a pain. Keep the region as it and work
 * around the problem, at least for now.
 * Future supported SoCs should properly describe the regions in the
 * DT bindings instead of using this trick.
 */
#define HDMITX_TOP_G12A_OFFSET	0x8000

/* HHI Registers */
#define HHI_HDMI_PHY_CNTL0	0x3a0 /* 0xe8 */
#define HHI_HDMI_PHY_CNTL1	0x3a4 /* 0xe9 */
#define  PHY_CNTL1_INIT		0x03900000
#define  PHY_INVERT		BIT(17)
#define  PHY_FIFOS		GENMASK(3, 2)
#define  PHY_CLOCK_EN		BIT(1)
#define  PHY_SOFT_RST		BIT(0)
#define HHI_HDMI_PHY_CNTL2	0x3a8 /* 0xea */
#define HHI_HDMI_PHY_CNTL3	0x3ac /* 0xeb */
#define HHI_HDMI_PHY_CNTL4	0x3b0 /* 0xec */
#define HHI_HDMI_PHY_CNTL5	0x3b4 /* 0xed */

struct meson_dw_hdmi_speed {
	const struct reg_sequence *regs;
	unsigned int reg_num;
	unsigned int limit;
};

struct meson_dw_hdmi_data {
	int (*reg_init)(struct device *dev);
	const struct meson_dw_hdmi_speed *speeds;
	unsigned int speed_num;
	bool use_drm_infoframe;
	u32 cntl0_init;
	u32 cntl1_init;
};

static int hdmi_tx_indirect_reg_read(void *context,
					 unsigned int reg,
					 unsigned int *result)
{
	void __iomem *base = context;

	/* Must write the read address twice ... */
	writel(reg, base + GX_ADDR_OFFSET);
	writel(reg, base + GX_ADDR_OFFSET);

	/* ... and read the data twice as well */
	*result = readl(base + GX_DATA_OFFSET);
	*result = readl(base + GX_DATA_OFFSET);

	return 0;
}

static int hdmi_tx_indirect_reg_write(void *context,
				      unsigned int reg,
				      unsigned int val)
{
	void __iomem *base = context;

	/* Must write the read address twice ... */
	writel(reg, base + GX_ADDR_OFFSET);
	writel(reg, base + GX_ADDR_OFFSET);

	/* ... but write the data only once */
	writel(val, base + GX_DATA_OFFSET);

	return 0;
}

static const struct regmap_bus hdmi_tx_indirect_mmio = {
	.fast_io = true,
	.reg_read = hdmi_tx_indirect_reg_read,
	.reg_write = hdmi_tx_indirect_reg_write,
};

struct meson_dw_hdmi {
	struct dw_hdmi_plat_data dw_plat_data;
	struct meson_drm *priv;
	struct device *dev;
	void __iomem *hdmitx;
	const struct meson_dw_hdmi_data *data;
	struct reset_control *hdmitx_apb;
	struct reset_control *hdmitx_ctrl;
	struct reset_control *hdmitx_phy;
	unsigned int irq_stat;
	struct dw_hdmi *hdmi;
	struct drm_bridge *bridge;
	struct regmap *top;
};

/* Setup PHY bandwidth modes */
static int meson_hdmi_phy_setup_mode(struct meson_dw_hdmi *dw_hdmi,
				      const struct drm_display_mode *mode,
				      bool mode_is_420)
{
	struct meson_drm *priv = dw_hdmi->priv;
	unsigned int pixel_clock = mode->clock;
	int i;

	/* For 420, pixel clock is half unlike venc clock */
	if (mode_is_420)
		pixel_clock /= 2;

	for (i = 0; i < dw_hdmi->data->speed_num; i++) {
		if (pixel_clock >= dw_hdmi->data->speeds[i].limit)
			break;
	}

	/* No match found - Last entry should have a 0 limit */
	if (WARN_ON(i == dw_hdmi->data->speed_num))
		return -EINVAL;

	regmap_multi_reg_write(priv->hhi,
			       dw_hdmi->data->speeds[i].regs,
			       dw_hdmi->data->speeds[i].reg_num);

	return 0;
}

static inline void meson_dw_hdmi_phy_reset(struct meson_dw_hdmi *dw_hdmi)
{
	struct meson_drm *priv = dw_hdmi->priv;

	/* Enable and software reset */
	regmap_update_bits(priv->hhi, HHI_HDMI_PHY_CNTL1,
			   PHY_FIFOS | PHY_CLOCK_EN | PHY_SOFT_RST,
			   PHY_FIFOS | PHY_CLOCK_EN | PHY_SOFT_RST);
	mdelay(2);

	/* Enable and unreset */
	regmap_update_bits(priv->hhi, HHI_HDMI_PHY_CNTL1,
			   PHY_FIFOS | PHY_CLOCK_EN | PHY_SOFT_RST,
			   PHY_FIFOS | PHY_CLOCK_EN);
	mdelay(2);
}

static int dw_hdmi_phy_init(struct dw_hdmi *hdmi, void *data,
			    const struct drm_display_info *display,
			    const struct drm_display_mode *mode)
{
	struct meson_dw_hdmi *dw_hdmi = (struct meson_dw_hdmi *)data;
	bool is_hdmi2_sink = display->hdmi.scdc.supported;
	struct meson_drm *priv = dw_hdmi->priv;
	bool mode_is_420 = false;

	DRM_DEBUG_DRIVER("\"%s\" div%d\n", mode->name,
			 mode->clock > 340000 ? 40 : 10);

	if (drm_mode_is_420_only(display, mode) ||
	    (!is_hdmi2_sink && drm_mode_is_420_also(display, mode)) ||
	    dw_hdmi_bus_fmt_is_420(hdmi))
		mode_is_420 = true;

	/* TMDS pattern setup */
	if (mode->clock > 340000 && !mode_is_420) {
		regmap_write(dw_hdmi->top, HDMITX_TOP_TMDS_CLK_PTTN_01,
			     0);
		regmap_write(dw_hdmi->top, HDMITX_TOP_TMDS_CLK_PTTN_23,
			     0x03ff03ff);
	} else {
		regmap_write(dw_hdmi->top, HDMITX_TOP_TMDS_CLK_PTTN_01,
			     0x001f001f);
		regmap_write(dw_hdmi->top, HDMITX_TOP_TMDS_CLK_PTTN_23,
			     0x001f001f);
	}

	/* Load TMDS pattern */
	regmap_write(dw_hdmi->top, HDMITX_TOP_TMDS_CLK_PTTN_CNTL,
		     TOP_TDMS_CLK_PTTN_LOAD);
	msleep(20);
	regmap_write(dw_hdmi->top, HDMITX_TOP_TMDS_CLK_PTTN_CNTL,
		     TOP_TDMS_CLK_PTTN_SHFT);

	/* Setup PHY parameters */
	meson_hdmi_phy_setup_mode(dw_hdmi, mode, mode_is_420);

	/* Disable clock, fifo, fifo_wr */
	regmap_update_bits(priv->hhi, HHI_HDMI_PHY_CNTL1,
			   PHY_FIFOS | PHY_CLOCK_EN | PHY_SOFT_RST, 0);

	dw_hdmi_set_high_tmds_clock_ratio(hdmi, display);

	msleep(100);

	/* Reset PHY 3 times in a row */
	meson_dw_hdmi_phy_reset(dw_hdmi);
	meson_dw_hdmi_phy_reset(dw_hdmi);
	meson_dw_hdmi_phy_reset(dw_hdmi);

	return 0;
}

static void dw_hdmi_phy_disable(struct dw_hdmi *hdmi,
				void *data)
{
	struct meson_dw_hdmi *dw_hdmi = (struct meson_dw_hdmi *)data;
	struct meson_drm *priv = dw_hdmi->priv;

	DRM_DEBUG_DRIVER("\n");

	/* Fallback to init mode */
	regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL1, dw_hdmi->data->cntl1_init);
	regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL0, dw_hdmi->data->cntl0_init);
}

static enum drm_connector_status dw_hdmi_read_hpd(struct dw_hdmi *hdmi,
			     void *data)
{
	struct meson_dw_hdmi *dw_hdmi = (struct meson_dw_hdmi *)data;
	unsigned int stat;

	regmap_read(dw_hdmi->top, HDMITX_TOP_STAT0, &stat);

	return !!stat ?
		connector_status_connected : connector_status_disconnected;
}

static void dw_hdmi_setup_hpd(struct dw_hdmi *hdmi,
			      void *data)
{
	struct meson_dw_hdmi *dw_hdmi = (struct meson_dw_hdmi *)data;

	/* Setup HPD Filter */
	regmap_write(dw_hdmi->top, HDMITX_TOP_HPD_FILTER,
		     FIELD_PREP(TOP_HPD_GLITCH_WIDTH, 10) |
		     FIELD_PREP(TOP_HPD_VALID_WIDTH, 160));

	/* Clear interrupts */
	regmap_write(dw_hdmi->top, HDMITX_TOP_INTR_STAT_CLR,
		     TOP_INTR_HPD_RISE | TOP_INTR_HPD_FALL);

	/* Unmask interrupts */
	regmap_update_bits(dw_hdmi->top, HDMITX_TOP_INTR_MASKN,
			   TOP_INTR_HPD_RISE | TOP_INTR_HPD_FALL,
			   TOP_INTR_HPD_RISE | TOP_INTR_HPD_FALL);
}

static const struct dw_hdmi_phy_ops meson_dw_hdmi_phy_ops = {
	.init = dw_hdmi_phy_init,
	.disable = dw_hdmi_phy_disable,
	.read_hpd = dw_hdmi_read_hpd,
	.setup_hpd = dw_hdmi_setup_hpd,
};

static irqreturn_t dw_hdmi_top_irq(int irq, void *dev_id)
{
	struct meson_dw_hdmi *dw_hdmi = dev_id;
	unsigned int stat;

	regmap_read(dw_hdmi->top, HDMITX_TOP_INTR_STAT, &stat);
	regmap_write(dw_hdmi->top, HDMITX_TOP_INTR_STAT_CLR, stat);

	/* HPD Events, handle in the threaded interrupt handler */
	if (stat & (TOP_INTR_HPD_RISE | TOP_INTR_HPD_FALL)) {
		dw_hdmi->irq_stat = stat;
		return IRQ_WAKE_THREAD;
	}

	/* HDMI Controller Interrupt */
	if (stat & TOP_INTR_CORE)
		return IRQ_NONE;

	/* TOFIX Handle HDCP Interrupts */
	return IRQ_HANDLED;
}

/* Threaded interrupt handler to manage HPD events */
static irqreturn_t dw_hdmi_top_thread_irq(int irq, void *dev_id)
{
	struct meson_dw_hdmi *dw_hdmi = dev_id;
	u32 stat = dw_hdmi->irq_stat;

	/* HPD Events */
	if (stat & (TOP_INTR_HPD_RISE | TOP_INTR_HPD_FALL)) {
		bool hpd_connected = false;

		if (stat & TOP_INTR_HPD_RISE)
			hpd_connected = true;

		dw_hdmi_setup_rx_sense(dw_hdmi->hdmi, hpd_connected,
				       hpd_connected);

		drm_helper_hpd_irq_event(dw_hdmi->bridge->dev);
		drm_bridge_hpd_notify(dw_hdmi->bridge,
				      hpd_connected ? connector_status_connected
						    : connector_status_disconnected);
	}

	return IRQ_HANDLED;
}

static const struct regmap_config top_gx_regmap_cfg = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.reg_shift	= -2,
	.val_bits	= 32,
	.max_register	= 0x40,
};

static const struct regmap_config top_g12_regmap_cfg = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= 0x40,
};

static const struct regmap_config dwc_regmap_cfg = {
	.reg_bits = 32,
	.val_bits = 8,
	.max_register = 0x8000,
};

static void meson_dw_hdmi_init(struct meson_dw_hdmi *meson_dw_hdmi)
{
	struct meson_drm *priv = meson_dw_hdmi->priv;

	/* Bring out of reset */
	regmap_write(meson_dw_hdmi->top, HDMITX_TOP_SW_RESET, 0);
	msleep(20);

	/* Enable clocks */
	regmap_write(meson_dw_hdmi->top, HDMITX_TOP_CLK_CNTL,
		     TOP_CLK_EN);

	/* Enable normal output to PHY */
	regmap_write(meson_dw_hdmi->top, HDMITX_TOP_BIST_CNTL,
		     TOP_BIST_TMDS_EN);

	/* Setup PHY */
	regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL1,
		     meson_dw_hdmi->data->cntl1_init);
	regmap_write(priv->hhi, HHI_HDMI_PHY_CNTL0,
		     meson_dw_hdmi->data->cntl0_init);

	/* Enable HDMI-TX Interrupt */
	regmap_write(meson_dw_hdmi->top, HDMITX_TOP_INTR_STAT_CLR,
		     GENMASK(31, 0));
	regmap_write(meson_dw_hdmi->top, HDMITX_TOP_INTR_MASKN,
		     TOP_INTR_CORE);
}

static int meson_dw_init_regmap_gx(struct device *dev)
{
	struct meson_dw_hdmi *meson_dw_hdmi = dev_get_drvdata(dev);
	struct regmap *map;

	/* Register TOP glue zone */
	writel_bits_relaxed(GX_CTRL_APB3_ERRFAIL, GX_CTRL_APB3_ERRFAIL,
			    meson_dw_hdmi->hdmitx + HDMITX_TOP_REGS + GX_CTRL_OFFSET);

	map = devm_regmap_init(dev, &hdmi_tx_indirect_mmio,
			       meson_dw_hdmi->hdmitx + HDMITX_TOP_REGS,
			       &top_gx_regmap_cfg);
	if (IS_ERR(map))
		return dev_err_probe(dev, PTR_ERR(map), "failed to init top regmap\n");

	meson_dw_hdmi->top = map;

	/* Register DWC zone */
	writel_bits_relaxed(GX_CTRL_APB3_ERRFAIL, GX_CTRL_APB3_ERRFAIL,
			    meson_dw_hdmi->hdmitx + HDMITX_DWC_REGS + GX_CTRL_OFFSET);

	map = devm_regmap_init(dev, &hdmi_tx_indirect_mmio,
			       meson_dw_hdmi->hdmitx + HDMITX_DWC_REGS,
			       &dwc_regmap_cfg);
	if (IS_ERR(map))
		return dev_err_probe(dev, PTR_ERR(map), "failed to init dwc regmap\n");

	meson_dw_hdmi->dw_plat_data.regm = map;

	return 0;
}

static int meson_dw_init_regmap_g12(struct device *dev)
{
	struct meson_dw_hdmi *meson_dw_hdmi = dev_get_drvdata(dev);
	struct regmap *map;

	/* Register TOP glue zone with the offset */
	map = devm_regmap_init_mmio(dev, meson_dw_hdmi->hdmitx + HDMITX_TOP_G12A_OFFSET,
				    &top_g12_regmap_cfg);
	if (IS_ERR(map))
		dev_err_probe(dev, PTR_ERR(map), "failed to init top regmap\n");

	meson_dw_hdmi->top = map;

	/* Register DWC zone */
	map = devm_regmap_init_mmio(dev, meson_dw_hdmi->hdmitx,
				    &dwc_regmap_cfg);
	if (IS_ERR(map))
		dev_err_probe(dev, PTR_ERR(map), "failed to init dwc regmap\n");

	meson_dw_hdmi->dw_plat_data.regm = map;

	return 0;
}

static const struct reg_sequence gxbb_3g7_regs[] = {
	{ .reg = HHI_HDMI_PHY_CNTL0, .def = 0x33353245 },
	{ .reg = HHI_HDMI_PHY_CNTL3, .def = 0x2100115b },
};

static const struct reg_sequence gxbb_3g_regs[] = {
	{ .reg = HHI_HDMI_PHY_CNTL0, .def = 0x33634283 },
	{ .reg = HHI_HDMI_PHY_CNTL3, .def = 0xb000115b },
};

static const struct reg_sequence gxbb_def_regs[] = {
	{ .reg = HHI_HDMI_PHY_CNTL0, .def = 0x33632122 },
	{ .reg = HHI_HDMI_PHY_CNTL3, .def = 0x2000115b },
};

static const struct meson_dw_hdmi_speed gxbb_speeds[] = {
	{
		.limit = 371250,
		.regs = gxbb_3g7_regs,
		.reg_num = ARRAY_SIZE(gxbb_3g7_regs)
	}, {
		.limit = 297000,
		.regs = gxbb_3g_regs,
		.reg_num = ARRAY_SIZE(gxbb_3g_regs)
	}, {
		.regs = gxbb_def_regs,
		.reg_num = ARRAY_SIZE(gxbb_def_regs)
	}
};

static const struct meson_dw_hdmi_data meson_dw_hdmi_gxbb_data = {
	.reg_init = meson_dw_init_regmap_gx,
	.cntl0_init = 0x0,
	.cntl1_init = PHY_CNTL1_INIT | PHY_INVERT,
	.use_drm_infoframe = false,
	.speeds = gxbb_speeds,
	.speed_num = ARRAY_SIZE(gxbb_speeds),
};

static const struct reg_sequence gxl_3g7_regs[] = {
	{ .reg = HHI_HDMI_PHY_CNTL0, .def = 0x333d3282 },
	{ .reg = HHI_HDMI_PHY_CNTL3, .def = 0x2136315b },
};

static const struct reg_sequence gxl_3g_regs[] = {
	{ .reg = HHI_HDMI_PHY_CNTL0, .def = 0x33303382 },
	{ .reg = HHI_HDMI_PHY_CNTL3, .def = 0x2036315b },
};

static const struct reg_sequence gxl_def_regs[] = {
	{ .reg = HHI_HDMI_PHY_CNTL0, .def = 0x33303362 },
	{ .reg = HHI_HDMI_PHY_CNTL3, .def = 0x2016315b },
};

static const struct reg_sequence gxl_270m_regs[] = {
	{ .reg = HHI_HDMI_PHY_CNTL0, .def = 0x33604142 },
	{ .reg = HHI_HDMI_PHY_CNTL3, .def = 0x0016315b },
};

static const struct meson_dw_hdmi_speed gxl_speeds[] = {
	{
		.limit = 371250,
		.regs = gxl_3g7_regs,
		.reg_num = ARRAY_SIZE(gxl_3g7_regs)
	}, {
		.limit = 297000,
		.regs = gxl_3g_regs,
		.reg_num = ARRAY_SIZE(gxl_3g_regs)
	}, {
		.limit = 148500,
		.regs = gxl_def_regs,
		.reg_num = ARRAY_SIZE(gxl_def_regs)
	}, {
		.regs = gxl_270m_regs,
		.reg_num = ARRAY_SIZE(gxl_270m_regs)
	}
};

static const struct meson_dw_hdmi_data meson_dw_hdmi_gxl_data = {
	.reg_init = meson_dw_init_regmap_gx,
	.cntl0_init = 0x0,
	.cntl1_init = PHY_CNTL1_INIT,
	.use_drm_infoframe = true,
	.speeds = gxl_speeds,
	.speed_num = ARRAY_SIZE(gxl_speeds),
};

static const struct reg_sequence g12a_3g7_regs[] = {
	{ .reg = HHI_HDMI_PHY_CNTL0, .def = 0x37eb65c4 },
	{ .reg = HHI_HDMI_PHY_CNTL3, .def = 0x2ab0ff3b },
	{ .reg = HHI_HDMI_PHY_CNTL5, .def = 0x0000080b },
};

static const struct reg_sequence g12a_3g_regs[] = {
	{ .reg = HHI_HDMI_PHY_CNTL0, .def = 0x33eb6262 },
	{ .reg = HHI_HDMI_PHY_CNTL3, .def = 0x2ab0ff3b },
	{ .reg = HHI_HDMI_PHY_CNTL5, .def = 0x00000003 },
};

static const struct reg_sequence g12a_def_regs[] = {
	{ .reg = HHI_HDMI_PHY_CNTL0, .def = 0x33eb4242 },
	{ .reg = HHI_HDMI_PHY_CNTL3, .def = 0x2ab0ff3b },
	{ .reg = HHI_HDMI_PHY_CNTL5, .def = 0x00000003 },
};

static const struct meson_dw_hdmi_speed g12a_speeds[] = {
	{
		.limit = 371250,
		.regs = g12a_3g7_regs,
		.reg_num = ARRAY_SIZE(g12a_3g7_regs)
	}, {
		.limit = 297000,
		.regs = g12a_3g_regs,
		.reg_num = ARRAY_SIZE(g12a_3g_regs)
	}, {
		.regs = g12a_def_regs,
		.reg_num = ARRAY_SIZE(g12a_def_regs)
	}
};

static const struct meson_dw_hdmi_data meson_dw_hdmi_g12a_data = {
	.reg_init = meson_dw_init_regmap_g12,
	.cntl0_init = 0x000b4242, /* Bandgap */
	.cntl1_init = PHY_CNTL1_INIT,
	.use_drm_infoframe = true,
	.speeds = g12a_speeds,
	.speed_num = ARRAY_SIZE(g12a_speeds),
};

static int meson_dw_hdmi_bind(struct device *dev, struct device *master,
			      void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	const struct meson_dw_hdmi_data *match;
	struct meson_dw_hdmi *meson_dw_hdmi;
	struct drm_device *drm = data;
	struct meson_drm *priv = drm->dev_private;
	struct dw_hdmi_plat_data *dw_plat_data;
	struct clk_bulk_data *clks;
	int irq;
	int ret;

	DRM_DEBUG_DRIVER("\n");

	match = of_device_get_match_data(&pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "failed to get match data\n");
		return -ENODEV;
	}

	meson_dw_hdmi = devm_kzalloc(dev, sizeof(*meson_dw_hdmi),
				     GFP_KERNEL);
	if (!meson_dw_hdmi)
		return -ENOMEM;

	platform_set_drvdata(pdev, meson_dw_hdmi);

	meson_dw_hdmi->priv = priv;
	meson_dw_hdmi->data = match;
	dw_plat_data = &meson_dw_hdmi->dw_plat_data;

	ret = devm_regulator_get_enable_optional(dev, "hdmi");
	if (ret < 0 && ret != -ENODEV)
		return ret;

	meson_dw_hdmi->hdmitx_apb = devm_reset_control_get_exclusive(dev,
						"hdmitx_apb");
	if (IS_ERR(meson_dw_hdmi->hdmitx_apb)) {
		dev_err(dev, "Failed to get hdmitx_apb reset\n");
		return PTR_ERR(meson_dw_hdmi->hdmitx_apb);
	}

	meson_dw_hdmi->hdmitx_ctrl = devm_reset_control_get_exclusive(dev,
						"hdmitx");
	if (IS_ERR(meson_dw_hdmi->hdmitx_ctrl)) {
		dev_err(dev, "Failed to get hdmitx reset\n");
		return PTR_ERR(meson_dw_hdmi->hdmitx_ctrl);
	}

	meson_dw_hdmi->hdmitx_phy = devm_reset_control_get_exclusive(dev,
						"hdmitx_phy");
	if (IS_ERR(meson_dw_hdmi->hdmitx_phy)) {
		dev_err(dev, "Failed to get hdmitx_phy reset\n");
		return PTR_ERR(meson_dw_hdmi->hdmitx_phy);
	}

	meson_dw_hdmi->hdmitx = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(meson_dw_hdmi->hdmitx))
		return PTR_ERR(meson_dw_hdmi->hdmitx);

	ret = devm_clk_bulk_get_all_enable(dev, &clks);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable all clocks\n");

	reset_control_reset(meson_dw_hdmi->hdmitx_apb);
	reset_control_reset(meson_dw_hdmi->hdmitx_ctrl);
	reset_control_reset(meson_dw_hdmi->hdmitx_phy);

	ret = meson_dw_hdmi->data->reg_init(dev);
	if (ret)
		return ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_threaded_irq(dev, irq, dw_hdmi_top_irq,
					dw_hdmi_top_thread_irq, IRQF_SHARED,
					"dw_hdmi_top_irq", meson_dw_hdmi);
	if (ret) {
		dev_err(dev, "Failed to request hdmi top irq\n");
		return ret;
	}

	meson_dw_hdmi_init(meson_dw_hdmi);

	/* Bridge / Connector */
	dw_plat_data->priv_data = meson_dw_hdmi;
	dw_plat_data->phy_ops = &meson_dw_hdmi_phy_ops;
	dw_plat_data->phy_name = "meson_dw_hdmi_phy";
	dw_plat_data->phy_data = meson_dw_hdmi;
	dw_plat_data->input_bus_encoding = V4L2_YCBCR_ENC_709;
	dw_plat_data->ycbcr_420_allowed = true;
	dw_plat_data->disable_cec = true;
	dw_plat_data->output_port = 1;
	dw_plat_data->use_drm_infoframe = meson_dw_hdmi->data->use_drm_infoframe;

	meson_dw_hdmi->hdmi = dw_hdmi_probe(pdev, &meson_dw_hdmi->dw_plat_data);
	if (IS_ERR(meson_dw_hdmi->hdmi))
		return PTR_ERR(meson_dw_hdmi->hdmi);

	meson_dw_hdmi->bridge = of_drm_find_bridge(pdev->dev.of_node);

	DRM_DEBUG_DRIVER("HDMI controller initialized\n");

	return 0;
}

static void meson_dw_hdmi_unbind(struct device *dev, struct device *master,
				   void *data)
{
	struct meson_dw_hdmi *meson_dw_hdmi = dev_get_drvdata(dev);

	dw_hdmi_unbind(meson_dw_hdmi->hdmi);
}

static const struct component_ops meson_dw_hdmi_ops = {
	.bind	= meson_dw_hdmi_bind,
	.unbind	= meson_dw_hdmi_unbind,
};

static int __maybe_unused meson_dw_hdmi_pm_suspend(struct device *dev)
{
	struct meson_dw_hdmi *meson_dw_hdmi = dev_get_drvdata(dev);

	if (!meson_dw_hdmi)
		return 0;

	/* FIXME: This actually bring top out reset on suspend, why ? */
	regmap_write(meson_dw_hdmi->top, HDMITX_TOP_SW_RESET, 0);

	return 0;
}

static int __maybe_unused meson_dw_hdmi_pm_resume(struct device *dev)
{
	struct meson_dw_hdmi *meson_dw_hdmi = dev_get_drvdata(dev);

	if (!meson_dw_hdmi)
		return 0;

	/* TODO: Is this really necessary/desirable on resume ? */
	reset_control_reset(meson_dw_hdmi->hdmitx_apb);
	reset_control_reset(meson_dw_hdmi->hdmitx_ctrl);
	reset_control_reset(meson_dw_hdmi->hdmitx_phy);

	meson_dw_hdmi_init(meson_dw_hdmi);

	dw_hdmi_resume(meson_dw_hdmi->hdmi);

	return 0;
}

static int meson_dw_hdmi_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &meson_dw_hdmi_ops);
}

static void meson_dw_hdmi_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &meson_dw_hdmi_ops);
}

static const struct dev_pm_ops meson_dw_hdmi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(meson_dw_hdmi_pm_suspend,
				meson_dw_hdmi_pm_resume)
};

static const struct of_device_id meson_dw_hdmi_of_table[] = {
	{ .compatible = "amlogic,meson-gxbb-dw-hdmi",
	  .data = &meson_dw_hdmi_gxbb_data },
	{ .compatible = "amlogic,meson-gxl-dw-hdmi",
	  .data = &meson_dw_hdmi_gxl_data },
	{ .compatible = "amlogic,meson-gxm-dw-hdmi",
	  .data = &meson_dw_hdmi_gxl_data },
	{ .compatible = "amlogic,meson-g12a-dw-hdmi",
	  .data = &meson_dw_hdmi_g12a_data },
	{ }
};
MODULE_DEVICE_TABLE(of, meson_dw_hdmi_of_table);

static struct platform_driver meson_dw_hdmi_platform_driver = {
	.probe		= meson_dw_hdmi_probe,
	.remove_new	= meson_dw_hdmi_remove,
	.driver		= {
		.name		= DRIVER_NAME,
		.of_match_table	= meson_dw_hdmi_of_table,
		.pm = &meson_dw_hdmi_pm_ops,
	},
};
module_platform_driver(meson_dw_hdmi_platform_driver);

MODULE_AUTHOR("Neil Armstrong <narmstrong@baylibre.com>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
