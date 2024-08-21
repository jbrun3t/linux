// SPDX-License-Identifier: GPL-2.0
//
// Copyright (c) 2024 BayLibre, SAS.
// Author: Jerome Brunet <jbrunet@baylibre.com>

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/iio/consumer.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <sound/jack.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <sound/pcm_params.h>

#include "axg-spdifin-utils.h"

#define CMDC_TOP_CTRL0			0x000
#define CMDC_TOP_CTRL1			0x004
#define  CMDC_TOP_SOFT_RST		GENMASK(4, 0)
#define CMDC_TOP_CTRL2			0x008
#define CMDC_VSM_CTRL0			0x01c
#define  CMDC_VSM_SW_STATE_EN		31
#define  CMDC_VSM_SW_STATE		GENMASK(30, 28)
#define  CMDC_VSM_ARC_INIT		BIT(27)
#define  CMDC_VSM_ARC_TERM		BIT(26)
#define  CMDC_VSM_ARC_EN		BIT(25)
#define  CMDC_VSM_COMMA_RST		BIT(19)
#define  CMDC_VSM_TIMEOUT_RST		BIT(18)
#define  CMDC_VSM_LOSTHB_RST		BIT(17)
#define  CMDC_VSM_FORCE_RST		BIT(16)
#define  CMDC_VSM_AUTO_STATE		BIT(15)
#define  CMDC_STATE_EN			BIT(14)
#define  CMDC_MAX_COMMA			GENMASK(7, 0)
#define CMDC_BIPHASE_CTRL0		0x064
#define  CMDC_BIPHASE_COMMA_RST		BIT(6)
#define CMDC_DEV_IDCTRL			0x074
#define  CMDC_APB_WRITE			BIT(31)
#define  CMDC_APB_READ			BIT(30)
#define  CMDC_APB_DONE			BIT(29)
#define  CMDC_APB_ID			GENMASK(15, 8)
#define  CMDC_APB_OFFSET		GENMASK(7, 0)
#define CMDC_DEV_WDATA			0x078
#define CMDC_DEV_RDATA			0x07c
#define CMDC_ANA_STAT0			0x098
#define CMDC_STATUS0			0x09c
#define  CMDC_STATE_SHIFT		0
#define  CMDC_STATE_WIDTH		2
#define  CMDC_STATE			GENMASK(CMDC_STATE_SHIFT + CMDC_STATE_WIDTH, \
						CMDC_STATE_SHIFT)
#define CMDC_STATUS1			0x0a0
#define CMDC_STATUS2			0x0a4
#define CMDC_STATUS3			0x0a8
#define CMDC_STATUS4			0x0ac
#define CMDC_STATUS5			0x0b0
#define CMDC_STATUS6			0x0b4

#define DMAC_TOP_CTRL0			0x400
#define  DMAC_TOP_EN			BIT(31)
#define DMAC_SYNC_CTRL0			0x404
#define  DMAC_SYNC_EN			BIT(31)
#define  DMAC_SYNC_FIFO_RST_OUT		BIT(30)
#define  DMAC_SYNC_FIFO_RST_IN		BIT(29)
#define  DMAC_SYNC_DELAY_EN		BIT(16)
#define  DMAC_SYNC_DELAY_CYCLES		GENMASK(14, 12)
#define  DMAC_SYNC_VALID_CLEAR		GENMASK(10, 8)
#define  DMAC_SYNC_VALID_SET		GENMASK(6, 4)
#define SPDIFIN_SAMPLE_CTRL0		0x40c
#define  SPDIFIN_SAMPLE_EN		BIT(31)
#define  SPDIFIN_DET_WIDTH_SEL		BIT(28)
#define  SPDIFIN_DET_BASE_TIMER		GENMASK(19, 0)
#define SPDIFIN_SAMPLE_CTRL1		0x410
#define  SPDIFIN_THRES_PER_REG		3
#define  SPDIFIN_THRES_WIDTH		10
#define SPDIFIN_SAMPLE_CTRL3		0x418
#define  SPDIFIN_TIMER_PER_REG		4
#define  SPDIFIN_TIMER_WIDTH		8
#define SPDIFIN_SAMPLE_STAT0		0x424
#define  SPDIFIN_MODE			GENMASK(30, 28)
#define  SPDIFIN_MINW			GENMASK(27, 18)
#define  SPDIFIN_MAXW			GENMASK(17, 8)
#define SPDIFIN_CTRL0			0x430
#define  SPDIFIN_EN			BIT(31)
#define  SPDIFIN_STATUS_CH		11
#define  SPDIFIN_STATUS_SEL		GENMASK(10, 8)
#define  SPDIFIN_PARITY			BIT(2)
#define SPDIFIN_CTRL1			0x434
#define SPDIFIN_CTRL2			0x438
#define SPDIFIN_CTRL3			0x43c
#define SPDIFIN_STAT1			0x444
#define DMAC_UBIT_CTRL0			0x44c
#define  DMAC_UBIT_EN			BIT(31)
#define UI_RDATA			0x450
#define ERR_CORRECT_CTRL0		0x458
#define  ERR_CORRECT_EN			BIT(31)
#define  ERR_FIFO_RST_OUT		BIT(29)
#define  ERR_FIFO_RST_IN		BIT(28)
#define  ERR_FORCE_MODE			BIT(0)
#define ANA_RST_CTRL0			0x460
#define  ANA_RST_EN			BIT(31)
#define  ANA_RST_EN_WITH_TOP		BIT(28)
#define  ANA_RST_DIV2_THRES		GENMASK(19, 0)
#define ANA_RST_CTRL1			0x464

#define TOP_CTRL0			0x600
#define TOP_DMAC_INT_MASK		0x604
#define TOP_DMAC_INT_PENDING		0x608
#define  TOP_DMAC_INT_ANA_FMT		BIT(17)
#define  TOP_DMAC_INT_ANA_DIV2		BIT(16)
#define  TOP_DMAC_INT_ERRC_BCH		BIT(15)
#define  TOP_DMAC_INT_ERRC_AFO		BIT(14)
#define  TOP_DMAC_INT_ERRC_FO		BIT(13)
#define  TOP_DMAC_INT_UBC_FO		BIT(12)
#define  TOP_DMAC_INT_UBC_FTP		BIT(11)
#define  TOP_DMAC_INT_UBC_UPL		BIT(10)
#define  TOP_DMAC_INT_UBC_IPE		BIT(9)
#define  TOP_DMAC_INT_BID_CMC		BIT(8)
#define  TOP_DMAC_INT_BID_FPAPB		BIT(7)
#define  TOP_DMAC_INT_BID_VC		BIT(6)
#define  TOP_DMAC_INT_BID_FN2P		BIT(5)
#define  TOP_DMAC_INT_BID_PCPDC		BIT(4)
#define  TOP_DMAC_INT_BID_CSC		BIT(3)
#define  TOP_DMAC_INT_BID_SMC		BIT(2)
#define  TOP_DMAC_INT_BID_PE		BIT(1)
#define  TOP_DMAC_INT_SYNC_AFO		BIT(0)
#define TOP_CMDC_INT_MASK		0x60c
#define TOP_CMDC_INT_PENDING		0x610
#define  TOP_CMDC_INT_IDLE2		BIT(15)
#define  TOP_CMDC_INT_IDLE1		BIT(14)
#define  TOP_CMDC_INT_DISC2		BIT(13)
#define  TOP_CMDC_INT_DISC1		BIT(12)
#define  TOP_CMDC_INT_EARC		BIT(11)
#define  TOP_CMDC_INT_HB_STATUS		BIT(10)
#define  TOP_CMDC_INT_HB_LOST		BIT(9)
#define  TOP_CMDC_INT_TIMEOUT		BIT(8)
#define  TOP_CMDC_INT_STATUS_CH		BIT(7)
#define  TOP_CMDC_INT_RECV_INVID	BIT(6)
#define  TOP_CMDC_INT_RECV_INVOFF	BIT(5)
#define  TOP_CMDC_INT_RECV_UNEXP	BIT(4)
#define  TOP_CMDC_INT_RECV_ECC		BIT(3)
#define  TOP_CMDC_INT_RECV_PARITY	BIT(2)
#define  TOP_CMDC_INT_RECV_PACKET	BIT(1)
#define  TOP_CMDC_INT_RECV_TIMEOUT	BIT(0)
#define TOP_ANA_CTRL0			0x614
#define  TOP_ANA_EN_D2A			BIT(31)
#define  TOP_ANA_RX_REFTRIM		GENMASK(28, 24)
#define  TOP_ANA_IDR_TRIM		GENMASK(23, 20)
#define  TOP_ANA_RTERM_TRIM		GENMASK(19, 15)
#define  TOP_ANA_TX_HYSTRIM		GENMASK(14, 12)
#define  TOP_ANA_TX_REFTRIM		GENMASK(11, 7)
#define  TOP_ANA_RX_VREFON		BIT(6)
#define  TOP_ANA_RX_RCFILTER		GENMASK(5, 4)
#define  TOP_ANA_RX_HYSTRIM		GENMASK(2, 0)
#define TOP_ANA_CTRL1			0x618
#define  TOP_ANA_RTERM_CAL_RSTN		BIT(31)
#define  TOP_ANA_RTERM_CAL_EN		BIT(30)
#define TOP_ANA_STAT0			0x61c
#define  TOP_ANA_RTERM_CAL_DONE		BIT(31)
#define  TOP_ANA_RTERM_CAL_CODE		GENMASK(4, 0)
#define TOP_PLL_CTRL0			0x620
#define  TOP_PLL_SELF_RST		BIT(29)
#define  TOP_PLL_EN			BIT(28)
#define TOP_PLL_CTRL1			0x624
#define TOP_PLL_CTRL2			0x628
#define TOP_PLL_CTRL3			0x62c
#define  TOP_PLL_TDC_MODE		BIT(15)
#define TOP_PLL_STAT			0x630
#define  TOP_PLL_DMAC_VALID		BIT(31)
#define  TOP_PLL_DMAC_VALIDA		BIT(30)
#define  TOP_PLL_AFC_DONE		BIT(29)
#define  TOP_PLL_REG_OUT		GENMASK(9, 0)

#define CMDC_IRQS (TOP_CMDC_INT_IDLE2 |		\
		   TOP_CMDC_INT_IDLE1 |		\
		   TOP_CMDC_INT_DISC2 |		\
		   TOP_CMDC_INT_DISC1 |		\
		   TOP_CMDC_INT_EARC |		\
		   TOP_CMDC_INT_HB_LOST |	\
		   TOP_CMDC_INT_TIMEOUT |	\
		   TOP_CMDC_INT_STATUS_CH )

#define DMAC_IRQS (TOP_DMAC_INT_ANA_FMT |	\
		   TOP_DMAC_INT_ANA_DIV2 |	\
		   TOP_DMAC_INT_ERRC_BCH |	\
		   TOP_DMAC_INT_ERRC_AFO |	\
		   TOP_DMAC_INT_ERRC_FO |	\
		   TOP_DMAC_INT_BID_CMC |	\
		   TOP_DMAC_INT_BID_VC |	\
		   TOP_DMAC_INT_BID_CSC |	\
		   TOP_DMAC_INT_BID_SMC |	\
		   TOP_DMAC_INT_BID_PE |	\
		   TOP_DMAC_INT_SYNC_AFO )

#define EARC_STAT_ID		0x74
#define  EARC_RX_STAT		0xd0
#define  EARC_TX_STAT		0xd1
#define   EARC_HPD		BIT(0)
#define   EARC_CAP_CHNG		BIT(3)
#define   EARC_STAT_CHNG	BIT(4)
#define   EARC_VALID		BIT(7)
#define  EARC_RX_LATENCY	0xd2
#define  EARC_RX_LATENCY_REQ	0xd3
#define EARC_CDS_ID		0xa0
#define  EARC_CDS_MAX_SIZE	256

/*
 * As described in the HDMI specification allow a maximum of 11
 * comma sequences before timing out to IDLE2
 */
#define EARC_MAX_COMMAS		11

enum cmdc_state {
	STATE_OFF,
	STATE_IDLE1,
	STATE_IDLE2,
	STATE_DISC1,
	STATE_DISC2,
	STATE_EARC,
	STATE_ARC,
};

static const unsigned int arc_mode_rates[] = {
	32000,   44100,  48000,  88200,  96000, 176400, 192000,
};

struct sm1_earcrx_cfg {
	const unsigned int *modes;
	unsigned int mode_num;
	const struct reg_sequence *pll_init;
	unsigned int pll_init_num;
};

struct sm1_earcrx {
	const struct sm1_earcrx_cfg *conf;
	struct regmap *map;
	struct clk *pclk;
	struct clk *cmdc_clk;
	struct clk *dmac_clk;
	struct iio_channel *earc_pll;
	struct mutex lock;
	struct snd_pcm_substream *substream;
	struct snd_soc_jack jack;
	unsigned int rate;
	bool rterm_done;
	int earc_irq;
	int hpd_irq;
};

static int top_rterm_calibration(struct sm1_earcrx *ed)
{
	unsigned int val, code = 0;
	int ret;

	regmap_update_bits(ed->map, TOP_ANA_CTRL1,
			   TOP_ANA_RTERM_CAL_RSTN | TOP_ANA_RTERM_CAL_EN,
			   TOP_ANA_RTERM_CAL_RSTN | TOP_ANA_RTERM_CAL_EN);

	/* Give it a second maximum to get the calibration done */
	ret = regmap_read_poll_timeout(ed->map, TOP_ANA_STAT0, val,
				       val & TOP_ANA_RTERM_CAL_DONE,
				       20, 1000000);

	if (ret) {
		pr_err("Failed to calibrate rterm: %d\n", ret);
	} else {
		code = FIELD_GET(TOP_ANA_RTERM_CAL_CODE, val);
		regmap_update_bits(ed->map, TOP_ANA_CTRL0,
				   TOP_ANA_RTERM_TRIM,
				   FIELD_PREP(TOP_ANA_RTERM_TRIM, code));
	}

	regmap_update_bits(ed->map, TOP_ANA_CTRL1,
			   TOP_ANA_RTERM_CAL_RSTN | TOP_ANA_RTERM_CAL_EN,
			   0);

	return ret;
}

static void cmdc_apb_prep(struct regmap *map, unsigned int devid,
			  unsigned int offset, unsigned int wflag)
{
	unsigned int val = wflag;

	val |= FIELD_PREP(CMDC_APB_ID, devid);
	val |= FIELD_PREP(CMDC_APB_OFFSET, offset);

	regmap_write(map, CMDC_DEV_IDCTRL, val);
}

static void cmdc_apb_done(struct regmap *map)
{
	regmap_update_bits(map, CMDC_DEV_IDCTRL, CMDC_APB_DONE, CMDC_APB_DONE);
	regmap_update_bits(map, CMDC_DEV_IDCTRL, CMDC_APB_DONE, 0);
}

static void cmdc_apb_read(struct regmap *map, unsigned int devid,
			  unsigned int offset, u8 *data, unsigned int bytes)
{
	unsigned int val;
	int i;

	cmdc_apb_prep(map, devid, offset, CMDC_APB_READ);

	for (i = 0; i < bytes; i++) {
		regmap_read(map, CMDC_DEV_RDATA, &val);
		data[i] = val;
	}

	cmdc_apb_done(map);
}

static void cmdc_apb_write(struct regmap *map, unsigned int devid,
			   unsigned int offset, const u8 *data,
			   unsigned int bytes)
{
	int i;

	cmdc_apb_prep(map, devid, offset, CMDC_APB_WRITE);

	for (i = 0; i < bytes; i++)
		regmap_write(map, CMDC_DEV_WDATA, data[i]);

	cmdc_apb_done(map);
}

static void cmdc_ack_rx_stat(struct sm1_earcrx *ed, u8 chng)
{
	u8 stat;

	cmdc_apb_read(ed->map, EARC_STAT_ID, EARC_RX_STAT, &stat, 1);
	stat |= chng;
	cmdc_apb_write(ed->map, EARC_STAT_ID, EARC_RX_STAT, &stat, 1);
}

static void cmdc_refresh_cds(struct sm1_earcrx *ed)
{
	u8 val;

	/*
	 * If there is already a Capability Data Structure in memory,
	 * Do not rewrite it, just notify the eARC source CDS is ready.
	 */
	cmdc_apb_read(ed->map, EARC_CDS_ID, 0, &val, 1);
	if (val)
	        cmdc_ack_rx_stat(ed, EARC_CAP_CHNG);
}

static unsigned int cmdc_comma_read(struct sm1_earcrx *ed)
{
	unsigned int val;

	regmap_read(ed->map, CMDC_VSM_CTRL0, &val);

	return FIELD_GET(CMDC_MAX_COMMA, val);
}

static void cmdc_comma_reset(struct sm1_earcrx *ed, unsigned int max)
{
	regmap_update_bits(ed->map, CMDC_VSM_CTRL0,
			   CMDC_VSM_COMMA_RST | CMDC_MAX_COMMA,
			   CMDC_VSM_COMMA_RST | FIELD_PREP(CMDC_MAX_COMMA, max));

	regmap_update_bits(ed->map, CMDC_VSM_CTRL0,
			   CMDC_VSM_COMMA_RST,
			   0);

	/* NOTE: CMDC_STATUS0 19:16 reports current comma sequence number */
}

static int cmdc_get_state(struct sm1_earcrx *ed)
{
	unsigned int val;

	regmap_read(ed->map, CMDC_STATUS0, &val);

	return FIELD_GET(CMDC_STATE, val);
}

static bool cmdc_is_capture_state(unsigned int state)
{
	switch (state) {
	case STATE_ARC:
	case STATE_EARC:
		return true;
	}

	return false;
}

static bool cmdc_is_plugged_state(unsigned int state)
{
	switch (state) {
	case STATE_OFF:
	case STATE_IDLE1:
		return false;
	}

	return true;
}

static void cmdc_start(struct sm1_earcrx *ed)
{
	regmap_update_bits(ed->map, CMDC_VSM_CTRL0,
			   CMDC_VSM_ARC_EN | CMDC_VSM_ARC_TERM | CMDC_VSM_ARC_INIT,
			   CMDC_VSM_ARC_EN | CMDC_VSM_ARC_INIT);
	regmap_update_bits(ed->map, CMDC_TOP_CTRL1,
			   CMDC_TOP_SOFT_RST, 0);
}

static void cmdc_stop(struct sm1_earcrx *ed)
{
	regmap_update_bits(ed->map, CMDC_TOP_CTRL1,
			   CMDC_TOP_SOFT_RST, CMDC_TOP_SOFT_RST);
	regmap_update_bits(ed->map, CMDC_VSM_CTRL0,
			   CMDC_VSM_ARC_EN | CMDC_VSM_ARC_TERM | CMDC_VSM_ARC_INIT,
			   CMDC_VSM_ARC_TERM);
}

static void cmdc_init(struct sm1_earcrx *ed)
{
	/* Make sure the CMDC is stopped */
	cmdc_stop(ed);

	/* Initialize comma sequence number limit */
	cmdc_comma_reset(ed, EARC_MAX_COMMAS);

	/* Setup the CMDC IRQs */
	regmap_write(ed->map, TOP_CMDC_INT_PENDING, GENMASK(31, 0));
	regmap_write(ed->map, TOP_CMDC_INT_MASK, CMDC_IRQS);
}

static void dmac_spdifin_mode_config(struct sm1_earcrx *ed)
{
	/* Threshold based on the maximum width between two edges */
	regmap_update_bits(ed->map, SPDIFIN_SAMPLE_CTRL0,
			   SPDIFIN_DET_WIDTH_SEL, 0);

	meson_spdifin_sample_mode_config(ed->map, ed->dmac_clk,
					 SPDIFIN_SAMPLE_CTRL0,
					 ed->conf->modes);
}

/*
 * NOTE: Identifying the eARC rate is tricky.
 * The usual SPDIF mode does not work for eARC as it does for ARC, as data
 * flows through the DMAC instead of the SPDIF input.
 * The DMAC has a PLL that must lock in eARC mode but there is no direct
 * way to query the clock rate locked by the PLL.
 *
 * Amlogic vendor code ignore this and rely only on the channel status
 * word once the PLL is locked. It works but it it is fragile as the rate
 * may be not be indicated in the channel status word.
 *
 * The Clock measure HW is able to check system clock rates including
 * the eARC PLL rate. The measured rate is not exact but it is fairly
 * precise. It is enough considering that the eARC possible rates are
 * known. Go through IIO to get that value.
 */
static unsigned int dmac_earc_get_rate(struct sm1_earcrx *ed)
{
	int ret, ioval, i = 0, delta = INT_MAX;
	unsigned int val, fs, chmod = 1;

	regmap_read(ed->map, TOP_PLL_STAT, &val);
	if (!(val & (TOP_PLL_DMAC_VALID | TOP_PLL_DMAC_VALIDA)))
		return 0;

	/*
	 * Reading may fail if the PLL changes/unlock while reading.
	 * This may happen when the source continously restart the stream,
	 * like when skipping on the source media.
	 * eARC will be notified again when the PLL stabilize so just
	 * report 0 in the meantime.
	 */
	ret = iio_read_channel_processed(ed->earc_pll, &ioval);
	if (ret < 0)
		return 0;

	/* Convert the SPDIF base rate locked to the SPDIF frame rate */
	ioval /= 128;

	/* Find the closest valid eARC rate */
	while (i < ed->conf->mode_num) {
		unsigned int f = chmod * ed->conf->modes[i];
		int d = ioval - f;

		/* Save the best match */
		if (abs(d) < abs(delta)) {
			fs = f;
			delta = d;
		}

		/* Do spdif rates in 2ch mode then 8ch mode */
		if ((chmod == 1) &&
		    (i == ed->conf->mode_num - 1)) {
			chmod = 4;
			i = 0;
		} else {
			i += 1;
		}
	}

	return fs;
}

static void dmac_capture_start(struct sm1_earcrx *ed, unsigned int state)
{
	regmap_update_bits(ed->map, ERR_CORRECT_CTRL0,
			   ERR_FIFO_RST_OUT,
			   ERR_FIFO_RST_OUT);
	regmap_update_bits(ed->map, ERR_CORRECT_CTRL0,
			   ERR_FIFO_RST_IN,
			   ERR_FIFO_RST_IN);
	regmap_update_bits(ed->map, ERR_CORRECT_CTRL0,
			   ERR_CORRECT_EN,
			   ERR_CORRECT_EN);
}

static void dmac_capture_stop(struct sm1_earcrx *ed)
{
	regmap_update_bits(ed->map, ERR_CORRECT_CTRL0,
			   ERR_CORRECT_EN |
			   ERR_FIFO_RST_OUT |
			   ERR_FIFO_RST_IN, 0);
}

static void dmac_monitor_start(struct sm1_earcrx *ed, unsigned int state)
{
	/* Provide CDS again if it is already set */
	if (state == STATE_EARC)
		cmdc_refresh_cds(ed);

	regmap_update_bits(ed->map, DMAC_TOP_CTRL0,
			   DMAC_TOP_EN, DMAC_TOP_EN);

	if (state == STATE_EARC) {
		regmap_update_bits(ed->map, DMAC_SYNC_CTRL0,
				   DMAC_SYNC_EN,
				   DMAC_SYNC_EN);

		regmap_update_bits(ed->map, DMAC_SYNC_CTRL0,
				   DMAC_SYNC_FIFO_RST_OUT,
				   DMAC_SYNC_FIFO_RST_OUT);
		regmap_update_bits(ed->map, DMAC_SYNC_CTRL0,
				   DMAC_SYNC_FIFO_RST_IN,
				   DMAC_SYNC_FIFO_RST_IN);
	} else {
		regmap_update_bits(ed->map, SPDIFIN_CTRL0,
				   SPDIFIN_PARITY, SPDIFIN_PARITY);

		regmap_update_bits(ed->map, SPDIFIN_SAMPLE_CTRL0,
				   SPDIFIN_SAMPLE_EN,
				   SPDIFIN_SAMPLE_EN);
	}

	regmap_update_bits(ed->map, SPDIFIN_CTRL0,
			   SPDIFIN_EN, SPDIFIN_EN);
}

static void dmac_monitor_stop(struct sm1_earcrx *ed)
{
	regmap_update_bits(ed->map, DMAC_SYNC_CTRL0,
			   DMAC_SYNC_EN |
			   DMAC_SYNC_FIFO_RST_OUT |
			   DMAC_SYNC_FIFO_RST_IN, 0);
	regmap_update_bits(ed->map, SPDIFIN_CTRL0,
			   SPDIFIN_EN | SPDIFIN_PARITY, 0);
	regmap_update_bits(ed->map, DMAC_TOP_CTRL0,
			   DMAC_TOP_EN, 0);
	regmap_update_bits(ed->map, SPDIFIN_SAMPLE_CTRL0,
			   SPDIFIN_SAMPLE_EN, 0);
}

static void dmac_init(struct sm1_earcrx *ed)
{
	/* Make sure DMAC and SPDIF path are stopped */
	dmac_capture_stop(ed);
	dmac_monitor_stop(ed);

	/* Setup SPDIF capture timers */
	dmac_spdifin_mode_config(ed);

	regmap_write(ed->map, DMAC_SYNC_CTRL0,
		     DMAC_SYNC_DELAY_EN |
		     FIELD_PREP(DMAC_SYNC_DELAY_CYCLES, 3) |
		     FIELD_PREP(DMAC_SYNC_VALID_CLEAR, 7) |
		     FIELD_PREP(DMAC_SYNC_VALID_SET, 7));

	regmap_write(ed->map, ANA_RST_CTRL0,
		     ANA_RST_EN |
		     ANA_RST_EN_WITH_TOP |
		     FIELD_PREP(ANA_RST_DIV2_THRES, 2000));

	/* Setup PA/PB */
	regmap_write(ed->map, SPDIFIN_CTRL3, 0xec375a5a);

	/*
	 * Set err correct in force mode
	 * NOTE: no idea what it does but TOP_DMAC_INT_ERRC_BCH flood the
	 * system if this is off, as soon as ERR_CORRECT_EN is on.
	 * Data looks correct with this bit set, so keep it for now
	 */
	regmap_update_bits(ed->map, ERR_CORRECT_CTRL0,
			   ERR_FORCE_MODE, ERR_FORCE_MODE);

	/* Setup DMAC IRQs */
	regmap_write(ed->map, TOP_DMAC_INT_PENDING, GENMASK(31, 0));
	regmap_write(ed->map, TOP_DMAC_INT_MASK, DMAC_IRQS);
}

static irqreturn_t sm1_earcrx_dmac_isr(int irq, void *dev_id)
{
	struct snd_soc_dai *dai = dev_id;
	struct sm1_earcrx *ed = snd_soc_dai_get_drvdata(dai);
	unsigned int dmac, cmdc;

	regmap_read(ed->map, TOP_DMAC_INT_PENDING, &dmac);
	regmap_read(ed->map, TOP_CMDC_INT_PENDING, &cmdc);

	/* Ack IRQ early so we don't miss any */
	regmap_write(ed->map, TOP_DMAC_INT_PENDING, dmac);
	regmap_write(ed->map, TOP_CMDC_INT_PENDING, cmdc);

	/* Re-arm rterm calibration when HPD goes through a low state */
	if (cmdc & TOP_CMDC_INT_IDLE1)
	        ed->rterm_done = false;

	return IRQ_WAKE_THREAD;
}

static unsigned int sm1_earcrx_get_rate(struct sm1_earcrx *ed, unsigned int state)
{
	if (state == STATE_ARC)
		return meson_spdifin_get_rate(ed->map, SPDIFIN_SAMPLE_STAT0,
					      ed->conf->modes);
	else if (state == STATE_EARC)
		return dmac_earc_get_rate(ed);
	else
		return 0;
}

static irqreturn_t sm1_earcrx_dmac_isr_thread(int irq, void *dev_id)
{
	struct snd_soc_dai *dai = dev_id;
	struct sm1_earcrx *ed = snd_soc_dai_get_drvdata(dai);
	unsigned int state = cmdc_get_state(ed);
	unsigned int rate, vsm;
	bool valid_change;

	/* Deal with rterm calibration on cable insertion */
	if (cmdc_is_plugged_state(state) && !ed->rterm_done) {
		top_rterm_calibration(ed);
		ed->rterm_done = true;
	}

	regmap_read(ed->map, CMDC_VSM_CTRL0, &vsm);

	/* Sync rate monitoring with state */
	if (cmdc_is_capture_state(state) &&
	    (vsm & CMDC_VSM_ARC_INIT)) {
		dmac_monitor_start(ed, state);
		regmap_update_bits(ed->map, CMDC_VSM_CTRL0,
				   CMDC_VSM_ARC_INIT, 0);

	} else if (!cmdc_is_capture_state(state) &&
		   !(vsm & CMDC_VSM_ARC_INIT)) {
		dmac_monitor_stop(ed);
		regmap_update_bits(ed->map, CMDC_VSM_CTRL0,
				   CMDC_VSM_ARC_INIT, CMDC_VSM_ARC_INIT);
	}

	/* Refresh cached rate */
	mutex_lock(&ed->lock);
	rate = sm1_earcrx_get_rate(ed, state);

	/* Check if the rate validity changed */
	valid_change = !!ed->rate != !!rate;

	/*
	 * If the stream rate changes while capturing, capture must
	 * restart to reset the constraints on the stream
	 */
	if ((ed->rate != rate) && ed->substream) {
		snd_soc_dpcm_be_stop(ed->substream, SNDRV_PCM_STATE_DISCONNECTED);
		ed->substream = NULL;
	}

	ed->rate = rate;
	mutex_unlock(&ed->lock);

	if (valid_change)
		snd_soc_jack_report(&ed->jack,
				    rate ? SND_JACK_LINEIN : 0,
				    SND_JACK_LINEIN);

	return IRQ_HANDLED;
}

/*
 * NOTE: the CMDC state machine has problems with H14b ARC mode. The CMDC
 * correctly follows the state machine as described in the HDMI spec until
 * it transitions from IDLE2 to H14b ARC. In this state, if the cable is
 * unplugged, CMDC should in theory transition to IDLE2 then IDLE1 but CMDC
 * stays in ARC mode until the init bit is cleared and term bit is set.
 * Only then will the state machine kick again.
 *
 * To work around this, the HPD must be monitored to manually terminate ARC
 * mode.
 */
static irqreturn_t sm1_earcrx_hpd_isr(int irq, void *dev_id)
{
	struct snd_soc_dai *dai = dev_id;
	struct sm1_earcrx *ed = snd_soc_dai_get_drvdata(dai);

	regmap_update_bits(ed->map, CMDC_VSM_CTRL0,
			   CMDC_VSM_ARC_TERM | CMDC_VSM_ARC_INIT,
			   CMDC_VSM_ARC_TERM);
	regmap_update_bits(ed->map, CMDC_VSM_CTRL0,
			   CMDC_VSM_ARC_TERM | CMDC_VSM_ARC_INIT,
			   0);

	return IRQ_HANDLED;
}

static int sm1_earcrx_dai_prepare(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct sm1_earcrx *ed = snd_soc_dai_get_drvdata(dai);
	unsigned int state = cmdc_get_state(ed);

	/* Reset DMAC Capture */
	dmac_capture_stop(ed);
	dmac_capture_start(ed, state);

	return 0;
}

static int sm1_earcrx_dai_hw_free(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct sm1_earcrx *ed = snd_soc_dai_get_drvdata(dai);

	dmac_capture_stop(ed);
	return 0;
}

static int sm1_earcrx_dai_startup(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct sm1_earcrx *ed = snd_soc_dai_get_drvdata(dai);
	int ret = 0;

        guard(mutex)(&ed->lock);

	/* Only allow start if a rate is locked from ARC or eARC */
	if (!ed->rate)
	        return -EIO;

	/* Save substream to stop from IRQ if necessary */
	ed->substream = substream;

	/* Only support the rate provided by the source */
	ret = snd_pcm_hw_constraint_single(substream->runtime,
					   SNDRV_PCM_HW_PARAM_RATE,
					   ed->rate);
	if (ret < 0)
		dev_err(dai->dev, "can't set eARC rate constraint\n");

	return ret;
}

static void sm1_earcrx_dai_shutdown(struct snd_pcm_substream *substream,
				    struct snd_soc_dai *dai)
{
	struct sm1_earcrx *ed = snd_soc_dai_get_drvdata(dai);

	guard(mutex)(&ed->lock);
	ed->substream = NULL;
}

static int sm1_earcrx_dai_probe(struct snd_soc_dai *dai)
{
	struct sm1_earcrx *ed = snd_soc_dai_get_drvdata(dai);
	struct device *dev = dai->dev;
	int ret;

	ret = request_threaded_irq(ed->earc_irq, sm1_earcrx_dmac_isr,
				   sm1_earcrx_dmac_isr_thread,
				   IRQF_ONESHOT, "earcrx", dai);
	if (ret) {
		dev_err(dev, "failed to install dmac isr\n");
		return ret;
	}

	ret = request_irq(ed->hpd_irq, sm1_earcrx_hpd_isr, 0, "earcrx-hpd",
			  dai);
	if (ret) {
		dev_err(dev, "failed to install hpd isr\n");
		goto earc_irq;
	}

	/* Initialize DMAC PLL */
	regmap_multi_reg_write(ed->map,
			       ed->conf->pll_init,
			       ed->conf->pll_init_num);

	/* Initially report JACK is disconnected */
	snd_soc_jack_report(&ed->jack, 0, SND_JACK_LINEIN);

	/* Power up the top block */
	regmap_update_bits(ed->map, TOP_ANA_CTRL0,
			   TOP_ANA_EN_D2A, TOP_ANA_EN_D2A);

	/* Finally start the eARC state machine */
	cmdc_start(ed);

	return 0;

earc_irq:
	free_irq(ed->earc_irq, dai);
	return ret;
}

static int sm1_earcrx_dai_remove(struct snd_soc_dai *dai)
{
	struct sm1_earcrx *ed = snd_soc_dai_get_drvdata(dai);

	free_irq(ed->hpd_irq, dai);
	free_irq(ed->earc_irq, dai);
	dmac_monitor_stop(ed);
	cmdc_stop(ed);

	return 0;
}

static int sm1_earcrx_component_probe(struct snd_soc_component *component)
{
	struct sm1_earcrx *ed = snd_soc_component_get_drvdata(component);
	struct device *dev = component->dev;
	int ret;

	/* Insert eARC jack in the card */
	ret = snd_soc_card_jack_new(component->card, "ARC/eARC Jack",
				    SND_JACK_LINEIN, &ed->jack);

	/* Clock TOP registers */
	ret = clk_prepare_enable(ed->pclk);
	if (ret) {
		dev_err(dev, "failed to enable pclk\n");
		return ret;
	}

	/* Clock CMDC and stop it */
	ret = clk_prepare_enable(ed->cmdc_clk);
	if (ret) {
		dev_err(dev, "failed to enable cmdc clk\n");
		goto pclk;
	}
	cmdc_init(ed);

	/* Same for the DMAC */
	ret = clk_prepare_enable(ed->dmac_clk);
	if (ret) {
		dev_err(dev, "failed to enable dmac clk\n");
		goto cmdc_clk;
	}
	dmac_init(ed);

	/* Finally setup TOP analog */
	regmap_write(ed->map, TOP_ANA_CTRL0,
		     FIELD_PREP(TOP_ANA_RX_REFTRIM,  16) |
		     FIELD_PREP(TOP_ANA_IDR_TRIM,    8)  |
		     FIELD_PREP(TOP_ANA_RTERM_TRIM,  16) |
		     FIELD_PREP(TOP_ANA_TX_HYSTRIM,  4)  |
		     FIELD_PREP(TOP_ANA_TX_REFTRIM,  16) |
		     FIELD_PREP(TOP_ANA_RX_RCFILTER, 1)  |
		     FIELD_PREP(TOP_ANA_RX_HYSTRIM,  4));

	return 0;

cmdc_clk:
	clk_disable_unprepare(ed->cmdc_clk);
pclk:
	clk_disable_unprepare(ed->pclk);

	return ret;
}

static void sm1_earcrx_component_remove(struct snd_soc_component *component)
{
	struct sm1_earcrx *ed = snd_soc_component_get_drvdata(component);

	clk_disable_unprepare(ed->dmac_clk);
	clk_disable_unprepare(ed->cmdc_clk);
	clk_disable_unprepare(ed->pclk);
}

struct apb_byte_control {
	unsigned int devid;
	unsigned int offset;
	unsigned int size;
	u8 chng;
};

static int sm1_earcrx_apb_bytes_info(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_info *uinfo)
{
	struct apb_byte_control *abc =
		(struct apb_byte_control *)kcontrol->private_value;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_BYTES;
	uinfo->count = abc->size;

	return 0;
}

static int sm1_earcrx_apb_bytes_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct sm1_earcrx *ed = snd_soc_component_get_drvdata(component);
	struct apb_byte_control *abc =
		(struct apb_byte_control *)kcontrol->private_value;
	u8 *data = (u8 *)ucontrol->value.bytes.data;

	cmdc_apb_read(ed->map, abc->devid, abc->offset, data, abc->size);

	return 0;

}

static int sm1_earcrx_apb_bytes_put(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct sm1_earcrx *ed = snd_soc_component_get_drvdata(component);
	struct apb_byte_control *abc =
		(struct apb_byte_control *)kcontrol->private_value;
	u8 *data = (u8 *)ucontrol->value.bytes.data;

	cmdc_apb_write(ed->map, abc->devid, abc->offset, data, abc->size);

	if (abc->chng)
	        cmdc_ack_rx_stat(ed, abc->chng);

	return 0;
}

static int sm1_earcrx_apb_u8_info(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_info *uinfo)
{
	struct apb_byte_control *abc =
		(struct apb_byte_control *)kcontrol->private_value;

	if (abc->size != 1)
		return -EINVAL;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 255;
	uinfo->count = 1;

	return 0;
}

static int sm1_earcrx_apb_u8_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct sm1_earcrx *ed = snd_soc_component_get_drvdata(component);
	struct apb_byte_control *abc =
		(struct apb_byte_control *)kcontrol->private_value;
	u8 val;

	cmdc_apb_read(ed->map, abc->devid, abc->offset, &val, 1);
	ucontrol->value.integer.value[0] = val;

	return 0;

}

static int sm1_earcrx_apb_u8_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct sm1_earcrx *ed = snd_soc_component_get_drvdata(component);
	struct apb_byte_control *abc =
		(struct apb_byte_control *)kcontrol->private_value;
	u8 val = ucontrol->value.integer.value[0];

	cmdc_apb_write(ed->map, abc->devid, abc->offset, &val, 1);

	if (abc->chng)
	        cmdc_ack_rx_stat(ed, abc->chng);

	return 0;
}

static int sm1_earcrx_get_comma_en(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_dapm_kcontrol_to_component(kcontrol);
	struct sm1_earcrx *ed = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] =
		cmdc_comma_read(ed) == EARC_MAX_COMMAS;

	return 0;
}

static int sm1_earcrx_set_comma_en(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_dapm_kcontrol_to_component(kcontrol);
	struct sm1_earcrx *ed = snd_soc_component_get_drvdata(component);
	unsigned int comma = ucontrol->value.integer.value[0] ?
		EARC_MAX_COMMAS : 0;

	if (cmdc_comma_read(ed) != comma) {
		cmdc_comma_reset(ed, comma);
		return 1;
	}

	return 0;
}

static const char * const sm1_earcrx_chsts_src_texts[] = {
	"A", "B",
};

static SOC_ENUM_SINGLE_DECL(sm1_earcrx_chsts_src_enum, SPDIFIN_CTRL0,
			    SPDIFIN_STATUS_CH,
			    sm1_earcrx_chsts_src_texts);

static const char * const sm1_earcrx_cmdc_state_texts[] = {
	"OFF", "IDLE1", "IDLE2", "DISC1", "DISC2", "eARC", "ARC"
};

static SOC_ENUM_SINGLE_DECL(sm1_earcrx_cmdc_state_enum, CMDC_STATUS0,
			    CMDC_STATE_SHIFT,
			    sm1_earcrx_cmdc_state_texts);

#define SOC_ENUM_RO_V(xname, xenum)				\
{								\
	.access = (SNDRV_CTL_ELEM_ACCESS_READ |			\
		   SNDRV_CTL_ELEM_ACCESS_VOLATILE),		\
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,			\
	.name = xname,						\
	.info = snd_soc_info_enum_double,			\
	.get = snd_soc_get_enum_double,				\
	.private_value = (unsigned long)&xenum			\
}

#define SM1_EARC_APB_VALUE(xid, xoff, xsize, xchng)		\
	((unsigned long)&(struct apb_byte_control)		\
	 {.devid = xid, .offset = xoff, .size = xsize, .chng = xchng})

#define SM1_EARC_APB_BYTES(xname, xid, xoff, xsize, xchng)	\
{								\
	.access	= SNDRV_CTL_ELEM_ACCESS_READWRITE,		\
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,			\
	.name = xname,						\
	.info = sm1_earcrx_apb_bytes_info,			\
	.get = sm1_earcrx_apb_bytes_get,			\
	.put = sm1_earcrx_apb_bytes_put,			\
	.private_value =					\
		SM1_EARC_APB_VALUE(xid, xoff, xsize, xchng),	\
}

#define SM1_EARC_APB_U8(xname, xid, xoff, xsize, xchng)		\
	{							\
	.access	= SNDRV_CTL_ELEM_ACCESS_READWRITE,		\
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,			\
	.name = xname,						\
	.info = sm1_earcrx_apb_u8_info,				\
	.get = sm1_earcrx_apb_u8_get,				\
	.put = sm1_earcrx_apb_u8_put,				\
	.private_value =					\
		SM1_EARC_APB_VALUE(xid, xoff, xsize, xchng),	\
}

#define SM1_EARC_APB_U8_RO_V(xname, xid, xoff, xsize)		\
	{							\
	.access	= (SNDRV_CTL_ELEM_ACCESS_READ |			\
		   SNDRV_CTL_ELEM_ACCESS_VOLATILE),		\
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,			\
	.name = xname,						\
	.info = sm1_earcrx_apb_u8_info,				\
	.get = sm1_earcrx_apb_u8_get,				\
	.private_value =					\
		SM1_EARC_APB_VALUE(xid, xoff, xsize, 0),	\
}

static const struct snd_kcontrol_new sm1_earcrx_controls[] = {
	SOC_ENUM_RO_V("STATE", sm1_earcrx_cmdc_state_enum),
	SOC_ENUM(SNDRV_CTL_NAME_IEC958("", CAPTURE, NONE) "SRC",
		 sm1_earcrx_chsts_src_enum),
        MESON_SPDIFIN_IEC958_MASK(""),
	MESON_SPDIFIN_IEC958_STATUS("", SPDIFIN_CTRL0,
				    SPDIFIN_STATUS_SEL,
				    SPDIFIN_STAT1),
	SM1_EARC_APB_BYTES("CDS", EARC_CDS_ID, 0, EARC_CDS_MAX_SIZE, EARC_CAP_CHNG),
	SM1_EARC_APB_U8("LATENCY", EARC_STAT_ID, EARC_RX_LATENCY, 1, EARC_STAT_CHNG),
	SM1_EARC_APB_U8_RO_V("LATENCY REQ", EARC_STAT_ID, EARC_RX_LATENCY_REQ, 1),
	SOC_SINGLE_BOOL_EXT("COMMA ENABLE", 0,
			    sm1_earcrx_get_comma_en, sm1_earcrx_set_comma_en),
};

static const struct snd_soc_dapm_widget sm1_earcrx_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_OUT("OUT", NULL, 0, SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route sm1_earcrx_dapm_routes[] = {
	{ "OUT", NULL, "Capture" },
};

static const struct snd_soc_dai_ops sm1_earcrx_dai_ops = {
	.probe		= sm1_earcrx_dai_probe,
	.remove		= sm1_earcrx_dai_remove,
	.startup	= sm1_earcrx_dai_startup,
	.shutdown	= sm1_earcrx_dai_shutdown,
	.hw_free	= sm1_earcrx_dai_hw_free,
	.prepare	= sm1_earcrx_dai_prepare,
};

static struct snd_soc_dai_driver sm1_earcrx_dai_driver[] = {
	{
		.name = "eARC Rx",
		.capture = {
			.stream_name	= "Capture",
			.channels_min	= 2,
			.channels_max	= 2,
			.rate_min	= 32000,
			.rate_max	= 768000,
			.rates		= SNDRV_PCM_RATE_KNOT,
			.formats	= SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_LE,
		},
		.ops = &sm1_earcrx_dai_ops,
	},
};

static const struct snd_soc_component_driver sm1_earcrx_component_driver = {
	.probe			= sm1_earcrx_component_probe,
	.remove			= sm1_earcrx_component_remove,
	.controls		= sm1_earcrx_controls,
	.num_controls		= ARRAY_SIZE(sm1_earcrx_controls),
	.dapm_widgets		= sm1_earcrx_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(sm1_earcrx_dapm_widgets),
	.dapm_routes		= sm1_earcrx_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(sm1_earcrx_dapm_routes),
};

static const struct regmap_config sm1_earcrx_regmap_cfg = {
	.reg_bits	= 32,
	.val_bits	= 32,
	.reg_stride	= 4,
};

static const struct reg_sequence sm1_pll_init_regs[] = {
	{ .reg = TOP_PLL_CTRL3,		.def = 0x00242000 },
	{ .reg = TOP_PLL_CTRL0,		.def = 0x10800400 },
	{ .reg = TOP_PLL_CTRL1,		.def = 0x001410a8 },
	{ .reg = TOP_PLL_CTRL2,		.def = 0x92772792 },
	{ .reg = TOP_PLL_CTRL3,		.def = 0x20046000 },
};

static const struct sm1_earcrx_cfg sm1_cfg = {
	.modes = arc_mode_rates,
	.mode_num = ARRAY_SIZE(arc_mode_rates),
	.pll_init = sm1_pll_init_regs,
	.pll_init_num = ARRAY_SIZE(sm1_pll_init_regs),
};

static const struct of_device_id sm1_earcrx_of_match[] = {
	{
		.compatible = "amlogic,sm1-earcrx",
		.data = &sm1_cfg,
	}, {}
};
MODULE_DEVICE_TABLE(of, sm1_earcrx_of_match);

static int sm1_earcrx_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sm1_earcrx *ed;
	void __iomem *regs;

	ed = devm_kzalloc(dev, sizeof(*ed), GFP_KERNEL);
	if (!ed)
		return -ENOMEM;
	platform_set_drvdata(pdev, ed);

	ed->conf = of_device_get_match_data(dev);
	if (!ed->conf) {
		dev_err(dev, "failed to match device\n");
		return -ENODEV;
	}

	regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	ed->map = devm_regmap_init_mmio(dev, regs, &sm1_earcrx_regmap_cfg);
	if (IS_ERR(ed->map)) {
		dev_err(dev, "failed to init regmap: %ld\n",
			PTR_ERR(ed->map));
		return PTR_ERR(ed->map);
	}

	ed->hpd_irq = of_irq_get_byname(dev->of_node, "hpd");
	if (ed->hpd_irq <= 0) {
		dev_err(dev, "failed to get irq: %d\n", ed->hpd_irq);
		return ed->hpd_irq;
	}

	ed->pclk = devm_clk_get(dev, "pclk");
	if (IS_ERR(ed->pclk))
		return dev_err_probe(dev, PTR_ERR(ed->pclk),
				     "failed to get pclk\n");

	ed->cmdc_clk = devm_clk_get(dev, "cmdc");
	if (IS_ERR(ed->cmdc_clk))
		return dev_err_probe(dev, PTR_ERR(ed->cmdc_clk),
				     "failed to get cmdc clock\n");

	ed->dmac_clk = devm_clk_get(dev, "dmac");
	if (IS_ERR(ed->dmac_clk))
		return dev_err_probe(dev, PTR_ERR(ed->dmac_clk),
				     "failed to get dmac clock\n");

	ed->earc_irq = of_irq_get_byname(dev->of_node, "earc");
	if (ed->earc_irq <= 0) {
		dev_err(dev, "failed to get irq: %d\n", ed->earc_irq);
		return ed->earc_irq;
	}

	ed->earc_pll = devm_iio_channel_get(dev, "earc-pll");
	if (IS_ERR(ed->earc_pll))
		return dev_err_probe(dev, PTR_ERR(ed->earc_pll),
				     "failed to get earc pll channel\n");

	mutex_init(&ed->lock);

	return devm_snd_soc_register_component(dev,
					       &sm1_earcrx_component_driver,
					       sm1_earcrx_dai_driver, 1);
}


static struct platform_driver sm1_earcrx_pdrv = {
	.probe = sm1_earcrx_probe,
	.driver = {
		.name = "sm1-earcrx",
		.of_match_table = sm1_earcrx_of_match,
	},
};
module_platform_driver(sm1_earcrx_pdrv);

MODULE_DESCRIPTION("Amlogic SM1 eARC Rx driver");
MODULE_AUTHOR("Jerome Brunet <jbrunet@baylibre.com>");
MODULE_LICENSE("GPL");
