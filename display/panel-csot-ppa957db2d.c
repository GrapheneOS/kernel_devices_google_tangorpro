// SPDX-License-Identifier: GPL-2.0-only
/*
 * MIPI-DSI based CSOT PPA957DB2-D LCD panel driver.
 *
 * Copyright (c) 2022 Google Inc.
 *
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/kthread.h>
#include <uapi/linux/sched/types.h>
#include <video/mipi_display.h>

#include "samsung/panel/panel-samsung-drv.h"

#define PPA957DB2D_WRCTRLD_DD_BIT	0x08
#define PPA957DB2D_WRCTRLD_BL_BIT	0x04
#define PPA957DB2D_WRCTRLD_BCTRL_BIT	0x20
#define PPA957DB2D_PANEL_ID_REG		0x00
#define PPA957DB2D_PANEL_ID_LEN		37

#if PPA957DB2D_PANEL_ID_LEN >= PANEL_ID_MAX
	#error PANEL_ID_MAX should be greater than PPA957DB2D_PANEL_ID_LEN
#endif

static const u32 ppa957db2d_panel_rev[] = {
	PANEL_REV_PROTO1,
	PANEL_REV_PROTO2,
	PANEL_REV_EVT1,
	PANEL_REV_EVT1_1,
	PANEL_REV_EVT2,
	PANEL_REV_DVT1,
	PANEL_REV_PVT,
};

static const struct exynos_dsi_cmd ppa957db2d_init_cmds[] = {
	/* CMD2, Page3 */
	EXYNOS_DSI_CMD_SEQ(0xFF, 0x23),
	EXYNOS_DSI_CMD_SEQ(0xFB, 0x01),
	/* 12 bits PWM */
	EXYNOS_DSI_CMD_SEQ(0x00, 0x80),
	/* PWM freq 3kHz */
	EXYNOS_DSI_CMD_SEQ(0x08, 0x04),

	EXYNOS_DSI_CMD_SEQ(0xFF, 0x10),
	EXYNOS_DSI_CMD_SEQ(0xB9, 0x05),
	EXYNOS_DSI_CMD_SEQ(0xFF, 0x20),
	EXYNOS_DSI_CMD_SEQ(0xFB, 0x01),
	EXYNOS_DSI_CMD_SEQ(0x18, 0x40),
	EXYNOS_DSI_CMD_SEQ(0xFF, 0x10),
	EXYNOS_DSI_CMD_SEQ(0xB9, 0x02),
	EXYNOS_DSI_CMD_SEQ(0xFF, 0xF0),
	EXYNOS_DSI_CMD_SEQ(0xFB, 0x01),
	EXYNOS_DSI_CMD_SEQ(0x3A, 0x08),

	/* CMD2, Page7 */
	EXYNOS_DSI_CMD_SEQ(0xFF, 0x27),
	EXYNOS_DSI_CMD_SEQ(0xFB, 0x01),

	/* Error flag detection */
	EXYNOS_DSI_CMD_SEQ(0xD0, 0x31),
	EXYNOS_DSI_CMD_SEQ(0xD1, 0x84),
	EXYNOS_DSI_CMD_SEQ(0xD2, 0x30),
	EXYNOS_DSI_CMD_SEQ(0xDE, 0x03),
	EXYNOS_DSI_CMD_SEQ(0xDF, 0x02),

	/* CMD1 */
	EXYNOS_DSI_CMD_SEQ(0xFF, 0x10),
	EXYNOS_DSI_CMD_SEQ(0xFB, 0x01),
	/* Write Primary & Secondary */
	EXYNOS_DSI_CMD_SEQ(0xB9, 0x02),
	EXYNOS_DSI_CMD_SEQ(0x51, 0x0F, 0xFF),
	EXYNOS_DSI_CMD_SEQ(0x53, 0x24),
	/* CABC initial OFF */
	EXYNOS_DSI_CMD_SEQ(0x55, 0x00),
	/* BBh (MIPI via/bypass RAM) */
	EXYNOS_DSI_CMD_SEQ(0xBB, 0x13),
	/* VBP + VFP = 200 + 26 = 226 */
	EXYNOS_DSI_CMD_SEQ(0x3B, 0x03, 0xC8, 0x1A, 0x04, 0x04),

	/* b/201704777: Flip 180 degrees */
	EXYNOS_DSI_CMD_SEQ(0x36, 0x03),

	EXYNOS_DSI_CMD_SEQ_DELAY(120, MIPI_DCS_EXIT_SLEEP_MODE),
	EXYNOS_DSI_CMD_SEQ(MIPI_DCS_SET_DISPLAY_ON)
};
static DEFINE_EXYNOS_CMD_SET(ppa957db2d_init);

static const struct exynos_dsi_cmd ppa957db2d_off_cmds[] = {
	EXYNOS_DSI_CMD_SEQ(0xFF, 0x10),
	EXYNOS_DSI_CMD_SEQ(0xFB, 0x01),
	EXYNOS_DSI_CMD_SEQ_DELAY(20, MIPI_DCS_SET_DISPLAY_OFF),
	EXYNOS_DSI_CMD_SEQ_DELAY(100, MIPI_DCS_ENTER_SLEEP_MODE),
};
static DEFINE_EXYNOS_CMD_SET(ppa957db2d_off);

static void ppa957db2d_reset(struct exynos_panel *ctx)
{
	dev_dbg(ctx->dev, "%s +\n", __func__);

	if (ctx->panel_state == PANEL_STATE_BLANK) {
		gpiod_set_value(ctx->reset_gpio, 0);
		usleep_range(1000, 1100);
	}
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(1000, 1100);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(1000, 1100);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(10000, 10100);

	dev_dbg(ctx->dev, "%s -\n", __func__);
}

static int ppa957db2d_prepare(struct drm_panel *panel)
{
	struct exynos_panel *ctx =
		container_of(panel, struct exynos_panel, panel);

	dev_dbg(ctx->dev, "%s +\n", __func__);

	exynos_panel_set_power(ctx, true);
	usleep_range(18500, 18600);
	ppa957db2d_reset(ctx);

	dev_dbg(ctx->dev, "%s -\n", __func__);

	return 0;
}

static void ts110f5mlg0_set_cabc_mode(struct exynos_panel *ctx,
					enum exynos_cabc_mode cabc_mode)
{
	u8 mode;

	switch (cabc_mode) {
	case CABC_UI_MODE:
		mode = 0x01;
		break;
	case CABC_STILL_MODE:
		mode = 0x02;
		break;
	case CABC_MOVIE_MODE:
		mode = 0x03;
		break;
	default:
		mode = 0x00;
	}
	EXYNOS_DCS_WRITE_SEQ(ctx, 0x55, mode);

	dev_dbg(ctx->dev, "%s CABC state: %u\n", __func__, mode);
}

static int ppa957db2d_enable(struct drm_panel *panel)
{
	struct exynos_panel *ctx;

	ctx = container_of(panel, struct exynos_panel, panel);

	exynos_panel_init(ctx);
	exynos_panel_send_cmd_set(ctx, &ppa957db2d_init_cmd_set);
	ctx->enabled = true;

	return 0;
}

static int ppa957db2d_read_id(struct exynos_panel *ctx)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int read_bytes = 0;
	u8 i;

	if (ctx->panel_rev < PANEL_REV_EVT2) {
		/* hardcode 0 as reading id is not supported in this panel_rev */
		dev_info(ctx->dev, "read_id is not supported in panel_rev: 0x%x\n", ctx->panel_rev);
		strlcpy(ctx->panel_id, "0", PANEL_ID_MAX);
		return 0;
	}

	/* Change to CMD2, Page2 */
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xFF, 0x22);
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xFB, 0x01);

	/* Serial number is stored in different registers, use loop to read it. */
	for (i = 0; i < PPA957DB2D_PANEL_ID_LEN; ++i) {
		read_bytes = mipi_dsi_dcs_read(dsi, PPA957DB2D_PANEL_ID_REG + i,
				ctx->panel_id + i, 1);
		if (read_bytes != 1)
			break;
	}

	/* Switch back to CMD1 */
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xFF, 0x10);
	EXYNOS_DCS_WRITE_SEQ(ctx, 0xFB, 0x01);

	if (read_bytes != 1) {
		dev_warn(ctx->dev, "Unable to read panel id (%d)\n", read_bytes);
		strlcpy(ctx->panel_id, "0", PANEL_ID_MAX);
		return -EIO;
	}

	ctx->panel_id[PPA957DB2D_PANEL_ID_LEN] = '\0';

	return 0;
}

static void ppa957db2d_update_wrctrld(struct exynos_panel *ctx)
{
	u8 val = PPA957DB2D_WRCTRLD_BCTRL_BIT |
			PPA957DB2D_WRCTRLD_BL_BIT;

	if (ctx->dimming_on)
		val |= PPA957DB2D_WRCTRLD_DD_BIT;

	dev_dbg(ctx->dev,
		"%s(wrctrld:0x%x, dimming: %s)\n",
		__func__, val, ctx->dimming_on ? "on" : "off");

	EXYNOS_DCS_WRITE_SEQ(ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY, val);
}

static void ppa957db2d_set_dimming_on(struct exynos_panel *ctx,
					bool dimming_on)
{
	ctx->dimming_on = dimming_on;
	ppa957db2d_update_wrctrld(ctx);
}

static void ppa957db2d_get_panel_rev(struct exynos_panel *ctx, u32 id)
{
	/* extract command 0xDB */
	u8 build_code = (id & 0xFF00) >> 8;
	u8 rev = build_code >> 4;

	if (rev >= ARRAY_SIZE(ppa957db2d_panel_rev)) {
		ctx->panel_rev = PANEL_REV_LATEST;
		dev_warn(ctx->dev,
			"unknown rev from panel (0x%x), default to latest\n",
			rev);
	} else {
		ctx->panel_rev = ppa957db2d_panel_rev[rev];
		dev_info(ctx->dev, "panel_rev: 0x%x\n", ctx->panel_rev);
	}
}

static const struct exynos_panel_mode ppa957db2d_modes[] = {
	{
		/* 1600x2560 @ 60 */
		.mode = {
			.clock = 309246,
			.hdisplay = 1600,
			.hsync_start = 1600 + 92, // add hfp
			.hsync_end = 1600 + 92 + 66, // add hsa
			.htotal = 1600 + 92 + 66 + 92, // add hbp
			.vdisplay = 2560,
			.vsync_start = 2560 + 26, // add vfp
			.vsync_end = 2560 + 26 + 4, // add vsa
			.vtotal = 2560 + 26 + 4 + 196, // add vbp
			.flags = 0,
			.width_mm = 147,
			.height_mm = 236,
		},
		.exynos_mode = {
			.mode_flags = MIPI_DSI_MODE_VIDEO,
			.bpc = 8,
			.dsc = {
				.enabled = false,
			},
		},
	},
};

static const struct drm_panel_funcs ppa957db2d_drm_funcs = {
	.disable = exynos_panel_disable,
	.unprepare = exynos_panel_unprepare,
	.prepare = ppa957db2d_prepare,
	.enable = ppa957db2d_enable,
	.get_modes = exynos_panel_get_modes,
};

static const struct exynos_panel_funcs ppa957db2d_exynos_funcs = {
	.read_id = ppa957db2d_read_id,
	.panel_reset = ppa957db2d_reset,
	.set_dimming_on = ppa957db2d_set_dimming_on,
	.set_brightness = exynos_panel_set_brightness,
	.set_cabc_mode = ts110f5mlg0_set_cabc_mode,
	.get_panel_rev = ppa957db2d_get_panel_rev,
};

const struct brightness_capability ppa957db2d_brightness_capability = {
	.normal = {
		.nits = {
			.min = 2,
			.max = 500,
		},
		.level = {
			.min = 16,
			.max = 4095,
		},
		.percentage = {
			.min = 0,
			.max = 100,
		},
	},
};

static const struct exynos_panel_desc csot_ppa957db2d = {
	.data_lane_cnt = 4,
	.max_brightness = 4095,
	.min_brightness = 16,
	.lower_min_brightness = 4,
	.dft_brightness = 1146,
	/* supported HDR format bitmask : 1(DOLBY_VISION), 2(HDR10), 3(HLG) */
	.hdr_formats = BIT(2) | BIT(3),
	.max_luminance = 5000000,
	.max_avg_luminance = 1200000,
	.min_luminance = 5,
	.brt_capability = &ppa957db2d_brightness_capability,
	.modes = ppa957db2d_modes,
	.num_modes = 1,
	.off_cmd_set = &ppa957db2d_off_cmd_set,
	.panel_func = &ppa957db2d_drm_funcs,
	.exynos_panel_func = &ppa957db2d_exynos_funcs,
};

static const struct of_device_id exynos_panel_of_match[] = {
	{ .compatible = "csot,ppa957db2d", .data = &csot_ppa957db2d },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos_panel_of_match);

static struct mipi_dsi_driver exynos_panel_driver = {
	.probe = exynos_panel_probe,
	.remove = exynos_panel_remove,
	.driver = {
		.name = "panel-csot-ppa957db2d",
		.of_match_table = exynos_panel_of_match,
	},
};
module_mipi_dsi_driver(exynos_panel_driver);

MODULE_AUTHOR("Ken Huang <kenbshuang@google.com>");
MODULE_DESCRIPTION("MIPI-DSI based CSOT ppa957db2d panel driver");
MODULE_LICENSE("GPL");
