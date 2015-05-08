/*
 * Copyright (C) 2013 Marvell Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __MV_USB2_H
#define __MV_USB2_H

#define MV_USB2_PHY_INDEX	0
#define MV_USB2_OTG_PHY_INDEX	1

#define PHY_28LP	0x2800
#define PHY_40LP	0x4000
#define PHY_55LP	0x5500

struct mv_usb2_phydata {
	u32 phy_type;
	u32 phy_rev;
};

struct mv_usb2_phy {
	struct usb_phy		phy;
	struct platform_device	*pdev;
	void __iomem		*base;
	struct clk		*clk;
	struct mv_usb2_phydata  drv_data;
};

struct pxa988_usb_phy {
	u16 utmi_id;		/* 0x00 */
	u16 pad0;
	u16 utmi_pll_reg0;	/* 0x04 */
#define PLLVDD18(x)		((x & 0x3) << 14)
#define REFDIV(x)		((x & 0x1f) << 9)
#define FBDIV(x)		(x & 0x1ff)
	u16 pad1;
	u16 utmi_pll_reg1;	/* 0x08 */
#define PLL_READY		(0x1 << 15)
#define PLL_CONTROL_BY_PIN	(0x1 << 14)
#define PU_PLL			(0x1 << 13)
#define PLL_LOCK_BYPASS		(0x1 << 12)
#define DLL_RESET_BLK		(0x1 << 11)
#define ICP(x)			((x & 0x7) << 8)
#define KVCO_EXT		(0x1 << 7)
#define KVCO(x)			((x & 0x7) << 4)
#define CLK_BLK_EN		(0x1 << 3)
#define VCOCAL_START		(0x1 << 2)
#define PLLCAL12(x)		(x & 0x3)
	u16 pad2;
	u32 rsvd0;		/* 0x0c */
	u16 utmi_tx_reg0;	/* 0x10 */
#define TXDATA_BLK_EN		(0x1 << 14)
#define RCAL_START		(0x1 << 13)
#define EXT_HS_RCAL_EN		(0x1 << 12)
#define EXT_FS_RCAL_EN		(0x1 << 11)
#define IMPCAL_VTH(x)		((x & 0x7) << 8)
#define EXT_HS_RCAL(x)		((x & 0xf) << 4)
#define EXT_FS_RCAL(x)		(x & 0xf)
	u16 pad3;
	u16 utmi_tx_reg1;	/* 0x14 */
#define TXVDD15(x)		((x & 0x3) << 10)
#define TXVDD12(x)		((x & 0x3) << 8)
#define LOWVDD_EN		(0x1 << 7)
#define AMP(x)			((x & 0x7) << 4)
#define CK60_PHSEL(x)		(x & 0xf)
	u16 pad4;
	u16 utmi_tx_reg2;	/* 0x18 */
#define DRV_SLEWRATE(x)		((x & 0x3) << 10)
#define IMP_CAL_DLY(x)		((x & 0x3) << 8)
#define FSDRV_EN(x)		((x & 0xf) << 4)
#define HSDEV_EN(x)		(x & 0xf)
	u16 pad5;
	u32 rsvd1;		/* 0x1c */
	u16 utmi_rx_reg0;	/* 0x20 */
#define PHASE_FREEZE_DLY	(0x1 << 15)
#define USQ_LENGTH		(0x1 << 14)
#define ACQ_LENGTH(x)		((x & 0x3) << 12)
#define SQ_LENGTH(x)		((x & 0x3) << 10)
#define DISCON_THRESH(x)	((x & 0x3) << 8)
#define SQ_THRESH(x)		((x & 0xf) << 4)
#define LPF_COEF(x)		((x & 0x3) << 2)
#define INTPI(x)		(x & 0x3)
	u16 pad6;
	u16 utmi_rx_reg1;	/* 0x24 */
#define EARLY_VOS_ON_EN		(0x1 << 13)
#define RXDATA_BLOCK_EN		(0x1 << 12)
#define EDGE_DET_EN		(0x1 << 11)
#define CAP_SEL(x)		((x & 0x7) << 8)
#define RXDATA_BLOCK_LENGTH(x)	((x & 0x3) << 6)
#define EDGE_DET_SEL(x)		((x & 0x3) << 4)
#define CDR_COEF_SEL		(0x1 << 3)
#define CDR_FASTLOCK_EN		(0x1 << 2)
#define S2TO3_DLY_SEL(x)	(x & 0x3)
	u16 pad7;
	u16 utmi_rx_reg2;	/* 0x28 */
#define USQ_FILTER		(0x1 << 8)
#define SQ_CM_SEL		(0x1 << 7)
#define SAMPLER_CTRL		(0x1 << 6)
#define SQ_BUFFER_EN		(0x1 << 5)
#define SQ_ALWAYS_ON		(0x1 << 4)
#define RXVDD18(x)		((x & 0x3) << 2)
#define RXVDD12(x)		(x & 0x3)
	u16 pad8;
	u32 rsvd2;		/* 0x2c */
	u16 utmi_ana_reg0;	/* 0x30 */
#define BG_VSEL(x)		((x & 0x3) << 8)
#define DIG_SEL(x)		((x & 0x3) << 6)
#define TOPVDD18(x)		((x & 0x3) << 4)
#define VDD_USB2_DIG_TOP_SEL	(0x1 << 3)
#define IPTAT_SEL(x)		(x & 0x7)
	u16 pad9;
	u16 utmi_ana_reg1;	/* 0x34 */
#define PU_ANA			(0x1 << 14)
#define ANA_CONTROL_BY_PIN	(0x1 << 13)
#define SEL_LPFR		(0x1 << 12)
#define V2I_EXT			(0x1 << 11)
#define V2I(x)			((x & 0x7) << 8)
#define R_ROTATE_SEL		(0x1 << 7)
#define STRESS_TEST_MODE	(0x1 << 6)
#define TESTMON_ANA(x)		(x & 0x3f)
	u16 pad10;
	u32 rsvd3;		/* 0x38 */
	u16 utmi_dig_reg0;	/* 0x3c */
#define FIFO_UF			(0x1 << 15)
#define FIFO_OV			(0x1 << 14)
#define FS_EOP_MODE		(0x1 << 13)
#define HOST_DISCON_SEL1	(0x1 << 12)
#define HOST_DISCON_SEL0	(0x1 << 11)
#define FORCE_END_EN		(0x1 << 10)
#define EARLY_TX_EN		(0x1 << 9)
#define SYNCDET_WINDOW_EN	(0x1 << 8)
#define CLK_SUSPEND_EN		(0x1 << 7)
#define HS_DRIBBLE_EN		(0x1 << 6)
#define SYNC_NUM(x)		((x & 0x3) << 4)
#define FIFO_FILL_NUM(x)	(x & 0xf)
	u16 pad11;
	u16 utmi_dig_reg1;	/* 0x40 */
#define FS_RX_ERROR_MODE2	(0x1 << 15)
#define FS_RX_ERROR_MODE1	(0x1 << 14)
#define FS_RX_ERROR_MODE	(0x1 << 13)
#define CLK_OUT_SEL		(0x1 << 12)
#define EXT_TX_CLK_SEL		(0x1 << 11)
#define ARC_DPDM_MODE		(0x1 << 10)
#define DP_PULLDOWN		(0x1 << 9)
#define DM_PULLDOWN		(0x1 << 8)
#define SYNC_IGNORE_SQ		(0x1 << 7)
#define SQ_RST_RX		(0x1 << 6)
#define MON_SEL(x)		(x & 0x3f)
	u16 pad12;
	u16 utmi_dig_reg2;	/* 0x44 */
#define PAD_STRENGTH(x)		((x & 0x1f) << 8)
#define LONG_EOP		(0x1 << 5)
#define NOVBUS_DPDM00		(0x1 << 4)
#define ALIGN_FS_OUTEN		(0x1 << 2)
#define HS_HDL_SYNC		(0x1 << 1)
#define FS_HDL_OPMD		(0x1 << 0)
	u16 pad13;
	u32 rsvd4;		/* 0x48 */
	u16 utmi_test_reg0;	/* 0x4c */
	u16 pad14;
	u16 utmi_test_reg1;	/* 0x50 */
	u16 pad15;
	u32 rsvd5;		/* 0x54 */
	u16 utmi_charger_reg0;	/* 0x58 */
#define ENABLE_SWITCH		(0x1 << 3)
#define PU_CHRG_DTC		(0x1 << 2)
#define TESTMON_CHRGDTC(x)	(x & 0x3)
	u16 pad16;
	u16 utmi_otg_reg;	/* 0x5c */
	u16 pad17;
	u16 utmi_phy_mon0;	/* 0x60 */
	u16 pad18;
	u16 utmi_reserve_reg0;	/* 0x64 */
	u16 pad19;
};

#endif
