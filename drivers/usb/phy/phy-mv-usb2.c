/*
 * Copyright (C) 2013 Marvell Inc.
 *
 * Author:
 *	Chao Xie <xiechao.mail@gmail.com>
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

#include <linux/resource.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/platform_data/mv_usb.h>
#include <linux/usb/phy.h>
#include <linux/usb/mv_usb2_phy.h>
#include <dt-bindings/usb/mv_usb_phy.h>


/* phy regs */
/* for pxa910 and mmp2, there is no revision register */
#define MV_USB2_UTMI_REVISION		0x0
#define MV_USB2_UTMI_CTRL		0x4
#define MV_USB2_UTMI_PLL		0x8
#define MV_USB2_UTMI_TX			0xc
#define MV_USB2_UTMI_RX			0x10
#define MV_USB2_UTMI_IVREF		0x14
#define MV_USB2_UTMI_T0			0x18
#define MV_USB2_UTMI_T1			0x1c
#define MV_USB2_UTMI_T2			0x20
#define MV_USB2_UTMI_T3			0x24
#define MV_USB2_UTMI_T4			0x28
#define MV_USB2_UTMI_T5			0x2c
#define MV_USB2_UTMI_RESERVE		0x30
#define MV_USB2_UTMI_USB_INT		0x34
#define MV_USB2_UTMI_DBG_CTL		0x38
#define MV_USB2_UTMI_OTG_ADDON		0x3c

/* For UTMICTRL Register */
#define MV_USB2_UTMI_CTRL_USB_CLK_EN			(1 << 31)
/* pxa168 */
#define MV_USB2_UTMI_CTRL_SUSPEND_SET1			(1 << 30)
#define MV_USB2_UTMI_CTRL_SUSPEND_SET2			(1 << 29)
#define MV_USB2_UTMI_CTRL_RXBUF_PDWN			(1 << 24)
#define MV_USB2_UTMI_CTRL_TXBUF_PDWN			(1 << 11)

#define MV_USB2_UTMI_CTRL_INPKT_DELAY_SHIFT		30
#define MV_USB2_UTMI_CTRL_INPKT_DELAY_SOF_SHIFT		28
#define MV_USB2_UTMI_CTRL_PU_REF_SHIFT			20
#define MV_USB2_UTMI_CTRL_ARC_PULLDN_SHIFT		12
#define MV_USB2_UTMI_CTRL_PLL_PWR_UP_SHIFT		1
#define MV_USB2_UTMI_CTRL_PWR_UP_SHIFT			0

/* For UTMI_PLL Register */
#define MV_USB2_UTMI_PLL_PLLCALI12_SHIFT		29
#define MV_USB2_UTMI_PLL_PLLCALI12_MASK			(0x3 << 29)

#define MV_USB2_UTMI_PLL_PLLVDD18_SHIFT			27
#define MV_USB2_UTMI_PLL_PLLVDD18_MASK			(0x3 << 27)

#define MV_USB2_UTMI_PLL_PLLVDD12_SHIFT			25
#define MV_USB2_UTMI_PLL_PLLVDD12_MASK			(0x3 << 25)

#define MV_USB2_UTMI_PLL_PLL_READY			(0x1 << 23)
#define MV_USB2_UTMI_PLL_KVCO_EXT			(0x1 << 22)
#define MV_USB2_UTMI_PLL_VCOCAL_START			(0x1 << 21)

#define MV_USB2_UTMI_PLL_KVCO_SHIFT			15
#define MV_USB2_UTMI_PLL_KVCO_MASK			(0x7 << 15)

#define MV_USB2_UTMI_PLL_ICP_SHIFT			12
#define MV_USB2_UTMI_PLL_ICP_MASK			(0x7 << 12)

#define MV_USB2_UTMI_PLL_FBDIV_SHIFT			4
#define MV_USB2_UTMI_PLL_FBDIV_MASK			(0xFF << 4)

#define MV_USB2_UTMI_PLL_REFDIV_SHIFT			0
#define MV_USB2_UTMI_PLL_REFDIV_MASK			(0xF << 0)

/* For UTMI_TX Register */
#define MV_USB2_UTMI_TX_REG_EXT_FS_RCAL_SHIFT		27
#define MV_USB2_UTMI_TX_REG_EXT_FS_RCAL_MASK		(0xf << 27)

#define MV_USB2_UTMI_TX_REG_EXT_FS_RCAL_EN_SHIFT	26
#define MV_USB2_UTMI_TX_REG_EXT_FS_RCAL_EN_MASK		(0x1 << 26)

#define MV_USB2_UTMI_TX_TXVDD12_SHIFT			22
#define MV_USB2_UTMI_TX_TXVDD12_MASK			(0x3 << 22)

#define MV_USB2_UTMI_TX_CK60_PHSEL_SHIFT		17
#define MV_USB2_UTMI_TX_CK60_PHSEL_MASK			(0xf << 17)

#define MV_USB2_UTMI_TX_IMPCAL_VTH_SHIFT		14
#define MV_USB2_UTMI_TX_IMPCAL_VTH_MASK			(0x7 << 14)

#define MV_USB2_UTMI_TX_REG_RCAL_START			(0x1 << 12)

#define MV_USB2_UTMI_TX_LOW_VDD_EN_SHIFT		11

#define MV_USB2_UTMI_TX_AMP_SHIFT			0
#define MV_USB2_UTMI_TX_AMP_MASK			(0x7 << 0)

/* For UTMI_RX Register */
#define MV_USB2_UTMI_RX_REG_SQ_LENGTH_SHIFT		15
#define MV_USB2_UTMI_RX_REG_SQ_LENGTH_MASK		(0x3 << 15)

#define MV_USB2_UTMI_RX_SQ_THRESH_SHIFT			4
#define MV_USB2_UTMI_RX_SQ_THRESH_MASK			(0xf << 4)

/* For UTMI_OTG_ADDON Register. Only for pxa168 */
#define MV_USB2_UTMI_OTG_ADDON_OTG_ON			(1 << 0)


/* For pxa988 the register mapping are changed*/
#define MV_USB2_PHY_PLL0		0x4
#define MV_USB2_PHY_PLL1		0x8
#define MV_USB2_PHY_TX0			0x10
#define MV_USB2_PHY_TX1			0x14
#define MV_USB2_PHY_TX2			0x18
#define MV_USB2_PHY_RX0			0x20
#define MV_USB2_PHY_RX1			0x24
#define MV_USB2_PHY_RX2			0x28
#define MV_USB2_PHY_ANA0		0x30
#define MV_USB2_PHY_ANA1		0x34
#define MV_USB2_PHY_DIG0		0x3c
#define MV_USB2_PHY_DIG1		0x40
#define MV_USB2_PHY_DIG2		0x44
#define MV_USB2_PHY_T0			0x4c
#define MV_USB2_PHY_T1			0x50
#define MV_USB2_PHY_CHARGE0		0x58
#define MV_USB2_PHY_OTG			0x5C
#define MV_USB2_PHY_PHY_MON		0x60
#define MV_USB2_PHY_CTRL		0x104

/* default values are got from spec */
#define MV_USB2_PHY_PLL0_DEFAULT	0x5A78
#define MV_USB2_PHY_PLL1_DEFAULT	0x0231
#define MV_USB2_PHY_TX0_DEFAULT		0x0488
#define MV_USB2_PHY_ANA1_DEFAULT	0x1680
#define MV_USB2_PHY_OTG_DEFAULT		0x0
#define MV_USB2_PHY_CTRL_DEFAULT	0x00801000
#define MV_USB2_PHY_CTRL_OTG_DEFAULT	0x0398F000

#define MV_USB2_PHY_PLL0_PLLVDD18(x)			(((x) & 0x3) << 14)
#define MV_USB2_PHY_PLL0_REFDIV(x)			(((x) & 0x1f) << 9)
#define MV_USB2_PHY_PLL0_FBDIV(x)			(((x) & 0x1ff) << 0)

#define MV_USB2_PHY_PLL1_PLL_READY			(0x1 << 15)
#define MV_USB2_PHY_PLL1_PLL_CONTROL_BY_PIN		(0x1 << 14)
#define MV_USB2_PHY_PLL1_PU_PLL				(0x1 << 13)
#define MV_USB2_PHY_PLL1_PLL_LOCK_BYPASS		(0x1 << 12)
#define MV_USB2_PHY_PLL1_DLL_RESET			(0x1 << 11)
#define MV_USB2_PHY_PLL1_ICP(x)				(((x) & 0x7) << 8)
#define MV_USB2_PHY_PLL1_KVCO_EXT			(0x1 << 7)
#define MV_USB2_PHY_PLL1_KVCO(x)			(((x) & 0x7) << 4)
#define MV_USB2_PHY_PLL1_CLK_BLK_EN			(0x1 << 3)
#define MV_USB2_PHY_PLL1_VCOCAL_START			(0x1 << 2)
#define MV_USB2_PHY_PLL1_PLLCAL12(x)			(((x) & 0x3) << 0)

#define MV_USB2_PHY_TX0_TXDATA_BLK_EN			(0x1 << 14)
#define MV_USB2_PHY_TX0_RCAL_START			(0x1 << 13)
#define MV_USB2_PHY_TX0_EXT_HS_RCAL_EN			(0x1 << 12)
#define MV_USB2_PHY_TX0_EXT_FS_RCAL_EN			(0x1 << 11)
#define MV_USB2_PHY_TX0_IMPCAL_VTH(x)			(((x) & 0x7) << 8)
#define MV_USB2_PHY_TX0_EXT_HS_RCAL(x)			(((x) & 0xf) << 4)
#define MV_USB2_PHY_TX0_EXT_FS_RCAL(x)			(((x) & 0xf) << 0)

#define MV_USB2_PHY_TX1_TXVDD15(x)			(((x) & 0x3) << 10)
#define MV_USB2_PHY_TX1_TXVDD12(x)			(((x) & 0x3) << 8)
#define MV_USB2_PHY_TX1_LOWVDD_EN			(0x1 << 7)
#define MV_USB2_PHY_TX1_AMP(x)				(((x) & 0x7) << 4)
#define MV_USB2_PHY_TX1_CK60_PHSEL(x)			(((x) & 0xf) << 0)

#define MV_USB2_PHY_TX2_DRV_SLEWRATE(x)			(((x) & 0x3) << 10)
#define MV_USB2_PHY_TX2_IMP_CAL_DLY(x)			(((x) & 0x3) << 8)
#define MV_USB2_PHY_TX2_FSDRV_EN(x)			(((x) & 0xf) << 4)
#define MV_USB2_PHY_TX2_HSDEV_EN(x)			(((x) & 0xf) << 0)

#define MV_USB2_PHY_RX0_PHASE_FREEZE_DLY		(0x1 << 15)
#define MV_USB2_PHY_RX0_USQ_LENGTH			(0x1 << 14)
#define MV_USB2_PHY_RX0_ACQ_LENGTH(x)			(((x) & 0x3) << 12)
#define MV_USB2_PHY_RX0_SQ_LENGTH(x)			(((x) & 0x3) << 10)
#define MV_USB2_PHY_RX0_DISCON_THRESH(x)		(((x) & 0x3) << 8)
#define MV_USB2_PHY_RX0_SQ_THRESH(x)			(((x) & 0xf) << 4)
#define MV_USB2_PHY_RX0_LPF_COEF(x)			(((x) & 0x3) << 2)
#define MV_USB2_PHY_RX0_INTPI(x)			(((x) & 0x3) << 0)

#define MV_USB2_PHY_RX1_EARLY_VOS_ON_EN			(0x1 << 13)
#define MV_USB2_PHY_RX1_RXDATA_BLOCK_EN			(0x1 << 12)
#define MV_USB2_PHY_RX1_EDGE_DET_EN			(0x1 << 11)
#define MV_USB2_PHY_RX1_CAP_SEL(x)			(((x) & 0x7) << 8)
#define MV_USB2_PHY_RX1_RXDATA_BLOCK_LENGTH(x)		(((x) & 0x3) << 6)
#define MV_USB2_PHY_RX1_EDGE_DET_SEL(x)			(((x) & 0x3) << 4)
#define MV_USB2_PHY_RX1_CDR_COEF_SEL			(0x1 << 3)
#define MV_USB2_PHY_RX1_CDR_FASTLOCK_EN			(0x1 << 2)
#define MV_USB2_PHY_RX1_S2TO3_DLY_SEL(x)		(((x) & 0x3) << 0)

#define MV_USB2_PHY_RX2_USQ_FILTER			(0x1 << 8)
#define MV_USB2_PHY_RX2_SQ_CM_SEL			(0x1 << 7)
#define MV_USB2_PHY_RX2_SAMPLER_CTRL			(0x1 << 6)
#define MV_USB2_PHY_RX2_SQ_BUFFER_EN			(0x1 << 5)
#define MV_USB2_PHY_RX2_SQ_ALWAYS_ON			(0x1 << 4)
#define MV_USB2_PHY_RX2_RXVDD18(x)			(((x) & 0x3) << 2)
#define MV_USB2_PHY_RX2_RXVDD12(x)			(((x) & 0x3) << 0)

#define MV_USB2_PHY_ANA0_BG_VSEL(x)			(((x) & 0x3) << 8)
#define MV_USB2_PHY_ANA0_DIG_SEL(x)			(((x) & 0x3) << 6)
#define MV_USB2_PHY_ANA0_TOPVDD18(x)			(((x) & 0x3) << 4)
#define MV_USB2_PHY_ANA0_VDD_USB2_DIG_TOP_SEL		(0x1 << 3)
#define MV_USB2_PHY_ANA0_IPTAT_SEL(x)			(((x) & 0x7) << 0)

#define MV_USB2_PHY_ANA1_PU_ANA				(0x1 << 14)
#define MV_USB2_PHY_ANA1_ANA_CONTROL_BY_PIN		(0x1 << 13)
#define MV_USB2_PHY_ANA1_SEL_LPFR			(0x1 << 12)
#define MV_USB2_PHY_ANA1_V2I_EXT			(0x1 << 11)
#define MV_USB2_PHY_ANA1_V2I(x)				(((x) & 0x7) << 8)
#define MV_USB2_PHY_ANA1_R_ROTATE_SEL			(0x1 << 7)
#define MV_USB2_PHY_ANA1_STRESS_TEST_MODE		(0x1 << 6)
#define MV_USB2_PHY_ANA1_TESTMON_ANA(x)			(((x) & 0x3f) << 0)

#define MV_USB2_PHY_DIG0_FIFO_UF			(0x1 << 15)
#define MV_USB2_PHY_DIG0_FIFO_OV			(0x1 << 14)
#define MV_USB2_PHY_DIG0_FS_EOP_MODE			(0x1 << 13)
#define MV_USB2_PHY_DIG0_HOST_DISCON_SEL1		(0x1 << 12)
#define MV_USB2_PHY_DIG0_HOST_DISCON_SEL0		(0x1 << 11)
#define MV_USB2_PHY_DIG0_FORCE_END_EN			(0x1 << 10)
#define MV_USB2_PHY_DIG0_SYNCDET_WINDOW_EN		(0x1 << 8)
#define MV_USB2_PHY_DIG0_CLK_SUSPEND_EN			(0x1 << 7)
#define MV_USB2_PHY_DIG0_HS_DRIBBLE_EN			(0x1 << 6)
#define MV_USB2_PHY_DIG0_SYNC_NUM(x)			(((x) & 0x3) << 4)
#define MV_USB2_PHY_DIG0_FIFO_FILL_NUM(x)		(((x) & 0xf) << 0)

#define MV_USB2_PHY_DIG1_FS_RX_ERROR_MODE2		(0x1 << 15)
#define MV_USB2_PHY_DIG1_FS_RX_ERROR_MODE1		(0x1 << 14)
#define MV_USB2_PHY_DIG1_FS_RX_ERROR_MODE		(0x1 << 13)
#define MV_USB2_PHY_DIG1_CLK_OUT_SEL			(0x1 << 12)
#define MV_USB2_PHY_DIG1_EXT_TX_CLK_SEL			(0x1 << 11)
#define MV_USB2_PHY_DIG1_ARC_DPDM_MODE			(0x1 << 10)
#define MV_USB2_PHY_DIG1_DP_PULLDOWN			(0x1 << 9)
#define MV_USB2_PHY_DIG1_DM_PULLDOWN			(0x1 << 8)
#define MV_USB2_PHY_DIG1_SYNC_IGNORE_SQ			(0x1 << 7)
#define MV_USB2_PHY_DIG1_SQ_RST_RX			(0x1 << 6)
#define MV_USB2_PHY_DIG1_MON_SEL(x)			(((x) & 0x3f) << 0)

#define MV_USB2_PHY_DIG2_PAD_STRENGTH(x)		(((x) & 0x1f) << 8)
#define MV_USB2_PHY_DIG2_LONG_EOP			(0x1 << 5)
#define MV_USB2_PHY_DIG2_NOVBUS_DPDM00			(0x1 << 4)
#define MV_USB2_PHY_DIG2_ALIGN_FS_OUTEN			(0x1 << 2)
#define MV_USB2_PHY_DIG2_HS_HDL_SYNC			(0x1 << 1)
#define MV_USB2_PHY_DIG2_FS_HDL_OPMD			(0x1 << 0)

#define MV_USB2_PHY_CHARGE0_ENABLE_SWITCH		(0x1 << 3)
#define MV_USB2_PHY_CHARGE0_PU_CHRG_DTC			(0x1 << 2)
#define MV_USB2_PHY_CHARGE0_TESTMON_CHRGDTC(x)		(((x) & 0x3) << 0)

#define MV_USB2_PHY_OTG_TESTMODE(x)			(((x) & 0x7) << 0)
#define MV_USB2_PHY_OTG_CONTROL_BY_PIN			(0x1 << 4)
#define MV_USB2_PHY_OTG_PU				(0x1 << 3)

#define MV_USB2_PHY_CTRL_PU_PLL				(0x1 << 1)
#define MV_USB2_PHY_CTRL_PU				(0x1 << 0)

/* USB eden PHY mapping (Eden) */
#define MV_USB2PHY_EDEN_PLL_REG0		0x0
#define MV_USB2PHY_EDEN_PLL_REG1		0x4
#define MV_USB2PHY_EDEN_CAL_REG		0x8
#define MV_USB2PHY_EDEN_TX_REG0		0x0C
#define MV_USB2PHY_EDEN_TX_REG1		0x10
#define MV_USB2PHY_EDEN_RX_REG0		0x14
#define MV_USB2PHY_EDEN_RX_REG1		0x18
#define MV_USB2PHY_EDEN_DIG_REG0		0x1C
#define MV_USB2PHY_EDEN_DIG_REG1		0x20
#define MV_USB2PHY_EDEN_TEST_REG0		0x24
#define MV_USB2PHY_EDEN_TEST_REG1		0x28
#define MV_USB2PHY_EDEN_MOC_REG		0x2C
#define MV_USB2PHY_EDEN_PHY_RESERVE	0x30
#define MV_USB2PHY_EDEN_OTG_REG		0x34
#define MV_USB2PHY_EDEN_CHRG_DET		0x38
#define MV_USB2PHY_EDEN_CTRL_REG0		0xC4
#define MV_USB2PHY_EDEN_CTRL_REG2		0xD4
#define MV_USB2PHY_EDEN_CTRL_REG1		0xDC

/* MV_USB2PHY_EDEN_PLL_REG0 */
#define MV_USB2PHY_EDEN_PLL_READY_MASK		(0x1 << 31)

#define MV_USB2PHY_EDEN_PLL_SELLPFR_SHIFT		28
#define MV_USB2PHY_EDEN_PLL_SELLPFR_MASK		(0x3 << 28)

#define MV_USB2PHY_EDEN_PLL_FBDIV_SHIFT		16
#define MV_USB2PHY_EDEN_PLL_FBDIV_MASK		(0x1ff << 16)

#define MV_USB2PHY_EDEN_PLL_ICP_SHIFT			8
#define MV_USB2PHY_EDEN_PLL_ICP_MASK			(0x7 << 8)

#define MV_USB2PHY_EDEN_PLL_REFDIV_SHIFT		0
#define MV_USB2PHY_EDEN_PLL_REFDIV_MASK		0x7f

/* MV_USB2PHY_EDEN_PLL_REG1 */
#define MV_USB2PHY_EDEN_PLL_PU_BY_REG_SHIFT		1
#define MV_USB2PHY_EDEN_PLL_PU_BY_REG_MASK		(0x1 << 1)

#define MV_USB2PHY_EDEN_PLL_PU_PLL_SHIFT		0
#define MV_USB2PHY_EDEN_PLL_PU_PLL_MASK		(0x1 << 0)

/* MV_USB2PHY_EDEN_CAL_REG */
#define MV_USB2PHY_EDEN_PLL_PLLCAL_DONE_SHIFT		31
#define MV_USB2PHY_EDEN_PLL_PLLCAL_DONE_MASK		(0x1 << 31)

#define MV_USB2PHY_EDEN_PLL_IMPCAL_DONE_SHIFT		23
#define MV_USB2PHY_EDEN_PLL_IMPCAL_DONE_MASK		(0x1 << 23)

#define MV_USB2PHY_EDEN_PLL_KVCO_SHIFT		16
#define MV_USB2PHY_EDEN_PLL_KVCO_MASK			(0x7 << 16)

#define MV_USB2PHY_EDEN_PLL_CAL12_SHIFT		20
#define MV_USB2PHY_EDEN_PLL_CAL12_MASK		(0x3 << 20)

#define MV_USB2PHY_EDEN_IMPCAL_VTH_SHIFT		8
#define MV_USB2PHY_EDEN_IMPCAL_VTH_MASK		(0x7 << 8)

#define MV_USB2PHY_EDEN_PLLCAL_START_SHIFT		22
#define MV_USB2PHY_EDEN_IMPCAL_START_SHIFT		13

/* MV_USB2PHY_EDEN_TX_REG0 */
#define MV_USB2PHY_EDEN_TX_PU_BY_REG_SHIFT		25

#define MV_USB2PHY_EDEN_TX_PU_ANA_SHIFT		24

#define MV_USB2PHY_EDEN_TX_AMP_SHIFT			20
#define MV_USB2PHY_EDEN_TX_AMP_MASK			(0x7 << 20)

/* MV_USB2PHY_EDEN_RX_REG0 */
#define MV_USB2PHY_EDEN_RX_SQ_THRESH_SHIFT		0
#define MV_USB2PHY_EDEN_RX_SQ_THRESH_MASK		(0xf << 0)

/* MV_USB2PHY_EDEN_RX_REG1 */
#define MV_USB2PHY_EDEN_RX_SQCAL_DONE_SHIFT		31
#define MV_USB2PHY_EDEN_RX_SQCAL_DONE_MASK		(0x1 << 31)

/* MV_USB2PHY_EDEN_DIG_REG0 */
#define MV_USB2PHY_EDEN_DIG_BITSTAFFING_ERR_MASK	(0x1 << 31)
#define MV_USB2PHY_EDEN_DIG_SYNC_ERR_MASK		(0x1 << 30)

#define MV_USB2PHY_EDEN_DIG_SQ_FILT_SHIFT		16
#define MV_USB2PHY_EDEN_DIG_SQ_FILT_MASK		(0x7 << 16)

#define MV_USB2PHY_EDEN_DIG_SQ_BLK_SHIFT		12
#define MV_USB2PHY_EDEN_DIG_SQ_BLK_MASK		(0x7 << 12)

#define MV_USB2PHY_EDEN_DIG_SYNC_NUM_SHIFT		0
#define MV_USB2PHY_EDEN_DIG_SYNC_NUM_MASK		(0x3 << 0)

#define MV_USB2PHY_EDEN_PLL_LOCK_BYPASS_SHIFT		7

/* MV_USB2PHY_EDEN_OTG_REG */
#define MV_USB2PHY_EDEN_OTG_CONTROL_BY_PIN_SHIFT	5
#define MV_USB2PHY_EDEN_OTG_PU_OTG_SHIFT		4

static int _mv_usb2_phy_55nm_init(struct mv_usb2_phy *mv_phy)
{
	struct platform_device *pdev = mv_phy->pdev;
	unsigned int loops = 0;
	void __iomem *base = mv_phy->base;
	unsigned int val;

	val = readl(base + MV_USB2_UTMI_CTRL);
	/* Initialize the USB PHY power */
	if (mv_phy->drv_data.phy_rev == REV_PXA910) {
		val |= (1 << MV_USB2_UTMI_CTRL_INPKT_DELAY_SOF_SHIFT)
			| (1 << MV_USB2_UTMI_CTRL_PU_REF_SHIFT);
	}

	val |= (1 << MV_USB2_UTMI_CTRL_PLL_PWR_UP_SHIFT)
		| (1 << MV_USB2_UTMI_CTRL_PWR_UP_SHIFT);
	writel(val, base + MV_USB2_UTMI_CTRL);

	/* UTMI_PLL settings */
	val = readl(base + MV_USB2_UTMI_PLL);
	val &= ~(MV_USB2_UTMI_PLL_PLLVDD18_MASK
		| MV_USB2_UTMI_PLL_PLLVDD12_MASK
		| MV_USB2_UTMI_PLL_PLLCALI12_MASK
		| MV_USB2_UTMI_PLL_FBDIV_MASK
		| MV_USB2_UTMI_PLL_REFDIV_MASK
		| MV_USB2_UTMI_PLL_ICP_MASK
		| MV_USB2_UTMI_PLL_KVCO_MASK
		| MV_USB2_UTMI_PLL_VCOCAL_START);

	val |= (0xee << MV_USB2_UTMI_PLL_FBDIV_SHIFT)
		| (0xb << MV_USB2_UTMI_PLL_REFDIV_SHIFT)
		| (3 << MV_USB2_UTMI_PLL_PLLVDD18_SHIFT)
		| (3 << MV_USB2_UTMI_PLL_PLLVDD12_SHIFT)
		| (3 << MV_USB2_UTMI_PLL_PLLCALI12_SHIFT)
		| (1 << MV_USB2_UTMI_PLL_ICP_SHIFT)
		| (3 << MV_USB2_UTMI_PLL_KVCO_SHIFT);
	writel(val, base + MV_USB2_UTMI_PLL);

	/* UTMI_TX */
	val = readl(base + MV_USB2_UTMI_TX);
	val &= ~(MV_USB2_UTMI_TX_REG_EXT_FS_RCAL_EN_MASK
		| MV_USB2_UTMI_TX_TXVDD12_MASK
		| MV_USB2_UTMI_TX_CK60_PHSEL_MASK
		| MV_USB2_UTMI_TX_IMPCAL_VTH_MASK
		| MV_USB2_UTMI_TX_REG_EXT_FS_RCAL_MASK
		| MV_USB2_UTMI_TX_AMP_MASK
		| MV_USB2_UTMI_TX_REG_RCAL_START);
	val |= (3 << MV_USB2_UTMI_TX_TXVDD12_SHIFT)
		| (4 << MV_USB2_UTMI_TX_CK60_PHSEL_SHIFT)
		| (4 << MV_USB2_UTMI_TX_IMPCAL_VTH_SHIFT)
		| (8 << MV_USB2_UTMI_TX_REG_EXT_FS_RCAL_SHIFT)
		| (3 << MV_USB2_UTMI_TX_AMP_SHIFT);
	writel(val, base + MV_USB2_UTMI_TX);

	/* UTMI_RX */
	val = readl(base + MV_USB2_UTMI_RX);
	val &= ~(MV_USB2_UTMI_RX_SQ_THRESH_MASK
		| MV_USB2_UTMI_RX_REG_SQ_LENGTH_MASK);
	val |= (7 << MV_USB2_UTMI_RX_SQ_THRESH_SHIFT)
		| (2 << MV_USB2_UTMI_RX_REG_SQ_LENGTH_SHIFT);
	writel(val, base + MV_USB2_UTMI_RX);

	/* UTMI_IVREF */
	if (mv_phy->drv_data.phy_rev == REV_PXA168)
		/*
		 * Fixing Microsoft Altair board interface with NEC hub issue -
		 * Set UTMI_IVREF from 0x4a3 to 0x4bf.
		 */
		writel(0x4bf, base + MV_USB2_UTMI_IVREF);

	/*
	 * Toggle VCOCAL_START bit of UTMI_PLL. It is active at rising edge.
	 * It is low level by UTMI_PLL initialization above.
	 */
	val = readl(base + MV_USB2_UTMI_PLL);
	/*
	 * Delay 200us for low level, and 40us for high level.
	 * Make sure we can get a effective rising edege.
	 * It should be set to low after issue rising edge.
	 * The delay value is suggested by DE.
	 */
	udelay(300);
	val |= MV_USB2_UTMI_PLL_VCOCAL_START;
	writel(val, base + MV_USB2_UTMI_PLL);
	udelay(40);
	val &= ~MV_USB2_UTMI_PLL_VCOCAL_START;
	writel(val, base + MV_USB2_UTMI_PLL);

	/*
	 * Toggle REG_RCAL_START bit of UTMI_TX.
	 * it is low level by UTMI_TX initialization above.
	 */
	val = readl(base + MV_USB2_UTMI_TX);
	/* same as VCOCAL_START, except it is triggered by low->high->low */
	udelay(400);
	val |= MV_USB2_UTMI_TX_REG_RCAL_START;
	writel(val, base + MV_USB2_UTMI_TX);
	udelay(40);
	val &= ~MV_USB2_UTMI_TX_REG_RCAL_START;
	writel(val, base + MV_USB2_UTMI_TX);
	udelay(400);

	/* Make sure PHY PLL is ready */
	loops = 0;
	while (1) {
		val = readl(base + MV_USB2_UTMI_PLL);
		if (val & MV_USB2_UTMI_PLL_PLL_READY)
			break;
		udelay(1000);
		loops++;
		if (loops > 100) {
			dev_warn(&pdev->dev, "calibrate timeout, UTMI_PLL %x\n",
				 readl(base + MV_USB2_UTMI_PLL));
			return -ETIME;
		}
	}

	if (mv_phy->drv_data.phy_rev == REV_PXA168) {
		val = readl(base + MV_USB2_UTMI_RESERVE);
		val |= (1 << 5);
		writel(val, base + MV_USB2_UTMI_RESERVE);
		/* Turn on UTMI PHY OTG extension */
		writel(MV_USB2_UTMI_OTG_ADDON_OTG_ON,
		       base + MV_USB2_UTMI_OTG_ADDON);
	}

	return 0;
}

static int _mv_usb2_phy_55nm_shutdown(struct mv_usb2_phy *mv_phy)
{
	void __iomem *base = mv_phy->base;
	unsigned int val;

	if (mv_phy->drv_data.phy_rev == REV_PXA168)
		writel(0, base + MV_USB2_UTMI_OTG_ADDON);

	val = readl(base + MV_USB2_UTMI_CTRL);
	val &= ~(MV_USB2_UTMI_CTRL_RXBUF_PDWN
		| MV_USB2_UTMI_CTRL_TXBUF_PDWN
		| MV_USB2_UTMI_CTRL_USB_CLK_EN
		| (1 << MV_USB2_UTMI_CTRL_PWR_UP_SHIFT)
		| (1 << MV_USB2_UTMI_CTRL_PLL_PWR_UP_SHIFT));

	writel(val, base + MV_USB2_UTMI_CTRL);

	return 0;
}

static int _mv_usb2_phy_40nm_init(struct mv_usb2_phy *mv_phy)
{
#if defined(CONFIG_MACH_AGERA)
	void __iomem *base = mv_phy->base;

	u16 tmp16;
	u32 tmp32;

	/* Program 0xd4207004[8:0]= 0xF0 */
	/* Program 0xd4207004[13:9]=0xD */
	writew((MV_USB2_PHY_PLL0_DEFAULT
		& (~MV_USB2_PHY_PLL0_FBDIV(~0))
		& (~MV_USB2_PHY_PLL0_REFDIV(~0)))
		| MV_USB2_PHY_PLL0_FBDIV(0xF0)
		| MV_USB2_PHY_PLL0_REFDIV(0xD), base + MV_USB2_PHY_PLL0);

	/* Program 0xd4207008[11:8]=0x1 */
	/* Program 0xd4207008[14:13]=0x1 */
	writew((MV_USB2_PHY_PLL1_DEFAULT
		& (~MV_USB2_PHY_PLL1_ICP(~0))
		& (~MV_USB2_PHY_PLL1_PLL_CONTROL_BY_PIN))
		| MV_USB2_PHY_PLL1_ICP(0x1)
		| MV_USB2_PHY_PLL1_PU_PLL, base + MV_USB2_PHY_PLL1);

	/* Program 0xd4207034[14]=0x1 */
	writew(MV_USB2_PHY_ANA1_DEFAULT
		| MV_USB2_PHY_ANA1_PU_ANA, base + MV_USB2_PHY_ANA1);

	/* Program 0xd420705c[3]=0x1 */
	writew(MV_USB2_PHY_OTG_DEFAULT
		| MV_USB2_PHY_OTG_PU, base + MV_USB2_PHY_OTG);

	/* Program 0xD4207104[1] = 0x1 */
	/* Program 0xD4207104[0] = 0x1 */
	tmp32 = readl(base + MV_USB2_PHY_CTRL);
	writel(tmp32 | MV_USB2_PHY_CTRL_PU_PLL
		| MV_USB2_PHY_CTRL_PU, base + MV_USB2_PHY_CTRL);

	/* Wait for 200us */
	udelay(200);

	/* Program 0xd4207008[2]=0x1 */
	tmp16 = readw(base + MV_USB2_PHY_PLL1);
	writew(tmp16
		| MV_USB2_PHY_PLL1_VCOCAL_START, base + MV_USB2_PHY_PLL1);

	/* Wait for 400us */
	udelay(400);

	/* Polling 0xd4207008[15]=0x1 */
	while ((readw(base + MV_USB2_PHY_PLL1)
		& MV_USB2_PHY_PLL1_PLL_READY) == 0)
		pr_info("polling usb phy\n");

	tmp16 = readw(base + MV_USB2_PHY_TX2);
	writew((tmp16 & (~MV_USB2_PHY_TX2_DRV_SLEWRATE(3)))
		| MV_USB2_PHY_TX2_DRV_SLEWRATE(2), base + MV_USB2_PHY_TX2);

	/* Program 0xd4207010[13]=0x1 */
	writew(MV_USB2_PHY_TX0_DEFAULT
		| MV_USB2_PHY_TX0_RCAL_START, base + MV_USB2_PHY_TX0);

	/* Wait for 40us */
	udelay(40);

	/* Program 0xd4207010[13]=0x0 */
	tmp16 = readw(base + MV_USB2_PHY_TX0);
	writew(tmp16
		& (~MV_USB2_PHY_TX0_RCAL_START), base + MV_USB2_PHY_TX0);

	/* Wait for 400us */
	udelay(400);

	pr_info("usb phy inited %x!\n", readl(base + MV_USB2_PHY_CTRL));

#else

	struct pxa988_usb_phy *phy = (struct pxa988_usb_phy *)mv_phy->base;
	int i;
	u32 phy_old, phy_power;

	pr_debug("init usb phy.\n");

	/*
	 * power up PHY by PIN.
	 * From the datasheet, it can be controlled by current regiter,
	 * but not pin.
	 * Will remove it after debug.
	 */
	phy_old = (u32)ioremap_nocache(0xD4207100, 0x10);
	phy_power = phy_old + 0x4;
	writel(0x10901003, phy_power);

	/* enable usb device PHY */
	writew(PLLVDD18(0x1) | REFDIV(0xd) | FBDIV(0xf0),
		&phy->utmi_pll_reg0);
	writew(PU_PLL | PLL_LOCK_BYPASS | ICP(0x1) | KVCO(0x3) | PLLCAL12(0x3),
		&phy->utmi_pll_reg1);
	writew(IMPCAL_VTH(0x1) | EXT_HS_RCAL(0x8) | EXT_FS_RCAL(0x8),
		&phy->utmi_tx_reg0);
	writew(TXVDD15(0x1) | TXVDD12(0x3) | LOWVDD_EN |
		AMP(0x5) | CK60_PHSEL(0x4),
		&phy->utmi_tx_reg1);
	writew(DRV_SLEWRATE(0x2) | IMP_CAL_DLY(0x2) |
		FSDRV_EN(0xf) | HSDEV_EN(0xf),
		&phy->utmi_tx_reg2);
#if defined(CONFIG_MACH_CS02) || defined(CONFIG_MACH_WILCOX) || defined(CONFIG_MACH_GOYA)
	writew(PHASE_FREEZE_DLY | ACQ_LENGTH(0x2) | SQ_LENGTH(0x2) |
		DISCON_THRESH(0x2) | SQ_THRESH(0xc) | INTPI(0x1),
		&phy->utmi_rx_reg0);
#elif defined(CONFIG_MACH_LT02)
	writew(PHASE_FREEZE_DLY | ACQ_LENGTH(0x2) | SQ_LENGTH(0x2) |
		DISCON_THRESH(0x2) | SQ_THRESH(0x8) | INTPI(0x1),
		&phy->utmi_rx_reg0);
#else
	writew(PHASE_FREEZE_DLY | ACQ_LENGTH(0x2) | SQ_LENGTH(0x2) |
		DISCON_THRESH(0x2) | SQ_THRESH(0xa) | INTPI(0x1),
		&phy->utmi_rx_reg0);
#endif
	writew(EARLY_VOS_ON_EN | RXDATA_BLOCK_EN | EDGE_DET_EN |
		RXDATA_BLOCK_LENGTH(0x2) | EDGE_DET_SEL(0x1) |
		S2TO3_DLY_SEL(0x2),
		&phy->utmi_rx_reg1);
#ifdef CONFIG_MACH_LT02
	writew(USQ_FILTER | SQ_BUFFER_EN | RXVDD18(0x1) | RXVDD12(0x1),
		&phy->utmi_rx_reg2);
#else
	writew(USQ_FILTER | RXVDD18(0x1) | RXVDD12(0x1),
		&phy->utmi_rx_reg2);
#endif
	writew(BG_VSEL(0x1) | TOPVDD18(0x1),
		&phy->utmi_ana_reg0);
	writew(PU_ANA | SEL_LPFR | V2I(0x6) | R_ROTATE_SEL,
		&phy->utmi_ana_reg1);
	writew(FS_EOP_MODE | FORCE_END_EN | SYNCDET_WINDOW_EN |
		CLK_SUSPEND_EN | FIFO_FILL_NUM(0x6),
		&phy->utmi_dig_reg0);
	writew(FS_RX_ERROR_MODE2 | FS_RX_ERROR_MODE1 |
		FS_RX_ERROR_MODE | ARC_DPDM_MODE,
		&phy->utmi_dig_reg1);
	writew(0x0, &phy->utmi_charger_reg0);

	for (i = 0; i < 0x80; i = i + 4)
		pr_debug("[0x%x] = 0x%x\n", (u32)mv_phy->base + i,
			readw((u32)mv_phy->base + i));

	iounmap((void __iomem *)phy_old);
#endif
	return 0;

}

static void _mv_usb2_phy_40nm_shutdown(struct mv_usb2_phy *mv_phy)
{
	void __iomem *base = mv_phy->base;
	u16 tmp16;
	u32 tmp32;

	pr_info("usb phy deinit!\n");

	/* Program 0xd4207008[14:13]=0x0 */
	tmp16 = readw(base + MV_USB2_PHY_PLL1);
	writew(tmp16
		& (~MV_USB2_PHY_PLL1_PLL_CONTROL_BY_PIN)
		& (~MV_USB2_PHY_PLL1_PU_PLL), base + MV_USB2_PHY_PLL1);

	/* Program 0xd4207034[14]=0x0 */
	tmp16 = readw(base + MV_USB2_PHY_ANA1);
	writew(tmp16
		& (~MV_USB2_PHY_ANA1_PU_ANA), base + MV_USB2_PHY_ANA1);

	/* Program 0xD4207104[1] = 0x0 */
	/* Program 0xD4207104[0] = 0x0 */
	tmp32 = readl(base + MV_USB2_PHY_CTRL);
	writel(tmp32
		& (~MV_USB2_PHY_CTRL_PU_PLL)
		& (~MV_USB2_PHY_CTRL_PU), base + MV_USB2_PHY_CTRL);
}

static int _mv_usb2_phy_28nm_init(struct mv_usb2_phy *mv_phy)
{
	struct platform_device *pdev = mv_phy->pdev;
	unsigned int loops = 0;
	void __iomem *base = mv_phy->base;
	unsigned int tmp, val;

	/* MV_USB2PHY_EDEN_PLL_REG0 */
	writel(readl(base + MV_USB2PHY_EDEN_PLL_REG0) &
		~(MV_USB2PHY_EDEN_PLL_SELLPFR_MASK
		| MV_USB2PHY_EDEN_PLL_FBDIV_MASK
		| MV_USB2PHY_EDEN_PLL_ICP_MASK
		| MV_USB2PHY_EDEN_PLL_REFDIV_MASK),
		base + MV_USB2PHY_EDEN_PLL_REG0);

	writel(readl(base + MV_USB2PHY_EDEN_PLL_REG0) |
		(0x1 << MV_USB2PHY_EDEN_PLL_SELLPFR_SHIFT
		| 0xf0 << MV_USB2PHY_EDEN_PLL_FBDIV_SHIFT
		| 0x3 << MV_USB2PHY_EDEN_PLL_ICP_SHIFT
		| 0xd << MV_USB2PHY_EDEN_PLL_REFDIV_SHIFT),
		base + MV_USB2PHY_EDEN_PLL_REG0);


	/* MV_USB2PHY_EDEN_PLL_REG1 */
	writel(readl(base + MV_USB2PHY_EDEN_PLL_REG1) &
		~(MV_USB2PHY_EDEN_PLL_PU_PLL_MASK
		| MV_USB2PHY_EDEN_PLL_PU_BY_REG_MASK),
		base + MV_USB2PHY_EDEN_PLL_REG1);

	writel(readl(base + MV_USB2PHY_EDEN_PLL_REG1) |
		(0x1 << MV_USB2PHY_EDEN_PLL_PU_PLL_SHIFT
		| 0x1 << MV_USB2PHY_EDEN_PLL_PU_BY_REG_SHIFT),
		base + MV_USB2PHY_EDEN_PLL_REG1);


	/* MV_USB2PHY_EDEN_TX_REG0 */
	writel(readl(base + MV_USB2PHY_EDEN_TX_REG0) &
		~MV_USB2PHY_EDEN_TX_AMP_MASK,
		base + MV_USB2PHY_EDEN_TX_REG0);


	writel(readl(base + MV_USB2PHY_EDEN_TX_REG0) |
		(0x1 << MV_USB2PHY_EDEN_TX_PU_BY_REG_SHIFT
		| 0x3 << MV_USB2PHY_EDEN_TX_AMP_SHIFT
		| 0x1 << MV_USB2PHY_EDEN_TX_PU_ANA_SHIFT),
		base + MV_USB2PHY_EDEN_TX_REG0);


	/* MV_USB2PHY_EDEN_RX_REG0 */
	writel(readl(base + MV_USB2PHY_EDEN_RX_REG0) &
		~MV_USB2PHY_EDEN_RX_SQ_THRESH_MASK,
		base + MV_USB2PHY_EDEN_RX_REG0);

	writel(readl(base + MV_USB2PHY_EDEN_RX_REG0) |
		0xa << MV_USB2PHY_EDEN_RX_SQ_THRESH_SHIFT,
		base + MV_USB2PHY_EDEN_RX_REG0);


	/* MV_USB2PHY_EDEN_DIG_REG0 */
	writel(readl(base + MV_USB2PHY_EDEN_DIG_REG0) &
		~(MV_USB2PHY_EDEN_DIG_BITSTAFFING_ERR_MASK
		| MV_USB2PHY_EDEN_DIG_SYNC_ERR_MASK
		| MV_USB2PHY_EDEN_DIG_SQ_FILT_MASK
		| MV_USB2PHY_EDEN_DIG_SQ_BLK_MASK
		| MV_USB2PHY_EDEN_DIG_SYNC_NUM_MASK),
		base + MV_USB2PHY_EDEN_DIG_REG0);


	writel(readl(base + MV_USB2PHY_EDEN_DIG_REG0) |
		(0x7 << MV_USB2PHY_EDEN_DIG_SQ_FILT_SHIFT
		| 0x4 << MV_USB2PHY_EDEN_DIG_SQ_BLK_SHIFT
		| 0x2 << MV_USB2PHY_EDEN_DIG_SYNC_NUM_SHIFT),
		base + MV_USB2PHY_EDEN_DIG_REG0);


	/* MV_USB2PHY_EDEN_OTG_REG */
	writel(readl(base + MV_USB2PHY_EDEN_OTG_REG) |
		0x1 << MV_USB2PHY_EDEN_OTG_PU_OTG_SHIFT,
		base + MV_USB2PHY_EDEN_OTG_REG);


	writel(readl(base + MV_USB2PHY_EDEN_OTG_REG) &
		~(0x1 << MV_USB2PHY_EDEN_OTG_CONTROL_BY_PIN_SHIFT),
		base + MV_USB2PHY_EDEN_OTG_REG);


	/*  Calibration Timing
	*		   ____________________________
	*  CAL START   ___|
	*			   ____________________
	*  CAL_DONE    ___________|
	*		  | 400us |
	*/
	/* Wait For Calibrate PHY */
	udelay(400);

	/* Make sure PHY Calibration is ready */
	loops = 0;
	do {
		tmp = readl(base + MV_USB2PHY_EDEN_CAL_REG);
		val = (MV_USB2PHY_EDEN_PLL_PLLCAL_DONE_MASK
			| MV_USB2PHY_EDEN_PLL_IMPCAL_DONE_MASK);
		tmp &= val;
		if (tmp == val) {
			tmp = readl(base + MV_USB2PHY_EDEN_RX_REG1);
			if (tmp & MV_USB2PHY_EDEN_RX_SQCAL_DONE_MASK)
				break;
		}

		udelay(1000);
	} while ((loops++) && (loops < 100));
	if (loops >= 100)
		dev_warn(&pdev->dev, "USB PHY Calibrate not done after 100mS.");

	/* Make sure PHY PLL is ready */
	loops = 0;
	tmp = readl(base + MV_USB2PHY_EDEN_PLL_REG0);
	while (!(tmp & MV_USB2PHY_EDEN_PLL_READY_MASK)) {
		udelay(1000);
		loops++;
		if (loops > 100) {
			dev_warn(&pdev->dev, "PLL_READY not set after 100mS.");
			break;
		}
	}

	return 0;
}

static void _mv_usb2_phy_28nm_shutdown(struct mv_usb2_phy *mv_phy)
{
	void __iomem *base = mv_phy->base;
	unsigned int val;

	val = readw(base + MV_USB2PHY_EDEN_PLL_REG1);
	val &= ~MV_USB2PHY_EDEN_PLL_PU_PLL_MASK;
	writew(val, base + MV_USB2PHY_EDEN_PLL_REG1);

	/* power down PHY Analog part */
	val = readw(base + MV_USB2PHY_EDEN_TX_REG0);
	val &= ~(0x1 << MV_USB2PHY_EDEN_TX_PU_ANA_SHIFT);
	writew(val, base + MV_USB2PHY_EDEN_TX_REG0);

	/* power down PHY OTG part */
	val = readw(base + MV_USB2PHY_EDEN_OTG_REG);
	val &= ~(0x1 << MV_USB2PHY_EDEN_OTG_PU_OTG_SHIFT);
	writew(val, base + MV_USB2PHY_EDEN_OTG_REG);
}

static int mv_usb2_phy_init(struct usb_phy *phy)
{
	struct mv_usb2_phy *mv_phy = container_of(phy, struct mv_usb2_phy, phy);

	clk_prepare_enable(mv_phy->clk);
	switch (mv_phy->drv_data.phy_type) {
	case PHY_40LP:
		_mv_usb2_phy_40nm_init(mv_phy);
		break;
	case PHY_28LP:
		_mv_usb2_phy_28nm_init(mv_phy);
		break;
	case PHY_55LP:
		_mv_usb2_phy_55nm_init(mv_phy);
		break;
	default:
		pr_err("No such phy type supported!\n");
		break;
	}

	return 0;
}

static void mv_usb2_phy_shutdown(struct usb_phy *phy)
{
	struct mv_usb2_phy *mv_phy = container_of(phy, struct mv_usb2_phy, phy);

	switch (mv_phy->drv_data.phy_type) {
	case PHY_40LP:
		_mv_usb2_phy_40nm_shutdown(mv_phy);
		break;
	case PHY_28LP:
		_mv_usb2_phy_28nm_shutdown(mv_phy);
		break;
	case PHY_55LP:
		_mv_usb2_phy_55nm_shutdown(mv_phy);
		break;
	default:
		pr_err("No such phy type supported!\n");
		break;
	}

	clk_disable_unprepare(mv_phy->clk);
}

static const struct of_device_id mv_usbphy_dt_match[];

static int mv_usb2_get_phydata(struct platform_device *pdev,
				struct mv_usb2_phy *mv_phy)
{
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *match;
	u32 phy_rev;

	match = of_match_device(mv_usbphy_dt_match, &pdev->dev);
	if (!match)
		return -ENODEV;

	mv_phy->drv_data.phy_type = (u32)match->data;

	if (!of_property_read_u32(np, "marvell,usb2-phy-rev", &phy_rev))
		mv_phy->drv_data.phy_rev = phy_rev;
	else
		pr_info("No PHY revision found, use the default setting!");

	return 0;
}

static void mv_usb2_phy_bind_device(struct mv_usb2_phy *mv_phy)
{
	const char *device_name;

	struct device_node *np = (mv_phy->phy.dev)->of_node;

	if (!of_property_read_string(np, "marvell,udc-name", &device_name))
		usb_bind_phy(device_name, MV_USB2_PHY_INDEX,
						dev_name(mv_phy->phy.dev));

	if (!of_property_read_string(np, "marvell,ehci-name", &device_name))
		usb_bind_phy(device_name, MV_USB2_PHY_INDEX,
						dev_name(mv_phy->phy.dev));

	if (!of_property_read_string(np, "marvell,otg-name", &device_name))
		usb_bind_phy(device_name, MV_USB2_PHY_INDEX,
						dev_name(mv_phy->phy.dev));
}

static int mv_usb2_phy_probe(struct platform_device *pdev)
{
	struct mv_usb2_phy *mv_phy;
	struct resource *r;
	int ret = 0;

	mv_phy = devm_kzalloc(&pdev->dev, sizeof(*mv_phy), GFP_KERNEL);
	if (mv_phy == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	mv_phy->pdev = pdev;

	ret = mv_usb2_get_phydata(pdev, mv_phy);
	if (ret) {
		dev_err(&pdev->dev, "No matching phy founded\n");
		return ret;
	}

	mv_phy->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(mv_phy->clk)) {
		dev_err(&pdev->dev, "failed to get clock.\n");
		return PTR_ERR(mv_phy->clk);
	}
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		dev_err(&pdev->dev, "no phy I/O memory resource defined\n");
		return -ENODEV;
	}
	mv_phy->base = devm_ioremap_resource(&pdev->dev, r);
	if (mv_phy->base == NULL) {
		dev_err(&pdev->dev, "error map register base\n");
		return -EBUSY;
	}

	mv_phy->phy.dev = &pdev->dev;
	mv_phy->phy.label = "mv-usb2";
	mv_phy->phy.type = USB_PHY_TYPE_USB2;
	mv_phy->phy.init = mv_usb2_phy_init;
	mv_phy->phy.shutdown = mv_usb2_phy_shutdown;

	mv_usb2_phy_bind_device(mv_phy);

	usb_add_phy_dev(&mv_phy->phy);

	platform_set_drvdata(pdev, mv_phy);

	return 0;
}

static int mv_usb2_phy_remove(struct platform_device *pdev)
{
	struct mv_usb2_phy *mv_phy = platform_get_drvdata(pdev);

	usb_remove_phy(&mv_phy->phy);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id mv_usbphy_dt_match[] = {
	{ .compatible = "marvell,usb2-phy-55lp", .data = (void *)PHY_55LP },
	{ .compatible = "marvell,usb2-phy-40lp", .data = (void *)PHY_40LP },
	{ .compatible = "marvell,usb2-phy-28lp", .data = (void *)PHY_28LP },
	{},
};
MODULE_DEVICE_TABLE(of, mv_usbphy_dt_match);

static struct platform_driver mv_usb2_phy_driver = {
	.probe	= mv_usb2_phy_probe,
	.remove = mv_usb2_phy_remove,
	.driver = {
		.name   = "mv-usb2-phy",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(mv_usbphy_dt_match),
	},
};

module_platform_driver(mv_usb2_phy_driver);
MODULE_ALIAS("platform: mv_usb2");
MODULE_AUTHOR("Marvell Inc.");
MODULE_DESCRIPTION("Marvell USB2 phy driver");
MODULE_LICENSE("GPL v2");
