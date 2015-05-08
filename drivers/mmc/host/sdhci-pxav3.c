/*
 * Copyright (C) 2010 Marvell International Ltd.
 *		Zhangfei Gao <zhangfei.gao@marvell.com>
 *		Kevin Wang <dwang4@marvell.com>
 *		Mingwei Wang <mwwang@marvell.com>
 *		Philip Rakity <prakity@marvell.com>
 *		Mark Brown <markb@marvell.com>
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
#include <linux/err.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/platform_data/pxa_sdhci.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/pm_qos.h>
#include <dt-bindings/mmc/pxa_sdhci.h>

#include <linux/pinctrl/consumer.h>
#include <linux/dma-mapping.h>
#include "sdhci.h"
#include "sdhci-pltfm.h"

#define PXAV3_RPM_DELAY_MS		50
#define SDCLK_SEL			0x100
#define SDCLK_DELAY_SHIFT		9
#define SDCLK_DELAY_MASK		0x1f
#define SDCFG_GEN_PAD_CLK_ON		(1<<6)
#define SDCFG_GEN_PAD_CLK_CNT_MASK	0xFF
#define SDCFG_GEN_PAD_CLK_CNT_SHIFT	24
#define SDCFG_PIO_RDFC			(1<<0)
#define SD_SPI_MODE			0x108
#define SDCE_MISC_INT			(1<<2)
#define SDCE_MISC_INT_EN		(1<<1)
#define RX_SDCLK_DELAY_SHIFT		8
#define RX_SDCLK_SEL0_MASK		0x3
#define RX_SDCLK_SEL1_MASK		0x3
#define RX_SDCLK_SEL0_SHIFT		0
#define RX_SDCLK_SEL1_SHIFT		2
#define PAD_CLK_GATE_MASK		(0x3<<11)
#define TX_DELAY1_SHIFT			16
#define TX_MUX_SEL			(0x1<<31)
#define TX_SEL_BUS_CLK			(0x1<<30)
#define RX_TUNING_CFG_REG		0x11C
#define RX_TUNING_WD_CNT_MASK		0x3F
#define RX_TUNING_WD_CNT_SHIFT		8
#define RX_TUNING_TT_CNT_MASK		0xFF
#define RX_TUNING_TT_CNT_SHIFT		0

#ifdef CONFIG_CPU_EDEN
#define TX_DELAY_MASK			0x3FF
#define RX_TUNING_DLY_INC_MASK		0x3FF
#define RX_TUNING_DLY_INC_SHIFT		18
#define RX_SDCLK_DELAY_MASK		0x3FF
#else
#define TX_DELAY_MASK			0x1FF
#define RX_TUNING_DLY_INC_MASK		0x1FF
#define RX_TUNING_DLY_INC_SHIFT		17
#define RX_SDCLK_DELAY_MASK		0x1FF
#endif

#ifdef CONFIG_CPU_PXA1986
#define SD_CLOCK_BURST_SIZE_SETUP	0x1A4
#define SD_CFG_FIFO_PARAM		0x19C
#define SD_CE_ATA_1			0x1A8
#define SD_CE_ATA_2			0x1AA
#define SD_FIFO_PARAM			0x1A0
#define SD_RX_CFG_REG			0x1B0
#define SD_TX_CFG_REG			0x1B4

#else /* not CONFIG_CPU_PXA1986 */

#define SD_CLOCK_BURST_SIZE_SETUP	0x10A
#define SD_CFG_FIFO_PARAM		0x100
#define SD_CE_ATA_1			0x10C
#define SD_CE_ATA_2			0x10E
#define SD_FIFO_PARAM			0x104
#define SD_RX_CFG_REG			0x114
#define SD_TX_CFG_REG			0x118
#endif

#define SD_RX_TUNE_MIN			0
#define SD_RX_TUNE_MAX			RX_SDCLK_DELAY_MASK
#define SD_RX_TUNE_STEP			1

static void pxav3_set_rx_cfg(struct sdhci_host *host,
		struct sdhci_pxa_platdata *pdata)
{
	u32 tmp_reg = 0;
	struct mmc_ios *ios = &host->mmc->ios;
	struct sdhci_pxa_dtr_data *dtr_data;

	if ((!pdata) || (!pdata->dtr_data))
		return;

	if ((MMC_TIMING_UHS_SDR104 == ios->timing)
			|| (MMC_TIMING_MMC_HS200 == ios->timing)) {
		return;
	}

	if (MMC_TIMING_LEGACY == ios->timing) {
		tmp_reg = 0x0;
		goto exit;
	}

	/* From PXA988, the RX_CFG Reg is changed to 0x114 */
	if (pdata->flags & PXA_FLAG_NEW_RX_CFG_REG) {
		tmp_reg = sdhci_readl(host, SD_RX_CFG_REG);

		dtr_data = pdata->dtr_data;

		/* set Rx delay */
		while (MMC_TIMING_MAX != dtr_data->timing) {
			if (dtr_data->timing == ios->timing) {
				if (dtr_data->rx_delay) {
					/* clock by SDCLK_SEL0, so it is default setting */
					tmp_reg &= ~(RX_SDCLK_SEL1_MASK << RX_SDCLK_SEL1_SHIFT);
					tmp_reg |= 0x1 << RX_SDCLK_SEL1_SHIFT;
					tmp_reg &= ~(RX_SDCLK_DELAY_MASK << RX_SDCLK_DELAY_SHIFT);
					tmp_reg |= (dtr_data->rx_delay & RX_SDCLK_DELAY_MASK) << RX_SDCLK_DELAY_SHIFT;
				} else
					tmp_reg = 0x0;
				break;
			}
			dtr_data++;
		}
	}
exit:
	sdhci_writel(host, tmp_reg, SD_RX_CFG_REG);
}

static ssize_t rx_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mmc_host *mmc_host =
		container_of(dev, struct mmc_host, class_dev);
	struct sdhci_host *host = mmc_priv(mmc_host);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_pxa *pxa = pltfm_host->priv;
	int ret = 0;
	u32 tmp_reg = 0;

	sdhci_access_constrain(host, 1);
	sdhci_runtime_pm_get(host);

	tmp_reg = sdhci_readl(host, SD_RX_CFG_REG);
	pxa->rx_dly_val = (tmp_reg >> RX_SDCLK_DELAY_SHIFT) & RX_SDCLK_DELAY_MASK;
	ret = sprintf(buf, "rx delay: 0x%x\t| RX config: 0x%08x\n", pxa->rx_dly_val, tmp_reg);

	sdhci_runtime_pm_put(host);
	sdhci_access_constrain(host, 0);

	return ret;
}
static ssize_t rx_delay_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct mmc_host *mmc_host =
		container_of(dev, struct mmc_host, class_dev);
	struct sdhci_host *host = mmc_priv(mmc_host);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_pxa *pxa = pltfm_host->priv;
	u32 tmp_reg = 0;

	sscanf(buf, "%d", &pxa->rx_dly_val);

	sdhci_access_constrain(host, 1);
	sdhci_runtime_pm_get(host);
	tmp_reg = sdhci_readl(host, SD_RX_CFG_REG);
	/* clock by SDCLK_SEL0, so it is default setting */
	tmp_reg &= ~(RX_SDCLK_SEL1_MASK << RX_SDCLK_SEL1_SHIFT);
	tmp_reg |= 0x1 << RX_SDCLK_SEL1_SHIFT;
	tmp_reg &= ~(RX_SDCLK_DELAY_MASK << RX_SDCLK_DELAY_SHIFT);
	tmp_reg |= (pxa->rx_dly_val & RX_SDCLK_DELAY_MASK) << RX_SDCLK_DELAY_SHIFT;
	sdhci_writel(host, tmp_reg, SD_RX_CFG_REG);
	sdhci_runtime_pm_put(host);
	sdhci_access_constrain(host, 0);

	return count;
}

static void pxav3_set_tx_cfg(struct sdhci_host *host,
		struct sdhci_pxa_platdata *pdata)
{
	u32 tmp_reg = 0;
	struct mmc_ios *ios = &host->mmc->ios;
	struct sdhci_pxa_dtr_data *dtr_data;

	tmp_reg = sdhci_readl(host, SD_TX_CFG_REG);

	if (pdata && pdata->flags & PXA_FLAG_TX_SEL_BUS_CLK) {
		/*
		 * For the hold time at default speed mode or high speed mode
		 * PXAV3 should enable the TX_SEL_BUS_CLK
		 * Which will select clock from inverter of internal work clock
		 * This setting will guarantee the hold time
		 */
		if (ios->timing > MMC_TIMING_UHS_SDR25)

			tmp_reg &= ~TX_SEL_BUS_CLK;
		else {
			tmp_reg &= ~TX_MUX_SEL;
			tmp_reg |= TX_SEL_BUS_CLK;
			sdhci_writel(host, tmp_reg, SD_TX_CFG_REG);
			return;
		}
	}

	if (pdata && pdata->dtr_data) {
		dtr_data = pdata->dtr_data;

		/* set Tx delay */
		while (MMC_TIMING_MAX != dtr_data->timing) {
			if (dtr_data->timing == ios->timing) {
				if (dtr_data->tx_delay) {
					tmp_reg |= TX_MUX_SEL;
					if ((MMC_TIMING_UHS_SDR104 == ios->timing)
							|| (MMC_TIMING_MMC_HS200 == ios->timing)) {
						tmp_reg &= ~(TX_DELAY_MASK << TX_DELAY1_SHIFT);
						tmp_reg |= (dtr_data->tx_delay & TX_DELAY_MASK) << TX_DELAY1_SHIFT;
					} else {
						tmp_reg &= ~TX_DELAY_MASK;
						tmp_reg |= (dtr_data->tx_delay & TX_DELAY_MASK);
					}
					break;
				} else
					tmp_reg &= ~TX_MUX_SEL;
			}
			dtr_data++;
		}
	}

	sdhci_writel(host, tmp_reg, SD_TX_CFG_REG);
}

static ssize_t tx_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mmc_host *mmc_host =
		container_of(dev, struct mmc_host, class_dev);
	struct sdhci_host *host = mmc_priv(mmc_host);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_pxa *pxa = pltfm_host->priv;
	struct mmc_ios *ios = &host->mmc->ios;
	int ret = 0;
	u32 tmp_reg = 0;

	sdhci_access_constrain(host, 1);
	sdhci_runtime_pm_get(host);
	tmp_reg = sdhci_readl(host, SD_TX_CFG_REG);
	if ((MMC_TIMING_UHS_SDR104 == ios->timing) || (MMC_TIMING_MMC_HS200 == ios->timing))
		pxa->tx_dly_val = (tmp_reg >> TX_DELAY1_SHIFT) & TX_DELAY_MASK;
	else
		pxa->tx_dly_val = tmp_reg & TX_DELAY_MASK;

	ret = sprintf(buf, "tx delay: 0x%x\t| TX config: 0x%08x\n", pxa->tx_dly_val, tmp_reg);

	sdhci_runtime_pm_put(host);
	sdhci_access_constrain(host, 0);
	return ret;
}

static ssize_t tx_delay_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct mmc_host *mmc_host =
		container_of(dev, struct mmc_host, class_dev);
	struct sdhci_host *host = mmc_priv(mmc_host);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_pxa *pxa = pltfm_host->priv;
	struct mmc_ios *ios = &host->mmc->ios;
	u32 tmp_reg = 0;

	if ((ios->timing <= MMC_TIMING_UHS_SDR25)) {
		/* For HS ans DS mode
		 * PXAV3 can handle the mmc bus timing issue by HW
		 * So we don't need to set tx delay for these modes
		 */
		printk(KERN_INFO "Can't set tx delay in HS or DS mode!\n");
		return count;
	}

	sscanf(buf, "%d", &pxa->tx_dly_val);
	sdhci_access_constrain(host, 1);
	sdhci_runtime_pm_get(host);

	tmp_reg = sdhci_readl(host, SD_TX_CFG_REG);
	tmp_reg |= TX_MUX_SEL;
	tmp_reg &= ~TX_SEL_BUS_CLK;
	if ((MMC_TIMING_UHS_SDR104 == ios->timing) || (MMC_TIMING_MMC_HS200 == ios->timing)) {
		tmp_reg &= ~(TX_DELAY_MASK << TX_DELAY1_SHIFT);
		tmp_reg |= (pxa->tx_dly_val & TX_DELAY_MASK) << TX_DELAY1_SHIFT;
	} else {
		tmp_reg &= ~TX_DELAY_MASK;
		tmp_reg |= (pxa->tx_dly_val & TX_DELAY_MASK);
	}
	sdhci_writel(host, tmp_reg, SD_TX_CFG_REG);
	sdhci_runtime_pm_put(host);
	sdhci_access_constrain(host, 0);

	return count;
}

static void pxav3_set_clock(struct sdhci_host *host, unsigned int clock)
{
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	struct sdhci_pxa_platdata *pdata = pdev->dev.platform_data;

	if (clock == 0)
		return;

	pxav3_set_tx_cfg(host, pdata);
	pxav3_set_rx_cfg(host, pdata);

	/*
	 * Configure pin state like drive strength according to bus clock.
	 * Use slow setting when new bus clock < FAST_CLOCK while current >= FAST_CLOCK.
	 * Use fast setting when new bus clock >= FAST_CLOCK while current < FAST_CLOCK.
	 */
#define FAST_CLOCK 100000000
	if (clock < FAST_CLOCK) {
		if ((host->clock >= FAST_CLOCK) && (!IS_ERR(pdata->pin_slow)))
			pinctrl_select_state(pdata->pinctrl, pdata->pin_slow);
	} else {
		if ((host->clock < FAST_CLOCK) && (!IS_ERR(pdata->pin_fast)))
			pinctrl_select_state(pdata->pinctrl, pdata->pin_fast);
	}
}

static unsigned long pxav3_clk_prepare(struct sdhci_host *host,
		unsigned long rate)
{
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	struct sdhci_pxa_platdata *pdata = pdev->dev.platform_data;
	struct sdhci_pxa_dtr_data *dtr_data;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct mmc_ios *ios = &host->mmc->ios;

	unsigned long preset_rate = 0, src_rate = 0;

	if (pdata && pdata->dtr_data) {
		dtr_data = pdata->dtr_data;
		/* search the dtr table */
		while (MMC_TIMING_MAX != dtr_data->timing) {
			if (ios->timing == dtr_data->timing) {
				if ((MMC_TIMING_LEGACY == ios->timing) &&
						(rate != PXA_SDH_DTR_25M))
					preset_rate = rate;
				else
					preset_rate = dtr_data->preset_rate;
				src_rate = dtr_data->src_rate;
				break;
			}
			dtr_data++;
		}
		/* if timing mode not in the dtr table, use default src rate */
		if (MMC_TIMING_MAX == dtr_data->timing) {
			src_rate = dtr_data->src_rate;
			preset_rate = rate;
		}
		clk_set_rate(pltfm_host->clk, src_rate);

		return preset_rate;
	} else
		return rate;
	return 0;
}

static void pxav3_clk_gate_auto(struct sdhci_host *host, unsigned int ctrl)
{
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	struct sdhci_pxa_platdata *pdata = pdev->dev.platform_data;
	u16 tmp;
	/*
	 * FIXME: according to MMC Spec, bit SDHCI_CTRL_ASYNC_INT
	 * should be used to deliver async interrupt requested by
	 * sdio device rather than auto clock gate. But in some
	 * platforms, such as PXA920/PXA988/PXA986/MMP2/MMP3, the
	 * mmc host controller use this bit to enable/disable auto
	 * clock gate, except PXA1088 platform. So in PXA1088 platform
	 * use the FORCE_CLK_ON bit to always enable the bus clock.
	 * In order to diff the PXA1088 and other platforms, use
	 * SDHCI_QUIRK2_BUS_CLK_GATE_ENABLED for PXA1088.
	 */
	if (pdata) {
		if (pdata->quirks2 & SDHCI_QUIRK2_BUS_CLK_GATE_ENABLED) {
			tmp = sdhci_readw(host, SD_FIFO_PARAM);
			if (ctrl)
				tmp &= ~PAD_CLK_GATE_MASK;
			else
				tmp |= PAD_CLK_GATE_MASK;

			sdhci_writew(host, tmp, SD_FIFO_PARAM);
		} else {
			tmp = sdhci_readw(host, SD_FIFO_PARAM);
			tmp &= ~PAD_CLK_GATE_MASK;
			sdhci_writew(host, tmp, SD_FIFO_PARAM);

			tmp = sdhci_readw(host, SDHCI_HOST_CONTROL2);

			if (ctrl)
				tmp |= SDHCI_CTRL_ASYNC_INT;
			else
				tmp &= ~SDHCI_CTRL_ASYNC_INT;

			sdhci_writew(host, tmp, SDHCI_HOST_CONTROL2);
		}
	}
}

static void pxav3_set_private_registers(struct sdhci_host *host, u8 mask)
{
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	struct sdhci_pxa_platdata *pdata = pdev->dev.platform_data;

	if (mask != SDHCI_RESET_ALL) {
		/* Return if not Reset All */
		return;
	}

	/*
	 * tune timing of read data/command when crc error happen
	 * no performance impact
	 */
	pxav3_set_tx_cfg(host, pdata);
	pxav3_set_rx_cfg(host, pdata);
}

#define MAX_WAIT_COUNT 74
static void pxav3_gen_init_74_clocks(struct sdhci_host *host, u8 power_mode)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_pxa *pxa = pltfm_host->priv;
	u16 tmp;
	int count = 0;

	if (pxa->power_mode == MMC_POWER_UP
			&& power_mode == MMC_POWER_ON) {

		dev_dbg(mmc_dev(host->mmc),
				"%s: slot->power_mode = %d,"
				"ios->power_mode = %d\n",
				__func__,
				pxa->power_mode,
				power_mode);

		/* clear the interrupt bit if posted and
		 * set we want notice of when 74 clocks are sent
		 */
		tmp = readw(host->ioaddr + SD_CE_ATA_2);
		tmp |= SDCE_MISC_INT | SDCE_MISC_INT_EN;
		writew(tmp, host->ioaddr + SD_CE_ATA_2);

		/* start sending the 74 clocks */
		tmp = readw(host->ioaddr + SD_CFG_FIFO_PARAM);
		tmp |= SDCFG_GEN_PAD_CLK_ON;
		writew(tmp, host->ioaddr + SD_CFG_FIFO_PARAM);

		/* slowest speed is about 100KHz or 10usec per clock */
		while (count++ < MAX_WAIT_COUNT) {
			if (readw(host->ioaddr + SD_CE_ATA_2)
						& SDCE_MISC_INT) {
				break;
			}
			udelay(10);
		}

		if (count >= MAX_WAIT_COUNT)
			dev_warn(mmc_dev(host->mmc), "74 clock interrupt not cleared\n");
	}
	pxa->power_mode = power_mode;
}

static int pxav3_set_uhs_signaling(struct sdhci_host *host, unsigned int uhs)
{
	u16 ctrl_2;
	u16 fparm;

	/*
	 * Set V18_EN -- UHS modes do not work without this.
	 * does not change signaling voltage
	 */
	ctrl_2 = sdhci_readw(host, SDHCI_HOST_CONTROL2);
	ctrl_2 &= ~SDHCI_CTRL_UHS_MASK;
	fparm = sdhci_readw(host, SD_CFG_FIFO_PARAM);
	fparm &= ~SDCFG_PIO_RDFC;

	/* Select Bus Speed Mode for host */
	switch (uhs) {
	case MMC_TIMING_UHS_SDR12:
		ctrl_2 |= SDHCI_CTRL_UHS_SDR12;
		break;
	case MMC_TIMING_UHS_SDR25:
		ctrl_2 |= SDHCI_CTRL_UHS_SDR25;
		break;
	case MMC_TIMING_UHS_SDR50:
		ctrl_2 |= SDHCI_CTRL_UHS_SDR50 | SDHCI_CTRL_VDD_180;
		break;
	case MMC_TIMING_UHS_SDR104:
		ctrl_2 |= SDHCI_CTRL_UHS_SDR104 | SDHCI_CTRL_VDD_180;
		/* PIO mode need this */
		fparm |= SDCFG_PIO_RDFC;
		break;
	case MMC_TIMING_UHS_DDR50:
		ctrl_2 |= SDHCI_CTRL_UHS_DDR50 | SDHCI_CTRL_VDD_180;
		break;
	}

	sdhci_writew(host, fparm, SD_CFG_FIFO_PARAM);
	sdhci_writew(host, ctrl_2, SDHCI_HOST_CONTROL2);
	dev_dbg(mmc_dev(host->mmc),
		"%s uhs = %d, ctrl_2 = %04X\n",
		__func__, uhs, ctrl_2);

	return 0;
}

static void pxav3_clr_wakeup_event(struct sdhci_host *host)
{
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	struct sdhci_pxa_platdata *pdata = pdev->dev.platform_data;

	if (!pdata)
		return;

	if (pdata->clear_wakeup_event)
		pdata->clear_wakeup_event();
}

static void pxav3_signal_vol_change(struct sdhci_host *host, u8 vol)
{
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	struct sdhci_pxa_platdata *pdata = pdev->dev.platform_data;
	unsigned int set;

	switch (vol) {
		case MMC_SIGNAL_VOLTAGE_330:
			set = 3300000;
			break;
		case MMC_SIGNAL_VOLTAGE_180:
			set = 1800000;
			break;
		case MMC_SIGNAL_VOLTAGE_120:
			set = 1200000;
			break;
		default:
			set = 3300000;
			break;
	}
	if (pdata && pdata->signal_vol_change)
		pdata->signal_vol_change(set);
}

static void pxav3_access_constrain(struct sdhci_host *host, unsigned int ac)
{
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	struct sdhci_pxa_platdata *pdata = pdev->dev.platform_data;

	if (!pdata)
		return;
	if (ac)
		pm_qos_update_request(&pdata->qos_idle, pdata->lpm_qos);
	else
		pm_qos_update_request(&pdata->qos_idle, PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE);
}

static void pxav3_prepare_tuning(struct sdhci_host *host, u32 val)
{
	u32 reg;

	/* delay 1ms for card to be ready for next tuning */
	mdelay(1);

	reg = sdhci_readl(host, SD_RX_CFG_REG);
	reg &= ~(RX_SDCLK_DELAY_MASK << RX_SDCLK_DELAY_SHIFT);
	reg |= (val & RX_SDCLK_DELAY_MASK) << RX_SDCLK_DELAY_SHIFT;
	reg &= ~((RX_SDCLK_SEL0_MASK << RX_SDCLK_SEL0_SHIFT) |
			(RX_SDCLK_SEL1_MASK << RX_SDCLK_SEL1_SHIFT));
	reg |= (1 << RX_SDCLK_SEL1_SHIFT);
	sdhci_writel(host, reg, SD_RX_CFG_REG);

	dev_dbg(mmc_dev(host->mmc), "tunning with delay 0x%x \n", val);
}

static void pxav3_request_done(struct mmc_request *mrq)
{
	complete(&mrq->completion);
}

static int pxav3_send_tuning_cmd(struct sdhci_host *host, u32 opcode)
{
	struct mmc_command cmd = {0};
	struct mmc_request mrq = {0};
	struct mmc_data data = {0};
	struct scatterlist sg;
	char tuning_pattern_64[64];
	char tuning_pattern_128[128];

	cmd.opcode = opcode;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;

	data.blocks = 1;
	data.flags = MMC_DATA_READ;
	data.sg = &sg;
	data.sg_len = 1;

	/*
	 * UHS-I modes only support 4bit width.
	 * HS200 support 4bit or 8bit width.
	 * 8bit used 128byte test pattern while 4bit used 64byte.
	 */
	if (host->mmc->ios.bus_width == MMC_BUS_WIDTH_8) {
		sg_init_one(&sg, tuning_pattern_128, sizeof(tuning_pattern_128));
		data.blksz = 128;
	} else {
		sg_init_one(&sg, tuning_pattern_64, sizeof(tuning_pattern_64));
		data.blksz = 64;
	}

	mrq.cmd = &cmd;
	mrq.cmd->mrq = &mrq;
	mrq.data = &data;
	mrq.data->mrq = &mrq;
	mrq.cmd->data = mrq.data;

	mrq.done = pxav3_request_done;
	init_completion(&(mrq.completion));

	sdhci_request(host->mmc, &mrq);

	wait_for_completion(&mrq.completion);

	if (cmd.error)
		return cmd.error;
	if (data.error)
		return data.error;

	return 0;
}

static int pxav3_executing_tuning(struct sdhci_host *host, u32 opcode)
{
	int min, max, ret;
	int len = 0, avg = 0;

	/* find the mininum delay first which can pass tuning */
	min = SD_RX_TUNE_MIN;
	do {
		while (min < SD_RX_TUNE_MAX) {
			pxav3_prepare_tuning(host, min);
			if (!pxav3_send_tuning_cmd(host, opcode))
				break;
			min += SD_RX_TUNE_STEP;
		}

		/* find the maxinum delay which can not pass tuning */
		max = min + SD_RX_TUNE_STEP;
		while (max < SD_RX_TUNE_MAX) {
			pxav3_prepare_tuning(host, max);
			if (pxav3_send_tuning_cmd(host, opcode))
				break;
			max += SD_RX_TUNE_STEP;
		}

		if ((max - min) > len) {
			len = max - min;
			avg = (min + max - 1) / 2;
		}

		min = max + SD_RX_TUNE_STEP;
	} while (min < SD_RX_TUNE_MAX);

	pxav3_prepare_tuning(host, avg);
	ret = pxav3_send_tuning_cmd(host, opcode);

	dev_dbg(mmc_dev(host->mmc), "tunning %s at 0x%x, pass window length is 0x%x\n",
			ret ? "failed" : "passed", avg, len);

	return ret;
}

/*
 * remove the caps that supported by the controller but not available
 * for certain platforms.
 */
static void pxav3_host_caps_disable(struct sdhci_host *host)
{
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	struct sdhci_pxa_platdata *pdata = pdev->dev.platform_data;

	if (pdata->host_caps_disable)
		host->mmc->caps &= ~(pdata->host_caps_disable);
	if (pdata->host_caps2_disable)
		host->mmc->caps2 &= ~(pdata->host_caps2_disable);
}

static const struct sdhci_ops pxav3_sdhci_ops = {
	.platform_reset_exit = pxav3_set_private_registers,
	.set_uhs_signaling = pxav3_set_uhs_signaling,
	.platform_send_init_74_clocks = pxav3_gen_init_74_clocks,
	.get_max_clock = sdhci_pltfm_clk_get_max_clock,
	.clk_prepare = pxav3_clk_prepare,
	.clr_wakeup_event = pxav3_clr_wakeup_event,
	.signal_vol_change = pxav3_signal_vol_change,
	.clk_gate_auto  = pxav3_clk_gate_auto,
	.access_constrain = pxav3_access_constrain,
	.set_clock = pxav3_set_clock,
	.platform_execute_tuning = pxav3_executing_tuning,
	.host_caps_disable = pxav3_host_caps_disable,
};

static struct sdhci_pltfm_data sdhci_pxav3_pdata = {
	.quirks = SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK
		| SDHCI_QUIRK_NO_ENDATTR_IN_NOPDESC
		| SDHCI_QUIRK_32BIT_ADMA_SIZE
		| SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.ops = &pxav3_sdhci_ops,
};

static int pxav3_init_host_with_pdata(struct sdhci_host *host,
		struct sdhci_pxa_platdata *pdata)
{
	struct platform_device *pdev = to_platform_device(mmc_dev(host->mmc));
	int ret = 0;

	/* If slot design supports 8 bit data, indicate this to MMC. */
	if (pdata->flags & PXA_FLAG_SD_8_BIT_CAPABLE_SLOT)
		host->mmc->caps |= MMC_CAP_8_BIT_DATA;

	if (pdata->flags & PXA_FLAG_DISABLE_PROBE_CDSCAN)
		host->mmc->caps2 |= MMC_CAP2_DISABLE_PROBE_CDSCAN;

	if (pdata->flags & PXA_FLAG_ENABLE_CLOCK_GATING)
		host->mmc->caps2 |= MMC_CAP2_BUS_AUTO_CLK_GATE;

	if (pdata->quirks)
		host->quirks |= pdata->quirks;
	if (pdata->quirks2)
		host->quirks2 |= pdata->quirks2;
	if (pdata->host_caps)
		host->mmc->caps |= pdata->host_caps;
	if (pdata->host_caps2)
		host->mmc->caps2 |= pdata->host_caps2;
	if (pdata->pm_caps)
		host->mmc->pm_caps |= pdata->pm_caps;

	if (pdata->flags & PXA_FLAG_WAKEUP_HOST) {
		device_init_wakeup(&pdev->dev, 1);
		host->mmc->pm_flags |= MMC_PM_WAKE_SDIO_IRQ;
	} else
		device_init_wakeup(&pdev->dev, 0);

	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id sdhci_pxav3_of_match[] = {
	{
		.compatible = "mrvl,pxav3-mmc",
	},
	{},
};
MODULE_DEVICE_TABLE(of, sdhci_pxav3_of_match);

static void pxav3_get_of_perperty(struct device *dev,
		struct sdhci_pxa_platdata *pdata)
{
	struct device_node *np = dev->of_node;
	u32 tmp;
	struct property *prop;
	const __be32 *p;
	u32 val, timing;
	int i = 0;
	struct sdhci_pxa_dtr_data *dtr_data;

	if (of_property_read_bool(np, "marvell,sdh-pm-runtime-en"))
		pdata->flags |= PXA_FLAG_EN_PM_RUNTIME;

	if (!of_property_read_u32(np, "marvell,sdh-flags", &tmp))
		pdata->flags |= tmp;

	of_property_read_u32(np, "mrvl,max-speed", &pdata->max_speed);

	if (!of_property_read_u32(np, "marvell,sdh-host-caps", &tmp))
		pdata->host_caps |= tmp;
	if (!of_property_read_u32(np, "marvell,sdh-host-caps2", &tmp))
		pdata->host_caps2 |= tmp;
	if (!of_property_read_u32(np, "marvell,sdh-host-caps-disable", &tmp))
		pdata->host_caps_disable |= tmp;
	if (!of_property_read_u32(np, "marvell,sdh-host-caps2-disable", &tmp))
		pdata->host_caps2_disable |= tmp;
	if (!of_property_read_u32(np, "marvell,sdh-quirks", &tmp))
		pdata->quirks |= tmp;
	if (!of_property_read_u32(np, "marvell,sdh-quirks2", &tmp))
		pdata->quirks2 |= tmp;
	if (!of_property_read_u32(np, "marvell,sdh-pm-caps", &tmp))
		pdata->pm_caps |= tmp;
	if (!of_property_read_u32(np, "lpm-qos", &tmp))
		pdata->lpm_qos = tmp;
	else
		pdata->lpm_qos = PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE;
	/*
	 * property "marvell,sdh-dtr-data": <timing preset_rate src_rate tx_delay rx_delay>, [<..>]
	 * allow to set clock related parameters.
	 */
	if (of_property_read_bool(np, "marvell,sdh-dtr-data")) {
		dtr_data = devm_kzalloc(dev,
				MMC_TIMING_MAX * sizeof(struct sdhci_pxa_dtr_data),
				GFP_KERNEL);
		if (!dtr_data) {
			dev_err(dev, "failed to allocate memory for sdh-dtr-data\n");
			return;
		}
		of_property_for_each_u32(np, "marvell,sdh-dtr-data", prop, p, timing) {
			if (timing > MMC_TIMING_MAX) {
				dev_err(dev, "invalid timing %d on sdh-dtr-data prop\n",
						timing);
				continue;
			} else {
				dtr_data[i].timing = timing;
			}
			p = of_prop_next_u32(prop, p, &val);
			if (!p) {
				dev_err(dev, "missing preset_rate for timing %d\n",
						timing);
			} else {
				dtr_data[i].preset_rate = val;
			}
			p = of_prop_next_u32(prop, p, &val);
			if (!p) {
				dev_err(dev, "missing src_rate for timing %d\n",
						timing);
			} else {
				dtr_data[i].src_rate = val;
			}
			p = of_prop_next_u32(prop, p, &val);
			if (!p) {
				dev_err(dev, "missing tx_delay for timing %d\n",
						timing);
			} else {
				dtr_data[i].tx_delay = val;
			}
			p = of_prop_next_u32(prop, p, &val);
			if (!p) {
				dev_err(dev, "missing rx_delay for timing %d\n",
						timing);
			} else {
				dtr_data[i].rx_delay = val;
			}
			if (timing == MMC_TIMING_MAX)
				pdata->dtr_data = dtr_data;
			else
				i++;
		}
	}
}
#endif

static int sdhci_pxav3_probe(struct platform_device *pdev)
{
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_pxa_platdata *pdata = pdev->dev.platform_data;
	struct device *dev = &pdev->dev;
	struct sdhci_host *host = NULL;
	struct sdhci_pxa *pxa = NULL;
	const struct of_device_id *match;

	int ret = 0;
	struct clk *clk;

	pxa = kzalloc(sizeof(struct sdhci_pxa), GFP_KERNEL);
	if (!pxa)
		return -ENOMEM;

	host = sdhci_pltfm_init(pdev, &sdhci_pxav3_pdata);
	if (IS_ERR(host)) {
		kfree(pxa);
		return PTR_ERR(host);
	}
	pltfm_host = sdhci_priv(host);
	pltfm_host->priv = pxa;

	clk = clk_get(dev, NULL);
	if (IS_ERR(clk)) {
		dev_err(dev, "failed to get io clock\n");
		ret = PTR_ERR(clk);
		goto err_clk_get;
	}
	pltfm_host->clk = clk;
	clk_prepare_enable(clk);

	host->quirks2 = SDHCI_QUIRK2_TIMEOUT_DIVIDE_4
		| SDHCI_QUIRK2_NO_CURRENT_LIMIT
		| SDHCI_QUIRK2_PRESET_VALUE_BROKEN;

	match = of_match_device(of_match_ptr(sdhci_pxav3_of_match), &pdev->dev);
	if (match) {
		mmc_of_parse(host->mmc);
		sdhci_get_of_property(pdev);
		if (!pdata) {
			pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
			if (!pdata) {
				dev_err(mmc_dev(host->mmc),
					"failed to alloc pdata\n");
				goto err_init_host;
			}
			pdev->dev.platform_data = pdata;
		}
		pxav3_get_of_perperty(dev, pdata);
	}
	if (pdata) {
		pdata->pinctrl = devm_pinctrl_get(dev);
		if (IS_ERR(pdata->pinctrl))
			dev_err(dev, "could not get pinctrl handle\n");
		pdata->pin_slow = pinctrl_lookup_state(pdata->pinctrl, "default");
		if (IS_ERR(pdata->pin_slow))
			dev_err(dev, "could not get default pinstate\n");
		pdata->pin_fast = pinctrl_lookup_state(pdata->pinctrl, "fast");
		if (IS_ERR(pdata->pin_fast))
			dev_info(dev, "could not get fast pinstate\n");

		ret = pxav3_init_host_with_pdata(host, pdata);
		if (ret) {
			dev_err(mmc_dev(host->mmc),
					"failed to init host with pdata\n");
			goto err_init_host;
		}
		pm_qos_add_request(&pdata->qos_idle, PM_QOS_CPUIDLE_BLOCK,
			PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE);
		if (pdata->flags & PXA_FLAG_EN_PM_RUNTIME) {
			pm_runtime_get_noresume(&pdev->dev);
			pm_runtime_set_active(&pdev->dev);
			pm_runtime_set_autosuspend_delay(&pdev->dev,
				PXAV3_RPM_DELAY_MS);
			pm_runtime_use_autosuspend(&pdev->dev);
			pm_suspend_ignore_children(&pdev->dev, 1);
			pm_runtime_enable(&pdev->dev);
		}
	}

	/* dma only 32 bit now */
	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;

	ret = sdhci_add_host(host);
	if (ret) {
		dev_err(&pdev->dev, "failed to add host\n");
		goto err_add_host;
	}

	/* remove the caps that supported by the controller but not available
	 * for certain platforms.
	 */
	if (pdata && pdata->host_caps_disable)
		host->mmc->caps &= ~(pdata->host_caps_disable);

	platform_set_drvdata(pdev, host);

	if (host->mmc->pm_caps & MMC_PM_KEEP_POWER) {
		device_init_wakeup(&pdev->dev, 1);
		host->mmc->pm_flags |= MMC_PM_WAKE_SDIO_IRQ;
	} else {
		device_init_wakeup(&pdev->dev, 0);
	}
	if (pdata && pdata->flags & PXA_FLAG_EN_PM_RUNTIME)
		pm_runtime_put_autosuspend(&pdev->dev);

#ifdef CONFIG_SD8XXX_RFKILL
	if (pdata && pdata->pmmc)
		*pdata->pmmc = host->mmc;
#endif

	pxa->tx_delay.store = tx_delay_store;
	pxa->tx_delay.show = tx_delay_show;
	sysfs_attr_init(&pxa->tx_delay.attr);
	pxa->tx_delay.attr.name = "tx_delay";
	pxa->tx_delay.attr.mode = S_IRUGO | S_IWUSR;
	ret = device_create_file(&host->mmc->class_dev, &pxa->tx_delay);

	pxa->rx_delay.store = rx_delay_store;
	pxa->rx_delay.show = rx_delay_show;
	sysfs_attr_init(&pxa->rx_delay.attr);
	pxa->rx_delay.attr.name = "rx_delay";
	pxa->rx_delay.attr.mode = S_IRUGO | S_IWUSR;
	ret = device_create_file(&host->mmc->class_dev, &pxa->rx_delay);


	return 0;

err_add_host:
	if (pdata && pdata->flags & PXA_FLAG_EN_PM_RUNTIME) {
		pm_runtime_put_noidle(&pdev->dev);
		pm_runtime_set_suspended(&pdev->dev);
		pm_runtime_disable(&pdev->dev);
	}
err_init_host:
	clk_disable_unprepare(clk);
	clk_put(clk);
	if (pdata)
		pm_qos_remove_request(&pdata->qos_idle);
err_clk_get:
	sdhci_pltfm_free(pdev);
	kfree(pxa);
	return ret;
}

static int sdhci_pxav3_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_pxa *pxa = pltfm_host->priv;
	struct sdhci_pxa_platdata *pdata = pdev->dev.platform_data;

	pm_runtime_get_sync(&pdev->dev);
	sdhci_remove_host(host, 1);
	pm_runtime_disable(&pdev->dev);

	if (pdata)
		pm_qos_remove_request(&pdata->qos_idle);

	clk_disable_unprepare(pltfm_host->clk);
	clk_put(pltfm_host->clk);

	sdhci_pltfm_free(pdev);
	kfree(pxa);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int sdhci_pxav3_suspend(struct device *dev)
{
	int ret;
	struct sdhci_host *host = dev_get_drvdata(dev);

	pm_runtime_get_sync(dev);
	ret = sdhci_suspend_host(host);
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return ret;
}

static int sdhci_pxav3_resume(struct device *dev)
{
	int ret;
	struct sdhci_host *host = dev_get_drvdata(dev);

	pm_runtime_get_sync(dev);
	ret = sdhci_resume_host(host);
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return ret;
}
#endif

#ifdef CONFIG_PM_RUNTIME
static int sdhci_pxav3_runtime_suspend(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	unsigned long flags;

	if (pltfm_host->clk) {
		spin_lock_irqsave(&host->lock, flags);
		host->runtime_suspended = true;
		spin_unlock_irqrestore(&host->lock, flags);

		clk_disable_unprepare(pltfm_host->clk);
	}

	return 0;
}

static int sdhci_pxav3_runtime_resume(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	unsigned long flags;

	if (pltfm_host->clk) {
		clk_prepare_enable(pltfm_host->clk);

		spin_lock_irqsave(&host->lock, flags);
		host->runtime_suspended = false;
		spin_unlock_irqrestore(&host->lock, flags);
	}

	return 0;
}
#endif

#ifdef CONFIG_PM
static const struct dev_pm_ops sdhci_pxav3_pmops = {
	SET_SYSTEM_SLEEP_PM_OPS(sdhci_pxav3_suspend, sdhci_pxav3_resume)
	SET_RUNTIME_PM_OPS(sdhci_pxav3_runtime_suspend,
		sdhci_pxav3_runtime_resume, NULL)
};

#define SDHCI_PXAV3_PMOPS (&sdhci_pxav3_pmops)

#else
#define SDHCI_PXAV3_PMOPS NULL
#endif

static struct platform_driver sdhci_pxav3_driver = {
	.driver		= {
		.name	= "sdhci-pxav3",
#ifdef CONFIG_OF
		.of_match_table = sdhci_pxav3_of_match,
#endif
		.owner	= THIS_MODULE,
		.pm	= SDHCI_PXAV3_PMOPS,
	},
	.probe		= sdhci_pxav3_probe,
	.remove		= sdhci_pxav3_remove,
};

module_platform_driver(sdhci_pxav3_driver);

MODULE_DESCRIPTION("SDHCI driver for pxav3");
MODULE_AUTHOR("Marvell International Ltd.");
MODULE_LICENSE("GPL v2");

