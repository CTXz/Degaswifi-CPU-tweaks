/*
 * Base driver for Marvell MAP
 *
 * Copyright (C) 2014 Marvell International Ltd.
 * Nenghua Cao <nhcao@marvell.com>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/mfd/core.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/mfd/mmp-map.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/io.h>
#include "mmp_map_pram.h"

#define DRV_NAME      "mmp-map"
#define DRIVER_VERSION  "1.00"
#define DRIVER_RELEASE_DATE     "Jun.17 2013"

/* tdm platform data */
static struct tdm_platform_data tdm_pdata = {
	.use_4_wires = 1,
	.slot_size = 20,
	.slot_space = 1,
	.start_slot = 0,
	.fsyn_pulse_width = 20,
};

static struct mfd_cell sub_devs[] = {
	{
		.name = "mmp-map-be",
		.id = -1,
	},
	{
		.name = "mmp-map-be-tdm",
		.platform_data = &tdm_pdata,
		.pdata_size = sizeof(struct tdm_platform_data),
		.id = -1,
	},
	{
		.name = "mmp-map-codec",
		.id = -1,
	},
};

static struct regmap_config mmp_map_regmap_config = {
	.name = "mmp-map",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = MAP_DAC_ANA_MISC,
	.cache_type = REGCACHE_RBTREE,
};

/* set port frame clock frequence */
void map_set_port_freq(struct map_private *map_priv, enum mmp_map_port port,
						unsigned int rate)
{
	struct regmap *regmap = map_priv->regmap;
	unsigned int val = 0, reg;
	unsigned int mask = 0;

	/* sample rate */
	switch (rate) {
	case 8000:
		val = MAP_SAMPLE_RATE_8000;
		break;
	case 11025:
		val = MAP_SAMPLE_RATE_11025;
		break;
	case 16000:
		val = MAP_SAMPLE_RATE_16000;
		break;
	case 22050:
		val = MAP_SAMPLE_RATE_22050;
		break;
	case 32000:
		val = MAP_SAMPLE_RATE_32000;
		break;
	case 44100:
		val = MAP_SAMPLE_RATE_44100;
		break;
	case 48000:
		val = MAP_SAMPLE_RATE_48000;
		break;
	default:
		return;
	}

	reg = MAP_LRCLK_RATE_REG;
	val = val << ((I2S_OUT - port) * 0x4);
	mask = 0xf << ((I2S_OUT - port) * 0x4);
	regmap_update_bits(regmap, reg, mask, val);
	return;
}
EXPORT_SYMBOL(map_set_port_freq);

void map_reset_port(struct map_private *map_priv, enum mmp_map_port port)
{
	struct regmap *regmap = map_priv->regmap;
	unsigned int reg, val = 0;
	unsigned int mask = 0;

	switch (port) {
	case I2S1:
		/* reset i2s1 interface(audio) */
		reg = MAP_TOP_CTRL_REG_1;
		val = I2S1_RESET | ASRC1_RESET;
		mask = I2S1_RESET | ASRC1_RESET;
		regmap_update_bits(regmap, reg, mask, val);

		/* out of reset */
		val &= ~(I2S1_RESET | ASRC1_RESET);
		regmap_update_bits(regmap, reg, mask, val);
		break;
	case I2S2:
		/* reset i2s2 interface(audio) */
		reg = MAP_TOP_CTRL_REG_1;
		val = I2S2_RESET | ASRC2_RESET;
		mask = I2S2_RESET | ASRC2_RESET;
		regmap_update_bits(regmap, reg, mask, val);

		/* out of reset */
		val &= ~(I2S2_RESET | ASRC2_RESET);
		regmap_update_bits(regmap, reg, mask, val);
		break;
	case I2S3:
		/* reset i2s3 interface(audio) */
		reg = MAP_TOP_CTRL_REG_1;
		val = I2S3_RESET | ASRC3_RESET;
		mask = I2S3_RESET | ASRC3_RESET;
		regmap_update_bits(regmap, reg, mask, val);

		/* out of reset */
		val &= ~(I2S3_RESET | ASRC3_RESET);
		regmap_update_bits(regmap, reg, mask, val);
		break;
	case I2S4:
		/* reset i2s4 interface (hifi) */
		reg = MAP_TOP_CTRL_REG_1;
		val = I2S4_RESET | ASRC4_RESET;
		mask = I2S4_RESET | ASRC4_RESET;
		regmap_update_bits(regmap, reg, mask, val);

		/* out of reset */
		val &= ~(I2S4_RESET | ASRC4_RESET);
		regmap_update_bits(regmap, reg, mask, val);
		break;
	case I2S_OUT:
		/* reset dei2s interface */
		reg = MAP_TOP_CTRL_REG_1;
		val = I2S_OUT_RESET;
		mask = I2S_OUT_RESET;
		regmap_update_bits(regmap, reg, mask, val);

		/* out of reset */
		val &= ~I2S_OUT_RESET;
		regmap_update_bits(regmap, reg, mask, val);
		break;
	default:
		return;
	}

	return;
}
EXPORT_SYMBOL(map_reset_port);

/* apply the change */
void map_apply_change(struct map_private *map_priv)
{
	struct regmap *regmap = map_priv->regmap;
	unsigned int val, reg;

	reg = MAP_DAC_ANA_MISC;
	val = APPLY_CHANGES;
	regmap_update_bits(regmap, reg, val, val);
	return;
}
EXPORT_SYMBOL(map_apply_change);

int map_raw_write(struct map_private *map_priv, unsigned int reg,
					unsigned int value)
{
	writel_relaxed(value, map_priv->regs_map + reg);
	return 0;
}
EXPORT_SYMBOL(map_raw_write);

unsigned int map_raw_read(struct map_private *map_priv,
					unsigned int reg)
{
	unsigned int value;

	value = readl_relaxed(map_priv->regs_map + reg);
	return value;
}
EXPORT_SYMBOL(map_raw_read);

void map_load_dsp_pram(struct map_private *map_priv)
{
	int i;
	u32 off;

	dev_info(map_priv->dev, "Loading MAP DSP %lu prams, please wait...\r\n",
					ARRAY_SIZE(map_pram));
	for (i = 0; i < ARRAY_SIZE(map_pram); i++) {
		off = map_pram[i].addr;
		writel_relaxed(map_pram[i].val, map_priv->regs_map + off);
	}
}

static int device_map_init(struct map_private *map_priv)
{
	int ret = 0;
	unsigned int reg, val;
	struct regmap *regmap = map_priv->regmap;

	if (!regmap) {
		dev_err(map_priv->dev, "regmap is invalid\n");
		return -EINVAL;
	}
	/* debug clock on real silicon */
#if 0
	clk_prepare_enable(map_priv->audio_clk);
	clk_prepare_enable(map_priv->apb_clk);
	clk_set_rate(map_priv->map_apb_clk, 0x0);
#endif
	ret = regmap_read(regmap, MMP_MAP_REV, &val);
	if (ret < 0) {
		dev_err(map_priv->dev, "Failed to read MAP REV: %d\n", ret);
		return ret;
	}
	map_priv->id = val;

	/* configure pad */
	val = readl_relaxed(map_priv->regs_aux + DSP_MAP_CONF_REG);
	val &=  ~MAP_RESET;
	writel_relaxed(val, map_priv->regs_aux + DSP_MAP_CONF_REG);
	val = readl_relaxed(map_priv->regs_aux + DSP_MAP_CONF_REG);
	val |=  MAP_RESET;
	writel_relaxed(val, map_priv->regs_aux + DSP_MAP_CONF_REG);

#ifndef CONFIG_MAP_BYPASS
	val = readl_relaxed(map_priv->regs_aux + DSP_MAP_CONF_REG);
	val &= ~(SSPA1_PAD_OUT_SRC_MASK | SSPA2_PAD_OUT_SRC_MASK);
	val |= TDM_DRIVE_SSPA1_PAD;
	val |= MAP_I2S4_DRIVE_SSPA2_PAD;
	val &= ~AUD_SSPA1_INPUT_SRC;
	val &= ~AUD_SSPA2_INPUT_SRC;
	writel_relaxed(val, map_priv->regs_aux + DSP_MAP_CONF_REG);
#endif

	/* reset map */
	reg = MAP_TOP_CTRL_REG_1;
	val = 0xfff10;
	regmap_write(regmap, reg, val);
	val = 0x3;
	regmap_write(regmap, reg, val);

	reg = MAP_ASRC_CTRL_REG;
	val = 0xff;
	regmap_write(regmap, reg, val);

	/* Load MAP DSP firmware */
	reg = MAP_TOP_CTRL_REG_2;
	regmap_read(regmap, reg, &val);
	val &= (1 << 6); /* choose apb pclk */
	regmap_write(regmap, reg, val);
	map_load_dsp_pram(map_priv);
	val |= (1 << 6); /* choose dig_clk */
	regmap_write(regmap, reg, val);


	ret = mfd_add_devices(map_priv->dev, 0, &sub_devs[0],
			      ARRAY_SIZE(sub_devs), 0, 0, NULL);
	if (ret < 0) {
		dev_err(map_priv->dev, "Failed to add sub_devs\n");
		return ret;
	} else
		dev_info(map_priv->dev, "[%s]:Added mfd sub_devs\n", __func__);

	/*
	 * Fixme: close clock when
	 * there is no any activity on MAP
	 */
#if 0
	clk_disable_unprepare(map_priv->apb_clk);
	clk_disable_unprepare(map_priv->audio_clk);
#endif

	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id mmp_map_of_match[] = {
	{ .compatible = "marvell,mmp-map"},
	{},
};
#endif

static int mmp_map_probe(struct platform_device *pdev)
{
	struct map_private *map_priv;
	struct resource *res0, *res1, *region;
	int ret = 0;

	map_priv = devm_kzalloc(&pdev->dev,
		sizeof(struct map_private), GFP_KERNEL);
	if (map_priv == NULL)
		return -ENOMEM;

	/* MAP AUX register area */
	res0 = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res0 == NULL) {
		dev_err(map_priv->dev, "resource for %s not exist\n",
				pdev->name);
		return -ENXIO;
	}

	/* map MAP register area */
	res1 = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res1 == NULL) {
		dev_err(map_priv->dev, "resource for %s not exist\n",
				pdev->name);
		return -ENXIO;
	}

	region = devm_request_mem_region(&pdev->dev, res0->start,
					 resource_size(res0), DRV_NAME);
	if (!region) {
		dev_err(&pdev->dev, "request region aux failed\n");
		return -EBUSY;
	}

	map_priv->regs_aux = devm_ioremap(&pdev->dev, res0->start,
				  resource_size(res0));
	if (!map_priv->regs_aux) {
		dev_err(&pdev->dev, "ioremap aux failed\n");
		return -ENOMEM;
	}

	region = devm_request_mem_region(&pdev->dev, res1->start,
					 resource_size(res1), DRV_NAME);
	if (!region) {
		dev_err(&pdev->dev, "request region map failed\n");
		return -EBUSY;
	}

	map_priv->regs_map = devm_ioremap(&pdev->dev, res1->start,
				  resource_size(res1));
	if (!map_priv->regs_map) {
		dev_err(&pdev->dev, "ioremap map failed\n");
		return -ENOMEM;
	}

	map_priv->dev = &pdev->dev;
	map_priv->regmap = devm_regmap_init_mmio(&pdev->dev, map_priv->regs_map,
					&mmp_map_regmap_config);
	if (IS_ERR(map_priv->regmap)) {
		ret = PTR_ERR(map_priv->regmap);
		dev_err(map_priv->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	map_priv->audio_clk = devm_clk_get(&pdev->dev, "apll1_slow");
	if (IS_ERR(map_priv->audio_clk)) {
		ret = PTR_ERR(map_priv->audio_clk);
		return ret;
	}

	map_priv->apb_clk = devm_clk_get(&pdev->dev, "mmp-apb");
	if (IS_ERR(map_priv->apb_clk)) {
		ret = PTR_ERR(map_priv->apb_clk);
		return ret;
	}
	map_priv->map_apb_clk = devm_clk_get(&pdev->dev, "mmp-map-apb");
	if (IS_ERR(map_priv->map_apb_clk)) {
		ret = PTR_ERR(map_priv->map_apb_clk);
		return ret;
	}

	/* Fixme: do we need to bypass the cache in regmap? */
	/* regcache_cache_bypass(map_priv->regmap, true); */
	platform_set_drvdata(pdev, map_priv);

	ret = device_map_init(map_priv);
	if (ret) {
		dev_err(map_priv->dev, "%s id 0x%x failed!\n",
				__func__, map_priv->id);
	}
	return ret;
}

static int mmp_map_remove(struct platform_device *pdev)
{
	struct map_private *map_priv;

	map_priv = platform_get_drvdata(pdev);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver mmp_map_driver = {
	.probe		= mmp_map_probe,
	.remove		= mmp_map_remove,
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(mmp_map_of_match),
#endif
	},
};

module_platform_driver(mmp_map_driver);

MODULE_DESCRIPTION("Marvell MAP MFD driver");
MODULE_AUTHOR("Nenghua Cao <nhcao@marvell.com>");
MODULE_LICENSE("GPL");
