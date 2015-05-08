/*
 * pxa-geu driver for generic encrypt unit
 *
 * Copyright (c) [2009-2013] Marvell International Ltd. and its affiliates.
 * All rights reserved.
 * This software file (the "File") is owned and distributed by Marvell
 * International Ltd. and/or its affiliates ("Marvell") under the following
 * licensing terms.
 * If you received this File from Marvell, you may opt to use, redistribute
 * and/or modify this File in accordance with the terms and conditions of
 * the General Public License Version 2, June 1991 (the "GPL License"), a
 * copy of which is available along with the File in the license.txt file
 * or by writing to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA 02111-1307 or on the worldwide web at
 * http://www.gnu.org/licenses/gpl.txt. THE FILE IS DISTRIBUTED AS-IS,
 * WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
 * DISCLAIMED. The GPL License provides additional details about this
 * warranty disclaimer.
 *
 */

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/uio.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/fb.h>
#include <linux/pagemap.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/scatterlist.h>
#include <linux/random.h>
#include <linux/miscdevice.h>
#include <linux/pm_qos.h>
#include <linux/mman.h>
#include <linux/mm.h>
#include <linux/highmem.h>
#include <mach/cputype.h>
#include <mach/dma.h>

#include <mach/regs-apmu.h>
#include <mach/irqs.h>

#include "pxa-geu.h"

/* #define GEU_DEBUG_INFO */
#ifdef GEU_DEBUG_INFO
#define GEU_DEBUG(x) \
		do {		\
				x;	\
		} while (0)
#else
#define GEU_DEBUG(x)
#endif

#define GEU_MODE_DMA

#define GEU_DRIVER_VERSION	"GEU drvier 0.1.3"
#define MMP_GEU_DRV_NAME "geu"

#define PXA_GEU_IOMEM_SIZE	0x00001000

/* max transfer pages in one dma command */
#define GEU_DMA_PAGES		512
#define GEU_DMA_SIZE		(2*GEU_DMA_PAGES*sizeof(pxa_dma_desc))

/* DMA threshold 2k: must < 8k &&  must < GEU_DMA_SIZE */
#define GEU_DMA_THRESHOLD	0x800
#define GEU_DMA_TIMEOUT		(msecs_to_jiffies(1000))	/*1s in msec */

/* GEU DMA require 16bytes aligned address */
#define GEU_DMA_ALIGN(x)	((unsigned long)(x) & 0xf)
#define DMA_PHY_ADDR(geu, x)	\
		(((struct geu_private *)(geu))->dma_phy_base + \
		((unsigned int)(x) - \
		 (unsigned int)(((struct geu_private *)(geu))->dma_buf_base)))

#define GET_PAGE_NUM(a, l)	\
	(1+(((((unsigned long)(a)+(l)-1) & PAGE_MASK) - \
	((unsigned long)(a) & PAGE_MASK)) >> PAGE_SHIFT))

#define GEU_IN_DRCMR		68	/* GEU DRCMR 0x1110 */
#define GEU_OUT_DRCMR		69	/* GEU DRCMR 0x1114 */

#define GEU_IOCTL_MAGIC 'g'
#define GEU_AES_INIT		_IOW(GEU_IOCTL_MAGIC, 1, unsigned int)
#define GEU_AES_PROCESS		_IOW(GEU_IOCTL_MAGIC, 2, unsigned int)
#define GEU_AES_FINISH		_IOW(GEU_IOCTL_MAGIC, 3, unsigned int)
#define GEU_AES_INIT_RKEK	_IOW(GEU_IOCTL_MAGIC, 4, unsigned int)
#define GEU_AES_INIT_CBC	_IOW(GEU_IOCTL_MAGIC, 5, unsigned int)
#define GEU_GEN_RAND		_IOW(GEU_IOCTL_MAGIC, 6, unsigned int)
#define GEU_READ_OEMHASHKEY _IOW(GEU_IOCTL_MAGIC, 7, unsigned int)
#define GEU_READ_LIFECYCLE	_IOW(GEU_IOCTL_MAGIC, 8, unsigned int)

#define GEU_SHA160_SIZE		20
#define GEU_SHA224_SIZE		28
#define GEU_SHA256_SIZE		32

struct dma_info {
	struct page *pages[GEU_DMA_PAGES];
	dma_addr_t addr[GEU_DMA_PAGES];
	unsigned int size[GEU_DMA_PAGES];
	unsigned int num;
};

struct geu_arg {
	int arg0;
	int arg1;
	int arg2;
	int arg3;
};

struct geu_private
{
	struct miscdevice pxa_geu_miscdev;
	void __iomem *geu_iobase;
	pid_t geu_owner;
	unsigned char *dma_buf_base;
	unsigned int dma_phy_base;
	struct clk *geu_clk;
	int geu_io_mem;
	int geu_io_len;
	int geu_irq;
	int geu_lpm;
	struct device *p_device;
	struct mutex geu_lock;
	struct completion cmd_complete;
#ifdef GEU_MODE_DMA
	struct pm_qos_request geu_qos_idle;

	struct dma_chan *geu_dma_in;
	struct dma_chan *geu_dma_out;
	struct dma_async_tx_descriptor *geu_dma_in_desc;
	struct dma_async_tx_descriptor *geu_dma_out_desc;
	dma_cookie_t geu_dma_in_cookie;
	dma_cookie_t geu_dma_out_cookie;
	struct dma_info *geu_dma_in_info;
	struct dma_info *geu_dma_out_info;
#endif
};
static void pxa_geu_dma_irq_in(void *data);
static void pxa_geu_dma_irq_out(void *data);

static inline void GEUWriteReg(
		struct geu_private *geu,
		unsigned int off, unsigned int value)
{
	if (geu)
		writel(value, geu->geu_iobase + off);
}

static inline unsigned int GEUReadReg(
		struct geu_private *geu,
		unsigned int off)
{
	if (geu)
		return readl(geu->geu_iobase + off);
	return 0;
}

static int GEU_AES_init(
		struct geu_private *geu,
		unsigned int *key,
		int keylen, int encrypt)
{
	int i;
	unsigned int conf;
	struct device *dev = geu->p_device;

	if (mutex_trylock(&(geu->geu_lock)) == 0) {
		dev_err(dev, "GEU busy. Engine has been initialized\n");
		return -EBUSY;
	}
	clk_enable(geu->geu_clk);
	geu->geu_owner = current->tgid;

	conf = GEUReadReg(geu, GEU_CONFIG);
	conf &= ~GEU_CONFIG_KEYSZ_MASK;
	/* 128 to 1, 192 to 2, 256 to 3 */
	conf |= ((keylen >> 3) - 1) << GEU_CONFIG_KEYSZ_SHIFT;
	/* set encrypt/decrypt */
	if (encrypt != 0)
		conf &= ~GEU_CONFIG_ENCDEC;
	else
		conf |= GEU_CONFIG_ENCDEC;

	conf |= GEU_CONFIG_KEYIMR;
	/* conf |= GEU_CONFIG_RNDKEYHA|GEU_CONFIG_SBOXHA; */
	GEUWriteReg(geu, GEU_CONFIG, conf);

	/* write user key */
	for (i = 0; i < 8; i++)
		GEUWriteReg(geu, GEU_INIT_KEY_VALUE + (i * 4), key[i]);

	GEUWriteReg(geu, GEU_STATUS, GEU_STATUS_KEYREADY);

	/* wait for complete, timeout = 1s */
	if (0 == wait_for_completion_timeout(&(geu->cmd_complete),
			GEU_DMA_TIMEOUT)) {
		dev_err(dev, "GEU init AES engine timeout. status(%x)",
				GEUReadReg(geu, GEU_STATUS));
		clk_disable(geu->geu_clk);
		geu->geu_owner = 0;
		mutex_unlock(&(geu->geu_lock));
		return -ETIME;
	}
	GEU_DEBUG(dev_dbg(dev,
			"##geu status(%x) config(%x)\n",
			GEUReadReg(geu, GEU_STATUS),
			GEUReadReg(geu, GEU_CONFIG)));
	/* dump_round_key(); */
	return 0;
}

#ifdef GEU_MODE_DMA
/*  */
static int GEU_AES_direct(
		struct geu_private *geu,
		unsigned char *in, unsigned char *out,
		unsigned int length)
{
	int ret = 0;
	unsigned int conf;
	struct dma_slave_config slave_config_in;
	struct dma_slave_config slave_config_out;
	struct device *dev = geu->p_device;

	conf = GEUReadReg(geu, GEU_CONFIG);
	while (length > 0) {
		unsigned int len =
			length > GEU_DMA_THRESHOLD ? GEU_DMA_THRESHOLD : length;
		if (copy_from_user(geu->dma_buf_base, in, len)) {
			ret = -EFAULT;
			break;
		}
		/* DMA to GEU Hardware */
		slave_config_in.direction = DMA_MEM_TO_DEV;
		slave_config_in.dst_addr = (dma_addr_t)
			(geu->geu_io_mem + GEU_INPUT_DATA_ENC_DEC);
		slave_config_in.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		slave_config_in.dst_maxburst = 16;
		slave_config_in.slave_id = 0; /* dma driver will set this value */

		ret = dmaengine_slave_config(geu->geu_dma_in, &slave_config_in);
		if (ret < 0) {
			dev_err(dev,
				"dmaengine_slave_config err. ret = 0x%x\n",
				ret);
			return -EFAULT;
		}
		geu->geu_dma_in_desc =
			dmaengine_prep_slave_single(
					geu->geu_dma_in,
					/* src addr */
					(dma_addr_t)(geu->dma_phy_base),
					len,
					DMA_MEM_TO_DEV,
					0);
		if (NULL == geu->geu_dma_in_desc) {
			dev_err(dev,
				"geu_dma_in_desc dmaengine_prep_slave_single error\n");
			return -EFAULT;
		}
		geu->geu_dma_in_desc->callback = pxa_geu_dma_irq_in;
		geu->geu_dma_in_desc->callback_param = NULL;
		/* GEU Hardware to DMA */
		slave_config_out.direction = DMA_DEV_TO_MEM;
		slave_config_out.src_addr = (dma_addr_t)
			(geu->geu_io_mem + GEU_OUT_DATA_AFTER_ENC_DEC);
		slave_config_out.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		slave_config_out.src_maxburst = 16;
		slave_config_out.slave_id = 0; /* dma driver will set this value */

		ret = dmaengine_slave_config(geu->geu_dma_out, &slave_config_out);
		if (ret < 0) {
			dev_err(dev,
				"dmaengine_slave_config err. ret = 0x%x\n",
				ret);
			return -EFAULT;
		}
		geu->geu_dma_out_desc =
			dmaengine_prep_slave_single(
				geu->geu_dma_out,
				(dma_addr_t)(geu->dma_phy_base),
				len,
				DMA_DEV_TO_MEM,
				0);
		if (NULL == geu->geu_dma_out_desc) {
			dev_err(dev,
				"geu_dma_out_desc dmaengine_prep_slave_single error\n");
			return -EFAULT;
		}
		geu->geu_dma_out_desc->callback = pxa_geu_dma_irq_out;
		geu->geu_dma_out_desc->callback_param = geu;
		geu->geu_dma_out_cookie = dmaengine_submit(geu->geu_dma_out_desc);
		dma_async_issue_pending(geu->geu_dma_out);
		geu->geu_dma_in_cookie = dmaengine_submit(geu->geu_dma_in_desc);
		dma_async_issue_pending(geu->geu_dma_in);
		GEUWriteReg(geu, GEU_CONFIG,
				conf | GEU_CONFIG_EN_DMA_MODE_AES_CIPHER);
		/* wait for complete */
		if (0 ==
			wait_for_completion_timeout(&(geu->cmd_complete),
						GEU_DMA_TIMEOUT)) {
			dev_err(dev, "GEU AES direct timeout statu(%x)\n",
					GEUReadReg(geu, GEU_STATUS));
			ret = -ETIME;
			break;
		}
		if (copy_to_user(out, geu->dma_buf_base, len)) {
			ret = -EFAULT;
			break;
		}
		in += len;
		out += len;
		length -= len;
	}

	GEUWriteReg(geu, GEU_CONFIG, conf);
	return ret;
}

#if 0
static int GEU_AES_chain(
		struct geu_private *geu,
		unsigned char *in, unsigned char *out,
		unsigned int length)
{

	int ret = 0;
	unsigned int i;
	unsigned int conf;
	struct dma_info *info;
	struct device *dev = geu->p_device;

	conf = GEUReadReg(geu, GEU_CONFIG);

	/*
	   for VIVT cache, we should flush user cache.
	   pxa_dma_map only flush kernel & L2 cache.
	*/
	__cpuc_flush_user_range((unsigned long)in, length, 0);
	while (length > 0) {
		/*
		   setup transfer length.
		   reserve 3 pages for unaligned address
		*/
		unsigned int len =
		    length >\
		    ((GEU_DMA_PAGES - 3) * PAGE_SIZE) ?\
		    ((GEU_DMA_PAGES - 3) * PAGE_SIZE) : length;
		if (in == out)
			ret =
			    pxa_dma_map(geu_dma_in_info, in, len,
					DMA_BIDIRECTIONAL);
		else {
			ret =
			    pxa_dma_map(geu_dma_in_info, in, len,
					DMA_TO_DEVICE);
			if (ret == 0) {
				ret =
				    pxa_dma_map(geu_dma_out_info, out, len,
						DMA_FROM_DEVICE);
				if (ret != 0) {
					pxa_dma_unmap(geu_dma_in_info, in, len,
						      DMA_TO_DEVICE);
				}
			}
		}
		if (ret != 0) {
			printk(KERN_ERR \
			"DMA map addr fail %p -> %p size 0x%x len 0x%x\n", \
			in, out, length, len);
			break;
		}

		/* setup input DMA desc */
		for (i = 0; i < geu_dma_in_info->num; i++) {
			geu_dma_in_desc[i].ddadr =
			    DMA_PHY_ADDR(geu, &geu_dma_in_desc[i + 1]);
			geu_dma_in_desc[i].dsadr = geu_dma_in_info->addr[i];
			geu_dma_in_desc[i].dtadr =
			    geu->geu_io_mem + GEU_INPUT_DATA_ENC_DEC;
			geu_dma_in_desc[i].dcmd =
			    DCMD_INCSRCADDR | DCMD_FLOWTRG | DCMD_WIDTH4 |
			    DCMD_BURST16 | geu_dma_in_info->size[i];
		}
		/* mark term */
		geu_dma_in_desc[i].ddadr = DDADR_STOP;
		geu_dma_in_desc[i].dcmd =
		    DCMD_FLOWTRG | DCMD_BURST16 | DCMD_ENDIRQEN;
		/* for in-place process */
		if (in == out)
			info = geu_dma_in_info;
		else
			info = geu_dma_out_info;

		/* setup output DMA desc */
		for (i = 0; i < info->num; i++) {
			geu_dma_out_desc[i].ddadr =
			    DMA_PHY_ADDR(geu, &geu_dma_out_desc[i + 1]);
			geu_dma_out_desc[i].dsadr =
			    geu->geu_io_mem + GEU_OUT_DATA_AFTER_ENC_DEC;
			geu_dma_out_desc[i].dtadr = info->addr[i];
			geu_dma_out_desc[i].dcmd =
			    DCMD_INCTRGADDR | DCMD_FLOWSRC | DCMD_WIDTH4 |
			    DCMD_BURST16 | info->size[i];
		}
		/* mark term */
		geu_dma_out_desc[i].ddadr = DDADR_STOP;
		geu_dma_out_desc[i].dcmd =
		    DCMD_FLOWSRC | DCMD_BURST16 | DCMD_ENDIRQEN;

#ifdef GEU_DEBUG_INFO
		dump_dma_desc(geu_dma_in_desc, geu_dma_in_info->num);
		dump_dma_desc(geu_dma_out_desc, info->num);
#endif
		/* start input/output dma engine */
		DDADR(geu_dma_in) = DMA_PHY_ADDR(geu, geu_dma_in_desc);
		DCSR(geu_dma_in) = DCSR_RUN;

		DDADR(geu_dma_out) = DMA_PHY_ADDR(geu, geu_dma_out_desc);
		DCSR(geu_dma_out) = DCSR_RUN;
		GEUWriteReg(geu, GEU_CONFIG,
			    conf | GEU_CONFIG_EN_DMA_MODE_AES_CIPHER);

		/* wait for dma complete */
		if (0 ==
		    wait_for_completion_timeout(&(geu->cmd_complete),
						GEU_DMA_TIMEOUT)) {
			printk(KERN_ERR "GEU DMA timeout status(%x)\n",
				GEUReadReg(geu, GEU_STATUS));
			ret = -ETIME;
		}

		if (in == out) {
			pxa_dma_unmap(geu_dma_in_info, in, len,
				      DMA_BIDIRECTIONAL);
		} else {
			pxa_dma_unmap(geu_dma_in_info, in, len, DMA_TO_DEVICE);
			pxa_dma_unmap(geu_dma_out_info, out, len,
				      DMA_FROM_DEVICE);
		}
		/* break on error. pxa_dma_unmap is need on error */
		if (ret != 0)
			break;

		in += len;
		out += len;
		length -= len;
	}

	GEUWriteReg(geu, GEU_CONFIG, conf);
	return ret;

return 0;
}
#endif

#else
static int GEU_AES_pio(
		struct geu_private *geu,
		unsigned char *in, unsigned char *out,
		unsigned int length)
{
	int ret = 0;
	unsigned int i;
	unsigned int conf;
	unsigned int blkbuf[4];
	struct device *dev = geu->p_device;

	conf = GEUReadReg(geu, GEU_CONFIG);
	GEUWriteReg(geu, GEU_CONFIG, conf | GEU_CONFIG_DATAIMR);

	/* setup transfer info */
	for (i = 0; i < length / 16; i++) {
		if (copy_from_user(blkbuf, in, sizeof(blkbuf))) {
			ret = -EFAULT;
			break;
		}
		GEUWriteReg(geu, GEU_INPUT_DATA_ENC_DEC, blkbuf[0]);
		GEUWriteReg(geu, GEU_INPUT_DATA_ENC_DEC + 4, blkbuf[1]);
		GEUWriteReg(geu, GEU_INPUT_DATA_ENC_DEC + 8, blkbuf[2]);
		GEUWriteReg(geu, GEU_INPUT_DATA_ENC_DEC + 12, blkbuf[3]);
		GEUWriteReg(geu, GEU_STATUS, GEU_STATUS_DINREADY);
		if (0 ==
			wait_for_completion_timeout(&(geu->cmd_complete),
						GEU_DMA_TIMEOUT)) {
			dev_err(dev, "GEU timeout status(%x) config(%x)\n",
				GEUReadReg(geu, GEU_STATUS),
				GEUReadReg(geu, GEU_CONFIG));
			ret = -ETIME;
			break;
		}
		/* read output data */
		blkbuf[0] = GEUReadReg(geu, GEU_OUT_DATA_AFTER_ENC_DEC);
		blkbuf[1] = GEUReadReg(geu, GEU_OUT_DATA_AFTER_ENC_DEC + 4);
		blkbuf[2] = GEUReadReg(geu, GEU_OUT_DATA_AFTER_ENC_DEC + 8);
		blkbuf[3] = GEUReadReg(geu, GEU_OUT_DATA_AFTER_ENC_DEC + 12);
		if (copy_to_user(out, blkbuf, sizeof(blkbuf))) {
			ret = -EFAULT;
			break;
		}
		in += 4;
		out += 4;
	}
	/* restore configure */
	GEUWriteReg(geu, GEU_CONFIG, conf);
	return ret;
}
#endif

static int GEU_AES_process(
		struct geu_private *geu,
		unsigned char *in, unsigned char *out,
		unsigned int length)
{
	int ret;
	struct device *dev = geu->p_device;

	if (!mutex_is_locked(&(geu->geu_lock))) {
		dev_err(dev,
			"invalid sequence. GEU engine is not initialized");
		return -EINVAL;
	}
#ifdef GEU_MODE_DMA
	pm_qos_update_request(&(geu->geu_qos_idle),
		geu->geu_lpm);
	ret = GEU_AES_direct(geu, in, out, length);
#if 0
	/* use direct mode in small length or unaligned input/output address */
	if (length < GEU_DMA_THRESHOLD || GEU_DMA_ALIGN(in)
		|| GEU_DMA_ALIGN(out)) {
		ret = GEU_AES_direct(geu, in, out, length);
	} else {
		ret = GEU_AES_chain(geu, in, out, length);
	}
#endif
	pm_qos_update_request(&(geu->geu_qos_idle),
				PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE);
#else
	ret = GEU_AES_pio(in, out, length);
#endif
	return ret;
}

static int GEU_AES_finish(struct geu_private *geu)
{
	int i;
	struct device *dev = geu->p_device;

	if (!mutex_is_locked(&(geu->geu_lock))) {
		dev_err(dev,
				"invalid sequence. GEU engine is not initialized");
		return -EINVAL;
	}

	/* clear iv for causion, otherwise next AES will fail  */
	for (i = 0; i < 4; i++)
		GEUWriteReg(geu, GEU_1ST_OFF_CODE_OCB_MODE + (i * 4), 0);

	GEUWriteReg(geu, GEU_STATUS, 0);
	GEUWriteReg(geu, GEU_CONFIG, 0);
	clk_disable(geu->geu_clk);
	geu->geu_owner = 0;
	mutex_unlock(&(geu->geu_lock));
	return 0;
}

static int GEU_AES_init_RKEK(
		struct geu_private *geu,
		int encrypt)
{
	unsigned int conf;
	struct device *dev = geu->p_device;

	if (mutex_trylock(&(geu->geu_lock)) == 0) {
		dev_err(dev,
			"GEU invalid sequence. Engine has been initialized\n");
		return -EBUSY;
	}
	clk_enable(geu->geu_clk);
	geu->geu_owner = current->tgid;

	conf = GEUReadReg(geu, GEU_CONFIG);
	conf &= ~GEU_CONFIG_KEYSZ_MASK;
	/* RKEK key size 256 */
	conf |= 3;
	/* set encrypt/decrypt */
	if (encrypt != 0)
		conf &= ~GEU_CONFIG_ENCDEC;
	else
		conf |= GEU_CONFIG_ENCDEC;

	conf |= GEU_CONFIG_KEYIMR;
	conf |= GEU_CONFIG_AES_KEY_SIZE_SEL;
	/* conf |= GEU_CONFIG_RNDKEYHA|GEU_CONFIG_SBOXHA; */
	GEUWriteReg(geu, GEU_CONFIG, conf);
	GEUWriteReg(geu, GEU_STATUS, GEU_STATUS_KEYREADY);

	/* wait for complete */
	if (0 == wait_for_completion_timeout(&(geu->cmd_complete),
		GEU_DMA_TIMEOUT)) {
		dev_err(dev,
				"GEU init RKEK timeout. status(%x), config(%x)\n",
				GEUReadReg(geu, GEU_STATUS),
				GEUReadReg(geu, GEU_CONFIG));
		clk_disable(geu->geu_clk);
		geu->geu_owner = 0;
		mutex_unlock(&(geu->geu_lock));
		return -ETIME;
	}
	/* dump_round_key(); */
	return 0;
}

static int GEU_AES_init_CBC(
		struct geu_private *geu,
		unsigned int *key, int keylen,
		unsigned int *iv,
		int encrypt)
{
	int i;
	unsigned int conf;
	struct device *dev = geu->p_device;

	if (mutex_trylock(&(geu->geu_lock)) == 0) {
		dev_err(dev,
			"GEU invalid sequence. Engine has been initialized\n");
		return -EBUSY;
	}
	clk_enable(geu->geu_clk);
	geu->geu_owner = current->tgid;

	conf = GEUReadReg(geu, GEU_CONFIG);
	conf &= ~GEU_CONFIG_KEYSZ_MASK;
	/* 128 to 1, 192 to 2, 256 to 3 */
	conf |= ((keylen >> 3) - 1) << GEU_CONFIG_KEYSZ_SHIFT;
	/* set encrypt/decrypt */
	if (encrypt != 0)
		conf &= ~GEU_CONFIG_ENCDEC;
	else
		conf |= GEU_CONFIG_ENCDEC;

	conf |= GEU_CONFIG_KEYIMR;
	GEUWriteReg(geu, GEU_CONFIG, conf);
	/* write user key */
	for (i = 0; i < 8; i++)
		GEUWriteReg(geu, GEU_INIT_KEY_VALUE + (i * 4), key[i]);

	/* set CBC IV */
	for (i = 0; i < 4; i++)
		GEUWriteReg(geu, GEU_1ST_OFF_CODE_OCB_MODE + (i * 4), iv[i]);

	/*Need to toggle bit24 according to SteveFeng  */
	GEUWriteReg(geu, GEU_CONFIG, conf | GEU_CONFIG_WRITE_INI_VAL_IN_CBC_MODE);
	GEUWriteReg(geu, GEU_CONFIG, conf);
	conf |= GEU_CONFIG_CBC_ECB_MODE | GEU_CONFIG_OCBBYP;
	GEUWriteReg(geu, GEU_CONFIG, conf);

	GEUWriteReg(geu, GEU_STATUS, GEU_STATUS_KEYREADY);

	/* wait for complete, timeout = 1s */
	if (0 == wait_for_completion_timeout(&(geu->cmd_complete),
		GEU_DMA_TIMEOUT)) {
		dev_err(dev,
				"GEU AES CBC init timeout. status(%x), config(%x)\n",
				GEUReadReg(geu, GEU_STATUS),
				GEUReadReg(geu, GEU_CONFIG));
		clk_disable(geu->geu_clk);
		geu->geu_owner = 0;
		mutex_unlock(&(geu->geu_lock));
		return -ETIME;
	}

	GEU_DEBUG(dev_err(dev,
			"##GEU AES CBC init status(%x) config(%x)\n",
			GEUReadReg(geu, GEU_STATUS),
			GEUReadReg(geu, GEU_CONFIG)));
	return 0;
}

static int GEU_gen_rand(
		struct geu_private *geu,
		unsigned char *rnd,
		unsigned int len)
{
	int copy;
	unsigned int old, new;
	unsigned int conf;
	struct device *dev = geu->p_device;

	if (mutex_trylock(&(geu->geu_lock)) == 0) {
		dev_err(dev,
		"GEU engine busy. Finish AES before generate random number\n");
		return -EBUSY;
	}
	clk_enable(geu->geu_clk);
	conf = GEUReadReg(geu, GEU_CONFIG);
	/* set to use analog RNG */
	GEUWriteReg(geu, GEU_CONFIG,
			conf | GEU_CONFIG_STICKY_CONTROL_BIT |
			(3 << GEU_CONFIG_FUSE_BLOCK_NUMBER_SHIFT));
	while (len > 0) {
		old = GEUReadReg(geu, GEU_HW_RANDOM_NUM_GEN);
		/* add 0x80 to accerlate RNG */
		GEUWriteReg(geu, GEU_FUSE_PROG_VAL1, (prandom_u32() & 0xFF) | 0x80);
		new = GEUReadReg(geu, GEU_HW_RANDOM_NUM_GEN);
		while (old == new) {
			udelay(10);
			new = GEUReadReg(geu, GEU_HW_RANDOM_NUM_GEN);
		}
		copy = len > sizeof(unsigned int) ? sizeof(unsigned int) : len;
		if (copy_to_user(rnd, &new, copy))
			break;

		rnd += copy;
		len -= copy;
	}
	/* clear iv for causion, otherwise next AES will fail */
	for (copy = 0; copy < 4; copy++)
		GEUWriteReg(geu, GEU_1ST_OFF_CODE_OCB_MODE + (copy * 4), 0);

	GEUWriteReg(geu, GEU_CONFIG, conf);
	clk_disable(geu->geu_clk);
	mutex_unlock(&(geu->geu_lock));
	return 0;
}

static int GEU_read_key_hash(
		struct geu_private *geu,
		unsigned int *puiKeyHash,
		unsigned int len)
{
	/* unsigned int scratch; */
	unsigned int i;
	unsigned int puiHashVal[8];
	struct device *dev = geu->p_device;

	if (mutex_trylock(&(geu->geu_lock)) == 0) {
		dev_err(dev, "GEU engine busy.\n");
		return -EBUSY;
	}
	clk_enable(geu->geu_clk);

	if ((len != GEU_SHA160_SIZE) &&
	    (len != GEU_SHA224_SIZE) && (len != GEU_SHA256_SIZE)) {
		clk_disable(geu->geu_clk);
		mutex_unlock(&(geu->geu_lock));
		return -EFAULT;
	}

	for (i = 0; i < (len / 4); i++)
		puiHashVal[i] = GEUReadReg(geu, GEU_FUSE_VAL_OEM_HASH_KEY + (i * 4));

#if 0
	/* Check if ECC enabled and any uncorrectable errors */
	scratch = GEUReadReg(geu, OEM_KEY_HASH_ECC);
	if (scratch != 0) {
		/* ECC enabled - Check the ECC_STATUS value */
		scratch = GEUReadReg(geu, GEU_ECC_STATUS);
		if (scratch & K_OEM_UNCORRECTABLE_ECC_ERROR_MASK) {
			clk_disable(geu_clk);
			mutex_unlock(&(geu->geu_lock));
			return -EFAULT;
		}
	}
#endif

	if (copy_to_user(puiKeyHash, puiHashVal, len)) {
		clk_disable(geu->geu_clk);
		mutex_unlock(&(geu->geu_lock));
		return -EFAULT;
	}

	clk_disable(geu->geu_clk);
	mutex_unlock(&(geu->geu_lock));
	return 0;
}

static int GEU_read_lifecycle(
		struct geu_private *geu,
		unsigned int *puiLifecycle,
		unsigned int uiReq)
{
	unsigned int puiRead[2];
	struct device *dev = geu->p_device;

	if (mutex_trylock(&(geu->geu_lock)) == 0) {
		dev_err(dev, "GEU engine busy.\n");
		return -EBUSY;
	}
	clk_enable(geu->geu_clk);

	/* Read lifecycle */
	/* CM & DM */
	puiRead[0] = GEUReadReg(geu, BLOCK0_RESERVED_1);
	/* DD & FA & PD */
	puiRead[1] = GEUReadReg(geu, BLOCK7_RESERVED_5);

	if (copy_to_user(puiLifecycle, puiRead, 2 * sizeof(unsigned int))) {
		clk_disable(geu->geu_clk);
		mutex_unlock(&(geu->geu_lock));
		return -EFAULT;
	}

	clk_disable(geu->geu_clk);
	mutex_unlock(&(geu->geu_lock));
	return 0;
}

static int pxa_geu_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int pxa_geu_close(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev;
	struct geu_private *geu;

	miscdev = file->private_data;
	geu = container_of(miscdev, struct geu_private, pxa_geu_miscdev);

	if (geu->geu_owner == current->tgid
		&& mutex_is_locked(&(geu->geu_lock))) {
		/* user close without unlock geu_lock */
		geu->geu_owner = 0;
		mutex_unlock(&(geu->geu_lock));
	}
	return 0;
}

#ifndef KERN_USE_UNLOCKEDIOCTL
static int pxa_geu_ioctl(struct inode *inode, struct file *file, u_int cmd,
			 u_long arg)
#else
static long pxa_geu_ioctl(struct file *file, u_int cmd, u_long arg)
#endif
{
	struct miscdevice *miscdev;
	struct geu_private *geu;
	struct device *dev;
	struct geu_arg geu_arg;
	int ret;

	miscdev = file->private_data;
	geu = container_of(miscdev, struct geu_private, pxa_geu_miscdev);
	if (geu == NULL)
		return -EFAULT;
	dev = geu->p_device;

	if (copy_from_user(&geu_arg, (void __user *)arg, sizeof(geu_arg)))
		return -EFAULT;

	switch (cmd) {
	case GEU_AES_INIT:
		{
			unsigned int userkey[8];
			if (geu_arg.arg1 != 16 && geu_arg.arg1 != 24
			    && geu_arg.arg1 != 32) {
				dev_err(dev, "GEU invalid key length\n");
				return -EINVAL;
			}
			memset(userkey, 0, sizeof(userkey));
			if (copy_from_user
			    (userkey, (void __user *)geu_arg.arg0,
			     geu_arg.arg1)) {
				return -EFAULT;
			}
			ret = GEU_AES_init(geu, userkey, geu_arg.arg1, geu_arg.arg2);
		}
		break;
	case GEU_AES_PROCESS:
		{
			if (geu_arg.arg2 & 15)
				return -EINVAL;

			if (!access_ok(VERIFY_READ, geu_arg.arg0, geu_arg.arg2)
			    || !access_ok(VERIFY_WRITE, geu_arg.arg1,
					  geu_arg.arg2)) {
				return -EFAULT;
			}
			ret = GEU_AES_process(
		            geu,
		            (unsigned char *)geu_arg.arg0,
		            (unsigned char *)geu_arg.arg1,
		             geu_arg.arg2);
		}
		break;
	case GEU_AES_FINISH:
		{
			ret = GEU_AES_finish(geu);
		}
		break;
	case GEU_AES_INIT_RKEK:
		{
			ret = GEU_AES_init_RKEK(geu, geu_arg.arg0);
		}
		break;
	case GEU_AES_INIT_CBC:
		{
			unsigned int userkey[8];
			unsigned int useriv[4];
			if (geu_arg.arg1 != 16 && geu_arg.arg1 != 24
			    && geu_arg.arg1 != 32) {
				dev_err(dev, "GEU invalid key length\n");
				return -EINVAL;
			}
			memset(userkey, 0, sizeof(userkey));
			memset(useriv, 0, sizeof(useriv));
			if (copy_from_user
			    (userkey, (void __user *)geu_arg.arg0,
			     geu_arg.arg1)) {
				return -EFAULT;
			}
			if (copy_from_user
			    (useriv, (void __user *)geu_arg.arg2, 16)) {
				return -EFAULT;
			}
			ret =
			    GEU_AES_init_CBC(geu, userkey, geu_arg.arg1, useriv,
					     geu_arg.arg3);
		}
		break;
	case GEU_GEN_RAND:
		{
			if (!access_ok
			    (VERIFY_WRITE, geu_arg.arg0, geu_arg.arg1)) {
				return -EFAULT;
			}
			ret =
			    GEU_gen_rand(geu, (void __user *)geu_arg.arg0,
					 geu_arg.arg1);
		}
		break;
	case GEU_READ_OEMHASHKEY:
		{
			if (!access_ok
			    (VERIFY_WRITE, geu_arg.arg0, geu_arg.arg1)) {
				return -EFAULT;
			}
			ret =
			    GEU_read_key_hash(geu, (void __user *)geu_arg.arg0,
					      geu_arg.arg1);
		}
		break;
	case GEU_READ_LIFECYCLE:
		{
			if (!access_ok
			    (VERIFY_WRITE, geu_arg.arg0, geu_arg.arg1)) {
				return -EFAULT;
			}
			ret =
				GEU_read_lifecycle(geu, (void __user *)geu_arg.arg0,
						geu_arg.arg1);
		}
		break;
	default:
		dev_err(dev, "GEU IOCTL invald command %x\n", cmd);
		ret = -EINVAL;
	}
	return ret;
}

static const struct file_operations pxa_geu_fops = {
	.owner = THIS_MODULE,
	.open = pxa_geu_open,
	.release = pxa_geu_close,
#ifndef KERN_USE_UNLOCKEDIOCTL
	.ioctl = pxa_geu_ioctl,
#else
	.unlocked_ioctl = pxa_geu_ioctl,
#endif
};

static irqreturn_t pxa_geu_irq(int irq, void *devid)
{
	unsigned int status;
	struct geu_private *geu = devid;

	status = GEUReadReg(geu, GEU_STATUS);
	/* clear interrupt */
	GEUWriteReg(geu, GEU_STATUS, status);
	complete(&(geu->cmd_complete));
	return IRQ_HANDLED;
}

#ifdef GEU_MODE_DMA

static void pxa_geu_dma_irq_in(void *data)
{
	return;
}

static void pxa_geu_dma_irq_out(void *data)
{
	struct geu_private *geu = data;

	if (dma_async_is_tx_complete(
			geu->geu_dma_out,
			geu->geu_dma_out_cookie,
			NULL, NULL) == DMA_SUCCESS)
		complete(&(geu->cmd_complete));

	return;
}

#endif

static int mmp_geu_probe(struct platform_device *pdev)
{
	struct geu_private *geu;
	struct device *dev = &pdev->dev;
	struct resource *r;
	struct resource *irqres;
	struct resource *mmres;
	dma_cap_mask_t mask;
	struct device_node *np = pdev->dev.of_node;
	int ret;
#ifdef GEU_MODE_DMA
	const void *qos;
#endif

	geu = devm_kzalloc(dev, sizeof(struct geu_private),
		GFP_KERNEL);
	if (geu == NULL)
		return -ENOMEM;

	geu->p_device = dev;
	mutex_init(&(geu->geu_lock));
	init_completion(&(geu->cmd_complete));

	geu->geu_clk = devm_clk_get(dev, "AESCLK");
	if (IS_ERR(geu->geu_clk)) {
		dev_err(dev, "GEU get clock fail\n");
		return -EBUSY;
	}

	ret = clk_prepare(geu->geu_clk);
	if (ret) {
		dev_err(dev, "GEU prepare clock fail\n");
		goto out;
	}

	geu->dma_buf_base = dma_alloc_coherent(NULL, GEU_DMA_SIZE,
						(dma_addr_t *)&(geu->dma_phy_base),
						GFP_KERNEL);
	if (geu->dma_buf_base == NULL) {
		dev_err(dev, "GEU: failed to allocate DMA memory\n");
		goto out;
	}
#ifdef GEU_MODE_DMA
	geu->geu_dma_in_info = devm_kzalloc(dev, 2 * sizeof(struct dma_info),
		GFP_KERNEL);
	if (geu->geu_dma_in_info == NULL) {
		dev_err(dev,
			"GEU: failed to allocate DMA descript memory\n");
		goto fail_free_dma;
	}
	geu->geu_dma_out_info =
		geu->geu_dma_in_info + 1;

	qos = of_get_property(np, "lpm-qos", NULL);
	if (!qos) {
		dev_err(dev, "GEU: failed to get property qos\n");
		goto fail_free_info;
	}
	geu->geu_lpm = be32_to_cpup(qos);

#endif
	mmres = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mmres) {
		dev_err(dev,
			"GEU: failed to get property reg\n");
		goto fail_free_info;
	}

	geu->geu_io_len = PXA_GEU_IOMEM_SIZE;
	geu->geu_io_mem = mmres->start;

	r = request_mem_region(geu->geu_io_mem,
				geu->geu_io_len, "geu");
	if (r == NULL) {
		dev_err(dev, "GEU: failed to request memory resource\n");
		goto fail_free_info;
	}

	geu->geu_iobase = devm_ioremap(dev, r->start, resource_size(r));
	if (geu->geu_iobase == NULL) {
		dev_err(dev, "GEU: ioremap fail from 0x%x to 0x%x\n",
			r->start, r->end);
		goto fail_free_res;
	}

	irqres = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irqres) {
		dev_err(dev, "GEU: failed to get property irq\n");
		goto fail_free_iomem;
	}

	geu->geu_irq = irqres->start;
	ret = devm_request_irq(dev, geu->geu_irq,
		pxa_geu_irq, 0, "geu", geu);
	if (ret < 0) {
		dev_err(dev, "GEU: failed to request IRQ\n");
		goto fail_free_iomem;
	}
#ifdef GEU_MODE_DMA

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	geu->geu_dma_in =
		dma_request_slave_channel(dev, "rx");
	if (NULL == geu->geu_dma_in) {
		dev_err(dev, "GEU: failed to request RX DMA channel\n");
		goto fail_free_irq;
	}
	geu->geu_dma_out =
		dma_request_slave_channel(dev, "tx");
	if (NULL == geu->geu_dma_out) {
		dev_err(dev, "GEU: failed to request TX DMA channel\n");
		goto fail_free_in_dma;
	}
	/* setup DMA channel mapping */
	geu->geu_qos_idle.name = "geu";
	pm_qos_add_request(&(geu->geu_qos_idle),
		PM_QOS_CPUIDLE_BLOCK,
		PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE);
#endif
	/* register the device */
	geu->pxa_geu_miscdev.minor = MISC_DYNAMIC_MINOR;
	geu->pxa_geu_miscdev.name = "geu";
	geu->pxa_geu_miscdev.fops = &pxa_geu_fops;
	geu->pxa_geu_miscdev.this_device = NULL;
	ret = misc_register(&(geu->pxa_geu_miscdev));
	if (ret < 0) {
		dev_err(dev,
			"GEU: unable to register device node /dev/geu\n");
		goto fail_free_out_dma;
	}
	clk_enable(geu->geu_clk);
	/* clear iv for causion, otherwise next AES will fail  */
	for (ret = 0; ret < 4; ret++)
		GEUWriteReg(geu, GEU_1ST_OFF_CODE_OCB_MODE + (ret * 4), 0);

	GEUWriteReg(geu, GEU_STATUS, 0);
	GEUWriteReg(geu, GEU_CONFIG, 0);
	clk_disable(geu->geu_clk);

	platform_set_drvdata(pdev, geu);
	dev_info(dev, "GEU driver info: %s\n", GEU_DRIVER_VERSION);
	return 0;

fail_free_out_dma:
#ifdef GEU_MODE_DMA
	pm_qos_remove_request(&(geu->geu_qos_idle));
	dma_release_channel(geu->geu_dma_out);
fail_free_in_dma:
	dma_release_channel(geu->geu_dma_in);
fail_free_irq:
#endif
fail_free_iomem:
fail_free_res:
	release_mem_region(r->start, resource_size(r));
fail_free_info:
#ifdef GEU_MODE_DMA
fail_free_dma:
#endif
	dma_free_coherent(NULL, GEU_DMA_SIZE,
		geu->dma_buf_base,
		(dma_addr_t) (geu->dma_phy_base));
out:
	devm_clk_put(dev, geu->geu_clk);
	return -EBUSY;
}

static int mmp_geu_remove(struct platform_device *pdev)
{
	struct geu_private *geu = platform_get_drvdata(pdev);

	if (geu) {
		misc_deregister(&(geu->pxa_geu_miscdev));
#ifdef GEU_MODE_DMA
		pm_qos_remove_request(&(geu->geu_qos_idle));
		dma_release_channel(geu->geu_dma_out);
		dma_release_channel(geu->geu_dma_in);
		geu->geu_dma_out = NULL;
		geu->geu_dma_in = NULL;
#endif
		release_mem_region(geu->geu_io_mem,
			geu->geu_io_len);
		dma_free_coherent(NULL, GEU_DMA_SIZE,
				geu->dma_buf_base,
				(dma_addr_t)(geu->dma_phy_base));
		platform_set_drvdata(pdev, NULL);
	}

	return 0;
}

static const struct of_device_id mv_geu_dt_match[] = {
	{ .compatible = "marvell,mmp-geu", .data = NULL },
	{},
};

static struct platform_driver mmp_geu_driver = {
	 .driver = {
		 .name = MMP_GEU_DRV_NAME,
		 .of_match_table = of_match_ptr(mv_geu_dt_match),
	 },
	 .probe = mmp_geu_probe,
	 .remove = mmp_geu_remove,
};

module_platform_driver(mmp_geu_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lindong Wu (ldwu@marvell.com)");
MODULE_DESCRIPTION("Generic Encrypt Uint driver");
