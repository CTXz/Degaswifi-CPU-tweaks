/*
 * linux/drivers/video/mmp/setup.c
 * Setup interfaces for Marvell Display Controller
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/of_fdt.h>
#include <linux/of.h>
#include <linux/memblock.h>
#include <video/mmp_disp.h>

char panel_type[20];
/*
 * FIXME: the string from cmdline should be like
 * "androidboot.lcd=1080_50"
 */
/* FB memeory reservation, 0x1800000@0x17000000 by default */
static u32 fb_size = 0x1800000;
static u32 fb_addr = 0x17000000;
static u32 xres;
static u32 yres;
static u32 buf_num = 3;
static u32 fb_carveout_mem;
static u32 skip_power;

#ifdef CONFIG_OF
static int __init mmp_fdt_find_fb_info(unsigned long node, const char *uname,
		int depth, void *data)
{
	__be32 *prop;
	unsigned long len;

	if (!of_flat_dt_is_compatible(node, "marvell,mmp-fb"))
		return 0;

	prop = of_get_flat_dt_prop(node, "marvell,buffer-num", &len);
	if (!prop || (len != sizeof(unsigned long))) {
		pr_err("%s: Can't find buf num property\n", __func__);
		return 0;
	}

	buf_num = be32_to_cpu(prop[0]);

	prop = of_get_flat_dt_prop(node, "marvell,fb-mem", &len);
	if (!prop || (len != sizeof(unsigned long))) {
		pr_err("%s: Can't find fb-mem property\n", __func__);
		return 0;
	}

	fb_addr = be32_to_cpu(prop[0]);
	if (fb_addr)
		fb_carveout_mem = 1;

	return 1;
}

static int __init mmp_fdt_find_panel_info(unsigned long node, const char *uname,
		int depth, void *data)
{
	__be32 *prop;
	unsigned long len;

	if (!of_flat_dt_is_compatible(node, "mmp-panel-modes"))
		return 0;

	prop = of_get_flat_dt_prop(node, "xres", &len);
	if (!prop || (len != sizeof(unsigned long))) {
		pr_err("%s: Can't find xres property\n", __func__);
		return 0;
	}

	xres = be32_to_cpu(prop[0]);

	prop = of_get_flat_dt_prop(node, "yres", &len);
	if (!prop || (len != sizeof(unsigned long))) {
		pr_err("%s: Can't find yres property\n", __func__);
		return 0;
	}
	yres = be32_to_cpu(prop[0]);

	return 1;

}
#endif

static void  __init sec_panel_setup(void)
{
#ifdef CONFIG_OF
	if (!of_scan_flat_dt(mmp_fdt_find_panel_info, NULL))
		return;
#endif

	if (xres && yres)
		fb_size = PAGE_ALIGN(MMP_XALIGN(xres) *
					MMP_YALIGN(yres) * 4 * buf_num);
}

static int __init panel_setup(char *str)
{
	strlcpy(panel_type, str, 20);
#ifdef CONFIG_OF
	if (!of_scan_flat_dt(mmp_fdt_find_fb_info, NULL))
		return 0;
#endif
	if ((!strcmp(str , "1080_50")) || (!strcmp(str , "1080p")))
		fb_size = PAGE_ALIGN(MMP_XALIGN(1080) *
				MMP_YALIGN(1920) * 4 * buf_num);
	else
		fb_size = PAGE_ALIGN(MMP_XALIGN(720) *
				MMP_YALIGN(1280) * 4 * buf_num);

	return 0;
}
early_param("androidboot.lcd", panel_setup);

static void __init mmpfb_carveout_reserve_mem(void)
{

	/*use MB aligned buffer size*/
	if (fb_size != (fb_size & (~0xfffff)))
		fb_size = ALIGN(fb_size, 0x100000);


	if (fb_addr != (fb_addr & (~0xfffff)))
		pr_err("%s: fb_addr not MB aligned, 0x%8x\n",
				__func__, fb_addr);

	BUG_ON(memblock_reserve(fb_addr, fb_size) != 0);
	memblock_free(fb_addr, fb_size);
	memblock_remove(fb_addr, fb_size);
	pr_info("Reserved FB memory : %dMB at %#.8x\n",
		(unsigned)fb_size/0x100000,
		(unsigned)fb_addr);
}

void __init mmp_reserve_fbmem(void)
{
#ifdef CONFIG_OF
	if (!of_scan_flat_dt(mmp_fdt_find_fb_info, NULL))
		fb_carveout_mem = 0;
#endif
	/* fb_carveout_mem is used to distinguish
	 * whether we need carveout of memory.
	 * This flag will be  set if we are passing
	 * framebuffer address in dts file. If set,
	 * we will do carveout of memory at that address
	 * based on size calculated using xres and yres,
	 * no of buffers
	 */
	if (fb_carveout_mem) {
		sec_panel_setup();
		skip_power = 1;
		mmpfb_carveout_reserve_mem();
	} else {
		BUG_ON(memblock_reserve(fb_addr, fb_size));
		pr_info("FRMEBUFFER reserved memory: 0x%08x@0x%08x\n",
				fb_size, fb_addr);
	}
}

u32 get_fb_carveout_mem_flag(void)
{
	return fb_carveout_mem;
}

u32 get_skip_power_on(void)
{
	return skip_power;
}

char *mmp_get_paneltype(void)
{
	return panel_type;
}
