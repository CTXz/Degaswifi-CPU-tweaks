/*
 * drivers/video/mmp/panel/mipi_tft10801920_1e.c
 * active panel using DSI interface to do init
 *
 * Copyright (C) 2013 Marvell Technology Group Ltd.
 * Authors: Xiaolong Ye<yexl@marvell.com>
 *          Yu Xu <yuxu@marvell.com>
 *          Guoqing Li <ligq@marvell.com>
 *          Lisa Du <cldu@marvell.com>
 *          Zhou Zhu <zzhu3@marvell.com>
 *          Jing Xiang <jxiang@marvell.com>
 *          Yossi Eliaz <yossie@marvell.com>
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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <video/mmp_disp.h>
#include <video/mipi_display.h>

struct tft_1e_plat_data {
	struct mmp_panel *panel;
	void (*plat_onoff)(int status);
};

#define UNLOCK_DELAY (0)

/*
 * On development panel. manufacturer's commands list.
 */

static char TFT_LCD_ARRAY_CMD1[] = { 0xFF, 0xEE };
static char TFT_LCD_ARRAY_CMD2[] = { 0xFB, 0x01 };
static char TFT_LCD_ARRAY_CMD4[] = { 0x24, 0x4F };
static char TFT_LCD_ARRAY_CMD5[] = { 0x38, 0xC8 };
static char TFT_LCD_ARRAY_CMD6[] = { 0x39, 0x27 };
static char TFT_LCD_ARRAY_CMD7[] = { 0x1E, 0x77 };
static char TFT_LCD_ARRAY_CMD8[] = { 0x1D, 0x0F };
static char TFT_LCD_ARRAY_CMD9[] = { 0x7E, 0x71 };
static char TFT_LCD_ARRAY_CMD10[] = { 0xFF, 0x01 };
static char TFT_LCD_ARRAY_CMD11[] = { 0xFB, 0x01 };
static char TFT_LCD_ARRAY_CMD16[] = { 0x00, 0x01 };
static char TFT_LCD_ARRAY_CMD17[] = { 0x01, 0x55 };
static char TFT_LCD_ARRAY_CMD18[] = { 0x02, 0x40 };
static char TFT_LCD_ARRAY_CMD19[] = { 0x05, 0x50 };
static char TFT_LCD_ARRAY_CMD20[] = { 0x06, 0x4A };
static char TFT_LCD_ARRAY_CMD21[] = { 0x07, 0x24 };
static char TFT_LCD_ARRAY_CMD22[] = { 0x08, 0x0C };
static char TFT_LCD_ARRAY_CMD23[] = { 0x0B, 0x87 };
static char TFT_LCD_ARRAY_CMD24[] = { 0x0C, 0x87 };
static char TFT_LCD_ARRAY_CMD25[] = { 0x0E, 0xB0 };
static char TFT_LCD_ARRAY_CMD26[] = { 0x0F, 0xB3 };
static char TFT_LCD_ARRAY_CMD27[] = { 0x11, 0x10 };
static char TFT_LCD_ARRAY_CMD28[] = { 0x12, 0x10 };
static char TFT_LCD_ARRAY_CMD29[] = { 0x13, 0x03 };
static char TFT_LCD_ARRAY_CMD30[] = { 0x14, 0x4A };
static char TFT_LCD_ARRAY_CMD31[] = { 0x15, 0x12 };
static char TFT_LCD_ARRAY_CMD32[] = { 0x16, 0x12 };
static char TFT_LCD_ARRAY_CMD33[] = { 0x18, 0x00 };
static char TFT_LCD_ARRAY_CMD34[] = { 0x19, 0x77 };
static char TFT_LCD_ARRAY_CMD35[] = { 0x1A, 0x55 };
static char TFT_LCD_ARRAY_CMD36[] = { 0x1B, 0x13 };
static char TFT_LCD_ARRAY_CMD37[] = { 0x1C, 0x00 };
static char TFT_LCD_ARRAY_CMD38[] = { 0x1D, 0x00 };
static char TFT_LCD_ARRAY_CMD39[] = { 0x1E, 0x13 };
static char TFT_LCD_ARRAY_CMD40[] = { 0x1F, 0x00 };
static char TFT_LCD_ARRAY_CMD41[] = { 0x23, 0x00 };
static char TFT_LCD_ARRAY_CMD42[] = { 0x24, 0x00 };
static char TFT_LCD_ARRAY_CMD43[] = { 0x25, 0x00 };
static char TFT_LCD_ARRAY_CMD44[] = { 0x26, 0x00 };
static char TFT_LCD_ARRAY_CMD45[] = { 0x27, 0x00 };
static char TFT_LCD_ARRAY_CMD46[] = { 0x28, 0x00 };
static char TFT_LCD_ARRAY_CMD47[] = { 0x35, 0x00 };
static char TFT_LCD_ARRAY_CMD48[] = { 0x66, 0x00 };
static char TFT_LCD_ARRAY_CMD49[] = { 0x58, 0x82 };
static char TFT_LCD_ARRAY_CMD50[] = { 0x59, 0x02 };
static char TFT_LCD_ARRAY_CMD51[] = { 0x5A, 0x02 };
static char TFT_LCD_ARRAY_CMD52[] = { 0x5B, 0x02 };
static char TFT_LCD_ARRAY_CMD53[] = { 0x5C, 0x82 };
static char TFT_LCD_ARRAY_CMD54[] = { 0x5D, 0x82 };
static char TFT_LCD_ARRAY_CMD55[] = { 0x5E, 0x02 };
static char TFT_LCD_ARRAY_CMD56[] = { 0x5F, 0x02 };
static char TFT_LCD_ARRAY_CMD57[] = { 0x72, 0x31 };
static char TFT_LCD_ARRAY_CMD58[] = { 0xFF, 0x05 };
static char TFT_LCD_ARRAY_CMD59[] = { 0xFB, 0x01 };
static char TFT_LCD_ARRAY_CMD60[] = { 0x00, 0x01 };
static char TFT_LCD_ARRAY_CMD61[] = { 0x01, 0x0B };
static char TFT_LCD_ARRAY_CMD62[] = { 0x02, 0x0C };
static char TFT_LCD_ARRAY_CMD63[] = { 0x03, 0x09 };
static char TFT_LCD_ARRAY_CMD64[] = { 0x04, 0x0A };
static char TFT_LCD_ARRAY_CMD65[] = { 0x05, 0x00 };
static char TFT_LCD_ARRAY_CMD66[] = { 0x06, 0x0F };
static char TFT_LCD_ARRAY_CMD67[] = { 0x07, 0x10 };
static char TFT_LCD_ARRAY_CMD68[] = { 0x08, 0x00 };
static char TFT_LCD_ARRAY_CMD69[] = { 0x09, 0x00 };
static char TFT_LCD_ARRAY_CMD70[] = { 0x0A, 0x00 };
static char TFT_LCD_ARRAY_CMD71[] = { 0x0B, 0x00 };
static char TFT_LCD_ARRAY_CMD72[] = { 0x0C, 0x00 };
static char TFT_LCD_ARRAY_CMD73[] = { 0x0D, 0x13 };
static char TFT_LCD_ARRAY_CMD74[] = { 0x0E, 0x15 };
static char TFT_LCD_ARRAY_CMD75[] = { 0x0F, 0x17 };
static char TFT_LCD_ARRAY_CMD76[] = { 0x10, 0x01 };
static char TFT_LCD_ARRAY_CMD77[] = { 0x11, 0x0B };
static char TFT_LCD_ARRAY_CMD78[] = { 0x12, 0x0C };
static char TFT_LCD_ARRAY_CMD79[] = { 0x13, 0x09 };
static char TFT_LCD_ARRAY_CMD80[] = { 0x14, 0x0A };
static char TFT_LCD_ARRAY_CMD81[] = { 0x15, 0x00 };
static char TFT_LCD_ARRAY_CMD82[] = { 0x16, 0x0F };
static char TFT_LCD_ARRAY_CMD83[] = { 0x17, 0x10 };
static char TFT_LCD_ARRAY_CMD84[] = { 0x18, 0x00 };
static char TFT_LCD_ARRAY_CMD85[] = { 0x19, 0x00 };
static char TFT_LCD_ARRAY_CMD86[] = { 0x1A, 0x00 };
static char TFT_LCD_ARRAY_CMD87[] = { 0x1B, 0x00 };
static char TFT_LCD_ARRAY_CMD88[] = { 0x1C, 0x00 };
static char TFT_LCD_ARRAY_CMD89[] = { 0x1D, 0x13 };
static char TFT_LCD_ARRAY_CMD90[] = { 0x1E, 0x15 };
static char TFT_LCD_ARRAY_CMD91[] = { 0x1F, 0x17 };
static char TFT_LCD_ARRAY_CMD92[] = { 0x20, 0x00 };
static char TFT_LCD_ARRAY_CMD93[] = { 0x21, 0x03 };
static char TFT_LCD_ARRAY_CMD94[] = { 0x22, 0x01 };
static char TFT_LCD_ARRAY_CMD95[] = { 0x23, 0x40 };
static char TFT_LCD_ARRAY_CMD96[] = { 0x24, 0x40 };
static char TFT_LCD_ARRAY_CMD97[] = { 0x25, 0xED };
static char TFT_LCD_ARRAY_CMD98[] = { 0x29, 0x58 };
static char TFT_LCD_ARRAY_CMD99[] = { 0x2A, 0x12 };
static char TFT_LCD_ARRAY_CMD100[] = { 0x2B, 0x01 };
static char TFT_LCD_ARRAY_CMD101[] = { 0x4B, 0x06 };
static char TFT_LCD_ARRAY_CMD102[] = { 0x4C, 0x11 };
static char TFT_LCD_ARRAY_CMD103[] = { 0x4D, 0x20 };
static char TFT_LCD_ARRAY_CMD104[] = { 0x4E, 0x02 };
static char TFT_LCD_ARRAY_CMD105[] = { 0x4F, 0x02 };
static char TFT_LCD_ARRAY_CMD106[] = { 0x50, 0x20 };
static char TFT_LCD_ARRAY_CMD107[] = { 0x51, 0x61 };
static char TFT_LCD_ARRAY_CMD108[] = { 0x52, 0x01 };
static char TFT_LCD_ARRAY_CMD109[] = { 0x53, 0x63 };
static char TFT_LCD_ARRAY_CMD110[] = { 0x54, 0x77 };
static char TFT_LCD_ARRAY_CMD111[] = { 0x55, 0xED };
static char TFT_LCD_ARRAY_CMD112[] = { 0x5B, 0x00 };
static char TFT_LCD_ARRAY_CMD113[] = { 0x5C, 0x00 };
static char TFT_LCD_ARRAY_CMD114[] = { 0x5D, 0x00 };
static char TFT_LCD_ARRAY_CMD115[] = { 0x5E, 0x00 };
static char TFT_LCD_ARRAY_CMD116[] = { 0x5F, 0x15 };
static char TFT_LCD_ARRAY_CMD117[] = { 0x60, 0x75 };
static char TFT_LCD_ARRAY_CMD118[] = { 0x61, 0x00 };
static char TFT_LCD_ARRAY_CMD119[] = { 0x62, 0x00 };
static char TFT_LCD_ARRAY_CMD120[] = { 0x63, 0x00 };
static char TFT_LCD_ARRAY_CMD121[] = { 0x64, 0x00 };
static char TFT_LCD_ARRAY_CMD122[] = { 0x65, 0x00 };
static char TFT_LCD_ARRAY_CMD123[] = { 0x66, 0x00 };
static char TFT_LCD_ARRAY_CMD124[] = { 0x67, 0x00 };
static char TFT_LCD_ARRAY_CMD125[] = { 0x68, 0x04 };
static char TFT_LCD_ARRAY_CMD126[] = { 0x69, 0x00 };
static char TFT_LCD_ARRAY_CMD127[] = { 0x6A, 0x00 };
static char TFT_LCD_ARRAY_CMD128[] = { 0x6C, 0x40 };
static char TFT_LCD_ARRAY_CMD129[] = { 0x75, 0x01 };
static char TFT_LCD_ARRAY_CMD130[] = { 0x76, 0x01 };
static char TFT_LCD_ARRAY_CMD131[] = { 0x7A, 0x80 };
static char TFT_LCD_ARRAY_CMD132[] = { 0x7B, 0xA3 };
static char TFT_LCD_ARRAY_CMD133[] = { 0x7C, 0xD8 };
static char TFT_LCD_ARRAY_CMD134[] = { 0x7D, 0x60 };
static char TFT_LCD_ARRAY_CMD135[] = { 0x7F, 0x15 };
static char TFT_LCD_ARRAY_CMD136[] = { 0x80, 0x81 };
static char TFT_LCD_ARRAY_CMD137[] = { 0x83, 0x05 };
static char TFT_LCD_ARRAY_CMD138[] = { 0x93, 0x08 };
static char TFT_LCD_ARRAY_CMD139[] = { 0x94, 0x10 };
static char TFT_LCD_ARRAY_CMD140[] = { 0x8A, 0x00 };
static char TFT_LCD_ARRAY_CMD141[] = { 0x9B, 0x0F };
static char TFT_LCD_ARRAY_CMD144[] = { 0xFF, 0x01 };
static char TFT_LCD_ARRAY_CMD145[] = { 0xFB, 0x01 };
static char TFT_LCD_ARRAY_CMD146[] = { 0x75, 0x00 };
static char TFT_LCD_ARRAY_CMD147[] = { 0x76, 0x18 };
static char TFT_LCD_ARRAY_CMD148[] = { 0x77, 0x00 };
static char TFT_LCD_ARRAY_CMD149[] = { 0x78, 0x38 };
static char TFT_LCD_ARRAY_CMD150[] = { 0x79, 0x00 };
static char TFT_LCD_ARRAY_CMD151[] = { 0x7A, 0x65 };
static char TFT_LCD_ARRAY_CMD152[] = { 0x7B, 0x00 };
static char TFT_LCD_ARRAY_CMD153[] = { 0x7C, 0x84 };
static char TFT_LCD_ARRAY_CMD154[] = { 0x7D, 0x00 };
static char TFT_LCD_ARRAY_CMD155[] = { 0x7E, 0x98 };
static char TFT_LCD_ARRAY_CMD156[] = { 0x7F, 0x00 };
static char TFT_LCD_ARRAY_CMD157[] = { 0x80, 0xAF };
static char TFT_LCD_ARRAY_CMD158[] = { 0x81, 0x00 };
static char TFT_LCD_ARRAY_CMD159[] = { 0x82, 0xC1 };
static char TFT_LCD_ARRAY_CMD160[] = { 0x83, 0x00 };
static char TFT_LCD_ARRAY_CMD161[] = { 0x84, 0xD2 };
static char TFT_LCD_ARRAY_CMD162[] = { 0x85, 0x00 };
static char TFT_LCD_ARRAY_CMD163[] = { 0x86, 0xDF };
static char TFT_LCD_ARRAY_CMD164[] = { 0x87, 0x01 };
static char TFT_LCD_ARRAY_CMD165[] = { 0x88, 0x11 };
static char TFT_LCD_ARRAY_CMD166[] = { 0x89, 0x01 };
static char TFT_LCD_ARRAY_CMD167[] = { 0x8A, 0x38 };
static char TFT_LCD_ARRAY_CMD168[] = { 0x8B, 0x01 };
static char TFT_LCD_ARRAY_CMD169[] = { 0x8C, 0x76 };
static char TFT_LCD_ARRAY_CMD170[] = { 0x8D, 0x01 };
static char TFT_LCD_ARRAY_CMD171[] = { 0x8E, 0xA7 };
static char TFT_LCD_ARRAY_CMD172[] = { 0x8F, 0x01 };
static char TFT_LCD_ARRAY_CMD173[] = { 0x90, 0xF3 };
static char TFT_LCD_ARRAY_CMD174[] = { 0x91, 0x02 };
static char TFT_LCD_ARRAY_CMD175[] = { 0x92, 0x2F };
static char TFT_LCD_ARRAY_CMD176[] = { 0x93, 0x02 };
static char TFT_LCD_ARRAY_CMD177[] = { 0x94, 0x30 };
static char TFT_LCD_ARRAY_CMD178[] = { 0x95, 0x02 };
static char TFT_LCD_ARRAY_CMD179[] = { 0x96, 0x66 };
static char TFT_LCD_ARRAY_CMD180[] = { 0x97, 0x02 };
static char TFT_LCD_ARRAY_CMD181[] = { 0x98, 0xA0 };
static char TFT_LCD_ARRAY_CMD182[] = { 0x99, 0x02 };
static char TFT_LCD_ARRAY_CMD183[] = { 0x9A, 0xC5 };
static char TFT_LCD_ARRAY_CMD184[] = { 0x9B, 0x02 };
static char TFT_LCD_ARRAY_CMD185[] = { 0x9C, 0xF8 };
static char TFT_LCD_ARRAY_CMD186[] = { 0x9D, 0x03 };
static char TFT_LCD_ARRAY_CMD187[] = { 0x9E, 0x1B };
static char TFT_LCD_ARRAY_CMD188[] = { 0x9F, 0x03 };
static char TFT_LCD_ARRAY_CMD189[] = { 0xA0, 0x46 };
static char TFT_LCD_ARRAY_CMD190[] = { 0xA2, 0x03 };
static char TFT_LCD_ARRAY_CMD191[] = { 0xA3, 0x52 };
static char TFT_LCD_ARRAY_CMD192[] = { 0xA4, 0x03 };
static char TFT_LCD_ARRAY_CMD193[] = { 0xA5, 0x62 };
static char TFT_LCD_ARRAY_CMD194[] = { 0xA6, 0x03 };
static char TFT_LCD_ARRAY_CMD195[] = { 0xA7, 0x71 };
static char TFT_LCD_ARRAY_CMD196[] = { 0xA9, 0x03 };
static char TFT_LCD_ARRAY_CMD197[] = { 0xAA, 0x83 };
static char TFT_LCD_ARRAY_CMD198[] = { 0xAB, 0x03 };
static char TFT_LCD_ARRAY_CMD199[] = { 0xAC, 0x94 };
static char TFT_LCD_ARRAY_CMD200[] = { 0xAD, 0x03 };
static char TFT_LCD_ARRAY_CMD201[] = { 0xAE, 0xA3 };
static char TFT_LCD_ARRAY_CMD202[] = { 0xAF, 0x03 };
static char TFT_LCD_ARRAY_CMD203[] = { 0xB0, 0xAD };
static char TFT_LCD_ARRAY_CMD204[] = { 0xB1, 0x03 };
static char TFT_LCD_ARRAY_CMD205[] = { 0xB2, 0xCC };
static char TFT_LCD_ARRAY_CMD206[] = { 0xB3, 0x00 };
static char TFT_LCD_ARRAY_CMD207[] = { 0xB4, 0x18 };
static char TFT_LCD_ARRAY_CMD208[] = { 0xB5, 0x00 };
static char TFT_LCD_ARRAY_CMD209[] = { 0xB6, 0x38 };
static char TFT_LCD_ARRAY_CMD210[] = { 0xB7, 0x00 };
static char TFT_LCD_ARRAY_CMD211[] = { 0xB8, 0x65 };
static char TFT_LCD_ARRAY_CMD212[] = { 0xB9, 0x00 };
static char TFT_LCD_ARRAY_CMD213[] = { 0xBA, 0x84 };
static char TFT_LCD_ARRAY_CMD214[] = { 0xBB, 0x00 };
static char TFT_LCD_ARRAY_CMD215[] = { 0xBC, 0x9B };
static char TFT_LCD_ARRAY_CMD216[] = { 0xBD, 0x00 };
static char TFT_LCD_ARRAY_CMD217[] = { 0xBE, 0xAF };
static char TFT_LCD_ARRAY_CMD218[] = { 0xBF, 0x00 };
static char TFT_LCD_ARRAY_CMD219[] = { 0xC0, 0xC1 };
static char TFT_LCD_ARRAY_CMD220[] = { 0xC1, 0x00 };
static char TFT_LCD_ARRAY_CMD221[] = { 0xC2, 0xD2 };
static char TFT_LCD_ARRAY_CMD222[] = { 0xC3, 0x00 };
static char TFT_LCD_ARRAY_CMD223[] = { 0xC4, 0xDF };
static char TFT_LCD_ARRAY_CMD224[] = { 0xC5, 0x01 };
static char TFT_LCD_ARRAY_CMD225[] = { 0xC6, 0x11 };
static char TFT_LCD_ARRAY_CMD226[] = { 0xC7, 0x01 };
static char TFT_LCD_ARRAY_CMD227[] = { 0xC8, 0x5D };
static char TFT_LCD_ARRAY_CMD228[] = { 0xC9, 0x01 };
static char TFT_LCD_ARRAY_CMD229[] = { 0xCA, 0x76 };
static char TFT_LCD_ARRAY_CMD230[] = { 0xCB, 0x01 };
static char TFT_LCD_ARRAY_CMD231[] = { 0xCC, 0xA7 };
static char TFT_LCD_ARRAY_CMD232[] = { 0xCD, 0x01 };
static char TFT_LCD_ARRAY_CMD233[] = { 0xCE, 0xF3 };
static char TFT_LCD_ARRAY_CMD234[] = { 0xCF, 0x02 };
static char TFT_LCD_ARRAY_CMD235[] = { 0xD0, 0x2F };
static char TFT_LCD_ARRAY_CMD236[] = { 0xD1, 0x02 };
static char TFT_LCD_ARRAY_CMD237[] = { 0xD2, 0x30 };
static char TFT_LCD_ARRAY_CMD238[] = { 0xD3, 0x02 };
static char TFT_LCD_ARRAY_CMD239[] = { 0xD4, 0x66 };
static char TFT_LCD_ARRAY_CMD240[] = { 0xD5, 0x02 };
static char TFT_LCD_ARRAY_CMD241[] = { 0xD6, 0xA0 };
static char TFT_LCD_ARRAY_CMD242[] = { 0xD7, 0x02 };
static char TFT_LCD_ARRAY_CMD243[] = { 0xD8, 0xC5 };
static char TFT_LCD_ARRAY_CMD244[] = { 0xD9, 0x02 };
static char TFT_LCD_ARRAY_CMD245[] = { 0xDA, 0xF8 };
static char TFT_LCD_ARRAY_CMD246[] = { 0xDB, 0x03 };
static char TFT_LCD_ARRAY_CMD247[] = { 0xDC, 0x1B };
static char TFT_LCD_ARRAY_CMD248[] = { 0xDD, 0x03 };
static char TFT_LCD_ARRAY_CMD249[] = { 0xDE, 0x46 };
static char TFT_LCD_ARRAY_CMD250[] = { 0xDF, 0x03 };
static char TFT_LCD_ARRAY_CMD251[] = { 0xE0, 0x52 };
static char TFT_LCD_ARRAY_CMD252[] = { 0xE1, 0x03 };
static char TFT_LCD_ARRAY_CMD253[] = { 0xE2, 0x62 };
static char TFT_LCD_ARRAY_CMD254[] = { 0xE3, 0x03 };
static char TFT_LCD_ARRAY_CMD255[] = { 0xE4, 0x71 };
static char TFT_LCD_ARRAY_CMD256[] = { 0xE5, 0x03 };
static char TFT_LCD_ARRAY_CMD257[] = { 0xE6, 0x83 };
static char TFT_LCD_ARRAY_CMD258[] = { 0xE7, 0x03 };
static char TFT_LCD_ARRAY_CMD259[] = { 0xE8, 0x94 };
static char TFT_LCD_ARRAY_CMD260[] = { 0xE9, 0x03 };
static char TFT_LCD_ARRAY_CMD261[] = { 0xEA, 0xA3 };
static char TFT_LCD_ARRAY_CMD262[] = { 0xEB, 0x03 };
static char TFT_LCD_ARRAY_CMD263[] = { 0xEC, 0xAD };
static char TFT_LCD_ARRAY_CMD264[] = { 0xED, 0x03 };
static char TFT_LCD_ARRAY_CMD265[] = { 0xEE, 0xCC };
static char TFT_LCD_ARRAY_CMD266[] = { 0xEF, 0x00 };
static char TFT_LCD_ARRAY_CMD267[] = { 0xF0, 0x18 };
static char TFT_LCD_ARRAY_CMD268[] = { 0xF1, 0x00 };
static char TFT_LCD_ARRAY_CMD269[] = { 0xF2, 0x38 };
static char TFT_LCD_ARRAY_CMD270[] = { 0xF3, 0x00 };
static char TFT_LCD_ARRAY_CMD271[] = { 0xF4, 0x65 };
static char TFT_LCD_ARRAY_CMD272[] = { 0xF5, 0x00 };
static char TFT_LCD_ARRAY_CMD273[] = { 0xF6, 0x84 };
static char TFT_LCD_ARRAY_CMD274[] = { 0xF7, 0x00 };
static char TFT_LCD_ARRAY_CMD275[] = { 0xF8, 0x9B };
static char TFT_LCD_ARRAY_CMD276[] = { 0xF9, 0x00 };
static char TFT_LCD_ARRAY_CMD277[] = { 0xFA, 0xAF };
static char TFT_LCD_ARRAY_CMD278[] = { 0xFF, 0x02 };
static char TFT_LCD_ARRAY_CMD279[] = { 0xFB, 0x01 };
static char TFT_LCD_ARRAY_CMD280[] = { 0x00, 0x00 };
static char TFT_LCD_ARRAY_CMD281[] = { 0x01, 0xC1 };
static char TFT_LCD_ARRAY_CMD282[] = { 0x02, 0x00 };
static char TFT_LCD_ARRAY_CMD283[] = { 0x03, 0xD2 };
static char TFT_LCD_ARRAY_CMD284[] = { 0x04, 0x00 };
static char TFT_LCD_ARRAY_CMD285[] = { 0x05, 0xDF };
static char TFT_LCD_ARRAY_CMD286[] = { 0x06, 0x01 };
static char TFT_LCD_ARRAY_CMD287[] = { 0x07, 0x11 };
static char TFT_LCD_ARRAY_CMD288[] = { 0x08, 0x01 };
static char TFT_LCD_ARRAY_CMD289[] = { 0x09, 0x38 };
static char TFT_LCD_ARRAY_CMD290[] = { 0x0A, 0x01 };
static char TFT_LCD_ARRAY_CMD291[] = { 0x0B, 0x76 };
static char TFT_LCD_ARRAY_CMD292[] = { 0x0C, 0x01 };
static char TFT_LCD_ARRAY_CMD293[] = { 0x0D, 0xA7 };
static char TFT_LCD_ARRAY_CMD294[] = { 0x0E, 0x01 };
static char TFT_LCD_ARRAY_CMD295[] = { 0x0F, 0xF3 };
static char TFT_LCD_ARRAY_CMD296[] = { 0x10, 0x02 };
static char TFT_LCD_ARRAY_CMD297[] = { 0x11, 0x2F };
static char TFT_LCD_ARRAY_CMD298[] = { 0x12, 0x02 };
static char TFT_LCD_ARRAY_CMD299[] = { 0x13, 0x30 };
static char TFT_LCD_ARRAY_CMD300[] = { 0x14, 0x02 };
static char TFT_LCD_ARRAY_CMD301[] = { 0x15, 0x66 };
static char TFT_LCD_ARRAY_CMD302[] = { 0x16, 0x02 };
static char TFT_LCD_ARRAY_CMD303[] = { 0x17, 0xA0 };
static char TFT_LCD_ARRAY_CMD304[] = { 0x18, 0x02 };
static char TFT_LCD_ARRAY_CMD305[] = { 0x19, 0xC5 };
static char TFT_LCD_ARRAY_CMD306[] = { 0x1A, 0x02 };
static char TFT_LCD_ARRAY_CMD307[] = { 0x1B, 0xF8 };
static char TFT_LCD_ARRAY_CMD308[] = { 0x1C, 0x03 };
static char TFT_LCD_ARRAY_CMD309[] = { 0x1D, 0x1B };
static char TFT_LCD_ARRAY_CMD310[] = { 0x1E, 0x03 };
static char TFT_LCD_ARRAY_CMD311[] = { 0x1F, 0x46 };
static char TFT_LCD_ARRAY_CMD312[] = { 0x20, 0x03 };
static char TFT_LCD_ARRAY_CMD313[] = { 0x21, 0x52 };
static char TFT_LCD_ARRAY_CMD314[] = { 0x22, 0x03 };
static char TFT_LCD_ARRAY_CMD315[] = { 0x23, 0x62 };
static char TFT_LCD_ARRAY_CMD316[] = { 0x24, 0x03 };
static char TFT_LCD_ARRAY_CMD317[] = { 0x25, 0x71 };
static char TFT_LCD_ARRAY_CMD318[] = { 0x26, 0x03 };
static char TFT_LCD_ARRAY_CMD319[] = { 0x27, 0x83 };
static char TFT_LCD_ARRAY_CMD320[] = { 0x28, 0x03 };
static char TFT_LCD_ARRAY_CMD321[] = { 0x29, 0x94 };
static char TFT_LCD_ARRAY_CMD322[] = { 0x2A, 0x03 };
static char TFT_LCD_ARRAY_CMD323[] = { 0x2B, 0xA3 };
static char TFT_LCD_ARRAY_CMD324[] = { 0x2D, 0x03 };
static char TFT_LCD_ARRAY_CMD325[] = { 0x2F, 0xAD };
static char TFT_LCD_ARRAY_CMD326[] = { 0x30, 0x03 };
static char TFT_LCD_ARRAY_CMD327[] = { 0x31, 0xCC };
static char TFT_LCD_ARRAY_CMD328[] = { 0x32, 0x00 };
static char TFT_LCD_ARRAY_CMD329[] = { 0x33, 0x18 };
static char TFT_LCD_ARRAY_CMD330[] = { 0x34, 0x00 };
static char TFT_LCD_ARRAY_CMD331[] = { 0x35, 0x38 };
static char TFT_LCD_ARRAY_CMD332[] = { 0x36, 0x00 };
static char TFT_LCD_ARRAY_CMD333[] = { 0x37, 0x65 };
static char TFT_LCD_ARRAY_CMD334[] = { 0x38, 0x00 };
static char TFT_LCD_ARRAY_CMD335[] = { 0x39, 0x84 };
static char TFT_LCD_ARRAY_CMD336[] = { 0x3A, 0x00 };
static char TFT_LCD_ARRAY_CMD337[] = { 0x3B, 0x9B };
static char TFT_LCD_ARRAY_CMD338[] = { 0x3D, 0x00 };
static char TFT_LCD_ARRAY_CMD339[] = { 0x3F, 0xAF };
static char TFT_LCD_ARRAY_CMD340[] = { 0x40, 0x00 };
static char TFT_LCD_ARRAY_CMD341[] = { 0x41, 0xC1 };
static char TFT_LCD_ARRAY_CMD342[] = { 0x42, 0x00 };
static char TFT_LCD_ARRAY_CMD343[] = { 0x43, 0xD2 };
static char TFT_LCD_ARRAY_CMD344[] = { 0x44, 0x00 };
static char TFT_LCD_ARRAY_CMD345[] = { 0x45, 0xDF };
static char TFT_LCD_ARRAY_CMD346[] = { 0x46, 0x01 };
static char TFT_LCD_ARRAY_CMD347[] = { 0x47, 0x11 };
static char TFT_LCD_ARRAY_CMD348[] = { 0x48, 0x01 };
static char TFT_LCD_ARRAY_CMD349[] = { 0x49, 0x38 };
static char TFT_LCD_ARRAY_CMD350[] = { 0x4A, 0x01 };
static char TFT_LCD_ARRAY_CMD351[] = { 0x4B, 0x76 };
static char TFT_LCD_ARRAY_CMD352[] = { 0x4C, 0x01 };
static char TFT_LCD_ARRAY_CMD353[] = { 0x4D, 0xA7 };
static char TFT_LCD_ARRAY_CMD354[] = { 0x4E, 0x01 };
static char TFT_LCD_ARRAY_CMD355[] = { 0x4F, 0xF3 };
static char TFT_LCD_ARRAY_CMD356[] = { 0x50, 0x02 };
static char TFT_LCD_ARRAY_CMD357[] = { 0x51, 0x2F };
static char TFT_LCD_ARRAY_CMD358[] = { 0x52, 0x02 };
static char TFT_LCD_ARRAY_CMD359[] = { 0x53, 0x30 };
static char TFT_LCD_ARRAY_CMD360[] = { 0x54, 0x02 };
static char TFT_LCD_ARRAY_CMD361[] = { 0x55, 0x66 };
static char TFT_LCD_ARRAY_CMD362[] = { 0x56, 0x02 };
static char TFT_LCD_ARRAY_CMD363[] = { 0x58, 0xA0 };
static char TFT_LCD_ARRAY_CMD364[] = { 0x59, 0x02 };
static char TFT_LCD_ARRAY_CMD365[] = { 0x5A, 0xC5 };
static char TFT_LCD_ARRAY_CMD366[] = { 0x5B, 0x02 };
static char TFT_LCD_ARRAY_CMD367[] = { 0x5C, 0xF8 };
static char TFT_LCD_ARRAY_CMD368[] = { 0x5D, 0x03 };
static char TFT_LCD_ARRAY_CMD369[] = { 0x5E, 0x1B };
static char TFT_LCD_ARRAY_CMD370[] = { 0x5F, 0x03 };
static char TFT_LCD_ARRAY_CMD371[] = { 0x60, 0x46 };
static char TFT_LCD_ARRAY_CMD372[] = { 0x61, 0x03 };
static char TFT_LCD_ARRAY_CMD373[] = { 0x62, 0x52 };
static char TFT_LCD_ARRAY_CMD374[] = { 0x63, 0x03 };
static char TFT_LCD_ARRAY_CMD375[] = { 0x64, 0x62 };
static char TFT_LCD_ARRAY_CMD376[] = { 0x65, 0x03 };
static char TFT_LCD_ARRAY_CMD377[] = { 0x66, 0x71 };
static char TFT_LCD_ARRAY_CMD378[] = { 0x67, 0x03 };
static char TFT_LCD_ARRAY_CMD379[] = { 0x68, 0x83 };
static char TFT_LCD_ARRAY_CMD380[] = { 0x69, 0x03 };
static char TFT_LCD_ARRAY_CMD381[] = { 0x6A, 0x94 };
static char TFT_LCD_ARRAY_CMD382[] = { 0x6B, 0x03 };
static char TFT_LCD_ARRAY_CMD383[] = { 0x6C, 0xA3 };
static char TFT_LCD_ARRAY_CMD384[] = { 0x6D, 0x03 };
static char TFT_LCD_ARRAY_CMD385[] = { 0x6E, 0xAD };
static char TFT_LCD_ARRAY_CMD386[] = { 0x6F, 0x03 };
static char TFT_LCD_ARRAY_CMD387[] = { 0x70, 0xCC };
static char TFT_LCD_ARRAY_CMD388[] = { 0x71, 0x00 };
static char TFT_LCD_ARRAY_CMD389[] = { 0x72, 0x18 };
static char TFT_LCD_ARRAY_CMD390[] = { 0x73, 0x00 };
static char TFT_LCD_ARRAY_CMD391[] = { 0x74, 0x38 };
static char TFT_LCD_ARRAY_CMD392[] = { 0x75, 0x00 };
static char TFT_LCD_ARRAY_CMD393[] = { 0x76, 0x65 };
static char TFT_LCD_ARRAY_CMD394[] = { 0x77, 0x00 };
static char TFT_LCD_ARRAY_CMD395[] = { 0x78, 0x84 };
static char TFT_LCD_ARRAY_CMD396[] = { 0x79, 0x00 };
static char TFT_LCD_ARRAY_CMD397[] = { 0x7A, 0x9B };
static char TFT_LCD_ARRAY_CMD398[] = { 0x7B, 0x00 };
static char TFT_LCD_ARRAY_CMD399[] = { 0x7C, 0xAF };
static char TFT_LCD_ARRAY_CMD400[] = { 0x7D, 0x00 };
static char TFT_LCD_ARRAY_CMD401[] = { 0x7E, 0xC1 };
static char TFT_LCD_ARRAY_CMD402[] = { 0x7F, 0x00 };
static char TFT_LCD_ARRAY_CMD403[] = { 0x80, 0xD2 };
static char TFT_LCD_ARRAY_CMD404[] = { 0x81, 0x00 };
static char TFT_LCD_ARRAY_CMD405[] = { 0x82, 0xDF };
static char TFT_LCD_ARRAY_CMD406[] = { 0x83, 0x01 };
static char TFT_LCD_ARRAY_CMD407[] = { 0x84, 0x11 };
static char TFT_LCD_ARRAY_CMD408[] = { 0x85, 0x01 };
static char TFT_LCD_ARRAY_CMD409[] = { 0x86, 0x38 };
static char TFT_LCD_ARRAY_CMD410[] = { 0x87, 0x01 };
static char TFT_LCD_ARRAY_CMD411[] = { 0x88, 0x76 };
static char TFT_LCD_ARRAY_CMD412[] = { 0x89, 0x01 };
static char TFT_LCD_ARRAY_CMD413[] = { 0x8A, 0xA7 };
static char TFT_LCD_ARRAY_CMD414[] = { 0x8B, 0x01 };
static char TFT_LCD_ARRAY_CMD415[] = { 0x8C, 0xF3 };
static char TFT_LCD_ARRAY_CMD416[] = { 0x8D, 0x02 };
static char TFT_LCD_ARRAY_CMD417[] = { 0x8E, 0x2F };
static char TFT_LCD_ARRAY_CMD418[] = { 0x8F, 0x02 };
static char TFT_LCD_ARRAY_CMD419[] = { 0x90, 0x30 };
static char TFT_LCD_ARRAY_CMD420[] = { 0x91, 0x02 };
static char TFT_LCD_ARRAY_CMD421[] = { 0x92, 0x66 };
static char TFT_LCD_ARRAY_CMD422[] = { 0x93, 0x02 };
static char TFT_LCD_ARRAY_CMD423[] = { 0x94, 0xA0 };
static char TFT_LCD_ARRAY_CMD424[] = { 0x95, 0x02 };
static char TFT_LCD_ARRAY_CMD425[] = { 0x96, 0xC5 };
static char TFT_LCD_ARRAY_CMD426[] = { 0x97, 0x02 };
static char TFT_LCD_ARRAY_CMD427[] = { 0x98, 0xF8 };
static char TFT_LCD_ARRAY_CMD428[] = { 0x99, 0x03 };
static char TFT_LCD_ARRAY_CMD429[] = { 0x9A, 0x1B };
static char TFT_LCD_ARRAY_CMD430[] = { 0x9B, 0x03 };
static char TFT_LCD_ARRAY_CMD431[] = { 0x9C, 0x46 };
static char TFT_LCD_ARRAY_CMD432[] = { 0x9D, 0x03 };
static char TFT_LCD_ARRAY_CMD433[] = { 0x9E, 0x52 };
static char TFT_LCD_ARRAY_CMD434[] = { 0x9F, 0x03 };
static char TFT_LCD_ARRAY_CMD435[] = { 0xA0, 0x62 };
static char TFT_LCD_ARRAY_CMD436[] = { 0xA2, 0x03 };
static char TFT_LCD_ARRAY_CMD437[] = { 0xA3, 0x71 };
static char TFT_LCD_ARRAY_CMD438[] = { 0xA4, 0x03 };
static char TFT_LCD_ARRAY_CMD439[] = { 0xA5, 0x83 };
static char TFT_LCD_ARRAY_CMD440[] = { 0xA6, 0x03 };
static char TFT_LCD_ARRAY_CMD441[] = { 0xA7, 0x94 };
static char TFT_LCD_ARRAY_CMD442[] = { 0xA9, 0x03 };
static char TFT_LCD_ARRAY_CMD443[] = { 0xAA, 0xA3 };
static char TFT_LCD_ARRAY_CMD444[] = { 0xAB, 0x03 };
static char TFT_LCD_ARRAY_CMD445[] = { 0xAC, 0xAD };
static char TFT_LCD_ARRAY_CMD446[] = { 0xAD, 0x03 };
static char TFT_LCD_ARRAY_CMD447[] = { 0xAE, 0xCC };
static char TFT_LCD_ARRAY_CMD448[] = { 0xAF, 0x00 };
static char TFT_LCD_ARRAY_CMD449[] = { 0xB0, 0x18 };
static char TFT_LCD_ARRAY_CMD450[] = { 0xB1, 0x00 };
static char TFT_LCD_ARRAY_CMD451[] = { 0xB2, 0x38 };
static char TFT_LCD_ARRAY_CMD452[] = { 0xB3, 0x00 };
static char TFT_LCD_ARRAY_CMD453[] = { 0xB4, 0x65 };
static char TFT_LCD_ARRAY_CMD454[] = { 0xB5, 0x00 };
static char TFT_LCD_ARRAY_CMD455[] = { 0xB6, 0x84 };
static char TFT_LCD_ARRAY_CMD456[] = { 0xB7, 0x00 };
static char TFT_LCD_ARRAY_CMD457[] = { 0xB8, 0x9B };
static char TFT_LCD_ARRAY_CMD458[] = { 0xB9, 0x00 };
static char TFT_LCD_ARRAY_CMD459[] = { 0xBA, 0xAF };
static char TFT_LCD_ARRAY_CMD460[] = { 0xBB, 0x00 };
static char TFT_LCD_ARRAY_CMD461[] = { 0xBC, 0xC1 };
static char TFT_LCD_ARRAY_CMD462[] = { 0xBD, 0x00 };
static char TFT_LCD_ARRAY_CMD463[] = { 0xBE, 0xD2 };
static char TFT_LCD_ARRAY_CMD464[] = { 0xBF, 0x00 };
static char TFT_LCD_ARRAY_CMD465[] = { 0xC0, 0xDF };
static char TFT_LCD_ARRAY_CMD466[] = { 0xC1, 0x01 };
static char TFT_LCD_ARRAY_CMD467[] = { 0xC2, 0x11 };
static char TFT_LCD_ARRAY_CMD468[] = { 0xC3, 0x01 };
static char TFT_LCD_ARRAY_CMD469[] = { 0xC4, 0x38 };
static char TFT_LCD_ARRAY_CMD470[] = { 0xC5, 0x01 };
static char TFT_LCD_ARRAY_CMD471[] = { 0xC6, 0x76 };
static char TFT_LCD_ARRAY_CMD472[] = { 0xC7, 0x01 };
static char TFT_LCD_ARRAY_CMD473[] = { 0xC8, 0xA7 };
static char TFT_LCD_ARRAY_CMD474[] = { 0xC9, 0x01 };
static char TFT_LCD_ARRAY_CMD475[] = { 0xCA, 0xF3 };
static char TFT_LCD_ARRAY_CMD476[] = { 0xCB, 0x02 };
static char TFT_LCD_ARRAY_CMD477[] = { 0xCC, 0x2F };
static char TFT_LCD_ARRAY_CMD478[] = { 0xCD, 0x02 };
static char TFT_LCD_ARRAY_CMD479[] = { 0xCE, 0x30 };
static char TFT_LCD_ARRAY_CMD480[] = { 0xCF, 0x02 };
static char TFT_LCD_ARRAY_CMD481[] = { 0xD0, 0x66 };
static char TFT_LCD_ARRAY_CMD482[] = { 0xD1, 0x02 };
static char TFT_LCD_ARRAY_CMD483[] = { 0xD2, 0xA0 };
static char TFT_LCD_ARRAY_CMD484[] = { 0xD3, 0x02 };
static char TFT_LCD_ARRAY_CMD485[] = { 0xD4, 0xC5 };
static char TFT_LCD_ARRAY_CMD486[] = { 0xD5, 0x02 };
static char TFT_LCD_ARRAY_CMD487[] = { 0xD6, 0xF8 };
static char TFT_LCD_ARRAY_CMD488[] = { 0xD7, 0x03 };
static char TFT_LCD_ARRAY_CMD489[] = { 0xD8, 0x1B };
static char TFT_LCD_ARRAY_CMD490[] = { 0xD9, 0x03 };
static char TFT_LCD_ARRAY_CMD491[] = { 0xDA, 0x46 };
static char TFT_LCD_ARRAY_CMD492[] = { 0xDB, 0x03 };
static char TFT_LCD_ARRAY_CMD493[] = { 0xDC, 0x52 };
static char TFT_LCD_ARRAY_CMD494[] = { 0xDD, 0x03 };
static char TFT_LCD_ARRAY_CMD495[] = { 0xDE, 0x62 };
static char TFT_LCD_ARRAY_CMD496[] = { 0xDF, 0x03 };
static char TFT_LCD_ARRAY_CMD497[] = { 0xE0, 0x71 };
static char TFT_LCD_ARRAY_CMD498[] = { 0xE1, 0x03 };
static char TFT_LCD_ARRAY_CMD499[] = { 0xE2, 0x83 };
static char TFT_LCD_ARRAY_CMD500[] = { 0xE3, 0x03 };
static char TFT_LCD_ARRAY_CMD501[] = { 0xE4, 0x94 };
static char TFT_LCD_ARRAY_CMD502[] = { 0xE5, 0x03 };
static char TFT_LCD_ARRAY_CMD503[] = { 0xE6, 0xA3 };
static char TFT_LCD_ARRAY_CMD504[] = { 0xE7, 0x03 };
static char TFT_LCD_ARRAY_CMD505[] = { 0xE8, 0xAD };
static char TFT_LCD_ARRAY_CMD506[] = { 0xE9, 0x03 };
static char TFT_LCD_ARRAY_CMD507[] = { 0xEA, 0xCC };
static char TFT_LCD_ARRAY_CMD508[] = { 0xFF, 0x01 };
static char TFT_LCD_ARRAY_CMD509[] = { 0xFB, 0x01 };
static char TFT_LCD_ARRAY_CMD510[] = { 0xFF, 0x02 };
static char TFT_LCD_ARRAY_CMD511[] = { 0xFB, 0x01 };
static char TFT_LCD_ARRAY_CMD512[] = { 0xFF, 0x04 };
static char TFT_LCD_ARRAY_CMD513[] = { 0xFB, 0x01 };
static char TFT_LCD_ARRAY_CMD514[] = { 0xFF, 0x00 };
static char TFT_LCD_ARRAY_CMD515[] = { 0xD3, 0x14 };
static char TFT_LCD_ARRAY_CMD516[] = { 0xD4, 0x14 };
static char TFT_LCD_ARRAY_CMD517[] = { 0x11 };
static char TFT_LCD_ARRAY_CMD518[] = { 0xFF, 0x00 };
static char TFT_LCD_ARRAY_CMD519[] = { 0x34, 0x00 };
static char TFT_LCD_ARRAY_CMD520[] = { 0x35, 0x00 };
static char TFT_LCD_ARRAY_CMD521[] = { 0x29 };

static struct mmp_dsi_cmd_desc tft_1e_display_on_cmds[] = {

	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD1), TFT_LCD_ARRAY_CMD1}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD2), TFT_LCD_ARRAY_CMD2}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD4), TFT_LCD_ARRAY_CMD4}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD5), TFT_LCD_ARRAY_CMD5}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD6), TFT_LCD_ARRAY_CMD6}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD7), TFT_LCD_ARRAY_CMD7}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD8), TFT_LCD_ARRAY_CMD8}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD9), TFT_LCD_ARRAY_CMD9}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD10), TFT_LCD_ARRAY_CMD10}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD11), TFT_LCD_ARRAY_CMD11}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD16), TFT_LCD_ARRAY_CMD16}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD17), TFT_LCD_ARRAY_CMD17}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD18), TFT_LCD_ARRAY_CMD18}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD19), TFT_LCD_ARRAY_CMD19}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD20), TFT_LCD_ARRAY_CMD20}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD21), TFT_LCD_ARRAY_CMD21}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD22), TFT_LCD_ARRAY_CMD22}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD23), TFT_LCD_ARRAY_CMD23}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD24), TFT_LCD_ARRAY_CMD24}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD25), TFT_LCD_ARRAY_CMD25}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD26), TFT_LCD_ARRAY_CMD26}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD27), TFT_LCD_ARRAY_CMD27}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD28), TFT_LCD_ARRAY_CMD28}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD29), TFT_LCD_ARRAY_CMD29}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD30), TFT_LCD_ARRAY_CMD30}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD31), TFT_LCD_ARRAY_CMD31}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD32), TFT_LCD_ARRAY_CMD32}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD33), TFT_LCD_ARRAY_CMD33}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD34), TFT_LCD_ARRAY_CMD34}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD35), TFT_LCD_ARRAY_CMD35}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD36), TFT_LCD_ARRAY_CMD36}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD37), TFT_LCD_ARRAY_CMD37}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD38), TFT_LCD_ARRAY_CMD38}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD39), TFT_LCD_ARRAY_CMD39}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD40), TFT_LCD_ARRAY_CMD40}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD41), TFT_LCD_ARRAY_CMD41}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD42), TFT_LCD_ARRAY_CMD42}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD43), TFT_LCD_ARRAY_CMD43}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD44), TFT_LCD_ARRAY_CMD44}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD45), TFT_LCD_ARRAY_CMD45}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD46), TFT_LCD_ARRAY_CMD46}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD47), TFT_LCD_ARRAY_CMD47}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD48), TFT_LCD_ARRAY_CMD48}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD49), TFT_LCD_ARRAY_CMD49}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD50), TFT_LCD_ARRAY_CMD50}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD51), TFT_LCD_ARRAY_CMD51}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD52), TFT_LCD_ARRAY_CMD52}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD53), TFT_LCD_ARRAY_CMD53}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD54), TFT_LCD_ARRAY_CMD54}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD55), TFT_LCD_ARRAY_CMD55}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD56), TFT_LCD_ARRAY_CMD56}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD57), TFT_LCD_ARRAY_CMD57}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD58), TFT_LCD_ARRAY_CMD58}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD59), TFT_LCD_ARRAY_CMD59}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD60), TFT_LCD_ARRAY_CMD60}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD61), TFT_LCD_ARRAY_CMD61}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD62), TFT_LCD_ARRAY_CMD62}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD63), TFT_LCD_ARRAY_CMD63}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD64), TFT_LCD_ARRAY_CMD64}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD65), TFT_LCD_ARRAY_CMD65}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD66), TFT_LCD_ARRAY_CMD66}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD67), TFT_LCD_ARRAY_CMD67}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD68), TFT_LCD_ARRAY_CMD68}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD69), TFT_LCD_ARRAY_CMD69}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD70), TFT_LCD_ARRAY_CMD70}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD71), TFT_LCD_ARRAY_CMD71}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD72), TFT_LCD_ARRAY_CMD72}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD73), TFT_LCD_ARRAY_CMD73}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD74), TFT_LCD_ARRAY_CMD74}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD75), TFT_LCD_ARRAY_CMD75}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD76), TFT_LCD_ARRAY_CMD76}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD77), TFT_LCD_ARRAY_CMD77}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD78), TFT_LCD_ARRAY_CMD78}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD79), TFT_LCD_ARRAY_CMD79}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD80), TFT_LCD_ARRAY_CMD80}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD81), TFT_LCD_ARRAY_CMD81}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD82), TFT_LCD_ARRAY_CMD82}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD83), TFT_LCD_ARRAY_CMD83}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD84), TFT_LCD_ARRAY_CMD84}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD85), TFT_LCD_ARRAY_CMD85}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD86), TFT_LCD_ARRAY_CMD86}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD87), TFT_LCD_ARRAY_CMD87}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD88), TFT_LCD_ARRAY_CMD88}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD89), TFT_LCD_ARRAY_CMD89}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD90), TFT_LCD_ARRAY_CMD90}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD91), TFT_LCD_ARRAY_CMD91}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD92), TFT_LCD_ARRAY_CMD92}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD93), TFT_LCD_ARRAY_CMD93}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD94), TFT_LCD_ARRAY_CMD94}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD95), TFT_LCD_ARRAY_CMD95}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD96), TFT_LCD_ARRAY_CMD96}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD97), TFT_LCD_ARRAY_CMD97}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD98), TFT_LCD_ARRAY_CMD98}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD99), TFT_LCD_ARRAY_CMD99}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD100), TFT_LCD_ARRAY_CMD100}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD101), TFT_LCD_ARRAY_CMD101}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD102), TFT_LCD_ARRAY_CMD102}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD103), TFT_LCD_ARRAY_CMD103}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD104), TFT_LCD_ARRAY_CMD104}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD105), TFT_LCD_ARRAY_CMD105}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD106), TFT_LCD_ARRAY_CMD106}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD107), TFT_LCD_ARRAY_CMD107}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD108), TFT_LCD_ARRAY_CMD108}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD109), TFT_LCD_ARRAY_CMD109}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD110), TFT_LCD_ARRAY_CMD110}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD111), TFT_LCD_ARRAY_CMD111}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD112), TFT_LCD_ARRAY_CMD112}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD113), TFT_LCD_ARRAY_CMD113}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD114), TFT_LCD_ARRAY_CMD114}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD115), TFT_LCD_ARRAY_CMD115}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD116), TFT_LCD_ARRAY_CMD116}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD117), TFT_LCD_ARRAY_CMD117}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD118), TFT_LCD_ARRAY_CMD118}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD119), TFT_LCD_ARRAY_CMD119}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD120), TFT_LCD_ARRAY_CMD120}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD121), TFT_LCD_ARRAY_CMD121}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD122), TFT_LCD_ARRAY_CMD122}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD123), TFT_LCD_ARRAY_CMD123}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD124), TFT_LCD_ARRAY_CMD124}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD125), TFT_LCD_ARRAY_CMD125}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD126), TFT_LCD_ARRAY_CMD126}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD127), TFT_LCD_ARRAY_CMD127}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD128), TFT_LCD_ARRAY_CMD128}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD129), TFT_LCD_ARRAY_CMD129}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD130), TFT_LCD_ARRAY_CMD130}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD131), TFT_LCD_ARRAY_CMD131}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD132), TFT_LCD_ARRAY_CMD132}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD133), TFT_LCD_ARRAY_CMD133}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD134), TFT_LCD_ARRAY_CMD134}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD135), TFT_LCD_ARRAY_CMD135}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD136), TFT_LCD_ARRAY_CMD136}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD137), TFT_LCD_ARRAY_CMD137}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD138), TFT_LCD_ARRAY_CMD138}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD139), TFT_LCD_ARRAY_CMD139}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD140), TFT_LCD_ARRAY_CMD140}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD141), TFT_LCD_ARRAY_CMD141}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD144), TFT_LCD_ARRAY_CMD144}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD145), TFT_LCD_ARRAY_CMD145}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD146), TFT_LCD_ARRAY_CMD146}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD147), TFT_LCD_ARRAY_CMD147}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD148), TFT_LCD_ARRAY_CMD148}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD149), TFT_LCD_ARRAY_CMD149}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD150), TFT_LCD_ARRAY_CMD150}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD151), TFT_LCD_ARRAY_CMD151}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD152), TFT_LCD_ARRAY_CMD152}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD153), TFT_LCD_ARRAY_CMD153}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD154), TFT_LCD_ARRAY_CMD154}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD155), TFT_LCD_ARRAY_CMD155}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD156), TFT_LCD_ARRAY_CMD156}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD157), TFT_LCD_ARRAY_CMD157}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD158), TFT_LCD_ARRAY_CMD158}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD159), TFT_LCD_ARRAY_CMD159}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD160), TFT_LCD_ARRAY_CMD160}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD161), TFT_LCD_ARRAY_CMD161}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD162), TFT_LCD_ARRAY_CMD162}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD163), TFT_LCD_ARRAY_CMD163}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD164), TFT_LCD_ARRAY_CMD164}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD165), TFT_LCD_ARRAY_CMD165}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD166), TFT_LCD_ARRAY_CMD166}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD167), TFT_LCD_ARRAY_CMD167}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD168), TFT_LCD_ARRAY_CMD168}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD169), TFT_LCD_ARRAY_CMD169}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD170), TFT_LCD_ARRAY_CMD170}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD171), TFT_LCD_ARRAY_CMD171}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD172), TFT_LCD_ARRAY_CMD172}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD173), TFT_LCD_ARRAY_CMD173}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD174), TFT_LCD_ARRAY_CMD174}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD175), TFT_LCD_ARRAY_CMD175}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD176), TFT_LCD_ARRAY_CMD176}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD177), TFT_LCD_ARRAY_CMD177}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD178), TFT_LCD_ARRAY_CMD178}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD179), TFT_LCD_ARRAY_CMD179}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD180), TFT_LCD_ARRAY_CMD180}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD181), TFT_LCD_ARRAY_CMD181}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD182), TFT_LCD_ARRAY_CMD182}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD183), TFT_LCD_ARRAY_CMD183}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD184), TFT_LCD_ARRAY_CMD184}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD185), TFT_LCD_ARRAY_CMD185}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD186), TFT_LCD_ARRAY_CMD186}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD187), TFT_LCD_ARRAY_CMD187}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD188), TFT_LCD_ARRAY_CMD188}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD189), TFT_LCD_ARRAY_CMD189}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD190), TFT_LCD_ARRAY_CMD190}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD191), TFT_LCD_ARRAY_CMD191}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD192), TFT_LCD_ARRAY_CMD192}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD193), TFT_LCD_ARRAY_CMD193}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD194), TFT_LCD_ARRAY_CMD194}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD195), TFT_LCD_ARRAY_CMD195}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD196), TFT_LCD_ARRAY_CMD196}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD197), TFT_LCD_ARRAY_CMD197}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD198), TFT_LCD_ARRAY_CMD198}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD199), TFT_LCD_ARRAY_CMD199}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD200), TFT_LCD_ARRAY_CMD200}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD201), TFT_LCD_ARRAY_CMD201}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD202), TFT_LCD_ARRAY_CMD202}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD203), TFT_LCD_ARRAY_CMD203}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD204), TFT_LCD_ARRAY_CMD204}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD205), TFT_LCD_ARRAY_CMD205}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD206), TFT_LCD_ARRAY_CMD206}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD207), TFT_LCD_ARRAY_CMD207}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD208), TFT_LCD_ARRAY_CMD208}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD209), TFT_LCD_ARRAY_CMD209}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD210), TFT_LCD_ARRAY_CMD210}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD211), TFT_LCD_ARRAY_CMD211}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD212), TFT_LCD_ARRAY_CMD212}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD213), TFT_LCD_ARRAY_CMD213}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD214), TFT_LCD_ARRAY_CMD214}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD215), TFT_LCD_ARRAY_CMD215}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD216), TFT_LCD_ARRAY_CMD216}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD217), TFT_LCD_ARRAY_CMD217}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD218), TFT_LCD_ARRAY_CMD218}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD219), TFT_LCD_ARRAY_CMD219}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD220), TFT_LCD_ARRAY_CMD220}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD221), TFT_LCD_ARRAY_CMD221}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD222), TFT_LCD_ARRAY_CMD222}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD223), TFT_LCD_ARRAY_CMD223}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD224), TFT_LCD_ARRAY_CMD224}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD225), TFT_LCD_ARRAY_CMD225}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD226), TFT_LCD_ARRAY_CMD226}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD227), TFT_LCD_ARRAY_CMD227}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD228), TFT_LCD_ARRAY_CMD228}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD229), TFT_LCD_ARRAY_CMD229}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD230), TFT_LCD_ARRAY_CMD230}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD231), TFT_LCD_ARRAY_CMD231}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD232), TFT_LCD_ARRAY_CMD232}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD233), TFT_LCD_ARRAY_CMD233}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD234), TFT_LCD_ARRAY_CMD234}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD235), TFT_LCD_ARRAY_CMD235}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD236), TFT_LCD_ARRAY_CMD236}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD237), TFT_LCD_ARRAY_CMD237}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD238), TFT_LCD_ARRAY_CMD238}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD239), TFT_LCD_ARRAY_CMD239}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD240), TFT_LCD_ARRAY_CMD240}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD241), TFT_LCD_ARRAY_CMD241}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD242), TFT_LCD_ARRAY_CMD242}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD243), TFT_LCD_ARRAY_CMD243}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD244), TFT_LCD_ARRAY_CMD244}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD245), TFT_LCD_ARRAY_CMD245}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD246), TFT_LCD_ARRAY_CMD246}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD247), TFT_LCD_ARRAY_CMD247}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD248), TFT_LCD_ARRAY_CMD248}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD249), TFT_LCD_ARRAY_CMD249}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD250), TFT_LCD_ARRAY_CMD250}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD251), TFT_LCD_ARRAY_CMD251}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD252), TFT_LCD_ARRAY_CMD252}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD253), TFT_LCD_ARRAY_CMD253}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD254), TFT_LCD_ARRAY_CMD254}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD255), TFT_LCD_ARRAY_CMD255}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD256), TFT_LCD_ARRAY_CMD256}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD257), TFT_LCD_ARRAY_CMD257}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD258), TFT_LCD_ARRAY_CMD258}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD259), TFT_LCD_ARRAY_CMD259}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD260), TFT_LCD_ARRAY_CMD260}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD261), TFT_LCD_ARRAY_CMD261}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD262), TFT_LCD_ARRAY_CMD262}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD263), TFT_LCD_ARRAY_CMD263}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD264), TFT_LCD_ARRAY_CMD264}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD265), TFT_LCD_ARRAY_CMD265}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD266), TFT_LCD_ARRAY_CMD266}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD267), TFT_LCD_ARRAY_CMD267}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD268), TFT_LCD_ARRAY_CMD268}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD269), TFT_LCD_ARRAY_CMD269}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD270), TFT_LCD_ARRAY_CMD270}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD271), TFT_LCD_ARRAY_CMD271}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD272), TFT_LCD_ARRAY_CMD272}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD273), TFT_LCD_ARRAY_CMD273}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD274), TFT_LCD_ARRAY_CMD274}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD275), TFT_LCD_ARRAY_CMD275}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD276), TFT_LCD_ARRAY_CMD276}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD277), TFT_LCD_ARRAY_CMD277}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD278), TFT_LCD_ARRAY_CMD278}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD279), TFT_LCD_ARRAY_CMD279}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD280), TFT_LCD_ARRAY_CMD280}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD281), TFT_LCD_ARRAY_CMD281}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD282), TFT_LCD_ARRAY_CMD282}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD283), TFT_LCD_ARRAY_CMD283}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD284), TFT_LCD_ARRAY_CMD284}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD285), TFT_LCD_ARRAY_CMD285}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD286), TFT_LCD_ARRAY_CMD286}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD287), TFT_LCD_ARRAY_CMD287}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD288), TFT_LCD_ARRAY_CMD288}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD289), TFT_LCD_ARRAY_CMD289}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD290), TFT_LCD_ARRAY_CMD290}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD291), TFT_LCD_ARRAY_CMD291}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD292), TFT_LCD_ARRAY_CMD292}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD293), TFT_LCD_ARRAY_CMD293}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD294), TFT_LCD_ARRAY_CMD294}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD295), TFT_LCD_ARRAY_CMD295}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD296), TFT_LCD_ARRAY_CMD296}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD297), TFT_LCD_ARRAY_CMD297}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD298), TFT_LCD_ARRAY_CMD298}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD299), TFT_LCD_ARRAY_CMD299}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD300), TFT_LCD_ARRAY_CMD300}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD301), TFT_LCD_ARRAY_CMD301}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD302), TFT_LCD_ARRAY_CMD302}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD303), TFT_LCD_ARRAY_CMD303}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD304), TFT_LCD_ARRAY_CMD304}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD305), TFT_LCD_ARRAY_CMD305}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD306), TFT_LCD_ARRAY_CMD306}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD307), TFT_LCD_ARRAY_CMD307}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD308), TFT_LCD_ARRAY_CMD308}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD309), TFT_LCD_ARRAY_CMD309}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD310), TFT_LCD_ARRAY_CMD310}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD311), TFT_LCD_ARRAY_CMD311}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD312), TFT_LCD_ARRAY_CMD312}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD313), TFT_LCD_ARRAY_CMD313}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD314), TFT_LCD_ARRAY_CMD314}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD315), TFT_LCD_ARRAY_CMD315}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD316), TFT_LCD_ARRAY_CMD316}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD317), TFT_LCD_ARRAY_CMD317}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD318), TFT_LCD_ARRAY_CMD318}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD319), TFT_LCD_ARRAY_CMD319}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD320), TFT_LCD_ARRAY_CMD320}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD321), TFT_LCD_ARRAY_CMD321}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD322), TFT_LCD_ARRAY_CMD322}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD323), TFT_LCD_ARRAY_CMD323}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD324), TFT_LCD_ARRAY_CMD324}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD325), TFT_LCD_ARRAY_CMD325}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD326), TFT_LCD_ARRAY_CMD326}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD327), TFT_LCD_ARRAY_CMD327}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD328), TFT_LCD_ARRAY_CMD328}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD329), TFT_LCD_ARRAY_CMD329}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD330), TFT_LCD_ARRAY_CMD330}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD331), TFT_LCD_ARRAY_CMD331}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD332), TFT_LCD_ARRAY_CMD332}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD333), TFT_LCD_ARRAY_CMD333}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD334), TFT_LCD_ARRAY_CMD334}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD335), TFT_LCD_ARRAY_CMD335}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD336), TFT_LCD_ARRAY_CMD336}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD337), TFT_LCD_ARRAY_CMD337}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD338), TFT_LCD_ARRAY_CMD338}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD339), TFT_LCD_ARRAY_CMD339}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD340), TFT_LCD_ARRAY_CMD340}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD341), TFT_LCD_ARRAY_CMD341}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD342), TFT_LCD_ARRAY_CMD342}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD343), TFT_LCD_ARRAY_CMD343}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD344), TFT_LCD_ARRAY_CMD344}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD345), TFT_LCD_ARRAY_CMD345}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD346), TFT_LCD_ARRAY_CMD346}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD347), TFT_LCD_ARRAY_CMD347}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD348), TFT_LCD_ARRAY_CMD348}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD349), TFT_LCD_ARRAY_CMD349}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD350), TFT_LCD_ARRAY_CMD350}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD351), TFT_LCD_ARRAY_CMD351}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD352), TFT_LCD_ARRAY_CMD352}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD353), TFT_LCD_ARRAY_CMD353}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD354), TFT_LCD_ARRAY_CMD354}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD355), TFT_LCD_ARRAY_CMD355}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD356), TFT_LCD_ARRAY_CMD356}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD357), TFT_LCD_ARRAY_CMD357}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD358), TFT_LCD_ARRAY_CMD358}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD359), TFT_LCD_ARRAY_CMD359}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD360), TFT_LCD_ARRAY_CMD360}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD361), TFT_LCD_ARRAY_CMD361}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD362), TFT_LCD_ARRAY_CMD362}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD363), TFT_LCD_ARRAY_CMD363}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD364), TFT_LCD_ARRAY_CMD364}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD365), TFT_LCD_ARRAY_CMD365}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD366), TFT_LCD_ARRAY_CMD366}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD367), TFT_LCD_ARRAY_CMD367}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD368), TFT_LCD_ARRAY_CMD368}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD369), TFT_LCD_ARRAY_CMD369}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD370), TFT_LCD_ARRAY_CMD370}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD371), TFT_LCD_ARRAY_CMD371}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD372), TFT_LCD_ARRAY_CMD372}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD373), TFT_LCD_ARRAY_CMD373}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD374), TFT_LCD_ARRAY_CMD374}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD375), TFT_LCD_ARRAY_CMD375}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD376), TFT_LCD_ARRAY_CMD376}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD377), TFT_LCD_ARRAY_CMD377}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD378), TFT_LCD_ARRAY_CMD378}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD379), TFT_LCD_ARRAY_CMD379}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD380), TFT_LCD_ARRAY_CMD380}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD381), TFT_LCD_ARRAY_CMD381}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD382), TFT_LCD_ARRAY_CMD382}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD383), TFT_LCD_ARRAY_CMD383}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD384), TFT_LCD_ARRAY_CMD384}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD385), TFT_LCD_ARRAY_CMD385}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD386), TFT_LCD_ARRAY_CMD386}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD387), TFT_LCD_ARRAY_CMD387}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD388), TFT_LCD_ARRAY_CMD388}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD389), TFT_LCD_ARRAY_CMD389}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD390), TFT_LCD_ARRAY_CMD390}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD391), TFT_LCD_ARRAY_CMD391}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD392), TFT_LCD_ARRAY_CMD392}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD393), TFT_LCD_ARRAY_CMD393}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD394), TFT_LCD_ARRAY_CMD394}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD395), TFT_LCD_ARRAY_CMD395}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD396), TFT_LCD_ARRAY_CMD396}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD397), TFT_LCD_ARRAY_CMD397}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD398), TFT_LCD_ARRAY_CMD398}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD399), TFT_LCD_ARRAY_CMD399}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD400), TFT_LCD_ARRAY_CMD400}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD401), TFT_LCD_ARRAY_CMD401}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD402), TFT_LCD_ARRAY_CMD402}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD403), TFT_LCD_ARRAY_CMD403}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD404), TFT_LCD_ARRAY_CMD404}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD405), TFT_LCD_ARRAY_CMD405}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD406), TFT_LCD_ARRAY_CMD406}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD407), TFT_LCD_ARRAY_CMD407}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD408), TFT_LCD_ARRAY_CMD408}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD409), TFT_LCD_ARRAY_CMD409}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD410), TFT_LCD_ARRAY_CMD410}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD411), TFT_LCD_ARRAY_CMD411}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD412), TFT_LCD_ARRAY_CMD412}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD413), TFT_LCD_ARRAY_CMD413}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD414), TFT_LCD_ARRAY_CMD414}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD415), TFT_LCD_ARRAY_CMD415}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD416), TFT_LCD_ARRAY_CMD416}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD417), TFT_LCD_ARRAY_CMD417}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD418), TFT_LCD_ARRAY_CMD418}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD419), TFT_LCD_ARRAY_CMD419}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD420), TFT_LCD_ARRAY_CMD420}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD421), TFT_LCD_ARRAY_CMD421}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD422), TFT_LCD_ARRAY_CMD422}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD423), TFT_LCD_ARRAY_CMD423}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD424), TFT_LCD_ARRAY_CMD424}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD425), TFT_LCD_ARRAY_CMD425}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD426), TFT_LCD_ARRAY_CMD426}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD427), TFT_LCD_ARRAY_CMD427}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD428), TFT_LCD_ARRAY_CMD428}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD429), TFT_LCD_ARRAY_CMD429}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD430), TFT_LCD_ARRAY_CMD430}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD431), TFT_LCD_ARRAY_CMD431}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD432), TFT_LCD_ARRAY_CMD432}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD433), TFT_LCD_ARRAY_CMD433}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD434), TFT_LCD_ARRAY_CMD434}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD435), TFT_LCD_ARRAY_CMD435}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD436), TFT_LCD_ARRAY_CMD436}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD437), TFT_LCD_ARRAY_CMD437}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD438), TFT_LCD_ARRAY_CMD438}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD439), TFT_LCD_ARRAY_CMD439}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD440), TFT_LCD_ARRAY_CMD440}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD441), TFT_LCD_ARRAY_CMD441}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD442), TFT_LCD_ARRAY_CMD442}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD443), TFT_LCD_ARRAY_CMD443}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD444), TFT_LCD_ARRAY_CMD444}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD445), TFT_LCD_ARRAY_CMD445}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD446), TFT_LCD_ARRAY_CMD446}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD447), TFT_LCD_ARRAY_CMD447}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD448), TFT_LCD_ARRAY_CMD448}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD449), TFT_LCD_ARRAY_CMD449}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD450), TFT_LCD_ARRAY_CMD450}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD451), TFT_LCD_ARRAY_CMD451}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD452), TFT_LCD_ARRAY_CMD452}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD453), TFT_LCD_ARRAY_CMD453}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD454), TFT_LCD_ARRAY_CMD454}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD455), TFT_LCD_ARRAY_CMD455}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD456), TFT_LCD_ARRAY_CMD456}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD457), TFT_LCD_ARRAY_CMD457}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD458), TFT_LCD_ARRAY_CMD458}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD459), TFT_LCD_ARRAY_CMD459}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD460), TFT_LCD_ARRAY_CMD460}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD461), TFT_LCD_ARRAY_CMD461}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD462), TFT_LCD_ARRAY_CMD462}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD463), TFT_LCD_ARRAY_CMD463}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD464), TFT_LCD_ARRAY_CMD464}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD465), TFT_LCD_ARRAY_CMD465}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD466), TFT_LCD_ARRAY_CMD466}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD467), TFT_LCD_ARRAY_CMD467}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD468), TFT_LCD_ARRAY_CMD468}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD469), TFT_LCD_ARRAY_CMD469}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD470), TFT_LCD_ARRAY_CMD470}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD471), TFT_LCD_ARRAY_CMD471}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD472), TFT_LCD_ARRAY_CMD472}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD473), TFT_LCD_ARRAY_CMD473}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD474), TFT_LCD_ARRAY_CMD474}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD475), TFT_LCD_ARRAY_CMD475}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD476), TFT_LCD_ARRAY_CMD476}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD477), TFT_LCD_ARRAY_CMD477}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD478), TFT_LCD_ARRAY_CMD478}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD479), TFT_LCD_ARRAY_CMD479}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD480), TFT_LCD_ARRAY_CMD480}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD481), TFT_LCD_ARRAY_CMD481}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD482), TFT_LCD_ARRAY_CMD482}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD483), TFT_LCD_ARRAY_CMD483}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD484), TFT_LCD_ARRAY_CMD484}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD485), TFT_LCD_ARRAY_CMD485}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD486), TFT_LCD_ARRAY_CMD486}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD487), TFT_LCD_ARRAY_CMD487}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD488), TFT_LCD_ARRAY_CMD488}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD489), TFT_LCD_ARRAY_CMD489}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD490), TFT_LCD_ARRAY_CMD490}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD491), TFT_LCD_ARRAY_CMD491}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD492), TFT_LCD_ARRAY_CMD492}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD493), TFT_LCD_ARRAY_CMD493}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD494), TFT_LCD_ARRAY_CMD494}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD495), TFT_LCD_ARRAY_CMD495}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD496), TFT_LCD_ARRAY_CMD496}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD497), TFT_LCD_ARRAY_CMD497}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD498), TFT_LCD_ARRAY_CMD498}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD499), TFT_LCD_ARRAY_CMD499}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD500), TFT_LCD_ARRAY_CMD500}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD501), TFT_LCD_ARRAY_CMD501}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD502), TFT_LCD_ARRAY_CMD502}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD503), TFT_LCD_ARRAY_CMD503}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD504), TFT_LCD_ARRAY_CMD504}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD505), TFT_LCD_ARRAY_CMD505}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD506), TFT_LCD_ARRAY_CMD506}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD507), TFT_LCD_ARRAY_CMD507}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD508), TFT_LCD_ARRAY_CMD508}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD509), TFT_LCD_ARRAY_CMD509}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD510), TFT_LCD_ARRAY_CMD510}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD511), TFT_LCD_ARRAY_CMD511}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD512), TFT_LCD_ARRAY_CMD512}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD513), TFT_LCD_ARRAY_CMD513}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD514), TFT_LCD_ARRAY_CMD514}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD515), TFT_LCD_ARRAY_CMD515}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD516), TFT_LCD_ARRAY_CMD516}
	,
	{MIPI_DSI_DCS_SHORT_WRITE, 0, 150, sizeof(TFT_LCD_ARRAY_CMD517),
	 TFT_LCD_ARRAY_CMD517}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD518), TFT_LCD_ARRAY_CMD518}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD519), TFT_LCD_ARRAY_CMD519}
	,
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD520), TFT_LCD_ARRAY_CMD520}
	,
	{MIPI_DSI_DCS_SHORT_WRITE, 0, UNLOCK_DELAY,
	 sizeof(TFT_LCD_ARRAY_CMD521), TFT_LCD_ARRAY_CMD521}
};

static void tft_1e_panel_on(struct mmp_path *path)
{
	mmp_phy_dsi_tx_cmd_array(path->phy, tft_1e_display_on_cmds,
				 ARRAY_SIZE(tft_1e_display_on_cmds));
}

#ifdef CONFIG_OF
static void tft_1e_panel_power(struct mmp_panel *panel, int skip_on, int on)
{
	static struct regulator *lcd_iovdd;
	int lcd_rst_n, boost_en_5v, ret = 0;

	lcd_rst_n = of_get_named_gpio(panel->dev->of_node, "rst_gpio", 0);
	boost_en_5v = of_get_named_gpio(panel->dev->of_node, "power_gpio", 0);
	if (lcd_rst_n < 0 || boost_en_5v < 0) {
		pr_err("%s: of_get_named_gpio failed\n", __func__);
		return;
	}

	if (gpio_request(lcd_rst_n, "lcd reset gpio")) {
		pr_err("gpio %d request failed\n", lcd_rst_n);
		return;
	}

	if (gpio_request(boost_en_5v, "5v boost en")) {
		pr_err("gpio %d request failed\n", boost_en_5v);
		gpio_free(lcd_rst_n);
		return;
	}

	if (!lcd_iovdd) {
		lcd_iovdd = regulator_get(panel->dev, "iovdd");
		if (IS_ERR(lcd_iovdd)) {
			pr_err("%s regulator get error!\n", __func__);
			ret = -EIO;
			goto regu_lcd_iovdd;
		}
	}

	if (on) {
		ret = regulator_enable(lcd_iovdd);
		if (ret < 0)
			goto regu_lcd_iovdd;
		usleep_range(1000, 1200);

		/* LCD_AVDD+ and LCD_AVDD- ON */
		gpio_direction_output(boost_en_5v, 1);
		usleep_range(50000, 60000);
		if (!skip_on) {
			gpio_direction_output(lcd_rst_n, 1);
			usleep_range(50000, 60000);
			gpio_direction_output(lcd_rst_n, 0);

			usleep_range(50000, 60000);
			/* set panel reset */
			gpio_direction_output(lcd_rst_n, 1);
			usleep_range(50000, 60000);
		}
	} else {
		gpio_direction_output(boost_en_5v, 0);
		usleep_range(1000, 1200);
		gpio_direction_output(lcd_rst_n, 0);
		usleep_range(10000, 12000);
		/* disable LCD_IOVDD 1.8v */
		regulator_disable(lcd_iovdd);
		usleep_range(1000, 1200);
	}

regu_lcd_iovdd:
	gpio_free(lcd_rst_n);
	gpio_free(boost_en_5v);
	if (ret < 0)
		lcd_iovdd = NULL;
}
#else
static void tft_1e_panel_power(struct mmp_panel *panel, int on) {}
#endif

static void tft_1e_onoff(struct mmp_panel *panel, int status)
{
	struct tft_1e_plat_data *plat = panel->plat_data;
	struct mmp_path *path = mmp_get_path(panel->plat_path_name);

	if (status) {
#ifdef CONFIG_DDR_DEVFREQ
		if (panel->ddrfreq_qos != PM_QOS_DEFAULT_VALUE)
			pm_qos_update_request(&panel->ddrfreq_qos_req_min,
				panel->ddrfreq_qos);
#endif
		/* power on */
		if (plat->plat_onoff)
			plat->plat_onoff(1);
		else
			tft_1e_panel_power(panel, 0, 1);
		tft_1e_panel_on(path);
	} else {
		/* power off */
		if (plat->plat_onoff)
			plat->plat_onoff(0);
		else
			tft_1e_panel_power(panel, 0, 0);
#ifdef CONFIG_DDR_DEVFREQ
		if (panel->ddrfreq_qos != PM_QOS_DEFAULT_VALUE)
			pm_qos_update_request(&panel->ddrfreq_qos_req_min,
				PM_QOS_DEFAULT_VALUE);
#endif
	}
}

static void tft_1e_reduced_onoff(struct mmp_panel *panel, int status)
{
	if (status) {
		/* power on */
		tft_1e_panel_power(panel, 1, 1);
	} else {
		/* power off */
		 tft_1e_panel_power(panel, 1, 0);
	}
}

static struct mmp_mode mmp_modes_tft_1e[] = {
	[0] = {
	       .pixclock_freq = 148455600,
	       .refresh = 60,
#if defined(CONFIG_MMP_VIRTUAL_RESOLUTION)
	       .xres = CONFIG_MMP_VIRTUAL_RESOLUTION_X,
	       .yres = CONFIG_MMP_VIRTUAL_RESOLUTION_Y,
#else
               .xres = 1080,
	       .yres = 1920,
#endif
	       .real_xres = 1080,
	       .real_yres = 1920,
	       .hsync_len = 2,
	       .left_margin = 40,
	       .right_margin = 160,
	       .vsync_len = 2,
	       .upper_margin = 8,
	       .lower_margin = 8,
	       .invert_pixclock = 0,
	       .pix_fmt_out = PIXFMT_RGB888PACK,
	       .hsync_invert = FB_SYNC_HOR_HIGH_ACT,
	       .vsync_invert = FB_SYNC_VERT_HIGH_ACT,
	       },
};

static int tft_1e_get_modelist(struct mmp_panel *panel,
			       struct mmp_mode **modelist)
{
	*modelist = mmp_modes_tft_1e;
	return 1;
}

static struct mmp_panel panel_tft_1e = {
	.name = "tft-10801920-1e",
	.panel_type = PANELTYPE_DSI_VIDEO,
	.get_modelist = tft_1e_get_modelist,
	.set_onoff = tft_1e_onoff,
	.reduced_onoff = tft_1e_reduced_onoff,
};

static int tft_1e_probe(struct platform_device *pdev)
{
	struct mmp_mach_panel_info *mi;
	struct tft_1e_plat_data *plat_data;
	struct device_node *np = pdev->dev.of_node;
	const char *path_name;
#ifdef CONFIG_DDR_DEVFREQ
	char qos_name[64];
#endif
	int ret;

	plat_data = kzalloc(sizeof(*plat_data), GFP_KERNEL);
	if (!plat_data)
		return -ENOMEM;

	if (IS_ENABLED(CONFIG_OF)) {
		ret = of_property_read_string(np, "marvell,path-name",
				&path_name);
		if (ret < 0)
			return ret;
		panel_tft_1e.plat_path_name = path_name;

#ifdef CONFIG_DDR_DEVFREQ
		ret = of_property_read_u32(np, "marvell,ddrfreq-qos",
				&panel_tft_1e.ddrfreq_qos);
		if (ret < 0) {
			panel_tft_1e.ddrfreq_qos = PM_QOS_DEFAULT_VALUE;
			pr_debug("panel %s didn't has ddrfreq min request\n",
				panel_tft_1e.name);
		} else {
			sprintf(qos_name, "panel-%s", panel_tft_1e.name);
			panel_tft_1e.ddrfreq_qos_req_min.name = qos_name;
			pm_qos_add_request(&panel_tft_1e.ddrfreq_qos_req_min,
					PM_QOS_DDR_DEVFREQ_MIN,
					PM_QOS_DEFAULT_VALUE);
			pr_debug("panel %s has ddrfreq min request: %u\n",
				 panel_r63311.name, panel_tft_1e.ddrfreq_qos);
		}
#endif
	} else {
		/* get configs from platform data */
		mi = pdev->dev.platform_data;
		if (mi == NULL) {
			dev_err(&pdev->dev, "no platform data defined\n");
			return -EINVAL;
		}
		plat_data->plat_onoff = mi->plat_set_onoff;
		panel_tft_1e.plat_path_name = mi->plat_path_name;
	}

	panel_tft_1e.plat_data = plat_data;
	panel_tft_1e.dev = &pdev->dev;
	plat_data->panel = &panel_tft_1e;
	mmp_register_panel(&panel_tft_1e);

	return 0;
}

static int tft_1e_remove(struct platform_device *dev)
{
#ifdef CONFIG_DDR_DEVFREQ
	if (panel_tft_1e.ddrfreq_qos != PM_QOS_DEFAULT_VALUE)
		pm_qos_remove_request(&panel_tft_1e.ddrfreq_qos_req_min);
#endif
	mmp_unregister_panel(&panel_tft_1e);
	kfree(panel_tft_1e.plat_data);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mmp_tft_1e_dt_match[] = {
	{ .compatible = "marvell,mmp-tft-10801920-1e" },
	{},
};
#endif

static struct platform_driver tft_1e_driver = {
	.driver = {
		   .name = "mmp-tft-10801920-1e",
		   .owner = THIS_MODULE,
		.of_match_table = of_match_ptr(mmp_tft_1e_dt_match),
		   },
	.probe = tft_1e_probe,
	.remove = tft_1e_remove,
};

static int tft_1e_init(void)
{
	return platform_driver_register(&tft_1e_driver);
}

static void tft_1e_exit(void)
{
	platform_driver_unregister(&tft_1e_driver);
}

module_init(tft_1e_init);
module_exit(tft_1e_exit);

MODULE_AUTHOR("Yossi Eliaz <yossie@marvell.com>");
MODULE_DESCRIPTION("Panel driver for MIPI panel TFT 1080-1920 1-E");
MODULE_LICENSE("GPL");
