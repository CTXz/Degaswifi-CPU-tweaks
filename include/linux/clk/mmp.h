#ifndef __MMP_H
#define __MMP_H
#include <linux/clk.h>
#include <linux/clk-private.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>

enum {
	CORE_1p2G = 1183,
	CORE_1p25G = 1248,
	CORE_1p5G = 1482,
};

extern unsigned long max_freq;

unsigned int get_ddr_op_num(void);
unsigned int get_ddr_op_rate(unsigned int index);
unsigned int __clk_periph_get_opnum(struct clk *clk);
unsigned long __clk_periph_get_oprate(struct clk *clk, unsigned int index);

unsigned int eden_get_vpu_op_num(unsigned int vpu_type);
unsigned int eden_get_vpu_op_rate(unsigned int vpu_type, unsigned int index);

#endif /* __MMP_H */
