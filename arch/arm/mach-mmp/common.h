#define ARRAY_AND_SIZE(x)	(x), ARRAY_SIZE(x)

extern struct smp_operations mmp_smp_ops;

extern void timer_init(int irq);

extern void __init mmp_map_io(void);
extern void mmp_restart(char, const char *);
extern void mmp_arch_restart(char mode, const char *cmd);
extern void __init pxa168_clk_init(void);
extern void __init pxa910_clk_init(void);
extern void __init pxa988_clk_init(void);
extern void __init pxa1986_clk_init(void);
extern void __init mmp2_clk_init(void);
extern void __init eden_clk_init(unsigned long apb_phy_base,
		unsigned long axi_phy_base, unsigned long aud_phy_base,
		unsigned long aud_phy_base2);
extern void __init eden_core_clk_init(unsigned long axi_phy_base);

extern void __init mmp_of_wakeup_init(void);
extern void __init mmp_smp_init_ops(void);
extern void gc_reset(void);
