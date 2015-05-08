#ifndef __MACH_MMP_CLK_H
#define __MACH_MMP_CLK_H
#include <linux/clk-private.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/clk/mmpdcstat.h>
#include <linux/pm_qos.h>

#define APBC_NO_BUS_CTRL	BIT(0)
#define APBC_POWER_CTRL		BIT(1)
#define APBC_FN_CTRL		BIT(2)
#define APBC_APB_CTRL		BIT(3)
#define APBC_RST_CTRL		BIT(4)
/* for APB pwm clock */
#define APBC_PWM		BIT(5)

#define MAX_OP_NUM	(10)

struct clk_factor_masks {
	unsigned int	factor;
	unsigned int	num_mask;
	unsigned int	den_mask;
	unsigned int	num_shift;
	unsigned int	den_shift;
};

struct clk_factor_tbl {
	unsigned int num;
	unsigned int den;
};

extern struct clk *mmp_clk_register_apbc(const char *name,
		const char *parent_name, void __iomem *base,
		unsigned int delay, unsigned int apbc_flags, spinlock_t *lock);
extern struct clk *mmp_clk_register_apmu(const char *name,
		const char *parent_name, void __iomem *base, u32 enable_mask,
		spinlock_t *lock);
extern struct clk *mmp_clk_register_factor(const char *name,
		const char *parent_name, unsigned long flags,
		void __iomem *base, struct clk_factor_masks *masks,
		struct clk_factor_tbl *ftbl, unsigned int ftbl_cnt);
extern struct clk *mmp_clk_register_audio(const char *name,
		const char *parent_name, void __iomem *apmu_base,
		void __iomem *aud_base, void __iomem *aud_aux_base,
		u32 enable_mask, spinlock_t *lock);

typedef void (*poweron_cb)(void __iomem*, int);
void audio_clk_init(void __iomem *apmu_base, void __iomem *audio_base,
			void __iomem *audio_aux_base, poweron_cb poweron);

#define MHZ_TO_HZ (1000000)
#define MHZ_TO_KHZ (1000)
#define MHZ (1000000)
/*
 * MMP PLL_VCO:
 *
 * VCO = REF * FBD / REFD
 *
 */

/*
 * struct pll_offset - offset and width of filed for pll
 * @fbd_shift:		shift to the feedback divider bit field
 * @fbd_width:		width of the feedback divider bit field
 * @refd_shift:		shift to the reference divider bit field
 * @refd_width:		width of the reference divider bit field
 * @sel_div_shift:	shift to the single ended divider bit field
 * @sel_div_width:	width of the single ended divider bit field
 * @diff_div_shift:	shift to the differential divider bit field
 * @diff_div_width:	width of the differential divider bit field
 * @kvco_shift:		shift to the kvco bit field
 * @kvco_width:		width of the kvco bit field
 * @vrng_shift:		shift to the vrng bit field
 * @vrng_width:		width of the vrng bit field
 */
struct pll_offset {
	u32		fbd_shift;
	u32		fbd_width;
	u32		refd_shift;
	u32		refd_width;
	u32		kvco_shift;
	u32		kvco_width;
	u32		vrng_shift;
	u32		vrng_width;
};

#define PLL_MASK(n)	((1 << (n)) - 1)
#define MASK(n)	((1 << (n)) - 1)

struct pi_cnf_reg {
	void __iomem	*regoff;
	u32		intpishift;
	u32		intpiwidth;
	u32		intprshift;
	u32		intprwidth;
	u32		diffclkselbit;
	u32		seclkselbit;
};

struct pi_ctrl_reg {
	void __iomem	*regoff;
	u32		enbit;
	u32		clkdetenbit;
	u32		sscclkenbit;
	u32		rstbit;
};

struct pi_loopmode_reg {
	void __iomem	*regoff;
	u32		modeselbit;
};

struct ssc_conf_reg {
	void __iomem	*regoff;
	u32		divshift;
	u32		divwidth;
	u32		rngshift;
	u32		rngwidth;
};

struct ssc_modsel_reg {
	void __iomem	*regoff;
	u32		modselbit;
};

struct ssc_ctrl_reg {
	void __iomem	*regoff;
	u32		rstbit;
	u32		enbit;
};

struct pi_ssc_reg_des {
	struct pi_cnf_reg *pi_cnf_reg;
	struct pi_ctrl_reg *pi_ctrl_reg;
	struct pi_loopmode_reg *pi_loopmode_reg;
	struct ssc_conf_reg *ssc_conf_reg;
	struct ssc_modsel_reg *ssc_modsel_reg;
	struct ssc_ctrl_reg *ssc_ctrl_reg;
};

struct intpi_range {
	int		vco_min;
	int		vco_max;
	u8		value;
};

/*
 * struct kvco_range -store kvco and vrng for different frequency range
 * @vco_min:	min frequency of vco
 * @vco_max:	max frequency of vco
 * @kvco:	kvco val for relevant frequency range
 * @vrng:	vrng val for relevant frequency range
 */
struct kvco_range {
	int		vco_min;
	int		vco_max;
	u8		kvco;
	u8		vrng;
};

struct div_map {
	unsigned int div;
	unsigned int hw_val;
};

enum ssc_mode {
	CENTER_SPREAD = 0x0,
	DOWN_SPREAD = 0x1,
};

enum pllip {
	PLL_28nm = 0x0,
	PLL_40nm = 0x1,
};

struct ssc_params {
	enum ssc_mode ssc_mode;
	int base;
	int amplitude;
	int desired_mod_freq;
	u32 intpr;
	struct pi_ssc_reg_des *des;
};

struct mmp_vco_params {
	unsigned long		vco_min;
	unsigned long		vco_max;
	void __iomem		*cr_reg;
	void __iomem		*pll_swcr;
	void __iomem		*lock_reg;
	u32			enable_bit;
	u32			enable_mask;
	u32			ctrl_bit;
	u32			ctrl_mask;
	u32			lock_enable_bit;
	struct kvco_range 	*kvco_rng_table;
	int			kvco_rng_size;
	struct pll_offset	*pll_offset;
	unsigned long		default_vco_rate;
	struct ssc_params	*ssc_params;
	bool			ssc_enabled;
};

struct mmp_pll_params {
	void __iomem		*pll_swcr;
	u32			div_shift;
	u32			div_width;
	struct div_map 		*div_map;
	int			div_map_size;
	unsigned long		default_pll_rate;
};

struct mmp_vco_freq_table {
	unsigned long	output_rate;
	u16		refd;
	u16		fbd;
	u8		kvco;
	u8		vcovnrg;
};

struct clk_vco {
	struct clk_hw			hw;
	spinlock_t			*lock;
	u32				flags;
	struct mmp_vco_freq_table	*freq_table;
	int				freq_table_size;
	struct mmp_vco_params		*params;
};

struct clk_pll {
	struct clk_hw		hw;
	const char		*parent;
	spinlock_t		*lock;
	int			div;
	u32			flags;
	struct mmp_pll_params	*params;
};

#define to_clk_vco(vco_hw) container_of(vco_hw, struct clk_vco, hw)
#define to_clk_pll(pll_hw) container_of(pll_hw, struct clk_pll, hw)

/**
 * Flags:
 */
#define MMP_PLL_LOCK_SETTING	BIT(0)
#define MMP_PLL_28NM		BIT(1)
#define MMP_PLL_SSC_FEAT	BIT(2)
#define MMP_PLL_SSC_AON		BIT(3)

extern struct clk *mmp_clk_register_vco(const char *name, const char *parent_name,
		unsigned long flags, u32 vco_flags, spinlock_t *lock,
		struct mmp_vco_params *params,
		struct mmp_vco_freq_table *freq_table, int table_size);

extern struct clk *mmp_clk_register_pll(const char *name, const char *parent_name,
		unsigned long flags, u32 vco_flags, spinlock_t *lock,
		struct mmp_pll_params *params);

#define EDEN_PLL_DIV_3	8
#define to_eden_clk_vco(_hw) container_of(_hw, struct eden_clk_pll_vco, hw)
#define to_eden_clk_pll(_hw) container_of(_hw, struct eden_clk_pll_out, hw)

struct eden_clk_pll_vco_table {
	unsigned long input_rate;
	unsigned long output_rate;
	unsigned long output_rate_offseted;
	u16 refdiv;
	u16 fbdiv;
	u16 icp;
	u16 kvco;
	u16 ssc_en;
	u16 offset_en;
};

struct eden_clk_pll_vco_params {
	unsigned long	vco_min;
	unsigned long	vco_max;
	/* refdiv, fbdiv, ctrl_bit, sw_en_bit */
	u32	reg_pll_cr;
	/* icp, kvco, rst */
	u32	reg_pll_ctrl1;
	u32	reg_pll_ctrl2;
	u32	reg_pll_ctrl3;
	u32	reg_pll_ctrl4;
	/* lock status */
	u32	reg_pll_lock;
	u32	lock_bit;
	/* gate control */
	u32	reg_pll_gate;
	u16	gate_width;
	u16	gate_shift;
};

struct eden_clk_pll_vco {
	struct clk_hw	hw;
	spinlock_t	*lock;
	void __iomem	*mpmu_base;
	void __iomem	*apmu_base;
	struct eden_clk_pll_vco_params	*params;
	struct eden_clk_pll_vco_table	*freq_table;
	struct eden_clk_pll_vco_table	freq_rounded;
};

struct eden_clk_pll_out_table {
	unsigned long	input_rate;
	unsigned long	output_rate;
	unsigned int	div_sel;
};

struct eden_clk_pll_out_params {
	unsigned long	input_rate_min;
	/* post divider selection */
	u32	reg_pll_out_div;
	u16	div_width;
	u16	div_shift;
	/* misc ctrl */
	u32	reg_pll_out_ctrl;
};

/* pll_out flags */
#define EDEN_PLL_USE_DIV_3	BIT(0)
#define EDEN_PLL_USE_SYNC_DDR	BIT(1)
#define EDEN_PLL_USE_ENABLE_BIT	BIT(2)
struct eden_clk_pll_out {
	struct clk_hw	hw;
	unsigned long	flags;
	spinlock_t	*lock;
	void __iomem	*mpmu_base;
	void __iomem	*apmu_base;
	struct eden_clk_pll_out_params	*params;
	struct eden_clk_pll_out_table	*freq_table;
};

extern struct clk *eden_clk_register_pll_vco(const char *name,
		const char *parent_name,
		unsigned long flags, spinlock_t *lock,
		void __iomem *mpmu_base, void __iomem *apmu_base,
		struct eden_clk_pll_vco_params *params,
		struct eden_clk_pll_vco_table *freq_table);
extern struct clk *eden_clk_register_pll_out(const char *name,
		const char *parent_name,
		unsigned long pll_flags, spinlock_t *lock,
		void __iomem *mpmu_base, void __iomem *apmu_base,
		struct eden_clk_pll_out_params *params,
		struct eden_clk_pll_out_table *freq_table);

/*
 * MMP CORE:
 */
struct core_reg_offset {
	/* core clk src sel set register */
	u32		fcap_off;
	u32		clk_sel_shift;
	u32		clk_sel_width;
	void __iomem	*pll1_pll3_swreg;
	u32		pll1_pll3_swbit;
};

/* RTC/WTC table used for solution change rtc/wtc on the fly */
struct cpu_rtcwtc {
	unsigned int max_pclk;	/* max rate could be used by this rtc/wtc */
	unsigned int l1_rtc;
	unsigned int l2_rtc;
};

/*
 * AP clock source:
 * 0x0 = PLL1 624 MHz
 * 0x1 = PLL1 1248 MHz  or PLL3_CLKOUT
 * (depending on PLL3_CR[18])
 * 0x2 = PLL2_CLKOUT
 * 0x3 = PLL2_CLKOUTP
 */
enum ap_clk_sel {
	AP_CLK_SRC_PLL1_624 = 0x0,
	AP_CLK_SRC_PLL1_1248 = 0x1,
	AP_CLK_SRC_PLL2 = 0x2,
	AP_CLK_SRC_PLL2P = 0x3,
	AP_CLK_SRC_PLL3P = 0x5,
};

struct cpu_opt {
	unsigned int pclk;		/* core clock */
	unsigned int l2clk;		/* L2 cache interface clock */
	unsigned int pdclk;		/* DDR interface clock */
	unsigned int baclk;		/* bus interface clock */
	unsigned int periphclk;		/* PERIPHCLK */
	enum ap_clk_sel ap_clk_sel;	/* core src sel val */
	struct clk *parent;		/* core clk parent node */
	unsigned int ap_clk_src;	/* core src rate */
	unsigned int pclk_div;		/* core clk divider*/
	unsigned int l2clk_div;		/* L2 clock divider */
	unsigned int pdclk_div;		/* DDR interface clock divider */
	unsigned int baclk_div;		/* bus interface clock divider */
	unsigned int periphclk_div;	/* PERIPHCLK divider */
	unsigned int l1_rtc;		/* L1 cache RTC/WTC */
	unsigned int l2_rtc;		/* L2 cache RTC/WTC */
	struct list_head node;
};

struct core_parents_table {
	char*			parent_name;
	enum ap_clk_sel		hw_sel_val;
};

/*
 * struct core_params store core specific data
 */
struct core_params {
	void __iomem			*apmu_base;
	void __iomem			*mpmu_base;
	void __iomem			*ciu_base;
	struct core_reg_offset		*core_offset;
	struct core_parents_table	*parent_table;
	int				parent_table_size;
	struct cpu_opt			*cpu_opt;
	int				cpu_opt_size;
	struct cpu_rtcwtc		*cpu_rtcwtc_table;
	int				cpu_rtcwtc_table_size;
	unsigned int			max_cpurate;
	unsigned int			*pp_disable;
	unsigned int			pp_discnt;
	/* dynamic dc stat support? */
	bool				dcstat_support;
	powermode			pxa_powermode;
	spinlock_t			*shared_lock;
};

struct clk_core {
	struct clk_hw		hw;
	struct core_params	*params;
	u32			flags;
	spinlock_t		*lock;
};

/* FLAGS */
#define MMP_CORE_PLL3_SEL	BIT(4)
extern struct clk *mmp_clk_register_core(const char *name,
		const char **parent_name,
		u8 num_parents, unsigned long flags, u32 core_flags,
		spinlock_t *lock, struct core_params *params);

/*
 * MMP DDR:
 */
struct parents_table {
	char			*parent_name;
	struct clk		*parent;
	u32			hw_sel_val;
};

struct ddr_opt {
	unsigned int dclk;		/* ddr clock */
	unsigned int ddr_tbl_index;	/* ddr FC table index */
	unsigned int ddr_freq_level;	/* ddr freq level(0~7) */
	unsigned int ddr_volt_level;	/* ddr voltage level (0~7) */
	u32 ddr_clk_sel;		/* ddr src sel val */
	unsigned int ddr_clk_src;	/* ddr src rate */
	struct clk *ddr_parent;		/* ddr clk parent node */
	unsigned int dclk_div;		/* ddr clk divider */
};

struct ddr_reg_offset {
	/* ddr clk src sel set register */
	u32		fcdclk_off;
	u32		tbl_ctrl_off;
	u32		clk_sel_shift;
	u32		clk_sel_width;
	u32		tbl_enable_shift;
	u32		tbl_index_shift;
	u32		tbl_index_width;
};

struct ddr_params {
	void __iomem			*apmu_base;
	void __iomem			*mpmu_base;
	void __iomem			*dmcu_base;
	struct ddr_reg_offset		*ddr_offset;
	struct parents_table		*parent_table;
	int				parent_table_size;
	struct ddr_opt			*ddr_opt;
	int				ddr_opt_size;
	unsigned long			*hwdfc_freq_table;
	int				hwdfc_table_size;
	/* dynamic dc stat support? */
	bool				dcstat_support;
};

struct clk_ddr {
	struct clk_hw		hw;
	struct ddr_params	*params;
	u32			flags;
	spinlock_t		*lock;
};

/* FLAGS */
/* indicate if hwdfc or sw lagency dfc is used */
#define MMP_DDR_HWDFC_FEAT		BIT(0)
#define MMP_DDR_FC_HARDWARE_ENABLE	BIT(1)
#define MMP_DDR_HAS_HW_VBLANK_DFC	BIT(2)
/* used for hwdfc WR, add more flag in case need to separate
   diff issue on diff plat */
#define MMP_DDR_HWDFC_WR		BIT(3)
#define MMP_DDR_PLLSEL_3BIT		BIT(4)
#define MMP_DDR_DLL_BYPASS		BIT(5)
struct clk *mmp_clk_register_ddr(const char *name, const char **parent_name,
		u8 num_parents, unsigned long flags, u32 ddr_flags,
		spinlock_t *lock, struct ddr_params *params);

/*
 * For combined ddr solution, other components can
 * register to let ddr change its rate
 */
struct ddr_combclk_relation {
	unsigned long dclk_rate;
	unsigned long combclk_rate;
};

struct ddr_combined_clk {
	struct clk *clk;
	unsigned long maxrate;
	struct list_head node;
	/* Describe the relationship with Dclk */
	struct ddr_combclk_relation *relationtbl;
	unsigned int num_relationtbl;
};

int register_clk_bind2ddr(struct clk *clk, unsigned long max_freq,
			  struct ddr_combclk_relation *relationtbl,
			  unsigned int num_relationtbl);
/*
 * MMP AXI:
 */
struct axi_opt {
	unsigned int aclk;		/* axi clock */
	u32 axi_clk_sel;		/* axi src sel val */
	unsigned int axi_clk_src;	/* axi src rate */
	struct clk *axi_parent;		/* axi clk parent node */
	unsigned int aclk_div;		/* axi clk divider */
};

struct axi_reg_offset {
	/* axi clk src sel set register */
	u32		fcaclk_off;
	u32		clk_sel0_shift;
	u32		clk_sel0_width;
	u32		clk_sel1_shift;
	u32		clk_sel1_width;
};

struct axi_params {
	void __iomem			*apmu_base;
	void __iomem			*mpmu_base;
	struct axi_reg_offset		*axi_offset;
	struct parents_table		*parent_table;
	int				parent_table_size;
	struct axi_opt			*axi_opt;
	int				axi_opt_size;
	/* dynamic dc stat support? */
	bool				dcstat_support;
};

struct clk_axi {
	struct clk_hw		hw;
	struct axi_params	*params;
	u32			flags;
	spinlock_t		*lock;
};

/* FLAGS */
#define MMP_AXI_SEPERATED_SRC_SEL	BIT(0)

struct clk *mmp_clk_register_axi(const char *name, const char **parent_name,
		u8 num_parents, unsigned long flags, u32 axi_flags,
		spinlock_t *lock, struct axi_params *params);

/*
 * MMP PERIPHERAL:
 */
/*
 * README:
 * 1. For clk which has fc_request bit, two step operation
 * is safer to enable clock with taget frequency
 * 1) set enable&rst bit
 * 2) set mux, div and fc to do FC, get target rate.
 */

/*
 * periph_clk is used to describe the clock which has ability
 * to scaling its rate on the fly.
 * They usually have src_sel/div/fcreq, some of them have
 * rtcwtc setting, like GC/VPU.
 * Normally it has freq-tbl with efficient setrate purpose
 */
struct clk_mux_sel {
	char *parent_name;
	u32 value;
	struct clk *parent;
};

struct periph_clk_tbl {
	unsigned long	clk_rate;	/* clk rate */
	char		*parent_name;	/* clk parent name*/
	struct clk	*parent;	/* clk parent */
	unsigned long	src_val;	/* clk src field reg val */
	unsigned long	div_val;	/* clk div field reg val */

	/* combined clck rate, such as bus clk that will changed with fclk */
	unsigned long comclk_rate;
	unsigned int rtcwtc;
	struct list_head node;
};

struct xpu_rtcwtc {
	unsigned long max_rate;
	unsigned int rtcwtc;
};

struct peri_reg_info {
	/* peripheral device source select */
	u32		src_sel_shift;
	u32		src_sel_width;
	u32		src_sel_mask;
	/* peripheral device divider set */
	u32		div_shift;
	u32		div_width;
	u32		div_mask;
	/* peripheral device freq change trigger */
	u32		fcreq_shift;
	u32		fcreq_width;
	u32		fcreq_mask;

	/* to indicate divisor is based on zero or one */
	u32		flags;
	u32		reg_offset;
	/* reg address for wtc/rtc setting */
	void __iomem		*reg_rtcwtc;

	/* value for clock enable (APMU) */
	u32			enable_val;
	u32			disable_val;
};

struct peri_params {
	/* periph clock tbl, list all possible freq here */
	struct periph_clk_tbl	*clktbl;
	unsigned int		clktblsize;

	struct clk_mux_sel	*inputs;
	unsigned int		inputs_size;

	/* periph clock has wtc/rtc setting */
	struct xpu_rtcwtc	*rwtctbl;
	unsigned int		rwtctblsize;

	/* combined clk, such as aclk that will changed with fclk */
	char			*comclk_name;
	struct clk		*comclk;
	/* dynamic dc stat support? */
	bool dcstat_support;

	u32			flags;
	int			qos_idle_value;
};

struct clk_peri {
	struct clk_hw		hw;
	struct peri_reg_info	*reg_info;
	struct peri_params	*params;
	void __iomem		*reg_addr;
	void __iomem		*fc_reg;
	/* list used represent all supported freq */
	struct list_head	clktbl_list;
	/* lock to protect same register access */
	spinlock_t		*lock;
	const char		**dependence;
	int			num_dependence;
	struct pm_qos_request	qos_idle;
	/* clk private data */
	void *clk_data;
};

#define to_clk_peri(peri_hw) container_of(peri_hw, struct clk_peri, hw)
struct clk *mmp_clk_register_peri(const char *name, const char **parent_name,
		u8 num_parents, unsigned long flags, void __iomem *reg_addr,
		spinlock_t *lock, struct peri_params *params,
		struct peri_reg_info *reg_info);

struct clk *mmp_clk_register_peri_gate(const char *name,
		const char *parent_name, unsigned long flags,
		void __iomem *reg_addr, spinlock_t *lock,
		struct peri_params *params,
		struct peri_reg_info *reg_info);
/* FLAGS */
#define MMP_PERI_QOS_FEAT	BIT(0)

struct plat_clk_list {
	const char *dev_id;
	const char *con_id;
	unsigned long initrate;
};

struct clk *eden_clk_register_gc_vpu(const char *name, const char **parent_name,
		u8 num_parents, unsigned long flags, void __iomem *reg_addr,
		void __iomem *fc_reg, spinlock_t *lock,
		struct peri_params *params, struct peri_reg_info *reg_info,
		const char **clk_depend, u8 num_depend);


#define to_mmp_clk_disp(_hw) container_of(_hw, struct mmp_clk_disp, hw)
#define MMP_DISP_DIV_ONLY	BIT(0)
#define MMP_DISP_MUX_ONLY	BIT(1)
#define MMP_DISP_BUS 		BIT(2)
#define MMP_DISP_RST_CTRL	BIT(3)
struct mmp_clk_disp {
	struct clk_hw		hw;
	struct clk_mux		mux;
	struct clk_divider	divider;

	spinlock_t	*lock;
	void __iomem	*apmu_base;
	u32		reg_mux;
	u32		reg_div;
	u32		reg_div_shadow;
	u32		reg_mux_shadow;

	u32		reg_rst;
	u32		reg_rst_shadow;
	u32		reg_rst_mask;

	const char	**dependence;
	int		num_dependence;
	const struct clk_ops	*mux_ops;
	const struct clk_ops	*div_ops;
};

extern struct clk *mmp_clk_register_disp(const char *name,
		const char **parent_name,
		int num_parents, u32 disp_flags, unsigned long flags,
		void __iomem *apmu_base, struct mmp_clk_disp *disp);
#endif
