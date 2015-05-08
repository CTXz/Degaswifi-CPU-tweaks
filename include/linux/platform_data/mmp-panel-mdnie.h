#ifndef	__MMP_PANEL_MDNIE_H__
#define __MMP_PANEL_MDNIE_H__
#include <linux/mutex.h>
#include <video/mmp_disp.h>
#define NUMBER_OF_SCR_DATA	9
struct mdnie_lite;

enum MDNIE_SCENARIO {
	MDNIE_UI_MODE		= 0,
	MDNIE_VIDEO_MODE	= 1,
	MDNIE_VIDEO_WARM_MODE	= 2,
	MDNIE_VIDEO_COLD_MODE	= 3,
	MDNIE_CAMERA_MODE	= 4,
	MDNIE_NAVI_MODE		= 5,
	MDNIE_GALLERY_MODE	= 6,
	MDNIE_VT_MODE		= 7,
	MDNIE_BROWSER_MODE	= 8,
	MDNIE_EBOOK_MODE	= 9,
	MDNIE_EMAIL_MODE	= 10,
	MDNIE_SCENARIO_MAX,
};

enum ACCESSIBILITY {
	ACCESSIBILITY_OFF,
	NEGATIVE,
	COLOR_BLIND,
	ACCESSIBILITY_MAX
};

struct mdnie_config {
	bool tuning;
	bool negative;
	int accessibility;
	int scenario;
};

struct mdnie_ops {
	int (*set_mdnie)(struct mdnie_config *);
	int (*get_mdnie)(struct mdnie_config *);
};

struct mdnie_lite {
	struct device *dev;
	struct class *class;
	bool support;
	unsigned char cmd_reg;
	unsigned int cmd_len;
	struct mdnie_config config;
	struct mutex ops_lock;
	const struct mdnie_ops *ops;
	struct mmp_dsi_cmds tuning_mode_cmds;
	struct mmp_dsi_cmds ui_mode_cmds;
	struct mmp_dsi_cmds video_mode_cmds;
	struct mmp_dsi_cmds camera_mode_cmds;
	struct mmp_dsi_cmds gallery_mode_cmds;
	struct mmp_dsi_cmds vt_mode_cmds;
	struct mmp_dsi_cmds browser_mode_cmds;
	struct mmp_dsi_cmds ebook_mode_cmds;
	struct mmp_dsi_cmds email_mode_cmds;
	struct mmp_dsi_cmds negative_mode_cmds;
	struct mmp_dsi_cmds color_adjustment_cmds;
	unsigned int scr[NUMBER_OF_SCR_DATA];
};

extern int mmp_panel_attach_mdnie(struct mdnie_lite *,
		const struct mdnie_ops *);
extern void mmp_panel_detach_mdnie(struct mdnie_lite *mdnie);
extern void update_mdnie_mode(struct mdnie_lite *mdnie);
#endif /* __MMP_PANEL_MDNIE_H__ */
