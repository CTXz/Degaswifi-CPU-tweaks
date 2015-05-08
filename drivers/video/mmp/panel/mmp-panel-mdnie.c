#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/err.h>
#include <linux/lcd.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <video/mmp_disp.h>
#include <video/mipi_display.h>
#include <linux/platform_data/mmp-panel-generic.h>
#include <linux/platform_data/mmp-panel-mdnie.h>

#define ENABLE_MDNIE_TUNING
#ifdef ENABLE_MDNIE_TUNING
#define MAX_TUNING_FILE_NAME (100)
const char *tuning_file_paths[] = {
	"/sdcard/",
	"/sdcard/mdnie/",
	"/data/"
};
static char tuning_filename[MAX_TUNING_FILE_NAME];
static unsigned char mdnie_tuning_data[256];
static struct mmp_dsi_cmd_desc mdnie_tuning_desc = {
	MIPI_DSI_DCS_LONG_WRITE, 0, 0, 0, mdnie_tuning_data
};
static struct mmp_dsi_cmds tuning_mode_cmds = {
	.cmds = &mdnie_tuning_desc,
	.nr_cmds = 1,
};

static int parse_text(char *dst, char *src, int len)
{
	int i;
	int data = 0, value = 0, count = 1, comment = 0;
	char *cur_position;

	cur_position = src;
	for (i = 0; i < len; i++, cur_position++) {
		char a = *cur_position;
		switch (a) {
		case '\r':
		case '\n':
			comment = 0;
			data = 0;
			break;
		case '/':
			comment++;
			data = 0;
			break;
		case '0'...'9':
			if (comment > 1)
				break;
			if (data == 0 && a == '0') {
				data = 1;
			} else if (data == 2) {
				data = 3;
				value = (a - '0') * 16;
			} else if (data == 3) {
				value += (a - '0');
				dst[count] = value;
				pr_info("[%d] = 0x%02X\n", count, value);
				count++;
				data = 0;
			}
			break;
		case 'a'...'f':
		case 'A'...'F':
			if (comment > 1)
				break;
			if (data == 2) {
				data = 3;
				if (a < 'a')
					value = (a - 'A' + 10) * 16;
				else
					value = (a - 'a' + 10) * 16;
			} else if (data == 3) {
				if (a < 'a')
					value += (a - 'A' + 10);
				else
					value += (a - 'a' + 10);
				dst[count] = value;
				pr_info("[%d] = 0x%02X\n", count, value);
				count++;
				data = 0;
			}
			break;
		case 'x':
		case 'X':
			if (data == 1)
				data = 2;
			break;
		default:
			if (comment == 1)
				comment = 0;
			data = 0;
			break;
		}
	}
	return count;
}

static int load_tuning_data(char *filename)
{
	struct file *filp;
	char *dp;
	long l;
	loff_t pos;
	int ret, num;
	mm_segment_t fs;

	pr_info("%s: loading (%s)\n", __func__, filename);
	fs = get_fs();
	set_fs(get_ds());

	filp = filp_open(filename, O_RDONLY, 0);
	if (IS_ERR(filp)) {
		pr_err("%s: failed to open (%ld)\n", __func__, IS_ERR(filp));
		return -1;
	}

	l = filp->f_path.dentry->d_inode->i_size;
	pr_info("%s: loading (%ld) bytes\n", __func__, l);

	dp = kmalloc(l + 10, GFP_KERNEL);
	if (dp == NULL) {
		pr_err("%s: not enough memory\n", __func__);
		filp_close(filp, current->files);
		return -1;
	}
	pos = 0;
	memset(dp, 0, l);
	ret = vfs_read(filp, (char __user *)dp, l, &pos);
	if (ret != l) {
		pr_err("%s: fail to read (%d)\n", __func__, ret);
		kfree(dp);
		filp_close(filp, current->files);
		return -1;
	}
	filp_close(filp, current->files);

	set_fs(fs);
	num = parse_text(mdnie_tuning_data, dp, l);
	if (!num) {
		pr_err("%s: parse error\n", __func__);
		kfree(dp);
		return -1;
	}
	pr_info("%s: loading count (%d)\n", __func__, num);
	kfree(dp);
	return num;
}

static ssize_t mdnie_tuning_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "filename : %s\n", tuning_filename);
}

static ssize_t mdnie_tuning_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct mdnie_lite *mdnie = dev_get_drvdata(dev);
	struct mdnie_config *config = &mdnie->config;
	char *fname, *name;
	unsigned int len, i;
	int num = 0;

	if (buf[0] == '0' || buf[0] == '1') {
		config->tuning = buf[0] - '0';
		pr_info("%s: tuning %s\n", __func__,
				config->tuning ? "enable" : "disable");
	} else {
		for (i = 0; i < ARRAY_SIZE(tuning_file_paths); i++) {
			len = strlen(tuning_file_paths[i]) + strlen(buf) + 1;
			fname = kzalloc(sizeof(char) * len, GFP_KERNEL);
			sprintf(fname, "%s%s", tuning_file_paths[i], buf);
			name = fname;
			while (*name) {
				if (*name == '\r' || *name == '\n') {
					*name = '\0';
					break;
				}
				name++;
			}
			pr_info("%s: Loading %s\n", __func__, fname);
			num = load_tuning_data(fname);
			if (num > 0) {
				strncpy(tuning_filename, fname, len);
				kfree(fname);
				break;
			}
			kfree(fname);
		}

		if (num <= 0) {
			pr_err("%s: failed to load (%s)\n", __func__, buf);
			return size;
		}
	}
	update_mdnie_mode(mdnie);

	return size;
}
#endif	/* ENABLE_MDNIE_TUNING */

/* Split 16 bit as 8bit x 2 */
#define BIT(nr)			(1UL << (nr))
#define GET_MSB_8BIT(x)     ((x >> 8) & (BIT(8) - 1))
#define GET_LSB_8BIT(x)     ((x >> 0) & (BIT(8) - 1))
#define NR_MDNIE_REG	6

static int mmp_dsi_panel_modify_accessibility_table(struct mdnie_lite *mdnie)
{
	int i, reg_e9 = 0;

	for (i = 0; i < NR_MDNIE_REG; i++)
		if (mdnie->color_adjustment_cmds.cmds[i].data[0] == 0xE9)
			reg_e9 = i;

	if (mdnie->color_adjustment_cmds.cmds[reg_e9].data[0] != 0xE9)
		goto err_wrong_reg;

	for (i = 0; i < ARRAY_SIZE(mdnie->scr); i++) {
		mdnie->color_adjustment_cmds.cmds[reg_e9].data[i * 2 + 1] =
			GET_LSB_8BIT(mdnie->scr[i]);
		mdnie->color_adjustment_cmds.cmds[reg_e9].data[i * 2 + 2] =
			GET_MSB_8BIT(mdnie->scr[i]);
	}

	return 1;

err_wrong_reg:
	pr_err("%s: access wrong register: 0x%2x\n", __func__,
			mdnie->color_adjustment_cmds.cmds[i].data[0]);
	return 0;
}

void update_mdnie_mode(struct mdnie_lite *mdnie)
{
	mutex_lock(&mdnie->ops_lock);
	if (mdnie->ops && mdnie->ops->set_mdnie)
		mdnie->ops->set_mdnie(&mdnie->config);
	mutex_unlock(&mdnie->ops_lock);
}
EXPORT_SYMBOL(update_mdnie_mode);

void update_accessibility_table(struct mdnie_lite *mdnie)
{
	mutex_lock(&mdnie->ops_lock);
	mmp_dsi_panel_modify_accessibility_table(mdnie);
	mutex_unlock(&mdnie->ops_lock);
}
EXPORT_SYMBOL(update_accessibility_table);

static ssize_t mdnie_scenario_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mdnie_lite *mdnie = dev_get_drvdata(dev);

	return sprintf(buf, "scenario : %d\n", mdnie->config.scenario);
}

static ssize_t mdnie_scenario_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct mdnie_lite *mdnie = dev_get_drvdata(dev);
	unsigned int value;

	if (kstrtoul(buf, 0, (unsigned long *)&value))
		return -EINVAL;

	if (mdnie->config.scenario != value) {
		mdnie->config.scenario = value;
		update_mdnie_mode(mdnie);
	}

	return size;
}

static ssize_t mdnie_accessibility_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mdnie_lite *mdnie = dev_get_drvdata(dev);

	return sprintf(buf, "accessibility : %d\n",
			mdnie->config.accessibility);
}

static ssize_t mdnie_accessibility_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int value, s[NUMBER_OF_SCR_DATA] = {0,};
	int ret, i;
	struct mdnie_lite *mdnie = dev_get_drvdata(dev);

	ret = sscanf(buf, "%d %x %x %x %x %x %x %x %x %x",
			&value, &s[0], &s[1], &s[2], &s[3],
			&s[4], &s[5], &s[6], &s[7], &s[8]);

	for (i = 0; i < NUMBER_OF_SCR_DATA; i++)
		mdnie->scr[i] = s[i];

	/*Modify color_adjustment table*/
	if (value == COLOR_BLIND)
		update_accessibility_table(mdnie);

	mdnie->config.accessibility = value;
	update_mdnie_mode(mdnie);

	return size;
}
static ssize_t mdnie_negative_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mdnie_lite *mdnie = dev_get_drvdata(dev);

	return sprintf(buf, "negative : %d\n", mdnie->config.negative);
}

static ssize_t mdnie_negative_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int value;

	struct mdnie_lite *mdnie = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, (unsigned long *)&value))
		return -EINVAL;

	mdnie->config.negative = !!value;
	update_mdnie_mode(mdnie);

	return size;
}

#ifdef ENABLE_MDNIE_TUNING
static DEVICE_ATTR(tuning, 0664, mdnie_tuning_show, mdnie_tuning_store);
#endif	/* ENABLE_MDNIE_TUNING */
static DEVICE_ATTR(scenario, 0664, mdnie_scenario_show, mdnie_scenario_store);
static DEVICE_ATTR(accessibility, 0664, mdnie_accessibility_show,
		mdnie_accessibility_store);
static DEVICE_ATTR(negative, 0664, mdnie_negative_show, mdnie_negative_store);

int mmp_panel_attach_mdnie(struct mdnie_lite *mdnie,
		const struct mdnie_ops *ops)
{
	if (mdnie->class || mdnie->dev)
		return 0;

	mdnie->class = class_create(THIS_MODULE, "mdnie");
	if (IS_ERR(mdnie->class)) {
		pr_warn("Unable to create mdnie class; errno = %ld\n",
				PTR_ERR(mdnie->class));
		return PTR_ERR(mdnie->class);
	}

	mdnie->dev = device_create(mdnie->class, NULL, 0, "%s", "mdnie");
#ifdef ENABLE_MDNIE_TUNING
	device_create_file(mdnie->dev, &dev_attr_tuning);
#endif	/* ENABLE_MDNIE_TUNING */
	device_create_file(mdnie->dev, &dev_attr_scenario);
	device_create_file(mdnie->dev, &dev_attr_accessibility);
	device_create_file(mdnie->dev, &dev_attr_negative);

	tuning_mode_cmds.cmds[0].length = mdnie->cmd_len;
	tuning_mode_cmds.cmds[0].data[0] = mdnie->cmd_reg;
	pr_info("%s: tuning command reg(%Xh) len(%d)\n", __func__,
			mdnie->cmd_reg,
			mdnie->cmd_len);

	mutex_init(&mdnie->ops_lock);
	mdnie->ops = ops;

	dev_set_drvdata(mdnie->dev, mdnie);
	pr_info("%s: done\n", __func__);

	return 0;
}
EXPORT_SYMBOL(mmp_panel_attach_mdnie);

void mmp_panel_detach_mdnie(struct mdnie_lite *mdnie)
{
	mutex_lock(&mdnie->ops_lock);
	mdnie->ops = NULL;
	mutex_unlock(&mdnie->ops_lock);
#ifdef ENABLE_MDNIE_TUNING
	device_remove_file(mdnie->dev, &dev_attr_tuning);
#endif	/* ENABLE_MDNIE_TUNING */
	device_remove_file(mdnie->dev, &dev_attr_accessibility);
	device_remove_file(mdnie->dev, &dev_attr_negative);
	device_remove_file(mdnie->dev, &dev_attr_scenario);
	device_destroy(mdnie->class, 0);
	class_destroy(mdnie->class);

	pr_info("%s: done\n", __func__);
}
EXPORT_SYMBOL(mmp_panel_detach_mdnie);
