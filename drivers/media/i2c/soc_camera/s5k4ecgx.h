/*
 * drivers/media/video/s5k4ecgx.h
 *
 * Register definitions for the S5K4ECGX camera from Samsung Electronics
 *
 *
 * Copyright (C) 2010 Samsung Electronics
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */
#ifndef S5K4ECGX_H
#define S5K4ECGX_H

#define CAM_S5K4ECGX_DBG_MSG            1
//#define USE_SD_CARD_TUNE
#define S5K4ECGX_DRIVER_NAME            "s5k4ecgx"
#define S5K4ECGX_MOD_NAME               "S5K4ECGX: "

#define S5K4ECGX_THUMBNAIL_OFFSET    	0x271000		// 0x271000		0x1EA000
#define S5K4ECGX_YUV_OFFSET		0x280A00		// 0x280A00		0x224980

#define S5K4ECGX_I2C_ADDR		(0x5A >> 1)
#define S5K4ECGX_I2C_RETRY		1
#define S5K4ECGX_XCLK			24000000

#define SENSOR_DETECTED			1
#define SENSOR_NOT_DETECTED		0

#define JPEG_CAPTURE_WIDTH  960
#define JPEG_CAPTURE_HEIGHT 3072

struct s5k4ecgx_position {
	int x;
	int y;
};

struct s5k4ecgx_datafmt {
	enum v4l2_mbus_pixelcode	code;
	enum v4l2_colorspace		colorspace;
};

/*for EXIF2_2, workaround fix by Vincent wan, don't set it in vediodev2.h */
typedef struct rational {
	__u32 denominal;
	__u32 inumerator;
} rational_t;

typedef struct v4l2_exif_info {
//	srational_t shutter_speed;
	rational_t exposure_time;
	__u16 iso_speed_rationg;
} v4l2_exif_info_t;

/**
 * struct s5k4ecgx_platform_data - platform data values and access functions
 * @power_set: Power state access function, zero is off, non-zero is on.
 * @ifparm: Interface parameters access function
 * @priv_data_set: device private data (pointer) access function
 */
struct s5k4ecgx_platform_data {
	int (*power_set)(enum v4l2_power power);
	int (*ifparm)(struct v4l2_ifparm *p);
	int (*priv_data_set)(void *);
};

/**
 * struct s5k4ecgx_sensor - main structure for storage of sensor information
 * @pdata: access functions and data for platform level information
 * @v4l2_int_device: V4L2 device structure structure
 * @i2c_client: iic client device structure
 * @pix: V4L2 pixel format information structure
 * @timeperframe: time per frame expressed as V4L fraction
 * @scaler:
 * @ver: s5k4ecgx chip version
 * @fps: frames per second value
 */
struct s5k4ecgx_sensor {
	const struct s5k4ecgx_platform_data *pdata;
	struct v4l2_int_device *v4l2_int_device;
	struct v4l2_pix_format pix;
	struct v4l2_fract timeperframe;
	struct s5k4ecgx_position position;
	int check_dataline;
	u32 state;
	u8 mode;
	u8 fps;
	//u16 bv;
	u8 preview_size;
	u8 capture_size;
	u8 focus_mode;
	u8 focus_type;
	u8 detect;
	u8 effect;
	u8 iso;
	u8 photometry;
	u8 ev;
	u8 wdr;
	u8 contrast;
	u8 saturation;
	u8 sharpness;
	u8 wb;
	//u8 isc;
	u8 scene;
	u8 ae;
	u8 awb;
	//u8 antishake;
	//u8 flash_capture;
	//u8 flash_movie;
	u8 flash_mode;
	u8 quality;
	//s32 zoom;
	u32 thumb_offset;
	u32 yuv_offset;
	u32 jpeg_main_size;
	u32 jpeg_main_offset;
	u32 jpeg_thumb_size;
	u32 jpeg_thumb_offset;
	u32 jpeg_postview_offset;
	u32 jpeg_capture_w;
	u32 jpeg_capture_h;
	struct v4l2_pix_format thumbnail;
	v4l2_exif_info_t exif_info;
	u8 initial;
	u8 main_flash_mode;
	u8 cam_mode;
};

#define S5K4ECGX_STATE_INITIAL		0x0000
#define S5K4ECGX_STATE_STABLE		0x0001

/* State */
#define S5K4ECGX_STATE_PREVIEW	  0x0000	/*  preview state */
#define S5K4ECGX_STATE_CAPTURE	  0x0001	/*  capture state */
#define S5K4ECGX_STATE_INVALID	  0x0002	/*  invalid state */

/* Mode */
#define S5K4ECGX_MODE_CAMERA     1
#define S5K4ECGX_MODE_CAMCORDER  2
#define S5K4ECGX_MODE_VT         3

/* Preview Size */
#define S5K4ECGX_PREVIEW_SIZE_144_176		0
#define S5K4ECGX_PREVIEW_SIZE_176_144		1
#define S5K4ECGX_PREVIEW_SIZE_320_240		2
#define S5K4ECGX_PREVIEW_SIZE_352_288		3
#define S5K4ECGX_PREVIEW_SIZE_640_480		4
#define S5K4ECGX_PREVIEW_SIZE_720_480		5
#define S5K4ECGX_PREVIEW_SIZE_800_480		6
#define S5K4ECGX_PREVIEW_SIZE_1280_720		7

/* Camcorder Preview Size */
#define S5K4ECGX_CAMCORDER_SIZE_144_176		0
#define S5K4ECGX_CAMCORDER_SIZE_176_144		1
#define S5K4ECGX_CAMCORDER_SIZE_320_240		2
#define S5K4ECGX_CAMCORDER_SIZE_352_288		3
#define S5K4ECGX_CAMCORDER_SIZE_640_480		4
#define S5K4ECGX_CAMCORDER_SIZE_720_480		5
#define S5K4ECGX_CAMCORDER_SIZE_800_480		6
#define S5K4ECGX_CAMCORDER_SIZE_1280_720	7

/* Focus Mode */
#define S5K4ECGX_AF_SET_NORMAL		1
#define S5K4ECGX_AF_SET_MACRO		2
#define S5K4ECGX_AF_SET_OFF		3
#define S5K4ECGX_AF_SET_NORMAL_1	10
#define S5K4ECGX_AF_SET_NORMAL_2	11
#define S5K4ECGX_AF_SET_NORMAL_3	12
#define S5K4ECGX_AF_SET_MACRO_1		15
#define S5K4ECGX_AF_SET_MACRO_2		16
#define S5K4ECGX_AF_SET_MACRO_3		17

/* Focust start/stop */
#define S5K4ECGX_AF_START			1
#define S5K4ECGX_AF_STOP			2
#define S5K4ECGX_AF_STOP_STEP_1			10
#define S5K4ECGX_AF_STOP_STEP_2			11
#define S5K4ECGX_AF_STOP_STEP_3			12

/* Auto Focus Status */
#define S5K4ECGX_AF_STATUS_PROGRESS	1
#define S5K4ECGX_AF_STATUS_SUCCESS	2
#define S5K4ECGX_AF_STATUS_FAIL		3
#define S5K4ECGX_AF_STATUS_CANCELED	4
#define S5K4ECGX_AF_STATUS_TIMEOUT	5
#define S5K4ECGX_AE_STATUS_STABLE	6
#define S5K4ECGX_AE_STATUS_UNSTABLE	7

#define S5K4ECGX_AF_CHECK_STATUS	0
#define S5K4ECGX_AF_OFF			1
#define S5K4ECGX_AF_DO			4
#define S5K4ECGX_AF_SET_MANUAL		5
#define S5K4ECGX_AF_CHECK_2nd_STATUS	6
#define S5K4ECGX_AF_SET_AE_FOR_FLASH	7
#define S5K4ECGX_AF_BACK_AE_FOR_FLASH	8
#define S5K4ECGX_AF_CHECK_AE_STATUS	9
#define S5K4ECGX_AF_POWEROFF		10

#define S5K4ECGX_FLASH_OFF		0
#define S5K4ECGX_FLASH_ON		1
#define S5K4ECGX_FLASH_AUTO		2
#define S5K4ECGX_FLASH_TORCH		3

#define FLASH_OFF		0
#define CAPTURE_FLASH	1
#define MOVIE_FLASH		2
#define PRE_CAPTURE_FLASH	3

#define FLASH_PULSE_CNT 4

/* Preview Size */
enum {
	PREVIEW_SIZE_176_144,
	PREVIEW_SIZE_320_240,
	PREVIEW_SIZE_424_318,
	PREVIEW_SIZE_480_320,
	PREVIEW_SIZE_640_480,
	PREVIEW_SIZE_720_480,
	PREVIEW_SIZE_720_540,
	PREVIEW_SIZE_960_540,
	PREVIEW_SIZE_1280_720,
};

struct s5k4ecgx_preview_size {
	unsigned long width;
	unsigned long height;
};

const static struct s5k4ecgx_preview_size s5k4ecgx_preview_sizes[] = {
	{176,144}, 	// QCIF 
	{320,240},	// QVGA
	{424, 318},
	{480,320},	// XVGA
	{640,480},	// VGA
	{720,480},	// D1
	{720,540},	// QHD(4:3)
	{960,540},	// QHD
	{1280,720},	// HD
};

/* Image Size */
enum {
	CAPTURE_SIZE_160_120,
	CAPTURE_SIZE_176_144,
	CAPTURE_SIZE_320_240,
	CAPTURE_SIZE_352_288,
	CAPTURE_SIZE_480_320,
	CAPTURE_SIZE_640_480,
	CAPTURE_SIZE_720_480,
	CAPTURE_SIZE_800_480,
	CAPTURE_SIZE_800_600,
	CAPTURE_SIZE_1024_768,
	CAPTURE_SIZE_1280_848,
	CAPTURE_SIZE_1280_960,
	CAPTURE_SIZE_1600_960,
	CAPTURE_SIZE_1600_1072,
	CAPTURE_SIZE_1600_1200,
	CAPTURE_SIZE_2048_1232,
	CAPTURE_SIZE_2048_1360,
	CAPTURE_SIZE_2048_1536,
	CAPTURE_SIZE_2560_1536,
	CAPTURE_SIZE_2560_1920,
};

struct s5k4ecgx_capture_size {
	unsigned long width;
	unsigned long height;
};

/* Image sizes */
const static struct s5k4ecgx_capture_size s5k4ecgx_image_sizes[] = {
	{160, 120},
	{176, 144},
	{320, 240},
	{352, 288},
	{480, 320},
	{640, 480},
	{720, 480},
	{800, 600},
	{1024, 768},
	{1280, 848},
	{1280, 960},
	{1600, 1072},
	{1600, 1200},
	{2048, 1360},
	{2048, 1536},
};

/* Image Effect */
enum {
	EFFECT_OFF,
	EFFECT_MONO,
	EFFECT_SEPIA,
	EFFECT_NEGATIVE,
	EFFECT_AQUA,
	EFFECT_SKETCH,
};

/* White Balance */
enum {
	WB_AUTO,
	WB_DAYLIGHT,
	WB_CLOUDY,
	WB_FLUORESCENT,
	WB_INCANDESCENT,
};

/* EV */
enum {
	EV_MINUS_4,
	EV_MINUS_3,
	EV_MINUS_2,
	EV_MINUS_1,
	EV_DEFAULT,
	EV_PLUS_1,
	EV_PLUS_2,
	EV_PLUS_3,
	EV_PLUS_4,
};

/* Scene Mode */
enum {
	SCENE_OFF,
	SCENE_PORTRAIT,
	SCENE_LANDSCAPE,
	SCENE_SPORTS,
	SCENE_PARTY,
	SCENE_BEACH,
	SCENE_SUNSET,
	SCENE_DAWN,
	SCENE_FALL,
	SCENE_NIGHT,
	SCENE_BACKLIGHT,
	SCENE_FIRE,
	SCENE_TEXT,
	SCENE_CANDLE,
	SCENE_SPORTS_OFF,
	SCENE_NIGHT_OFF,
};

/* Contrast */
enum {
	CONTRAST_MINUS_2,
	CONTRAST_MINUS_1,
	CONTRAST_DEFAULT,
	CONTRAST_PLUS_1,
	CONTRAST_PLUS_2,
};

/* Saturation */
enum {
	SATURATION_MINUS_2,
	SATURATION_MINUS_1,
	SATURATION_DEFAULT,
	SATURATION_PLUS_1,
	SATURATION_PLUS_2,
};

/* Sharpness */
enum {
	SHARPNESS_MINUS_2,
	SHARPNESS_MINUS_1,
	SHARPNESS_DEFAULT,
	SHARPNESS_PLUS_1 ,
	SHARPNESS_PLUS_2,
};

/* Photometry */
enum {
	METERING_MATRIX,
	METERING_SPOT,
	METERING_CENTER,
};

/* ISO */
enum {
	ISO_AUTO,
	ISO_50,
	ISO_100,
	ISO_200,
	ISO_400,
};

/* fps */
enum {
	FPS_auto 	= 0,
	FPS_7		= 7,
	FPS_10		= 10,
	FPS_12		= 12,
	FPS_15		= 15,
	FPS_20		= 20,
	FPS_25		= 25,
	FPS_30		= 30,
};

/* Auto Exposure & Auto White Balance */
enum {
	AE_LOCK,
	AE_UNLOCK,
	AWB_LOCK,
	AWB_UNLOCK,
};

enum {
	AWB_AE_LOCK,
	AWB_AE_UNLOCK,
};

enum {
	MAIN_FLASH_OFF,
	MAIN_FLASH_ON,
};

enum {
	AUTO_FOCUS_MODE,
	TOUCH_FOCUS_MODE,
};

/* JPEG Quality */
enum {
	QUALITY_SUPERFINE,
	QUALITY_FINE,
	QUALITY_NORMAL,
};

/* Lightness Check for Preview */
enum {
	Lightness_Low,
	Lightness_Normal,
};

/*ESD Status*/
enum {
	ESD_ERROR,
	ESD_NONE,
};

/* Preview Mode */
enum {
	CAMERA_MODE,
	CAMCORDER_MODE,
	VT_MODE
};


typedef enum {
	CAMERA_SENSOR_LIGHT_STATUS_LOW,
	CAMERA_SENSOR_LIGHT_STATUS_NORMAL,
	CAMERA_SENSOR_LIGHT_STATUS_HIGH,
	CAMERA_SENSOR_LIGHT_STATUS_INVALID,
} camera_light_status_type;

typedef enum {
	CAMERA_SENSOR_AE_UNSTABLE,
	CAMERA_SENSOR_AE_STABLE,
	CAMERA_SENSOR_AE_INVALID,
} camera_ae_stable_type;

typedef enum {
	SET_FOCUS_STEP1,
	SET_FOCUS_STEP2,
	SET_FOCUS_STEP3,
	SET_FOCUS_ALL
} camera_set_focus_type;

#endif /* ifndef S5K4ECGX_H */

