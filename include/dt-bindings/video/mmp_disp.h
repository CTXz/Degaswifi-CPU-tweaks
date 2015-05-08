#ifndef _DT_BINDINGS_VIDEO_MMP_DISP_H
#define _DT_BINDINGS_VIDEO_MMP_DISP_H

/* mmp display */
#define PN_VID		0
#define PN_GRA		1
#define TV_VID		2
#define TV_GRA		3
#define PN2_VID		4
#define PN2_GRA		5
#define MAX_OVERLAY	6

#define PATH_OUT_PARALLEL	0
#define PATH_OUT_DSI		1
#define PATH_OUT_HDMI		2

#define PIXFMT_RGB565		0x100
#define PIXFMT_BGR565		0x101
#define PIXFMT_RGB1555		0x102
#define PIXFMT_BGR1555		0x103
#define PIXFMT_RGB888PACK	0x104
#define	PIXFMT_BGR888PACK	0x105
#define	PIXFMT_RGB888UNPACK	0x106
#define	PIXFMT_BGR888UNPACK	0x107
#define	PIXFMT_RGBA888		0x108
#define	PIXFMT_BGRA888		0x109

/* mmp generic panel */
#define EXT_PIN_GPIO		0
#define EXT_PIN_REGULATOR	1

#endif /* _DT_BINDINGS_VIDEO_MMP_DISP_H */
