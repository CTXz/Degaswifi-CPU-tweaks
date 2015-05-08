#ifndef _MAP_VIDEO_DEVICE_H
#define _MAP_VIDEO_DEVICE_H

#include <media/videobuf2-core.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/videobuf2-core.h>

#include <media/socisp-mdev.h>

enum isp_vnode_state {
	ISP_VNODE_STREAMOFF = 0,
	ISP_VNODE_STREAMON,
};

enum isp_vnode_mode {
	ISP_VNODE_MODE_STREAM	= 0,
	ISP_VNODE_MODE_SINGLE,
};

struct isp_format_convert_info {
	enum v4l2_mbus_pixelcode code;
	u32 pixelformat;
	unsigned int bpp;
	unsigned int num_planes;
};

struct isp_vnode {
	struct video_device	vdev;	/* has entity */

	/* buffer */
	spinlock_t		vb_lock;
	struct list_head	idle_buf;
	struct list_head	busy_buf;
	__u8			idle_buf_cnt;
	__u8			busy_buf_cnt;
	__u8			min_buf_cnt;
	__u8			mode;
	struct vb2_alloc_ctx	*alloc_ctx;
	struct vb2_queue	vq;
	enum v4l2_buf_type	buf_type;

	/* H/W and MC */
	struct isp_pipeline	*pipeline;
	struct media_pad	pad;
	struct isp_event	*event_qbuf;
	struct isp_event	*event_stream;

	/* state and pipeline-video coherency */
	struct mutex		st_lock;
	enum isp_vnode_state	state;

	struct mutex		vdev_lock;	/* lock for vdev */
	struct v4l2_format	format;		/* internal format cache */
};

struct isp_videobuf {
	struct vb2_buffer	vb;
	dma_addr_t		paddr[VIDEO_MAX_PLANES];
	struct list_head	hook;
};

struct isp_format_desc {
	enum v4l2_mbus_pixelcode code;
	u32 pixelformat;
	unsigned int bpp;
	unsigned int num_planes;
};

#define ALIGN_SIZE	16/*(cache_line_size())*/
#define ALIGN_MASK	(ALIGN_SIZE - 1)

int isp_vnode_add(struct isp_vnode *vnode, struct v4l2_device *vdev,
			int dir, int vdev_nr);
void isp_vnode_remove(struct isp_vnode *vdev);
struct isp_vnode *ispsd_get_video(struct isp_subdev *ispsd);
struct isp_subdev *video_get_ispsd(struct isp_vnode *video);

/* Should be called upon every IRQ, and before streamon. */
/* First fetch full buffer from DMA-busy queue,
 * if there is enough buffer to susutain DMA. Then move a idle buffer to busy
 * queue if any. The function will access idle queue and busy queue with a lock
 * hold */
struct isp_videobuf *isp_vnode_xchg_buffer(struct isp_vnode *vnode,
						bool discard);
struct isp_videobuf *isp_vnode_get_buffer(struct isp_vnode *vnode,
						int hw_buf_depth);

#define MC_PIPELINE_LEN_MAX	10

enum pipeline_state {
	ISP_PIPELINE_DEAD = 0,
	ISP_PIPELINE_IDLE,
	ISP_PIPELINE_BUSY,
};

struct isp_stream {
	struct isp_subdev	shell;
	u32			id;

	struct media_entity	*def_src;
	struct media_entity	*input;
	struct isp_vnode	*output;

	int			drv_own:1; /* driver setup this pipeline with
						default configuration */
	enum pipeline_state	state;
	struct mutex		st_lock;
	struct mutex		fmt_lock; /* MUST hold this lock in G/S/T_FMT */
	int (*find_link)(struct media_entity *start, struct media_entity *end,
				struct media_link **link, int max_pads,
				int flag_set, int flag_clr);
};

struct isp_pipeline {
	struct list_head	hook;
	u32			id;

	struct media_entity	*def_src;
	struct media_entity	*input;
	struct isp_vnode	*output;

	struct media_link	*link[MC_PIPELINE_LEN_MAX];
	int			num_links;
	struct isp_build	*mngr;
	int			drv_own:1; /* driver setup this pipeline with
						default configuration */
	enum pipeline_state	state;
	struct mutex		st_lock;

	struct mutex		fmt_lock; /* MUST hold this lock in G/S/T_FMT */
	int (*find_link)(struct media_entity *start, struct media_entity *end,
				struct media_link **link, int max_pads,
				int flag_set, int flag_clr);
};

struct isp_pipeline *isp_pipeline_create(struct isp_build *build);
int isp_pipeline_setup(struct isp_pipeline *pipe);
void isp_pipeline_clean(struct isp_pipeline *pipe);
int isp_pipeline_validate(struct isp_pipeline *pipe);

int isp_pipeline_querycap(struct isp_pipeline *pipeline,
			struct v4l2_capability *cap);
int isp_pipeline_set_format(struct isp_pipeline *pipeline,
				struct v4l2_mbus_framefmt *fmt);
int isp_pipeline_get_format(struct isp_pipeline *pipeline,
				struct v4l2_mbus_framefmt *fmt);
int isp_pipeline_try_format(struct isp_pipeline *pipeline,
				struct v4l2_mbus_framefmt *fmt);
int isp_pipeline_set_stream(struct isp_pipeline *, int enable);

#endif
