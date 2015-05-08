#include <linux/module.h>
#include <linux/export.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>

#include <media/socisp-mdev.h>
#include <media/socisp-vdev.h>

static int trace = 2;
module_param(trace, int, 0644);
MODULE_PARM_DESC(trace,
		"how many trace do you want to see? (0-4):"
		"0 - mute "
		"1 - only actual errors"
		"2 - milestone log"
		"3 - briefing log"
		"4 - detailed log");

static struct isp_format_desc isp_format_table[] = {
	{ V4L2_MBUS_FMT_SBGGR10_1X10,	V4L2_PIX_FMT_SBGGR10,	10,	1},
	{ V4L2_MBUS_FMT_UYVY8_1X16,	V4L2_PIX_FMT_UYVY,	16,	1},
	{ V4L2_MBUS_FMT_Y12_1X12,	V4L2_PIX_FMT_YUV420M,	12,	3},
	{ V4L2_MBUS_FMT_YUYV8_1_5X8,	V4L2_PIX_FMT_NV12M,	12,	2},
	{ V4L2_MBUS_FMT_YVYU8_1_5X8,	V4L2_PIX_FMT_NV21M,	12,	2},
	{ V4L2_MBUS_FMT_SBGGR8_1X8,	V4L2_PIX_FMT_SBGGR8,	8,	1},
/*FIXME: UYVY*_1_5X8  for fvts test. MBUS does not
 * distinguish packed and planar format. YUV420M use 3 planes.
 * YUV420 use 1 planes. NV12M and NV21M use 2 planes.Y*/
	{ V4L2_MBUS_FMT_YUYV8_1_5X8,    V4L2_PIX_FMT_YUV420,    12,     1},
};

static int isp_video_calc_mplane_sizeimage(
		struct v4l2_pix_format_mplane *pix_mp, int idx)
{
	unsigned int width = pix_mp->width;
	unsigned int height = pix_mp->height;
	unsigned int pitch;
	int ret = 0;

	switch (pix_mp->pixelformat) {
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_SBGGR10:
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_YUV420:
		pitch = isp_format_table[idx].bpp * width >> 3;
		pix_mp->plane_fmt[0].bytesperline = pitch;
		pix_mp->plane_fmt[0].sizeimage = pitch * height;
		break;
	case V4L2_PIX_FMT_NV12M:
	case V4L2_PIX_FMT_NV21M:
		pitch = width;
		pix_mp->plane_fmt[0].bytesperline = pitch;
		pix_mp->plane_fmt[0].sizeimage = pitch * height;

		pitch = width >> 1;
		pix_mp->plane_fmt[1].bytesperline = pitch;
		pix_mp->plane_fmt[1].sizeimage =  pitch * height / 2;
		break;
	case V4L2_PIX_FMT_YUV420M:
		pitch = width;
		pix_mp->plane_fmt[0].bytesperline = pitch;
		pix_mp->plane_fmt[0].sizeimage = pitch * height;

		pitch = width >> 1;
		pix_mp->plane_fmt[1].bytesperline = pitch;
		pix_mp->plane_fmt[1].sizeimage =  pitch * height / 2;

		pix_mp->plane_fmt[2].bytesperline = pitch;
		pix_mp->plane_fmt[2].sizeimage = pitch * height / 2;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int isp_video_mbus_to_pix(const struct v4l2_mbus_framefmt *mbus,
					struct v4l2_pix_format_mplane *pix_mp)
{
	unsigned int i;

	memset(pix_mp, 0, sizeof(*pix_mp));

	for (i = 0; i < ARRAY_SIZE(isp_format_table); ++i) {
		if (isp_format_table[i].code == mbus->code)
			break;
	}
	if (WARN_ON(i >= ARRAY_SIZE(isp_format_table)))
		return -EINVAL;

	pix_mp->width = mbus->width;
	pix_mp->height = mbus->height;
	pix_mp->pixelformat = isp_format_table[i].pixelformat;
	pix_mp->colorspace = mbus->colorspace;
	pix_mp->field = mbus->field;
	pix_mp->num_planes = isp_format_table[i].num_planes;

	return isp_video_calc_mplane_sizeimage(pix_mp, i);
}

static int isp_video_pix_to_mbus(struct v4l2_pix_format_mplane *pix_mp,
				  struct v4l2_mbus_framefmt *mbus)
{
	unsigned int i;

	memset(mbus, 0, sizeof(*mbus));

	for (i = 0; i < ARRAY_SIZE(isp_format_table); ++i) {
		if (isp_format_table[i].pixelformat == pix_mp->pixelformat)
			break;
	}

	if (WARN_ON(i >= ARRAY_SIZE(isp_format_table)))
		return -EINVAL;

	if (isp_format_table[i].num_planes != pix_mp->num_planes)
		return -EINVAL;

	mbus->code = isp_format_table[i].code;
	mbus->width = pix_mp->width;
	mbus->height = pix_mp->height;
	mbus->colorspace = pix_mp->colorspace;
	mbus->field = pix_mp->field;

	return 0;
}

struct isp_vnode *ispsd_get_video(struct isp_subdev *ispsd)
{
	struct media_entity *vdev_me, *me = &ispsd->subdev.entity;
	int i;

	if (ispsd == NULL || (!ispsd->single))
		return NULL;

	if ((ispsd->sd_type != ISP_BLOCK_DMA_OUT)
		&& (ispsd->sd_type != ISP_BLOCK_DMA_IN))
		return NULL;

	for (i = 0; i < me->num_links; i++) {
		struct media_link *link = &me->links[i];

		if (link->source->entity == me)
			vdev_me = link->sink->entity;
		else
			vdev_me = link->source->entity;
		if (vdev_me->type == MEDIA_ENT_T_DEVNODE_V4L)
			goto find;
	}
	return NULL;
find:
	return video_get_drvdata(media_entity_to_video_device(vdev_me));
}

struct isp_subdev *video_get_ispsd(struct isp_vnode *vnode)
{
	struct media_entity *ispsd_me, *me = &vnode->vdev.entity;
	struct isp_subdev *ispsd;
	int i;

	if (unlikely(vnode == NULL))
		return NULL;

	for (i = 0; i < me->num_links; i++) {
		struct media_link *link = &me->links[i];
		if (link->source->entity == me)
			ispsd_me = link->sink->entity;
		else
			ispsd_me = link->source->entity;
		if (ispsd_me->type == MEDIA_ENT_T_V4L2_SUBDEV)
			goto find;
	}
	return NULL;
find:
	ispsd = me_to_ispsd(ispsd_me);
	if (!ispsd || (!ispsd->single) ||
		((ispsd->sd_type != ISP_BLOCK_DMA_OUT) &&
		(ispsd->sd_type != ISP_BLOCK_DMA_IN)))
		return NULL;
	return ispsd;
}

/* buffer exchange function used in ISR context, to deliver hot frame to VB,
 * and get new buffer for incoming frame */
struct isp_videobuf *isp_vnode_xchg_buffer(struct isp_vnode *vnode,
						bool discard)
{
	struct isp_videobuf *del_buf = NULL, *new_buf = NULL;

	spin_lock(&vnode->vb_lock);

	/* Grab the hot frame first */
	del_buf = list_first_entry(&vnode->busy_buf, struct isp_videobuf, hook);
	list_del_init(&del_buf->hook);
	vnode->busy_buf_cnt--;

	if (discard) {
		new_buf = del_buf;
		/* use discarded buffer as new buffer, don't bother find another
		 * new buffer */
		goto attach_frame;
	}

	/* in streaming mode: if no more buffer on idle Q, have to steal buffer
	 * from busy Q, and this means frame drop! */
	if ((vnode->mode == ISP_VNODE_MODE_STREAM)
		&& (vnode->idle_buf_cnt <= 0)) {
		new_buf = del_buf;
		goto attach_frame;
	}

	/* Actually deliver the hot frame to VB */
	v4l2_get_timestamp(&del_buf->vb.v4l2_buf.timestamp);
	vb2_buffer_done(&del_buf->vb, VB2_BUF_STATE_DONE);

	/* Reaching this step means either in none-streaming mode,
	 * or in streaming mode, and the stream is in normal state */
	/* in both case, try to move a buffer from idle to busy Q */
	if (vnode->idle_buf_cnt) {
		new_buf = list_first_entry(&vnode->idle_buf,
						struct isp_videobuf, hook);
		list_del(&new_buf->hook);
		vnode->idle_buf_cnt--;
		goto attach_frame;
	}

	/* Silent frame drop due to buffer shortage is allowed only in
	 * none-streaming mode! */
	BUG_ON(vnode->mode == ISP_VNODE_MODE_STREAM);
	/* This time, in none-streaming mode, we already start to drop frame */

attach_frame:
	if (new_buf) {
		list_add_tail(&new_buf->hook, &vnode->busy_buf);
		vnode->busy_buf_cnt++;
	}
	spin_unlock(&vnode->vb_lock);
	return new_buf;
}
EXPORT_SYMBOL(isp_vnode_xchg_buffer);

/* buffer retrive function used in process context, to get one idle buffer */
struct isp_videobuf *isp_vnode_get_buffer(struct isp_vnode *vnode,
						int hw_buf_depth)
{
	struct isp_videobuf *new_buf = NULL;
	unsigned long flags;

	/* In process context, it's possible that ISR jump in,
	 * so must lock & disable IRQ */
	spin_lock_irqsave(&vnode->vb_lock, flags);
	if (vnode->busy_buf_cnt >= hw_buf_depth)
		goto unlock;

	if (vnode->idle_buf_cnt) {
		new_buf = list_first_entry(&vnode->idle_buf,
				struct isp_videobuf, hook);
		list_move_tail(&new_buf->hook, &vnode->busy_buf);
		vnode->idle_buf_cnt--;
		vnode->busy_buf_cnt++;
	}

unlock:
	spin_unlock_irqrestore(&vnode->vb_lock, flags);
	return new_buf; /* should be written to DMA H/W */
}
EXPORT_SYMBOL(isp_vnode_get_buffer);

/********************************** VB2 ops ***********************************/

static int isp_vbq_setup(struct vb2_queue *vq, const struct v4l2_format *fmt,
			unsigned int *num_buffers, unsigned int *num_planes,
			unsigned int sizes[], void *alloc_ctxs[])
{
	struct isp_vnode *vnode = vb2_get_drv_priv(vq);
	struct v4l2_pix_format_mplane *pix_mp = &vnode->format.fmt.pix_mp;
	int i;

	if (*num_buffers < vnode->min_buf_cnt)
		*num_buffers = vnode->min_buf_cnt;

	*num_planes = pix_mp->num_planes;
	for (i = 0; i < *num_planes; i++) {
		sizes[i] = ALIGN(pix_mp->plane_fmt[i].sizeimage,
				ALIGN_SIZE);
		alloc_ctxs[i] = vnode->alloc_ctx;
	}

	INIT_LIST_HEAD(&vnode->busy_buf);
	INIT_LIST_HEAD(&vnode->idle_buf);

	vnode->busy_buf_cnt = 0;
	vnode->idle_buf_cnt = 0;

	return 0;
}

static void isp_vb_cleanup(struct vb2_buffer *vb)
{
	return;
}

static int isp_vb_init(struct vb2_buffer *vb)
{
	struct isp_videobuf *isp_vb = container_of(vb, struct isp_videobuf, vb);
	int i;

	for (i = 0; i < VIDEO_MAX_PLANES; i++)
		isp_vb->paddr[i] = 0;
	return 0;
}

static int isp_vb_prepare(struct vb2_buffer *vb)
{
	struct isp_vnode *vnode = container_of(vb->vb2_queue,
					struct isp_vnode, vq);
	struct isp_videobuf *isp_vb = container_of(vb, struct isp_videobuf, vb);
	struct v4l2_pix_format_mplane *pix_mp = &vnode->format.fmt.pix_mp;
	unsigned long size;
	int i;

	if (vb->num_planes < pix_mp->num_planes)
		return -EINVAL;
	vb->num_planes = pix_mp->num_planes;
	for (i = 0; i < vb->num_planes; i++) {
		size = vb2_plane_size(vb, i);
		vb2_set_plane_payload(vb, i, size);
	}
	INIT_LIST_HEAD(&isp_vb->hook);
	return 0;
}

static void isp_vb_queue(struct vb2_buffer *vb)
{
	struct isp_vnode *vnode = container_of(vb->vb2_queue,
					struct isp_vnode, vq);
	struct isp_videobuf *isp_vb = container_of(vb, struct isp_videobuf, vb);
	dma_addr_t dma_handle;
	unsigned long flags;
	int i;

	for (i = 0; i < vb->num_planes; i++) {
		dma_handle = vb2_dma_contig_plane_dma_addr(&isp_vb->vb, i);
		/*  adjust the dma address according to the offset */
		dma_handle += isp_vb->vb.v4l2_planes[i].data_offset;
		BUG_ON(!dma_handle);
		isp_vb->paddr[i] = dma_handle;
	}
	spin_lock_irqsave(&vnode->vb_lock, flags);
	list_add_tail(&isp_vb->hook, &vnode->idle_buf);
	vnode->idle_buf_cnt++;
	spin_unlock_irqrestore(&vnode->vb_lock, flags);

	vnode->event_qbuf->owner = vnode;
	vnode->event_qbuf->msg = vnode;
	WARN_ON(unlikely(isp_event_dispatch(vnode->event_qbuf) < 0));

	d_inf(4, "%s: buffer<%d> pushed into driver", vnode->vdev.name,
		vb->v4l2_buf.index);
	return;
}

static void isp_vb_lock(struct vb2_queue *vq)
{
	struct isp_vnode *vnode = vb2_get_drv_priv(vq);
	mutex_lock(&vnode->vdev_lock);
	return;
}

static void isp_vb_unlock(struct vb2_queue *vq)
{
	struct isp_vnode *vnode = vb2_get_drv_priv(vq);
	mutex_unlock(&vnode->vdev_lock);
	return;
}

static struct vb2_ops isp_vnode_vb2_ops = {
	.queue_setup	= isp_vbq_setup,
	.buf_prepare	= isp_vb_prepare,
	.buf_queue	= isp_vb_queue,
	.buf_cleanup	= isp_vb_cleanup,
	.buf_init	= isp_vb_init,
	.wait_prepare	= isp_vb_unlock,
	.wait_finish	= isp_vb_lock,
};

static int isp_vnode_vb2_init(struct vb2_queue *vq, struct isp_vnode *vnode)
{
	vq->type	= vnode->buf_type;
	vq->io_modes	= VB2_USERPTR | VB2_DMABUF;
	vq->drv_priv	= vnode;
	vq->ops		= &isp_vnode_vb2_ops;
	vq->mem_ops	= &vb2_dma_contig_memops;
	vq->buf_struct_size	= sizeof(struct isp_videobuf);
	vq->timestamp_type	= V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;

	/* FIXME: hard coding, not all DMA device needs contig mem, try pass
	 * alloc_ctx handle, and call */
	vnode->alloc_ctx = vb2_dma_contig_init_ctx(&vnode->vdev.dev);
	return vb2_queue_init(vq);
}

/********************************* V4L2 APIs *********************************/

static int isp_vnode_querycap(struct file *file, void *fh,
				struct v4l2_capability *cap)
{
	struct isp_vnode *vnode = video_drvdata(file);
	int ret = 0;

	if (vnode->pipeline == NULL)
		return -EPIPE;

	ret = isp_pipeline_querycap(vnode->pipeline, cap);
	if (ret < 0)
		return ret;

	if (V4L2_TYPE_IS_OUTPUT(vnode->buf_type)) {
		cap->capabilities = V4L2_CAP_VIDEO_OUTPUT_MPLANE |
			V4L2_CAP_STREAMING;
	} else {
		cap->capabilities = V4L2_CAP_VIDEO_CAPTURE_MPLANE |
			V4L2_CAP_STREAMING;
	}

	return ret;
}

static int isp_vnode_get_format(struct file *file, void *fh,
				struct v4l2_format *format)
{
	struct isp_vnode *vnode = video_drvdata(file);
	struct isp_pipeline *pipeline = vnode->pipeline;
	struct v4l2_mbus_framefmt fmt;
	int ret = 0;

	if (format->type != vnode->buf_type)
		return -EINVAL;

	mutex_lock(&vnode->pipeline->fmt_lock);
	ret = isp_pipeline_get_format(pipeline, &fmt);
	if (ret)
		goto out;
	ret = isp_video_mbus_to_pix(&fmt, &format->fmt.pix_mp);
	if (ret)
		goto out;
out:
	mutex_unlock(&vnode->pipeline->fmt_lock);
	return ret;
}

static int isp_vnode_set_format(struct file *file, void *fh,
				struct v4l2_format *format)
{
	struct isp_vnode *vnode = video_drvdata(file);
	struct v4l2_mbus_framefmt fmt;
	int ret = 0;

	if (format->type != vnode->buf_type)
		return -EINVAL;

	ret = isp_video_pix_to_mbus(&format->fmt.pix_mp, &fmt);
	if (ret)
		goto out;

	if (vnode->pipeline) {
		d_inf(2, "%s: set mbus fmt:%X(%d, %d)", vnode->vdev.name,
			fmt.code, fmt.width, fmt.height);
		mutex_lock(&vnode->pipeline->fmt_lock);
		ret = isp_pipeline_set_format(vnode->pipeline, &fmt);
		/* TODO: also change pipeline state here */
		mutex_unlock(&vnode->pipeline->fmt_lock);
		if (ret)
			goto out;
	} else {
		struct media_pad *pad = media_entity_remote_source(&vnode->pad);
		struct v4l2_subdev *sd;
		struct v4l2_subdev_format sd_fmt;

		if (pad == NULL || pad->entity == NULL)
			return -EPIPE;
		sd = media_entity_to_v4l2_subdev(pad->entity);
		memcpy(&sd_fmt.format, &fmt, sizeof(struct v4l2_mbus_framefmt));
		sd_fmt.pad = pad->index;
		sd_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
		ret = v4l2_subdev_call(sd, pad, set_fmt, NULL, &sd_fmt);
		if (ret)
			goto out;
	}

	ret = isp_video_mbus_to_pix(&fmt, &format->fmt.pix_mp);
	if (ret)
		goto out;

	vnode->format = *format;
out:
	/* TODO: also change pipeline state here */
	return ret;
}

static int isp_vnode_try_format(struct file *file, void *fh,
				struct v4l2_format *format)
{
	struct isp_vnode *vnode = video_drvdata(file);
	struct isp_pipeline *pipeline = vnode->pipeline;
	struct v4l2_mbus_framefmt fmt;
	int ret;

	if (format->type != vnode->buf_type)
		return -EINVAL;

	mutex_lock(&vnode->pipeline->fmt_lock);
	ret = isp_video_pix_to_mbus(&format->fmt.pix_mp, &fmt);
	if (ret)
		goto out;
	ret = isp_pipeline_try_format(pipeline, &fmt);
	if (ret)
		goto out;
	ret = isp_video_mbus_to_pix(&fmt, &format->fmt.pix_mp);
out:
	mutex_unlock(&vnode->pipeline->fmt_lock);
	return ret;
}

static int isp_vnode_reqbufs(struct file *file, void *fh,
				struct v4l2_requestbuffers *rb)
{
	struct isp_vnode *vnode = video_drvdata(file);
	return vb2_reqbufs(&vnode->vq, rb);
}

static int isp_vnode_querybuf(struct file *file, void *fh,
				struct v4l2_buffer *vb)
{
	struct isp_vnode *vnode = video_drvdata(file);
	return vb2_querybuf(&vnode->vq, vb);
}

static int isp_vnode_qbuf(struct file *file, void *fh, struct v4l2_buffer *vb)
{
	struct isp_vnode *vnode = video_drvdata(file);
	if (vb->memory == V4L2_MEMORY_USERPTR && vb->length == 0)
		return -EINVAL;
	return vb2_qbuf(&vnode->vq, vb);
}

static int isp_vnode_dqbuf(struct file *file, void *fh,	struct v4l2_buffer *vb)
{
	struct isp_vnode *vnode = video_drvdata(file);
	return vb2_dqbuf(&vnode->vq, vb, file->f_flags & O_NONBLOCK);
}


static int isp_vnode_streamon(struct file *file, void *fh,
				enum v4l2_buf_type type)
{
	struct isp_vnode *vnode = video_drvdata(file);
	struct isp_pipeline *pipeline = vnode->pipeline;
	int ret = 0;

	if (type != vnode->buf_type)
		return -EINVAL;

	mutex_lock(&vnode->st_lock);
	if (vnode->state == ISP_VNODE_STREAMON) {
		ret = -EBUSY;
		goto err_exit;
	}

	/* TODO: Apply final format: maybe default or user defined */
	/* 1st: if not set format, get default format */
	/* 2nd: apply format to pipeline */

	INIT_LIST_HEAD(&vnode->busy_buf);
	vnode->busy_buf_cnt = 0;
	ret = vb2_streamon(&vnode->vq, vnode->buf_type);
	if (ret < 0)
		goto err_exit;

	if (vnode->pipeline) {
		ret = isp_pipeline_set_stream(pipeline, 1);
		if (ret < 0)
			goto err_exit;
	} else {
		struct media_pad *pad = media_entity_remote_source(&vnode->pad);
		struct v4l2_subdev *sd;

		if (pad == NULL || pad->entity == NULL) {
			ret = -EPIPE;
			goto err_vb_num;
		}
		sd = media_entity_to_v4l2_subdev(pad->entity);
		ret = v4l2_subdev_call(sd, video, s_stream, 1);
	}

	vnode->event_stream->owner = vnode;
	vnode->event_stream->msg = vnode;
	vnode->event_stream->type = 1;
	ret = isp_event_dispatch(vnode->event_stream);
	if (ret < 0)
		goto err_vb_num;

	vnode->state = ISP_VNODE_STREAMON;
	mutex_unlock(&vnode->st_lock);
	d_inf(2, "%s: stream on", vnode->vdev.name);
	return ret;

err_vb_num:
	vb2_streamoff(&vnode->vq, vnode->buf_type);
err_exit:
	mutex_unlock(&vnode->st_lock);
	d_inf(2, "%s: failed to stream on ", vnode->vdev.name);
	return ret;
}

static int isp_vnode_streamoff(struct file *file, void *fh,
				enum v4l2_buf_type type)
{
	struct isp_vnode *vnode = video_drvdata(file);
	unsigned long flags;
	int ret;

	if (type != vnode->buf_type)
		return -EINVAL;
	mutex_lock(&vnode->st_lock);

	if (vnode->state == ISP_VNODE_STREAMOFF) {
		mutex_unlock(&vnode->st_lock);
		return 0;
	}

	vnode->event_stream->owner = vnode;
	vnode->event_stream->msg = vnode;
	vnode->event_stream->type = 0;
	ret = isp_event_dispatch(vnode->event_stream);
	if (ret < 0) {
		mutex_unlock(&vnode->st_lock);
		return ret;
	}

	if (vnode->pipeline)
		isp_pipeline_set_stream(vnode->pipeline, 0);

	spin_lock_irqsave(&vnode->vb_lock, flags);
	vb2_streamoff(&vnode->vq, type);
	vnode->state = ISP_VNODE_STREAMOFF;
	INIT_LIST_HEAD(&vnode->idle_buf);
	INIT_LIST_HEAD(&vnode->busy_buf);
	vnode->busy_buf_cnt = 0;
	vnode->idle_buf_cnt = 0;
	spin_unlock_irqrestore(&vnode->vb_lock, flags);

	mutex_unlock(&vnode->st_lock);
	d_inf(2, "%s: stream off", vnode->vdev.name);
	return 0;
}

static int isp_vnode_g_input(struct file *file, void *fh, unsigned int *input)
{
	*input = 0;
	return 0;
}

static int isp_vnode_s_input(struct file *file, void *fh, unsigned int input)
{
	return input == 0 ? 0 : -EINVAL;
}

static const struct v4l2_ioctl_ops isp_vnode_ioctl_ops = {
	.vidioc_querycap		= isp_vnode_querycap,
	.vidioc_g_fmt_vid_cap_mplane	= isp_vnode_get_format,
	.vidioc_s_fmt_vid_cap_mplane	= isp_vnode_set_format,
	.vidioc_try_fmt_vid_cap_mplane	= isp_vnode_try_format,
	.vidioc_g_fmt_vid_out_mplane	= isp_vnode_get_format,
	.vidioc_s_fmt_vid_out_mplane	= isp_vnode_set_format,
	.vidioc_try_fmt_vid_out_mplane	= isp_vnode_try_format,
	.vidioc_reqbufs			= isp_vnode_reqbufs,
	.vidioc_querybuf		= isp_vnode_querybuf,
	.vidioc_qbuf			= isp_vnode_qbuf,
	.vidioc_dqbuf			= isp_vnode_dqbuf,
	.vidioc_g_input			= isp_vnode_g_input,
	.vidioc_s_input			= isp_vnode_s_input,
	.vidioc_streamon		= isp_vnode_streamon,
	.vidioc_streamoff		= isp_vnode_streamoff,
};

static int isp_vnode_close(struct file *file)
{
	struct isp_vnode *vnode = video_drvdata(file);
	struct isp_pipeline *pipeline = vnode->pipeline;

	if (pipeline == NULL)
		goto exit;

	/* In case of close, make sure everything is released, regardless of
	 * pipeline state, becase app may crash and don't have the chance to do
	 * a reasonable close */
	mutex_lock(&pipeline->st_lock);
	if (pipeline->state == ISP_PIPELINE_BUSY)
		isp_vnode_streamoff(file, NULL, vnode->buf_type);
	mutex_unlock(&pipeline->st_lock);

	isp_pipeline_clean(pipeline);
	pipeline->drv_own = 0;
	pipeline->output = NULL;
	vnode->pipeline = NULL;
	vnode->state = ISP_VNODE_STREAMOFF;
	d_inf(2, "%s: pipeline released", vnode->vdev.name);
	vb2_queue_release(&vnode->vq);
	vb2_dma_contig_cleanup_ctx(vnode->alloc_ctx);
exit:
	return 0;
}

static int isp_vnode_open(struct file *file)
{
	struct isp_vnode *vnode = video_drvdata(file);
	struct isp_pipeline *pipeline = vnode->pipeline;
	struct isp_build *build;
	int i, ret = 0;
	char buf[200];

	ret = isp_vnode_vb2_init(&vnode->vq, vnode);
	if (ret < 0)
		return ret;

	memset(&vnode->format, 0, sizeof(vnode->format));
	vnode->format.type = vnode->buf_type;

	/* For output vnode, the pipeline is never empty,
	 * but input vnode may have a empty pipeline */
	if (vnode->buf_type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return 0;
	/* find a pipeline descriptor for this video node */
	build = container_of(vnode->vdev.v4l2_dev, struct isp_build, v4l2_dev);
	list_for_each_entry(pipeline, &build->pipeline_pool, hook) {
		mutex_lock(&pipeline->st_lock);
		if (pipeline->state == ISP_PIPELINE_DEAD)
			goto found;
		mutex_unlock(&pipeline->st_lock);
	}
	return -EBUSY;
found:
	/* already hold the pipeline state lock now!!! attach to video node */
	BUG_ON(vnode->pipeline);
	vnode->pipeline = pipeline;
	pipeline->output = vnode;

	ret = isp_pipeline_validate(pipeline);
	if (ret >= 0)
		goto dump_link;
	d_inf(1, "%s: pipeline invalid, try to setup default pipeline",
		vnode->vdev.name);
	/* Attempt to establish the default pipeline */
	pipeline->drv_own = 1;
	pipeline->input = pipeline->def_src;
	ret = isp_pipeline_setup(pipeline);
	if (ret < 0)
		goto exit_unlock;
	vnode->state = ISP_VNODE_STREAMOFF;
dump_link:
	memset(buf, 0, sizeof(buf));
	for (i = pipeline->num_links-1; i >= 0; i--) {
		char tmp[32];
		sprintf(tmp, "[%s] => ",
			pipeline->link[i]->source->entity->name);
		strcat(buf, tmp);
	}
	d_inf(2, "%s: pipeline established as following:\n%s/dev/video%d",
		vnode->vdev.name, buf, vnode->vdev.num);

exit_unlock:
	mutex_unlock(&pipeline->st_lock);
	return ret;
}

static unsigned int isp_vnode_poll(struct file *file, poll_table *wait)
{
	struct isp_vnode *vnode = video_drvdata(file);
	struct vb2_queue *vbq = &vnode->vq;
	if ((vbq == NULL) || (vbq->num_buffers == 0 && vbq->fileio == NULL)
		|| list_empty(&vbq->queued_list))
		return POLLERR;
	return vb2_poll(vbq, file, wait);
}

static ssize_t isp_vnode_read(struct file *file, char __user *buf, size_t count,
			loff_t *ppos)
{
	struct isp_vnode *vnode = video_drvdata(file);
	d_inf(1, "%s: read() is empty", vnode->vdev.entity.name);
	count = 0;
	return 0;
}

static int isp_vnode_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct isp_vnode *vnode = video_drvdata(file);
	struct vb2_queue *vbq = &vnode->vq;
	return vb2_mmap(vbq, vma);
}

static const struct v4l2_file_operations isp_vnode_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= video_ioctl2,
	.open		= &isp_vnode_open,
	.release	= &isp_vnode_close,
	.poll		= &isp_vnode_poll,
	.read		= &isp_vnode_read,
	.mmap		= &isp_vnode_mmap,
};

/* initialize isp video node for a dma-capabile agent, dir: O-Output, 1-Input */
int isp_vnode_add(struct isp_vnode *vnode, struct v4l2_device *vdev,
			int dir, int vdev_nr)
{
	struct isp_build *build = container_of(vdev,
					struct isp_build, v4l2_dev);
	char name[V4L2_SUBDEV_NAME_SIZE * 2];
	int ret = 0;

	spin_lock_init(&vnode->vb_lock);
	mutex_init(&vnode->vdev_lock);
	mutex_init(&vnode->st_lock);
	INIT_LIST_HEAD(&vnode->idle_buf);
	INIT_LIST_HEAD(&vnode->busy_buf);
	vnode->idle_buf_cnt = vnode->busy_buf_cnt = 0;
	if (dir == 0) {
		vnode->buf_type		= V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		vnode->pad.flags	= MEDIA_PAD_FL_SINK;
		vnode->mode			= ISP_VNODE_MODE_STREAM;
	} else {
		vnode->buf_type		= V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
		vnode->pad.flags	= MEDIA_PAD_FL_SOURCE;
		vnode->vdev.vfl_dir	= VFL_DIR_TX;
		vnode->mode			= ISP_VNODE_MODE_SINGLE;
	}
	ret = media_entity_init(&vnode->vdev.entity, 1, &vnode->pad, 0);
	if (ret < 0)
		goto exit;
	vnode->vdev.entity.group_id = MEDIA_ENT_T_DEVNODE_V4L;
	vnode->vdev.vfl_type	= VFL_TYPE_GRABBER;
	vnode->vdev.release	= video_device_release_empty;
	vnode->vdev.ioctl_ops	= &isp_vnode_ioctl_ops;
	vnode->vdev.fops	= &isp_vnode_fops;
	vnode->vdev.lock	= &vnode->vdev_lock;
	vnode->vdev.v4l2_dev	= vdev;
	video_set_drvdata(&vnode->vdev, vnode);
	ret = video_register_device(&vnode->vdev, VFL_TYPE_GRABBER, vdev_nr);

	sprintf(name, "%s:qbuf", vnode->vdev.name);
	ret = isp_event_register(build, name);
	if (ret < 0)
		goto exit;
	vnode->event_qbuf = isp_event_find(build, name);
	sprintf(name, "%s:stream", vnode->vdev.name);
	ret = isp_event_register(build, name);
	if (ret < 0)
		goto exit;
	vnode->event_stream = isp_event_find(build, name);
exit:
	return ret;
}
EXPORT_SYMBOL(isp_vnode_add);

void isp_vnode_remove(struct isp_vnode *vnode)
{
	struct isp_build *build = container_of(vnode->vdev.v4l2_dev,
					struct isp_build, v4l2_dev);
	if (video_is_registered(&vnode->vdev)) {
		char name[V4L2_SUBDEV_NAME_SIZE * 2];
		struct isp_event;
		media_entity_cleanup(&vnode->vdev.entity);
		video_unregister_device(&vnode->vdev);
		sprintf(name, "%s:qbuf", vnode->vdev.name);
		isp_event_unregister(build, name);
		sprintf(name, "%s:stream", vnode->vdev.name);
		isp_event_unregister(build, name);
		devm_kfree(&vnode->vdev.dev, vnode->event_qbuf);
	}
}
EXPORT_SYMBOL(isp_vnode_remove);

int isp_vnode_attach(struct isp_build *build)
{
	struct isp_vnode *vnode;
	struct isp_subdev *ispsd;
	int ret, dir;

	list_for_each_entry(ispsd, &build->ispsd_list, hook) {
		switch (ispsd->sd_type) {
		case ISP_BLOCK_DMA_OUT:
			/* Create pipeline here */
			dir = 0;
			break;
		case ISP_BLOCK_DMA_IN:
			dir = 1;
			break;
		default:
			continue;
		}

		vnode = devm_kzalloc(build->dev,
					sizeof(struct isp_vnode), GFP_KERNEL);
		if (vnode == NULL)
			return -ENOMEM;

		snprintf(vnode->vdev.name, sizeof(vnode->vdev.name),
				"%s %s", ispsd->subdev.name, "video");
		ret = isp_vnode_add(vnode, &build->v4l2_dev, dir, -1);
		if (ret < 0)
			return ret;
	}
	return 0;
}

static inline void isp_pipeline_destroy(struct isp_pipeline *pipe)
{
	if (pipe->output)
		pipe->output->pipeline = NULL;
	list_del(&pipe->hook);
	devm_kfree(pipe->mngr->dev, pipe);
}

struct isp_pipeline *isp_pipeline_create(struct isp_build *build)
{
	struct isp_pipeline *pipe;

	pipe = devm_kzalloc(build->dev, sizeof(struct isp_pipeline),
				GFP_KERNEL);
	if (pipe == NULL)
		return NULL;

	INIT_LIST_HEAD(&pipe->hook);
	list_add_tail(&pipe->hook, &build->pipeline_pool);
	pipe->id = build->pipeline_cnt++;
	mutex_init(&pipe->st_lock);
	mutex_init(&pipe->fmt_lock);
	pipe->mngr = build;
	return pipe;
}

static inline int pipeline_get(struct isp_pipeline *pipeline, int *err_link)
{
	int i, ret = 0;

	*err_link = 0;
	for (i = 0; i < pipeline->num_links; i++) {
		struct media_entity *src = pipeline->link[i]->source->entity;
		struct media_entity *dst = pipeline->link[i]->sink->entity;
		struct isp_subdev *src_ispsd = me_to_ispsd(src);
		struct isp_subdev *dst_ispsd = me_to_ispsd(dst);
		int alive = 0;
		int src_open = 0;
		int dst_open = 0;

		mutex_lock(&pipeline->mngr->graph_mutex);
		alive = src->use_count & dst->use_count;
		/* use media_entity::use_count as reference isp */
		/* open ispsd when use_count = 0 */
		if (!src->use_count && src_ispsd && src_ispsd->ops->open)
			src_open = 1;
		if (!dst->use_count && dst_ispsd && dst_ispsd->ops->open)
			dst_open = 1;

		src->use_count |= (1 << pipeline->id);
		dst->use_count |= (1 << pipeline->id);
		if (pipeline->drv_own && !alive)
			ret = media_entity_setup_link(pipeline->link[i],
						MEDIA_LNK_FL_ENABLED);
		if (WARN_ON(ret < 0)) {
			src->use_count &= ~(1 << pipeline->id);
			dst->use_count &= ~(1 << pipeline->id);
			d_inf(1, "isp: link %s => %s enable failed, ret %d",
				pipeline->link[i]->source->entity->name,
				pipeline->link[i]->sink->entity->name, ret);
			mutex_unlock(&pipeline->mngr->graph_mutex);
			return -EPIPE;
		}

		if (src_open) {
			ret = (*src_ispsd->ops->open)(src_ispsd);
			if (ret < 0)
				return ret;
			src_open = 0;
		}
		if (dst_open) {
			ret = (*dst_ispsd->ops->open)(dst_ispsd);
			if (ret < 0)
				return ret;
			dst_open = 0;
		}
		mutex_unlock(&pipeline->mngr->graph_mutex);
		d_inf(4, "link %s => %s enabled",
			pipeline->link[i]->source->entity->name,
			pipeline->link[i]->sink->entity->name);
	}
	pipeline->state = ISP_PIPELINE_IDLE;
	return 0;
}

static inline void pipeline_put(struct isp_pipeline *pipeline, int start_link)
{
	int i, ret = 0;

	WARN_ON((pipeline->state != ISP_PIPELINE_DEAD)
		&& (pipeline->state != ISP_PIPELINE_IDLE));

	for (i = start_link; i >= 0; i--) {
		struct media_entity *src = pipeline->link[i]->source->entity;
		struct media_entity *dst = pipeline->link[i]->sink->entity;
		struct isp_subdev *src_ispsd = me_to_ispsd(src);
		struct isp_subdev *dst_ispsd = me_to_ispsd(dst);
		int alive = 0;

		mutex_lock(&pipeline->mngr->graph_mutex);
		/* close ispsd when use_count is current pipeline mask,
		 * when one ispsd in middle of pipeline, close it one time when
		 * one of it's right and left link is been destroyed.*/
		/* FIXME: here have a potential problem:
		 * we destroy one ispsd's right link and close ispsd,
		 * it caused we cann't do anythings use it's left link. */
		if ((src->use_count == (1 << pipeline->id)) && src_ispsd
				&& src_ispsd->ops->close)
			(*src_ispsd->ops->close)(src_ispsd);
		if ((dst->use_count == (1 << pipeline->id)) && dst_ispsd
				&& dst_ispsd->ops->close)
			(*dst_ispsd->ops->close)(dst_ispsd);
		/* clear reference isp for this pipeline */
		src->use_count &= ~(1 << pipeline->id);
		dst->use_count &= ~(1 << pipeline->id);
		alive = src->use_count & dst->use_count;
		if (pipeline->drv_own && !alive)
			ret = media_entity_setup_link(pipeline->link[i],
						!MEDIA_LNK_FL_ENABLED);
		if (WARN_ON(ret < 0))
			d_inf(1, "isp: link %s => %s disable failed, ret %d",
				pipeline->link[i]->source->entity->name,
				pipeline->link[i]->sink->entity->name, ret);
		mutex_unlock(&pipeline->mngr->graph_mutex);
		d_inf(4, "link %s => %s disabled",
			pipeline->link[i]->source->entity->name,
			pipeline->link[i]->sink->entity->name);
	}
}

void isp_pipeline_clean(struct isp_pipeline *pipe)
{
	pipeline_put(pipe, pipe->num_links - 1);
	memset(pipe->link, 0, sizeof(pipe->link));
	pipe->num_links = 0;
	pipe->input = NULL;
	pipe->state = ISP_PIPELINE_DEAD;
}

int isp_pipeline_setup(struct isp_pipeline *pipe)
{
	int err, ret = 0;

	BUG_ON(pipe->find_link == NULL);
	memset(pipe->link, 0, sizeof(pipe->link));
	/* the passed pads will be saved as link */
	pipe->num_links = (*pipe->find_link)(
				pipe->input, &pipe->output->vdev.entity,
				pipe->link, MC_PIPELINE_LEN_MAX, 0, 0);
	if (pipe->num_links <= 0) {
		d_inf(1, "isp_cam: can't find a link from %s to %s",
			(pipe->input) ? pipe->input->name : "<NULL>",
			pipe->output->vdev.entity.name);
		return -EPIPE;
	}
	/* Check & get refcnt for every subdev on the pipeline, link[0].sink is
	 * video_dev, no refcnt */
	ret = pipeline_get(pipe, &err);
	if (ret < 0 || err)
		goto err;
	return ret;

err:
	pipeline_put(pipe, err);
	return ret;
}

/* check the integraty of pipeline */
/* only called after pipeline is modified by user space APP */
int isp_pipeline_validate(struct isp_pipeline *pipe)
{
	BUG_ON(pipe->find_link == NULL);
	memset(pipe->link, 0, sizeof(pipe->link));
	/* When validate the pipeline, only the enabled link is passable*/
	pipe->num_links = (*pipe->find_link)(NULL,
				&pipe->output->vdev.entity,
				pipe->link, MC_PIPELINE_LEN_MAX,
				MEDIA_LNK_FL_ENABLED, 0);
	if (pipe->num_links <= 0)
		return -EPIPE;

	pipe->input = pipe->link[pipe->num_links-1]->source->entity;
	pipe->state = ISP_PIPELINE_IDLE;
	return 0;
}
EXPORT_SYMBOL(isp_pipeline_validate);

/**************************** Implement V4L2 APIs *****************************/

int isp_pipeline_querycap(struct isp_pipeline *pipeline,
			struct v4l2_capability *cap)
{
	struct isp_build *mngr;
	int ret = 0;

	mutex_lock(&pipeline->st_lock);
	if (pipeline->state <= ISP_PIPELINE_DEAD) {
		ret = -EPERM;
		d_inf(1, "pipeline not established yet");
		goto exit_unlock;
	}

	mngr = pipeline->mngr;
	strlcpy(cap->driver, mngr->name, sizeof(cap->driver));
	/* Get pipeline head entity name */
	strlcpy(cap->card,
		pipeline->link[pipeline->num_links-1]->source->entity->name,
		sizeof(cap->card));
	/* TODO: get platform specific information in cap->reserved */

exit_unlock:
	mutex_unlock(&pipeline->st_lock);
	return ret;
}

int isp_pipeline_set_stream(struct isp_pipeline *pipeline, int enable)
{
	int i = pipeline->num_links - 1, ret = 0, err = 0;

	if (pipeline->num_links <= 0)
		return -EPERM;

	if (enable) {
		if (WARN_ON(pipeline->state != ISP_PIPELINE_IDLE)) {
			ret = -EPIPE;
			goto exit;
		}
		pipeline->link[0]->sink->entity->stream_count++;
		/* invoke s_stream on each subdev,
		 * down the direction of dataflow */
		for (i = 0; i < pipeline->num_links; i++) {
			struct media_entity *me =
				pipeline->link[i]->source->entity;
			struct v4l2_subdev *sd =
				media_entity_to_v4l2_subdev(me);

			if (me->stream_count >= 1)
				goto inc_cnt;
			if (media_entity_type(me) != MEDIA_ENT_T_V4L2_SUBDEV)
				goto inc_cnt;
			/* TODO: If it's an subdev, apply final format here! */
			ret = v4l2_subdev_call(sd, video, s_stream, 1);
			if (ret < 0) {
				d_inf(1, "stream on failed at %s: %d",
					me->name, ret);
				goto stream_off;
			}
			d_inf(4, "stream on %s success", me->name);
inc_cnt:
			me->stream_count++;
		}
		pipeline->state = ISP_PIPELINE_BUSY;
		goto exit;
	}

stream_off:
	/* stream_on failed or duplicated stream_off*/
	if (WARN_ON(pipeline->state != ISP_PIPELINE_BUSY))
		goto exit;
	for (; i >= 0; i--) {
		struct media_entity *me =
				pipeline->link[i]->source->entity;
		struct v4l2_subdev *sd =
				media_entity_to_v4l2_subdev(me);

		if (me->stream_count > 1)
			goto dec_cnt;
		if (media_entity_type(me) != MEDIA_ENT_T_V4L2_SUBDEV)
			goto dec_cnt;
		ret = v4l2_subdev_call(sd, video, s_stream, 0);
		if (ret < 0) {
			err = ret;
			d_inf(1, "stream off failed at %s: %d", me->name, ret);
		}
		d_inf(4, "stream off %s success", me->name);
dec_cnt:
		me->stream_count--;
	}
	pipeline->link[0]->sink->entity->stream_count--;
	pipeline->state = ISP_PIPELINE_IDLE;
exit:
	return ret | err;
};
EXPORT_SYMBOL(isp_pipeline_set_stream);

static inline int _isp_pipe_s_fmt(struct isp_pipeline *pipeline,
				struct v4l2_mbus_framefmt *fmt, __u32 which)
{
	struct v4l2_subdev_format sd_fmt;
	int ret, i;

	memcpy(&sd_fmt.format, fmt, sizeof(struct v4l2_mbus_framefmt));
	sd_fmt.which = which;
	for (i = 0; i < pipeline->num_links; i++) {
		/* The format of this link should be the get from link sink,
		 * beware that it's possible for sink don't support g_fmt */
		sd_fmt.pad = pipeline->link[i]->sink->index;
		ret = isp_entity_get_fmt(pipeline->link[i]->sink->entity,
					&sd_fmt);
		if (ret < 0)
			return ret;

		ret = isp_link_try_set_fmt(pipeline->link[i], &sd_fmt.format,
					which);
		if (ret < 0)
			return ret;
	}
	return 0;
}

/* apply set_format on the whole pipeline */
int isp_pipeline_set_format(struct isp_pipeline *pipeline,
				struct v4l2_mbus_framefmt *fmt)
{
	return _isp_pipe_s_fmt(pipeline, fmt, V4L2_SUBDEV_FORMAT_ACTIVE);
}
EXPORT_SYMBOL(isp_pipeline_set_format);

/* apply set_format on the whole pipeline */
int isp_pipeline_try_format(struct isp_pipeline *pipeline,
				struct v4l2_mbus_framefmt *fmt)
{
	return _isp_pipe_s_fmt(pipeline, fmt, V4L2_SUBDEV_FORMAT_TRY);
}
EXPORT_SYMBOL(isp_pipeline_try_format);

int isp_pipeline_get_format(struct isp_pipeline *pipeline,
				struct v4l2_mbus_framefmt *fmt)
{
	struct v4l2_subdev_format sd_fmt;
	struct media_entity *me;
	struct v4l2_subdev *sd;
	int i;

	/* pipeline format is determined by the last subdev in the pipeline,
	 * that is capable to do format convertion */
	me = pipeline->link[0]->sink->entity;
	sd_fmt.pad = pipeline->link[0]->sink->index;
	for (i = -1; i < pipeline->num_links;) {
		if (media_entity_type(me) == MEDIA_ENT_T_V4L2_SUBDEV) {
			sd = media_entity_to_v4l2_subdev(me);
			if (subdev_has_fn(sd, pad, get_fmt))
				goto get_fmt;
		}
		i++;
		me = pipeline->link[i]->source->entity;
		sd_fmt.pad = pipeline->link[i]->source->index;
	}
	return -EPIPE;

get_fmt:
	return isp_entity_get_fmt(me, &sd_fmt);
}
EXPORT_SYMBOL(isp_pipeline_get_format);
