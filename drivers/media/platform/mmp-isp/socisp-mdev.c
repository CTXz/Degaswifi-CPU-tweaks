#include <linux/module.h>
#include <linux/export.h>
#include <linux/slab.h>

#include <media/socisp-mdev.h>

static int trace = 2;
module_param(trace, int, 0644);
MODULE_PARM_DESC(trace,
		"how many trace do you want to see? (0-4):"
		"0 - mute "
		"1 - only actual errors"
		"2 - milestone log"
		"3 - briefing log"
		"4 - detailed log");

struct isp_subdev *isp_subdev_find(struct isp_build *build, int sd_code)
{
	struct isp_subdev *ispsd;

	list_for_each_entry(ispsd, &build->ispsd_list, hook) {
		if (ispsd->sd_code == sd_code)
			return ispsd;
	}
	return NULL;
};

void isp_subdev_unregister(struct isp_subdev *ispsd)
{
	ispsd->build = NULL;
	media_entity_cleanup(&ispsd->subdev.entity);
}

int isp_subdev_register(struct isp_subdev *ispsd, struct isp_build *build)
{
	struct v4l2_subdev *sd = &ispsd->subdev;
	struct media_entity *me = &sd->entity;
	int ret;

	if (ispsd->single) {
		void *guest = isp_sd2blk(ispsd);
		if (guest) {
			struct isp_block *blk = guest;
			if (strlen(sd->name) == 0)
				strlcpy(sd->name, blk->name, sizeof(sd->name));
			ispsd->sd_type = blk->id.mod_type;
			isp_block_register(blk, &build->resrc_pool);
		}
	} else {
		/* Combined isp-subdev */
		if (strlen(sd->name) == 0)
			return -EINVAL;
	}

	v4l2_set_subdev_hostdata(sd, ispsd);
	sd->ctrl_handler = &ispsd->ctrl_handler;
	sd->grp_id = GID_ISP_SUBDEV;

	/* media entity init */
	ret = media_entity_init(me, ispsd->pads_cnt, ispsd->pads, 0);
	if (ret < 0)
		return ret;
	/* This media entity is contained by a subdev */
	me->type = MEDIA_ENT_T_V4L2_SUBDEV;
	/* ispsd member init */
	ispsd->build = build;
	INIT_LIST_HEAD(&ispsd->hook);
	list_add_tail(&ispsd->hook, &build->ispsd_list);
	d_log("module '%s' registered to '%s'", me->name, build->name);
	return ret;
}

int isp_subdev_add_guest(struct isp_subdev *ispsd,
				void *guest, enum isp_gdev_type type)
{
	struct isp_dev_ptr *desc = devm_kzalloc(ispsd->build->dev,
						sizeof(*desc), GFP_KERNEL);
	if (desc == NULL)
		return -ENOMEM;
	desc->ptr = guest;
	desc->type = type;
	INIT_LIST_HEAD(&desc->hook);
	list_add_tail(&desc->hook, &ispsd->gdev_list);
	return 0;
}

int isp_entity_get_fmt(struct media_entity *me, struct v4l2_subdev_format *fmt)
{
	struct v4l2_subdev *sd;

	if (unlikely(fmt->pad >= me->num_pads))
		return -EINVAL;
	if (media_entity_type(me) != MEDIA_ENT_T_V4L2_SUBDEV)
		return 0;
	sd = media_entity_to_v4l2_subdev(me);
	/* If this subdev don't care about format */
	if (!subdev_has_fn(sd, pad, get_fmt))
		return 0;
	d_inf(4, "%s: get fmt on pad(%d)", me->name, fmt->pad);
	return v4l2_subdev_call(sd, pad, get_fmt, NULL, fmt);
}

static inline int _mbus_fmt_same(struct v4l2_mbus_framefmt *fmt1,
				struct v4l2_mbus_framefmt *fmt2)
{
	return (fmt1->code == fmt2->code)
		&& (fmt1->width == fmt2->width)
		&& (fmt1->height == fmt2->height);
}

/* A pad can belong to a devnode / subdev */
static inline int isp_entity_try_set_fmt(struct media_entity *me,
				struct v4l2_subdev_format *fmt)
{
	struct isp_subdev *ispsd = NULL;
	struct v4l2_subdev *sd;
	struct v4l2_mbus_framefmt *fmt_now = NULL;
	int ret = 0;

	if (media_entity_type(me) != MEDIA_ENT_T_V4L2_SUBDEV)
		return 0;
	sd = media_entity_to_v4l2_subdev(me);
	/* This media entity is a subdev, but don't care about format */
	if (!subdev_has_fn(sd, pad, set_fmt))
		return 0;

	d_inf(3, "set format on %s", sd->name);
	/* Maybe new format is the same with current format? */
	if (sd->grp_id == GID_ISP_SUBDEV) {
		ispsd = v4l2_get_subdev_hostdata(sd);
		fmt_now = ispsd->fmt_pad + fmt->pad;
		/* if current format is the same as new format, skip */
		if (_mbus_fmt_same(fmt_now, &fmt->format)) {
			d_inf(4, "new format same as current format, skip");
			return 0;
		}
	}

	/* Bad Luck, now really apply the format on entity */
	ret = v4l2_subdev_call(sd, pad, set_fmt, NULL, fmt);
	if (ret < 0)
		return ret;
	if (ispsd)
		memcpy(fmt_now, &fmt->format, sizeof(*fmt_now));

	return ret;
}

int isp_link_try_set_fmt(struct media_link *link,
				struct v4l2_mbus_framefmt *fmt, __u32 which)
{
	struct v4l2_subdev_format pad_fmt;
	int ret;

	d_inf(4, "link %s<%d> => %s<%d>: set fmt %X(%d, %d)",
		link->source->entity->name, link->source->index,
		link->sink->entity->name, link->sink->index,
		fmt->code, fmt->width, fmt->height);
	memcpy(&pad_fmt.format, fmt, sizeof(struct v4l2_mbus_framefmt));
	pad_fmt.which = which;
	pad_fmt.pad = link->source->index;

	ret = isp_entity_try_set_fmt(link->source->entity, &pad_fmt);
	if (ret < 0) {
		d_inf(1, "set format failed on source %s<%d>: ret %d",
			link->source->entity->name, link->source->index, ret);
		return ret;
	}
	pad_fmt.which = which;
	pad_fmt.pad = link->sink->index;

	ret = isp_entity_try_set_fmt(link->sink->entity, &pad_fmt);
	if (ret < 0) {
		d_inf(1, "set format failed on sink %s<%d>: ret %d",
			link->sink->entity->name, link->sink->index, ret);
		return ret;
	}
	memcpy(fmt, &pad_fmt.format, sizeof(*fmt));
	return ret;
}

int isp_link_notify(struct media_pad *src, struct media_pad *dst, u32 flags)
{
	struct isp_subdev *src_ispsd = NULL, *dst_ispsd = NULL;
	struct v4l2_subdev *src_sd = NULL, *dst_sd = NULL;
	void *guest;
	int ret = 0;

	if (unlikely(src == NULL || dst == NULL))
		return -EINVAL;
	/* Source and sink of the link may not be an ispsd, carefully convert
	 * to isp_subdev if possible */
	if (media_entity_type(src->entity) == MEDIA_ENT_T_V4L2_SUBDEV) {
		src_sd = media_entity_to_v4l2_subdev(src->entity);
		if (src_sd->grp_id == GID_ISP_SUBDEV)
			src_ispsd = v4l2_get_subdev_hostdata(src_sd);
	}
	if (media_entity_type(dst->entity) == MEDIA_ENT_T_V4L2_SUBDEV) {
		dst_sd = media_entity_to_v4l2_subdev(dst->entity);
		if (dst_sd->grp_id == GID_ISP_SUBDEV)
			dst_ispsd = v4l2_get_subdev_hostdata(dst_sd);
	}
	if ((flags & MEDIA_LNK_FL_ENABLED) == 0)
		goto dst_sd_off;

	/* power on sequence */
	if (src_ispsd && src_ispsd->single) {
		guest = isp_sd2blk(src_ispsd);
		if (guest) {
			ret = isp_block_tune_power(guest, 1);
			if (ret < 0)
				goto exit;
		}
	}
	if (src_sd) {
		/*
		 * For single subdev, simply call v4l2_subdev's s_power.
		 * For combined subdev, s_power will be implemented by shell
		 * subdev. i.e. for host_subdev, s_power setup links between
		 * guest subdev.
		 */
		ret = v4l2_subdev_call(src_sd, core, s_power, 1);
		if (ret == -ENOIOCTLCMD)
			ret = 0;
		if (ret < 0)
			goto src_hw_off;
	}

	if (dst_ispsd && dst_ispsd->single) {
		guest = isp_sd2blk(dst_ispsd);
		if (guest) {
			ret = isp_block_tune_power(guest, 1);
			if (ret < 0)
				goto src_sd_off;
		}
	}
	if (dst_sd) {
		ret = v4l2_subdev_call(dst_sd, core, s_power, 1);
		if (ret == -ENOIOCTLCMD)
			ret = 0;
		if (ret < 0)
			goto dst_hw_off;
	}
	return 0;

	/* power off sequence */
dst_sd_off:
	if (dst_sd)
		v4l2_subdev_call(dst_sd, core, s_power, 0);
dst_hw_off:
	if (dst_ispsd && dst_ispsd->single) {
		guest = isp_sd2blk(dst_ispsd);
		if (guest)
			isp_block_tune_power(guest, 0);
	}
src_sd_off:
	if (src_sd)
		v4l2_subdev_call(src_sd, core, s_power, 0);
src_hw_off:
	if (src_ispsd && src_ispsd->single) {
		guest = isp_sd2blk(src_ispsd);
		if (guest)
			isp_block_tune_power(guest, 0);
	}
exit:
	return ret;
}

void isp_build_exit(struct isp_build *build)
{
	/* FIXME: add clean up code here */
}

int isp_build_init(struct isp_build *mngr)
{
	int ret = 0;

	/* suppose by this point, all H/W subdevs had been registed to
	 * H/W manager */
	if (unlikely(mngr == NULL))
		return -EINVAL;

	/* distribute H/W resource */

	INIT_LIST_HEAD(&mngr->pipeline_pool);
	mngr->pipeline_cnt = 0;
	mutex_init(&mngr->graph_mutex);

	/* register media entity */
	BUG_ON(mngr->name == NULL);
	mngr->media_dev.dev = mngr->dev;
	strlcpy(mngr->media_dev.model, mngr->name,
		sizeof(mngr->media_dev.model));
	mngr->media_dev.link_notify = isp_link_notify;
	ret = media_device_register(&mngr->media_dev);
	/* v4l2 device register */
	mngr->v4l2_dev.mdev = &mngr->media_dev;
	ret = v4l2_device_register(mngr->dev, &mngr->v4l2_dev);
	return ret;
}

int isp_build_attach_ispsd(struct isp_build *build)
{
	int ret;
	struct isp_subdev *ispsd;

	list_for_each_entry(ispsd, &build->ispsd_list, hook) {
		if (build->add_subdev) {
			/*
			 * Platform based ISP driver can use this call back to
			 * define video_dev/pipeline creating strategy for
			 * its own.
			 */
			ret = (*build->add_subdev)(build, ispsd);
			if (ret < 0)
				return ret;
		}
		ret = v4l2_device_register_subdev(&build->v4l2_dev,
							&ispsd->subdev);
		if (ret < 0)
			return ret;
		d_log("subdev '%s' add to '%s'",
			ispsd->subdev.name, build->name);
	}

	/* Finally, create all the file nodes for each subdev */
	return v4l2_device_register_subdev_nodes(&build->v4l2_dev);
}

/************************* isp-event related function *************************/
struct isp_event *isp_event_find(struct isp_build *build, const char *name)
{
	struct isp_event *event;
	list_for_each_entry(event, &build->event_pool, hook) {
		if (!strcmp(event->name, name))
			return event;
	}
	return NULL;
}

int isp_event_register(struct isp_build *build, const char *name)
{
	struct isp_event *event;

	if (unlikely(!build || !name || !strlen(name)))
		return -EINVAL;
	if (isp_event_find(build, name))
		return -EPERM;

	event = devm_kzalloc(build->dev, sizeof(struct isp_event), GFP_KERNEL);
	if (event == NULL)
		return -ENOMEM;
	strlcpy(event->name, name, sizeof(event->name));
	INIT_LIST_HEAD(&event->hook);
	INIT_LIST_HEAD(&event->dispatch_list);
	list_add_tail(&event->hook, &build->event_pool);
	return 0;
}

int isp_event_unregister(struct isp_build *build, const char *name)
{
	struct isp_event *event;

	if (unlikely(!build || !name || !strlen(name)))
		return -EINVAL;
	event = isp_event_find(build, name);
	if (event == NULL)
		return -ENOENT;

	list_del_init(&event->hook);
	while (!list_empty(&event->dispatch_list)) {
		struct isp_dispatch_entry *item =
			list_first_entry(&event->dispatch_list,
			struct isp_dispatch_entry, hook);
		d_inf(1, "isp-subdev %s still listening to %s",
			item->ispsd->subdev.name, event->name);
		list_del(&item->hook);
		devm_kfree(item->ispsd->build->dev, item);
	}
	devm_kfree(build->dev, event);
	return 0;
}

int isp_event_subscribe(struct isp_event *event, struct isp_subdev *rx_sd,
			event_handler_t handler)
{
	struct isp_dispatch_entry *item = NULL;

	item = devm_kzalloc(rx_sd->build->dev,
				sizeof(struct isp_dispatch_entry), GFP_KERNEL);
	if (unlikely(item == NULL))
		return -ENOMEM;
	INIT_LIST_HEAD(&item->hook);
	item->ispsd = rx_sd;
	item->handler = handler;
	list_add_tail(&item->hook, &event->dispatch_list);
	return 0;
}

int isp_event_unsubscribe(struct isp_event *event, struct isp_subdev *rx_sd)
{
	struct isp_dispatch_entry *item;

	list_for_each_entry(item, &event->dispatch_list, hook) {
		if (item->ispsd == rx_sd)
			goto found;
	}
	return -ENOENT;
found:
	list_del(&item->hook);
	devm_kfree(rx_sd->build->dev, item);
	return 0;
}

int isp_event_dispatch(struct isp_event *event)
{
	struct isp_dispatch_entry *item;
	int ret;

	list_for_each_entry(item, &event->dispatch_list, hook) {
		ret = (*item->handler)(item->ispsd, event);
		if (ret < 0) {
			d_inf(1, "error dispatch event '%s' to %s: %d",
				event->name, item->ispsd->subdev.name, ret);
			return ret;
		}
	}
	return 0;
}
