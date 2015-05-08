/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.

 *(C) Copyright 2007 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/device.h>
#include <linux/uaccess.h>

#include <linux/tty_ldisc.h>
#include <linux/sched.h>
#include <linux/delay.h>

#include "common_datastub.h"
#include "data_channel_kernel.h"

#define DRIVER_VERSION "v1.0"
#define DRIVER_AUTHOR "Johnson Wu"
#define DRIVER_DESC "CI Data TTY driver"

/* Module information */
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

/* Our fake UART values */
#define MCR_DTR         0x01
#define MCR_RTS         0x02
#define MCR_LOOP        0x04
#define MSR_CTS         0x08
#define MSR_CD          0x10
#define MSR_RI          0x20
#define MSR_DSR         0x40
#define RELEVANT_IFLAG(iflag) ((iflag) & (IGNBRK | BRKINT | \
			IGNPAR | PARMRK | INPCK))

#define CIDATATTY_TTY_MAJOR             0	/* experimental range */
#define CIDATATTY_TTY_MINORS            3
#define CCTDATADEV_NR_DEVS 3
#define PSDTTYINDEX		1
#define CSDTTYINDEX			0
#define IMSTTYINDEX			2
int cctdatadev_major;
int cctdatadev_minor;
int cctdatadev_nr_devs = CCTDATADEV_NR_DEVS;

#define TIOENABLE       _IOW('T', 206, int)	/* enable data */
#define TIODISABLE      _IOW('T', 207, int)	/* disable data */
#define TIOPPPON _IOW('T', 208, int)
#define TIOPPPOFF _IOW('T', 209, int)
#define TIOPPPONCSD _IOW('T', 211, int)
/*
 *  The following Macro is used for debugging purpose
 */

/* #define CIDATATTY_DEBUG */
/* #define DEBUG_BUF_CONTENT */ /* define this to see the buffer content */

#undef PDEBUG			/* undef it, just in case */
#ifdef CIDATATTY_DEBUG
#define PDEBUG(fmt, args ...) printk(KERN_DEBUG "CIDATATTY: " fmt, ## args)
#define F_ENTER() printk(KERN_DEBUG "CIDATATTY: ENTER %s\n", __func__)
#define F_LEAVE() printk(KERN_DEBUG "CIDATATTY: LEAVE %s\n", __func__)
#else
#define PDEBUG(fmt, args ...) do {} while (0)
#define F_ENTER()       do {} while (0)
#define F_LEAVE()       do {} while (0)
#endif

struct cidatatty_port {
	struct tty_port port;	/* tty_port struct */
	spinlock_t port_lock;	/* protect port_* access */
	struct semaphore sem_lock_tty;	/* used for hold tty in use. */
	bool	openclose;	/* open/close in progress */

	/* for tiocmget and tiocmset functions */
	int msr;		/* MSR shadow */
	int mcr;		/* MCR shadow */

	/* for ioctl fun */
	struct serial_struct serial;
	wait_queue_head_t wait;
	struct async_icount icount;
	unsigned char cid;
	int writeokflag;
	int devinuse;
};

static struct port_master {
	struct mutex lock;	/* protect open/close */
	struct cidatatty_port *data_port;
} cidatatty_table[CIDATATTY_TTY_MINORS];

struct cctdatadev_dev {
	int nreaders, nwriters;
	struct cdev cdev;	/* Char device structure */
};

static struct tty_driver *cidatatty_tty_driver;
struct cctdatadev_dev *cctdatadev_devices;
/* int ttycurindex; */
unsigned char currcid_cs = 0xff;
unsigned char currcid_ps = 0xff;
unsigned char currcid_ims = 0xff;

/*
 * Prototypes for shared functions
 */
int data_rx_ppp(char *packet, int len, unsigned char cid)
{
	struct tty_struct *tty = NULL;
	struct cidatatty_port *cidatatty;
	int remain = len;
	int ret;

	int c = -1;

	F_ENTER();

	if (!cidatatty_table[PSDTTYINDEX].data_port
	    || cidatatty_table[PSDTTYINDEX].data_port->cid != cid
	    || cidatatty_table[PSDTTYINDEX].data_port->devinuse != 1) {
		printk(KERN_ERR "data_rx_ppp: not found tty device.\n ");
		return 0;
	} else {
		tty = cidatatty_table[PSDTTYINDEX].data_port->port.tty;
	}

	if (!tty) {
		printk(KERN_ERR "data_rx_ppp: not found PSD tty.\n ");
		return 0;
	}

	cidatatty = tty->driver_data;
	/* drop packets if nobody uses cidatatty */
	if (cidatatty == NULL) {
		printk(KERN_ERR
		       "data_rx_ppp: drop packets if nobody uses cidatatty.\n ");
		return 0;
	}

	down(&cidatatty->sem_lock_tty);

	if (cidatatty->port.count == 0) {
		up(&cidatatty->sem_lock_tty);
		printk(KERN_ERR "data_rx_ppp: cidatatty device is closed\n");
		return 0;
	}

	while (remain > 0) {
		/* PDEBUG( "data_rx_ppp: receive_room=0x%x\n ",
		 * tty->ldisc.receive_room ); */
		c = tty->receive_room;

		/* give up if no room at all */
		if (c == 0)
			break;

		if (c > len)
			c = len;
		/* PDEBUG( "data_rx_ppp: receive_buf=0x%x\n ",
		 * tty->ldisc.receive_buf ) */
		tty->ldisc->ops->receive_buf(tty, packet, NULL, c);
		/* for N_TTY type */
		/* if (tty->ldisc.ops->flush_buffer) */
		/*      tty->ldisc.ops->flush_buffer(tty); */
		wake_up_interruptible(&tty->read_wait);

		remain -= c;
		packet += c;
	}

	/* return the actual bytes got */
	ret = len - remain;

	up(&cidatatty->sem_lock_tty);

	if (remain > 0 && c == 0)
		printk(KERN_ERR
		       "data_rx_ppp: give up because receive_room left\n");

	return ret;

}

int data_rx_csd(char *packet, int len, unsigned char cid)
{
	struct tty_struct *tty = NULL;
	struct cidatatty_port *cidatatty;
	int ret;

	F_ENTER();

	if (!cidatatty_table[CSDTTYINDEX].data_port
	    || cidatatty_table[CSDTTYINDEX].data_port->cid != cid
	    || cidatatty_table[CSDTTYINDEX].data_port->devinuse != 1) {
		printk(KERN_ERR "data_rx_csd: not found tty device. cid = %d\n",
		       cid);
		return 0;
	} else {
		tty = cidatatty_table[CSDTTYINDEX].data_port->port.tty;
	}

	if (!tty) {
		printk(KERN_ERR "data_rx_csd: tty device is null. cid = %d\n",
		       cid);
		return 0;
	}
	cidatatty = tty->driver_data;
	/* drop packets if nobody uses cidatatty */
	if (cidatatty == NULL) {
		printk(KERN_ERR
		       "data_rx_csd: drop packets if nobody uses cidatatty. cid = %d\n",
		       cid);
		return 0;
	}

	down(&cidatatty->sem_lock_tty);

	if (cidatatty->port.count == 0) {
		up(&cidatatty->sem_lock_tty);
		printk(KERN_ERR "data_rx_csd: cidatatty device is closed\n");
		return 0;
	}

	ret = tty_insert_flip_string(&cidatatty->port, packet, len);
	tty_flip_buffer_push(&cidatatty->port);

	up(&cidatatty->sem_lock_tty);

	if (ret < len)
		printk(KERN_ERR
		       "data_rx_csd: give up because receive_room left\n");

	return ret;
}

int data_rx_ims(char *packet, int len, unsigned char cid)
{
	struct tty_struct *tty = NULL;
	struct cidatatty_port *cidatatty;
	int ret;

	F_ENTER();

	if (!cidatatty_table[IMSTTYINDEX].data_port
	    || cidatatty_table[IMSTTYINDEX].data_port->cid != cid
	    || cidatatty_table[IMSTTYINDEX].data_port->devinuse != 1) {
		printk(KERN_ERR "data_rx_ims: not found tty device. cid = %d\n",
		       cid);
		return 0;
	} else {
		tty = cidatatty_table[IMSTTYINDEX].data_port->port.tty;
	}

	if (!tty) {
		printk(KERN_ERR "data_rx_ims: tty device is null. cid = %d\n",
		       cid);
		return 0;
	}

	cidatatty = tty->driver_data;
	/* drop packets if nobody uses cidatatty */
	if (cidatatty == NULL) {
		printk(KERN_ERR
		       "data_rx_ims: drop packets if nobody uses cidatatty. cid = %d\n",
		       cid);
		return 0;
	}

	down(&cidatatty->sem_lock_tty);

	if (cidatatty->port.count == 0) {
		up(&cidatatty->sem_lock_tty);
		printk(KERN_ERR "data_rx_ims: cidatatty device is closed\n");
		return 0;
	} else {
		u32 nleft = 0, npushed = 0;
		unsigned char *ptr = NULL;
		u32 retry = 0, retry_limits = 200;
		nleft = len;
		ptr = packet;
		while (nleft > 0 && retry < retry_limits) {
			npushed = tty_insert_flip_string(&cidatatty->port,
							 ptr,
							 nleft);
			tty_flip_buffer_push(&cidatatty->port);
			nleft -= npushed;
			ptr += npushed;
			if (nleft) {
				if (!retry)
					printk(KERN_DEBUG "data_rx_ims: wait receive_buf available\n");
				msleep(20);
			}
			retry++;
		}
		ret = len - nleft;
		if (ret < len)
			printk(KERN_DEBUG
			       "data_rx_ims: wait receive_buf available timeout, lose this packet\n");
	}

	up(&cidatatty->sem_lock_tty);
	return ret;
}

static int cidatatty_open(struct tty_struct *tty, struct file *file)
{
	struct cidatatty_port *cidatatty;
	int index;
	int status;
	unsigned long flags;

	F_ENTER();
	/* initialize the pointer in case something fails */
	tty->driver_data = NULL;

	/* get the serial object associated with this tty pointer */
	index = tty->index;
	/* ttycurindex = index; */
	/* PDEBUG("ttycurindex=%d\n", ttycurindex); */
	do {
		mutex_lock(&cidatatty_table[index].lock);
		cidatatty = cidatatty_table[index].data_port;
		if (!cidatatty)
			status = -ENODEV;
		else {
			spin_lock_irqsave(&cidatatty->port_lock, flags);
			/* already open? great */
			if (cidatatty->port.count) {
				status = 0;
				cidatatty->port.count++;
			/* currently opening/closing? wait */
			} else if (cidatatty->openclose) {
				status = -EBUSY;
			/* else we do the work */
			} else {
				status = -EAGAIN;
				cidatatty->openclose = true;
			}
			spin_unlock_irqrestore(&cidatatty->port_lock, flags);
		}
		mutex_unlock(&cidatatty_table[index].lock);

		switch (status) {
		default:
			/* fully handled */
			return status;
		case -EAGAIN:
			/* must do the work */
			break;
		case -EBUSY:
			/* wait for EAGAIN task to finish */
			msleep(20);
			/* REVIST could have a wait channel here, if
			 * concurrent open performance is important */
			break;
		}
	} while (status != -EAGAIN);

	/* do the real open operation */
	spin_lock_irqsave(&cidatatty->port_lock, flags);

	/* save our structure within the tty structure */
	tty->driver_data = cidatatty;

	/* jzwu1 */
	tty->flags = TTY_NO_WRITE_SPLIT | tty->flags;
	cidatatty->port.tty = tty;
	cidatatty->port.count = 0;
	cidatatty->openclose = false;
	cidatatty->port.low_latency = 1;
	status = 0;

	++cidatatty->port.count;
	if (cidatatty->port.count == 1) {
		/* this is the first time this port is opened */
		/* do any hardware initialization needed here */
		if (index == PSDTTYINDEX)
			registerRxCallBack(PDP_PPP,
					   (DataRxCallbackFunc) data_rx_ppp);
		else if (index == CSDTTYINDEX)
			registerRxCallBack(CSD_RAW,
					   (DataRxCallbackFunc) data_rx_csd);
		else if (index == IMSTTYINDEX)
			registerRxCallBack(IMS_RAW,
					   (DataRxCallbackFunc) data_rx_ims);
		else {
			cidatatty_table[index].data_port = NULL;
			spin_unlock_irqrestore(&cidatatty->port_lock, flags);
			tty_port_destroy(&cidatatty->port);
			kfree(cidatatty);
			printk(KERN_ERR "Not supported index: %d\n", index);
			return -EIO;
		}
	}
	cidatatty->writeokflag = 1;
	cidatatty->devinuse = 1;
	if (index == PSDTTYINDEX)
		cidatatty->cid = currcid_ps;
	else if (index == CSDTTYINDEX)
		cidatatty->cid = currcid_cs;
	else if (index == IMSTTYINDEX)
		cidatatty->cid = currcid_ims;

	spin_unlock_irqrestore(&cidatatty->port_lock, flags);

	F_LEAVE();
	return status;
}

static void do_close(struct cidatatty_port *cidatatty)
{
	unsigned long flags;

	F_ENTER();

	down(&cidatatty->sem_lock_tty);
	spin_lock_irqsave(&cidatatty->port_lock, flags);
	if (!cidatatty->port.count) {
		/* port was never opened */
		goto exit;
	}

	--cidatatty->port.count;
	if (cidatatty->port.count <= 0) {
		/* The port is being closed by the last user. */
		/* Do any hardware specific stuff here */
		if (cidatatty->port.tty->index == PSDTTYINDEX)
			unregisterRxCallBack(PDP_PPP);
		else if (cidatatty->port.tty->index == CSDTTYINDEX)
			unregisterRxCallBack(CSD_RAW);
		else if (cidatatty->port.tty->index == IMSTTYINDEX)
			unregisterRxCallBack(IMS_RAW);
	}
exit:
	spin_unlock_irqrestore(&cidatatty->port_lock, flags);
	up(&cidatatty->sem_lock_tty);

	PDEBUG("Leaving do_close: ");
}

static void cidatatty_close(struct tty_struct *tty, struct file *file)
{
	struct cidatatty_port *cidatatty = tty->driver_data;

	F_ENTER();

	if (cidatatty)
		do_close(cidatatty);

	F_LEAVE();
}

static int cidatatty_write(struct tty_struct *tty, const unsigned char *buffer,
			   int count)
{
	struct cidatatty_port *cidatatty = tty->driver_data;
	int retval = -EINVAL;
	int tty_index;
	int cid;
	int writeokflag;

	/*int i; */

	F_ENTER();
	/* for some reason, this function is called with count == 0 */
	if (count <= 0) {
		printk(KERN_ERR "Error: count is %d.\n", count);
		return 0;
	}
	if (!cidatatty) {
		printk(KERN_ERR
		       "Warning: cidatatty_write: cidatatty is NULL\n");
		return -ENODEV;
	}
	/* may be called in PPP  IRQ,  on interrupt time */
	spin_lock(&cidatatty->port_lock);

	if (!cidatatty->port.count) {
		spin_unlock(&cidatatty->port_lock);
		printk(KERN_ERR "Error: cidatatty_write: port was not open\n");
		/* port was not opened */
		goto exit;
	}

	tty_index = tty->index;
	writeokflag = cidatatty_table[tty_index].data_port->writeokflag;
	cid = cidatatty_table[tty_index].data_port->cid;

	spin_unlock(&cidatatty->port_lock);

#ifdef DEBUG_BUF_CONTENT
	/* int i; */
	printk(KERN_DEBUG "CIDATATTY Tx Buffer datalen is %d\n data:", count);
	for (i = 0; i < count; i++)	/* i=14? */
		printk(KERN_DEBUG "%02x(%c)", buffer[i]&0xff, buffer[i]&0xff);
	/*        printk(KERN_DEBUG " %02x", buffer[i]&0xff ); */
	printk(KERN_DEBUG "\n");
#endif

	if (writeokflag == 1) {
		if (tty_index == PSDTTYINDEX) {
			sendData(cid, (char *)buffer, count);
		} else if (tty_index == CSDTTYINDEX) {
			int sent = 0;
			while (sent < count) {
				int len = count - sent;
				if (len > 160)
					len = 160;
				sendCSData(cid, (char *)buffer + sent, len);
				sent += len;
			}
		} else if (tty_index == IMSTTYINDEX) {
			int sent = 0;
			while (sent < count) {
				int len = count - sent;
				if (len > IMS_DATA_STUB_WRITE_LIMIT)
					len = IMS_DATA_STUB_WRITE_LIMIT;
				retval = sendIMSData(cid,
					(char *)buffer + sent, len);
				if (retval < 0) {
					count = -EIO;
					goto exit;
				}
				sent += len;
			}
		} else
			printk(KERN_ERR "Write error for index:%d!\n",
			       tty_index);
	}

exit:
	F_LEAVE();
	retval = count;

	return retval;
}

static void cidatatty_set_termios(struct tty_struct *tty,
				  struct ktermios *old_termios)
{
	unsigned int cflag;

	F_ENTER();
	cflag = tty->termios.c_cflag;

	/* check that they really want us to change something */
	if (old_termios) {
		if ((cflag == old_termios->c_cflag) &&
		    (RELEVANT_IFLAG(tty->termios.c_iflag) ==
		     RELEVANT_IFLAG(old_termios->c_iflag))) {
			PDEBUG(" - nothing to change...\n");
			return;
		}
	}

	/* get the byte size */
	switch (cflag & CSIZE) {
	case CS5:
		PDEBUG(" - data bits = 5\n");
		break;
	case CS6:
		PDEBUG(" - data bits = 6\n");
		break;
	case CS7:
		PDEBUG(" - data bits = 7\n");
		break;
	default:
	case CS8:
		PDEBUG(" - data bits = 8\n");
		break;
	}

	/* determine the parity */
	if (cflag & PARENB)
		if (cflag & PARODD)
			PDEBUG(" - parity = odd\n");
		else
			PDEBUG(" - parity = even\n");
	else
		PDEBUG(" - parity = none\n");

	/* figure out the stop bits requested */
	if (cflag & CSTOPB)
		PDEBUG(" - stop bits = 2\n");
	else
		PDEBUG(" - stop bits = 1\n");

	/* figure out the hardware flow control settings */
	if (cflag & CRTSCTS)
		PDEBUG(" - RTS/CTS is enabled\n");
	else
		PDEBUG(" - RTS/CTS is disabled\n");

	/* determine software flow control */
	/* if we are implementing XON/XOFF, set the start and
	 * stop character in the device */
	if (I_IXOFF(tty) || I_IXON(tty))
		;/* CHECKPOINT */

	/* get the baud rate wanted */
	PDEBUG(" - baud rate = %d", tty_get_baud_rate(tty));

	F_LEAVE();
}

static int cidatatty_write_room(struct tty_struct *tty)
{
	/*
	 * cidatatty no need to support this function,
	 * and add this stub to avoid tty system complain
	 * no write_room function
	 */
	return 2048;
}

#if 0
static int cidatatty_tiocmget(struct tty_struct *tty, struct file *file)
{
	struct cidatatty_port *cidatatty = tty->driver_data;
	unsigned int result = 0;
	unsigned int msr = cidatatty->msr;
	unsigned int mcr = cidatatty->mcr;

	F_ENTER();

	result = ((mcr & MCR_DTR)  ? TIOCM_DTR  : 0) |  /* DTR is set */
		 ((mcr & MCR_RTS)  ? TIOCM_RTS  : 0) |  /* RTS is set */
		 ((mcr & MCR_LOOP) ? TIOCM_LOOP : 0) |  /* LOOP is set */
		 ((msr & MSR_CTS)  ? TIOCM_CTS  : 0) |  /* CTS is set */
		 ((msr & MSR_CD)   ? TIOCM_CAR  : 0) |  /* CD is set*/
		 ((msr & MSR_RI)   ? TIOCM_RI   : 0) |  /* RI is set */
		 ((msr & MSR_DSR)  ? TIOCM_DSR  : 0);   /* DSR is set */

	F_LEAVE();
	return result;
}

static int cidatatty_tiocmset(struct tty_struct *tty, struct file *file,
			      unsigned int set, unsigned int clear)
{
	struct cidatatty_port *cidatatty = tty->driver_data;
	unsigned int mcr = cidatatty->mcr;

	F_ENTER();
	if (set & TIOCM_RTS)
		mcr |= MCR_RTS;
	if (set & TIOCM_DTR)
		mcr |= MCR_RTS;
	if (set & TIOCM_LOOP)
		mcr |= MCR_RTS;
	if (set & TIOCM_CTS)
		mcr |= MCR_RTS;
	if (set & TIOCM_RI)
		mcr |= MCR_RTS;
	if (set & TIOCM_DSR)
		mcr |= MCR_RTS;
	if (set & TIOCM_CAR)
		mcr |= MCR_RTS;

	if (clear & TIOCM_RTS)
		mcr &= ~MCR_RTS;
	if (clear & TIOCM_DTR)
		mcr &= ~MCR_RTS;
	if (clear & TIOCM_LOOP)
		mcr &= ~MCR_RTS;
	if (clear & TIOCM_CTS)
		mcr &= ~MCR_RTS;
	if (clear & TIOCM_CAR)
		mcr &= ~MCR_RTS;
	if (clear & TIOCM_RI)
		mcr &= ~MCR_RTS;
	if (clear & TIOCM_DSR)
		mcr &= ~MCR_RTS;

	/* set the new MCR value in the device */
	cidatatty->mcr = mcr;

	F_LEAVE();
	return 0;
}
#endif

static int cidatatty_ioctl_tiocgserial(struct tty_struct *tty,
				       struct file *file, unsigned int cmd,
				       unsigned long arg)
{
	struct cidatatty_port *cidatatty = tty->driver_data;

	F_ENTER();

	if (cmd == TIOCGSERIAL) {
		struct serial_struct tmp;

		if (!arg)
			return -EFAULT;

		memset(&tmp, 0, sizeof(tmp));

		tmp.type                = cidatatty->serial.type;
		tmp.line                = cidatatty->serial.line;
		tmp.port                = cidatatty->serial.port;
		tmp.irq                 = cidatatty->serial.irq;
		tmp.flags               = ASYNC_SKIP_TEST | ASYNC_AUTO_IRQ;
		tmp.xmit_fifo_size      = cidatatty->serial.xmit_fifo_size;
		tmp.baud_base           = cidatatty->serial.baud_base;
		tmp.close_delay         = 5 * HZ;
		tmp.closing_wait        = 30 * HZ;
		tmp.custom_divisor      = cidatatty->serial.custom_divisor;
		tmp.hub6                = cidatatty->serial.hub6;
		tmp.io_type             = cidatatty->serial.io_type;

		if (copy_to_user
		    ((void __user *)arg, &tmp, sizeof(struct serial_struct)))
			return -EFAULT;
		return 0;
	}

	F_LEAVE();
	return -ENOIOCTLCMD;
}

static int cidatatty_ioctl_tiocmiwait(struct tty_struct *tty, struct file *file,
				      unsigned int cmd, unsigned long arg)
{
	struct cidatatty_port *cidatatty = tty->driver_data;

	F_ENTER();

	if (cmd == TIOCMIWAIT) {
		DECLARE_WAITQUEUE(wait, current);
		struct async_icount cnow;
		struct async_icount cprev;

		cprev = cidatatty->icount;
		while (1) {
			add_wait_queue(&cidatatty->wait, &wait);
			set_current_state(TASK_INTERRUPTIBLE);
			schedule();
			remove_wait_queue(&cidatatty->wait, &wait);

			/* see if a signal woke us up */
			if (signal_pending(current))
				return -ERESTARTSYS;

			cnow = cidatatty->icount;
			if (cnow.rng == cprev.rng && cnow.dsr == cprev.dsr &&
			    cnow.dcd == cprev.dcd && cnow.cts == cprev.cts)
				return -EIO;  /* no change => error */
			if (((arg & TIOCM_RNG) && (cnow.rng != cprev.rng)) ||
			    ((arg & TIOCM_DSR) && (cnow.dsr != cprev.dsr)) ||
			    ((arg & TIOCM_CD)  && (cnow.dcd != cprev.dcd)) ||
			    ((arg & TIOCM_CTS) && (cnow.cts != cprev.cts))) {
				return 0;
			}
			cprev = cnow;
		}

	}

	F_LEAVE();
	return -ENOIOCTLCMD;
}

static int cidatatty_ioctl_tiocgicount(struct tty_struct *tty,
				       struct file *file, unsigned int cmd,
				       unsigned long arg)
{
	struct cidatatty_port *cidatatty = tty->driver_data;

	F_ENTER();
	if (cmd == TIOCGICOUNT) {
		struct async_icount cnow = cidatatty->icount;
		struct serial_icounter_struct icount;

		icount.cts      = cnow.cts;
		icount.dsr      = cnow.dsr;
		icount.rng      = cnow.rng;
		icount.dcd      = cnow.dcd;
		icount.rx       = cnow.rx;
		icount.tx       = cnow.tx;
		icount.frame    = cnow.frame;
		icount.overrun  = cnow.overrun;
		icount.parity   = cnow.parity;
		icount.brk      = cnow.brk;
		icount.buf_overrun = cnow.buf_overrun;

		if (copy_to_user((void __user *)arg, &icount, sizeof(icount)))
			return -EFAULT;
		return 0;
	}

	F_LEAVE();
	return -ENOIOCTLCMD;
}

static int cidatatty_ioctl_tcsets(struct tty_struct *tty, struct file *file,
				  unsigned int cmd, unsigned long arg)
{
	F_ENTER();

	memcpy((void *)&tty->termios, (void *)arg, sizeof(struct ktermios));
	/*
	   struct termios * new_termios = (struct termios *) arg;

	   tty->termios->c_iflag = new_termios->c_iflag;
	   tty->termios->c_oflag = new_termios->c_oflag;
	   tty->termios->c_cflag = new_termios->c_cflag;
	   tty->termios->c_lflag = new_termios->c_lflag;
	   tty->termios->c_line  = new_termios->c_line;

	   int i;

	   for(i = 0; i <NCCS; i++)
	   tty->termios->c_cc[i] = new_termios->c_cc[i];
	 */
	F_LEAVE();

	return 0;
}

static int cidatatty_ioctl_tcgets(struct tty_struct *tty, struct file *file,
				  unsigned int cmd, unsigned long arg)
{
	F_ENTER();

	if (copy_to_user((void *)arg,
			 (void *)&tty->termios,
			 sizeof(struct ktermios))) {
		PDEBUG("Failed to copy to user for tcgets.\n");
		return -EFAULT;
	}
	F_LEAVE();

	return 0;
}

int enable_datapath(struct tty_struct *tty)
{
	struct cidatatty_port *cidatatty = tty->driver_data;

	if (!cidatatty)
		return 0;

	if (tty->index == PSDTTYINDEX)
		registerRxCallBack(PDP_PPP, (DataRxCallbackFunc) data_rx_ppp);
	else if (tty->index == CSDTTYINDEX)
		registerRxCallBack(CSD_RAW, (DataRxCallbackFunc) data_rx_csd);
	else if (tty->index == IMSTTYINDEX)
		registerRxCallBack(IMS_RAW, (DataRxCallbackFunc)data_rx_ims);

	return 0;
}

int disable_datapath(struct tty_struct *tty)
{
	if (tty->index == PSDTTYINDEX)
		unregisterRxCallBack(PDP_PPP);
	else if (tty->index == CSDTTYINDEX)
		unregisterRxCallBack(CSD_RAW);
	else if	(tty->index == IMSTTYINDEX)
		unregisterRxCallBack(IMS_RAW);
	return 0;
}

/*
 * the real cidatatty_ioctl function.
 * The above is done to get the small functions
 */
static int cidatatty_ioctl(struct tty_struct *tty,
			   unsigned int cmd, unsigned long arg)
{
	struct file *file = NULL;
	/* unsigned char cid; */
	F_ENTER();
	switch (cmd) {
	case TIOCGSERIAL:
		return cidatatty_ioctl_tiocgserial(tty, file, cmd, arg);
	case TIOCMIWAIT:
		return cidatatty_ioctl_tiocmiwait(tty, file, cmd, arg);
	case TIOCGICOUNT:
		return cidatatty_ioctl_tiocgicount(tty, file, cmd, arg);
	case TCSETS:
		return cidatatty_ioctl_tcsets(tty, file, cmd, arg);
	case TCGETS:		/* 0x5401 ioctls.h */
		return cidatatty_ioctl_tcgets(tty, file, cmd, arg);
	case TCSETSF:		/* 0x5404 */
	case TCSETAF:		/* 0x5408 */
		return 0;	/* has to return zero for qtopia to work */

	default:
		PDEBUG("cidatatty_ioctl cmd: %d.\n", cmd);
		return -ENOIOCTLCMD;	/* for PPPD to work? */

		break;
	}

	F_LEAVE();

}

static int cidatatty_chars_in_buffer(struct tty_struct *tty)
{
	return 0;
}

#if 0
static void cidatatty_flush_chars(struct tty_struct *tty)
{
	F_ENTER();
	return;
}

static void cidatatty_wait_until_sent(struct tty_struct *tty, int timeout)
{
	F_ENTER();
	return;
}
#endif
static void cidatatty_throttle(struct tty_struct *tty)
{
	F_ENTER();
	return;
}

static void cidatatty_unthrottle(struct tty_struct *tty)
{
	F_ENTER();
	return;
}

#if 0
static void cidatatty_stop(struct tty_struct *tty)
{
	F_ENTER();
	return;
}

static void cidatatty_start(struct tty_struct *tty)
{
	F_ENTER();
	return;
}

static void cidatatty_hangup(struct tty_struct *tty)
{
	F_ENTER();
	return;
}
#endif
static void cidatatty_flush_buffer(struct tty_struct *tty)
{
	F_ENTER();
	return;
}

#if 0
static void cidatatty_set_ldisc(struct tty_struct *tty)
{
	F_ENTER();
	return;
}
#endif

static const struct tty_operations serial_ops = {
	.open = cidatatty_open,
	.close = cidatatty_close,
	.write = cidatatty_write,
	.set_termios = cidatatty_set_termios,
	.write_room = cidatatty_write_room,
	/* .tiocmget = cidatatty_tiocmget, */
	/* .tiocmset = cidatatty_tiocmset, */
	.ioctl = cidatatty_ioctl,
	/* for PPPD to work chars_in_buffer needs to return zero. */
	/* uncomment this, minicom works */
	.chars_in_buffer = cidatatty_chars_in_buffer,
	/* .flush_chars = cidatatty_flush_chars, */
	/* .wait_until_sent = cidatatty_wait_until_sent, */
	.throttle = cidatatty_throttle,
	.unthrottle = cidatatty_unthrottle,
	/* .stop = cidatatty_stop, */
	/* .start = cidatatty_start, */
	/* .hangup = cidatatty_hangup, */
	.flush_buffer = cidatatty_flush_buffer,
	/* .set_ldisc = cidatatty_set_ldisc, */
};

int cctdatadev_open(struct inode *inode, struct file *filp)
{
	struct cctdatadev_dev *dev;
	F_ENTER();
#if 1
	dev = container_of(inode->i_cdev, struct cctdatadev_dev, cdev);

	filp->private_data = dev;	/* for other methods */

	/* used to keep track of how many readers */
	if (filp->f_mode & FMODE_READ)
		dev->nreaders++;
	if (filp->f_mode & FMODE_WRITE)
		dev->nwriters++;
#endif
	F_LEAVE();

	return nonseekable_open(inode, filp);	/* success */
}

long cctdatadev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{

	unsigned char cid;
	int i, index;
	struct cctdatadev_dev *dev = filp->private_data;

	index = MINOR(dev->cdev.dev);

	switch (cmd) {
	case TIOENABLE:
		cid = (unsigned char)arg;
		printk(KERN_DEBUG "cctdatadev_ioctl: TIOENABLE cid=%d\n", cid);
		if (cidatatty_table[index].data_port) {
			cidatatty_table[index].data_port->cid = cid;
			cidatatty_table[index].data_port->devinuse = 1;
		}
		break;
	case TIODISABLE:
		cid = (unsigned char)arg;
		printk(KERN_DEBUG "cctdatadev_ioctl: TIODISABLE cid=%d\n", cid);
		for (i = 0; i < CIDATATTY_TTY_MINORS; i++) {
			if (cidatatty_table[i].data_port
			    && (cidatatty_table[i].data_port->cid == cid)
			    && (cidatatty_table[i].data_port->devinuse == 1)) {
				cidatatty_table[i].data_port->devinuse = 0;
				cidatatty_table[i].data_port->writeokflag = 0;
				break;
			}
		}
		if (i >= CIDATATTY_TTY_MINORS)
			printk(KERN_ERR
			       "cctdatadev_ioctl: cidatatty dev not found.\n");
		break;
	case TIOPPPON:
	case TIOPPPONCSD:
		cid = (unsigned char)arg;
		printk(KERN_DEBUG
		       "cctdatadev_ioctl: TIOPPPON: cid=%d, index=%d\n", cid,
		       index);
#if 0
		for (i = 0; i < CIDATATTY_TTY_MINORS; i++) {
			if ((cidatatty_table[i].data_port->cid == cid) &&
			    (cidatatty_table[i].data_port->devinuse == 1)) {
				cidatatty_table[i].data_port->writeokflag = 1;
				break;
			}
		}
#endif
		/* if (cidatatty_table[index].data_port) */
		/*      cidatatty_table[index].data_port->cid =
		 *					(unsigned char)arg; */
		if (index == CSDTTYINDEX)
			currcid_cs = (unsigned char)arg;
		else if (index == PSDTTYINDEX)
			currcid_ps = (unsigned char)arg;
		else if (index == IMSTTYINDEX)
			currcid_ims = (unsigned char)arg;
		break;
	case TIOPPPOFF:
		/*      if (cidatatty_table[index].data_port) */
		/*      cidatatty_table[index].data_port->cid =  0xff; */

		printk(KERN_DEBUG "cctdatadev_ioctl: TIOPPPOFF: index=%d\n",
		       index);
		if (index == CSDTTYINDEX)
			currcid_cs = 0xff;
		else if (index == PSDTTYINDEX)
			currcid_ps = 0xff;
		else if (index == IMSTTYINDEX)
			currcid_ims = 0xff;

		break;
	default:
		printk(KERN_DEBUG "cctdatadev_ioctl cmd: %d.\n", cmd);
		return -ENOIOCTLCMD;

		break;
	}
	return 0;

}

static struct class *cctdatadev_class;

const struct file_operations cctdatadev_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = cctdatadev_ioctl,
	.open = cctdatadev_open,
	.llseek = no_llseek,
};

void cctdatadev_cleanup_module(void)
{
	int i;
	dev_t devno = MKDEV(cctdatadev_major, cctdatadev_minor);

	/* Get rid of our char dev entries */
	if (cctdatadev_devices) {
		for (i = 0; i < cctdatadev_nr_devs; i++) {
			cdev_del(&cctdatadev_devices[i].cdev);
			device_destroy(cctdatadev_class,
				       MKDEV(cctdatadev_major,
					     cctdatadev_minor + i));
		}
		kfree(cctdatadev_devices);
	}

	class_destroy(cctdatadev_class);

	/* cleanup_module is never called if registering failed */
	unregister_chrdev_region(devno, cctdatadev_nr_devs);

}

static void cctdatadev_setup_cdev(struct cctdatadev_dev *dev, int index)
{
	int err, devno = MKDEV(cctdatadev_major, cctdatadev_minor + index);

	F_ENTER();

	cdev_init(&dev->cdev, &cctdatadev_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &cctdatadev_fops;
	err = cdev_add(&dev->cdev, devno, 1);
	/* Fail gracefully if need be */
	if (err)
		printk(KERN_NOTICE "Error %d adding cctdatadev%d", err, index);

	F_LEAVE();

}

int cctdatadev_init_module(void)
{
	int result, i;
	dev_t dev = 0;
	char name[256];

	F_ENTER();

	/*
	 * Get a range of minor numbers to work with, asking for a dynamic
	 * major unless directed otherwise at load time.
	 */
	if (cctdatadev_major) {
		dev = MKDEV(cctdatadev_major, cctdatadev_minor);
		result =
		    register_chrdev_region(dev, cctdatadev_nr_devs,
					   "cctdatadev");
	} else {
		result =
		    alloc_chrdev_region(&dev, cctdatadev_minor,
					cctdatadev_nr_devs, "cctdatadev");
		cctdatadev_major = MAJOR(dev);
	}

	if (result < 0) {
		printk(KERN_WARNING "cctdatadev: can't get major %d\n",
		       cctdatadev_major);
		return result;
	}

	/*
	 * allocate the devices -- we can't have them static, as the number
	 * can be specified at load time
	 */
	cctdatadev_devices =
	    kmalloc(cctdatadev_nr_devs * sizeof(struct cctdatadev_dev),
		    GFP_KERNEL);
	if (!cctdatadev_devices) {
		result = -ENOMEM;
		goto fail;
	}
	memset(cctdatadev_devices, 0,
	       cctdatadev_nr_devs * sizeof(struct cctdatadev_dev));

	/* Initialize each device. */
	cctdatadev_class = class_create(THIS_MODULE, "cctdatadev");
	for (i = 0; i < cctdatadev_nr_devs; i++) {
		sprintf(name, "%s%d", "cctdatadev", cctdatadev_minor + i);
		device_create(cctdatadev_class, NULL,
			      MKDEV(cctdatadev_major, cctdatadev_minor + i),
			      NULL, name);
		cctdatadev_setup_cdev(&cctdatadev_devices[i], i);

	}

	/* At this point call the init function for any friend device */
	/* dev = MKDEV(cctdatadev_major,
	       cctdatadev_minor + cctdatadev_nr_devs); */

	F_LEAVE();

	return 0;		/* succeed */

fail:
	cctdatadev_cleanup_module();

	return result;
}

static int cidatatty_port_alloc(unsigned int index)
{
	struct cidatatty_port *cidatatty;
	int ret = 0;

	mutex_lock(&cidatatty_table[index].lock);
	cidatatty = kzalloc(sizeof(struct cidatatty_port), GFP_KERNEL);
	if (cidatatty == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	tty_port_init(&cidatatty->port);
	spin_lock_init(&cidatatty->port_lock);
	sema_init(&cidatatty->sem_lock_tty, 1);
	init_waitqueue_head(&cidatatty->wait);

	cidatatty_table[index].data_port = cidatatty;

out:
	mutex_unlock(&cidatatty_table[index].lock);
	return ret;
}

static int __init cidatatty_init(void)
{
	int retval;
	int i;

	F_ENTER();

	/* allocate the tty driver */
	cidatatty_tty_driver = alloc_tty_driver(CIDATATTY_TTY_MINORS);
	if (!cidatatty_tty_driver)
		return -ENOMEM;

	/* initialize the tty driver */
	cidatatty_tty_driver->owner = THIS_MODULE;
	cidatatty_tty_driver->driver_name = "cidatatty_tty";
	cidatatty_tty_driver->name = "cidatatty";
	cidatatty_tty_driver->major = CIDATATTY_TTY_MAJOR;
	cidatatty_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	cidatatty_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	cidatatty_tty_driver->flags =
	    TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	cidatatty_tty_driver->init_termios = tty_std_termios;
	/* B115200 | CS8 | CREAD | HUPCL | CLOCAL; */
	cidatatty_tty_driver->init_termios.c_cflag =
	    B9600 | CS8 | CREAD | CLOCAL;
	cidatatty_tty_driver->init_termios.c_iflag = IGNBRK | IGNPAR;
	cidatatty_tty_driver->init_termios.c_oflag = 0;
	cidatatty_tty_driver->init_termios.c_lflag = 0;

	tty_set_operations(cidatatty_tty_driver, &serial_ops);

	for (i = 0; i < CIDATATTY_TTY_MINORS; i++) {
		mutex_init(&cidatatty_table[i].lock);
		cidatatty_port_alloc(i); /* need defence code */
	}

	/* register the tty driver */
	retval = tty_register_driver(cidatatty_tty_driver);
	if (retval) {
		printk(KERN_ERR "failed to register cidatatty tty driver");
		put_tty_driver(cidatatty_tty_driver);
		cidatatty_tty_driver = NULL;
		return retval;
	}

	/* register tty devices */
	for (i = 0; i < CIDATATTY_TTY_MINORS; ++i) {
		struct device *tty_dev;

		tty_dev = tty_port_register_device(
				&cidatatty_table[i].data_port->port,
				cidatatty_tty_driver,
				i,
				NULL);
		if (IS_ERR(tty_dev)) {
			struct cidatatty_port *cidatatty;
			pr_err("%s: failed to register tty for port %d, err %ld",
					__func__, i, PTR_ERR(tty_dev));
			retval = PTR_ERR(tty_dev);
			cidatatty = cidatatty_table[i].data_port;
			cidatatty_table[i].data_port = NULL;
			tty_port_destroy(&cidatatty->port);
			kfree(cidatatty);
		}
	}

	printk(KERN_INFO DRIVER_DESC " " DRIVER_VERSION "\n");
	cctdatadev_init_module();
	F_LEAVE();
	return retval;
}

static void __exit cidatatty_exit(void)
{
	struct cidatatty_port *cidatatty;
	int i;

	F_ENTER();

	/* unregister device */
	for (i = 0; i < CIDATATTY_TTY_MINORS; ++i) {
		cidatatty = cidatatty_table[i].data_port;
		if (cidatatty) {
			/* close the port */
			while (cidatatty->port.count)
				do_close(cidatatty);

			cidatatty_table[i].data_port = NULL;
			tty_port_destroy(&cidatatty->port);
			kfree(cidatatty);
		}
		tty_unregister_device(cidatatty_tty_driver, i);
	}

	/* unregister driver */
	tty_unregister_driver(cidatatty_tty_driver);

	cctdatadev_cleanup_module();
	F_LEAVE();
}

module_init(cidatatty_init);
module_exit(cidatatty_exit);
