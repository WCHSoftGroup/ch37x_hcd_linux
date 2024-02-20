/*
 * USB host controller driver for SPI to USB chip ch370/ch374.
 *
 * Copyright (C) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Web:      http://wch.cn
 * Author:   WCH <tech@wch.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <asm/irq.h>
#include <asm/byteorder.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/list.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include "ch37x.h"

#define DRIVER_AUTHOR "WCH"
#define DRIVER_DESC   "CH370/CH374 USB Host-Controller Driver"
#define VERSION_DESC  "V1.7 On 2024.01"

//#define USE_IRQ_FROM_DTS
#define GPIO_NUMBER 0

#define DEBUG_FLAG_LEVEL 0

/* 11-bit counter that wraps around (USB 2.0 Section 8.3.3): */
#define USB_MAX_FRAME_NUMBER 0x7ff
#define USB_MAX_RETRIES	     3 /* # of retries before error is reported */

/*
 * Max. # of times we're willing to retransmit a request immediately in
 * resposne to a NAK.  Afterwards, we fall back on trying once a frame.
 */
#define NAK_MAX_FAST_RETRANSMITS 5

#define POWER_BUDGET 500

#define RH_PORT_STATUS_STAYED  0
#define RH_PORT_STATUS_CHANGED 1

#define CMD_SPI_374READ	 0xC0
#define CMD_SPI_374WRITE 0x80

#define ERR_USB_UNKNOWN 0xFA
#ifndef USB_INT_SUCCESS
#define USB_INT_SUCCESS	   0x14
#define USB_INT_CONNECT	   0x15
#define USB_INT_DISCONNECT 0x16
#define USB_INT_BUF_OVER   0x17
#define USB_INT_DISK_ERR   0x1F
#endif
/* ======================================= */
#define DEF_USB_PID_NULL  0x00
#define DEF_USB_PID_SOF	  0x05
#define DEF_USB_PID_SETUP 0x0D
#define DEF_USB_PID_IN	  0x09
#define DEF_USB_PID_OUT	  0x01
#define DEF_USB_PID_ACK	  0x02
#define DEF_USB_PID_NAK	  0x0A
#define DEF_USB_PID_STALL 0x0E
#define DEF_USB_PID_DATA0 0x03
#define DEF_USB_PID_DATA1 0x0B
#define DEF_USB_PID_PRE	  0x0C

#define CH374_TOKEN_SETUP	 M_MK_HOST_PID_ENDP(DEF_USB_PID_SETUP, 0)
#define CH374_TOKEN_BULK_IN(ep)	 M_MK_HOST_PID_ENDP(DEF_USB_PID_IN, ep)
#define CH374_TOKEN_BULK_OUT(ep) M_MK_HOST_PID_ENDP(DEF_USB_PID_OUT, ep)

#define MAX_FIFO_SIZE 64
#define NUM_PORTS     3
/* Port-change mask: */
#define PORT_C_MASK                                                                                                    \
	(USB_PORT_STAT_C_CONNECTION | USB_PORT_STAT_C_ENABLE | USB_PORT_STAT_C_SUSPEND | USB_PORT_STAT_C_OVERCURRENT | \
	 USB_PORT_STAT_C_RESET)

enum pkt_state {
	PKT_STATE_NONE = 0,
	PKT_STATE_SETUP,    /* waiting to send setup packet to ctrl pipe */
	PKT_STATE_TRANSFER, /* waiting to xfer transfer_buffer */
	PKT_STATE_TERMINATE
};

enum ch37x_rh_state {
	CH374_RH_RESET = 0,
	CH374_RH_SUSPENDED,
	CH374_RH_RUNNING
};

enum ch37x_pid_state {
	CH374_DEF_USB_PID_SETUP = 0,
	CH374_DEF_USB_PID_OUT,
	CH374_DEF_USB_PID_IN
};

enum {
	ENABLE_IRQ = 0,
	RESET_HCD,
	RESET_PORT,
	CHECK_CONNECT,
	CHECK_UNLINK
};

enum {
	UNINTERRUPT = 0,
	INTERRUPT
};

struct ch37x_chip_data {
	atomic_t flag_low_speed; //if usb 1.0 then set 1
	//int flag_device_status;
	unsigned char endp_num; //num of hid such as key or mouse
	unsigned char endp_in_addr;
	unsigned char endp_out_addr;
	unsigned char hid_dest_len;
	unsigned char UsbDevEndpSize;
	unsigned char FlagDeviceStatus;
};

struct xgold_spi_chip {
	u8 poll_mode; /* 0 for contoller polling mode */
	u8 type;      /* SPI/SSP/Micrwire */
	u8 enable_dma;
	int hcd_irq;
	void (*cs_control)(u32 command);
};

struct ch37x_dma_buf {
	u8 data[3];
};

enum usb_dev_sta {
	USB_UNLINK = 0,
	USB_RESET,
	USB_NOADDR,
	USB_ADDR
};

struct ch37x_port {
	u8 bus_addr;
	u8 bus_addr_backup;
	bool controltog; /*setup packet tog*/
	bool transferintog[4];
	bool transferouttog[4];
	unsigned long todoport;
	/* lower 16 bits contain port status, upper 16 bits the change mask: */
	struct usb_port_status port_status;
	unsigned short wPortTrigger;
	enum usb_dev_sta usbsta;
	bool todosuspend;

	unsigned int ch37x_usb_flag;
};

struct ch37x_hcd_data {
	wait_queue_head_t wthread; /* for thread */
	bool kthread_running;
	spinlock_t lock;
	struct kthread_worker spi_worker;
	struct task_struct *spi_thread;
	struct kthread_work spi_work;

	struct ch37x_hcd_data *next;

	enum ch37x_rh_state rh_state;
	enum ch37x_pid_state pid_state;

	unsigned active : 1;

	struct list_head ep_list; /* list of EP's with work */
	u16 frame_number;
	int olddevnum;

	int conn_port; /* Port that is currently connecting, -1 if none.*/
	struct ch37x_port ports[NUM_PORTS + 1];

	/*
	 * kmalloc'd buffers guaranteed to be in separate (DMA)
	 * cache-lines:
	 */
	struct ch37x_dma_buf *tx;
	struct ch37x_dma_buf *rx;

	/*
	 * URB we're currently processing.  Must not be reset to NULL
	 */
	struct urb *curr_urb;
	struct urb *unlink_urb;
	int unlink_status;

	bool isLastSetup; /*last setup packet finished flag*/
	int urb_done;	  /* > 0 -> no errors, < 0: errno */
	int interrupt;
	size_t curr_len;
	unsigned long todo;

	struct device *dev;
	struct spi_device *spi;
	int irq;

	struct mutex thread_mutex;
	atomic_t suspend_flag;
	struct delayed_work delaywork;
	int addr;
	atomic_t debug_flag;
	atomic_t urb_process;
#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs;
	struct dentry *debugfile;
#endif
};

struct ch37x_ep {
	struct usb_host_endpoint *ep;
	struct list_head ep_list;
	u32 naks;
	u8 retransmit; /* packet needs retransmission */
	enum pkt_state pkt_state;
	enum pkt_state curr_pkt;
};

void ch37x_hcd_dbg(struct ch37x_hcd_data *hcd, const char *format, ...)
{
	va_list args;

	if (atomic_read(&hcd->debug_flag) >= 1) {
		va_start(args, format);
		vprintk(format, args);
		va_end(args);
	}
}

void ch37x_hcd_dbg_data(struct ch37x_hcd_data *hcd, const char *format, ...)
{
	va_list args;

	if (atomic_read(&hcd->debug_flag) >= 2) {
		va_start(args, format);
		vprintk(format, args);
		va_end(args);
	}
}

static struct ch37x_hcd_data *ch37x_hcd_list;
static spinlock_t spilock;

static inline s16 frame_diff(u16 left, u16 right)
{
	return ((unsigned)(left - right)) % (USB_MAX_FRAME_NUMBER + 1);
}

static inline struct ch37x_hcd_data *hcd_to_ch37x(struct usb_hcd *hcd)
{
	return (struct ch37x_hcd_data *)hcd->hcd_priv;
}

static inline struct usb_hcd *ch37x_to_hcd(struct ch37x_hcd_data *ch37x_hcd)
{
	return container_of((void *)ch37x_hcd, struct usb_hcd, hcd_priv);
}

static void ch37x_setpid(struct usb_hcd *hcd, struct urb *urb, bool tog);

static bool change_hub_port_speed(struct usb_hcd *hcd, int index);

static int ch37x_get_port_from_addr(struct ch37x_hcd_data *ch37x_hcd, u8 bus_addr);

static int ch37x_get_port_from_addr_backup(struct ch37x_hcd_data *ch37x_hcd, u8 bus_addr);

static int Write374Byte(struct usb_hcd *hcd, unsigned char mAddr, unsigned char mData)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	struct spi_device *spi = to_spi_device(hcd->self.controller);

	unsigned char txbuf[3];
	int ret;
	txbuf[0] = mAddr;
	txbuf[1] = CMD_SPI_374WRITE;
	txbuf[2] = mData;

	ret = spi_write(spi, txbuf, ARRAY_SIZE(txbuf));
	ch37x_hcd_dbg_data(ch37x_hcd, "%s:tx:0x%02x,0x%02x,0x%02x\n", __func__, txbuf[0], txbuf[1], txbuf[2]);

	return ret;
}

static unsigned char Read374Byte(struct usb_hcd *hcd, unsigned char mAddr)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	struct spi_device *spi = to_spi_device(hcd->self.controller);

	unsigned char txbuf[2];
	unsigned char rxbuf[1] = { 0 };
	int ret;

	txbuf[0] = mAddr;
	txbuf[1] = CMD_SPI_374READ;

	ret = spi_write_then_read(spi, txbuf, ARRAY_SIZE(txbuf), rxbuf, ARRAY_SIZE(rxbuf));

	ch37x_hcd_dbg_data(ch37x_hcd, "%s:tx:0x%02x,0x%02x, rx:0x%02x\n", __func__, txbuf[0], txbuf[1], rxbuf[0]);

	return rxbuf[0];
}

void Read374Block(struct usb_hcd *hcd, unsigned char mAddr, unsigned char mLen, unsigned char *mBuf)
{
	int ret = 0;
	int i = 0;

	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	struct spi_device *spi = to_spi_device(hcd->self.controller);
	unsigned char txbuf[2];

	txbuf[0] = mAddr;
	txbuf[1] = CMD_SPI_374READ;

	ret = spi_write_then_read(spi, txbuf, ARRAY_SIZE(txbuf), mBuf, mLen);
	if (ret)
		return;

	ch37x_hcd_dbg_data(ch37x_hcd, "%s:mAddr=0x%02x, mLen=%d , ret = %d \n", __func__, mAddr, mLen, ret);

	for (i = 0; i < mLen; i++)
		ch37x_hcd_dbg_data(ch37x_hcd, "0x%02x,", mBuf[i]);

	ch37x_hcd_dbg_data(ch37x_hcd, "\n");
}

void Write374Block(struct usb_hcd *hcd, unsigned char mAddr, unsigned char mLen, unsigned char *mBuf)
{
	unsigned char txbuf[256];
	int ret = 0;
	int i = 0;

	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	struct spi_device *spi = to_spi_device(hcd->self.controller);

	txbuf[0] = mAddr;
	txbuf[1] = CMD_SPI_374WRITE;
	memcpy(&txbuf[2], mBuf, mLen);

	ret = spi_write(spi, txbuf, mLen + 2);
	if (ret)
		return;

	ch37x_hcd_dbg_data(ch37x_hcd, "%s:mAddr=0x%02x, mLen=%d\n", __func__, mAddr, mLen);

	for (i = 0; i < mLen; i++)
		ch37x_hcd_dbg_data(ch37x_hcd, "0x%02x,", mBuf[i]);

	ch37x_hcd_dbg_data(ch37x_hcd, "\n");
}

#ifdef CONFIG_DEBUG_FS
#define SPI_REGS_BUFSIZE 1024

static ssize_t ch37x_write_proc_data(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	struct ch37x_hcd_data *ch37x_hcd = NULL;
	char *buf;
	ssize_t ret;

	ch37x_hcd = file->private_data;

	buf = kzalloc(32, GFP_KERNEL);
	if (!buf)
		return 0;

	ret = copy_from_user(buf, buffer, count);
	if (ret) {
		return ret;
	}

	if ((strstr(buf, "debug") != NULL) || (strstr(buf, "DEBUG") != NULL)) {
		atomic_set(&ch37x_hcd->debug_flag, 1);
		kfree(buf);
		printk("%s:open debug\n", __func__);
		return count;
	} else if ((strstr(buf, "data") != NULL) || (strstr(buf, "DATA") != NULL)) {
		atomic_set(&ch37x_hcd->debug_flag, 2);
		printk("%s:open data debug\n", __func__);
	} else if ((strstr(buf, "stop") != NULL) || (strstr(buf, "STOP") != NULL)) {
		atomic_set(&ch37x_hcd->debug_flag, 0);
		printk("%s:close debug\n", __func__);
	} else {
		printk("%s:invalid command\n", __func__);
		kfree(buf);
		return count;
	}
	kfree(buf);
	return count;
}

static ssize_t ch37x_show_regs(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	struct usb_hcd *hcd = NULL;
	struct ch37x_hcd_data *ch37x_hcd = NULL;
	char *buf;
	u32 len = 0;
	ssize_t ret;

	ch37x_hcd = file->private_data;
	hcd = ch37x_to_hcd(ch37x_hcd);

	buf = kzalloc(SPI_REGS_BUFSIZE, GFP_KERNEL);
	if (!buf)
		return 0;

	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len, "ch37x usb registers:\n");
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len, "=================================\n");
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len, "REG_HUB_CTRL: \t\t0x%08x\n",
			Read374Byte(hcd, REG_HUB_CTRL));
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len, "REG_SYS_INFO: \t0x%08x\n", Read374Byte(hcd, REG_SYS_INFO));
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len, "REG_SYS_CTRL: \t\t0x%08x\n",
			Read374Byte(hcd, REG_SYS_CTRL));
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len, "REG_USB_SETUP: \t\t0x%08x\n",
			Read374Byte(hcd, REG_USB_SETUP));
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len, "REG_INTER_EN: \t0x%08x\n", Read374Byte(hcd, REG_INTER_EN));
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len, "REG_USB_ADDR: \t0x%08x\n", Read374Byte(hcd, REG_USB_ADDR));
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len, "REG_INTER_FLAG: \t\t0x%08x\n",
			Read374Byte(hcd, REG_INTER_FLAG));
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len, "REG_USB_STATUS: \t\t0x%08x\n",
			Read374Byte(hcd, REG_USB_STATUS));
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len, "REG_USB_LENGTH: \t\t0x%08x\n",
			Read374Byte(hcd, REG_USB_LENGTH));
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len, "REG_USB_ENDP0: \t\t0x%08x\n",
			Read374Byte(hcd, REG_USB_ENDP0));
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len, "REG_USB_ENDP1: \t\t0x%08x\n",
			Read374Byte(hcd, REG_USB_ENDP1));
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len, "REG_USB_H_PID: \t\t0x%08x\n",
			Read374Byte(hcd, REG_USB_H_PID));
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len, "REG_USB_ENDP2: \t0x%08x\n",
			Read374Byte(hcd, REG_USB_ENDP2));
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len, "REG_USB_H_CTRL: \t0x%08x\n",
			Read374Byte(hcd, REG_USB_H_CTRL));
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len, "=================================\n");

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, len);

	kfree(buf);
	return ret;
}

static const struct file_operations ch37x_regs_ops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = ch37x_show_regs,
	.write = ch37x_write_proc_data,
	.llseek = default_llseek,
};

static int ch37x_debugfs_init(struct usb_hcd *hcd)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);

	ch37x_hcd->debugfs = debugfs_create_dir("ch37x", NULL);
	if (!ch37x_hcd->debugfs)
		return -ENOMEM;

	ch37x_hcd->debugfile = debugfs_create_file("registers", S_IFREG | S_IRUGO, ch37x_hcd->debugfs,
						   (void *)ch37x_hcd, &ch37x_regs_ops);
	if (!ch37x_hcd->debugfile)
		return -ENOMEM;

	return 0;
}

static int ch37x_debugfs_exit(struct usb_hcd *hcd)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);

	debugfs_remove(ch37x_hcd->debugfile);
	debugfs_remove_recursive(ch37x_hcd->debugfs);
	return 0;
}

#else
static inline int ch37x_debugfs_init(struct usb_hcd *hcd)
{
	return 0;
}

#endif /* CONFIG_DEBUG_FS */

static void host_set_bus_free(struct usb_hcd *hcd)
{
	Write374Byte(hcd, REG_USB_SETUP, BIT_SETP_HOST_MODE | BIT_SETP_AUTO_SOF); // USB host mode, enable SOF
	Write374Byte(hcd, REG_HUB_SETUP, 0x00); // clear BIT_HUB_DISABLE, allow internal ROOT-HUB
	Write374Byte(hcd, REG_HUB_CTRL, 0x00);	// clear ROOT-HUB message
}

static void set_host_usb_addr(struct usb_hcd *hcd, unsigned char addr)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	Write374Byte(hcd, REG_USB_ADDR, addr);
	if (addr)
		ch37x_hcd->addr = addr;
	ch37x_hcd_dbg(ch37x_hcd, "%s:line[%d]\n", __func__, __LINE__);
}

static void host_reset_bus(struct usb_hcd *hcd, int index)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	u8 reghub = 0;
	u8 bitpolar = 0;
	u8 bitreset = 0;
	u8 i = 0;

	set_host_usb_addr(hcd, 0x00);
	Write374Byte(hcd, REG_USB_H_CTRL, 0x00);

	if (index == 0) {
		reghub = REG_HUB_SETUP;
		bitpolar = BIT_HUB0_POLAR;
		bitreset = BIT_HUB0_RESET;
	} else if (index == 1) {
		reghub = REG_HUB_CTRL;
		bitpolar = BIT_HUB1_POLAR;
		bitreset = BIT_HUB1_RESET;
	} else if (index == 2) {
		reghub = REG_HUB_CTRL;
		bitpolar = BIT_HUB2_POLAR;
		bitreset = BIT_HUB2_RESET;
	}

	Write374Byte(hcd, reghub, (Read374Byte(hcd, reghub) & ~bitpolar) | bitreset);
	mdelay(15);
	Write374Byte(hcd, reghub, Read374Byte(hcd, reghub) & ~bitreset);
	mdelay(1);
	Write374Byte(hcd, REG_INTER_FLAG, BIT_IF_USB_PAUSE | BIT_IF_DEV_DETECT | BIT_IF_USB_SUSPEND);

	/* check attach & speed & en */
	for (i = 0; i < 100; i++) {			 // wait USB device re-connect when reset
		if (change_hub_port_speed(hcd, index)) { // enable ROOT-HUB port
			break;				 // already connected stably
		}
		mdelay(1);
	}
	mdelay(30);

	ch37x_hcd_dbg(ch37x_hcd, "%s:line[%d]\n", __func__, __LINE__);
}

static void check_hub_port_speed(struct usb_hcd *hcd, int index)
{
	u8 reghub = 0;
	u8 regdxin = 0;
	u8 bitpolar = 0;
	u8 bitdxin = 0;
	u8 bitattach = 0;
	u8 biten = 0;
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);

	ch37x_hcd_dbg(ch37x_hcd, "%s: index=%d, line[%d]\n", __func__, index, __LINE__);

	if (index == 0) {
		reghub = REG_HUB_SETUP;
		regdxin = REG_INTER_FLAG;
		bitpolar = BIT_HUB0_POLAR;
		bitdxin = BIT_HUB0_DX_IN;
		bitattach = BIT_HUB0_ATTACH;
		biten = BIT_HUB0_EN;
	} else if (index == 1) {
		reghub = REG_HUB_CTRL;
		regdxin = REG_HUB_SETUP;
		bitpolar = BIT_HUB1_POLAR;
		bitdxin = BIT_HUB1_DX_IN;
		bitattach = BIT_HUB1_ATTACH;
		biten = BIT_HUB1_EN;
	} else if (index == 2) {
		reghub = REG_HUB_CTRL;
		regdxin = REG_HUB_SETUP;
		bitpolar = BIT_HUB2_POLAR;
		bitdxin = BIT_HUB2_DX_IN;
		bitattach = BIT_HUB2_ATTACH;
		biten = BIT_HUB2_EN;
	}

	/* If the speed does not match, set the polarity, because the default is full speed enumeration, if there is a mismatch, it is low speed */
	if (!(Read374Byte(hcd, regdxin) & bitdxin)) {
		ch37x_hcd->ports[index].port_status.wPortStatus |= USB_PORT_STAT_LOW_SPEED;
		Write374Byte(hcd, reghub, Read374Byte(hcd, reghub) ^ bitpolar);
		ch37x_hcd_dbg(ch37x_hcd, "%s: line[%d], lowspeed\n", __func__, __LINE__);
	} else {
		ch37x_hcd->ports[index].port_status.wPortStatus &= ~USB_PORT_STAT_LOW_SPEED;
		ch37x_hcd_dbg(ch37x_hcd, "%s: line[%d], fullspeed\n", __func__, __LINE__);
	}
}

static bool change_hub_port_speed(struct usb_hcd *hcd, int index)
{
	u8 reghub = 0;
	u8 regdxin = 0;
	u8 bitpolar = 0;
	u8 bitdxin = 0;
	u8 bitattach = 0;
	u8 biten = 0;
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);

	ch37x_hcd_dbg(ch37x_hcd, "%s: index=%d, line[%d]\n", __func__, index, __LINE__);

	if (index == 0) {
		reghub = REG_HUB_SETUP;
		regdxin = REG_INTER_FLAG;
		bitpolar = BIT_HUB0_POLAR;
		bitdxin = BIT_HUB0_DX_IN;
		bitattach = BIT_HUB0_ATTACH;
		biten = BIT_HUB0_EN;
	} else if (index == 1) {
		reghub = REG_HUB_CTRL;
		regdxin = REG_HUB_SETUP;
		bitpolar = BIT_HUB1_POLAR;
		bitdxin = BIT_HUB1_DX_IN;
		bitattach = BIT_HUB1_ATTACH;
		biten = BIT_HUB1_EN;
	} else if (index == 2) {
		reghub = REG_HUB_CTRL;
		regdxin = REG_HUB_SETUP;
		bitpolar = BIT_HUB2_POLAR;
		bitdxin = BIT_HUB2_DX_IN;
		bitattach = BIT_HUB2_ATTACH;
		biten = BIT_HUB2_EN;
	}

	if (Read374Byte(hcd, reghub) & bitattach) {	   // device connected
		if (!(Read374Byte(hcd, reghub) & biten)) { // not yet enabled
			/* If the speed does not match, set the polarity, because the default is full speed enumeration, if there is a mismatch, it is low speed */
			if (!(Read374Byte(hcd, regdxin) & bitdxin)) {
				ch37x_hcd->ports[index].port_status.wPortStatus |= USB_PORT_STAT_LOW_SPEED;
				Write374Byte(hcd, reghub, Read374Byte(hcd, reghub) ^ bitpolar);
				ch37x_hcd_dbg(ch37x_hcd, "%s: line[%d], lowspeed\n", __func__, __LINE__);
			} else {
				ch37x_hcd->ports[index].port_status.wPortStatus &= ~USB_PORT_STAT_LOW_SPEED;
				ch37x_hcd_dbg(ch37x_hcd, "%s: line[%d], fullspeed\n", __func__, __LINE__);
			}
		}
		Write374Byte(hcd, reghub, Read374Byte(hcd, reghub) | biten);
		ch37x_hcd->ports[index].usbsta = USB_NOADDR;

		return true;
	}
	return false;
}

static void ch37x_manage_address(struct usb_hcd *hcd, struct usb_device *dev)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	int i;
	bool newdev = true;

	/* only handle nonzero device address */
	if (dev->devnum != 0) {
		for (i = 0; i < NUM_PORTS; i++) {
			if (ch37x_hcd->ports[i].bus_addr == dev->devnum) {
				newdev = false;
				break;
			}
		}
		if (newdev) {
			for (i = 0; i < NUM_PORTS; i++) {
				if (ch37x_hcd->ports[i].usbsta == USB_NOADDR) {
					ch37x_hcd->ports[i].usbsta = USB_ADDR;
					ch37x_hcd->ports[i].bus_addr = ch37x_hcd->ports[i].bus_addr_backup =
						dev->devnum;
					ch37x_hcd_dbg(ch37x_hcd, "%s:line[%d], new device connect to ports[%d]\n",
						      __func__, __LINE__, i);
					break;
				}
			}
		}
	}
}

static void ch37x_set_address(struct usb_hcd *hcd, struct usb_device *dev, int epnum)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);

	ch37x_hcd_dbg(ch37x_hcd, "%s:line[%d], devnum:%d, %d\n", __func__, __LINE__, dev->devnum, ch37x_hcd->olddevnum);
	if (dev->devnum != ch37x_hcd->olddevnum) {
		ch37x_hcd_dbg(ch37x_hcd, "%s:line[%d], devnum:%d, %d\n", __func__, __LINE__, dev->devnum,
			      ch37x_hcd->olddevnum);
		ch37x_hcd->olddevnum = dev->devnum;
	}
	set_host_usb_addr(hcd, dev->devnum);
}

static void ch37x_set_speed(struct usb_hcd *hcd, struct usb_device *dev)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	u8 reg_usbsetup;

	reg_usbsetup = Read374Byte(hcd, REG_USB_SETUP);
	if (dev->speed == USB_SPEED_LOW) {
		ch37x_hcd_dbg(ch37x_hcd, "%s:line[%d] lowspeed\n", __func__, __LINE__);
		Write374Byte(hcd, REG_USB_SETUP,
			     (reg_usbsetup & BIT_SETP_RAM_MODE) | BIT_SETP_HOST_MODE | BIT_SETP_AUTO_SOF |
				     BIT_SETP_LOW_SPEED);
	} else {
		ch37x_hcd_dbg(ch37x_hcd, "%s:line[%d] fullspeed\n", __func__, __LINE__);
		Write374Byte(hcd, REG_USB_SETUP,
			     (reg_usbsetup & BIT_SETP_RAM_MODE) | BIT_SETP_HOST_MODE | BIT_SETP_AUTO_SOF); // full-speed
		Write374Byte(hcd, REG_HUB_SETUP, Read374Byte(hcd, REG_HUB_SETUP) & ~BIT_HUB_PRE_PID); // diable PRE PID
	}
}

static void init_ch37x_host(struct usb_hcd *hcd)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	unsigned char val;

	ch37x_hcd_dbg(ch37x_hcd, "%s:line[%d]\n", __func__, __LINE__);

	Write374Byte(hcd, REG_SYS_CTRL, 0x08);
	msleep(50);
	Write374Byte(hcd, REG_SYS_CTRL, 0x00);
	msleep(50);
	Write374Byte(hcd, REG_USB_SETUP, 0x00);
	set_host_usb_addr(hcd, 0x00);
	Write374Byte(hcd, REG_USB_H_CTRL, 0x00);
	Write374Byte(hcd, REG_INTER_FLAG, BIT_IF_USB_PAUSE | BIT_IF_INTER_FLAG);
	/* modified on 20220817 */
	Write374Byte(hcd, REG_INTER_EN, 0x00);
	Write374Byte(hcd, REG_INTER_EN, BIT_IE_TRANSFER | BIT_IE_DEV_DETECT);
	Write374Byte(hcd, REG_SYS_CTRL, BIT_CTRL_OE_POLAR);
	host_set_bus_free(hcd);

	pr_info("%s: REG_INTER_EN:0x%x, REG_SYS_CTRL:0x%x, REG_USB_SETUP:0x%x, REG_HUB_SETUP:0x%x\n", __func__,
		Read374Byte(hcd, REG_INTER_EN), Read374Byte(hcd, REG_SYS_CTRL), Read374Byte(hcd, REG_USB_SETUP),
		Read374Byte(hcd, REG_HUB_SETUP));

	Write374Byte(hcd, REG_USB_ADDR, 0x55);
	val = Read374Byte(hcd, REG_USB_ADDR);
	if (val != 0x55)
		pr_err("SPI test error, write 0x55, read 0x%2x.\n", val);

	Write374Byte(hcd, REG_USB_ADDR, 0xaa);
	val = Read374Byte(hcd, REG_USB_ADDR);
	if (val != 0xaa)
		pr_err("SPI test error, write 0xaa, read 0x%2x.\n", val);
}

static void disable_hub_port(struct usb_hcd *hcd, int index)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);

	ch37x_hcd_dbg(ch37x_hcd, "%s:line[%d]\n", __func__, __LINE__);

	ch37x_hcd->ports[index].port_status.wPortStatus &=
		~(USB_PORT_STAT_CONNECTION | USB_PORT_STAT_ENABLE | USB_PORT_STAT_LOW_SPEED);
	ch37x_hcd->ports[index].port_status.wPortChange |= USB_PORT_STAT_C_CONNECTION | USB_PORT_STAT_C_ENABLE;
	ch37x_hcd->ports[index].wPortTrigger = RH_PORT_STATUS_CHANGED;
	/* check if the current urb related to this device */
	if (ch37x_hcd->curr_urb != NULL) {
		/* modfied on 202307 */
		if (ch37x_hcd->curr_urb->dev != NULL) {
			if (ch37x_hcd->ports[index].bus_addr != 0x00) {
				if (ch37x_hcd->ports[index].bus_addr == ch37x_hcd->curr_urb->dev->devnum) {
					ch37x_hcd->curr_urb = NULL;
					ch37x_hcd->curr_len = 0;
					ch37x_hcd_dbg(ch37x_hcd, "%s:line[%d] clear current urb here\n", __func__,
						      __LINE__);
				}
			}
		} else {
			ch37x_hcd->curr_urb = NULL;
			ch37x_hcd->curr_len = 0;
		}
	}
	ch37x_hcd->ports[index].bus_addr = 0x00;
	ch37x_hcd->ports[index].usbsta = USB_UNLINK;
	if (index == 0) {
		/* clear control data about HUB0, actually no need to clear it */
		Write374Byte(hcd, REG_HUB_SETUP, Read374Byte(hcd, REG_HUB_SETUP) & 0xF0);
	} else if (index == 1) {
		/* clear control data about HUB1, actually no need to clear it */
		Write374Byte(hcd, REG_HUB_CTRL, Read374Byte(hcd, REG_HUB_CTRL) & 0xF0);
	} else if (index == 2) {
		/* clear control data about HUB2, actually no need to clear it */
		Write374Byte(hcd, REG_HUB_CTRL, Read374Byte(hcd, REG_HUB_CTRL) & 0x0F);
	}
}

static int ch37x_ctrl_setup(struct usb_hcd *hcd, struct urb *urb)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);

	ch37x_hcd_dbg(ch37x_hcd, "%s:line[%d], first:0x%x 0x%x\n", __func__, __LINE__,
		      *((unsigned char *)urb->setup_packet), *((unsigned char *)urb->setup_packet + 1));

	Write374Block(hcd, RAM_HOST_TRAN, 8, urb->setup_packet);
	Write374Byte(hcd, REG_USB_LENGTH, 8);
	return CH374_TOKEN_SETUP;
}

static int ch37x_transfer_in(struct usb_hcd *hcd, struct urb *urb)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	int epnum = usb_pipeendpoint(urb->pipe);

	ch37x_hcd_dbg(ch37x_hcd, "%s:line[%d],epnum=%d\n", __func__, __LINE__, epnum);

	ch37x_hcd->curr_len = 0;

	return CH374_TOKEN_BULK_IN(epnum);
}

static int ch37x_transfer_out(struct usb_hcd *hcd, struct urb *urb)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	int epnum = usb_pipeendpoint(urb->pipe);
	u32 max_packet;
	void *src;

	src = urb->transfer_buffer + urb->actual_length;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
	max_packet = usb_maxpacket(urb->dev, urb->pipe);
#else
	max_packet = usb_maxpacket(urb->dev, urb->pipe, 1);
#endif
	if (max_packet > MAX_FIFO_SIZE) {
		ch37x_hcd->urb_done = -EMSGSIZE;
		return -EMSGSIZE;
	}
	ch37x_hcd->curr_len = min((urb->transfer_buffer_length - urb->actual_length), max_packet);

	Write374Block(hcd, RAM_HOST_TRAN, ch37x_hcd->curr_len, src);
	Write374Byte(hcd, REG_USB_LENGTH, ch37x_hcd->curr_len);

	ch37x_hcd_dbg(ch37x_hcd, "%s:line[%d]\n", __func__, __LINE__);

	return CH374_TOKEN_BULK_OUT(epnum);
}

/*
 * Issue the next host-transfer command.
 * Caller must NOT hold HCD spinlock.
 */
static void ch37x_next_transfer(struct usb_hcd *hcd, int epnum)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	struct urb *urb = ch37x_hcd->curr_urb;
	struct ch37x_ep *ch37x_ep;
	int cmd = -EINVAL;
	int index = -1;

	if (!urb) {
		ch37x_hcd_dbg(ch37x_hcd, "%s:URB NULL\n", __func__);
		return; /* nothing to do */
	}

	if (urb->ep->hcpriv) {
		ch37x_ep = urb->ep->hcpriv;
		ch37x_hcd_dbg(ch37x_hcd, "ch37x_next_transfer:full %d\n", ch37x_ep->pkt_state);
	} else {
		ch37x_hcd_dbg(ch37x_hcd, "%s:NULL\n", __func__);
		return;
	}

	index = ch37x_get_port_from_addr(ch37x_hcd, urb->dev->devnum);

	switch (ch37x_ep->pkt_state) {
	case PKT_STATE_SETUP:
		cmd = ch37x_ctrl_setup(hcd, urb);
		ch37x_hcd->pid_state = CH374_DEF_USB_PID_SETUP;
		break;
	case PKT_STATE_TRANSFER:
		if (usb_urb_dir_in(urb)) {
			cmd = ch37x_transfer_in(hcd, urb);
			ch37x_hcd->pid_state = CH374_DEF_USB_PID_IN;
		} else {
			cmd = ch37x_transfer_out(hcd, urb);
			ch37x_hcd->pid_state = CH374_DEF_USB_PID_OUT;
		}
		break;
	default:
		break;
	}
	if (cmd < 0) {
		ch37x_hcd_dbg(ch37x_hcd, "%s:cmd < 0\n", __func__);
		return;
	}

	Write374Byte(hcd, REG_USB_H_PID, cmd);
	if (ch37x_hcd->pid_state == CH374_DEF_USB_PID_SETUP && ch37x_ep->curr_pkt == PKT_STATE_SETUP) {
		ch37x_hcd_dbg(ch37x_hcd, "%s: index:%d, line[%d]\n", __func__, index, __LINE__);
		Write374Byte(hcd, REG_USB_H_CTRL, BIT_HOST_START);
	} else if (ch37x_ep->curr_pkt == PKT_STATE_SETUP) {
		ch37x_hcd_dbg(ch37x_hcd, "%s: index:%d, line[%d], controltog: %d\n", __func__, index, __LINE__,
			      ch37x_hcd->ports[index].controltog);
		Write374Byte(hcd, REG_USB_H_CTRL,
			     (ch37x_hcd->ports[index].controltog ?
				      (BIT_HOST_START | BIT_HOST_TRAN_TOG | BIT_HOST_RECV_TOG) :
				      BIT_HOST_START));
	} else if (ch37x_ep->curr_pkt == PKT_STATE_TRANSFER) {
		if (ch37x_hcd->pid_state == CH374_DEF_USB_PID_IN) {
			ch37x_hcd_dbg(ch37x_hcd, "%s - transferintog:%d, epnum:%d!!!!!\n", __func__,
				      ch37x_hcd->ports[index].transferintog[epnum], epnum);
			Write374Byte(hcd, REG_USB_H_CTRL,
				     (ch37x_hcd->ports[index].transferintog[epnum] ?
					      (BIT_HOST_START | BIT_HOST_TRAN_TOG | BIT_HOST_RECV_TOG) :
					      BIT_HOST_START));
		} else {
			ch37x_hcd_dbg(ch37x_hcd, "%s - transferouttog:%d, epnum:%d!!!!!\n", __func__,
				      ch37x_hcd->ports[index].transferouttog[epnum], epnum);
			Write374Byte(hcd, REG_USB_H_CTRL,
				     (ch37x_hcd->ports[index].transferouttog[epnum] ?
					      (BIT_HOST_START | BIT_HOST_TRAN_TOG | BIT_HOST_RECV_TOG) :
					      BIT_HOST_START));
		}
	}
	ch37x_hcd_dbg(ch37x_hcd, "%s: index:%d, line[%d], end\n", __func__, index, __LINE__);
}

/*
 * Find the next URB to process and start its execution.
 */
static int ch37x_select_and_start_urb(struct usb_hcd *hcd)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	struct urb *urb, *curr_urb = NULL;
	struct ch37x_ep *ch37x_ep;
	int epnum;
	struct usb_host_endpoint *ep;
	struct list_head *pos;
	unsigned long flags;
	int index;
	u8 setup_packet_suspend[] = { 0x00, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };

	ch37x_hcd_dbg(ch37x_hcd, "%s:line[%d] begin\n", __func__, __LINE__);

	spin_lock_irqsave(&ch37x_hcd->lock, flags);
	if (list_empty(&ch37x_hcd->ep_list)) {
		ch37x_hcd_dbg(ch37x_hcd, "%s:line[%d] list empty now\n", __func__, __LINE__);
		spin_unlock_irqrestore(&ch37x_hcd->lock, flags);
		return 0;
	}

	list_for_each(pos, &ch37x_hcd->ep_list) {
		urb = NULL;
		ch37x_ep = container_of(pos, struct ch37x_ep, ep_list);
		ep = ch37x_ep->ep;

		if (list_empty(&ep->urb_list))
			continue;
		urb = list_first_entry(&ep->urb_list, struct urb, urb_list);
		if (urb->unlinked) {
			ch37x_hcd_dbg(ch37x_hcd, "%s: URB %p unlinked=%d", __func__, urb, urb->unlinked);
			ch37x_hcd->curr_urb = urb;
			/* modified on 202307 */
			ch37x_hcd->urb_done = 1;
			spin_unlock_irqrestore(&ch37x_hcd->lock, flags);
			return 1;
		}
		if (urb->dev->devnum != 0) {
			index = ch37x_get_port_from_addr_backup(ch37x_hcd, urb->dev->devnum);
			if (index != NUM_PORTS) {
				if (ch37x_hcd->ports[index].usbsta == USB_UNLINK) {
					ch37x_hcd_dbg(ch37x_hcd, "%s: URB-ep %d canceled cause disconnect", __func__,
						      urb->dev->devnum);
					ch37x_hcd->curr_urb = urb;
					ch37x_hcd->urb_done = -ESHUTDOWN;
					spin_unlock_irqrestore(&ch37x_hcd->lock, flags);
					return 1;
				}
			}
		}
		/*move current ep to tail*/
		list_move_tail(pos, &ch37x_hcd->ep_list);
		curr_urb = urb;
		goto done;
	}
done:
	atomic_set(&ch37x_hcd->urb_process, 0);
	if (!curr_urb) {
		spin_unlock_irqrestore(&ch37x_hcd->lock, flags);
		ch37x_hcd_dbg(ch37x_hcd, "%s: curr_urb null here!\n ", __func__);
		return 0;
	}
	urb = ch37x_hcd->curr_urb = curr_urb;
	epnum = usb_endpoint_num(&curr_urb->ep->desc);
	ch37x_manage_address(hcd, urb->dev);
	index = ch37x_get_port_from_addr(ch37x_hcd, urb->dev->devnum);
	if (ch37x_ep->retransmit) {
		ch37x_ep->retransmit = 0;
		ch37x_hcd_dbg(ch37x_hcd, "Retransmit here!\n");
	} else {
		if (usb_endpoint_xfer_control(&ep->desc)) {
			/*
			 * See USB 2.0 spec section 8.6.1
			 * Initialization via SETUP Token:
			 */
			usb_settoggle(urb->dev, epnum, 0, 1);
			usb_settoggle(urb->dev, epnum, 1, 1);
			ch37x_ep->pkt_state = PKT_STATE_SETUP;
			ch37x_ep->curr_pkt = PKT_STATE_SETUP;
			ch37x_hcd->ports[index].controltog = true;

			// added on 20231221
			if (memcmp(setup_packet_suspend, urb->setup_packet, sizeof(setup_packet_suspend)) == 0) {
				ch37x_hcd_dbg(ch37x_hcd, "Suspend Setup packet captured for ports[%d].\n", index);
				ch37x_hcd->ports[index].todosuspend = true;
			}

			ch37x_hcd_dbg(ch37x_hcd, "New Setup packet! --> ports[%d].controltog: %d\n", index,
				      ch37x_hcd->ports[index].controltog);
			ch37x_hcd_dbg(
				ch37x_hcd, "%s:line[%d], first:0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n", __func__,
				__LINE__, *((unsigned char *)urb->setup_packet),
				*((unsigned char *)urb->setup_packet + 1), *((unsigned char *)urb->setup_packet + 2),
				*((unsigned char *)urb->setup_packet + 3), *((unsigned char *)urb->setup_packet + 4),
				*((unsigned char *)urb->setup_packet + 5), *((unsigned char *)urb->setup_packet + 6),
				*((unsigned char *)urb->setup_packet + 7));
		} else {
			ch37x_ep->pkt_state = PKT_STATE_TRANSFER;
			ch37x_ep->curr_pkt = PKT_STATE_TRANSFER;
		}
	}
	spin_unlock_irqrestore(&ch37x_hcd->lock, flags);

	/*
	 * set address
	 */
	ch37x_set_address(hcd, urb->dev, epnum);
	ch37x_set_speed(hcd, urb->dev);
	ch37x_hcd_dbg(ch37x_hcd, "%s:line[%d], index=%d, epnum=%d\n", __func__, __LINE__, index, epnum);
	ch37x_next_transfer(hcd, epnum);
	return 1;
}

/*
 * Caller must NOT hold HCD spinlock.
 */
static int ch37x_recv_data_available(struct usb_hcd *hcd, struct urb *curr_urb)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	struct urb *urb = curr_urb;
	size_t remaining, transfer_size;
	u32 max_packet;
	u8 rcvLen;

	rcvLen = Read374Byte(hcd, REG_USB_LENGTH);

	if (rcvLen > MAX_FIFO_SIZE) {
		rcvLen = MAX_FIFO_SIZE;
	}

	if (urb->actual_length >= urb->transfer_buffer_length)
		remaining = 0;
	else
		remaining = urb->transfer_buffer_length - urb->actual_length;

	ch37x_hcd_dbg(ch37x_hcd, "%s:remaining=%d, rcvLen=0x%x\n", __func__, remaining, rcvLen);
	ch37x_hcd_dbg(ch37x_hcd, "%s:transfer_buffer_length=%d, actual_length=0x%x\n", __func__,
		      urb->transfer_buffer_length, urb->actual_length);

	transfer_size = rcvLen;
	if (transfer_size > remaining) {
		transfer_size = remaining;
	}
	if (transfer_size > 0) {
		void *dst = urb->transfer_buffer + urb->actual_length;

		Read374Block(hcd, RAM_HOST_RECV, transfer_size, dst);

		urb->actual_length += transfer_size;
		ch37x_hcd->curr_len = transfer_size;
	}

	if (urb->actual_length >= urb->transfer_buffer_length)
		return 1; /* read is complete, so we're done */
			  /*
	 * USB 2.0 Section 5.3.2 Pipes: packets must be full size
	 * except for last one.
	 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
	max_packet = usb_maxpacket(urb->dev, urb->pipe);
#else
	max_packet = usb_maxpacket(urb->dev, urb->pipe, 0);
#endif
	if (max_packet > MAX_FIFO_SIZE) {
		pr_err("%s: packet-size of %u too big for ch37x (limit is %u bytes)", __func__, max_packet,
		       MAX_FIFO_SIZE);
		return -EINVAL;
	}

	if (rcvLen == 0 || (rcvLen & (max_packet - 1)))
		return 1;
	else
		return 0;
}

static int ch37x_urb_done(struct usb_hcd *hcd)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	unsigned long flags;
	struct urb *urb;
	int status;
	int oldstatus;

	oldstatus = status = ch37x_hcd->urb_done;
	ch37x_hcd_dbg(ch37x_hcd, "%s start:line[%d], status=%d\n", __func__, __LINE__, status);
	ch37x_hcd->urb_done = 0;
	ch37x_hcd->isLastSetup = false;
	if (status > 0)
		status = 0;

	urb = ch37x_hcd->curr_urb;
	if (urb) {
		ch37x_hcd_dbg(ch37x_hcd, "%s line[%d], unlink here!\n", __func__, __LINE__);
		ch37x_hcd->curr_urb = NULL;
		if (oldstatus != 2) {
			spin_lock_irqsave(&ch37x_hcd->lock, flags);
			usb_hcd_unlink_urb_from_ep(hcd, urb);
			spin_unlock_irqrestore(&ch37x_hcd->lock, flags);
			/* must be called without the HCD spinlock: */
			usb_hcd_giveback_urb(hcd, urb, status);
		} else
			ch37x_hcd_dbg(ch37x_hcd, "%s line[%d], unlinked in check_unlink\n", __func__, __LINE__);
	}
	ch37x_hcd_dbg(ch37x_hcd, "%s end:line[%d]\n", __func__, __LINE__);
	return 1;
}

static void ch37x_slow_retransmit(struct usb_hcd *hcd)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	struct urb *urb = ch37x_hcd->curr_urb;
	struct ch37x_ep *ch37x_ep;

	ch37x_hcd_dbg(ch37x_hcd, "%s:line[%d] Slow retransmit\n", __func__, __LINE__);
	ch37x_ep = urb->ep->hcpriv;
	ch37x_ep->retransmit = 1;
	ch37x_hcd->curr_urb = NULL;
}

static void ch37x_handle_error(struct usb_hcd *hcd, u8 result)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	u8 result_code = result & BIT_STAT_DEV_RESP;
	struct urb *urb = ch37x_hcd->curr_urb;
	//struct ch37x_ep *ch37x_ep = urb->ep->hcpriv;
	struct ch37x_ep *ch37x_ep = NULL;
	bool tmp_tog = false;
	int epnum;
	int dirin;
	int index = -1;

	if (!urb) {
		ch37x_hcd_dbg(ch37x_hcd, "%s: 0x%p\n ", __func__, urb);
		return;
	}
	if (!urb->ep) {
		ch37x_hcd_dbg(ch37x_hcd, "%s:urb->ep=0x%p\n ", __func__, urb->ep);
		return;
	}
	if (!urb->ep->hcpriv) {
		ch37x_hcd_dbg(ch37x_hcd, "%s:urb->ep=0x%p\n ", __func__, urb->ep->hcpriv);
		/* added on 202307 */
		ch37x_hcd->curr_urb = NULL;
		return;
	}

	ch37x_ep = urb->ep->hcpriv;
	epnum = usb_endpoint_num(&urb->ep->desc);
	dirin = usb_endpoint_dir_in(&urb->ep->desc);

	index = ch37x_get_port_from_addr(ch37x_hcd, urb->dev->devnum);

	switch (result_code) {
	case DEF_USB_PID_ACK: /* this shouldn't happen */
		return;
	case DEF_USB_PID_STALL:
		ch37x_hcd->urb_done = -EPIPE;
		//		ch37x_urb_done(hcd);
		if (dirin)
			ch37x_hcd->ports[index].transferintog[epnum] = false;
		else
			ch37x_hcd->ports[index].transferouttog[epnum] = false;
		ch37x_hcd_dbg_data(ch37x_hcd, "%s: urb_done=%d\n", __func__, ch37x_hcd->urb_done);
		break;
	case DEF_USB_PID_NAK:
		/*
		 * Device wasn't ready for data or has no data
		 * available: retry the packet again.
		 */
		if (!ch37x_ep) {
			ch37x_hcd_dbg_data(ch37x_hcd, "%s: ch37x_ep=0x%p, naks=0x%x\n ", __func__, ch37x_ep,
					   ch37x_ep->naks);
			return;
		}

		if (ch37x_ep->curr_pkt == PKT_STATE_SETUP && ch37x_hcd->isLastSetup == true) {
			ch37x_hcd_dbg_data(ch37x_hcd, "%s: ----> DEF_USB_PID_NAK, get IN package again \n ", __func__);
			if (usb_urb_dir_in(urb))
				tmp_tog = false;
			else
				tmp_tog = true;
			ch37x_setpid(hcd, urb, tmp_tog);
			return;
		}

		if (ch37x_ep->naks++ < NAK_MAX_FAST_RETRANSMITS) {
			ch37x_next_transfer(hcd, epnum);
		} else
			ch37x_slow_retransmit(hcd);

		ch37x_hcd_dbg_data(ch37x_hcd, "%s:line[%d] DEF_USB_PID_NAK\n", __func__, __LINE__);

		break;
	default:
		ch37x_slow_retransmit(hcd);
		break;
	}
}

/*
 * Caller must NOT hold HCD spinlock.
 */
static int ch37x_transfer_out_done(struct usb_hcd *hcd, struct urb *urb)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);

	ch37x_hcd_dbg_data(ch37x_hcd, "%s:line[%d], urb=0x%p\n", __func__, __LINE__, urb);
	urb->actual_length += ch37x_hcd->curr_len;
	if (urb->actual_length < urb->transfer_buffer_length)
		return 0;
	if (urb->transfer_flags & URB_ZERO_PACKET) {
		/*
		 * Some hardware needs a zero-size packet at the end
		 * of a bulk-out transfer if the last transfer was a
		 * full-sized packet (i.e., such hardware use <
		 * max_packet as an indicator that the end of the
		 * packet has been reached).
		 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
		u32 max_packet = usb_maxpacket(urb->dev, urb->pipe);
#else
		u32 max_packet = usb_maxpacket(urb->dev, urb->pipe, 1);
#endif

		if (ch37x_hcd->curr_len == max_packet)
			return 0;
	}
	return 1;
}

static int ch37x_handle_pid_status(struct usb_hcd *hcd, int result)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	int retval = result & BIT_STAT_DEV_RESP;

	switch (ch37x_hcd->pid_state) {
	case CH374_DEF_USB_PID_SETUP:
	case CH374_DEF_USB_PID_OUT:
		if (retval == DEF_USB_PID_ACK)
			return result;
		else
			return (retval | 0x20);
		break;
	case CH374_DEF_USB_PID_IN:
		if (M_IS_HOST_IN_DATA(retval)) {
			if (result & BIT_STAT_TOG_MATCH) {
				/*keep high four bits*/
				result = ((result & BIT_HOST_PID_TOKEN) | DEF_USB_PID_ACK);
				return result;
			}

		} else if (retval == DEF_USB_PID_STALL || retval == DEF_USB_PID_NAK) {
			return (retval | 0x20);
		} else if (!M_IS_HOST_TIMEOUT(retval)) {
			return (retval | 0x20);
		}
		break;
	default:
		break;
	}
	return result;
}

static void ch37x_setpid(struct usb_hcd *hcd, struct urb *urb, bool tog)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	int epnum = usb_pipeendpoint(urb->pipe);

	Write374Byte(hcd, REG_USB_LENGTH, 0);
	usleep_range(500, 600);
	Write374Byte(hcd, REG_USB_H_PID, tog ? CH374_TOKEN_BULK_IN(epnum) : CH374_TOKEN_BULK_OUT(epnum));
	Write374Byte(hcd, REG_USB_H_CTRL, BIT_HOST_START | BIT_HOST_TRAN_TOG | BIT_HOST_RECV_TOG);
}

/*
 * Caller must NOT hold HCD spinlock.
 */
static void ch37x_host_transfer_done(struct usb_hcd *hcd, u8 status)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	struct urb *urb = ch37x_hcd->curr_urb;
	struct ch37x_ep *ch37x_ep;
	bool tog = false;
	u8 result_code;
	int urb_done = 0;
	int epnum;
	int index = -1;

	result_code = ch37x_handle_pid_status(hcd, status);
	result_code &= BIT_STAT_DEV_RESP;

	ch37x_hcd_dbg_data(ch37x_hcd, "%s:REG_USB_STATUS(0x%x), pid_status=0x%x\n", __func__, status, result_code);

	usleep_range(200, 240);

	if (!urb) {
		ch37x_hcd_dbg(ch37x_hcd, "%s: 0x%p\n ", __func__, urb);
		return;
	}
	if (!urb->ep) {
		ch37x_hcd_dbg(ch37x_hcd, "%s:urb->ep = 0x%p\n ", __func__, urb->ep);
		return;
	}
	if (!urb->ep->hcpriv) {
		ch37x_hcd_dbg(ch37x_hcd, "%s:urb->ep->hcpriv = 0x%p\n ", __func__, urb->ep->hcpriv);
		/* added on 20220630 */
		ch37x_hcd->curr_urb = NULL;
		return;
	}
	if (unlikely(result_code != DEF_USB_PID_ACK)) {
		if (ch37x_hcd->curr_urb)
			ch37x_handle_error(hcd, result_code);
		return;
	}

	if (urb->unlinked) {
		ch37x_hcd_dbg(ch37x_hcd, "%s:line[%d] urb unlinked here !!!!\n ", __func__, __LINE__);
		if (ch37x_hcd->curr_urb) {
			ch37x_hcd->curr_urb = NULL;
		}
		ch37x_hcd->isLastSetup = false;
		return;
	}
	ch37x_ep = urb->ep->hcpriv;
	ch37x_ep->naks = 0;
	epnum = usb_endpoint_num(&urb->ep->desc);

	index = ch37x_get_port_from_addr(ch37x_hcd, urb->dev->devnum);

	if (ch37x_ep->curr_pkt == PKT_STATE_SETUP && ch37x_hcd->isLastSetup == true) {
		ch37x_hcd->isLastSetup = false;
		urb_done = 1;
		ch37x_hcd->urb_done = urb_done;
		ch37x_hcd_dbg(ch37x_hcd, "%s:line[%d], done for setup\n", __func__, __LINE__);
		return;
	}

	switch (ch37x_ep->pkt_state) {
	case PKT_STATE_SETUP:
		if (urb->transfer_buffer_length > 0) {
			ch37x_ep->pkt_state = PKT_STATE_TRANSFER;
			ch37x_hcd_dbg_data(ch37x_hcd, "%s:line[%d], PKT_STATE_SETUP, len: %d\n", __func__, __LINE__,
					   urb->transfer_buffer_length);
		} else {
			ch37x_hcd_dbg_data(ch37x_hcd, "%s:line[%d], PKT_STATE_SETUP No data\n", __func__, __LINE__);
			if (usb_urb_dir_in(urb)) {
				tog = false;
			} else {
				/*urb dir is out and transfer length = 0*/
				tog = true;
			}
			ch37x_hcd->isLastSetup = true;
			urb_done = 1;
		}
		break;

	case PKT_STATE_TRANSFER:
		if (usb_urb_dir_in(urb)) {
			urb_done = ch37x_recv_data_available(hcd, urb);
			if (ch37x_ep->curr_pkt == PKT_STATE_SETUP) {
				ch37x_hcd->ports[index].controltog = ch37x_hcd->ports[index].controltog ? false : true;
				if (urb_done) {
					ch37x_hcd->isLastSetup = true;
					ch37x_hcd->ports[index].controltog = true;
					tog = false;
				}
			} else {
				ch37x_hcd->ports[index].transferintog[epnum] =
					ch37x_hcd->ports[index].transferintog[epnum] ? false : true;
				ch37x_hcd_dbg_data(ch37x_hcd, "%s - transferintog:%d, epnum:%d !!!!!\n", __func__,
						   ch37x_hcd->ports[index].transferintog[epnum], epnum);
			}
		} else {
			urb_done = ch37x_transfer_out_done(hcd, urb);
			if (ch37x_ep->curr_pkt == PKT_STATE_SETUP) {
				ch37x_hcd->ports[index].controltog = ch37x_hcd->ports[index].controltog ? false : true;
				if (urb_done) {
					ch37x_hcd->isLastSetup = true;
					ch37x_hcd->ports[index].controltog = true;
					tog = true;
				}
			} else {
				ch37x_hcd->ports[index].transferouttog[epnum] =
					ch37x_hcd->ports[index].transferouttog[epnum] ? false : true;
				ch37x_hcd_dbg_data(ch37x_hcd, "%s - transferouttog:%d, epnum:%d !!!!!\n", __func__,
						   ch37x_hcd->ports[index].transferouttog[epnum], epnum);
			}
		}
		break;
	default:
		break;
	}

	/* still need to wait pid status */
	if (urb_done && ch37x_ep->curr_pkt == PKT_STATE_SETUP && ch37x_hcd->isLastSetup == true) {
		urb_done = 0;
		if (tog) {
			ch37x_hcd->pid_state = CH374_DEF_USB_PID_IN;
		} else {
			ch37x_hcd->pid_state = CH374_DEF_USB_PID_OUT;
		}
		ch37x_setpid(hcd, urb, tog);
	} else if (urb_done) {
		ch37x_hcd->urb_done = urb_done;
		//		ch37x_urb_done(hcd);
	} else {
		ch37x_hcd_dbg_data(ch37x_hcd, "%s: go to next transfer\n", __func__);
		ch37x_next_transfer(hcd, epnum);
	}
	ch37x_hcd_dbg_data(ch37x_hcd, "%s:end, urb_done = %d\n", __func__, urb_done);
}

/*
 * Caller must NOT hold HCD spinlock.
 */
static void ch37x_detect_conn(struct usb_hcd *hcd, int index)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	int i = 0;
	u8 ureg = 0;
	u8 reghub = 0;
	u8 bitattach = 0;
	u8 usbstatusall = 0;

	if (index == 0) {
		reghub = REG_HUB_SETUP;
		bitattach = BIT_HUB0_ATTACH;
	} else if (index == 1) {
		reghub = REG_HUB_CTRL;
		bitattach = BIT_HUB1_ATTACH;
	} else if (index == 2) {
		reghub = REG_HUB_CTRL;
		bitattach = BIT_HUB2_ATTACH;
	}
	ch37x_hcd_dbg(ch37x_hcd, "%s: wPortStatus[%d]:0x%x, REG_HUB_SETUP(CTRL) = 0x%x start\n", __func__, index,
		      ch37x_hcd->ports[index].port_status.wPortStatus, Read374Byte(hcd, reghub));

	if ((Read374Byte(hcd, reghub) & bitattach) && (ch37x_hcd->ports[index].port_status.wPortStatus == 0)) {
		msleep(50);
		if ((Read374Byte(hcd, reghub) & bitattach) && (ch37x_hcd->ports[index].port_status.wPortStatus == 0)) {
			pr_info("%s:-------connect-------, index=%d\n", __func__, index);
			ch37x_hcd->ports[index].ch37x_usb_flag = 1;
			ch37x_hcd->ports[index].bus_addr_backup = 0x00;
			check_hub_port_speed(hcd, index);
			ch37x_hcd->ports[index].port_status.wPortStatus |= USB_PORT_STAT_CONNECTION;
			ch37x_hcd->ports[index].port_status.wPortChange |= USB_PORT_STAT_C_CONNECTION;
			ch37x_hcd->ports[index].wPortTrigger = RH_PORT_STATUS_CHANGED;
			msleep(50);
		}
	} else if (!(Read374Byte(hcd, reghub) & bitattach) && ch37x_hcd->ports[index].port_status.wPortStatus) {
		pr_info("%s:-------disconnect-------, index=%d\n", __func__, index);
		ch37x_hcd->ports[index].ch37x_usb_flag = 0;
		disable_hub_port(hcd, index);
		msleep(50);
		for (i = 0; i < NUM_PORTS; i++) {
			usbstatusall += ch37x_hcd->ports[index].ch37x_usb_flag;
		}
		// check if all devices disconnected
		if (usbstatusall == 0) {
			ch37x_hcd->curr_urb = NULL;
			ch37x_hcd->curr_len = 0;
		}
	} else {
		ureg = Read374Byte(hcd, REG_INTER_FLAG);
		pr_info("%s:-------ureg3 = 0x%x-------\n", __func__, ureg);
		return;
	}

	// only clear the device hotpluged
	ch37x_hcd->ports[index].bus_addr = 0x00;
	ch37x_hcd->ports[index].controltog = false;
	for (i = 0; i < 4; i++) {
		ch37x_hcd->ports[index].transferintog[i] = false;
		ch37x_hcd->ports[index].transferouttog[i] = false;
	}
	Write374Byte(hcd, REG_INTER_FLAG, BIT_IF_USB_PAUSE | BIT_IF_INTER_FLAG);
	//	pr_info("%s: wPortStatus=0x%x i=%d end\n",__func__, ch37x_hcd->port_status.wPortStatus, i);
}

static void ch37x_set_check_connect(struct usb_hcd *hcd)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	set_bit(CHECK_CONNECT, &ch37x_hcd->todo);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	queue_kthread_work(&ch37x_hcd->spi_worker, &ch37x_hcd->spi_work);
#else
	kthread_queue_work(&ch37x_hcd->spi_worker, &ch37x_hcd->spi_work);
#endif
	wake_up_process(ch37x_hcd->spi_thread);
	ch37x_hcd_dbg(ch37x_hcd, "%s:set bit CHECK_CONNECT\n", __func__);
}

/**
 * interrupt handler
 */
static irqreturn_t ch37x_irq_thread_handler(int irq, void *dev_id)
{
	struct usb_hcd *hcd = dev_id;
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	u8 intvalue;

	intvalue = Read374Byte(hcd, REG_INTER_FLAG);

	ch37x_hcd_dbg_data(ch37x_hcd, "line[%d], reg:0x%02x\n", __LINE__, intvalue);

	if ((intvalue & 0x03) == 0)
		return IRQ_HANDLED;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 14, 0))
	if (ch37x_hcd->spi_thread && ch37x_hcd->spi_thread->state != TASK_RUNNING) {
#else
	if (ch37x_hcd->spi_thread) {
#endif
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
		queue_kthread_work(&ch37x_hcd->spi_worker, &ch37x_hcd->spi_work);
#else
		kthread_queue_work(&ch37x_hcd->spi_worker, &ch37x_hcd->spi_work);
#endif
		wake_up_process(ch37x_hcd->spi_thread);
	}

	if (!test_and_set_bit(ENABLE_IRQ, &ch37x_hcd->todo)) {
		disable_irq_nosync(ch37x_hcd->irq);
	}

	return IRQ_HANDLED;
}

static irqreturn_t ch37x_irq_handler(int irq, void *dev_id)
{
	return IRQ_WAKE_THREAD;
}

/* return zero if no work was performed, 1 otherwise.  */
static int ch37x_handle_irqs(struct usb_hcd *hcd)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	u8 interrupt;
	u8 status;

	interrupt = Read374Byte(hcd, REG_INTER_FLAG);
	ch37x_hcd_dbg_data(ch37x_hcd, "%s:line[%d], interrupt: 0x%x, REG_H_PID(0x%x)\n", __func__, __LINE__, interrupt,
			   Read374Byte(hcd, REG_USB_H_PID));

	if ((interrupt & 0x03) == 0) {
		return 0;
	}
	if (interrupt & BIT_IF_DEV_DETECT) {
		/*
      	 * set 1 to clear BIT_IF_DEV_DETECT
         */
		Write374Byte(hcd, REG_INTER_FLAG, BIT_IF_DEV_DETECT);
		ch37x_set_check_connect(hcd);
	}

	if (interrupt & BIT_IF_TRANSFER) {
		Write374Byte(hcd, REG_INTER_FLAG, BIT_IF_USB_PAUSE | BIT_IF_TRANSFER);
		status = Read374Byte(hcd, REG_USB_STATUS);
		ch37x_host_transfer_done(hcd, status);
	}

	/*
	 * Now process interrupts that may affect HCD state
	 * other than the end-points:
	 */

	return 1;
}

static int ch37x_reset_hcd(struct usb_hcd *hcd)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	int index;

	init_ch37x_host(hcd);
	if (ch37x_hcd->addr)
		set_host_usb_addr(hcd, ch37x_hcd->addr);

	for (index = 0; index < NUM_PORTS; index++)
		ch37x_detect_conn(hcd, index);

	pr_info("%s:line[%d]\n", __func__, __LINE__);

	return 1;
}

static int ch37x_check_unlink(struct usb_hcd *hcd)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	struct list_head *pos, *upos, *next_upos;
	struct ch37x_ep *ch37x_ep;
	struct urb *urb;
	struct usb_host_endpoint *ep;

	unsigned long flags;
	int retval = 0;

	spin_lock_irqsave(&ch37x_hcd->lock, flags);
	list_for_each(pos, &ch37x_hcd->ep_list) {
		ch37x_ep = container_of(pos, struct ch37x_ep, ep_list);
		ep = ch37x_ep->ep;
		list_for_each_safe(upos, next_upos, &ep->urb_list) {
			urb = container_of(upos, struct urb, urb_list);
			ch37x_hcd_dbg(ch37x_hcd, "%s:line[%d], urb here! epnum=%d dir=%s\n", __func__, __LINE__,
				      usb_endpoint_num(&urb->ep->desc),
				      usb_endpoint_dir_in(&urb->ep->desc) ? "in" : "out");
			if (urb->unlinked) {
				ch37x_hcd_dbg(ch37x_hcd, "%s:line[%d], urb unlinked here! epnum=%d dir=%s\n", __func__,
					      __LINE__, usb_endpoint_num(&urb->ep->desc),
					      usb_endpoint_dir_in(&urb->ep->desc) ? "in" : "out");
				ch37x_hcd->urb_done = 2;
				retval = 1;
				usb_hcd_unlink_urb_from_ep(hcd, urb);
				spin_unlock_irqrestore(&ch37x_hcd->lock, flags);
				usb_hcd_giveback_urb(hcd, urb, 0);
				spin_lock_irqsave(&ch37x_hcd->lock, flags);
			}
		}
	}
	spin_unlock_irqrestore(&ch37x_hcd->lock, flags);

	ch37x_hcd_dbg(ch37x_hcd, "%s:line[%d], retval=%d\n", __func__, __LINE__, retval);

	return retval;
}

static void ch37x_spi_thread(struct kthread_work *work)
{
	struct ch37x_hcd_data *ch37x_hcd = container_of(work, struct ch37x_hcd_data, spi_work);
	struct usb_hcd *hcd = ch37x_to_hcd(ch37x_hcd);
	int i_worked = 1;
	int urb_null = 0;
	int index = 0;

	while (!kthread_should_stop()) {
		if (ch37x_hcd->rh_state == CH374_RH_RUNNING) {
			msleep(100);
			break;
		}
		pr_info("hcd is not in CH374_RH_RUNNING.\n");
		msleep(1000);
	}

	while (!kthread_should_stop() && !atomic_read(&ch37x_hcd->suspend_flag)) {
		if (!i_worked) {
			set_current_state(TASK_INTERRUPTIBLE);
			if (test_and_clear_bit(ENABLE_IRQ, &ch37x_hcd->todo)) {
				enable_irq(ch37x_hcd->irq);
				ch37x_hcd_dbg_data(ch37x_hcd, "%s:enable irq %d\n", __func__, ch37x_hcd->irq);
			}
			// check if urb is really null (enqueue comes)
			if ((urb_null == 1) && (atomic_read(&ch37x_hcd->urb_process) == 1)) {
				if (!test_and_set_bit(ENABLE_IRQ, &ch37x_hcd->todo)) {
					disable_irq_nosync(ch37x_hcd->irq);
					set_current_state(TASK_RUNNING);
				}
				urb_null = 0;
				i_worked = 1;
				ch37x_hcd_dbg_data(ch37x_hcd, "%s:new urb_enqueue, dont schedule\n", __func__);
				continue;
			}
			schedule();
			set_current_state(TASK_RUNNING);
		}

		i_worked = 0;

		if (ch37x_hcd->urb_done)
			i_worked |= ch37x_urb_done(hcd);
		else if (ch37x_handle_irqs(hcd))
			i_worked = 1;
		else if (!ch37x_hcd->curr_urb) {
			i_worked |= ch37x_select_and_start_urb(hcd);
			if (i_worked == 0) {
				urb_null = 1;
			} else {
				urb_null = 0;
			}
		} else if (ch37x_hcd->curr_urb) {
			ch37x_hcd_dbg_data(ch37x_hcd, "%s:line[%d], curr_urb not null\n", __func__, __LINE__);
		}

		if (test_and_clear_bit(RESET_HCD, &ch37x_hcd->todo))
			/* reset the HCD: */
			i_worked |= ch37x_reset_hcd(hcd);

		for (index = 0; index < NUM_PORTS; index++) {
			if (test_and_clear_bit(RESET_PORT, &ch37x_hcd->ports[index].todoport)) {
				/* perform a USB bus reset: */
				host_reset_bus(hcd, index);
				i_worked = 1;
				ch37x_hcd_dbg(ch37x_hcd, "%s:line[%d], wPortStatus[%d]=0x%x RESET_PORT\n", __func__,
					      __LINE__, index, ch37x_hcd->ports[index].port_status.wPortStatus);
			}
		}

		if (test_and_clear_bit(CHECK_CONNECT, &ch37x_hcd->todo)) {
			pr_info("%s:line[%d]\n", __func__, __LINE__);
			for (index = 0; index < NUM_PORTS; index++)
				ch37x_detect_conn(hcd, index);
			i_worked = 1;
		}

		// 处理已经dequeue的URB
		if (test_and_clear_bit(CHECK_UNLINK, &ch37x_hcd->todo)) {
			pr_info("%s:line[%d]\n", __func__, __LINE__);
			i_worked |= ch37x_check_unlink(hcd);
		}

		ch37x_hcd_dbg_data(ch37x_hcd, "%s:line[%d], i_worked=%d continue\n", __func__, __LINE__, i_worked);
	}
	set_current_state(TASK_RUNNING);
	ch37x_hcd->kthread_running = false;
	wake_up_interruptible(&ch37x_hcd->wthread);

	return;
}

static int ch37x_get_frame_number(struct usb_hcd *hcd)
{
	struct ch37x_hcd_data *ch37x = hcd_to_ch37x(hcd);
	ch37x->frame_number = 0;
	return ch37x->frame_number;
}
#if 1

#ifdef CONFIG_PM
static int ch37x_bus_suspend(struct usb_hcd *hcd)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);

	dev_info(&hcd->self.root_hub->dev, "%s\n", __func__);

	return -1;

	spin_lock_irq(&ch37x_hcd->lock);
	ch37x_hcd->rh_state = CH374_RH_SUSPENDED;
	//	set_link_state(dum_hcd);
	hcd->state = HC_STATE_SUSPENDED;
	spin_unlock_irq(&ch37x_hcd->lock);

	return 0;
}

static int ch37x_bus_resume(struct usb_hcd *hcd)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	int rc = 0;

	dev_info(&hcd->self.root_hub->dev, "%s\n", __func__);

	return -1;

	spin_lock_irq(&ch37x_hcd->lock);
	if (!HCD_HW_ACCESSIBLE(hcd)) {
		rc = -ESHUTDOWN;
	} else {
		ch37x_hcd->rh_state = CH374_RH_RUNNING;
		//		set_link_state(dum_hcd);
		hcd->state = HC_STATE_RUNNING;
	}
	spin_unlock_irq(&ch37x_hcd->lock);

	return rc;
}

#else

#define ch37x_bus_suspend NULL
#define ch37x_bus_resume  NULL

#endif
#endif

static int ch37x_map_urb_for_dma(struct usb_hcd *hcd, struct urb *urb, gfp_t mem_flags)
{
	return 0;
}

static void ch37x_unmap_urb_for_dma(struct usb_hcd *hcd, struct urb *urb)
{
}

static void ch37x_endpoint_disable(struct usb_hcd *hcd, struct usb_host_endpoint *hep)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	unsigned long flags;

	ch37x_hcd_dbg_data(ch37x_hcd, "%s:line[%d] start\n", __func__, __LINE__);
	spin_lock_irqsave(&ch37x_hcd->lock, flags);
	if (hep->hcpriv) {
		struct ch37x_ep *ch37x_ep = hep->hcpriv;
		ch37x_hcd_dbg_data(ch37x_hcd, "%s:line[%d] epnum=%d\n", __func__, __LINE__,
				   usb_endpoint_num(&ch37x_ep->ep->desc));
		if (!list_empty(&ch37x_ep->ep_list)) {
			ch37x_hcd_dbg_data(ch37x_hcd, "%s:line[%d] epnum=%d list_del %p.\n", __func__, __LINE__,
					   usb_endpoint_num(&ch37x_ep->ep->desc), ch37x_ep);
			list_del(&ch37x_ep->ep_list);
		}
		kfree(ch37x_ep);
		hep->hcpriv = NULL;
	}
	spin_unlock_irqrestore(&ch37x_hcd->lock, flags);
	ch37x_hcd_dbg_data(ch37x_hcd, "%s:line[%d] end\n", __func__, __LINE__);

	return;
}

static inline void ch37x_hub_descriptor(struct usb_hub_descriptor *desc)
{
	memset(desc, 0, sizeof(*desc));
	/*
	 * See Table 11-13: Hub Descriptor in USB 2.0 spec.
	 */
	desc->bDescriptorType = 0x29; /* hub descriptor */
	desc->bDescLength = 9;
	desc->bHubContrCurrent = 0;
	// Power switching, device type, overcurrent.
	desc->wHubCharacteristics = 0x12; //
	desc->bPwrOn2PwrGood = 10;	  // 20ms
	desc->bNbrPorts = NUM_PORTS;

	// two bitmaps:  ports removable, and legacy PortPwrCtrlMask
	desc->u.hs.DeviceRemovable[0] = 0;
	desc->u.hs.DeviceRemovable[1] = ~0;
}

static int ch37x_hub_status_data(struct usb_hcd *hcd, char *buf)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	unsigned long flags;
	int retval = 0;
	int index;

	// ch37x_hcd_dbg(ch37x_hcd, "%s:line[%d] start\n", __func__, __LINE__);

	buf[0] = 0;

	if (!HC_IS_RUNNING(hcd->state))
		return -ESHUTDOWN;
	// Report no status change now, if we are scheduled to be called later
	if (timer_pending(&hcd->rh_timer))
		return 0;

	spin_lock_irqsave(&ch37x_hcd->lock, flags);

	/* bit 0 denotes hub change, b1..n port change */
	for (index = 0; index < NUM_PORTS; index++) {
		if (ch37x_hcd->ports[index].wPortTrigger) {
			buf[0] |= 1 << (index + 1);
			ch37x_hcd_dbg(ch37x_hcd, "port %d status 0x%08x has changes\n", index,
				      ch37x_hcd->ports[index].port_status);
			ch37x_hcd->ports[index].wPortTrigger = RH_PORT_STATUS_STAYED;
		}
	}
	spin_unlock_irqrestore(&ch37x_hcd->lock, flags);

	if (buf[0] != 0)
		retval = 1;

	// ch37x_hcd_dbg(ch37x_hcd, "%s:line[%d] buf[0]=0x%2x retval=%d\n", __func__, __LINE__, buf[0], retval);

	return retval;
}

static int ch37x_reset_port(struct usb_hcd *hcd, int index)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);

	ch37x_hcd->ports[index].port_status.wPortStatus &= ~(USB_PORT_STAT_RESET);
	ch37x_hcd->ports[index].port_status.wPortStatus |= USB_PORT_STAT_ENABLE;
	ch37x_hcd->ports[index].port_status.wPortChange |= USB_PORT_STAT_C_RESET;
	ch37x_hcd->ports[index].bus_addr = ch37x_hcd->ports[index].bus_addr_backup = 0x00;
	ch37x_hcd->ports[index].usbsta = USB_RESET;

	set_bit(RESET_PORT, &ch37x_hcd->ports[index].todoport);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	queue_kthread_work(&ch37x_hcd->spi_worker, &ch37x_hcd->spi_work);
#else
	kthread_queue_work(&ch37x_hcd->spi_worker, &ch37x_hcd->spi_work);
#endif
	wake_up_process(ch37x_hcd->spi_thread);

	ch37x_hcd_dbg(ch37x_hcd, "%s:line[%d], index:%d\n", __func__, __LINE__, index);
	return 0;
}

static int ch37x_hub_control(struct usb_hcd *hcd, u16 typeReq, u16 wValue, u16 wIndex, char *buf, u16 wLength)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	int retval = 0;
	unsigned long flags;
	u16 *wdata_buf = (u16 *)buf;
	u8 index = (u8)wIndex;

	pr_info("hub control -> typeReq: %04x wValue: %04x wIndex:%04x\n", typeReq, wValue, wIndex);
	if (index > 0)
		pr_info("hub control -> wPortStatus[%d]: %04x wPortChange: %4x\n", index - 1,
			ch37x_hcd->ports[index - 1].port_status.wPortStatus,
			ch37x_hcd->ports[index - 1].port_status.wPortChange);

	spin_lock_irqsave(&ch37x_hcd->lock, flags);

	switch (typeReq) {
	case ClearHubFeature:
		switch (wValue) {
		case C_HUB_OVER_CURRENT:
		case C_HUB_LOCAL_POWER:
			break;
		default:
			goto error;
		}
		break;
	case SetHubFeature:
		switch (wValue) {
		case C_HUB_OVER_CURRENT:
		case C_HUB_LOCAL_POWER:
			break;
		default:
			goto error;
		}
		break;
	case GetHubDescriptor:
		ch37x_hub_descriptor((struct usb_hub_descriptor *)buf);
		break;
	case GetHubStatus:
		*(__le32 *)buf = cpu_to_le32(0);
		break;
	case GetPortStatus:
		if ((index < 1) || (index > NUM_PORTS)) {
			retval = -EPIPE;
			goto error;
		}
		*wdata_buf = cpu_to_le16(ch37x_hcd->ports[index - 1].port_status.wPortStatus);
		wdata_buf++;
		*wdata_buf = cpu_to_le16(ch37x_hcd->ports[index - 1].port_status.wPortChange);
		break;
	case ClearPortFeature:
		if ((index < 1) || (index > NUM_PORTS)) {
			retval = -EPIPE;
			goto error;
		}
		switch (wValue) {
		case USB_PORT_FEAT_ENABLE:
			if (ch37x_hcd->ports[index - 1].port_status.wPortStatus & USB_PORT_STAT_CONNECTION) {
				ch37x_hcd->ports[index - 1].port_status.wPortStatus &= ~USB_PORT_STAT_ENABLE;
			}
			break;
		case USB_PORT_FEAT_C_ENABLE:
			ch37x_hcd->ports[index - 1].port_status.wPortChange &= ~USB_PORT_STAT_C_ENABLE;
			break;
		case USB_PORT_FEAT_SUSPEND:
			ch37x_hcd->ports[index - 1].port_status.wPortStatus &= ~USB_PORT_STAT_SUSPEND;
			break;
		case USB_PORT_FEAT_C_SUSPEND:
			ch37x_hcd->ports[index - 1].port_status.wPortChange &= ~USB_PORT_STAT_C_SUSPEND;
			break;
		case USB_PORT_FEAT_POWER:
			ch37x_hcd->ports[index - 1].port_status.wPortStatus &= ~USB_PORT_STAT_POWER;
			break;
		case USB_PORT_FEAT_C_CONNECTION:
			ch37x_hcd->ports[index - 1].port_status.wPortChange &= ~USB_PORT_STAT_C_CONNECTION;
			break;
		case USB_PORT_FEAT_C_OVER_CURRENT:
			ch37x_hcd->ports[index - 1].port_status.wPortChange &= ~USB_PORT_STAT_C_OVERCURRENT;
			break;
		case USB_PORT_FEAT_C_RESET:
			ch37x_hcd->ports[index - 1].port_status.wPortChange &= ~USB_PORT_STAT_C_RESET;
			break;
		default:
			goto error;
		}
		break;
	case SetPortFeature:
		if ((index < 1) || (index > NUM_PORTS)) {
			retval = -EPIPE;
			goto error;
		}
		switch (wValue) {
		case USB_PORT_FEAT_ENABLE:
			ch37x_hcd->ports[index - 1].port_status.wPortStatus |= USB_PORT_STAT_ENABLE;
			break;
		case USB_PORT_FEAT_SUSPEND:
			break;
		case USB_PORT_FEAT_POWER:
			break;
		case USB_PORT_FEAT_RESET:
			/*reset port*/
			ch37x_reset_port(hcd, index - 1);
			break;
		default:
			goto error;
		}
		break;

	default:
		pr_info("hub control typeReq:%04x wValue: %04x wIndex:%04x\n", typeReq, wValue, wIndex);

error: /* "protocol stall" on error */
		retval = -EPIPE;
	}
	spin_unlock_irqrestore(&ch37x_hcd->lock, flags);

	ch37x_hcd_dbg(ch37x_hcd, "%s:line[%d] retval=%d end\n", __func__, __LINE__, retval);

	return retval;
}

/*
 * Searches list of ports to find the index of the one with a specified  USB
 * bus address. If none of the ports has the bus address then the connection
 * port is returned, if there is one or -1 otherwise.
 * Context: any
 */
static int ch37x_get_port_from_addr(struct ch37x_hcd_data *ch37x_hcd, u8 bus_addr)
{
	int i;

	ch37x_hcd_dbg_data(ch37x_hcd, "%s:line[%d] bus_addr:%d\n", __func__, __LINE__, bus_addr);

	if (bus_addr == 0x00)
		return NUM_PORTS;

	for (i = 0; i < NUM_PORTS; i++) {
		if (ch37x_hcd->ports[i].bus_addr == bus_addr) {
			return i;
		}
	}

	return NUM_PORTS;
}

/*
 * Searches list of ports to find the index of the one with a specified  USB
 * bus address. If none of the ports has the bus address then the connection
 * port is returned, if there is one or -1 otherwise.
 * Context: any
 */
static int ch37x_get_port_from_addr_backup(struct ch37x_hcd_data *ch37x_hcd, u8 bus_addr)
{
	int i;

	if (bus_addr == 0x00)
		return NUM_PORTS;

	for (i = 0; i < NUM_PORTS; i++) {
		if (ch37x_hcd->ports[i].bus_addr_backup == bus_addr)
			return i;
	}
	ch37x_hcd_dbg_data(ch37x_hcd, "%s:line[%d] bus_addr:%d\n", __func__, __LINE__, bus_addr);

	return NUM_PORTS;
}

/*
 * Called to queue an urb for the device.
 * This function should return a non-zero error code if it fails the urb but
 * should not call usb_hcd_giveback_urb().
 * Context: any
 */
static int ch37x_urb_enqueue(struct usb_hcd *hcd, struct urb *urb, gfp_t mem_flags)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	struct ch37x_ep *ch37x_ep;
	unsigned long flags;
	int retval = 0;

	spin_lock_irqsave(&ch37x_hcd->lock, flags);

	ch37x_ep = urb->ep->hcpriv;
	if (!ch37x_ep) {
		ch37x_hcd_dbg_data(ch37x_hcd, "%s:line[%d] ch37x_ep null here\n", __func__, __LINE__);
		ch37x_ep = kzalloc(sizeof(struct ch37x_ep), GFP_ATOMIC);
		if (!ch37x_ep) {
			retval = -ENOMEM;
			goto exit;
		}
		ch37x_ep->ep = urb->ep;
		urb->ep->hcpriv = ch37x_ep;
		list_add_tail(&ch37x_ep->ep_list, &ch37x_hcd->ep_list);
	}

	retval = usb_hcd_link_urb_to_ep(hcd, urb);
	if (retval == 0) {
		atomic_set(&ch37x_hcd->urb_process, 1);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
		queue_kthread_work(&ch37x_hcd->spi_worker, &ch37x_hcd->spi_work);
#else
		kthread_queue_work(&ch37x_hcd->spi_worker, &ch37x_hcd->spi_work);
#endif
		wake_up_process(ch37x_hcd->spi_thread);
	}

	ch37x_hcd_dbg_data(ch37x_hcd, "%s:line[%d] retval=%d epnum=%d dir=%s\n", __func__, __LINE__, retval,
			   usb_endpoint_num(&urb->ep->desc), usb_endpoint_dir_in(&urb->ep->desc) ? "in" : "out");
exit:
	spin_unlock_irqrestore(&ch37x_hcd->lock, flags);
	return retval;
}

static int ch37x_urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int status)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	unsigned long flags;
	int retval = 0;

	spin_lock_irqsave(&ch37x_hcd->lock, flags);
	retval = usb_hcd_check_unlink_urb(hcd, urb, status);
	if (retval == 0) {
		set_bit(CHECK_UNLINK, &ch37x_hcd->todo);
//		atomic_set(&ch37x_hcd->urb_process, 0);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
		queue_kthread_work(&ch37x_hcd->spi_worker, &ch37x_hcd->spi_work);
#else
		kthread_queue_work(&ch37x_hcd->spi_worker, &ch37x_hcd->spi_work);
#endif
		wake_up_process(ch37x_hcd->spi_thread);
	}
	spin_unlock_irqrestore(&ch37x_hcd->lock, flags);

	ch37x_hcd_dbg_data(ch37x_hcd, "%s:line[%d] retval=%d epnum=%d dir=%s\n", __func__, __LINE__, retval,
			   usb_endpoint_num(&urb->ep->desc), usb_endpoint_dir_in(&urb->ep->desc) ? "in" : "out");
	return 0;
}

static int ch37x_start(struct usb_hcd *hcd)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	int index;
	int i;

	ch37x_hcd_dbg(ch37x_hcd, "%s:line[%d] start\n", __func__, __LINE__);
	hcd->state = HC_STATE_RUNNING;
	ch37x_hcd->rh_state = CH374_RH_RUNNING;
	INIT_LIST_HEAD(&ch37x_hcd->ep_list);
	for (index = 0; index < NUM_PORTS; index++) {
		ch37x_hcd->ports[index].port_status.wPortStatus = 0;
		ch37x_hcd->ports[index].port_status.wPortChange = 0;
		ch37x_hcd->ports[index].wPortTrigger = RH_PORT_STATUS_STAYED;
		ch37x_hcd->ports[index].bus_addr = ch37x_hcd->ports[index].bus_addr_backup = 0x00;
		ch37x_hcd->ports[index].usbsta = USB_UNLINK;
		ch37x_hcd->ports[index].controltog = false;
		for (i = 0; i < 4; i++) {
			ch37x_hcd->ports[index].transferintog[i] = false;
			ch37x_hcd->ports[index].transferouttog[i] = false;
		}
	}

	return 0;
}

static void ch37x_stop(struct usb_hcd *hcd)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);
	unsigned long flags;
#if 1
	spin_lock_irqsave(&ch37x_hcd->lock, flags);
	hcd->state = HC_STATE_HALT;
	spin_unlock_irqrestore(&ch37x_hcd->lock, flags);
#endif
	ch37x_hcd_dbg(ch37x_hcd, "%s:line[%d] end\n", __func__, __LINE__);
}

static int ch37x_reset(struct usb_hcd *hcd)
{
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);

	ch37x_hcd_dbg(ch37x_hcd, "%s:line[%d]\n", __func__, __LINE__);
	hcd->self.sg_tablesize = 0;
	hcd->speed = HCD_USB2;
	hcd->self.root_hub->speed = USB_SPEED_FULL;
	set_bit(RESET_HCD, &ch37x_hcd->todo);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	queue_kthread_work(&ch37x_hcd->spi_worker, &ch37x_hcd->spi_work);
#else
	kthread_queue_work(&ch37x_hcd->spi_worker, &ch37x_hcd->spi_work);
#endif
	wake_up_process(ch37x_hcd->spi_thread);

	return 0;
}

#ifdef CONFIG_OF
static struct xgold_spi_chip *spi_ch37x_hcd_parse_dt(struct usb_hcd *hcd, struct device *dev)
{
	u32 temp;
	struct xgold_spi_chip *spi_chip_data;
	struct ch37x_hcd_data *ch37x_hcd = hcd_to_ch37x(hcd);

	spi_chip_data = devm_kzalloc(dev, sizeof(*spi_chip_data), GFP_KERNEL);
	if (!spi_chip_data) {
		dev_err(dev, "memory allocation for spi_chip_data failed\n");
		return ERR_PTR(-ENOMEM);
	}

	if (of_property_read_u32(dev->of_node, "poll_mode", &temp)) {
		dev_warn(dev, "fail to get poll_mode, default set 0\n");
		spi_chip_data->poll_mode = 0;
	} else {
		spi_chip_data->poll_mode = temp;
	}

	if (of_property_read_u32(dev->of_node, "type", &temp)) {
		dev_warn(dev, "fail to get type, default set 0\n");
		spi_chip_data->type = 0;
	} else {
		spi_chip_data->type = temp;
	}

	if (of_property_read_u32(dev->of_node, "enable_dma", &temp)) {
		dev_warn(dev, "fail to get enable_dma, default set 0\n");
		spi_chip_data->enable_dma = 0;
	} else {
		spi_chip_data->enable_dma = temp;
	}

	ch37x_hcd_dbg(ch37x_hcd, "%s: poll_mode=%d, type=%d, enable_dma=%d\n", __func__, spi_chip_data->poll_mode,
		      spi_chip_data->type, spi_chip_data->enable_dma);

	return spi_chip_data;
}
#else
static struct spi_board_info *spi_ch37x_hcd_parse_dt(struct usb_hcd *hcd, struct device *dev)
{
	return dev->platform_data;
}
#endif

static struct hc_driver ch37x_hcd_desc = {
	.description = "ch37x",
	.product_desc = DRIVER_DESC,
	.hcd_priv_size = sizeof(struct ch37x_hcd_data),
	.flags = HCD_USB11,
	.reset = ch37x_reset,
	.start = ch37x_start,
	.stop = ch37x_stop,
	.get_frame_number = ch37x_get_frame_number,
	.urb_enqueue = ch37x_urb_enqueue,
	.urb_dequeue = ch37x_urb_dequeue,
	.map_urb_for_dma = ch37x_map_urb_for_dma,
	.unmap_urb_for_dma = ch37x_unmap_urb_for_dma,
	.endpoint_disable = ch37x_endpoint_disable,
	.hub_status_data = ch37x_hub_status_data,
	.hub_control = ch37x_hub_control,
	.bus_suspend = ch37x_bus_suspend,
	.bus_resume = ch37x_bus_resume,
};

static void ch37x_resume_delaywork_func(struct work_struct *work)
{
	struct delayed_work *delaywork = container_of(work, struct delayed_work, work);
	struct ch37x_hcd_data *ch37x_hcd = container_of(delaywork, struct ch37x_hcd_data, delaywork);
	struct usb_hcd *hcd = ch37x_to_hcd(ch37x_hcd);

	return;

#if 0
	atomic_set(&ch37x_hcd->suspend_flag, 0);
	//	Write374Byte(hcd,REG_HUB_SETUP, Read374Byte(hcd, REG_HUB_SETUP) | BIT_HUB0_EN);
	//	Write374Byte(hcd, REG_SYS_CTRL, Read374Byte(hcd, REG_SYS_CTRL ) & (~ BIT_CTRL_OSCIL_OFF) );
	msleep(100);
	ch37x_reset_hcd(hcd);
	enable_irq(ch37x_hcd->irq);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	queue_kthread_work(&ch37x_hcd->spi_worker, &ch37x_hcd->spi_work);
#else
	kthread_queue_work(&ch37x_hcd->spi_worker, &ch37x_hcd->spi_work);
#endif
	wake_up_process(ch37x_hcd->spi_thread);
	msleep(1000); //wait for usb host regist,neccessary?
	pr_info("%s:exit\n", __func__);
#endif
}

static int ch37x_probe(struct spi_device *spi)
{
	struct ch37x_hcd_data *ch37x_hcd;
	struct usb_hcd *hcd = NULL;
	int irq;
	unsigned long flags = IRQF_TRIGGER_LOW;
#ifdef CONFIG_OF
	static struct xgold_spi_chip *spi_chip_data;
#else
	struct spi_board_info *spi_chip_data;
#endif
	struct device_node *np = of_node_get(spi->dev.of_node);
	int retval = -ENOMEM;

	if (!spi)
		return -ENOMEM;

	spi->max_speed_hz = 20000000;

	if (spi_setup(spi) < 0) {
		dev_err(&spi->dev, "Unable to setup SPI bus");
		return -EFAULT;
	}

	if (usb_disabled())
		return -ENODEV;
	hcd = usb_create_hcd(&ch37x_hcd_desc, &spi->dev, dev_name(&spi->dev));
	if (!hcd) {
		dev_err(&spi->dev, "failed to create HCD structure\n");
		goto error;
	}

	ch37x_hcd = hcd_to_ch37x(hcd);
	ch37x_hcd->next = ch37x_hcd_list;
	ch37x_hcd_list = ch37x_hcd;
	INIT_LIST_HEAD(&ch37x_hcd->ep_list);
	spin_lock_init(&ch37x_hcd->lock);
	init_waitqueue_head(&ch37x_hcd->wthread);
	spin_lock_init(&spilock);

	if (!spi_chip_data && np) {
		spi_chip_data = spi_ch37x_hcd_parse_dt(hcd, &spi->dev);
		if (IS_ERR(spi_chip_data))
			return -ENOMEM;
	}

	spi->bits_per_word = 8;
	spi->controller_data = spi_chip_data;

/* if your platform supports acquire irq number from dts */
#ifdef USE_IRQ_FROM_DTS
	irq = spi->irq;
#else
	retval = devm_gpio_request(&spi->dev, GPIO_NUMBER, "gpioint");
	if (retval) {
		dev_err(&spi->dev, "gpio_request\n");
		goto error;
	}
	retval = gpio_direction_input(GPIO_NUMBER);
	if (retval) {
		dev_err(&spi->dev, "gpio_direction_input\n");
		goto error;
	}
	irq_set_irq_type(gpio_to_irq(GPIO_NUMBER), flags);
	irq = gpio_to_irq(GPIO_NUMBER);
#endif

	ch37x_hcd->spi = spi;
	ch37x_hcd->dev = &spi->dev;
	ch37x_hcd->irq = irq;

	ch37x_hcd->tx = devm_kzalloc(&spi->dev, sizeof(*ch37x_hcd->tx), GFP_KERNEL);
	if (!ch37x_hcd->tx) {
		dev_err(&spi->dev, "failed to kmalloc tx buffer\n");
		goto error;
	}
	ch37x_hcd->rx = devm_kzalloc(&spi->dev, sizeof(*ch37x_hcd->rx), GFP_KERNEL);
	if (!ch37x_hcd->rx) {
		dev_err(&spi->dev, "failed to kmalloc rx buffer\n");
		goto error;
	}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	init_kthread_worker(&ch37x_hcd->spi_worker);
#else
	kthread_init_worker(&ch37x_hcd->spi_worker);
#endif
	ch37x_hcd->spi_thread = kthread_run(kthread_worker_fn, &ch37x_hcd->spi_worker, "ch37x_spi_thread");
	if (IS_ERR(ch37x_hcd->spi_thread)) {
		retval = PTR_ERR(ch37x_hcd->spi_thread);
		ch37x_hcd->spi_thread = NULL;
		pr_err("failed to run spi_thread\n");
		goto error;
	}
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	init_kthread_work(&ch37x_hcd->spi_work, ch37x_spi_thread);
#else
	kthread_init_work(&ch37x_hcd->spi_work, ch37x_spi_thread);
#endif

	retval = usb_add_hcd(hcd, 0, 0);
	if (retval) {
		dev_err(&spi->dev, "failed to add HCD\n");
		goto error;
	}

	spi_set_drvdata(spi, ch37x_hcd);
	atomic_set(&ch37x_hcd->suspend_flag, 0);
	INIT_DELAYED_WORK(&ch37x_hcd->delaywork, ch37x_resume_delaywork_func);
	atomic_set(&ch37x_hcd->debug_flag, DEBUG_FLAG_LEVEL);
	atomic_set(&ch37x_hcd->urb_process, 0);
	mutex_init(&ch37x_hcd->thread_mutex);

	retval = devm_request_threaded_irq(&spi->dev, irq, ch37x_irq_handler, ch37x_irq_thread_handler,
					   flags | IRQF_ONESHOT, "spi-ch37x-hcd", hcd);
	if (retval < 0) {
		dev_err(&spi->dev, "failed to request irq %d error %d\n", irq, retval);
		goto error;
	}

	printk("spi->max_speed_hz: %d Hz\n", spi->max_speed_hz);
	ch37x_debugfs_init(hcd);

	return 0;

error:
	if (hcd) {
		if (ch37x_hcd->spi_thread)
			kthread_stop(ch37x_hcd->spi_thread);
		usb_put_hcd(hcd);
	}
	return retval;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
static void ch37x_remove(struct spi_device *spi)
#else
static int ch37x_remove(struct spi_device *spi)
#endif
{
	struct ch37x_hcd_data *ch37x_hcd = spi_get_drvdata(spi);
	struct usb_hcd *hcd = NULL;
	int timeout;

	hcd = ch37x_to_hcd(ch37x_hcd);

	ch37x_debugfs_exit(hcd);

	usb_remove_hcd(hcd);

	kthread_stop(ch37x_hcd->spi_thread);
	timeout = wait_event_interruptible_timeout(ch37x_hcd->wthread, !ch37x_hcd->kthread_running,
						   msecs_to_jiffies(2000));
	if (timeout <= 0) {
	}

	devm_free_irq(&spi->dev, ch37x_hcd->irq, hcd);

#ifndef USE_IRQ_FROM_DTS
	devm_gpio_free(&spi->dev, GPIO_NUMBER);
#endif

	usb_put_hcd(hcd);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
	return;
#else
	return 0
#endif
}

#ifdef CONFIG_OF
static const struct of_device_id spi_ch37x_hcd_dt_match[] = {
	{
		.compatible = "wch,ch37x_hcd",
	},
	{},
};
MODULE_DEVICE_TABLE(of, spi_ch37x_hcd_dt_match);

#endif

static struct spi_driver ch37x_driver = {
	.probe =	ch37x_probe,
	.remove =	ch37x_remove,
	.driver = {
		.name =	"wch, spi_ch37x_hcd",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(spi_ch37x_hcd_dt_match),
	},
};

static int __init ch37x_init(void)
{
	int retval;
	if (usb_disabled())
		return -ENODEV;

	retval = spi_register_driver(&ch37x_driver);

	return retval;
}

static void __exit ch37x_exit(void)
{
	spi_unregister_driver(&ch37x_driver);
}

module_init(ch37x_init);
module_exit(ch37x_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(VERSION_DESC);
MODULE_LICENSE("GPL");
