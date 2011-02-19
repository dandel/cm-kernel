/***************************************************************************** 
 * sensor.c 
 * 
 * Copyright (c) 2009~2014 ShangHai Infotm Ltd all rights reserved. 
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * Description: main file of imapx200 media sensor driver
 *
 * Author:
 *     Sololz <sololz@infotm.com>
 *      
 * Revision History: 
 * ­­­­­­­­­­­­­­­­­ 
 * 1.1  12/9/2009 Sololz 
 ******************************************************************************/

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/poll.h>  
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/types.h>    
#include <linux/interrupt.h>
#include <linux/init.h>      
#include <linux/string.h>
#include <linux/mm.h>             
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/clk.h>
#include <asm/delay.h>
#include <mach/imapx_gpio.h>
#include <linux/semaphore.h>
#include <linux/workqueue.h>
/*
 * XXX: sensor hard ware directly connect to system bus, there is 
 * no use to clock set
 */ 
#include <plat/clock.h>

#include <mach/imapx_base_reg.h>
#include <mach/irqs.h>

#include "common_sensor.h"

/*
 * functions delare
 */
static int reset_hw_reg_sensor(void);		/* this function just set all resigster to be 0 */
static int sensor_driver_register(void);	/* register driver as an char device */
static int sensor_driver_unregister(void);

/*
 * this structure include global varaibles
 */
sensor_param_t	sensor_param;		/* global variables group */
static struct	clk *imap_vdec_clock;	/* sensor hardware clock */
static unsigned int sensor_open_count;	/* record open syscall times */
static wait_queue_head_t wait_sensor;	/* a wait queue for poll system call */
struct sensor_orientation {
	char buff[3];
	int buffersize;
	struct semaphore sem;
	struct work_struct work;
};

enum sensor_state {
	sensor_init,
	sensor_top
};
static int sensor_state_ori;
int sensor_flag, sensor_old_flag;
struct sensor_orientation *sensor_dev; 
static int sensor_direction = 0;
/*
 * open system call just mark file private data as a sensor 
 * instance by default, and you can change it by a ioctl call
 */
static int imapx200_sensor_open(struct inode *inode, struct file *file)
{
	sensor_open_count++;
	sensor_flag=0;
	sensor_old_flag=0;
	/* dec instance by default, you can change it by ioctl pp instance */
	file->private_data = &(sensor_param.dec_instance);

	sensor_debug("IMAPX200 Decode open OK\n");

	return IMAPX200_SENSOR_RET_OK;
}
static int imapx200_sensor_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
        volatile unsigned  int  tmp,tmp1,tmp2; 	
	/* dec instance by default, you can change it by ioctl pp instance */
	sensor_debug("IMAPX200 read OK\n");

	//if (down_interruptible(&sensor_dev->sem))
/*	while (sensor_flag == sensor_old_flag) {
		sensor_debug("goto sleep!\n");
		if (wait_event_interruptible(wait_sensor, sensor_flag == 1 )){
			copy_to_user(buf, "wak", count);
			sensor_debug("wait event fail!\n");
			return count;
		}
	}
	if (sensor_flag == 1){
		sensor_debug("now, return to main loop!\n");
		sensor_flag = 0;
	}*/
	tmp = __raw_readl(rGPIDAT);
//	printk(KERN_EMERG"gpldat:%08x\n",tmp);
	tmp1 = (tmp>>10) & 0x1;
	tmp2 = (tmp >> 11) & 0x1;
//	printk(KERN_EMERG "oritation is %x,%x\n", tmp1, tmp2);

        if ((tmp1 == 0) && (tmp2 == 0)){ 
		printk(KERN_EMERG "*************1\n");
		sensor_dev->buff[0]=0; 
		sensor_dev->buff[1]=1;
		sensor_dev->buff[2]=1;
	}else if ((tmp1 == 0) && (tmp2 == 1)){
		printk("*************2\n");
		sensor_dev->buff[0]=0; 
		sensor_dev->buff[1]=1;
		sensor_dev->buff[2]=0;
	}else if ((tmp1 == 1) && (tmp2 == 0)){
		sensor_dev->buff[0]=1; 
		sensor_dev->buff[1]=1;
		sensor_dev->buff[2]=0;
	}else if((tmp1 == 1) && (tmp2 == 1)){
		sensor_dev->buff[0]=1; 
		sensor_dev->buff[1]=0;
		sensor_dev->buff[2]=0;
        }
	if (copy_to_user(buf, sensor_dev->buff, count)) {
		return -EFAULT;
	}
	return count;
}


/*
 * fasync system call be called here
 */
static int imapx200_sensor_release(struct inode *inode, struct file *file)
{
	sensor_open_count--;

	sensor_debug("IMAPX200 Decode release OK\n");
	cancel_work_sync(&sensor_dev->work);
//	free_irq(IRQ_GPIO, NULL);
	return IMAPX200_SENSOR_RET_OK;
}

static struct file_operations imapx200_sensor_fops = 
{
	owner:		THIS_MODULE,
	open:		imapx200_sensor_open,
	read:		imapx200_sensor_read,
	release:	imapx200_sensor_release,
};


static void sensor_event(struct work_struct *work){
	sensor_flag =1;
	wake_up_interruptible(&wait_sensor);
}
static irqreturn_t sensor_keys_orientation(int irq, void *dev_id)
{
	volatile unsigned int tmp1,tmp2, tmp, i;

	printk(KERN_EMERG "sensor_direction ");
	//sensor_state_ori ++;
#ifdef CONFIG_IMAP_PRODUCTION
        tmp= __raw_readl(rEINTG6MASK);
	tmp |= (0x3<<14);
	__raw_writel(tmp, rEINTG6MASK);
	//__raw_writel(tmp, rEINTG5MASK);
	
	for (i=0; i<100; i++)
	udelay(500);
	
	__raw_writel(tmp, rEINTG6PEND);
//	__raw_writel(tmp, rEINTG5PEND);
	
	tmp = __raw_readl(rGPIDAT);
	printk(KERN_EMERG"gpldat:0x%x\n",tmp);
	tmp1 = (tmp>>10) & 0x1;
	tmp2 = (tmp >> 11) & 0x1;
	printk(KERN_EMERG "oritation is %x,%x\n", tmp1, tmp2);

        tmp= __raw_readl(rEINTG6MASK);	
	tmp &= ~(0x3<<14);
	__raw_writel(tmp, rEINTG6MASK);
//	tmp = 0x7fffffff;
//	__raw_writel(tmp, rEINTG5MASK);

        printk(KERN_EMERG "sensorcount is %d\n",sensor_open_count);

	if (sensor_open_count != 0) {
	if ((tmp1 == 0x0) && (tmp2 == 0x0)){ 
//		printk(KERN_EMERG "*************1\n");
		sensor_dev->buff[0]=0; 
		sensor_dev->buff[1]=1;
		sensor_dev->buff[2]=1;
	}else if ((tmp1 == 0x0) && (tmp2 == 0x1)){
//		printk("*************2\n");
		sensor_dev->buff[0]=0; 
		sensor_dev->buff[1]=1;
		sensor_dev->buff[2]=0;
	}else if ((tmp1 == 0x1) && (tmp2 == 0x0)){
		sensor_dev->buff[0]=1; 
		sensor_dev->buff[1]=1;
		sensor_dev->buff[2]=0;
	}else if((tmp1 == 0x1) && (tmp2 == 0x1)){
		sensor_dev->buff[0]=1; 
		sensor_dev->buff[1]=0;
		sensor_dev->buff[2]=0;
        }

#else	
	tmp = 0xffffffff;
	__raw_writel(tmp, rEINTG4MASK);
	for (i=0; i<100; i++)
	udelay(500);
	__raw_writel(tmp, rEINTG4PEND);
	tmp1 = __raw_readl(rGPEDAT);
	printk(KERN_EMERG "GPEDAT is %x\n", tmp1);
	tmp = __raw_readl(rGPECON);
	//printk(KERN_EMERG "GPECON is %x\n", tmp);
	tmp = __raw_readl(rGPEPUD);
	tmp = 0xffffffcf;
	__raw_writel(tmp, rEINTG4MASK);
        printk(KERN_EMERG "sensorcount is %d\n",sensor_open_count);

	if (sensor_open_count != 0) {
	if (tmp1 == 0x7880){ 
		printk(KERN_EMERG "*************1\n");
		sensor_dev->buff[0]=1; 
		sensor_dev->buff[1]=0;
		sensor_dev->buff[2]=0;
	}else if (tmp1 == 0x78a0){
		printk("*************2\n");
		sensor_dev->buff[0]=0; 
		sensor_dev->buff[1]=1;
		sensor_dev->buff[2]=0;
	}else if (tmp1==0x78b0){
		sensor_dev->buff[0]=0; 
		sensor_dev->buff[1]=1;
		sensor_dev->buff[2]=1;
	}else if(tmp1==0x7890){
            sensor_dev->buff[0]=1; 
		sensor_dev->buff[1]=1;
		sensor_dev->buff[2]=0;
        }
#endif
	//sensor_state_ori =0;
//	sensor_flag =1;
//	wake_up_interruptible(&wait_sensor);
     printk(KERN_EMERG "x is %d,y is %d,z is %d",sensor_dev->buff[0],sensor_dev->buff[1],sensor_dev->buff[2]);
	schedule_work(&sensor_dev->work);
	}
	return IRQ_HANDLED;
}



/*
 * platform operation relate functions
 */
static int imapx200_sensor_probe(struct platform_device *pdev)
{
	int ret, error =0;
	unsigned int	size;
	struct resource	*res;
	 unsigned long rEINTCON_value, rEINTFLTCON0_value, \
		 rEINTG4MASK_value,rEINTGCON_value, rEINTGFLTCON0_value,  \
		 rGPECON_value, rGPEDAT_value, rGPGDAT_value, rGPEPUD_value,	\
		 rGPICON_value, rEINTGFLTCON1_value, rEINTG5MASK_value,	rEINTG6MASK_value;

	/* initualize wait queue */
	init_waitqueue_head(&wait_sensor);
	ret = 0;
#ifdef CONFIG_IMAP_PRODUCTION
	//rGPEPUD_value = __raw_readl(rGPEPUD);
	//printk("*********************rGPEPUD is %x\n", rGPEPUD_value);
	//rGPEPUD_value &= 0xf000;
	//__raw_writel(rGPEPUD_value, rGPEPUD);        //EINTFILTER

	/*config GPI10 and GPI11 as input gpio*/
	rGPICON_value = __raw_readl(rGPICON);
	rGPICON_value &= ~(0xf <<20) ;
	__raw_writel(rGPICON_value, rGPICON);

	/*setting the method of the eint group 6 */
	rEINTGCON_value = __raw_readl(rEINTGCON);
	rEINTGCON_value |= (0x6 <<20);
	__raw_writel(rEINTGCON_value, rEINTGCON);

	/*setting the external group filter control register*/
	rEINTGFLTCON1_value = __raw_readl(rEINTGFLTCON1);
	rEINTGFLTCON1_value |= (0xff << 8);
	__raw_writel(rEINTGFLTCON0_value, rEINTGFLTCON1);

	/*enable external interupt group 6 mask register*/
	rEINTG6MASK_value = __raw_readl(rEINTG6MASK);
	rEINTG6MASK_value &= 0xffffcfff;
	__raw_writel(rEINTG6MASK_value, rEINTG6MASK);

/*	rEINTG5MASK_value = __raw_readl(rEINTG5MASK);
	rEINTG5MASK_value = 0x7fffffff;
	__raw_writel(rEINTG5MASK_value, rEINTG5MASK);
*/

#else
	rGPEPUD_value = __raw_readl(rGPEPUD);
	//printk("*********************rGPEPUD is %x\n", rGPEPUD_value);
	rGPEPUD_value &= 0xf000;
	__raw_writel(rGPEPUD_value, rGPEPUD);        //EINTFILTER

	/*config GPE4 and GPE5 as input gpio*/
	rGPECON_value = __raw_readl(rGPECON);
	rGPECON_value &= 0xf000;
	__raw_writel(rGPECON_value, rGPECON);
	/*setting the method of the eint group 4*/
	rEINTGCON_value = __raw_readl(rEINTGCON);
	rEINTGCON_value |= (0x6<<12);
	__raw_writel(rEINTGCON_value, rEINTGCON);
	/*setting the external group filter control register*/
	rEINTGFLTCON0_value = __raw_readl(rEINTGFLTCON0);
	rEINTGFLTCON0_value |= (0xff<<24);
	__raw_writel(rEINTGFLTCON0_value, rEINTGFLTCON0);
	/*enable external interupt group 4 mask register*/
	rEINTG4MASK_value = __raw_readl(rEINTG4MASK);
	//printk("rEINTG4MASK is %x\n", rEINTG4MASK_value);
	//rEINTG4MASK_value &= ~(0x1<<4);
	rEINTG4MASK_value = 0xffffffcf;
	__raw_writel(rEINTG4MASK_value, rEINTG4MASK);
#endif

//	error = request_irq(IRQ_GPIO, sensor_keys_orientation,IRQF_DISABLED,"sensor_keys_orientation",NULL);
	if (error) {
		pr_err("gpio-keys: Unable to claim irq %d; error %d\n",
			IRQ_GPIO, error);
		return error;
		}

	if(ret)
	{
		sensor_error("Fail to request irq for IMAPX200 Decode device\n");
		return ret;
	}
	/* register char device driver */
	ret = -1;
	ret = sensor_driver_register();
	if(ret)
	{
		sensor_error("Fail to register char device for IMAPX200 Decode\n");
		return ret;
	}
	sensor_debug("IMAPX200 Decode Driver probe OK\n");
	return IMAPX200_SENSOR_RET_OK;
}

static int imapx200_sensor_remove(struct platform_device *pdev)
{
	/* release irq */
	sensor_debug("IMAPX200_SENSOR_REMOVE\n");
	free_irq(IRQ_GPIO, pdev);
	sensor_driver_unregister();
	return IMAPX200_SENSOR_RET_OK;
}

#ifdef CONFIG_PM
static int imapx200_sensor_suspend(struct platform_device *pdev, pm_message_t state)
{
	return IMAPX200_SENSOR_RET_OK;
}

static int imapx200_sensor_resume(struct platform_device *pdev)
{
	return IMAPX200_SENSOR_RET_OK;
}
#endif

static struct platform_driver imapx200_sensor_driver = 
{
	.probe		= imapx200_sensor_probe,
	.remove		= imapx200_sensor_remove,
#ifdef CONFIG_PM
//	.suspend	= imapx200_sensor_suspend,
	.resume		= imapx200_sensor_resume,
#endif
	.driver		=
	{
		.owner		= THIS_MODULE,
		.name		= "imapx200-sensor-orientation",
	},
};

/*
 * init and exit
 */
static int __init imapx200_sensor_init(void)
{
	/* call probe */
	//enum sensor_state sensor_state_ori;
	if(platform_driver_register(&imapx200_sensor_driver))
	{
		sensor_error("Fail to register platform driver for IMAPX200 Decode Driver\n");
		return -EPERM;
	}


	sensor_dev = kmalloc(sizeof(struct sensor_orientation), GFP_KERNEL);
	if (sensor_dev == NULL) {
	sensor_debug("malloc sensor_dev failed\n");
		return -1;
	}
	memset(sensor_dev, 0, sizeof(struct sensor_orientation));
//	init_MUTEX(&(sensor_dev->sem));
//	INIT_WORK(&sensor_dev->work, sensor_event);
	return IMAPX200_SENSOR_RET_OK;
}

static void __exit imapx200_sensor_exit(void)
{
	/* call remove */
	platform_driver_unregister(&imapx200_sensor_driver);

	sensor_debug("IMAPX200 Decode Driver exit OK\n");
}

module_init(imapx200_sensor_init);
module_exit(imapx200_sensor_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sololz of InfoTM");
MODULE_DESCRIPTION("IMAPX200 Decode Driver");

/*
 * just write 0 to all registers to reset harware
 * TODO: we have check whether it's needed
 */
int reset_hw_reg_sensor(void)
{
	return IMAPX200_SENSOR_RET_OK;
}

/*
 * this function do driver register to regist a node under /dev
 */
static struct class *sensor_class;

int sensor_driver_register(void)
{
	int ret;

	ret = -1;
	ret = register_chrdev(SENSOR_DEFAULT_MAJOR, "imapx200-sensor", &imapx200_sensor_fops);
	if(ret < 0)
	{
		sensor_error("register char deivce error\n");
		return IMAPX200_SENSOR_RET_ERROR;
	}

	sensor_class = class_create(THIS_MODULE, "imapx200-sensor");
	device_create(sensor_class, NULL, MKDEV(SENSOR_DEFAULT_MAJOR, SENSOR_DEFAULT_MINOR), NULL, "imapx200-sensor");

	return IMAPX200_SENSOR_RET_OK;
}

int sensor_driver_unregister(void)
{
	device_destroy(sensor_class, MKDEV(SENSOR_DEFAULT_MAJOR, SENSOR_DEFAULT_MINOR));
	class_destroy(sensor_class);
	unregister_chrdev(SENSOR_DEFAULT_MAJOR, "imapx200-sensor");

	return IMAPX200_SENSOR_RET_OK;
}


