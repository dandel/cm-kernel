/***************************************************************************** 
 * ** linux/drivers/i2c/busses/i2c-imapx200.c 
 * ** 
 * ** Copyright (c) 2009~2014 ShangHai Infotm Ltd all rights reserved. 
 * ** 
 * ** Use of Infotm's code is governed by terms and conditions 
 * ** stated in the accompanying licensing statement. 
 * ** 
 * ** Description: Main file of I2C Bus driver.
 * **
 * ** Author:
 * **     Alex Zhang   <alex.zhang@infotmic.com.cn>
 * **      
 * ** Revision History: 
 * ** ----------------- 
 * ** 1.1  01/08/2010  Alex Zhang   
 * *****************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <asm/irq.h>
#include <asm/io.h>

#include <plat/imapx.h>

#define IMAP_IIC_DEBUG

#define IMAPX200_IIC_BUS_NUM		2

#ifdef IMAP_IIC_DEBUG
#define printk(format, arg...)		\
	printk(KERN_INFO format, ##arg)	
#else
#define printk	do { \
				} while (0)	
			
#endif

enum imapx200_i2c_state
{
	IIC_CUR_IDLE,
	IIC_CUR_READ,
	IIC_CUR_WRITE
};

struct imapx200_i2c {
	spinlock_t		lock;
	wait_queue_head_t	wait;
	unsigned int		suspended:1;

	struct i2c_msg		*msg;
	unsigned int		msg_num;
	unsigned int		msg_idx;
	unsigned int		msg_ptr;

	unsigned short		CurReadCount;
	unsigned short 		CurWriteCount;
	unsigned short 		CurCmdCount;
	unsigned char		*pCurWrite;
	unsigned char		*pCurRead;

	unsigned int		tx_setup;
	unsigned int		irq;

	enum imapx200_i2c_state	state;
	unsigned long		clkrate;
	unsigned int		dirty_bit:1;
	unsigned int		MasterArbitration:1;
	unsigned int		IgnoreAck;

	void __iomem		*base_reg;
	struct clk		*clk;
	struct device		*dev;
	struct resource		*ioarea;
	struct i2c_adapter	adap;
};

/*imapx200_init_register
 *
 *initilize registers for setting up iic bus
 */
static int imapx200_i2c_init_register(struct imapx200_i2c *i2c)
{
	int retry = 3;
//	unsigned int addr = (i2c->msg->addr & 0x7f) << 1;
	unsigned int addr = i2c->msg->addr;
	struct timeval time;

//       printk(KERN_INFO "Entering %s\n",__func__);
	if(i2c->dirty_bit == 1)
	{
		dev_dbg(i2c->dev, "[IMAPX200_IIC] Initialization registers\n");
		
		__raw_writel(~IMAPX200_IIC_ENABLE, i2c->base_reg + rIC_ENABLE);

		while(1)
		{
			if(!(__raw_readl(i2c->base_reg + rIC_ENABLE_STATUS) & IMAPX200_IIC_ENABLE))
			{
				break;
			}
				
			retry--;

			if(retry <= 0)
			{
				dev_err(i2c->dev, "[IMAPX200_IIC] IIC device cannot be enable\n");
				return -1;
			}
			
//			msleep(100);
			udelay(1000);			
		}
#if 0	
		if (i2c->msg->flags & I2C_M_RD) 
		{
			addr |= 1;
		}

		if (i2c->msg->flags & I2C_M_REV_DIR_ADDR)
		{
			addr ^= 1;
		}
#endif
		__raw_writel(0,i2c->base_reg + rIC_CON);
		__raw_writel(addr, i2c->base_reg + rIC_TAR);
//		printk(KERN_INFO "slave address is 0x%x\n",addr);
		if(addr	== 0x50)
		{
			__raw_writel((IMAPX200_IIC_DEFAULT_SS_SCL_HCNT_SLOW & 0xfff), i2c->base_reg + rIC_SS_SCL_HCNT);
			__raw_writel((IMAPX200_IIC_DEFAULT_SS_SCL_LCNT_SLOW & 0xfff), i2c->base_reg + rIC_SS_SCL_LCNT);
		}	
		else
		{
			__raw_writel((IMAPX200_IIC_DEFAULT_SS_SCL_HCNT & 0xff), i2c->base_reg + rIC_SS_SCL_HCNT);
			__raw_writel((IMAPX200_IIC_DEFAULT_SS_SCL_LCNT & 0xff), i2c->base_reg + rIC_SS_SCL_LCNT);
		}
		__raw_writel((IMAPX200_IIC_DEFAULT_RX_TL & 0xff), i2c->base_reg + rIC_RX_TL);
		__raw_writel((IMAPX200_IIC_DEFAULT_TX_TL & 0xff), i2c->base_reg + rIC_TX_TL);
		__raw_writel((IMAPX200_IIC_DEFAULT_SDA & 0xff), i2c->base_reg + rIC_IGNORE_ACK0);
		__raw_writel((IMAPX200_STANDARD_SPEED | IMAPX200_MATER_MODE | IMAPX200_SLAVE_DISABLE), i2c->base_reg + rIC_CON);
		if(i2c->IgnoreAck == 1)
		{
			__raw_writel(__raw_readl(i2c->base_reg + rIC_IGNORE_ACK0) | IMAPX200_IGNOREACK, i2c->base_reg + rIC_IGNORE_ACK0);
		}
		else
		{
			__raw_writel(__raw_readl(i2c->base_reg + rIC_IGNORE_ACK0) & (~IMAPX200_IGNOREACK), i2c->base_reg + rIC_IGNORE_ACK0);
		}

		__raw_writel(__raw_readl(i2c->base_reg + rIC_CON) | IMAPX200_RESTART_ENABLE, i2c->base_reg + rIC_CON);
		__raw_writel(__raw_readl(i2c->base_reg + rIC_INTR_MASK) | IMAPX200_MASK_ALL_INT, i2c->base_reg + rIC_INTR_MASK);
		__raw_writel(IMAPX200_IIC_ENABLE, i2c->base_reg + rIC_ENABLE);

		i2c->dirty_bit = 0;

		i2c->state = IIC_CUR_IDLE;

		//printk(KERN_INFO "Leaving %s\n",__func__);
		dev_dbg(i2c->dev, "[IMAPX200_IIC] Registers initialize complete\n");
	}

	return 0;
}

/*imapx200_i2c_write_read
 *
 * for special iic device which donot accept stop signal between write command and read command
 */ 
static void imapx200_i2c_write_read(struct imapx200_i2c *i2c,
					struct i2c_msg *msg)
{
	struct i2c_msg	*write_msg = &msg[0];
	struct i2c_msg	*read_msg = &msg[1];
	unsigned char byte;
	int  txFifoEmptyNum, readCmdNum;	
	int i;

	while(__raw_readl(i2c->base_reg + rIC_RXFLR))
	{
		__raw_readl(i2c->base_reg + rIC_DATA_CMD);	//read dummy data
	}

	__raw_writel(__raw_readl(i2c->base_reg + rIC_INTR_MASK) & ~IMAPX200_IIC_TX_EMPTY, i2c->base_reg + rIC_INTR_MASK);

	i2c->CurWriteCount = write_msg->len;
	i2c->pCurWrite	= write_msg->buf;
	i2c->CurReadCount = read_msg->len;
	i2c->CurCmdCount = read_msg->len;
	i2c->pCurRead = read_msg->buf;

	while(i2c->CurWriteCount && i2c->MasterArbitration)
	{
		for(i=0;i<write_msg->len;i++)
		{
			byte = write_msg->buf[i];
			//		printk(KERN_INFO "[imapx200_i2c_write_read] Transmitted data is 0x%x\n",byte);
			__raw_writel((byte & 0xff), i2c->base_reg + rIC_DATA_CMD);
			while(__raw_readl(i2c->base_reg + rIC_TXFLR));
			i2c->CurWriteCount--;
		}

		i2c->msg_num--;
		i2c->msg_idx++;

		txFifoEmptyNum = IMAPX200_IIC_TX_FIFO_DEPTH - __raw_readl(i2c->base_reg + rIC_TXFLR);
		readCmdNum = i2c->CurCmdCount > txFifoEmptyNum ? txFifoEmptyNum : i2c->CurCmdCount;

		for(i = readCmdNum; i > 0; i--)
		{   
			__raw_writel(IMAPX200_IIC_READ_CMD, i2c->base_reg + rIC_DATA_CMD);
			i2c->CurCmdCount--;
		} 

		i2c->state = IIC_CUR_READ;
		__raw_writel(__raw_readl(i2c->base_reg + rIC_INTR_MASK) | IMAPX200_IIC_RX_FULL, i2c->base_reg + rIC_INTR_MASK);
	}

}

/*imapx200_i2c_read
 *
 *iic bus data read 
 */
static void imapx200_i2c_read(struct imapx200_i2c *i2c,
				struct i2c_msg *msg)
{
	int  txFifoEmptyNum, readCmdNum;	
	int i;

//       printk(KERN_INFO "Entering %s\n",__func__);
	while(__raw_readl(i2c->base_reg + rIC_RXFLR))
	{
		__raw_readl(i2c->base_reg + rIC_DATA_CMD);	//read dummy data
	}

	i2c->CurReadCount = msg->len;
	i2c->CurCmdCount = msg->len;
	i2c->pCurRead = msg->buf;

	txFifoEmptyNum = IMAPX200_IIC_TX_FIFO_DEPTH - __raw_readl(i2c->base_reg + rIC_TXFLR);
	readCmdNum = i2c->CurCmdCount > txFifoEmptyNum ? txFifoEmptyNum : i2c->CurCmdCount;

	for(i = readCmdNum; i > 0; i--)
	{   
		__raw_writel(IMAPX200_IIC_READ_CMD, i2c->base_reg + rIC_DATA_CMD);
		i2c->CurCmdCount--;
	} 

	__raw_writel(__raw_readl(i2c->base_reg + rIC_INTR_MASK) | IMAPX200_IIC_RX_FULL, i2c->base_reg + rIC_INTR_MASK);
//	__raw_writel(BIT_IIC , rINTMSK);
  //     printk(KERN_INFO "Leaving %s\n",__func__);
}

/*imapx200_i2c_write
 *
 *iic bus data write
 */
static int imapx200_i2c_write(struct imapx200_i2c *i2c,
				struct i2c_msg *msg)
{
    //   printk(KERN_INFO "Entering %s\n",__func__);
	__raw_writel(IMAPX200_IIC_TX_ABORT, i2c->base_reg + rIC_INTR_MASK);

	if(__raw_readl(i2c->base_reg + rIC_TXFLR))
	{
		dev_err(i2c->dev, "[IMAPX200_IIC:ERR] Tx FIFO not empty\n");
		return -1;
	}

	i2c->CurWriteCount = msg->len;
	i2c->pCurWrite	= msg->buf;

	__raw_writel(__raw_readl(i2c->base_reg + rIC_INTR_MASK) | IMAPX200_IIC_TX_EMPTY, i2c->base_reg + rIC_INTR_MASK);
//	__raw_writel(BIT_IIC , rINTMSK);
      // printk(KERN_INFO "Leavring %s\n",__func__);

	return 0;
}

/* imapx200_i2c_message_start
 *
 * put the start of a message onto the bus
*/

static void imapx200_i2c_message_start(struct imapx200_i2c *i2c,
				      struct i2c_msg *msg)
{
//	unsigned int addr = (msg->addr & 0x7f) << 1;
	
//       printk(KERN_INFO "Entering %s\n",__func__);
#if 0
	if (msg->flags & I2C_M_RD) 
	{
		addr |= 1;
	}

	if (msg->flags & I2C_M_REV_DIR_ADDR)
	{
		addr ^= 1;
	}

	__raw_writel(~IMAPX200_IIC_ENABLE, i2c->base_reg + rIC_ENABLE);

	__raw_writel(addr, i2c->base_reg + rIC_TAR);
	printk(KERN_INFO "slave address is 0x%x\n",addr);

	__raw_writel(IMAPX200_IIC_ENABLE, i2c->base_reg + rIC_ENABLE);
#endif
	if(i2c->state == IIC_CUR_IDLE)
	{
		if(i2c->dirty_bit == 1)
		{
			imapx200_i2c_init_register(i2c);
		}
		
		if(i2c->msg_num > 1)	
		{
			i2c->msg = &msg[1];
			imapx200_i2c_write_read(i2c, msg);
		}
		else
		{
			if(msg->flags & I2C_M_RD)
			{
				i2c->state = IIC_CUR_READ;
				i2c->msg = &msg[0];
				imapx200_i2c_read(i2c,&msg[0]);
			}
			else
			{
				i2c->state = IIC_CUR_WRITE;
				i2c->msg = &msg[0];
				imapx200_i2c_write(i2c,&msg[0]);
			}
		}
	}
	/* delay here to ensure the data byte has gotten onto the bus
	 * before the transaction is started */

  //     printk(KERN_INFO "Leaving %s\n",__func__);
	ndelay(i2c->tx_setup);
}

/* imapx200_i2c_set_master
 *
 * get the i2c bus for a master transaction
*/

static int imapx200_i2c_set_master(struct imapx200_i2c *i2c)
{
	unsigned long iicstat;
	int timeout = 400;

 //      printk(KERN_INFO "Entering %s\n",__func__);
	while (timeout-- > 0) {
		iicstat = readl(i2c->base_reg + rIC_TX_ABRT_SOURCE);

		if (!(iicstat & IMAPX200_IIC_LOST_ARBITRATION))
		{
			i2c->MasterArbitration = 1;
			return 0;
		}
		msleep(1);
	}
	
	i2c->MasterArbitration = 0;
	
//	imapx200_i2c_init_register(i2c);	
//	printk(KERN_INFO "Leaving %s\n",__func__);
	return -ETIMEDOUT;
}

/* imapx200_i2c_doxfer
 *
 * this starts an i2c transfer
*/

static int imapx200_i2c_doxfer(struct imapx200_i2c *i2c,
			      struct i2c_msg *msgs, int num)
{
	unsigned long timeout;
	int i;
	int ret;
	struct timeval time;

  //     printk(KERN_INFO "Entering %s\n",__func__);
	if (i2c->suspended)
		return -EIO;

	ret = imapx200_i2c_set_master(i2c);
	if (ret != 0) {
		dev_err(i2c->dev, "cannot get bus (error %d)\n", ret);
		ret = -EAGAIN;
		goto out;
	}

	spin_lock_irq(&i2c->lock);
	
	if(num == 3)
	{
		num = 1;
		i2c->IgnoreAck = 1;
	}
	else
	{
		i2c->IgnoreAck = 0;
	}


	i2c->msg_idx = 0;
	i2c->msg_num = num;
	i2c->msg     = msgs;
	i2c->msg_ptr = 0;
	imapx200_i2c_message_start(i2c, msgs);
/*
	for(i = 0; i < num; i++)
	{
		i2c->msg     = &msgs[i];
		i2c->msg_ptr = 0;
		imapx200_i2c_message_start(i2c, &msgs[i]);
	}
*/
	spin_unlock_irq(&i2c->lock);

	timeout = wait_event_timeout(i2c->wait, i2c->msg_num == 0, HZ);

	ret = i2c->msg_idx;

	/* having these next two as dev_err() makes life very
	 * noisy when doing an i2cdetect */

	if (timeout == 0)
	{
		i2c->state = IIC_CUR_IDLE;
		dev_dbg(i2c->dev, "timeout\n");
	}
	else if (ret != num)
		dev_err(i2c->dev, "incomplete xfer (%d)\n", ret);
//		dev_dbg(i2c->dev, "incomplete xfer (%d)\n", ret);

	/* ensure the stop has been through the bus */

//       printk(KERN_INFO "Leaving %s\n",__func__);
	msleep(1);

 out:
	return ret;
}

/* imapx200_i2c_xfer
*
* first port of call from the i2c bus code when an message needs
* transferring across the i2c bus.
*/

static int imapx200_i2c_xfer(struct i2c_adapter *adap,
		       struct i2c_msg *msgs, int num)
{
       struct imapx200_i2c *i2c = (struct imapx200_i2c *)adap->algo_data;
       int retry;
       int ret;

       //printk(KERN_INFO "Entering %s\n",__func__);
       for (retry = 0; retry < adap->retries; retry++) {
	       i2c->dirty_bit = 1;

	       ret = imapx200_i2c_doxfer(i2c, msgs, num);

	       if (ret != -EAGAIN)
		       return ret;

	       dev_dbg(i2c->dev, "Retrying transmission (%d)\n", retry);

	       msleep(100);
       }

//	printk(KERN_INFO "leaving %s\n",__func__);
       return -EREMOTEIO;
}
/* declare our i2c functionality */
static u32 imapx200_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_PROTOCOL_MANGLING;
}

/* i2c bus registration info */

static const struct i2c_algorithm imapx200_i2c_algorithm = {
	.master_xfer		= imapx200_i2c_xfer,
	.functionality		= imapx200_i2c_func,
};



/* imapx200_i2c_irq
 *
 * top level IRQ servicing routine
*/
static irqreturn_t imapx200_i2c_irq(int irqno, void *dev_id)
{
	struct imapx200_i2c *i2c = dev_id;
	unsigned char byte;
	int txFifoEmptyNum, writeNum;
	int rx_level, readNum;
	int i = 0;

//	printk(KERN_INFO "Entering %s\n",__func__);
	dev_dbg(i2c->dev, "[IMAPX200_IIC] Deal with interrupt thread\n");
		
	if(__raw_readl(i2c->base_reg + rIC_INTR_STAT) & (IMAPX200_IIC_TX_ABORT))
	{
		dev_err(i2c->dev, "[IMAPX200_IIC: ERR] iic thread TX_ABORT occur\n");
		dev_err(i2c->dev, "[IMAPX200_IIC: ERR] TX_ABORT source is %x\n",__raw_readl(i2c->base_reg + rIC_TX_ABRT_SOURCE));
		dev_err(i2c->dev, "[IMAPX200_IIC: ERR] slave address is %x\r\n",__raw_readl(i2c->base_reg + rIC_TAR));
		if(__raw_readl(i2c->base_reg + rIC_TX_ABRT_SOURCE) & (IMAPX200_IIC_LOST_ARBITRATION))
		{
			i2c->MasterArbitration = 0;

			__raw_writel(~(IMAPX200_IIC_RX_FULL | IMAPX200_IIC_TX_EMPTY), i2c->base_reg + rIC_INTR_MASK);

			dev_dbg(i2c->dev, "[IMAPX200_IIC: ERR] IIC Bus lost arbitration\n");
		}
		
		__raw_readl(i2c->base_reg + rIC_CLR_TX_ABRT);
	}

	if(i2c->state == IIC_CUR_WRITE)
	{
		txFifoEmptyNum = IMAPX200_IIC_TX_FIFO_DEPTH - (__raw_readl(i2c->base_reg + rIC_TXFLR));
		 writeNum = i2c->CurWriteCount > txFifoEmptyNum ? txFifoEmptyNum : i2c->CurWriteCount;

		 for(i = 0; i < writeNum; i++)
		 {
//			byte = *i2c->pCurWrite++;
			byte = i2c->msg->buf[i2c->msg_ptr++];
//			printk(KERN_INFO "Transmitted data is 0x%x\n",byte);
		 	__raw_writel((byte & 0xff), i2c->base_reg + rIC_DATA_CMD);
			i2c->CurWriteCount--;
		 }

		 if(i2c->CurWriteCount <= 0)
		 {
//			printk(KERN_INFO "data written over\n");
		 	__raw_readl(i2c->base_reg + rIC_CLR_INTR);
			__raw_writel(__raw_readl(i2c->base_reg + rIC_INTR_MASK) & (~IMAPX200_IIC_TX_EMPTY), i2c->base_reg + rIC_INTR_MASK);
			i2c->state = IIC_CUR_IDLE;
			i2c->msg_ptr = 0;
			i2c->msg = NULL;
			i2c->msg_idx++;
			i2c->msg_num--;

			wake_up(&i2c->wait);
		 }
	}

	if(i2c->state == IIC_CUR_READ)
	{
		rx_level = __raw_readl(i2c->base_reg + rIC_RXFLR);

		readNum = i2c->CurReadCount > rx_level ? rx_level : i2c->CurReadCount;
		for(i = readNum; i > 0; i--)
		{
			byte = __raw_readl(i2c->base_reg + rIC_DATA_CMD);
//			i2c->pCurRead = (byte & 0xff);
			i2c->msg->buf[i2c->msg_ptr++] = (byte & 0xff);
//			printk("[imapx200_i2c1_irq] data read is %x\r\n",byte & 0xff);
//			i2c->pCurRead++;
			i2c->CurReadCount--;

			if(i2c->CurCmdCount > 0)
			{
				__raw_writel(IMAPX200_IIC_READ_CMD, i2c->base_reg + rIC_DATA_CMD);
				i2c->CurCmdCount--;
			}
		}

		if(i2c->CurReadCount <= 0)
		{
			dev_dbg(i2c->dev, "i2c->CurReadCount is %d\n",i2c->CurReadCount);
			__raw_writel(0, i2c->base_reg + rIC_RX_TL);
		}

		if(i2c->CurReadCount <= 0)
		{
			__raw_readl(i2c->base_reg + rIC_CLR_INTR);
			__raw_writel(__raw_readl(i2c->base_reg + rIC_INTR_MASK) & (~IMAPX200_IIC_RX_FULL), i2c->base_reg + rIC_INTR_MASK);

			i2c->state = IIC_CUR_IDLE;
			i2c->msg_ptr = 0;
			i2c->msg = NULL;
			i2c->msg_idx++;
			i2c->msg_num--;

			wake_up(&i2c->wait);
		}
	}

//       printk(KERN_INFO "Leaving %s\n",__func__);
	return IRQ_HANDLED;
}


/*imapx200_i2c_init_gpio
 *
 *initilize gpio port fot i2c output
 */
static void imapx200_i2c_init_gpio(void)
{
	unsigned long tmp;

	tmp = __raw_readl(rGPCCON);
	tmp &= ~(0xf<<4);
	tmp |= 0xa<<4;
	__raw_writel(tmp, rGPCCON);
}


/* imapx200_i2c_init
 *
 * initialise the controller, set the IO lines and other SFRs
*/

static int imapx200_i2c_init(struct imapx200_i2c *i2c)
{
	i2c->dirty_bit = 1;

	imapx200_i2c_init_gpio();
	
//	imapx200_i2c_init_register(i2c);
	return 0;
}

/* imapx200_i2c_probe
 *
 * called by the bus driver when a suitable device is found
*/

static int imapx200_i2c_probe(struct platform_device *pdev)
{
	struct imapx200_i2c *i2c;
	struct resource *res;
	int ret;

//	printk(KERN_INFO "Entering %s\n",__func__);

	i2c = kzalloc(sizeof(struct imapx200_i2c), GFP_KERNEL);
	if (!i2c) {
		dev_err(&pdev->dev, "no memory for state\n");
		return -ENOMEM;
	}

	strlcpy(i2c->adap.name, "imapx200-iic1", sizeof(i2c->adap.name));
	i2c->adap.owner   = THIS_MODULE;
	i2c->adap.algo    = &imapx200_i2c_algorithm;
	i2c->adap.retries = 2;
	i2c->adap.class   = I2C_CLASS_HWMON | I2C_CLASS_SPD;
	i2c->tx_setup     = 50;

	spin_lock_init(&i2c->lock);
	init_waitqueue_head(&i2c->wait);

	/* find the clock and enable it */

	i2c->dev = &pdev->dev;
	i2c->clk = clk_get(NULL, "i2c1");
	if (IS_ERR(i2c->clk)) {
		dev_err(&pdev->dev, "cannot get clock\n");
		ret = -ENOENT;
		goto err_noclk;
	}

	dev_dbg(&pdev->dev, "clock source %p\n", i2c->clk);

	clk_enable(i2c->clk);

	/* map the registers */

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "cannot find IO resource\n");
		ret = -ENOENT;
		goto err_clk;
	}

	i2c->ioarea = request_mem_region(res->start, resource_size(res),
					 pdev->name);

	if (i2c->ioarea == NULL) {
		dev_err(&pdev->dev, "cannot request IO\n");
		ret = -ENXIO;
		goto err_clk;
	}

	i2c->base_reg = ioremap(res->start, resource_size(res));

	if (i2c->base_reg == NULL) {
		dev_err(&pdev->dev, "cannot map IO\n");
		ret = -ENXIO;
		goto err_ioarea;
	}

	dev_dbg(&pdev->dev, "registers %p (%p, %p)\n",
		i2c->base_reg, i2c->ioarea, res);

	/* setup info block for the i2c core */

	i2c->adap.algo_data = i2c;
	i2c->adap.dev.parent = &pdev->dev;

	/* initialise the i2c controller */

	ret = imapx200_i2c_init(i2c);
	if (ret != 0)
		goto err_iomap;

	/* find the IRQ for this unit (note, this relies on the init call to
	 * ensure no current IRQs pending
	 */

	i2c->irq = ret = platform_get_irq(pdev, 0);
	if (ret <= 0) {
		dev_err(&pdev->dev, "cannot find IRQ\n");
		goto err_iomap;
	}

	ret = request_irq(i2c->irq, imapx200_i2c_irq, IRQF_DISABLED,
			  dev_name(&pdev->dev), i2c);

	if (ret != 0) {
		dev_err(&pdev->dev, "cannot claim IRQ %d\n", i2c->irq);
		goto err_iomap;
	}

	/* Note, previous versions of the driver used i2c_add_adapter()
	 * to add the bus at any number. We now pass the bus number via
	 * the platform data, so if unset it will now default to always
	 * being bus 0.
	 */

	i2c->adap.nr = IMAPX200_IIC_BUS_NUM;

	ret = i2c_add_numbered_adapter(&i2c->adap);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to add bus to i2c core\n");
		return ret;
	}

	platform_set_drvdata(pdev, i2c);
	
//	printk(KERN_INFO "leaving %s\n",__func__);
	dev_info(&pdev->dev, "%s: IMAPX200 I2C adapter\n", dev_name(&i2c->adap.dev));
	return 0;

 err_irq:
	free_irq(i2c->irq, i2c);

 err_iomap:
	iounmap(i2c->base_reg);

 err_ioarea:
	release_resource(i2c->ioarea);
	kfree(i2c->ioarea);

 err_clk:
	clk_disable(i2c->clk);
	clk_put(i2c->clk);

 err_noclk:
	kfree(i2c);
	return ret;
}


/*imapx200_i2c_remove
 *
 *called when device is removed form bus
 */
  
static int imapx200_i2c_remove(struct platform_device *pdev)
{
	struct imapx200_i2c *i2c = platform_get_drvdata(pdev);

	i2c_del_adapter(&i2c->adap);
	free_irq(i2c->irq, i2c);

	clk_disable(i2c->clk);
	clk_put(i2c->clk);

	iounmap(i2c->base_reg);

	release_resource(i2c->ioarea);
	kfree(i2c->ioarea);
	kfree(i2c);

	return 0;
}

#ifdef CONFIG_PM
static int imapx200_i2c_suspend_late(struct platform_device *dev,
				    pm_message_t msg)
{
	struct imapx200_i2c *i2c = platform_get_drvdata(dev);
	i2c->suspended = 1;
	return 0;
}

static int imapx200_i2c_resume(struct platform_device *dev)
{
	struct imapx200_i2c *i2c = platform_get_drvdata(dev);

	i2c->suspended = 0;
	imapx200_i2c_init(i2c);

	return 0;
}

#else
#define imapx200_i2c_suspend_late NULL
#define imapx200_i2c_resume NULL
#endif

static struct platform_driver imapx200_i2c_driver = {
	.probe		= imapx200_i2c_probe,
	.remove		= imapx200_i2c_remove,
	.suspend	= imapx200_i2c_suspend_late,
	.resume		= imapx200_i2c_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "imapx200_iic1",
	},
};

static int __init imapx200_i2c_adap_init(void)
{
	return platform_driver_register(&imapx200_i2c_driver);
}
subsys_initcall(imapx200_i2c_adap_init);

static void __exit imapx200_i2c_adap_exit(void)
{
	platform_driver_unregister(&imapx200_i2c_driver);
}
module_exit(imapx200_i2c_adap_exit);

MODULE_DESCRIPTION("IMAPX200 I2C Bus driver");
MODULE_AUTHOR("Alex Zhang, <alex.zhang@infotmic.com.cn>");
MODULE_LICENSE("GPL");
