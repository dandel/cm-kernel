/*
 * * Fake Battery driver for android
 * *
 * * Copyright © 2009 Rockie Cheng <aokikyon@gmail.com>
 * *
 * * This program is free software; you can redistribute it and/or modify
 * * it under the terms of the GNU General Public License version 2 as
 * * published by the Free Software Foundation.
 * */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
//#undef CONFIG_IMAP_PRODUCTION
#include <linux/spi/ads7846.h>
#include <mach/imapx_gpio.h>
#include <mach/imapx_base_reg.h>
#include <linux/io.h>
#include <asm/delay.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <mach/imapx_gpio.h>
#define BAT_STAT_PRESENT 0x01
#define BAT_STAT_FULL   0x02
#define BAT_STAT_LOW   0x04
#define BAT_STAT_DESTROY 0x08
//#define BAT_STAT_AC   0x10
#define BAT_STAT_CHARGING 0x20
#define BAT_STAT_DISCHARGING 0x40

#define BAT_ERR_INFOFAIL 0x02
#define BAT_ERR_OVERVOLTAGE 0x04
#define BAT_ERR_OVERTEMP 0x05
#define BAT_ERR_GAUGESTOP 0x06
#define BAT_ERR_OUT_OF_CONTROL 0x07
#define BAT_ERR_ID_FAIL   0x09
#define BAT_ERR_ACR_FAIL 0x10

#define BAT_ADDR_MFR_TYPE 0x5F
#define IMAP_ADAPTER 630
#define IMAP_BAT_FULL 591
//#define IMAP_BAT_CRITICAL 511
#define IMAP_BAT_CRITICAL 490

#define BATT_INITIAL 0
#define BATT_ON 1
#define BATT_AGAIN 2

static int battery_state=BATT_INITIAL;
static struct timer_list supply_timer;
static int batt, old_batt=750;
volatile static int adaptor_disconnect_num=0;
//volatile static int charging_status = 0;

//static int batt, old_batt;
static int ischargingfull(void)
{
	unsigned long tmp;
	tmp = (__raw_readl(rGPODAT)) & 0x100;
//	printk("ischaringfull is %d\n",tmp);
//	printk("batt is %d\n",batt);
	return tmp; 
}
static int android_ac_get_prop(struct power_supply *psy,
   enum power_supply_property psp,
   union power_supply_propval *val)
{

	switch (psp)
	{
		case POWER_SUPPLY_PROP_ONLINE:
                        if (!ischargingfull() && batt > 630 /* || (ischargingfull() && batt > 620 )*/)
                        {
                                val->intval = POWER_SUPPLY_STATUS_CHARGING;
                        }
                        else{
                                val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
                        }

//			val->intval = BAT_STAT_AC;
			break;
		default:
			break;
	}
	return 0;
}

static enum power_supply_property android_ac_props[] =
{
	POWER_SUPPLY_PROP_ONLINE,
};

static struct power_supply android_ac =
{
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = android_ac_props,
	.num_properties = ARRAY_SIZE(android_ac_props),
	.get_property = android_ac_get_prop,
};

static int android_bat_get_status(union power_supply_propval *val)
{
	val->intval = POWER_SUPPLY_STATUS_FULL;
#ifdef CONFIG_IMAP_PRODUCTION

	if (!ischargingfull()){

		val->intval = POWER_SUPPLY_STATUS_CHARGING;
	}else {
	
	if (old_batt >= (IMAP_BAT_FULL)) {
			val->intval = POWER_SUPPLY_STATUS_FULL; //ac
	}else {
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING; 
	}
	}

#endif
	return 0;
}

static int android_bat_get_capacity(union power_supply_propval *val)
{
        volatile int tmp_batt;
        tmp_batt = old_batt;
	val->intval = 100;
#ifdef CONFIG_IMAP_PRODUCTION

	if (!ischargingfull()){
		       val->intval = 30;
    	}
	else {
	
		if (tmp_batt < IMAP_BAT_CRITICAL+10)
		{
			val->intval = 0;

			}

	       	else	val->intval = (100*(tmp_batt - IMAP_BAT_CRITICAL))/(IMAP_BAT_FULL - IMAP_BAT_CRITICAL);
	
	
		if (val->intval >100 )
			val->intval = 100;
	}
//	printk("val->intval is %d\n",val->intval);
#endif
	return 0;
}


static int android_bat_get_health(union power_supply_propval *val)
{

	val->intval = POWER_SUPPLY_HEALTH_GOOD;
	return 0;
}

static int android_bat_get_mfr(union power_supply_propval *val)
{

	val->strval = "Rockie";
	return 0;
}

static int android_bat_get_tech(union power_supply_propval *val)
{
	val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
	return 0;
}

static int android_bat_get_property(struct power_supply *psy,
   enum power_supply_property psp,
   union power_supply_propval *val)
{
	int ret = 0;

	switch (psp)
	{
		case POWER_SUPPLY_PROP_STATUS:

			ret = android_bat_get_status(val);
			if (ret)
			  return ret;
			break;
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = BAT_STAT_PRESENT;
			break;

		case POWER_SUPPLY_PROP_HEALTH:
			ret = android_bat_get_health(val);
			break;
		case POWER_SUPPLY_PROP_MANUFACTURER:
			ret = android_bat_get_mfr(val);
			if (ret)
			  return ret;
			break;
		case POWER_SUPPLY_PROP_TECHNOLOGY:
			ret = android_bat_get_tech(val);
			if (ret)
			  return ret;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_AVG:
			val->intval = 3;
			break;
		case POWER_SUPPLY_PROP_CURRENT_AVG:
			val->intval = 3;
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			//val->intval = 100;
			android_bat_get_capacity(val);
			break;
		case POWER_SUPPLY_PROP_TEMP:
			val->intval = 50;
			break;
		case POWER_SUPPLY_PROP_TEMP_AMBIENT:
			val->intval = 50;
			break;
		case POWER_SUPPLY_PROP_CHARGE_COUNTER:
			val->intval = 10;
			break;
		case POWER_SUPPLY_PROP_SERIAL_NUMBER:
			break;
		default:
			ret = -EINVAL;
			break;
	}
	return ret;
}

static enum power_supply_property android_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TEMP_AMBIENT,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
};

/*********************************************************************
 * *   Initialisation
 * *********************************************************************/

static struct platform_device *bat_pdev;

static struct power_supply android_bat =
{
	.properties = android_bat_props,
	.num_properties = ARRAY_SIZE(android_bat_props),
	.get_property = android_bat_get_property,
	.use_for_apm = 1,
};
#ifdef CONFIG_IMAP_PRODUCTION
static void supply_timer_func(unsigned long unused)
{	
	batt = battery_val;


/*        if (!ischargingfull()){ //adaptor is connected
                adaptor_disconnect_num =0;
                charging_status = 1;
                 printk("chong dian\n");
                 printk("old_batt is %d\n",old_batt);
                printk("batt is %d\n",batt);
                printk("adaptor_disconnect_num is %d\n",adaptor_disconnect_num);
                printk("charging_status is %d\n",charging_status);
                if (old_batt < batt){
                        printk("zheng chang chong dian\nold_batt is %d\nbatt is %d\n",old_batt,batt);
                        old_batt = batt;
                        power_supply_changed(&android_bat);
                }
                else {
                        if (battery_state == BATT_INITIAL){
                                 printk("kai ji chong dian\nold_batt is %d\nbatt is %d\n",old_batt,batt);
                                old_batt = batt;
                                power_supply_changed(&android_bat);
                                battery_state = BATT_ON;
                        }else{
                                printk("****we have not sample the battery\n");
                                goto time_work;
                        }
                }
        }
        else {

        if(old_batt > batt && battery_state == BATT_INITIAL){//initializing
                charging_status = 0;
                printk("kai ji ,mei you chong dian\ncharging_status is %d\nold_batt is %d\nbatt is %d\n",charging_status,old_batt,batt);
                old_batt = batt;
                power_supply_changed(&android_bat);
                battery_state = BATT_ON;
                printk("battery_state = BATT_ON\n");
        }
        else if (charging_status == 1){
                 printk("adaptor_disconnect\n");
                if (adaptor_disconnect_num ==4){
                        battery_state = BATT_ON;
                        charging_status = 0;
                        adaptor_disconnect_num =0;
                }
                else {
                        if (old_batt > batt&&adaptor_disconnect_num>=3) {
                                 printk("old_batt is %d\nbatt is %d\n",old_batt,batt);
                                old_batt = batt;
                                power_supply_changed(&android_bat);
                        }
                        printk(" adaptor_disconnect_num is %d\n", adaptor_disconnect_num);
                        adaptor_disconnect_num ++;
                }
        }
        else if((old_batt - batt < 3) && (old_batt - batt >0)&&(battery_state = BATT_ON)) {//using battery
        printk("using battery\n");
        printk("old_batt is %d\nbatt is %d\n",old_batt,batt);
        old_batt = batt;
        charging_status = 0;
        power_supply_changed(&android_bat);
        }

        }
*/



       if (!ischargingfull()){//charging
		adaptor_disconnect_num =0;
		if (battery_state == BATT_INITIAL){
                                old_batt = batt;
                                power_supply_changed(&android_bat);
                                battery_state = BATT_ON;
                        }
	   	 else  {
			if (batt > IMAP_ADAPTER && battery_state != BATT_INITIAL){
	                old_batt = batt; 
			power_supply_changed(&android_bat);		
                }
                    /*  else { if (battery_state == BATT_INITIAL&&batt<610){
                                old_batt = batt;
                                power_supply_changed(&android_bat);
                                battery_state = BATT_ON;
                        }*/
                else { 
                        goto time_work;
                } 
		}
        }
	else {
        if(old_batt > batt && battery_state == BATT_INITIAL){//initializing
        	old_batt = batt;
                power_supply_changed(&android_bat); 
                battery_state = BATT_ON;
	}
        else if (((old_batt > IMAP_ADAPTER) || (batt > IMAP_ADAPTER)) && battery_state == BATT_ON){
	        old_batt = IMAP_BAT_FULL;
                battery_state = BATT_AGAIN; 
                goto time_work; 
	}
        else if ((battery_state == BATT_AGAIN)){ 
                if (adaptor_disconnect_num ==2){ 
                        battery_state = BATT_ON; 
                        adaptor_disconnect_num =0;
		}
                else { 
                        if (old_batt >= batt&&batt<625) {
		                old_batt = batt;
		                power_supply_changed(&android_bat);
                        }
                        adaptor_disconnect_num ++;
                }
                        }
        else if((old_batt - batt < 3) && (old_batt - batt >=0)&&(battery_state = BATT_ON)) {//using battery
      	old_batt = batt; 
        power_supply_changed(&android_bat);
        }
        } 
time_work:
	mod_timer(&supply_timer,\
		  jiffies + msecs_to_jiffies(2000));
}
#endif
static int __init android_bat_init(void)
{
	int ret = 0;
	unsigned long tmp;
	/**********************************************/
	//here, we need to know the power type, AC or Battery and detect the voltage each 5 second.
	//need to register a irq for pluging in or out the AC.
	//at last, need to monitor the voltage each 5 second.
	//now, we have a question, cannot display the AC icon.
	/***************************************************/
/*******************************************************/
#ifdef CONFIG_IMAP_PRODUCTION
	tmp = __raw_readl(rGPOCON);
	tmp &= ~(0x3<<16);
	__raw_writel(tmp, rGPOCON);
	tmp = __raw_readl(rGPOPUD);
	tmp &= ~(0x1<<8);
	__raw_writel(tmp, rGPOPUD);
	setup_timer(&supply_timer, supply_timer_func, 0);
	mod_timer(&supply_timer,\
		  jiffies + msecs_to_jiffies(3000));
#endif
	/*******************************/
	bat_pdev = platform_device_register_simple("battery", 0, NULL, 0);

	ret = power_supply_register(&bat_pdev->dev, &android_ac);
	if (ret)
	  goto ac_failed;

	android_bat.name = bat_pdev->name;

	ret = power_supply_register(&bat_pdev->dev, &android_bat);
	if (ret)
	  goto battery_failed;

	goto success;

	power_supply_unregister(&android_bat);
battery_failed:
	power_supply_unregister(&android_ac);
ac_failed:
	platform_device_unregister(bat_pdev);
success:
	return ret;
}

static void __exit android_bat_exit(void)
{
	power_supply_unregister(&android_bat);
	power_supply_unregister(&android_ac);
	platform_device_unregister(bat_pdev);
}

module_init(android_bat_init);
module_exit(android_bat_exit);

MODULE_AUTHOR("Rockie Cheng <aokikyon@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Fake Battery driver for android");
