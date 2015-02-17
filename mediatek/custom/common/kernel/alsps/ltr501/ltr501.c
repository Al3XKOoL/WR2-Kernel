/* drivers/hwmon/mt6516/amit/tmd2771.c - TMD2771 ALS/PS driver
 * 
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>



#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>



#define POWER_NONE_MACRO MT65XX_POWER_NONE


#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
#include "ltr501.h"
//<Line> <add wakelock include> <20131113> <mickal.ma>
#include <linux/wakelock.h>


//LINE<JIRA_ID><DATE20131219><add PS Calibration>zenghaihui
static int g_ps_cali_flag = 0;
static int g_ps_base_value = 0;
static int g_tp_tpye_checked = 0;
extern char * synaptics_get_vendor_info(void);
static void ap3220_ps_cali_tp_check(void);
static void ap3220_ps_cali_set_threshold(void);
static void ap3220_ps_cali_start(void);

//<Line> <add p-sensor wake_lock> <20131113> <mickal.ma>
static struct wake_lock psensor_timer_lock;




/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/

#define LTR501_DEV_NAME   "LTR_501ALS"

/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk( APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(APS_TAG fmt, ##args)                 
/******************************************************************************
 * extern functions
*******************************************************************************/
	extern void mt_eint_mask(unsigned int eint_num);
	extern void mt_eint_unmask(unsigned int eint_num);
	extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
	extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
	extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
	extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
	extern void mt_eint_print_status(void);
/*----------------------------------------------------------------------------*/

static struct i2c_client *ltr501_i2c_client = NULL;

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id ltr501_i2c_id[] = {{LTR501_DEV_NAME,0},{}};
/*the adapter id & i2c address will be available in customization*/
static struct i2c_board_info __initdata i2c_ltr501={ I2C_BOARD_INFO("LTR_501ALS", (0x46>>1))};

//static unsigned short ltr501_force[] = {0x00, 0x46, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const ltr501_forces[] = { ltr501_force, NULL };
//static struct i2c_client_address_data ltr501_addr_data = { .forces = ltr501_forces,};
/*----------------------------------------------------------------------------*/
static int ltr501_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int ltr501_i2c_remove(struct i2c_client *client);
static int ltr501_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
/*----------------------------------------------------------------------------*/
static int ltr501_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int ltr501_i2c_resume(struct i2c_client *client);


static int ps_gainrange;
static int als_gainrange;

static int final_prox_val;
static int final_lux_val;

//LINE<JIRA_ID><DATE20140109><BUG_INFO>zenghaihui
#define ALS_QUEUE_LEN	4
static u16 g_als_value_queue[ALS_QUEUE_LEN]={0,0,0,0};
static u16 g_als_index = 0;
static u16 g_als_init_flag= 0;



/*----------------------------------------------------------------------------*/
typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;

/*----------------------------------------------------------------------------*/
struct ltr501_i2c_addr {    /*define a series of i2c slave address*/
    u8  write_addr;  
    u8  ps_thd;     /*PS INT threshold*/
};

/*----------------------------------------------------------------------------*/

struct ltr501_priv {
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct work_struct  eint_work;
    struct mutex lock;
	/*i2c address group*/
    struct ltr501_i2c_addr  addr;

     /*misc*/
    u16		    als_modulus;
    atomic_t    i2c_retry;
    atomic_t    als_debounce;   /*debounce time after enabling als*/
    atomic_t    als_deb_on;     /*indicates if the debounce is on*/
    atomic_t    als_deb_end;    /*the jiffies representing the end of debounce*/
    atomic_t    ps_mask;        /*mask ps: always return far away*/
    atomic_t    ps_debounce;    /*debounce time after enabling ps*/
    atomic_t    ps_deb_on;      /*indicates if the debounce is on*/
    atomic_t    ps_deb_end;     /*the jiffies representing the end of debounce*/
    atomic_t    ps_suspend;
    atomic_t    als_suspend;

    /*data*/
    u16         als;
    u16          ps;
    u8          _align;
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];

    atomic_t    als_cmd_val;    /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_cmd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val;     /*the cmd value can't be read, stored in ram*/
	atomic_t    ps_thd_val_high;     /*the cmd value can't be read, stored in ram*/
	atomic_t    ps_thd_val_low;     /*the cmd value can't be read, stored in ram*/
    ulong       enable;         /*enable mask*/
    ulong       pending_intr;   /*pending interrupt*/

    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
};

 struct PS_CALI_DATA_STRUCT
{
    int close;
    int far_away;
    int valid;
} ;

static struct PS_CALI_DATA_STRUCT ps_cali={0,0,0};
static int intr_flag_value = 0;


static struct ltr501_priv *ltr501_obj = NULL;
static struct platform_driver ltr501_alsps_driver;

/*----------------------------------------------------------------------------*/
static struct i2c_driver ltr501_i2c_driver = {	
	.probe      = ltr501_i2c_probe,
	.remove     = ltr501_i2c_remove,
	.detect     = ltr501_i2c_detect,
	.suspend    = ltr501_i2c_suspend,
	.resume     = ltr501_i2c_resume,
	.id_table   = ltr501_i2c_id,
	//.address_data = &ltr501_addr_data,
	.driver = {
		//.owner          = THIS_MODULE,
		.name           = LTR501_DEV_NAME,
	},
};


/* 
 * #########
 * ## I2C ##
 * #########
 */

// I2C Read
static int ltr501_i2c_read_reg(u8 regnum)
{
    u8 buffer[1],reg_value[1];
	int res = 0;
	
	buffer[0]= regnum;
	res = i2c_master_send(ltr501_obj->client, buffer, 0x1);
	if(res <= 0)
	{
		return res;
	}
	res = i2c_master_recv(ltr501_obj->client, reg_value, 0x1);
	if(res <= 0)
	{
		return res;
	}
	return reg_value[0];
}

// I2C Write
static int ltr501_i2c_write_reg(u8 regnum, u8 value)
{
	u8 databuf[2];    
	int res = 0;
   
	databuf[0] = regnum;   
	databuf[1] = value;
	res = i2c_master_send(ltr501_obj->client, databuf, 0x2);

	if (res < 0)
		return res;
	else
		return 0;
}

/* 
 * ###############
 * ## PS CONFIG ##
 * ###############

 */

static int ltr501_ps_set_thres()
{
	APS_FUN();

	int res;
	u8 databuf[2];
	
		struct i2c_client *client = ltr501_obj->client;
		struct ltr501_priv *obj = ltr501_obj;
	
	if(1 == ps_cali.valid)
	{
		databuf[0] = LTR501_PS_THRES_LOW_0; 
		databuf[1] = (u8)(ps_cali.far_away & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr501_ERR_I2C;
		}
		databuf[0] = LTR501_PS_THRES_LOW_1; 
		databuf[1] = (u8)((ps_cali.far_away & 0xFF00) >> 8);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr501_ERR_I2C;
		}
		databuf[0] = LTR501_PS_THRES_UP_0;	
		databuf[1] = (u8)(ps_cali.close & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr501_ERR_I2C;
		}
		databuf[0] = LTR501_PS_THRES_UP_1;	
		databuf[1] = (u8)((ps_cali.close & 0xFF00) >> 8);;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr501_ERR_I2C;
		}
	}
	else
	{
		databuf[0] = LTR501_PS_THRES_LOW_0; 
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr501_ERR_I2C;
		}
		databuf[0] = LTR501_PS_THRES_LOW_1; 
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low )>> 8) & 0x00FF);
		
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr501_ERR_I2C;
		}
		databuf[0] = LTR501_PS_THRES_UP_0;	
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr501_ERR_I2C;
		}
		databuf[0] = LTR501_PS_THRES_UP_1;	
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high) >> 8) & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr501_ERR_I2C;
		}
	
	}

	res = 0;
	return res;
	
	EXIT_ERR:
	APS_ERR("set thres: %d\n", res);
	return res;

}


static int ltr501_ps_enable(int gainrange)
{
	struct i2c_client *client = ltr501_obj->client;
	struct ltr501_priv *obj = ltr501_obj;
	u8 databuf[2];	
	int res;

	int error;
	int setgain;
    //APS_LOG("ltr501_ps_enable() ...start!\n");

	switch (gainrange) {
		case PS_RANGE1:
			setgain = MODE_PS_ON_Gain1;
			break;

		case PS_RANGE4:
			setgain = MODE_PS_ON_Gain4;
			break;

		case PS_RANGE8:
			setgain = MODE_PS_ON_Gain8;
			break;

		case PS_RANGE16:
			setgain = MODE_PS_ON_Gain16;
			break;

		default:
			setgain = MODE_PS_ON_Gain1;
			break;
	}

	//APS_LOG("LTR501_PS_CONTR==>%d!\n",setgain);

	error = ltr501_i2c_write_reg(LTR501_PS_CONTR, setgain); 
	if(error<0)
	{
	    APS_LOG("ltr501_ps_enable() error1\n");
	    return error;
	}
	
	mdelay(WAKEUP_DELAY);
    
	/* =============== 
	 * ** IMPORTANT **
	 * ===============
	 * Other settings like timing and threshold to be set here, if required.
 	 * Not set and kept as device default for now.
 	 */
   error = ltr501_i2c_write_reg(LTR501_PS_N_PULSES, 16);  // 2
	if(error<0)
    {
        APS_LOG("ltr501_ps_enable() error2\n");
	    return error;
	} 
	error = ltr501_i2c_write_reg(LTR501_PS_LED, 0x7b); //0x63
	if(error<0)
    {
        APS_LOG("ltr501_ps_enable() error3...\n");
	    return error;
	}

    //LINE<JIRA_ID><DATE20140107><set 50ms>zenghaihui
	error = ltr501_i2c_write_reg(LTR501_PS_MEAS_RATE, 0x00);
	if(error<0)
    {
        APS_LOG("ltr501_ps_enable() error4...\n");
	    return error;
	}


	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
		if(0 == obj->hw->polling_mode_ps)
		{		

			ltr501_ps_set_thres();
			
			databuf[0] = LTR501_INTERRUPT;	
			databuf[1] = 0x01;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return ltr501_ERR_I2C;
			}
	
			databuf[0] = LTR501_INTERRUPT_PERSIST;	
			databuf[1] = 0x20;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return ltr501_ERR_I2C;
			}
			mt_eint_unmask(CUST_EINT_ALS_NUM);
	
		}
	
 	APS_LOG("ltr501_ps_enable ...OK!\n");


 	
	return error;

	EXIT_ERR:
	APS_ERR("set thres: %d\n", res);
	return res;
}

// Put PS into Standby mode
static int ltr501_ps_disable(void)
{
	int error;
	struct ltr501_priv *obj = ltr501_obj;
		
	error = ltr501_i2c_write_reg(LTR501_PS_CONTR, MODE_PS_StdBy); 
	if(error<0)
 	    APS_LOG("ltr501_ps_disable ...ERROR\n");
 	else
        APS_LOG("ltr501_ps_disable ...OK\n");

	if(0 == obj->hw->polling_mode_ps)
	{
		cancel_work_sync(&obj->eint_work);
		mt_eint_mask(CUST_EINT_ALS_NUM);
	}
	
	return error;
}


static int ltr501_ps_read(void)
{
	int psval_lo, psval_hi, psdata;

	psval_lo = ltr501_i2c_read_reg(LTR501_PS_DATA_0);
	if (psval_lo < 0){
	    
	    APS_DBG("psval_lo error\n");
		psdata = psval_lo;
		goto out;
	}
	//APS_DBG("psval_lo = %d\n", psval_lo);	
	psval_hi = ltr501_i2c_read_reg(LTR501_PS_DATA_1);
	if (psval_hi < 0){
	    APS_DBG("psval_hi error\n");
		psdata = psval_hi;
		goto out;
	}
	//APS_DBG("psval_hi = %d\n", psval_hi);	
	
	psdata = ((psval_hi & 7)* 256) + psval_lo;
    //psdata = ((psval_hi&0x7)<<8) + psval_lo;
    APS_DBG("ps_rawdata = %d\n", psdata);
    
	out:
	final_prox_val = psdata;
	
	return psdata;
}

/* 
 * ################
 * ## ALS CONFIG ##
 * ################
 */

static int ltr501_als_enable(int gainrange)
{
	int error;

	if (gainrange == 1)
		error = ltr501_i2c_write_reg(LTR501_ALS_CONTR, MODE_ALS_ON_Range1);
	else if (gainrange == 2)
		error = ltr501_i2c_write_reg(LTR501_ALS_CONTR, MODE_ALS_ON_Range2);
	else
		error = -1;

	mdelay(WAKEUP_DELAY);

	/* =============== 
	 * ** IMPORTANT **
	 * ===============
	 * Other settings like timing and threshold to be set here, if required.
 	 * Not set and kept as device default for now.
 	 */
 	if(error<0)
 	    APS_LOG("ltr501_als_enable ...ERROR\n");
 	else
        APS_LOG("ltr501_als_enable ...OK\n");
        
	return error;
}


// Put ALS into Standby mode
static int ltr501_als_disable(void)
{
	int error;
	error = ltr501_i2c_write_reg(LTR501_ALS_CONTR, MODE_ALS_StdBy); 
	if(error<0)
 	    APS_LOG("ltr501_als_disable ...ERROR\n");
 	else
        APS_LOG("ltr501_als_disable ...OK\n");
	return error;
}

static int ltr501_als_read(int gainrange)
{
	int alsval_ch0_lo, alsval_ch0_hi, alsval_ch0;
	int alsval_ch1_lo, alsval_ch1_hi, alsval_ch1;
	int luxdata_int;
	int ratio;
	int vl_gainrange;
	int vl_index;
	int vl_old_cal_flag;
	int luxdata_int_temp;
	int luxdata_int_temp1;
	int luxdata_int_temp2;
	int luxdata_int_temp3;

	alsval_ch0_lo = ltr501_i2c_read_reg(LTR501_ALS_DATA_CH0_0);
	alsval_ch0_hi = ltr501_i2c_read_reg(LTR501_ALS_DATA_CH0_1);
	alsval_ch0 = (alsval_ch0_hi * 256) + alsval_ch0_lo;

	alsval_ch1_lo = ltr501_i2c_read_reg(LTR501_ALS_DATA_CH1_0);
	alsval_ch1_hi = ltr501_i2c_read_reg(LTR501_ALS_DATA_CH1_1);
	alsval_ch1 = (alsval_ch1_hi * 256) + alsval_ch1_lo;
	
    //APS_DBG("alsval_ch0=%d,  alsval_ch1=%d\n", alsval_ch0, alsval_ch1);

    
    //LINE<JIRA_ID><DATE20140107><als range check>zenghaihui
    vl_gainrange = gainrange;
    if((alsval_ch0 >0x1000) || (alsval_ch1>0x1000))
    {   
        vl_gainrange = ALS_RANGE2_64K;
    }
    else if((alsval_ch0 <0x20) && (alsval_ch1<0x10))
    {
        vl_gainrange = ALS_RANGE1_320;
    }

    if(vl_gainrange != gainrange)
    {
        APS_DBG("vl_gainrange = %d, gainrange=%d,  als_gainrange=%d\n", vl_gainrange, 
        gainrange, als_gainrange);
        
        ltr501_als_disable();
        mdelay(10);
        ltr501_als_enable(vl_gainrange);
        mdelay(10);
        als_gainrange = vl_gainrange;
        gainrange = vl_gainrange;

        for(vl_index = 0; vl_index <4; vl_index++)
        {
            alsval_ch0_lo = ltr501_i2c_read_reg(LTR501_ALS_DATA_CH0_0);
            alsval_ch0_hi = ltr501_i2c_read_reg(LTR501_ALS_DATA_CH0_1);
            alsval_ch0 = (alsval_ch0_hi * 256) + alsval_ch0_lo;
            
            alsval_ch1_lo = ltr501_i2c_read_reg(LTR501_ALS_DATA_CH1_0);
            alsval_ch1_hi = ltr501_i2c_read_reg(LTR501_ALS_DATA_CH1_1);
            alsval_ch1 = (alsval_ch1_hi * 256) + alsval_ch1_lo;
            
            //APS_DBG("switch range read: alsval_ch0=%d,  alsval_ch1=%d\n", alsval_ch0, alsval_ch1);
        }
    }

    


    if((alsval_ch1==0)||(alsval_ch0==0))
    {
        luxdata_int = 0;
        goto err;
    }
	ratio = alsval_ch1*100 / alsval_ch0;

    //LINE<JIRA_ID><DATE20140108><BUG_INFO>zenghaihui
    vl_old_cal_flag = 0;
    if(als_gainrange == ALS_RANGE2_64K)
    {
        vl_old_cal_flag = 1;
    }
    
    if(als_gainrange == ALS_RANGE1_320)
    {
        if((alsval_ch0 < 0x100) || (alsval_ch1 < 0x100))
        {
            vl_old_cal_flag = 1;
        }
    }

    
    if(vl_old_cal_flag)
    {
	if (ratio < 69){
		luxdata_int = ((1361 * alsval_ch0) - (1500 * alsval_ch1))/1000;
	}
	else if ((ratio >= 69) && (ratio < 100)){
		luxdata_int = ((570 * alsval_ch0) - (345 * alsval_ch1))/1000;
	}
	else {
		luxdata_int = 0;
	}
    }
    else
    {
        if (ratio < 69){
            luxdata_int = ((1361 * alsval_ch0) - (1500 * alsval_ch1))/1000;
        }
        else if ((ratio >= 69) && (ratio < 100)){
            luxdata_int = ((570 * alsval_ch0) - (345 * alsval_ch1))/1000;
        }
        else if ((ratio >= 100) && (ratio < 450)){
            luxdata_int = (17743 * alsval_ch0) - (-11059 * alsval_ch1);
            luxdata_int = luxdata_int / 10000;
        } else if ((ratio >= 450) && (ratio < 640)){
            luxdata_int = (37725 * alsval_ch0) - (13363 * alsval_ch1);
            luxdata_int = luxdata_int / 10000;
        } else if ((ratio >= 640) && (ratio < 850)){
            luxdata_int = (16900 * alsval_ch0) - (1690 * alsval_ch1);
            luxdata_int = luxdata_int / 10000;
        }
        else {
            luxdata_int = 0;
        }
    }

	// For Range1
	if (gainrange == ALS_RANGE1_320)
		luxdata_int = luxdata_int / 150;

        //LINE<JIRA_ID><DATE20140107><BUG_INFO>zenghaihui
        luxdata_int_temp = luxdata_int;
        luxdata_int_temp1 = (luxdata_int_temp)/8;
        
        luxdata_int_temp = luxdata_int%8;
        luxdata_int_temp2 = (luxdata_int_temp+3)/4;
        
        luxdata_int_temp3 = luxdata_int_temp%4;
        
        luxdata_int += (luxdata_int_temp1*10) + (luxdata_int_temp2*6) + (luxdata_int_temp3*4);

	// convert float to integer;
	//luxdata_int = luxdata_flt;
/*	if ((luxdata_flt - luxdata_int) > 0.5){
		luxdata_int = luxdata_int + 1;
	}
	else {
		luxdata_int = luxdata_flt;
	}*/
err:
	final_lux_val = luxdata_int;
	//APS_DBG("als_value_lux = 0x%x\n", luxdata_int);
	return luxdata_int;
}



/*----------------------------------------------------------------------------*/
int ltr501_get_addr(struct alsps_hw *hw, struct ltr501_i2c_addr *addr)
{
	/***
	if(!hw || !addr)
	{
		return -EFAULT;
	}
	addr->write_addr= hw->i2c_addr[0];
	***/
	return 0;
}


/*-----------------------------------------------------------------------------*/
void ltr501_eint_func(void)
{
	APS_FUN();

	struct ltr501_priv *obj = ltr501_obj;
	if(!obj)
	{
		return;
	}
	
	schedule_work(&obj->eint_work);
	//schedule_delayed_work(&obj->eint_work);
}



/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
int ltr501_setup_eint(struct i2c_client *client)
{
	APS_FUN();
	struct ltr501_priv *obj = (struct ltr501_priv *)i2c_get_clientdata(client);        

	ltr501_obj = obj;
	
	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, TRUE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);
	
	mt_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_TYPE, ltr501_eint_func, 0);

	mt_eint_unmask(CUST_EINT_ALS_NUM);
    return 0;
}


/*----------------------------------------------------------------------------*/
static void ltr501_power(struct alsps_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	//APS_LOG("power %s\n", on ? "on" : "off");

	if(hw->power_id != POWER_NONE_MACRO)
	{
		if(power_on == on)
		{
			APS_LOG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "TMD2771")) 
			{
				APS_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "TMD2771")) 
			{
				APS_ERR("power off fail!!\n");   
			}
		}
	}
	power_on = on;
}

/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
static int ltr501_check_and_clear_intr(struct i2c_client *client) 
{
//***
	APS_FUN();

	int res,intp,intl;
	u8 buffer[2];	
	u8 temp;
		//if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/	
		//	  return 0;
	
		buffer[0] = LTR501_ALS_PS_STATUS;
		res = i2c_master_send(client, buffer, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		res = i2c_master_recv(client, buffer, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		temp = buffer[0];
		//printk("yucong tmd2772_check_and_clear_intr status=0x%x\n", buffer[0]);
		res = 1;
		intp = 0;
		intl = 0;
		if(0 != (buffer[0] & 0x02))
		{
			res = 0;
			intp = 1;
		}
		if(0 != (buffer[0] & 0x08))
		{
			res = 0;
			intl = 1;		
		}
	
		if(0 == res)
		{
			if((1 == intp) && (0 == intl))
			{
				buffer[1] = buffer[0] & 0xfD;
				
			}
			else if((0 == intp) && (1 == intl))
			{
				buffer[1] = buffer[0] & 0xf7;
			}
			else
			{
				buffer[1] = buffer[0] & 0xf5;
			}
			buffer[0] = LTR501_ALS_PS_STATUS	;
			res = i2c_master_send(client, buffer, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			else
			{
				res = 0;
			}
		}
	
		return res;
	
	EXIT_ERR:
		APS_ERR("tmd2772_check_and_clear_intr fail\n");
		return 1;

}
/*----------------------------------------------------------------------------*/


/*yucong add for interrupt mode support MTK inc 2012.3.7*/
static int ltr501_check_intr(struct i2c_client *client) 
{
//	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	APS_FUN();

	int res,intp,intl;
	u8 buffer[2];

	//if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/  
	//    return 0;

	buffer[0] = LTR501_ALS_PS_STATUS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//APS_ERR("tmd2772_check_and_clear_intr status=0x%x\n", buffer[0]);
	res = 1;
	intp = 0;
	intl = 0;
	if(0 != (buffer[0] & 0x02))
	{
		res = 0;
		intp = 1;
	}
	if(0 != (buffer[0] & 0x08))
	{
		res = 0;
		intl = 1;		
	}

	return res;

EXIT_ERR:
	APS_ERR("tmd2772_check_intr fail\n");
	return 1;
}

static int ltr501_clear_intr(struct i2c_client *client) 
{
//	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	int res;
	u8 buffer[2];

	APS_FUN();
	
	buffer[0] = LTR501_ALS_PS_STATUS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	buffer[1] = buffer[0] & 0xf5;
	buffer[0] = LTR501_ALS_PS_STATUS	;

	res = i2c_master_send(client, buffer, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	else
	{
		res = 0;
	}

	return res;

EXIT_ERR:
	APS_ERR("tmd2772_check_and_clear_intr fail\n");
	return 1;
}




static int ltr501_devinit(void)
{
	int res;
	int init_ps_gain;
	int init_als_gain;
	u8 databuf[2];	

	struct i2c_client *client = ltr501_obj->client;

	struct ltr501_priv *obj = ltr501_obj;   
	
	mdelay(PON_DELAY);

	// Enable PS to Gain1 at startup
	init_ps_gain = PS_RANGE16; // PS_RANGE1;
	ps_gainrange = init_ps_gain;

	res = ltr501_ps_enable(init_ps_gain);
	if (res < 0)
		goto EXIT_ERR;


	// Enable ALS to Full Range at startup
	//LINE<JIRA_ID><DATE20140107><als range check>zenghaihui
	init_als_gain = ALS_RANGE1_320; // ALS_RANGE2_64K;
	als_gainrange = init_als_gain;

	res = ltr501_als_enable(init_als_gain);
	if (res < 0)
		goto EXIT_ERR;


	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
	if(0 == obj->hw->polling_mode_ps)
	{	
		APS_LOG("eint enable");
		ltr501_ps_set_thres();
		
		databuf[0] = LTR501_INTERRUPT;	
		databuf[1] = 0x01;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr501_ERR_I2C;
		}

		databuf[0] = LTR501_INTERRUPT_PERSIST;	
		databuf[1] = 0x20;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr501_ERR_I2C;
		}

	}

	if((res = ltr501_setup_eint(client))!=0)
	{
		APS_ERR("setup eint: %d\n", res);
		return res;
	}
	
	if((res = ltr501_check_and_clear_intr(client)))
	{
		APS_ERR("check/clear intr: %d\n", res);
		//    return res;
	}

	res = 0;

	EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;

}
/*----------------------------------------------------------------------------*/


static int ltr501_get_als_value(struct ltr501_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;

    //LINE<JIRA_ID><DATE20140109><BUG_INFO>zenghaihui
    u32 als_sum;

    als_sum = 0;

    if(!g_als_init_flag)
    {
        for (idx = 0; idx < (ALS_QUEUE_LEN); idx++)
        {
            g_als_value_queue[idx] = als;
        }

        g_als_init_flag = 1;
    }

    g_als_value_queue[g_als_index] = als;

    g_als_index = (g_als_index+1)%(ALS_QUEUE_LEN);
    

    for (idx = 0; idx < ALS_QUEUE_LEN; idx++)
    {
        als_sum+= g_als_value_queue[idx];
    }

    if (als_sum > 0)
    {
        als_sum /= 4;
    }
    else
    {
        als_sum = 0;
    }

    als = als_sum;
        
    
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(als < obj->hw->als_level[idx])
		{
			break;
		}
	}
	
	if(idx >= obj->als_value_num)
	{
		APS_ERR("exceed range\n"); 
		idx = obj->als_value_num - 1;
	}
	
	if(1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->als_deb_on, 0);
		}
		
		if(1 == atomic_read(&obj->als_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		//APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);	
		return obj->hw->als_value[idx];
	}
	else
	{
		APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);    
		return -1;
	}
}
/*----------------------------------------------------------------------------*/
static int ltr501_get_ps_value(struct ltr501_priv *obj, u16 ps)
{
	int val,  mask = atomic_read(&obj->ps_mask);
	int invalid = 0;

	static int val_temp = 1;
	if((ps > atomic_read(&obj->ps_thd_val_high)))
	{
		val = 0;  /*close*/
		val_temp = 0;
		intr_flag_value = 1;
	}
			//else if((ps < atomic_read(&obj->ps_thd_val_low))&&(temp_ps[0]  < atomic_read(&obj->ps_thd_val_low)))
	else if((ps < atomic_read(&obj->ps_thd_val_low)))
	{
		val = 1;  /*far away*/
		val_temp = 1;
		intr_flag_value = 0;
	}
	else
		val = val_temp;	
			
	
	if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}
	else if(1 == atomic_read(&obj->ps_deb_on))
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		if (1 == atomic_read(&obj->ps_deb_on))
		{
			invalid = 1;
		}
	}
	else if (obj->als > 50000)
	{
		//invalid = 1;
		APS_DBG("ligh too high will result to failt proximiy\n");
		return 1;  /*far away*/
	}

	if(!invalid)
	{
		//APS_DBG("PS:  %05d => %05d\n", ps, val);
		return val;
	}	
	else
	{
		return -1;
	}	
}

/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
static void ltr501_eint_work(struct work_struct *work)
{
	struct ltr501_priv *obj = (struct ltr501_priv *)container_of(work, struct ltr501_priv, eint_work);
	int err;
	hwm_sensor_data sensor_data;
//	u8 buffer[1];
//	u8 reg_value[1];
	u8 databuf[2];
	int res = 0;
	APS_FUN();
	if((err = ltr501_check_intr(obj->client)))
	{
		APS_ERR("tmd2772_eint_work check intrs: %d\n", err);
	}
	else
	{
		//get raw data
		obj->ps = ltr501_ps_read();
    	if(obj->ps < 0)
    	{
    		err = -1;
    		return;
    	}
				
		APS_DBG("tmd2772_eint_work rawdata ps=%d als_ch0=%d!\n",obj->ps,obj->als);
		//printk("tmd2772_eint_work rawdata ps=%d als_ch0=%d!\n",obj->ps,obj->als);
		sensor_data.values[0] = ltr501_get_ps_value(obj, obj->ps);
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;			

        
        //<Begin> <fix problem when phone talking> <20131113> <mickal.ma>
        if(sensor_data.values[0]==1)//far away
            wake_lock_timeout(&psensor_timer_lock, 1*HZ);
        //<End> <fix problem when phone talking> <20131113> <mickal.ma>
        
/*singal interrupt function add*/
#if 1
		if(intr_flag_value){
				//printk("yucong interrupt value ps will < 750");

				databuf[0] = LTR501_PS_THRES_LOW_0;	
				databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR501_PS_THRES_LOW_1;	
				databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_low)) & 0xFF00) >> 8);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR501_PS_THRES_UP_0;	
				databuf[1] = (u8)(0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR501_PS_THRES_UP_1; 
				databuf[1] = (u8)((0xFF00) >> 8);;
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
		}
		else{	
				//printk("yucong interrupt value ps will > 900");
				databuf[0] = LTR501_PS_THRES_LOW_0;	
				databuf[1] = (u8)(0 & 0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR501_PS_THRES_LOW_1;	
				databuf[1] = (u8)((0 & 0xFF00) >> 8);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR501_PS_THRES_UP_0;	
				databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR501_PS_THRES_UP_1; 
				databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_high)) & 0xFF00) >> 8);;
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
		}
#endif
		//let up layer to know
		if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
		{
		  APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
		}
	}
	ltr501_clear_intr(obj->client);
	mt_eint_unmask(CUST_EINT_ALS_NUM);      
}



/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int ltr501_open(struct inode *inode, struct file *file)
{
	file->private_data = ltr501_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int ltr501_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/

//LINE<JIRA_ID><DATE20131219><add PS Calibration>zenghaihui
static void ap3220_ps_cali_tp_check(void)
{
#if 0
    u8 data, data2, data3;
    int res =0;
    char *product_id = NULL;

    if(g_tp_tpye_checked)
    {
        APS_LOG("ap3220_ps_tp_check tp already checked \n");
        return;
    }
    
    product_id = synaptics_get_vendor_info();
    
    APS_LOG("ap3220_ps_tp_check product_id = %s \n", product_id);
    
    if( 0 == memcmp(product_id, "JTOUCH", 6))
    {
        //PS LED Control
        data = (AP3220_PS_SETTING_LED_PULSE_2<< AP3220_PS_LED_PULSE_SHIFT) & AP3220_PS_LED_PULSE_MASK;
        data2 = (AP3220_PS_SETTING_LED_RATIO_16 << AP3220_PS_LED_RATIO_SHIFT) & AP3220_PS_LED_RATIO_MASK;
        data |= data2;
        res = ap3220_i2c_write_reg(AP3220_REG_PS_LED,data);
        
        if(res < 0)
        {
            APS_LOG("i2c_master_send function err in ap3220_jtouch_white_tp_threshold_reset\n");
        }
        
        mdelay(50);
    }

    g_tp_tpye_checked = 1;
#endif    
}

static void ap3220_ps_cali_set_threshold(void)
{
	u8 data, data2, data3;
	u16 value_high,value_low;
	int res =0;

    struct alsps_hw *hw = get_cust_alsps_hw();

    APS_LOG("ap3220_ps_cali_set_threshold:g_ps_base_value=%x, hw->ps_threshold_high=%x,hw->ps_threshold_low=%x \n",
        g_ps_base_value, hw->ps_threshold_high, hw->ps_threshold_low);
    
    value_high= g_ps_base_value + hw->ps_threshold_high;
    value_low= g_ps_base_value + hw->ps_threshold_low;

    
    if( value_high > 0x7f0)
    {
        value_high= 0x7f0;
        value_low= 0x7e0;
        APS_LOG("ap3220_ps_cali_set_threshold: set value_high=0x3f0,value_low=0x3e0, please check the phone \n");
    }
    

        
	atomic_set(&ltr501_obj->ps_thd_val_high, value_high);
	atomic_set(&ltr501_obj->ps_thd_val_low,  value_low);


        ltr501_ps_set_thres();
        /*
	res = ap3220_i2c_write_reg(0x2A,value_low&0x0003);
	res = ap3220_i2c_write_reg(0x2B,(value_low>>2)&0x00ff);
	res = ap3220_i2c_write_reg(0x2C,value_high&0x0003);
	res = ap3220_i2c_write_reg(0x2D,(value_high>>2)&0x00ff);
	*/

	APS_LOG("ap3220_ps_cali_set_threshold:value_high=%x,value_low=%x! \n",value_high,value_low);
}

static void ap3220_ps_cali_start(void)
{
    long err = 0;
    u16 			vl_read_ps = 0;
    u16 			vl_ps_count = 0;
    u16 			vl_ps_sun = 0;
    u16 			vl_index = 0;
    
    

    APS_LOG("entry ap3220_ps_cali_start \n");
    
    if(NULL == ltr501_obj->client)
    {
        APS_ERR("ltr501_obj->client == NULL\n"); 
        return;
    }

    
    //ap3220_ps_cali_tp_check();

    // enable ps and backup reg data
    /*
    if((err = ap3220_enable_ps(ltr501_obj->client, 1)))
    {
        APS_ERR("enable ps fail: %ld\n", err); 
        goto exit_handle;
    }
    mdelay(50);
    */


    // read ps
    for(vl_index = 0; vl_index < 4; vl_index++)
    {
        vl_read_ps = ltr501_ps_read();
        
        APS_LOG("vl_index=%d, vl_read_ps = %d \n",vl_index, vl_read_ps);

        if(vl_index >=2)
        {
            vl_ps_sun += vl_read_ps;
            
            vl_ps_count ++;
        }
        
        vl_read_ps = 0;
        
        mdelay(30);
    }

    g_ps_base_value = (vl_ps_sun/vl_ps_count);
    g_ps_cali_flag = 1;
    
    APS_LOG("ap3220_ps_cali_start:g_ps_base_value=%x \n",g_ps_base_value);
    
    
exit_handle:
	APS_LOG("tag: exit_handle\n");
    /*
    if((err = ap3220_enable_ps(ltr501_obj->client, 0)))
    {
        APS_ERR("disable ps fail: %d\n", err); 
    }
    */
    
}


//LINE<JIRA_ID><DATE20140217><wallpaper check for ms color>zenghaihui
#ifdef TINNO_MS_COLOR_SELECT
extern int phone_color_write_nvram(int *dat);
extern int phone_color_read_nvram(int *dat);

static int g_phone_color_data = 0;
static int g_factory_reset_flag = 0;

static ssize_t ltr501_show_phone_color(struct device_driver *ddri, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", g_phone_color_data);
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr501_store_phone_color(struct device_driver *ddri, const char *buf, size_t count)
{
    int phone_color, res;
    int phone_color_nv_array[4];
	
	if(1 == sscanf(buf, "%d", &phone_color))
	{
		if(phone_color > 0 && phone_color < 8)
		{
		    g_phone_color_data = phone_color;
            
                // read ps cali nvram
                //phone_color_read_nvram(&phone_color_nv_array);

                //phone_color_nv_array[0]  --> ps cali data
                //phone_color_nv_array[1]  --> ps cali data
                
                //phone_color_nv_array[2]  --> phone color data
                //phone_color_nv_array[3]  --> factory reset flag
                phone_color_nv_array[2] = phone_color;
                //phone_color_write_nvram(&phone_color_nv_array);
		}
              else
              {
                  APS_ERR("invalid phone_color  =  \n", phone_color);
              }
	}
	else 
	{
		APS_ERR("invalid enable content: '%s', length = %d\n", buf, count);
	}
	return count;    
}
static DRIVER_ATTR(phone_color,    0444, ltr501_show_phone_color, NULL);


static ssize_t ltr501_show_factory_reset_flag(struct device_driver *ddri, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", g_factory_reset_flag);
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr501_store_factory_reset_flag(struct device_driver *ddri, const char *buf, size_t count)
{
    int vl_factory_reset_flag;
    int phone_color_nv_array[4];
	
	if(1 == sscanf(buf, "%d", &vl_factory_reset_flag))
	{
	    g_factory_reset_flag = vl_factory_reset_flag;
        
            // read ps cali nvram
            //phone_color_read_nvram(&phone_color_nv_array);

            //phone_color_nv_array[0]  --> ps cali data
            //phone_color_nv_array[1]  --> ps cali data
            
            //phone_color_nv_array[2]  --> phone color data
            //phone_color_nv_array[3]  --> factory reset flag
            phone_color_nv_array[3] = vl_factory_reset_flag;
            //phone_color_write_nvram(&phone_color_nv_array);

	}
	else 
	{
		APS_ERR("invalid enable content: '%s', length = %d\n", buf, count);
	}
	return count;    
}
static DRIVER_ATTR(factory_reset_flag,    0444, ltr501_show_factory_reset_flag, NULL);
/*----------------------------------------------------------------------------*/



/*----------------------------------------------------------------------------*/
static struct driver_attribute *ltr501_attr_list[] = {
    &driver_attr_phone_color,
    &driver_attr_factory_reset_flag,
};

/*----------------------------------------------------------------------------*/
static int ltr501_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(ltr501_attr_list)/sizeof(ltr501_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, ltr501_attr_list[idx])))
		{            
			APS_ERR("driver_create_file (%s) = %d\n", ltr501_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
	static int ltr501_delete_attr(struct device_driver *driver)
	{
	int idx ,err = 0;
	int num = (int)(sizeof(ltr501_attr_list)/sizeof(ltr501_attr_list[0]));

	if (!driver)
	return -EINVAL;

	for (idx = 0; idx < num; idx++) 
	{
		driver_remove_file(driver, ltr501_attr_list[idx]);
	}
	
	return err;
}
#endif


//static int ltr501_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
  //     unsigned long arg)
static int ltr501_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)       
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct ltr501_priv *obj = i2c_get_clientdata(client);  
	int err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;

            //LINE<JIRA_ID><DATE20131219><add PS Calibration>zenghaihui        
            int ps_cali_data[2] = {0x00};
            
//LINE<JIRA_ID><DATE20140217><wallpaper check for ms color>zenghaihui
#ifdef TINNO_MS_COLOR_SELECT
            int phone_color_data[2] = {0x00};
#endif            
        
        
	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
			    err = ltr501_ps_enable(ps_gainrange);
				if(err < 0)
				{
					APS_ERR("enable ps fail: %d\n", err); 
					goto err_out;
				}
				set_bit(CMC_BIT_PS, &obj->enable);
			}
			else
			{
			    err = ltr501_ps_disable();
				if(err < 0)
				{
					APS_ERR("disable ps fail: %d\n", err); 
					goto err_out;
				}
				
				clear_bit(CMC_BIT_PS, &obj->enable);
			}
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:
		    obj->ps = ltr501_ps_read();
			if(obj->ps < 0)
			{
				goto err_out;
			}
			
			dat = ltr501_get_ps_value(obj, obj->ps);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_GET_PS_RAW_DATA:    
			obj->ps = ltr501_ps_read();
			if(obj->ps < 0)
			{
				goto err_out;
			}
			dat = obj->ps;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
			    err = ltr501_als_enable(als_gainrange);
				if(err < 0)
				{
					APS_ERR("enable als fail: %d\n", err); 
					goto err_out;
				}
				set_bit(CMC_BIT_ALS, &obj->enable);
			}
			else
			{
			    err = ltr501_als_disable();
				if(err < 0)
				{
					APS_ERR("disable als fail: %d\n", err); 
					goto err_out;
				}
				clear_bit(CMC_BIT_ALS, &obj->enable);
			}
			break;

		case ALSPS_GET_ALS_MODE:
			enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA: 
		    obj->als = ltr501_als_read(als_gainrange);
			if(obj->als < 0)
			{
				goto err_out;
			}

			dat = ltr501_get_als_value(obj, obj->als);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		case ALSPS_GET_ALS_RAW_DATA:    
			obj->als = ltr501_als_read(als_gainrange);
			if(obj->als < 0)
			{
				goto err_out;
			}

			dat = obj->als;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;


            //LINE<JIRA_ID><DATE20131218><add PS Calibration>zenghaihui
            case ALSPS_IOCTL_PS_CALI_START:
                APS_LOG("case ALSPS_IOCTL_PS_CALI_START: \n");
                
                ap3220_ps_cali_start();
                
                if (ptr == NULL) {
                    APS_ERR("%s ptr == NULL", __FUNCTION__);
                    err = -EINVAL;
                    break;
                }
                
                ps_cali_data[0] = g_ps_cali_flag;
                ps_cali_data[1] = g_ps_base_value;
                
                APS_LOG("g_ps_cali_flag = %x, g_ps_base_value = %x \n", g_ps_cali_flag, g_ps_base_value);
                
                if (copy_to_user(ptr, ps_cali_data, sizeof(ps_cali_data))) {
                    APS_ERR("%s copy_from_user error", __FUNCTION__);
                    err = -EFAULT;
                    break;
                }
                break;

            case ALSPS_IOCTL_PS_SET_CALI:
                APS_LOG("case ALSPS_IOCTL_PS_SET_CALI: \n");
                
                if (ptr == NULL) {
                    APS_ERR("%s ptr == NULL", __FUNCTION__);
                    err = -EINVAL;
                    break;
                }
                
                if (copy_from_user(&ps_cali_data, ptr, sizeof(ps_cali_data))) {
                    APS_ERR("%s copy_from_user error", __FUNCTION__);
                    err = -EFAULT;
                    break;
                }

                g_ps_cali_flag = ps_cali_data[0];
                g_ps_base_value = ps_cali_data[1];

                if(!g_ps_cali_flag)
                {
                    g_ps_base_value = 0x90; // set default base value
                    APS_LOG("not calibration!!! set g_ps_base_value = 0x80 \n");
                }
                
                APS_LOG("g_ps_cali_flag = %x, g_ps_base_value = %x \n", g_ps_cali_flag, g_ps_base_value);

                ap3220_ps_cali_set_threshold();
                
                break;

            case ALSPS_IOCTL_PS_GET_CALI:
                APS_LOG("case ALSPS_IOCTL_PS_GET_CALI: \n");
                
                if (ptr == NULL) {
                    APS_ERR("%s ptr == NULL", __FUNCTION__);
                    err = -EINVAL;
                    break;
                }
                
                ps_cali_data[0] = g_ps_cali_flag;
                ps_cali_data[1] = g_ps_base_value;
                
                APS_LOG("g_ps_cali_flag = %x, g_ps_base_value = %x \n", g_ps_cali_flag, g_ps_base_value);
                
                if (copy_to_user(ptr, ps_cali_data, sizeof(ps_cali_data))) {
                    APS_ERR("%s copy_to_user error", __FUNCTION__);
                    err = -EFAULT;
                    break;
                }
                break;

            case ALSPS_IOCTL_PS_CLR_CALI:
                APS_LOG("case ALSPS_IOCTL_PS_CLR_CALI: \n");
                g_ps_cali_flag = 0;
                g_ps_base_value = 0;
                ap3220_ps_cali_set_threshold();
                break;
                
            case ALSPS_IOCTL_PS_CALI_RAW_DATA:    
                obj->ps = ltr501_ps_read();
                
                dat = obj->ps;
                            
                if(copy_to_user(ptr, &dat, sizeof(dat)))
                {
                    err = -EFAULT;
                    goto err_out;
                }  
                break;        


        
//LINE<JIRA_ID><DATE20140217><wallpaper check for ms color>zenghaihui
#ifdef TINNO_MS_COLOR_SELECT
        case ALSPS_IOCTL_PHONE_COLOR_SET_CALI:
            APS_LOG("case ALSPS_IOCTL_PHONE_COLOR_SET_CALI: \n");
            
            if (ptr == NULL) {
                APS_ERR("%s ptr == NULL", __FUNCTION__);
                err = -EINVAL;
                break;
            }
            
            if (copy_from_user(&phone_color_data, ptr, sizeof(phone_color_data))) {
                APS_ERR("%s copy_from_user error", __FUNCTION__);
                err = -EFAULT;
                break;
            }
        
            g_phone_color_data = phone_color_data[0];
            g_factory_reset_flag = phone_color_data[1];
        
            
            APS_LOG("g_phone_color_data = %x, g_factory_reset_flag = %x \n", g_phone_color_data, g_factory_reset_flag);
        
            if((err = ltr501_create_attr(&ltr501_alsps_driver.driver)))
            {
                APS_ERR("create attribute err = %d\n", err);
                goto err_out;
            }
            
            break;
#endif            
        
		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;    
}

/*----------------------------------------------------------------------------*/
static struct file_operations ltr501_fops = {
	//.owner = THIS_MODULE,
	.open = ltr501_open,
	.release = ltr501_release,
	.unlocked_ioctl = ltr501_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice ltr501_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &ltr501_fops,
};

static int ltr501_i2c_suspend(struct i2c_client *client, pm_message_t msg) 
{
//LINE<SSSOOBP-124><DATE20140115><通话时贴近距离感应器5s以上再离开，屏幕要5s以上才会被点亮>zenghaihui
#if 1
	APS_FUN();
	return 0;
#else
	struct ltr501_priv *obj = i2c_get_clientdata(client);    
	int err;
	APS_FUN();    

	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(!obj)
		{
			APS_ERR("null pointer!!\n");
			return -EINVAL;
		}
		
		atomic_set(&obj->als_suspend, 1);
		err = ltr501_als_disable();
		if(err < 0)
		{
			APS_ERR("disable als: %d\n", err);
			return err;
		}

		atomic_set(&obj->ps_suspend, 1);
		err = ltr501_ps_disable();
		if(err < 0)
		{
			APS_ERR("disable ps:  %d\n", err);
			return err;
		}
		
		ltr501_power(obj->hw, 0);
	}
	return 0;
#endif    
}
/*----------------------------------------------------------------------------*/
static int ltr501_i2c_resume(struct i2c_client *client)
{
//LINE<SSSOOBP-124><DATE20140115><通话时贴近距离感应器5s以上再离开，屏幕要5s以上才会被点亮>zenghaihui
#if 1
        APS_FUN();
        return 0;
#else
	struct ltr501_priv *obj = i2c_get_clientdata(client);        
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	ltr501_power(obj->hw, 1);
/*	err = ltr501_devinit();
	if(err < 0)
	{
		APS_ERR("initialize client fail!!\n");
		return err;        
	}*/
	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
	    err = ltr501_als_enable(als_gainrange);
	    if (err < 0)
		{
			APS_ERR("enable als fail: %d\n", err);        
		}
	}
	atomic_set(&obj->ps_suspend, 0);
	if(test_bit(CMC_BIT_PS,  &obj->enable))
	{
		err = ltr501_ps_enable(ps_gainrange);
	    if (err < 0)
		{
			APS_ERR("enable ps fail: %d\n", err);                
		}
	}

	return 0;
#endif    
}

static void ltr501_early_suspend(struct early_suspend *h) 
{   /*early_suspend is only applied for ALS*/
	struct ltr501_priv *obj = container_of(h, struct ltr501_priv, early_drv);   
	int err;
	APS_FUN();    

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}
	
	atomic_set(&obj->als_suspend, 1); 
	err = ltr501_als_disable();
	if(err < 0)
	{
		APS_ERR("disable als fail: %d\n", err); 
	}
}

static void ltr501_late_resume(struct early_suspend *h)
{   /*early_suspend is only applied for ALS*/
	struct ltr501_priv *obj = container_of(h, struct ltr501_priv, early_drv);         
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
	    err = ltr501_als_enable(als_gainrange);
		if(err < 0)
		{
			APS_ERR("enable als fail: %d\n", err);        

		}
	}
}

int ltr501_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct ltr501_priv *obj = (struct ltr501_priv *)self;
	
	//APS_FUN(f);
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{				
				value = *(int *)buff_in;
				if(value)
				{
				    err = ltr501_ps_enable(ps_gainrange);
					if(err < 0)
					{
						APS_ERR("enable ps fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_PS, &obj->enable);
				}
				else
				{
				    err = ltr501_ps_disable();
					if(err < 0)
					{
						APS_ERR("disable ps fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_PS, &obj->enable);
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				APS_ERR("get sensor data !\n");
				sensor_data = (hwm_sensor_data *)buff_out;
				obj->ps = ltr501_ps_read();
    			if(obj->ps < 0)
    			{
    				err = -1;
    				break;
    			}
				sensor_data->values[0] = ltr501_get_ps_value(obj, obj->ps);
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;			
			}
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

int ltr501_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct ltr501_priv *obj = (struct ltr501_priv *)self;

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;				
				if(value)
				{
				    err = ltr501_als_enable(als_gainrange);
					if(err < 0)
					{
						APS_ERR("enable als fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
				}
				else
				{
				    err = ltr501_als_disable();
					if(err < 0)
					{
						APS_ERR("disable als fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_ALS, &obj->enable);
				}
				
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;
				obj->als = ltr501_als_read(als_gainrange);
				#if defined(MTK_AAL_SUPPORT)
				sensor_data->values[0] = obj->als;
				#else				
				sensor_data->values[0] = ltr501_get_als_value(obj, obj->als);
				#endif
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;
		default:
			APS_ERR("light sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}


/*----------------------------------------------------------------------------*/
static int ltr501_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) 
{    
	strcpy(info->type, LTR501_DEV_NAME);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int ltr501_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	//struct tmd2771_priv *obj;
	struct ltr501_priv *obj;
	struct hwmsen_object obj_ps, obj_als;
	int err = 0;

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	ltr501_obj = obj;

	obj->hw = get_cust_alsps_hw();
	ltr501_get_addr(obj->hw, &obj->addr);

	INIT_WORK(&obj->eint_work, ltr501_eint_work);
	obj->client = client;
	i2c_set_clientdata(client, obj);	
	atomic_set(&obj->als_debounce, 300);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 300);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->ps_thd_val_high,  obj->hw->ps_threshold_high);
	atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low);
	//atomic_set(&obj->als_cmd_val, 0xDF);
	//atomic_set(&obj->ps_cmd_val,  0xC1);
	atomic_set(&obj->ps_thd_val,  obj->hw->ps_threshold);
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);   
	obj->als_modulus = (400*100)/(16*150);//(1/Gain)*(400/Tine), this value is fix after init ATIME and CONTROL register value
										//(400)/16*2.72 here is amplify *100
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	set_bit(CMC_BIT_ALS, &obj->enable);
	set_bit(CMC_BIT_PS, &obj->enable);

	APS_LOG("ltr501_devinit() start...!\n");
	ltr501_i2c_client = client;

	if(err = ltr501_devinit())
	{
		goto exit_init_failed;
	}
	APS_LOG("ltr501_devinit() ...OK!\n");

	//printk("@@@@@@ manufacturer value:%x\n",ltr501_i2c_read_reg(0x87));

	if(err = misc_register(&ltr501_device))
	{
		APS_ERR("tmd2771_device register failed\n");
		goto exit_misc_device_register_failed;
	}
/*
	if(err = tmd2771_create_attr(&tmd2771_alsps_driver.driver))
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
*/
	obj_ps.self = ltr501_obj;
	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
	if(1 == obj->hw->polling_mode_ps)
	{
		obj_ps.polling = 1;
	}
	else
	{
		obj_ps.polling = 0;
	}
	obj_ps.sensor_operate = ltr501_ps_operate;
	if(err = hwmsen_attach(ID_PROXIMITY, &obj_ps))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}
	
	obj_als.self = ltr501_obj;
	obj_als.polling = 1;
	obj_als.sensor_operate = ltr501_als_operate;
	if(err = hwmsen_attach(ID_LIGHT, &obj_als))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}


#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = ltr501_early_suspend,
	obj->early_drv.resume   = ltr501_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif

	APS_LOG("%s: OK\n", __func__);
	return 0;

	exit_create_attr_failed:
	misc_deregister(&ltr501_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	//i2c_detach_client(client);
	exit_kfree:
	kfree(obj);
	exit:
	ltr501_i2c_client = NULL;           
//	MT6516_EINTIRQMask(CUST_EINT_ALS_NUM);  /*mask interrupt if fail*/
	APS_ERR("%s: err = %d\n", __func__, err);
	return err;
}
/*----------------------------------------------------------------------------*/
static int ltr501_i2c_remove(struct i2c_client *client)
{
	int err;	
    
//LINE<JIRA_ID><DATE20140217><wallpaper check for ms color>zenghaihui
#ifdef TINNO_MS_COLOR_SELECT
    //LINE<JIRA_ID><DATE20140217><wallpaper check for ms color>zenghaihui
	/*------------------------ap3220 attribute file for debug--------------------------------------*/	
	if((err = ltr501_delete_attr(&ltr501_alsps_driver.driver)))
	{
		APS_ERR("ltr501_delete_attr fail: %d\n", err);
	} 
	/*----------------------------------------------------------------------------------------*/
#endif            
        
	

	if(err = misc_deregister(&ltr501_device))
	{
		APS_ERR("misc_deregister fail: %d\n", err);    
	}
	
	ltr501_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}
/*----------------------------------------------------------------------------*/
static int ltr501_probe(struct platform_device *pdev) 
{
	struct alsps_hw *hw = get_cust_alsps_hw();

	ltr501_power(hw, 1);
	//ltr501_force[0] = hw->i2c_num;
	//ltr501_force[1] = hw->i2c_addr[0];
	//APS_DBG("I2C = %d, addr =0x%x\n",ltr501_force[0],ltr501_force[1]);
	if(i2c_add_driver(&ltr501_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	} 
    
	//<Line> <add p-sensor wake_lock> <20131113> <mickal.ma>
	wake_lock_init(&psensor_timer_lock, WAKE_LOCK_SUSPEND, "P-Sensor timer wakelock");

	return 0;
}
/*----------------------------------------------------------------------------*/
static int ltr501_remove(struct platform_device *pdev)
{
	struct alsps_hw *hw = get_cust_alsps_hw();
	APS_FUN();    
	ltr501_power(hw, 0);    
	i2c_del_driver(&ltr501_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
static struct platform_driver ltr501_alsps_driver = {
	.probe      = ltr501_probe,
	.remove     = ltr501_remove,    
	.driver     = {
		.name  = "als_ps",
		//.owner = THIS_MODULE,
	}
};
/*----------------------------------------------------------------------------*/
static int __init ltr501_init(void)
{
	APS_FUN();
	struct alsps_hw *hw = get_cust_alsps_hw();
	i2c_register_board_info(hw->i2c_num, &i2c_ltr501, 1);
	if(platform_driver_register(&ltr501_alsps_driver))
	{
		APS_ERR("failed to register driver");
		return -ENODEV;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit ltr501_exit(void)
{
	APS_FUN();
	platform_driver_unregister(&ltr501_alsps_driver);
}
/*----------------------------------------------------------------------------*/
module_init(ltr501_init);
module_exit(ltr501_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Keqi Li");
MODULE_DESCRIPTION("LTR-501ALS Driver");
MODULE_LICENSE("GPL");

