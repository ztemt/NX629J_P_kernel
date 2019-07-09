/* AMS tcs3707 ALS/PS driver
*
* Author: Marky Zhang<marky.zhang@ams.com>
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

#include "tcs3707.h"
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/uaccess.h>
#ifndef TCS3707_I2C
#include <media/v4l2-event.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ioctl.h>
#include <media/cam_sensor.h>
#include <cam_sensor_i2c.h>
#include <cam_sensor_spi.h>
#include <cam_sensor_io.h>
#include <cam_cci_dev.h>
#include <cam_req_mgr_util.h>
#include <cam_req_mgr_interface.h>
#include <cam_mem_mgr.h>
#include <cam_subdev.h>
#include "cam_soc_util.h"
#endif

#define ARR_SIZE(arr) (sizeof(arr)/sizeof((arr)[0]) + __must_be_array(arr))
#define ASTEP 999

static int tcs3707_init_flag = -1; /* 0<==>OK -1 <==> fail*/
static DEFINE_MUTEX(tcs3707_mutex);
static bool pon_is_opened = false;
static bool first_als = false;
static bool als_opened = false;
static bool als_timer_stoped = true;

#define cct_coef_row 3
#define cct_coef_col 5
#define H_L_IR_Devers 0.5
#define raw_num 5
#define astep 999
float Cct_Coef_Matrix_HIR[cct_coef_row][cct_coef_col] = {
    {-0.044001,	0.135381,	0.424572,	-0.147447,	-0.063961},
    {0.014121,	-0.002556,	0.416529,	-0.198618,	-0.039713},
    {0.153285,	-0.280569,	-0.052648,	0.098794,	0.023832}
};

float Cct_Coef_Matrix_LIR[cct_coef_row][cct_coef_col] = {
    {0.080012,	-0.011142,	0.029001,	-0.097069,	0.012832},
    {0.102216,	-0.084337,	0.062239,	-0.129410,	0.021062},
    {0.117444,	-0.156963,	-0.114635,	0.121181,	0.015381}
};

#define ALSPS							0X84
#define TCS3707_ENABLE_ALS              _IOR(ALSPS, 0x17, int)
#define TCS3707_CALCULATE_CCT           _IOR(ALSPS, 0x18, int)
#define TCS3707_READ_ALS                _IOR(ALSPS, 0x19, int)

typedef struct tcs3707_reg_setting {
    uint8_t reg;
    uint8_t value;
} tcs3707_reg_setting;

tcs3707_reg_setting default_setting[] = {
    {TCS3707_ATIME_REG,    0},
    {TCS3707_CFG0_REG,	   0x00},
    {TCS3707_CFG1_REG,	   AGAIN_4X},
    {TCS3707_CFG3_REG,	   0x0C},
    {TCS3707_CFG4_REG,	   0x80},//use fd_gain as the adata5's gain value,not again
    {TCS3707_CFG8_REG,	   0x98},//disable flicker AGC
    {TCS3707_CFG10_REG,    0xf2},
    {TCS3707_CFG11_REG,    0x40},
    {TCS3707_CFG12_REG,    0x00},
    {TCS3707_CFG14_REG,    0x00},
    {TCS3707_PERS_REG,	   (ALS_PERSIST(0)|PROX_PERSIST(0))},
    {TCS3707_GPIO_REG,	   0x02},			
    {TCS3707_ASTEPL_REG,   0xE7},//default astep is 2.78ms
    {TCS3707_ASTEPH_REG,   0x03},
    {TCS3707_AGC_GAIN_MAX_REG, 0x99},
    {TCS3707_AZ_CONFIG_REG, 0x00},//close auto als zero
    {TCS3707_FD_CFG0,	0x80},//enable fd_fifo_mode
    {TCS3707_FD_CFG1,	0x67},//default fd_time = 1ms
    {TCS3707_FD_CFG3,	0x21},////default fd_time = 1ms, default fd_gain = 8x
    {TCS3707_FIFO_MAP,	0x00}
};


static uint16_t const als_gains[] = {
    0,
    1,
    2,
    4,
    8,
    16,
    32,
    64,
    128,
    256,
    512
};
#ifndef TCS3707_I2C
#define TCS3707_DEVICE_TYPE         0x000100fe

struct tcs3707_data {
    void *client_object;	/*!< cci or i2c model i/f specific ptr  */
    struct timer_list als_timer;
    struct work_struct als_work;	
    uint16_t c_raw;
    uint16_t r_raw;
    uint16_t g_raw;
    uint16_t b_raw;
    uint16_t w_raw;
    uint16_t als_gain;
    uint16_t als_time;
    uint16_t cct;
};

struct tcs3707_ctrl_t {
    struct platform_device *pdev;
    enum msm_camera_device_type_t device_type;	
    enum cci_device_num cci_num;
    enum cci_i2c_master_t cci_master;
    struct camera_io_master io_master_info;	
    struct cam_subdev v4l2_dev_str;
    struct tcs3707_data *tcs3707_data;
    char device_name[20];
    /*!< if null no regulator use for power ctrl */
    struct regulator *power_supply;
    //struct regulator *cci_supply;
    int boot_reg;
};

struct cam_subdev  *tcs_v4l2_dev_str = NULL;

static struct platform_device *mdev = NULL;
#else
struct tcs3707_i2c_addr {	/*define a series of i2c slave address */
    uint8_t write_addr;
    uint8_t ps_thd;		/*PS INT threshold */
};
struct tcs3707_priv {
    struct i2c_client *client;
    struct work_struct irq_work;
    
    struct timer_list als_timer;
    struct work_struct als_work;	
    struct regulator *vdd;
    /*i2c address group */
    struct tcs3707_i2c_addr addr;
    /*misc */
    uint16_t als_modulus;
    /*data */
    uint16_t als;
    uint16_t ps;
    uint8_t _align;
    int ps_cali;
    ulong enable;		/*enable mask */
    ulong pending_intr;	/*pending interrupt */
    uint16_t c_raw;
    uint16_t r_raw;
    uint16_t g_raw;
    uint16_t b_raw;
    uint16_t w_raw;
    uint16_t als_gain;
    uint16_t als_time;
    uint16_t cct;
};
static struct tcs3707_priv *tcs3707_obj;
#endif
#ifndef TCS3707_I2C
static int tcs3707_cci_i2c_read(struct tcs3707_ctrl_t *tcs3707_ctrl, uint8_t reg, uint8_t *val)
{
    int rc = 0;
    rc = cam_camera_cci_i2c_read_seq(tcs3707_ctrl->io_master_info.cci_client, reg, val, CAMERA_SENSOR_I2C_TYPE_BYTE,
    CAMERA_SENSOR_I2C_TYPE_BYTE,1);
    return rc != 0;

}
static int32_t tcs3707_cci_i2c_read_block(struct tcs3707_ctrl_t *tcs3707_ctrl,
uint8_t reg, uint8_t *val, int32_t size)
{

    int rc = 0;
    rc = cam_camera_cci_i2c_read_seq(tcs3707_ctrl->io_master_info.cci_client, reg, val, CAMERA_SENSOR_I2C_TYPE_BYTE,
    CAMERA_SENSOR_I2C_TYPE_BYTE, size);
    return rc != 0;

}
static int tcs3707_cci_i2c_write(struct tcs3707_ctrl_t *tcs3707_ctrl, uint8_t reg, uint8_t val)
{
    
    int i = 0, rc = 0;
    struct cam_sensor_i2c_reg_setting  write_reg_setting;
    struct cam_sensor_i2c_reg_array    *reg_array  = NULL;
    
    
    reg_array = kzalloc(sizeof(struct cam_sensor_i2c_reg_array), GFP_KERNEL);
    if (!reg_array) {
        TCS_DBG("cci_write malloc failed %d\n");
    }
    reg_array[i].reg_addr = reg;
    reg_array[i].reg_data = val;
    
    write_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
    write_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
    write_reg_setting.size		= 1;
    write_reg_setting.reg_setting = reg_array;
    write_reg_setting.delay 	= 0;
    
    rc = cam_cci_i2c_write_table(&tcs3707_ctrl->io_master_info, &write_reg_setting);
    
    kfree(reg_array);
return rc;

}


static int32_t tcs3707_cci_i2c_modify(struct tcs3707_ctrl_t *tcs3707_ctrl, uint8_t reg, uint8_t mask, uint8_t val)
{
    int32_t ret;
    uint8_t temp;
    ret = tcs3707_cci_i2c_read(tcs3707_ctrl, reg, &temp);
    temp &= ~mask;
    temp |= val&mask;
    ret |= tcs3707_cci_i2c_write(tcs3707_ctrl, reg, temp);	
    return ret;
}

static void TCS3707_MUTEX_LOCK(void)
{
    mutex_lock(&tcs3707_mutex);
    //TCS_DBG("tcs3707_mutex lock!!!\n");
}

static void TCS3707_MUTEX_UNLOCK(void)
{
    //TCS_DBG("tcs3707_mutex unlock\n");
    mutex_unlock(&tcs3707_mutex);	
}

static int32_t tcs3707_set_als_time_ms(struct tcs3707_ctrl_t *t_ctrl, uint32_t time_ms)
{
    int32_t ret = 0;
    uint16_t atime;
    
    tcs3707_cci_i2c_modify(t_ctrl,TCS3707_ENABLE_REG,AEN,0);//close als	
    atime = ATIME_MS(time_ms);
    ret = tcs3707_cci_i2c_write(t_ctrl, TCS3707_ATIME_REG, atime);
    tcs3707_cci_i2c_modify(t_ctrl,TCS3707_ENABLE_REG,AEN,AEN);//open als
    t_ctrl->tcs3707_data->als_time = time_ms;
    //TCS_DBG("tcs3707_obj->als_time =  %d, atime_reg_value = 0x%02x\n", tcs3707_obj->als_time,atime);
    return ret;
}

static int32_t tcs3707_set_als_gain(struct tcs3707_ctrl_t *tcs3707_ctrl, uint16_t again)
{
    int32_t ret = 0;
    uint8_t cfg1 = 0;
    
    switch (again)
    {
        case 0:
            cfg1 = AGAIN_0_5X;
            break;
        case 1:
            cfg1 = AGAIN_1X;
            break;
        case 2:
            cfg1 = AGAIN_2X;
            break;
        case 4:
            cfg1 = AGAIN_4X;
            break;
        case 8:
            cfg1 = AGAIN_8X;
            break;
        case 16:
            cfg1 = AGAIN_16X;
            break;
        case 32:
            cfg1 = AGAIN_32X;
            break;
        case 64:
            cfg1 = AGAIN_64X;
            break;
        case 128:
            cfg1 = AGAIN_128X;
            break;
        case 256:
            cfg1 = AGAIN_256X;
            break;
        case 512:
            cfg1 = AGAIN_512X;
            break;
        
        default:
            break;
    }
    
    tcs3707_cci_i2c_modify(tcs3707_ctrl,TCS3707_ENABLE_REG,AEN,0);//close als	
    
    ret = tcs3707_cci_i2c_modify(tcs3707_ctrl, TCS3707_CFG1_REG, AGAIN_MASK, cfg1);	
    
    tcs3707_cci_i2c_modify(tcs3707_ctrl,TCS3707_ENABLE_REG,AEN,AEN);//open als
    //TCS_DBG("tcs3707_set_als_gain again =  %d, cfg1 = 0x%02x\n", again,cfg1);
    tcs3707_ctrl->tcs3707_data->als_gain = again;
    
    return ret;
}
static int tcs3707_init_client(struct tcs3707_ctrl_t *t_ctrl)
{
    
    uint8_t id = 0;
    uint8_t i = 0;
    int res = 0;
    res = tcs3707_cci_i2c_read(t_ctrl, 0x92, &id);
    
    //res = tcs3707_cci_i2c_read(t_ctrl, 0x92, &id);
    TCS_DBG("tcs3707_init_client id =  %d, res = %d\n", id,res);
    if (res < 0)
        return -EIO;
    if(id != 24)
        return -EINVAL;
    
    
    for (i = 0; i < ARR_SIZE(default_setting); i++){
        res = tcs3707_cci_i2c_write(t_ctrl,
        default_setting[i].reg,
        default_setting[i].value);
        if (res < 0){
            return -EIO;
        }
    }		
    
    pon_is_opened = false;
    als_opened = false;
    return 0;
}
static int tcs3707_open(struct inode *inode, struct file *file)
{
    
    struct tcs3707_data *tcs3707_data = platform_get_drvdata(mdev);
    struct tcs3707_ctrl_t *tcs3707_ctrl = (struct tcs3707_ctrl_t *)tcs3707_data->client_object;
    file->private_data = tcs3707_ctrl;
    
    if (!file->private_data) {
        //TCS_ERR("null pointer!!\n");
        return -EINVAL;
    }
    
    return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int tcs3707_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    return 0;
}


static int calculate_cct_lux(struct tcs3707_data *obj)
{
    int i = 0, j = 0, k = 0;
    float X = 0,Y = 0, Z = 0, x= 0, y = 0;
    int cct = 0;
    float sum = 0,temp = 0;
    uint16_t raw_array[cct_coef_col] = {obj->c_raw, obj->r_raw, obj->g_raw, obj->b_raw, obj->w_raw};
    uint16_t raw_norm_array[cct_coef_col] = {0};
    float matrix[cct_coef_row] = {0};
    float ir_pctg = 0;
    
    //calculate itime
    int itime = (obj->als_time + 1)*(astep +1) / 360;
    
    //normalize the raw data to 100ms itime and 128x again
    for(k = 0; k < cct_coef_col; k++){
        raw_norm_array[k] = (raw_array[k] * 100 * 128)/(itime * obj->als_gain);
    }
    //TCS_ERR("itime = %d, again = %d\n\n",itime,obj->als_gain);
    
    //calculate IR
    ir_pctg = 1 - (raw_norm_array[0]/(2.7 * raw_norm_array[4]));
    //TCS_ERR("ir_pctg = %f\n",ir_pctg);
    
    //calculate X/Y/Z	
    for(j = 0; j < cct_coef_row; j++){
       for(i = 0; i < cct_coef_col; i++){
           if(ir_pctg >= H_L_IR_Devers){
               matrix[j] += Cct_Coef_Matrix_HIR[j][i] * (float)raw_norm_array[i];	
           }else{
               matrix[j] += Cct_Coef_Matrix_LIR[j][i] * (float)raw_norm_array[i];	
           }				
       }
    }	
    X = matrix[0];
    Y = matrix[1];
    Z = matrix[2];
    
    //calculate x/y and cct
    sum = X + Y + Z;
    x = X/sum;
    y = Y/sum;
    temp = (x - 0.332)/(y - 0.1858);
    
    
    cct =(uint16_t) (-499*temp*temp*temp + 3525*temp*temp - 6823.3*temp + 5520.33);
    //TCS_ERR("x = %f, y = %f, temp = %f,cct = %d\n",x,y,temp,cct);
    //TCS_ERR("cct = %d\n",cct);
    
    return cct;

}
static void tcs3707_report_als_data(struct tcs3707_data *obj)
{	
    uint8_t data[10] = {0};
    uint8_t atime_val = 0,again_val = 0;
    uint16_t again = 0;
    uint16_t saturation = 0, again_adjust_high_thr = 0, again_adjust_low_thr = 0;
    bool again_adjusted = false;
    
    tcs3707_cci_i2c_read_block(obj->client_object, TCS3707_ADATA0L_REG, &data[0], 10);	
    obj->c_raw = (uint16_t)(((uint16_t)data[1]<<8)|data[0]);
    obj->r_raw   = (uint16_t)(((uint16_t)data[3]<<8)|data[2]);
    obj->g_raw = (uint16_t)(((uint16_t)data[5]<<8)|data[4]);
    obj->b_raw  = (uint16_t)(((uint16_t)data[7]<<8)|data[6]);
    obj->w_raw = (uint16_t)(((uint16_t)data[9]<<8)|data[8]);
    //TCS_DBG("clear_raw = %d,red_raw = %d,green_raw = %d,blue_raw = %d,wideband_raw = %d\n"
    //	,obj->c_raw, obj->r_raw, obj->g_raw, obj->b_raw, obj->w_raw);
    tcs3707_cci_i2c_read(obj->client_object,TCS3707_ATIME_REG,&atime_val);
    tcs3707_cci_i2c_read(obj->client_object,TCS3707_CFG1_REG,&again_val);
    again = als_gains[again_val&AGAIN_MASK];
    saturation = (atime_val + 1)*(ASTEP +1);
    again_adjust_high_thr = (saturation*8)/10;
    again_adjust_low_thr = saturation/10;	
    //TCS_DBG("again = %d,saturation = %d,again_adjust_high_thr = %d,again_adjust_low_thr = %d\n"
    //	,again, saturation, again_adjust_high_thr, again_adjust_low_thr);
    
    
    if(first_als){
        first_als = false;
        //init atime is 2.78ms, init again is 4x, saturation value is 999+1=1000;1000*80%=800
        //if change to 16x, not saturated, 800/4 = 200; if change to 64x, not saturated, 200/4 = 50;
        //if change to 256x, not saturated, 50/4 = 13;
        //use 1x 4x 16x 64x 256x gain values
        if(obj->c_raw <= 13){
            tcs3707_set_als_gain(obj->client_object,256);
            //TCS_DBG("tcs3707_report_als_data, quickly fix gain to 256x\n");
        }else if(obj->c_raw <= 50){
            tcs3707_set_als_gain(obj->client_object,64);
            //TCS_DBG("tcs3707_report_als_data, quickly fix gain to 64x\n");
        }else if(obj->c_raw <= 200){
            tcs3707_set_als_gain(obj->client_object,16);
            //TCS_DBG("tcs3707_report_als_data, quickly fix gain to 16x\n");
        }else if(obj->c_raw <= 800){			
            //TCS_DBG("tcs3707_report_als_data, keep 4x gain\n");
        }else{
            tcs3707_set_als_gain(obj->client_object,1);
            //TCS_DBG("tcs3707_report_als_data, quickly fix gain to 1x\n");
        }
        //change atime back to 100ms
        tcs3707_set_als_time_ms(obj->client_object, 100);
        again_adjusted = true;
        
    }else{
        //auto gain control, use 1x 4x 16x 64x 256x gain values
        if((again == 64)&&(obj->c_raw <= again_adjust_low_thr)){
            tcs3707_set_als_gain(obj->client_object,256);
            //TCS_DBG("tcs3707_report_als_data, set gain to 256x\n");
            again_adjusted = true;
        }else if(((again == 16)&&(obj->c_raw <= again_adjust_low_thr))
        ||((again == 256)&&(obj->c_raw >= again_adjust_high_thr))){
            tcs3707_set_als_gain(obj->client_object,64);
            //TCS_DBG("tcs3707_report_als_data, set gain to 64x\n");
            again_adjusted = true;
        }else if(((again == 4)&&(obj->c_raw <= again_adjust_low_thr))
        ||((again == 64)&&(obj->c_raw >= again_adjust_high_thr))){
            tcs3707_set_als_gain(obj->client_object,16);
            //TCS_DBG("tcs3707_report_als_data, set gain to 16x\n");
            again_adjusted = true;
        }else if(((again == 1)&&(obj->c_raw <= again_adjust_low_thr))
        ||((again == 16)&&(obj->c_raw >= again_adjust_high_thr))){
            tcs3707_set_als_gain(obj->client_object,4);
            //TCS_DBG("tcs3707_report_als_data, set gain to 4x\n");
            again_adjusted = true;
        }else if((again == 4)&&(obj->c_raw >= again_adjust_high_thr)){
            tcs3707_set_als_gain(obj->client_object,1);
            //TCS_DBG("tcs3707_report_als_data, set gain to 1x\n");
            again_adjusted = true;
        }
    }
    
    
    if(als_timer_stoped == false){
        mod_timer(&obj->als_timer, jiffies + 110/(1000/HZ));//110ms timer
        //TCS_DBG("tcs3707_report_als_data als_timer 110ms start!!!");
    }	
    if(again_adjusted == true){
        goto exit;
    }
    obj->cct = calculate_cct_lux(obj);
    //calculate lux and cct, need implement here
    
    exit:
        return;

}

static void tcs3707_als_work(struct work_struct *work)
{
    struct tcs3707_data *tcs3707_data =
    container_of(work, struct tcs3707_data, als_work);
    TCS3707_MUTEX_LOCK();
    tcs3707_report_als_data(tcs3707_data);
    TCS3707_MUTEX_UNLOCK();
};
void tcs3707_als_poll_handle(unsigned long data)
{		
    struct tcs3707_data *tcs3707_data = (struct tcs3707_data *)data;		
    schedule_work(&tcs3707_data->als_work);
}

static int tcs3707_als_enable(struct tcs3707_ctrl_t *t_ctrl, unsigned int enable)
{
    int ret = 0;
    TCS3707_MUTEX_LOCK();
    //TCS_DBG("tcs3707_als_enable, enable = %d\n",enable);
    if(enable == 1){
        if(als_opened == false){
            if(pon_is_opened == false){
                pon_is_opened = true;
                tcs3707_cci_i2c_write(t_ctrl, TCS3707_ENABLE_REG, PON);
                mdelay(1);//delay 1ms	
                tcs3707_cci_i2c_modify(t_ctrl, TCS3707_CONTROL_REG, ALS_MANUAL_AZ,ALS_MANUAL_AZ);//trigger a auto zero
            }		
        tcs3707_set_als_time_ms(t_ctrl, 3);//use a short atime to quickly fix a proper gain
        tcs3707_set_als_gain(t_ctrl,4);
        
        //use a short timer to quickly fix a again							
        mod_timer(&t_ctrl->tcs3707_data->als_timer, jiffies + 5/(1000/HZ));
        //TCS_DBG("tcs3707_als_enable, start als timer 5ms\n");
        first_als = true;
        als_opened = true;
        als_timer_stoped = false;
        }
    }
    else if(enable == 0)
    {
        if(als_opened == true){
            als_opened = false;
            als_timer_stoped = true;
            tcs3707_cci_i2c_modify(t_ctrl,TCS3707_ENABLE_REG,AEN,0);//close als 
        }		
    }
    else
    {
        //TCS_ERR("tcs3707_als_enable, invalid value of enable\n");
    }
    TCS3707_MUTEX_UNLOCK();
    
    return ret;
}
static long tcs3707_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{	
    //struct i2c_client *client = (struct i2c_client *)file->private_data;
    //	struct tcs3707_priv *obj = i2c_get_clientdata(client);
    struct tcs3707_ctrl_t * tcs3707_ctrl =  (struct tcs3707_ctrl_t *)file->private_data;
    long err = 0;
    void __user *ptr = (void __user *)arg;
    
    uint32_t enable = 100;
    int als_recieved_value = -1;
    uint16_t als_data[7] = {0};
    uint16_t cct = 0;
    switch (cmd) {
        case TCS3707_ENABLE_ALS:		
            //TCS_DBG("tcs3707_unlocked_ioctl, case TCS3707_ENABLE_ALS\n");
            
            if (copy_from_user(&als_recieved_value, ptr, sizeof(als_recieved_value))) {
                err = -EFAULT;
                goto err_out;
            }
            //TCS_DBG("tcs3707_unlocked_ioctl, als_recieved_value = %d\n",als_recieved_value);
            if(als_recieved_value == 1){//enable als
                tcs3707_als_enable(tcs3707_ctrl, 1);
            }
            else if(als_recieved_value == 0)
            {//disable als
                tcs3707_als_enable(tcs3707_ctrl, 0);
            }
            else
            {
                //TCS_ERR("tcs3707_unlocked_ioctl, als_recieved_value is invalid\n");
            }
            
            if (copy_to_user(ptr, &enable, sizeof(enable))) {
                err = -EFAULT;
                goto err_out;
            }
            break;
        case TCS3707_CALCULATE_CCT:
            TCS3707_MUTEX_LOCK();
            cct = tcs3707_ctrl->tcs3707_data->cct;
            
            if (copy_to_user(ptr, &cct, sizeof(uint16_t))) {
                TCS3707_MUTEX_UNLOCK();
                err = -EFAULT;
                goto err_out;
            }
            TCS3707_MUTEX_UNLOCK();
            break;
        case TCS3707_READ_ALS:	
            //TCS_ERR("tcs3707_unlocked_ioctl, case TCS3707_READ_ALS\n");		
            TCS3707_MUTEX_LOCK();
            als_data[0] = tcs3707_ctrl->tcs3707_data->c_raw;
            als_data[1] = tcs3707_ctrl->tcs3707_data->r_raw;
            als_data[2] = tcs3707_ctrl->tcs3707_data->g_raw;
            als_data[3] = tcs3707_ctrl->tcs3707_data->b_raw;
            als_data[4] = tcs3707_ctrl->tcs3707_data->w_raw;
            als_data[5] = tcs3707_ctrl->tcs3707_data->als_gain;
            als_data[6] = tcs3707_ctrl->tcs3707_data->als_time;
            TCS3707_MUTEX_UNLOCK();
            if (copy_to_user(ptr, &als_data[0], sizeof(als_data))) {
                err = -EFAULT;
                goto err_out;
            }
            
            break;
        
        default:
            //TCS_ERR("%s not supported = 0x%04x\n", __func__, cmd);
            err = -ENOIOCTLCMD;
            break;
    }
    
    err_out:
        return err;
}
static ssize_t tcs3707_read(struct file* file, char __user* buf, size_t count, loff_t* f_pos)
{
    struct tcs3707_ctrl_t * tcs3707_ctrl =  (struct tcs3707_ctrl_t *)file->private_data;
    ssize_t          	status = 0;
    unsigned long       missing;
    uint16_t cct = 0;
    
    
    TCS3707_MUTEX_LOCK();
    cct = tcs3707_ctrl->tcs3707_data->cct;
    missing = copy_to_user(buf, &cct, count);
    if (missing == 0) {
        TCS_DBG("copy to user sucsess");
    }
    else 
    {//if missing > 0, means missing bytes not copied
        TCS_ERR("copy to user error") ;
        status = -EFAULT;
    }
    
    TCS3707_MUTEX_UNLOCK();
    return status;
}

/*----------------------------------------------------------------------------*/

static int tcs3707_get_dt_info(struct device *dev, struct tcs3707_ctrl_t *t_ctrl)
{
    int rc = 0;
    struct device_node   *of_node  = NULL;
    
    if (!dev || !t_ctrl)
        return -EINVAL;
    
    of_node  = dev->of_node;
    if (!of_node) {
        TCS_DBG("of_node is NULL %d\n", __LINE__);
        return -EINVAL;
    }
    rc = of_property_read_u32(of_node, "cell-index", &t_ctrl->pdev->id);
    if (rc < 0) {
        TCS_DBG("failed to read cell index %d\n", __LINE__);
        return rc;
    }
    
    rc = of_property_read_u32(of_node, "cci-master", &t_ctrl->cci_master);
    if (rc < 0) {
        TCS_DBG("failed to get the cci master %d\n", __LINE__);
        return rc;
    }
    rc = of_property_read_u32(of_node, "cci-device", &t_ctrl->cci_num);
    if (rc < 0) {
        /* Set default master 0 */
        t_ctrl->cci_num = CCI_DEVICE_0;
        rc = 0;
    }
    t_ctrl->io_master_info.cci_client->cci_device = t_ctrl->cci_num;
    
    t_ctrl->power_supply = regulator_get(dev, "vdd");
    if (IS_ERR(t_ctrl->power_supply) || t_ctrl->power_supply == NULL) {
        t_ctrl->power_supply = NULL;
    
    }
    return rc;
}
static int tcs3707_power_up(struct tcs3707_ctrl_t *t_ctrl){
    int rc = 0;
    if (t_ctrl->power_supply) {
        //rc = cam_soc_util_regulator_enable(t_ctrl->power_supply, "tcs3707", 2800000, 2800000, 80000, 0);
        rc = regulator_enable(t_ctrl->power_supply);
        if (rc) {
            TCS_ERR("fail to turn on regulator");
            return rc;
        }
    }
    rc = camera_io_init(&t_ctrl->io_master_info);
    if (rc < 0)
        TCS_ERR("cci init failed: rc: %d", rc);
    return rc;
}
static void tcs3707_power_down(struct tcs3707_ctrl_t *t_ctrl){
    if (t_ctrl->power_supply) {
        regulator_put(t_ctrl->power_supply);
        t_ctrl->power_supply= NULL;
    } 	
}
static int msm_tcs3707_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
    int rc = 0;
    return rc;
}

static const struct v4l2_subdev_internal_ops msm_tcs3707_internal_ops = {
    .close = msm_tcs3707_close,
};

static long msm_tcs3707_subdev_ioctl(struct v4l2_subdev *sd,
unsigned int cmd, void *arg)
{
    int32_t rc = 0;
    return rc;
}
static int32_t msm_tcs3707_power(struct v4l2_subdev *sd, int on)
{
    return 0;
}
static struct v4l2_subdev_core_ops msm_tcs3707_subdev_core_ops = {
    .ioctl = msm_tcs3707_subdev_ioctl,
    .s_power = msm_tcs3707_power,
};

static struct v4l2_subdev_ops msm_tcs3707_subdev_ops = {
    .core = &msm_tcs3707_subdev_core_ops,
};



static const struct file_operations tcs3707_fops = {
    .owner = THIS_MODULE,
    .open = tcs3707_open,
    .release = tcs3707_release,
    .read = tcs3707_read,
    .unlocked_ioctl = tcs3707_unlocked_ioctl,
};

/*----------------------------------------------------------------------------*/
static struct miscdevice tcs3707_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "tcs3707",
    .fops = &tcs3707_fops,
};
static int32_t tcs3707_platform_probe(struct platform_device *pdev)
{
    int32_t rc = 0;
    struct tcs3707_data *tcs3707_data	= NULL;
    struct tcs3707_ctrl_t *tcs3707_ctrl   = NULL;
    struct cam_sensor_cci_client *cci_client = NULL;
    TCS_FUN();
    mdev = pdev;
    if (!cam_cci_get_subdev(0)){
        return -EPROBE_DEFER;
    }
    
    tcs3707_data = kzalloc(sizeof(struct tcs3707_data), GFP_KERNEL);
    if (!tcs3707_data) {
        rc = -ENOMEM;
        return rc;
    }
    
    tcs3707_data->client_object = kzalloc(sizeof(struct tcs3707_ctrl_t), GFP_KERNEL);
    if (!tcs3707_data->client_object) {
        rc = -ENOMEM;
        goto free_tcs3707_data; 
    }
    tcs3707_ctrl = (struct tcs3707_ctrl_t *)tcs3707_data->client_object;
    tcs3707_ctrl->pdev = pdev;
    tcs3707_ctrl->tcs3707_data = tcs3707_data;
    tcs3707_ctrl->device_type = MSM_CAMERA_PLATFORM_DEVICE;
    tcs3707_ctrl->io_master_info.master_type = CCI_MASTER;
    tcs3707_ctrl->io_master_info.cci_client = kzalloc(
    sizeof(struct cam_sensor_cci_client), GFP_KERNEL);
    if (!tcs3707_ctrl->io_master_info.cci_client)
        goto free_tcs3707_ctrl;
    rc = tcs3707_get_dt_info(&pdev->dev, tcs3707_ctrl);
    if (rc < 0) {
        TCS_ERR("%d, failed to get dt info rc %d\n", __LINE__, rc);
        goto free_cci_client;
    }
    cci_client = tcs3707_ctrl->io_master_info.cci_client;
    cci_client->cci_i2c_master = tcs3707_ctrl->cci_master;
    cci_client->sid = 0x39;
    cci_client->retries = 3;
    cci_client->id_map = 0;
    cci_client->i2c_freq_mode = I2C_FAST_PLUS_MODE;
    
    tcs3707_ctrl->v4l2_dev_str.internal_ops = &msm_tcs3707_internal_ops;
    tcs3707_ctrl->v4l2_dev_str.ops = &msm_tcs3707_subdev_ops;
    strlcpy(tcs3707_ctrl->device_name, tcs3707_DEV_NAME,
    sizeof(tcs3707_ctrl->device_name));
    tcs3707_ctrl->v4l2_dev_str.name = tcs3707_ctrl->device_name;
    tcs3707_ctrl->v4l2_dev_str.sd_flags =
    (V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS);
    tcs3707_ctrl->v4l2_dev_str.ent_function = TCS3707_DEVICE_TYPE;
    tcs3707_ctrl->v4l2_dev_str.token = tcs3707_ctrl;
    
    rc = cam_register_subdev(&(tcs3707_ctrl->v4l2_dev_str));
    if (rc) {
        TCS_ERR("fail to create subdev");
        goto unregister_subdev;
    }
    
    tcs_v4l2_dev_str = &tcs3707_ctrl->v4l2_dev_str;
    dev_set_drvdata(&pdev->dev, tcs3707_data);
    rc = tcs3707_power_up(tcs3707_ctrl);
    if(rc){
        TCS_ERR("power_up failed.\n");
        goto power_up_fail;
    }
    rc  = tcs3707_init_client(tcs3707_ctrl);
    if (rc  < 0)
        goto exit_init_failed;
    rc  = misc_register(&tcs3707_device);
    if (rc ) {
        TCS_ERR("tcs3707_device register failed\n");
        goto exit_misc_device_register_failed;
    }
    
    INIT_WORK(&tcs3707_data->als_work, tcs3707_als_work);
    init_timer(&tcs3707_data->als_timer);
    tcs3707_data->als_timer.expires	= jiffies + 100/(1000/HZ);
    tcs3707_data->als_timer.function	= tcs3707_als_poll_handle;
    tcs3707_data->als_timer.data	= (unsigned long)tcs3707_data;
    tcs3707_init_flag = 0;
    TCS_ERR("%s: OK\n", __func__);
    return 0;
    
    exit_misc_device_register_failed:
        misc_deregister(&tcs3707_device);
    exit_init_failed:
        //tcs3707_power_down(tcs3707_ctrl);	
    power_up_fail:
    unregister_subdev:
        cam_unregister_subdev(&(tcs3707_ctrl->v4l2_dev_str));
    free_cci_client:
        kfree(tcs3707_ctrl->io_master_info.cci_client);	
    free_tcs3707_ctrl:
        kfree(tcs3707_ctrl);
    free_tcs3707_data:
        kfree(tcs3707_data);
        tcs3707_init_flag = -1;
        return -1;
}
static int32_t tcs3707_platform_remove(struct platform_device *pdev)
{
    struct tcs3707_data *tcs3707_data = platform_get_drvdata(pdev);
    struct tcs3707_ctrl_t *tcs3707_ctrl = (struct tcs3707_ctrl_t *)tcs3707_data->client_object;
    
    misc_deregister(&tcs3707_device);
    cam_unregister_subdev(tcs_v4l2_dev_str);
    tcs3707_power_down(tcs3707_ctrl);
        
    camera_io_release(&tcs3707_ctrl->io_master_info);
    if (tcs3707_ctrl && tcs3707_ctrl->io_master_info.cci_client)
        kfree(tcs3707_ctrl->io_master_info.cci_client);
    
    platform_set_drvdata(pdev, NULL);
    kfree(tcs3707_data->client_object);
    kfree(tcs3707_data);
    
    return 0;
}
static const struct of_device_id tcs3707_dt_match[] = {
    {.compatible = "nubia,tcs3707",},
    {},
};

static struct platform_driver tcs3707_platform_driver = {
    .probe = tcs3707_platform_probe,
    .remove = tcs3707_platform_remove,
    .driver = {
        .name = tcs3707_DEV_NAME,
        .owner = THIS_MODULE,
        .of_match_table = tcs3707_dt_match,
    },
};
static int __init tcs3707_init(void)
{
    TCS_FUN();
    
    if (platform_driver_register(&tcs3707_platform_driver)){
        TCS_ERR("%d, error\n", __LINE__);
        return -1;
    }
    if (-1 == tcs3707_init_flag)
        return -1;
    
    return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit tcs3707_exit(void )
{
    TCS_FUN();
    
    platform_driver_unregister(&tcs3707_platform_driver);

}
#else

static int tcs3707_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tcs3707_i2c_remove(struct i2c_client *client);

static struct i2c_client *tcs3707_i2c_client;
static const struct i2c_device_id tcs3707_i2c_id[] = { {tcs3707_DEV_NAME, 0}, {} };


static struct i2c_driver tcs3707_i2c_driver = {
    .probe = tcs3707_i2c_probe,
    .remove = tcs3707_i2c_remove,
    .id_table = tcs3707_i2c_id,
    .driver = {
        .name = tcs3707_DEV_NAME,
    },
};
static int tcs3707_i2c_read(struct i2c_client *client, uint8_t reg, uint8_t *val)
{
    int ret;
    s32 read;		
    
    ret = i2c_smbus_write_byte(client, reg);
    if (ret < 0) {
        mdelay(3);
        ret = i2c_smbus_write_byte(client, reg);
        if (ret < 0) {
            dev_err(&client->dev, "%s: failed 2x to write register %x\n",
            __func__, reg);			
            return ret;
        }
    }

    read = i2c_smbus_read_byte(client);
    if (read < 0) {
        mdelay(3);
        read = i2c_smbus_read_byte(client);
        if (read < 0) {
            dev_err(&client->dev, "%s: failed 2x to read from register %x\n",
            __func__, reg);			
            return read;
        }
    }
    
    *val = (uint8_t)read;	
    return 0;
}


static int32_t tcs3707_i2c_read_block(struct i2c_client *client,
uint8_t reg, uint8_t *val, int32_t size)
{
    int32_t ret;
    ret =  i2c_smbus_read_i2c_block_data(client, reg, size, val);
    if (ret < 0) {
        printk(KERN_ERR "%s: failed to read block from address %x (%d bytes)\n",
        __func__, reg, size);
    }
    return ret;
}

static int tcs3707_i2c_write(struct i2c_client *client, uint8_t reg, uint8_t val)
{
    int ret;
    ret = i2c_smbus_write_byte_data(client, reg, val);
    if (ret < 0) {
        mdelay(3);
        ret = i2c_smbus_write_byte_data(client, reg, val);
        if (ret < 0) {
            dev_err(&client->dev, "%s: failed 2x to write register %x err= %d\n",
            __func__, reg, ret);
        }
    }	
    return ret;
}

static int32_t tcs3707_i2c_modify(struct i2c_client *client, uint8_t reg, uint8_t mask, uint8_t val)
{
    int32_t ret;
    uint8_t temp;
    ret = tcs3707_i2c_read(client, reg, &temp);
    temp &= ~mask;
    temp |= val&mask;
    ret |= tcs3707_i2c_write(client, reg, temp);	
    return ret;
}

static void TCS3707_MUTEX_LOCK(void)
{
    mutex_lock(&tcs3707_mutex);
    //TCS_DBG("tcs3707_mutex lock!!!\n");
}

static void TCS3707_MUTEX_UNLOCK(void)
{
    //TCS_DBG("tcs3707_mutex unlock\n");
    mutex_unlock(&tcs3707_mutex);	
}

static int32_t tcs3707_set_als_time_ms(struct i2c_client *client, uint32_t time_ms)
{
    int32_t ret = 0;
    uint16_t atime;
    
    tcs3707_i2c_modify(client,TCS3707_ENABLE_REG,AEN,0);//close als	
    atime = ATIME_MS(time_ms);
    ret = tcs3707_i2c_write(client, TCS3707_ATIME_REG, atime);
    tcs3707_i2c_modify(client,TCS3707_ENABLE_REG,AEN,AEN);//open als
    tcs3707_obj->als_time = time_ms;
    //TCS_DBG("tcs3707_obj->als_time =  %d, atime_reg_value = 0x%02x\n", tcs3707_obj->als_time,atime);
    return ret;
}

static int32_t tcs3707_set_als_gain(struct i2c_client *client, uint16_t again)
{
    int32_t ret = 0;
    uint8_t cfg1 = 0;
    
    switch (again)
    {
        case 0:
            cfg1 = AGAIN_0_5X;
            break;
        case 1:
            cfg1 = AGAIN_1X;
            break;
        case 2:
            cfg1 = AGAIN_2X;
            break;
        case 4:
            cfg1 = AGAIN_4X;
            break;
        case 8:
            cfg1 = AGAIN_8X;
            break;
        case 16:
            cfg1 = AGAIN_16X;
            break;
        case 32:
            cfg1 = AGAIN_32X;
            break;
        case 64:
            cfg1 = AGAIN_64X;
            break;
        case 128:
            cfg1 = AGAIN_128X;
            break;
        case 256:
            cfg1 = AGAIN_256X;
            break;
        case 512:
            cfg1 = AGAIN_512X;
            break;
        
        default:
            break;
    }
    
    tcs3707_i2c_modify(client,TCS3707_ENABLE_REG,AEN,0);//close als	
    
    ret = tcs3707_i2c_modify(client, TCS3707_CFG1_REG, AGAIN_MASK, cfg1);	
    
    tcs3707_i2c_modify(client,TCS3707_ENABLE_REG,AEN,AEN);//open als
    //TCS_DBG("tcs3707_set_als_gain again =  %d, cfg1 = 0x%02x\n", again,cfg1);
    tcs3707_obj->als_gain = again;
    
    return ret;
}
static int tcs3707_init_client(struct i2c_client *client)
{

    uint8_t id = 0;
    uint8_t i = 0;
    int res = 0;	
    
    res = tcs3707_i2c_read(client, 0x92, &id);
    TCS_DBG("tcs3707_init_client id =  %d, res = %d\n", id,res);
    if (res < 0)
        return -EIO;
    if(id != 24)
        return -EINVAL;
    
    
    for (i = 0; i < ARR_SIZE(default_setting); i++){
        res = tcs3707_i2c_write(client,
        default_setting[i].reg,
        default_setting[i].value);
        if (res < 0){
            return -EIO;
        }
        //TCS_DBG("ARR_SIZE(default_setting) i = %d\n", i);
    }		
    
    pon_is_opened = false;
    als_opened = false;
    return 0;
}
static int tcs3707_open(struct inode *inode, struct file *file)
{
    file->private_data = tcs3707_i2c_client;
    
    if (!file->private_data) {
        //TCS_ERR("null pointer!!\n");
        return -EINVAL;
    }
    
    return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int tcs3707_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    return 0;
}


static int calculate_cct_lux(struct tcs3707_priv *obj)
{
    int i = 0, j = 0, k = 0;
    float X = 0,Y = 0, Z = 0, x= 0, y = 0;
    int cct = 0;
    float sum = 0,temp = 0;
    uint16_t raw_array[cct_coef_col] = {obj->c_raw, obj->r_raw, obj->g_raw, obj->b_raw, obj->w_raw};
    uint16_t raw_norm_array[cct_coef_col] = {0};
    float matrix[cct_coef_row] = {0};
    float ir_pctg = 0;
    
    //calculate itime
    int itime = (obj->als_time + 1)*(astep +1) / 360;
    
    //normalize the raw data to 100ms itime and 128x again
    for(k = 0; k < cct_coef_col; k++){
        raw_norm_array[k] = (raw_array[k] * 100 * 128)/(itime * obj->als_gain);
    }
    //TCS_ERR("itime = %d, again = %d\n\n",itime,obj->als_gain);
    
    //calculate IR
    ir_pctg = 1 - (raw_norm_array[0]/(2.7 * raw_norm_array[4]));
    //TCS_ERR("ir_pctg = %f\n",ir_pctg);
    
    //calculate X/Y/Z	
    for(j = 0; j < cct_coef_row; j++){
        for(i = 0; i < cct_coef_col; i++){
            if(ir_pctg >= H_L_IR_Devers){
                matrix[j] += Cct_Coef_Matrix_HIR[j][i] * (float)raw_norm_array[i];	
            }else{
                matrix[j] += Cct_Coef_Matrix_LIR[j][i] * (float)raw_norm_array[i];	
            }				
        }
    }	
    X = matrix[0];
    Y = matrix[1];
    Z = matrix[2];
    
    //calculate x/y and cct
    sum = X + Y + Z;
    x = X/sum;
    y = Y/sum;
    temp = (x - 0.332)/(y - 0.1858);
    
    
    cct =(uint16_t) (-499*temp*temp*temp + 3525*temp*temp - 6823.3*temp + 5520.33);
    //TCS_ERR("x = %f, y = %f, temp = %f,cct = %d\n",x,y,temp,cct);
    //TCS_ERR("cct = %d\n",cct);
    
    return cct;

}
static void tcs3707_report_als_data(struct tcs3707_priv *obj)
{	
    uint8_t data[10] = {0};
    uint8_t atime_val = 0,again_val = 0;
    uint16_t again = 0;
    uint16_t saturation = 0, again_adjust_high_thr = 0, again_adjust_low_thr = 0;
    bool again_adjusted = false;
    
    tcs3707_i2c_read_block(obj->client, TCS3707_ADATA0L_REG, &data[0], 10);	
    obj->c_raw = (uint16_t)(((uint16_t)data[1]<<8)|data[0]);
    obj->r_raw   = (uint16_t)(((uint16_t)data[3]<<8)|data[2]);
    obj->g_raw = (uint16_t)(((uint16_t)data[5]<<8)|data[4]);
    obj->b_raw  = (uint16_t)(((uint16_t)data[7]<<8)|data[6]);
    obj->w_raw = (uint16_t)(((uint16_t)data[9]<<8)|data[8]);
    //TCS_DBG("clear_raw = %d,red_raw = %d,green_raw = %d,blue_raw = %d,wideband_raw = %d\n"
    //	,obj->c_raw, obj->r_raw, obj->g_raw, obj->b_raw, obj->w_raw);
    tcs3707_i2c_read(obj->client,TCS3707_ATIME_REG,&atime_val);
    tcs3707_i2c_read(obj->client,TCS3707_CFG1_REG,&again_val);
    again = als_gains[again_val&AGAIN_MASK];
    saturation = (atime_val + 1)*(ASTEP +1);
    again_adjust_high_thr = (saturation*8)/10;
    again_adjust_low_thr = saturation/10;	
    //TCS_DBG("again = %d,saturation = %d,again_adjust_high_thr = %d,again_adjust_low_thr = %d\n"
    //	,again, saturation, again_adjust_high_thr, again_adjust_low_thr);
    
    
    if(first_als){
        first_als = false;
        //init atime is 2.78ms, init again is 4x, saturation value is 999+1=1000;1000*80%=800
        //if change to 16x, not saturated, 800/4 = 200; if change to 64x, not saturated, 200/4 = 50;
        //if change to 256x, not saturated, 50/4 = 13;
        //use 1x 4x 16x 64x 256x gain values
        if(obj->c_raw <= 13){
            tcs3707_set_als_gain(obj->client,256);
            //TCS_DBG("tcs3707_report_als_data, quickly fix gain to 256x\n");
        }else if(obj->c_raw <= 50){
            tcs3707_set_als_gain(obj->client,64);
            //TCS_DBG("tcs3707_report_als_data, quickly fix gain to 64x\n");
        }else if(obj->c_raw <= 200){
            tcs3707_set_als_gain(obj->client,16);
            //TCS_DBG("tcs3707_report_als_data, quickly fix gain to 16x\n");
        }else if(obj->c_raw <= 800){			
            //TCS_DBG("tcs3707_report_als_data, keep 4x gain\n");
        }else{
            tcs3707_set_als_gain(obj->client,1);
            //TCS_DBG("tcs3707_report_als_data, quickly fix gain to 1x\n");
        }
        //change atime back to 100ms
        tcs3707_set_als_time_ms(obj->client, 100);
        again_adjusted = true;
        
    }else{
        //auto gain control, use 1x 4x 16x 64x 256x gain values
        if((again == 64)&&(obj->c_raw <= again_adjust_low_thr)){
            tcs3707_set_als_gain(obj->client,256);
            //TCS_DBG("tcs3707_report_als_data, set gain to 256x\n");
            again_adjusted = true;
        }else if(((again == 16)&&(obj->c_raw <= again_adjust_low_thr))
        ||((again == 256)&&(obj->c_raw >= again_adjust_high_thr))){
            tcs3707_set_als_gain(obj->client,64);
            //TCS_DBG("tcs3707_report_als_data, set gain to 64x\n");
            again_adjusted = true;
        }else if(((again == 4)&&(obj->c_raw <= again_adjust_low_thr))
        ||((again == 64)&&(obj->c_raw >= again_adjust_high_thr))){
            tcs3707_set_als_gain(obj->client,16);
            //TCS_DBG("tcs3707_report_als_data, set gain to 16x\n");
            again_adjusted = true;
        }else if(((again == 1)&&(obj->c_raw <= again_adjust_low_thr))
        ||((again == 16)&&(obj->c_raw >= again_adjust_high_thr))){
            tcs3707_set_als_gain(obj->client,4);
            //TCS_DBG("tcs3707_report_als_data, set gain to 4x\n");
            again_adjusted = true;
        }else if((again == 4)&&(obj->c_raw >= again_adjust_high_thr)){
            tcs3707_set_als_gain(obj->client,1);
            //TCS_DBG("tcs3707_report_als_data, set gain to 1x\n");
            again_adjusted = true;
        }
    }
    
    
    if(als_timer_stoped == false){
        mod_timer(&obj->als_timer, jiffies + 110/(1000/HZ));//110ms timer
        //TCS_DBG("tcs3707_report_als_data als_timer 110ms start!!!");
    }	
    if(again_adjusted == true){
        goto exit;
    }
    obj->cct = calculate_cct_lux(obj);
    //calculate lux and cct, need implement here
    
    
    
    exit:
    return;

}

static void tcs3707_als_work(struct work_struct *work)
{
    struct tcs3707_priv *tcs3707_data =
    container_of(work, struct tcs3707_priv, als_work);
    TCS3707_MUTEX_LOCK();
    tcs3707_report_als_data(tcs3707_data);
    TCS3707_MUTEX_UNLOCK();
};

void tcs3707_als_poll_handle(unsigned long data)
{		
    struct tcs3707_priv *tcs3707_data = (struct tcs3707_priv *)data;		
    schedule_work(&tcs3707_data->als_work);
}

static int tcs3707_als_enable(struct tcs3707_priv *obj, unsigned int enable)
{
    int ret = 0;
    TCS3707_MUTEX_LOCK();
    //TCS_DBG("tcs3707_als_enable, enable = %d\n",enable);
    if(enable == 1){
        if(als_opened == false){
            if(pon_is_opened == false){
                pon_is_opened = true;
                tcs3707_i2c_write(obj->client, TCS3707_ENABLE_REG, PON);
                mdelay(1);//delay 1ms	
                tcs3707_i2c_modify(obj->client, TCS3707_CONTROL_REG, ALS_MANUAL_AZ,ALS_MANUAL_AZ);//trigger a auto zero
            }		
        tcs3707_set_als_time_ms(obj->client, 3);//use a short atime to quickly fix a proper gain
        tcs3707_set_als_gain(obj->client,4);
        
        //use a short timer to quickly fix a again							
        mod_timer(&tcs3707_obj->als_timer, jiffies + 5/(1000/HZ));
        //TCS_DBG("tcs3707_als_enable, start als timer 5ms\n");
        first_als = true;
        als_opened = true;
        als_timer_stoped = false;
        }
    }else if(enable == 0){
        if(als_opened == true){
            als_opened = false;
            als_timer_stoped = true;
            tcs3707_i2c_modify(obj->client,TCS3707_ENABLE_REG,AEN,0);//close als 
        }		
    }else{
        //TCS_ERR("tcs3707_als_enable, invalid value of enable\n");
    }
    TCS3707_MUTEX_UNLOCK();
    
    return ret;
}
static long tcs3707_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{	
    struct i2c_client *client = (struct i2c_client *)file->private_data;
    struct tcs3707_priv *obj = i2c_get_clientdata(client);
    
    long err = 0;
    void __user *ptr = (void __user *)arg;
    
    uint32_t enable = 100;
    int als_recieved_value = -1;
    uint16_t als_data[7] = {0};
    uint16_t cct = 0;
    switch (cmd) {
        case TCS3707_ENABLE_ALS:		
            //TCS_DBG("tcs3707_unlocked_ioctl, case TCS3707_ENABLE_ALS\n");
            
            if (copy_from_user(&als_recieved_value, ptr, sizeof(als_recieved_value))) {
                err = -EFAULT;
                goto err_out;
            }
            //TCS_DBG("tcs3707_unlocked_ioctl, als_recieved_value = %d\n",als_recieved_value);
            if(als_recieved_value == 1){//enable als
                tcs3707_als_enable(obj, 1);
            }else if(als_recieved_value == 0){//disable als
                tcs3707_als_enable(obj, 0);
            }else{
                //TCS_ERR("tcs3707_unlocked_ioctl, als_recieved_value is invalid\n");
            }
            
            if (copy_to_user(ptr, &enable, sizeof(enable))) {
                err = -EFAULT;
                goto err_out;
            }
            break;
        case TCS3707_CALCULATE_CCT:
            TCS3707_MUTEX_LOCK();
            cct = obj->cct;
            TCS3707_MUTEX_UNLOCK();
            if (copy_to_user(ptr, &cct, sizeof(uint16_t))) {
                err = -EFAULT;
                goto err_out;
            }
            
            break;
        case TCS3707_READ_ALS:	
            //TCS_ERR("tcs3707_unlocked_ioctl, case TCS3707_READ_ALS\n");		
            TCS3707_MUTEX_LOCK();
            als_data[0] = obj->c_raw;
            als_data[1] = obj->r_raw;
            als_data[2] = obj->g_raw;
            als_data[3] = obj->b_raw;
            als_data[4] = obj->w_raw;
            als_data[5] = obj->als_gain;
            als_data[6] = obj->als_time;
            TCS3707_MUTEX_UNLOCK();
            if (copy_to_user(ptr, &als_data[0], sizeof(als_data))) {
                err = -EFAULT;
                goto err_out;
            }
            
            break;
        
        default:
            //TCS_ERR("%s not supported = 0x%04x\n", __func__, cmd);
            err = -ENOIOCTLCMD;
            break;
    }
    
    err_out:
    return err;
}
static ssize_t tcs3707_read(struct file* file, char __user* buf, size_t count, loff_t* f_pos)
{
	struct i2c_client *client = (struct i2c_client *)file->private_data;
	struct tcs3707_priv *obj = i2c_get_clientdata(client);   
    ssize_t          	status = 0;
    unsigned long       missing;
    uint16_t cct = 0;
    TCS3707_MUTEX_LOCK();
    cct = obj->cct;
    missing = copy_to_user(buf, &cct, count);
    if (missing == 0) {
        TCS_DBG("copy to user sucsess");
    }
    else 
    {//if missing > 0, means missing bytes not copied
        TCS_ERR("copy to user error") ;
        status = -EFAULT;
    }
    
    TCS3707_MUTEX_UNLOCK();
    return status;
}


/*----------------------------------------------------------------------------*/

static int tcs3707_parse_tree(struct tcs3707_priv *obj){
    obj->vdd = regulator_get_optional(&obj->client->dev, "vdd");//vdd
    if (IS_ERR(obj->vdd) || obj->vdd == NULL) {
        TCS_ERR("no regulator vdd.\n");
        return 0;
    }
    return 1;
}
static int tcs3707_power_up(struct tcs3707_priv *obj){
    int rc = 0;
    if (obj->vdd) {
        rc = regulator_enable(obj->vdd);
    if (rc) {
        TCS_ERR("fail to turn on regulator");
        return rc;
    }
} 	

return rc;
}
static void tcs3707_power_down(struct tcs3707_priv *obj){
    if (obj->vdd) {
        regulator_put(obj->vdd);
        obj->vdd = NULL;
    } 	
}

static const struct file_operations tcs3707_fops = {
    .owner = THIS_MODULE,
    .open = tcs3707_open,
    .release = tcs3707_release,
    .read = tcs3707_read,
    .unlocked_ioctl = tcs3707_unlocked_ioctl,
};

/*----------------------------------------------------------------------------*/
static struct miscdevice tcs3707_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "tcs3707",
    .fops = &tcs3707_fops,
};
static int tcs3707_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct tcs3707_priv *obj;
    
    int err = 0;
    //int rc = -1;
    TCS_FUN();
    /* configure the gpio pins */
    obj = kzalloc(sizeof(*obj), GFP_KERNEL);
    if (!(obj)) {
        err = -ENOMEM;
        goto exit;
    }
    memset(obj, 0, sizeof(*obj));
    tcs3707_obj = obj;
    
    obj->client = client;
    
    err = tcs3707_parse_tree(obj);
    if(err == 0){
        TCS_ERR("no vdd.\n");
    }
    
    i2c_set_clientdata(client, obj);
    
    tcs3707_i2c_client = client;
    
    err = tcs3707_power_up(obj);
    if(err){
        TCS_ERR("power_up failed.\n");
        goto power_up_fail;
    }
    err = tcs3707_init_client(client);
    if (err < 0)
        goto exit_init_failed;
    //TCS_LOG("tcs3707_init_client() OK!\n");
    err = misc_register(&tcs3707_device);
    if (err) {
        TCS_ERR("tcs3707_device register failed\n");
        goto exit_misc_device_register_failed;
    }
    
    
    INIT_WORK(&obj->als_work, tcs3707_als_work);
    init_timer(&obj->als_timer);
    obj->als_timer.expires	= jiffies + 100/(1000/HZ);
    obj->als_timer.function	= tcs3707_als_poll_handle;
    obj->als_timer.data	= (unsigned long)obj;
    
    
    
    
    tcs3707_init_flag = 0;
    TCS_ERR("%s: OK\n", __func__);
    return 0;
    
    exit_misc_device_register_failed:
        misc_deregister(&tcs3707_device);
    exit_init_failed:
        tcs3707_power_down(obj);
    power_up_fail:
    exit:
        tcs3707_i2c_client = NULL;
        kfree(obj);
        //TCS_ERR("%s: err = %d\n", __func__, err);
        tcs3707_init_flag = -1;
        return -1;
}
/*----------------------------------------------------------------------------*/
static int tcs3707_i2c_remove(struct i2c_client *client)
{
misc_deregister(&tcs3707_device);

    tcs3707_i2c_client = NULL;
    i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));
    
    return 0;
}
/*----------------------------------------------------------------------------*/
static int __init tcs3707_init(void)
{
    TCS_FUN();
    
    if (i2c_add_driver(&tcs3707_i2c_driver)) {
        TCS_ERR("add driver error\n");
        return -1;
    }
    if (-1 == tcs3707_init_flag)
        return -1;
    
    return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit tcs3707_exit(void)
{
    TCS_FUN();
    
    i2c_del_driver(&tcs3707_i2c_driver);
}
#endif
/*----------------------------------------------------------------------------*/
module_init(tcs3707_init);
module_exit(tcs3707_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Yongwu Wang");
MODULE_DESCRIPTION("tcs3707 driver");
MODULE_LICENSE("GPL");
