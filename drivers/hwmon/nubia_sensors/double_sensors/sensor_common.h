/*
* This file is part of the tmd2725 sensor driver.
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
* 02110-1301 USA
*
*Reversion
*
====================================================================================
*/
#ifndef __SENSOR_COMMON_H__
#define __SENSOR_COMMON_H__

#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/string.h>
#include <linux/irq.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <asm/atomic.h>
//#include <linux/wakelock.h>

#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>

#include "virtual_light.h"
#include "virtual_proximity.h"

#define DRIVER_VERSION "1.0"

#define DEBUG_ON //DEBUG SWITCH
#define LOG_TAG "DOUBLE_SENSOR"
#define SENSOR_LOG_LEVEL 1

#define SENSOR_LOG_ERROR(fmt, args...) printk(KERN_ERR "[%s] [%s:%d] " fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#define SENSOR_LOG_INFO(fmt, args...)  printk(KERN_INFO "[%s] [%s:%d] "  fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#define SENSOR_LOG_DEBUG_IF(en, fmt, args...) \
do { \
    if (en>=SENSOR_LOG_LEVEL) { \
        printk(KERN_ERR "[%s] [%s:%d] " fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args); \
    }; \
} while (0)

#define SENSOR_LOG_IF(en, fmt, args...) \
do { \
    if (en>=SENSOR_LOG_LEVEL) { \
        printk(KERN_ERR "[%s] [%s:%d] " fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args); \
    }; \
} while (0)

#ifdef  DEBUG_ON
#define SENSOR_LOG_DEBUG(fmt, args...) printk(KERN_DEBUG "[%s] [%s:%d] "  fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#else
#define SENSOR_LOG_DEBUG(fmt, args...)
#endif

#ifdef AMS_MUTEX_DEBUG
#define AMS_MUTEX_LOCK(m) { \
        printk(KERN_INFO "%s: Mutex Lock\n", __func__); \
        mutex_lock(m); \
    }
#define AMS_MUTEX_UNLOCK(m) { \
        printk(KERN_INFO "%s: Mutex Unlock\n", __func__); \
        mutex_unlock(m); \
    }
#else
#define AMS_MUTEX_LOCK(m) { \
        mutex_lock(m); \
    }
#define AMS_MUTEX_UNLOCK(m) { \
        mutex_unlock(m); \
    }
#endif

int sensor_create_sysfs_interfaces(struct device *dev, struct device_attribute *dev_attrs, int count);
void sensor_remove_sysfs_interfaces(struct device *dev, struct device_attribute *dev_attrs, int count);
int sensor_write_file(char *file_path, const char *write_buf, int count);
int sensor_read_file(char *file_path, char *read_buf ,int count);

extern int tmd2725_init(void);
extern void tmd2725_exit(void);
extern int stk3x3x_init(void);
extern void stk3x3x_exit(void);
extern int rgb_bh1745_init(void);
extern void rgb_bh1745_exit(void);

#endif
