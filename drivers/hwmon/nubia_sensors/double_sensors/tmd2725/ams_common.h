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
when         	who         		Remark : what, where, why          		version
-----------   	------------     	-------------------------           	--------
201x/x/xx       AMS															 V1.0
2016/7/11       Bao QI              Tmd2725 code refcatoring                 V2.0
====================================================================================
*/
#ifndef __AMS_COMMON_H__
#define __AMS_COMMON_H__

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

#include "ams_tmd2725.h"
#include "../sensor_common.h"

/* POWER SUPPLY VOLTAGE RANGE */
#define POWER_VDD_MIN_UV  2850000
#define POWER_VDD_MAX_UV  3300000
#define POWER_VIO_MIN_UV  1750000
#define POWER_VIO_MAX_UV  1950000

enum {
    ERR_NAKED_CAL = 1,
    ERR_THRES_CAL,
    ERR_FILE_OPS,
    ERR_DEV_OPS,
    ERR_OTHER,
};

int sensor_regulator_power_on(struct tmd2725_chip *chip, bool on);
int sensor_regulator_configure(struct tmd2725_chip *chip, bool on);
int sensor_hw_pinctrl_init(struct tmd2725_chip *chip, struct device *dev);
void sensor_irq_enable(struct tmd2725_chip *chip, bool enable, bool flag_sync);
void sensor_quick_sort(int *pdata, int len, int left, int right);
#endif
