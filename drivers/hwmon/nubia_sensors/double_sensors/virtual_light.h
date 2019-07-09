/*
 *  virtual_proximity.h - Linux kernel modules for proximity sensor
 *
 *  Copyright (c) 2013, All rights reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 and
 *  only version 2 as published by the Free Software Foundation.

 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __VIRTUAL_LIGHT_H__
#define __VIRTUAL_LIGHT_H__

#include <linux/types.h>
#include <linux/gpio.h>

//#define DEBUG_ON //DEBUG SWITCH
//nubia add debug log mesage
//#define TMD3702_DEBUG_REGISTER_RW


//#define ABI_SET_GET_REGISTERS
struct virtual_light_data {
	struct device *als_dev;
	struct input_dev *als_input_dev;
	struct mutex rw_lock;

	volatile u8 fb_status;
    u8 report_fb_status;
    int report_value;
    volatile bool enable_status;
	bool flag_prox_debug;

#ifdef CONFIG_NUBIA_SWITCH_LCD
    struct notifier_block ps_fb_notify;
#endif
    u8 debug_level;
};
#if 0
enum{
	ERR_NAKED_CAL = 1,
	ERR_THRES_CAL,
	ERR_FILE_OPS,
	ERR_DEV_OPS,
	ERR_OTHER,
};
#endif
//extern struct virtual_proximity_data *virtual_als_data;
extern struct input_dev *virtual_als_input_dev;
extern int virtual_light_register(void);
extern void virtual_light_unregister(void);

#endif
