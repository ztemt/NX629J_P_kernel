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

#ifndef __VIRTUAL_PROXIMITY_H__
#define __VIRTUAL_PROXIMITY_H__

#include <linux/types.h>
#include <linux/gpio.h>

//#define DEBUG_ON //DEBUG SWITCH
//nubia add debug log mesage
//#define TMD3702_DEBUG_REGISTER_RW


//#define ABI_SET_GET_REGISTERS
struct virtual_proximity_data {
	struct device *proximity_dev;
	struct input_dev *ps_input_dev;
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

//extern struct virtual_proximity_data *vp_data;
extern struct input_dev *virtual_ps_input_dev;
extern int virtual_proximity_register(void);
extern void virtual_proximity_unregister(void);
#endif
