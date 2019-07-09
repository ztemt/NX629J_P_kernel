/* Copyright (c) 2014-2015 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt) "dock: %s: " fmt, __func__

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>
#include <linux/debugfs.h>
#include <linux/pmic-voter.h>
#include "smb5-lib.h"
#if defined(CONFIG_FB)
#include <linux/fb.h>
#endif
extern int nubia_get_hw_id(void);
/***********************************************************************/

enum channel{
	CHG_CHANNEL_NONE	= 0,
	CHG_CHANNEL_TYPEC	= 1,
	CHG_CHANNEL_DOCK	= 2,
};

struct dt_params {
	int			cfg_typec_vbus_int_gpio;
	//int			cfg_typec_vbus_en_gpio;
	int			cfg_dock_vbus_int_gpio;
	int			cfg_dock_vbus_en_gpio;
};

struct dock_chip
{
	struct device			*dev;
	struct dt_params		dt;
	struct mutex			control_lock;
	struct power_supply		*batt_psy;
	struct power_supply		*usb_psy;
	struct power_supply		*main_psy;
	struct pinctrl *dock_pinctrl;
	struct pinctrl_state *dock_int_gpio_active;
	struct pinctrl_state *dock_int_gpio_suspend;
	struct delayed_work	    monitor_work;
	struct votable			*apsd_rerun_votable;
	struct votable			*dock_chg_enable_votable;
	struct votable			*awake_votable;
	int						typec_online;
	int						dock_online;
	enum channel			chg_channel;
	
	
};

static enum power_supply_property dock_main_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_DOCK_RERUN_APSD,
	POWER_SUPPLY_PROP_DOCK_ENABLE_FLOAT,
};

static int dock_charger_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{	
    //struct dock_chip *chip = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_PRESENT:
		break;
	default:
		return -EINVAL;
	}
	
	return 0;
}

static int dock_charger_set_property(struct power_supply *psy,
				     enum power_supply_property prop,
				     const union power_supply_propval *val)
{
	//struct dock_chip *chip = power_supply_get_drvdata(psy);
	int rc = 0;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		break;
	default:
		return 0;
	}

	return rc;
}

static int dock_charger_property_is_writeable(struct power_supply *psy,
					      enum power_supply_property prop)
{
	switch (prop) {
		case POWER_SUPPLY_PROP_CHARGING_ENABLED:
			return 1;
		default:
			break;
	}

	return 0;
}

static void dock_charger_monitor_work(struct work_struct *work)
{
	int irq_typec = 1;
	int irq_dock = 1;
	union power_supply_propval val = {0, };
	struct dock_chip *chip = container_of(work, struct dock_chip, monitor_work.work);
	
	irq_typec = gpio_get_value(chip->dt.cfg_typec_vbus_int_gpio);	
	irq_dock = gpio_get_value(chip->dt.cfg_dock_vbus_int_gpio);

	pr_err("typec_irq:%d,dock_irq:%d,chg_channel:%d\n",irq_typec,irq_dock,chip->chg_channel);

	switch(chip->chg_channel) {
	case CHG_CHANNEL_NONE:
		if(irq_dock == 0){
			//dock is online
			val.intval = 1;
			power_supply_set_property(chip->batt_psy, POWER_SUPPLY_PROP_DOCK_ENABLE_FLOAT, &val);
			chip->chg_channel = CHG_CHANNEL_DOCK;
		}else if(irq_typec == 0)
			chip->chg_channel = CHG_CHANNEL_TYPEC;
		break;
	case CHG_CHANNEL_TYPEC:
		if(irq_typec == 1){
			if(irq_dock == 0){
				val.intval = 1;
				power_supply_set_property(chip->batt_psy, POWER_SUPPLY_PROP_DOCK_ENABLE_FLOAT, &val);
				chip->chg_channel = CHG_CHANNEL_DOCK;
			}else{
				val.intval = 0;
				power_supply_set_property(chip->batt_psy, POWER_SUPPLY_PROP_DOCK_ENABLE_FLOAT, &val);
				chip->chg_channel = CHG_CHANNEL_NONE;
			}
		}
		break;
	case CHG_CHANNEL_DOCK:
		if(irq_typec ==0){
			chip->chg_channel = CHG_CHANNEL_TYPEC;
		}else if(irq_dock == 1){
			val.intval = 0;
			power_supply_set_property(chip->batt_psy, POWER_SUPPLY_PROP_DOCK_ENABLE_FLOAT, &val);
			chip->chg_channel = CHG_CHANNEL_NONE;
		}
		break;
	default:
		break;
	}
	if((irq_typec ==1)&&(irq_dock==1))
		vote(chip->awake_votable, DOCK_VOTER, false, 0);
	else
		vote(chip->awake_votable, DOCK_VOTER, true, 0);
	pr_err("chg_channel:%d\n",chip->chg_channel);
	return;
}

static irqreturn_t dock_charger_vbus_irq_handler(int irq, void *_chip)
{
	int irq_typec = 1;
	int irq_dock = 1;
	int schedule_ms = 10000;
	struct dock_chip *chip = _chip;
	irq_typec = gpio_get_value(chip->dt.cfg_typec_vbus_int_gpio);
	irq_dock = gpio_get_value(chip->dt.cfg_dock_vbus_int_gpio);
	pr_err("typec_irq=%d,dock_irq=%d\n",irq_typec,irq_dock);
	if (((irq_typec==1)&&(irq_dock==1))||(chip->chg_channel == CHG_CHANNEL_NONE))
		schedule_ms = 200;
	else
		schedule_ms = 10000;
	cancel_delayed_work_sync(&chip->monitor_work);
	schedule_delayed_work(&chip->monitor_work, msecs_to_jiffies(schedule_ms));
	return IRQ_HANDLED;
}

static int dock_charger_parse_dt(struct dock_chip *chip)
{

	struct device_node *node = chip->dev->of_node;
	
	chip->dt.cfg_typec_vbus_int_gpio = of_get_named_gpio(node, "dock,typec-vbus-int-gpio", 0);

	//chip->dt.cfg_typec_vbus_en_gpio = of_get_named_gpio(node, "dock,typec-vbus-en-gpio", 0);
			
	chip->dt.cfg_dock_vbus_int_gpio = of_get_named_gpio(node, "dock,dock-vbus-int-gpio", 0);

	chip->dt.cfg_dock_vbus_en_gpio = of_get_named_gpio(node, "dock,dock-vbus-en-gpio", 0);
		
	pr_err("NEO:typec_vbus_int:%d,dock_vbus_int:%d,dock_vbus_en:%d\n",
			chip->dt.cfg_typec_vbus_int_gpio,chip->dt.cfg_dock_vbus_int_gpio,chip->dt.cfg_dock_vbus_en_gpio);

	return 0;
}


static const struct power_supply_desc dock_psy_desc = {
	.name = "dock-charger",
	.type = POWER_SUPPLY_TYPE_DOCK,
	.properties = dock_main_props,
	.num_properties = ARRAY_SIZE(dock_main_props),
	.get_property = dock_charger_get_property,
	.set_property = dock_charger_set_property,
	.property_is_writeable = dock_charger_property_is_writeable,
};

static int dock_charger_init_power_supply(struct dock_chip *chip)
{
	struct power_supply_config main_cfg = {};
	int rc = 0;

	chip->batt_psy = power_supply_get_by_name("battery");
	if (!chip->batt_psy){
		pr_err("Couldn't get battery power supply.\n");
		rc = -EPROBE_DEFER;
		return rc;
	}

	chip->usb_psy = power_supply_get_by_name("usb");
	if (!chip->usb_psy){
		pr_err("Couldn't get usb power supply.\n");
		rc = -EPROBE_DEFER;
		return rc;
	}

	main_cfg.drv_data = chip;
	main_cfg.of_node = chip->dev->of_node;
	chip->main_psy = devm_power_supply_register(chip->dev,
						   &dock_psy_desc,
						   &main_cfg);
	if (IS_ERR(chip->main_psy)) {
		pr_err("Couldn't register battery power supply\n");
		return PTR_ERR(chip->main_psy);
	}
	return rc;
}

static int dock_charger_probe(struct platform_device *pdev)
{	
    int rc;
	struct dock_chip *chip;
	int ret;
	int pcb_type;
	
	pr_err("dock Probe start.\n");
	
	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&pdev->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	pcb_type = nubia_get_hw_id();
	pr_err("pcb_type=%d\n",pcb_type);
	//in 629, dock only support pcb_C,pcb_D,and higher, 
	if(pcb_type < 2)
		goto cleanup;
   
	chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, chip);
	
	chip->awake_votable = find_votable("AWAKE");
	if (IS_ERR(chip->awake_votable)) {
		dev_err(chip->dev, "Couldn't find votable.\n");
		rc = PTR_ERR(chip->awake_votable);
		goto cleanup;
	}
	
	rc = dock_charger_init_power_supply(chip);
	if (rc < 0) {
		pr_err( "Couldn't initialize power supply rc=%d\n", rc);
		goto cleanup;
	}
	mutex_init(&chip->control_lock);
	INIT_DELAYED_WORK(&chip->monitor_work, dock_charger_monitor_work);
	
	rc = dock_charger_parse_dt(chip);
	if(rc < 0)
		goto cleanup;

	chip->dock_pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->dock_pinctrl)) {
		pr_err("NEO:No pinctrl config specified\n");
		ret = PTR_ERR(chip->dev);
		return ret;
	}
	chip->dock_int_gpio_active = pinctrl_lookup_state(chip->dock_pinctrl, "dock_active");
	if (IS_ERR_OR_NULL(chip->dock_int_gpio_active)) {
		pr_err("NEO:No pinctrl config dock_int_gpio_active specified\n");
		ret = PTR_ERR(chip->dock_int_gpio_active);
		return ret;
	}
	chip->dock_int_gpio_suspend = pinctrl_lookup_state(chip->dock_pinctrl, "dock_suspend");
	if (IS_ERR_OR_NULL(chip->dock_int_gpio_suspend)) {
		pr_err("NEO:No pinctrl config dock_int_gpio_suspend specified\n");
		ret = PTR_ERR(chip->dock_int_gpio_suspend);
		return ret;
	}

	ret = pinctrl_select_state(chip->dock_pinctrl,
			chip->dock_int_gpio_active);
	if (ret < 0) {
		pr_err("NEO:fail to select pinctrl active rc=%d\n", ret);
		return ret;
	}
	
	ret = gpio_direction_input(chip->dt.cfg_typec_vbus_int_gpio);
	pr_err("set cfg_typec_vbus_int_gpio input ret=%d\n",ret);
	rc = devm_request_threaded_irq(chip->dev, gpio_to_irq(chip->dt.cfg_typec_vbus_int_gpio), 
			NULL, 
			dock_charger_vbus_irq_handler, 
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT, 
			"typec-vbus-irq",	
			chip);
	if (rc < 0){						
		pr_err("Unable to request typec_vbus_irq: %d\n",rc);
		goto cleanup;
	}
	
	gpio_direction_input(chip->dt.cfg_dock_vbus_int_gpio);
	pr_err("set cfg_dock_vbus_int_gpio input ret=%d\n",ret);
	rc = devm_request_threaded_irq(chip->dev, gpio_to_irq(chip->dt.cfg_dock_vbus_int_gpio), 
			NULL, 
			//dock_charger_dock_vbus_irq_handler, 
			dock_charger_vbus_irq_handler,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT, 
			"dock-vbus-irq",	
			chip);

	if (rc < 0){						
		pr_err( "Unable to request dock_vbus_irq: %d\n",rc);
		goto cleanup;
	}

	//gpio_set_value(chip->dt.cfg_dock_vbus_en_gpio,1);
	enable_irq_wake(gpio_to_irq(chip->dt.cfg_typec_vbus_int_gpio));
	enable_irq_wake(gpio_to_irq(chip->dt.cfg_dock_vbus_int_gpio));
		
	chip->typec_online = gpio_get_value(chip->dt.cfg_typec_vbus_int_gpio);	
	chip->dock_online = gpio_get_value(chip->dt.cfg_dock_vbus_int_gpio);
	chip->chg_channel = CHG_CHANNEL_NONE;
	
	if ((chip->typec_online == 0) || (chip->dock_online ==0))
		schedule_delayed_work(&chip->monitor_work, msecs_to_jiffies(10000));
	
	pr_err("dock Probe Success,typec_gpio is %d, dock_gpio is %d.\n", chip->typec_online,chip->dock_online);
	return 0;
	
cleanup:
	pr_err( "dock Probe :cleanup\n");
	if (chip->main_psy)
		power_supply_unregister(chip->main_psy);

	platform_set_drvdata(pdev, NULL);
	return rc;
}

static int dock_charger_remove(struct platform_device *pdev)
{	
	struct dock_chip *chip = platform_get_drvdata(pdev);

	kfree(chip);
	return 0;
}

static struct of_device_id dock_dt_match[] = {
	{
		.compatible     = "dock,charger",
	},
	{ },
};

static struct platform_driver dock_charger_driver = {	
	.probe	= dock_charger_probe,	
	.remove	= dock_charger_remove,	
	.driver	= {		
		.name	= "dock_charger_driver",		
		.owner	= THIS_MODULE,		
		.of_match_table = dock_dt_match,	
	},
};  

static int __init dock_charger_init(void) 
{    
	return platform_driver_register(&dock_charger_driver);	 
}

static void __exit dock_charger_exit(void)
{
	return platform_driver_unregister(&dock_charger_driver);
}

late_initcall(dock_charger_init);
module_exit(dock_charger_exit);

MODULE_AUTHOR("hong.danlong");
MODULE_DESCRIPTION("NUBIA dock charger driver");
MODULE_LICENSE("GPL");
