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
#include "ams_common.h"
#include "ams_tmd2725.h"

int sensor_regulator_configure(struct tmd2725_chip *chip, bool on)
{
	int rc;
	if (unlikely(IS_ERR_OR_NULL(chip->pdata))) {
		SENSOR_LOG_ERROR("null pointer.\n");
		rc = -PTR_ERR(chip->pdata);
		return rc;
	}

	if (!on) {
		if(!IS_ERR_OR_NULL(chip->pdata->vdd)) {
			if (regulator_count_voltages(chip->pdata->vdd) > 0)
				regulator_set_voltage(chip->pdata->vdd, 0, POWER_VDD_MAX_UV);
			regulator_put(chip->pdata->vdd);
			regulator_disable(chip->pdata->vdd);
		}
		if(!IS_ERR_OR_NULL(chip->pdata->vio)) {
			if (regulator_count_voltages(chip->pdata->vio) > 0)
				regulator_set_voltage(chip->pdata->vio, 0, POWER_VIO_MAX_UV);
			regulator_put(chip->pdata->vio);
			regulator_disable(chip->pdata->vio);
		}
	} else {
		chip->pdata->vdd = regulator_get(&chip->client->dev, "vdd");
		if (IS_ERR_OR_NULL(chip->pdata->vdd)) {
			rc = -PTR_ERR(chip->pdata->vdd);
			SENSOR_LOG_ERROR("Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}
		if (!IS_ERR_OR_NULL(chip->pdata->vdd)) {
			if (regulator_count_voltages(chip->pdata->vdd) > 0) {
				rc = regulator_set_voltage(chip->pdata->vdd,
					POWER_VDD_MIN_UV, POWER_VDD_MAX_UV);
				if (rc) {
					SENSOR_LOG_ERROR("Regulator set failed vdd rc=%d\n",
						rc);
					rc = - EINVAL;
					goto reg_vdd_put;
				}
			}

			rc = regulator_enable(chip->pdata->vdd);
			if (rc) {
				SENSOR_LOG_ERROR("Regulator enable vdd failed. rc=%d\n", rc);
				rc = - EINVAL;
				goto reg_vdd_put;
			}
			SENSOR_LOG_INFO("vdd regulator ok\n");
		}

		chip->pdata->vio = regulator_get(&chip->client->dev, "vio");
		if (IS_ERR_OR_NULL(chip->pdata->vio)) {
			rc = -PTR_ERR(chip->pdata->vio);
			SENSOR_LOG_ERROR("Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}
		if (!IS_ERR_OR_NULL(chip->pdata->vio)) {
			if (regulator_count_voltages(chip->pdata->vio) > 0) {
				rc = regulator_set_voltage(chip->pdata->vio,
					POWER_VIO_MIN_UV, POWER_VIO_MAX_UV);
				if (rc) {
					SENSOR_LOG_ERROR("Regulator set failed vdd rc=%d\n",
						rc);
					rc = - EINVAL;
					goto reg_vio_put;
				}
			}

			rc = regulator_enable(chip->pdata->vio);
			if (rc) {
				SENSOR_LOG_ERROR("Regulator enable vdd failed. rc=%d\n", rc);
				rc = - EINVAL;
				goto reg_vio_put;
			}
			SENSOR_LOG_INFO("vio regulator ok\n");
		}
	}
	return 0;
reg_vio_put:
	regulator_put(chip->pdata->vio);
reg_vdd_put:
	regulator_put(chip->pdata->vdd);
	return rc;
}


int sensor_regulator_power_on(struct tmd2725_chip *chip, bool on)
{
	int rc = 0;

	if (IS_ERR_OR_NULL(chip->pdata))
		return -EINVAL;

	if (!on) {
		if (!IS_ERR_OR_NULL(chip->pdata->vdd)) {
			rc = regulator_disable(chip->pdata->vdd);
			if (rc) {
				SENSOR_LOG_ERROR(
					"Regulator vdd disable failed rc=%d\n", rc);
				return rc;
			}
		}
		if (!IS_ERR_OR_NULL(chip->pdata->vio)) {
			rc = regulator_disable(chip->pdata->vio);
			if (rc) {
				SENSOR_LOG_ERROR(
					"Regulator vdd disable failed rc=%d\n", rc);
				return rc;
			}
		}
	} else {
		if (!IS_ERR_OR_NULL(chip->pdata->vdd)) {
			rc = regulator_enable(chip->pdata->vdd);
			if (rc) {
				SENSOR_LOG_ERROR(
					"Regulator vdd enable failed rc=%d\n", rc);
				return rc;
			}
		}
		if (!IS_ERR_OR_NULL(chip->pdata->vio)) {
			rc = regulator_enable(chip->pdata->vio);
			if (rc) {
				SENSOR_LOG_ERROR(
					"Regulator vdd enable failed rc=%d\n", rc);
				return rc;
			}
		}
	}

	SENSOR_LOG_INFO("power state : (%s)\n", on ? "on":"off");
	mdelay(5);
	return rc;
}

int sensor_hw_pinctrl_init(struct tmd2725_chip *chip, struct device *dev)
{
	int rc;
	if (unlikely(IS_ERR_OR_NULL(chip) || IS_ERR_OR_NULL(dev)))
		return -EINVAL;

	chip->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		SENSOR_LOG_ERROR("data->pinctrl is NULL\n");
		return PTR_ERR(chip->pinctrl);
	}

	chip->pin_default = pinctrl_lookup_state(chip->pinctrl, "tmd2725_default");
	if (IS_ERR_OR_NULL(chip->pin_default)) {
		SENSOR_LOG_ERROR("lookup default state failed\n");
		return PTR_ERR(chip->pin_default);
	}

	chip->pin_sleep = pinctrl_lookup_state(chip->pinctrl, "tmd2725_sleep");
	if (IS_ERR_OR_NULL(chip->pin_sleep)) {
		SENSOR_LOG_ERROR("lookup sleep state failed\n");
		return PTR_ERR(chip->pin_sleep);
	}

	if (!IS_ERR_OR_NULL(chip->pinctrl)) {
		rc = pinctrl_select_state(chip->pinctrl, chip->pin_default);
		if (rc) {
			SENSOR_LOG_ERROR("select default state failed\n");
			return rc;
		}
	}
	SENSOR_LOG_INFO("pinctrl init success\n");
	return 0;
}

void sensor_irq_enable(struct tmd2725_chip *data, bool enable, bool flag_sync)
{
	SENSOR_LOG_DEBUG_IF(data->pdata->debug_level, "irq %s\n",enable ? "enable" : "disable");
	if (enable == data->irq_enabled) {
		SENSOR_LOG_DEBUG("doubule %s irq %d\n",enable? "enable" : "disable",
			data->irq);
		return;
	} else {
			data->irq_enabled = enable;
	}
	if (enable) {
		enable_irq(data->irq);
	} else {
		if (flag_sync) {
			disable_irq(data->irq);
		} else {
			disable_irq_nosync(data->irq);
		}
	}
}
void sensor_quick_sort(int *pdata, int len, int left, int right)
{
	int i, j, tmp, t;

	if(left > right || IS_ERR_OR_NULL(pdata) || len == 0)
		return;
	tmp = pdata[left];
	i = left;
	j = right;
	while(i != j){
		while(pdata[j] >= tmp && i < j)
			j--;
		while(pdata[i] <= tmp && i < j)
			i++;

		if (i < j) {
			t = pdata[i];
			pdata[i] = pdata[j];
			pdata[j] = t;
		}
	}
	pdata[left] = pdata[i];
	pdata[i] = tmp;
	sensor_quick_sort(pdata, len, left, i-1);
	sensor_quick_sort(pdata, len, i + 1, right);
}

