// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2021, Google LLC
 *
 * Pogo management driver
 */

#include <linux/delay.h>
#include <linux/extcon.h>
#include <linux/extcon-provider.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/usb/tcpm.h>

#include "../tcpci.h"
#include "google_psy.h"
#include "tcpci_max77759.h"

#define POGO_TIMEOUT_MS 10000
#define POGO_USB_CAPABLE_THRESHOLD_UV 10500000
#define POGO_USB_RETRY_COUNT 10
#define POGO_USB_RETRY_INTEREVAL_MS 50
#define POGO_PSY_DEBOUNCE_MS 50
#define POGO_PSY_NRDY_RETRY_MS 500
#define POGO_ACC_GPIO_DEBOUNCE_MS 20

#define KEEP_USB_PATH 2

enum pogo_event_type {
	/* Reported when docking status changes */
	EVENT_DOCKING,
	/* Enable USB-C data, when pogo usb data is active */
	EVENT_MOVE_DATA_TO_USB,
	/* Enable pogo data, when pogo is available */
	EVENT_MOVE_DATA_TO_POGO,
	/* Retry reading power supply voltage to detect dock type */
	EVENT_RETRY_READ_VOLTAGE,
	/* Reported when data over USB-C is enabled/disabled */
	EVENT_DATA_ACTIVE_CHANGED,

	/* 5 */
	/* Hub operation; workable only if hub_embedded is true */
	EVENT_ENABLE_HUB,
	EVENT_DISABLE_HUB,
	EVENT_HALL_SENSOR_ACC_DETECTED,
	EVENT_HALL_SENSOR_ACC_MALFUNCTION,
	EVENT_HALL_SENSOR_ACC_UNDOCKED,

	/* 10 */
	EVENT_POGO_ACC_DETECTED,
	EVENT_POGO_ACC_CONNECTED,
};

static bool modparam_force_usb;
module_param_named(force_usb, modparam_force_usb, bool, 0644);
MODULE_PARM_DESC(force_usb, "Force enabling usb path over pogo");

struct pogo_event {
	struct kthread_delayed_work work;
	struct pogo_transport *pogo_transport;
	enum pogo_event_type event_type;
};

struct pogo_transport {
	struct device *dev;
	struct max77759_plat *chip;
	struct logbuffer *log;
	int pogo_gpio;
	int pogo_irq;
	int pogo_data_mux_gpio;
	int pogo_hub_sel_gpio;
	int pogo_hub_reset_gpio;
	int pogo_ovp_en_gpio;
	int pogo_acc_gpio;
	int pogo_acc_irq;
	unsigned int pogo_acc_gpio_debounce_ms;
	struct regulator *hub_ldo;
	struct regulator *acc_detect_ldo;
	/* Raw value of the active state. Set to 1 when pogo_ovp_en is ACTIVE_HIGH */
	bool pogo_ovp_en_active_state;
	struct pinctrl *pinctrl;
	struct pinctrl_state *susp_usb_state;
	struct pinctrl_state *susp_pogo_state;
	struct pinctrl_state *hub_state;
	/* When true, Usb data active over pogo pins. */
	bool pogo_usb_active;
	/* When true, Pogo connection is capable of usb transport. */
	bool pogo_usb_capable;
	/* When true, both pogo and usb-c have equal priority. */
	bool equal_priority;
	/* When true, USB data is routed to the hub. */
	bool pogo_hub_active;
	/* When true, the board has a hub embedded in the pogo system. */
	bool hub_embedded;
	/* When true, pogo takes higher priority */
	bool force_pogo;
	/* When true, pogo irq is enabled */
	bool pogo_irq_enabled;
	/* When true, hall1_s sensor reports attach event */
	bool hall1_s_state;
	/* When true, the path won't switch to pogo if accessory is attached */
	bool mfg_acc_test;
	struct kthread_worker *wq;
	/* To read voltage at the pogo pins */
	struct power_supply *pogo_psy;
	/* Retry when voltage is less than POGO_USB_CAPABLE_THRESHOLD_UV */
	unsigned int retry_count;
	/* To signal userspace extcon observer */
	struct extcon_dev *extcon;
};

static const unsigned int pogo_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_DOCK,
};

static void pogo_transport_event(struct pogo_transport *pogo_transport,
				 enum pogo_event_type event_type, int delay_ms);

static void update_extcon_dev(struct pogo_transport *pogo_transport, bool docked, bool usb_capable)
{
	int ret;

	ret = extcon_set_state_sync(pogo_transport->extcon, EXTCON_USB, usb_capable ? 1 : 0);
	if (ret)
		dev_err(pogo_transport->dev, "%s Failed to %s EXTCON_USB\n", __func__,
			usb_capable ? "set" : "clear");
	ret = extcon_set_state_sync(pogo_transport->extcon, EXTCON_DOCK, docked ? 1 : 0);
	if (ret)
		dev_err(pogo_transport->dev, "%s Failed to %s EXTCON_DOCK\n", __func__,
			docked ? "set" : "clear");
}

static void disable_and_bypass_hub(struct pogo_transport *pogo_transport)
{
	int ret;

	if (!pogo_transport->hub_embedded)
		return;

	/* USB_MUX_HUB_SEL set to 0 to bypass the hub */
	gpio_set_value(pogo_transport->pogo_hub_sel_gpio, 0);
	logbuffer_log(pogo_transport->log, "POGO: hub-mux:%d",
		      gpio_get_value(pogo_transport->pogo_hub_sel_gpio));
	pogo_transport->pogo_hub_active = false;

	if (pogo_transport->hub_ldo && regulator_is_enabled(pogo_transport->hub_ldo) > 0) {
		ret = regulator_disable(pogo_transport->hub_ldo);
		if (ret)
			logbuffer_log(pogo_transport->log, "Failed to disable hub_ldo %d", ret);
	}
}

static void switch_to_usbc_locked(struct pogo_transport *pogo_transport)
{
	struct max77759_plat *chip = pogo_transport->chip;
	int ret;

	if (pogo_transport->pogo_usb_active) {
		ret = extcon_set_state_sync(chip->extcon, EXTCON_USB_HOST, 0);
		logbuffer_log(pogo_transport->log, "%s: %s turning off host for Pogo", __func__,
			      ret < 0 ? "Failed" : "Succeeded");
		pogo_transport->pogo_usb_active = false;
	}

	disable_and_bypass_hub(pogo_transport);

	ret = pinctrl_select_state(pogo_transport->pinctrl, pogo_transport->susp_usb_state);
	if (ret)
		dev_err(pogo_transport->dev, "failed to select suspend in usb state ret:%d\n", ret);

	gpio_set_value(pogo_transport->pogo_data_mux_gpio, 0);
	logbuffer_log(pogo_transport->log, "POGO: data-mux:%d",
		      gpio_get_value(pogo_transport->pogo_data_mux_gpio));
	data_alt_path_active(chip, false);
	enable_data_path_locked(chip);
}

static void switch_to_pogo_locked(struct pogo_transport *pogo_transport)
{
	struct max77759_plat *chip = pogo_transport->chip;
	int ret;

	data_alt_path_active(chip, true);
	if (chip->data_active) {
		ret = extcon_set_state_sync(chip->extcon, chip->active_data_role == TYPEC_HOST ?
					    EXTCON_USB_HOST : EXTCON_USB, 0);

		logbuffer_log(pogo_transport->log, "%s turning off %s", ret < 0 ?
			      "Failed" : "Succeeded", chip->active_data_role == TYPEC_HOST ?
			      "Host" : "Device");
		chip->data_active = false;
	}

	disable_and_bypass_hub(pogo_transport);

	ret = pinctrl_select_state(pogo_transport->pinctrl, pogo_transport->susp_pogo_state);
	if (ret)
		dev_err(pogo_transport->dev, "failed to select suspend in pogo state ret:%d\n",
			ret);

	gpio_set_value(pogo_transport->pogo_data_mux_gpio, 1);
	logbuffer_log(pogo_transport->log, "POGO: data-mux:%d",
		      gpio_get_value(pogo_transport->pogo_data_mux_gpio));
	ret = extcon_set_state_sync(chip->extcon, EXTCON_USB_HOST, 1);
	logbuffer_log(pogo_transport->log, "%s: %s turning on host for Pogo", __func__, ret < 0 ?
		      "Failed" : "Succeeded");
	pogo_transport->pogo_usb_active = true;
}

static void switch_to_hub_locked(struct pogo_transport *pogo_transport)
{
	struct max77759_plat *chip = pogo_transport->chip;
	int ret;

	/*
	 * TODO: set alt_path_active; re-design this function for
	 * 1. usb-c only (hub disabled)
	 * 2. pogo only (hub disabled)
	 * 3. hub enabled for both usb-c host and pogo host
	 */
	data_alt_path_active(chip, true);

	/* if usb-c is active, disable it */
	if (chip->data_active) {
		ret = extcon_set_state_sync(chip->extcon, chip->active_data_role == TYPEC_HOST ?
					    EXTCON_USB_HOST : EXTCON_USB, 0);

		logbuffer_log(pogo_transport->log, "%s turning off %s", ret < 0 ?
			      "Failed" : "Succeeded", chip->active_data_role == TYPEC_HOST ?
			      "Host" : "Device");
		chip->data_active = false;
	}

	/* if pogo-usb is active, disable it */
	if (pogo_transport->pogo_usb_active) {
		ret = extcon_set_state_sync(chip->extcon, EXTCON_USB_HOST, 0);
		logbuffer_log(pogo_transport->log, "%s: %s turning off host for Pogo", __func__,
			      ret < 0 ? "Failed" : "Succeeded");
		pogo_transport->pogo_usb_active = false;
	}

	if (pogo_transport->hub_ldo) {
		ret = regulator_enable(pogo_transport->hub_ldo);
		if (ret)
			logbuffer_log(pogo_transport->log, "%s: Failed to enable hub_ldo %d",
				      __func__, ret);
	}

	ret = pinctrl_select_state(pogo_transport->pinctrl, pogo_transport->hub_state);
	if (ret)
		dev_err(pogo_transport->dev, "failed to select hub state ret:%d\n", ret);

	/* USB_MUX_POGO_SEL set to 0 to direct usb-c to AP or hub */
	gpio_set_value(pogo_transport->pogo_data_mux_gpio, 0);

	/* USB_MUX_HUB_SEL set to 1 to switch the path to hub */
	gpio_set_value(pogo_transport->pogo_hub_sel_gpio, 1);
	logbuffer_log(pogo_transport->log, "POGO: data-mux:%d hub-mux:%d",
		      gpio_get_value(pogo_transport->pogo_data_mux_gpio),
		      gpio_get_value(pogo_transport->pogo_hub_sel_gpio));

	/* wait for the host mode to be turned off completely */
	mdelay(60);

	ret = extcon_set_state_sync(chip->extcon, EXTCON_USB_HOST, 1);
	logbuffer_log(pogo_transport->log, "%s: %s turning on host for hub", __func__, ret < 0 ?
		      "Failed" : "Succeeded");

	/* TODO: re-design the flags */
	pogo_transport->pogo_usb_active = true;
	pogo_transport->pogo_hub_active = true;
}

static void update_pogo_transport(struct kthread_work *work)
{
	struct pogo_event *event =
		container_of(container_of(work, struct kthread_delayed_work, work),
			     struct pogo_event, work);
	struct pogo_transport *pogo_transport = event->pogo_transport;
	struct max77759_plat *chip = pogo_transport->chip;
	int ret;
	union power_supply_propval voltage_now = {0};
	bool docked = !gpio_get_value(pogo_transport->pogo_gpio);
	bool acc_detected = gpio_get_value(pogo_transport->pogo_acc_gpio);

	ret = power_supply_get_property(pogo_transport->pogo_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW,
					&voltage_now);
	if (ret) {
		dev_err(pogo_transport->dev, "%s voltage now read err: %d\n", __func__, ret);
		if (ret == -EAGAIN)
			pogo_transport_event(pogo_transport, EVENT_RETRY_READ_VOLTAGE,
					     POGO_PSY_NRDY_RETRY_MS);
		goto free;
	}

	if (event->event_type == EVENT_DOCKING || event->event_type == EVENT_RETRY_READ_VOLTAGE) {
		if (docked) {
			if (voltage_now.intval >= POGO_USB_CAPABLE_THRESHOLD_UV) {
				pogo_transport->pogo_usb_capable = true;
				update_extcon_dev(pogo_transport, true, true);
			} else {
				/* retry every 50ms * 10 times */
				if (pogo_transport->retry_count < POGO_USB_RETRY_COUNT) {
					pogo_transport->retry_count++;
					pogo_transport_event(pogo_transport,
							     EVENT_RETRY_READ_VOLTAGE,
							     POGO_USB_RETRY_INTEREVAL_MS);
				} else {
					pogo_transport->pogo_usb_capable = false;
					update_extcon_dev(pogo_transport, true, false);
				}
				goto free;
			}
		} else {
			/* Clear retry count when un-docked */
			pogo_transport->retry_count = 0;
			pogo_transport->pogo_usb_capable = false;
			update_extcon_dev(pogo_transport, false, false);
		}
	}

	mutex_lock(&chip->data_path_lock);

	/* Special case for force_usb: ignore everything */
	if (modparam_force_usb)
		goto exit;

	/*
	 * Special case for force_pogo: switch to pogo if available; switch to usbc when undocking.
	 */
	if (pogo_transport->force_pogo) {
		if (pogo_transport->pogo_usb_capable && !pogo_transport->pogo_usb_active)
			switch_to_pogo_locked(pogo_transport);
		else if (!pogo_transport->pogo_usb_capable && pogo_transport->pogo_usb_active)
			switch_to_usbc_locked(pogo_transport);
		goto exit;
	}

	switch (event->event_type) {
	case EVENT_DOCKING:
	case EVENT_RETRY_READ_VOLTAGE:
		if (pogo_transport->pogo_usb_capable && !pogo_transport->pogo_usb_active) {
			/*
			 * Pogo treated with same priority as USB-C, hence skip enabling
			 * pogo usb as USB-C is active.
			 */
			if (chip->data_active && pogo_transport->equal_priority) {
				dev_info(pogo_transport->dev,
					 "usb active, skipping enable pogo usb\n");
				goto exit;
			}
			switch_to_pogo_locked(pogo_transport);
		} else if (!pogo_transport->pogo_usb_capable && pogo_transport->pogo_usb_active) {
			switch_to_usbc_locked(pogo_transport);
		}
		break;
	case EVENT_MOVE_DATA_TO_USB:
		if (pogo_transport->pogo_usb_active)
			switch_to_usbc_locked(pogo_transport);
		break;
	case EVENT_MOVE_DATA_TO_POGO:
		/* Currently this event is bundled to force_pogo. This case is unreachable. */
		break;
	case EVENT_ENABLE_HUB:
		pogo_transport->pogo_usb_capable = true;
		switch_to_hub_locked(pogo_transport);
		break;
	case EVENT_DISABLE_HUB:
		if (pogo_transport->pogo_usb_capable)
			switch_to_pogo_locked(pogo_transport);
		else
			switch_to_usbc_locked(pogo_transport);
		break;
	case EVENT_HALL_SENSOR_ACC_DETECTED:
		/* Disable OVP to prevent the voltage going through POGO_VIN */
		if (pogo_transport->pogo_ovp_en_gpio >= 0)
			gpio_set_value_cansleep(pogo_transport->pogo_ovp_en_gpio,
						!pogo_transport->pogo_ovp_en_active_state);

		if (pogo_transport->acc_detect_ldo) {
			ret = regulator_enable(pogo_transport->acc_detect_ldo);
			if (ret)
				logbuffer_log(pogo_transport->log, "%s: Failed to enable acc_detect %d",
					      __func__, ret);
		}
		break;
	case EVENT_HALL_SENSOR_ACC_UNDOCKED:
		ret = GPSY_SET_PROP(pogo_transport->pogo_psy, GBMS_PROP_POGO_VOUT_ENABLED, 0);
		if (ret)
			logbuffer_log(pogo_transport->log, "%s: Failed to disable pogo_vout %d\n",
				      __func__, ret);

		if (pogo_transport->acc_detect_ldo &&
		    regulator_is_enabled(pogo_transport->acc_detect_ldo)) {
			ret = regulator_disable(pogo_transport->acc_detect_ldo);
			if (ret)
				logbuffer_log(pogo_transport->log, "%s: Failed to disable acc_detect %d",
					      __func__, ret);
		}

		if (!pogo_transport->pogo_irq_enabled) {
			enable_irq(pogo_transport->pogo_irq);
			pogo_transport->pogo_irq_enabled = true;
		}

		switch_to_usbc_locked(pogo_transport);
		pogo_transport->pogo_usb_capable = false;
		break;
	case EVENT_POGO_ACC_DETECTED:
		logbuffer_log(pogo_transport->log, "%s: acc detect debounce %s", __func__,
			      acc_detected ? "success, enabling pogo_vout" : "fail");
		if (!acc_detected) {
			pogo_transport_event(pogo_transport, EVENT_HALL_SENSOR_ACC_UNDOCKED, 0);
			break;
		}

		ret = GPSY_SET_PROP(pogo_transport->pogo_psy, GBMS_PROP_POGO_VOUT_ENABLED, 1);
		if (ret)
			logbuffer_log(pogo_transport->log, "%s: Failed to enable pogo_vout %d\n",
				      __func__, ret);
		break;
	case EVENT_POGO_ACC_CONNECTED:
		/*
		 * Enable pogo only if the acc regulator was enabled. If the regulator has been
		 * disabled, it means EVENT_HALL_SENSOR_ACC_UNDOCKED was triggered before this
		 * event.
		 */
		if (pogo_transport->acc_detect_ldo &&
		    regulator_is_enabled(pogo_transport->acc_detect_ldo)) {
			ret = regulator_disable(pogo_transport->acc_detect_ldo);
			if (ret)
				logbuffer_log(pogo_transport->log, "%s: Failed to disable acc_detect_ldo %d",
					      __func__, ret);

			if (!pogo_transport->mfg_acc_test) {
				switch_to_pogo_locked(pogo_transport);
				pogo_transport->pogo_usb_capable = true;
			}
		}
		break;
	default:
		break;
	}

exit:
	mutex_unlock(&chip->data_path_lock);
	kobject_uevent(&pogo_transport->dev->kobj, KOBJ_CHANGE);
free:
	logbuffer_logk(pogo_transport->log, LOGLEVEL_INFO,
		       "evt:%d docked:%d force_u:%d force_p:%d pogo_usb:%d pogo_usb_active:%d data_active:%d vol_now:%d retry:%u",
		       event->event_type,
		       docked ? 1 : 0,
		       modparam_force_usb ? 1 : 0,
		       pogo_transport->force_pogo ? 1 : 0,
		       pogo_transport->pogo_usb_capable ? 1 : 0,
		       pogo_transport->pogo_usb_active ? 1 : 0,
		       chip->data_active ? 1 : 0,
		       voltage_now.intval,
		       pogo_transport->retry_count);

	devm_kfree(pogo_transport->dev, event);
}

static void pogo_transport_event(struct pogo_transport *pogo_transport,
				 enum pogo_event_type event_type, int delay_ms)
{
	struct pogo_event *evt;

	evt = devm_kzalloc(pogo_transport->dev, sizeof(*evt), GFP_KERNEL);
	if (!evt) {
		logbuffer_log(pogo_transport->log, "POGO: Dropping event");
		return;
	}
	kthread_init_delayed_work(&evt->work, update_pogo_transport);
	evt->pogo_transport = pogo_transport;
	evt->event_type = event_type;
	kthread_mod_delayed_work(pogo_transport->wq, &evt->work, msecs_to_jiffies(delay_ms));
}

static irqreturn_t pogo_acc_irq(int irq, void *dev_id)
{
	struct pogo_transport *pogo_transport = dev_id;
	int pogo_acc_gpio = gpio_get_value(pogo_transport->pogo_acc_gpio);

	logbuffer_log(pogo_transport->log, "Pogo acc threaded irq running, acc_detect %u",
		      pogo_acc_gpio);

	if (pogo_acc_gpio)
		pogo_transport_event(pogo_transport, EVENT_POGO_ACC_DETECTED,
				     pogo_transport->pogo_acc_gpio_debounce_ms);
	return IRQ_HANDLED;
}

static irqreturn_t pogo_acc_isr(int irq, void *dev_id)
{
	struct pogo_transport *pogo_transport = dev_id;

	logbuffer_log(pogo_transport->log, "POGO ACC IRQ triggered ");
	/* FIXME: timeout value? */
	pm_wakeup_event(pogo_transport->dev, POGO_TIMEOUT_MS);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t pogo_irq(int irq, void *dev_id)
{
	struct pogo_transport *pogo_transport = dev_id;
	int pogo_gpio = gpio_get_value(pogo_transport->pogo_gpio);

	logbuffer_log(pogo_transport->log, "Pogo threaded irq running, pogo_gpio %u", pogo_gpio);

	if (pogo_transport->acc_detect_ldo &&
	    regulator_is_enabled(pogo_transport->acc_detect_ldo) > 0) {
		/* disable the irq to prevent the interrupt storm after pogo 5v out */
		disable_irq_nosync(pogo_transport->pogo_irq);
		pogo_transport->pogo_irq_enabled = false;
		pogo_transport_event(pogo_transport, EVENT_POGO_ACC_CONNECTED, 0);
		return IRQ_HANDLED;
	}

	if (pogo_transport->pogo_ovp_en_gpio >= 0) {
		/*
		 * set to active state if pogo_gpio (ACTIVE_LOW) is in active state (0)
		 * set to disable state if pogo_gpio (ACTIVE_LOW) is in disable state (1)
		 */
		gpio_set_value_cansleep(pogo_transport->pogo_ovp_en_gpio, pogo_gpio ?
					!pogo_transport->pogo_ovp_en_active_state :
					pogo_transport->pogo_ovp_en_active_state);
	}

	/*
	 * Signal pogo status change event.
	 * Debounce on docking to differentiate between different docks by
	 * reading power supply voltage.
	 */
	pogo_transport_event(pogo_transport, EVENT_DOCKING, !pogo_gpio ? POGO_PSY_DEBOUNCE_MS : 0);
	return IRQ_HANDLED;
}

static void data_active_changed(void *data)
{
	struct pogo_transport *pogo_transport = data;

	logbuffer_log(pogo_transport->log, "data active changed");
	pogo_transport_event(pogo_transport, EVENT_DATA_ACTIVE_CHANGED, 0);
}

static irqreturn_t pogo_isr(int irq, void *dev_id)
{
	struct pogo_transport *pogo_transport = dev_id;

	logbuffer_log(pogo_transport->log, "POGO IRQ triggered ");
	pm_wakeup_event(pogo_transport->dev, POGO_TIMEOUT_MS);

	return IRQ_WAKE_THREAD;
}

static int init_regulator(struct pogo_transport *pogo_transport)
{
	if (of_property_read_bool(pogo_transport->dev->of_node, "usb-hub-supply")) {
		pogo_transport->hub_ldo = devm_regulator_get(pogo_transport->dev, "usb-hub");
		if (IS_ERR(pogo_transport->hub_ldo)) {
			dev_err(pogo_transport->dev, "Failed to get usb-hub, ret:%ld\n",
				PTR_ERR(pogo_transport->hub_ldo));
			return PTR_ERR(pogo_transport->hub_ldo);
		}
	}

	if (of_property_read_bool(pogo_transport->dev->of_node, "acc-detect-supply")) {
		pogo_transport->acc_detect_ldo = devm_regulator_get(pogo_transport->dev,
								    "acc-detect");
		if (IS_ERR(pogo_transport->acc_detect_ldo)) {
			dev_err(pogo_transport->dev, "Failed to get acc-detect, ret:%ld\n",
				PTR_ERR(pogo_transport->acc_detect_ldo));
			return PTR_ERR(pogo_transport->acc_detect_ldo);
		}
	}

	return 0;
}

static int init_acc_gpio(struct pogo_transport *pogo_transport)
{
	int ret;

	pogo_transport->pogo_acc_gpio = of_get_named_gpio(pogo_transport->dev->of_node,
							  "pogo-acc-detect", 0);
	if (pogo_transport->pogo_acc_gpio < 0) {
		dev_err(pogo_transport->dev, "pogo acc detect gpio not found ret:%d\n",
			pogo_transport->pogo_acc_gpio);
		return pogo_transport->pogo_acc_gpio;
	}

	ret = devm_gpio_request(pogo_transport->dev, pogo_transport->pogo_acc_gpio,
				"pogo-acc-detect");
	if (ret) {
		dev_err(pogo_transport->dev, "failed to request pogo-acc-detect gpio, ret:%d\n",
			ret);
		return ret;
	}

	ret = gpio_direction_input(pogo_transport->pogo_acc_gpio);
	if (ret) {
		dev_err(pogo_transport->dev, "failed to set pogo-acc-detect as input, ret:%d\n",
			ret);
		return ret;
	}

	ret = gpio_set_debounce(pogo_transport->pogo_acc_gpio, POGO_ACC_GPIO_DEBOUNCE_MS * 1000);
	if (ret < 0) {
		dev_info(pogo_transport->dev, "failed to set debounce, ret:%d\n", ret);
		pogo_transport->pogo_acc_gpio_debounce_ms = POGO_ACC_GPIO_DEBOUNCE_MS;
	}

	pogo_transport->pogo_acc_irq = gpio_to_irq(pogo_transport->pogo_acc_gpio);
	if (pogo_transport->pogo_acc_irq <= 0) {
		dev_err(pogo_transport->dev, "Pogo acc irq not found\n");
		return -ENODEV;
	}

	ret = devm_request_threaded_irq(pogo_transport->dev, pogo_transport->pogo_acc_irq,
					pogo_acc_isr, pogo_acc_irq,
					(IRQF_SHARED | IRQF_ONESHOT |
					 IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING),
					dev_name(pogo_transport->dev), pogo_transport);
	if (ret < 0) {
		dev_err(pogo_transport->dev, "pogo-acc-detect request irq failed ret:%d\n", ret);
		return ret;
	}

	ret = enable_irq_wake(pogo_transport->pogo_acc_irq);
	if (ret) {
		dev_err(pogo_transport->dev, "Enable acc irq wake failed ret:%d\n", ret);
		devm_free_irq(pogo_transport->dev, pogo_transport->pogo_acc_irq, pogo_transport);
	}

	return ret;
}

static int init_hub_gpio(struct pogo_transport *pogo_transport)
{
	pogo_transport->pogo_hub_sel_gpio = of_get_named_gpio(pogo_transport->dev->of_node,
							      "pogo-hub-sel", 0);
	if (pogo_transport->pogo_hub_sel_gpio < 0) {
		dev_err(pogo_transport->dev, "Pogo hub sel gpio not found ret:%d\n",
			pogo_transport->pogo_hub_sel_gpio);
		return pogo_transport->pogo_hub_sel_gpio;
	}

	pogo_transport->pogo_hub_reset_gpio = of_get_named_gpio(pogo_transport->dev->of_node,
								"pogo-hub-reset", 0);
	if (pogo_transport->pogo_hub_reset_gpio < 0) {
		dev_err(pogo_transport->dev, "Pogo hub reset gpio not found ret:%d\n",
			pogo_transport->pogo_hub_reset_gpio);
		return pogo_transport->pogo_hub_reset_gpio;
	}

	pogo_transport->hub_state = pinctrl_lookup_state(pogo_transport->pinctrl, "hub");
	if (IS_ERR(pogo_transport->hub_state)) {
		dev_err(pogo_transport->dev, "failed to find pinctrl hub ret:%ld\n",
			PTR_ERR(pogo_transport->hub_state));
		return PTR_ERR(pogo_transport->hub_state);
	}

	return 0;
}

static int init_pogo_gpio(struct pogo_transport *pogo_transport)
{
	enum of_gpio_flags flags;
	int ret;

	pogo_transport->pogo_gpio = of_get_named_gpio(pogo_transport->dev->of_node,
						      "pogo-transport-status", 0);
	if (pogo_transport->pogo_gpio < 0) {
		dev_err(pogo_transport->dev, "Pogo status gpio not found ret:%d\n",
			pogo_transport->pogo_gpio);
		return pogo_transport->pogo_gpio;
	}

	ret = devm_gpio_request(pogo_transport->dev, pogo_transport->pogo_gpio,
				"pogo-transport-status");
	if (ret) {
		dev_err(pogo_transport->dev,
			"failed to request pogo-transport-status gpio, ret:%d\n",
			ret);
		return ret;
	}

	ret = gpio_direction_input(pogo_transport->pogo_gpio);
	if (ret) {
		dev_err(pogo_transport->dev,
			"failed set pogo-transport-status as input, ret:%d\n",
			ret);
		return ret;
	}

	pogo_transport->pogo_irq = gpio_to_irq(pogo_transport->pogo_gpio);
	if (pogo_transport->pogo_irq <= 0) {
		dev_err(pogo_transport->dev, "Pogo irq not found\n");
		return -ENODEV;
	}

	ret = devm_request_threaded_irq(pogo_transport->dev, pogo_transport->pogo_irq, pogo_isr,
					pogo_irq, (IRQF_SHARED | IRQF_ONESHOT |
						   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING),
					dev_name(pogo_transport->dev), pogo_transport);
	if (ret < 0) {
		dev_err(pogo_transport->dev, "pogo-transport-status request irq failed ret:%d\n",
			ret);
		return ret;
	}

	pogo_transport->pogo_irq_enabled = true;

	ret = enable_irq_wake(pogo_transport->pogo_irq);
	if (ret) {
		dev_err(pogo_transport->dev, "Enable irq wake failed ret:%d\n", ret);
		goto free_irq;
	}

	pogo_transport->pinctrl = devm_pinctrl_get_select(pogo_transport->dev, "suspend-to-usb");
	if (IS_ERR(pogo_transport->pinctrl)) {
		dev_err(pogo_transport->dev, "failed to allocate pinctrl ret:%ld\n",
			PTR_ERR(pogo_transport->pinctrl));
		return PTR_ERR(pogo_transport->pinctrl);
	}

	pogo_transport->susp_usb_state = pinctrl_lookup_state(pogo_transport->pinctrl,
							      "suspend-to-usb");
	if (IS_ERR(pogo_transport->susp_usb_state)) {
		dev_err(pogo_transport->dev, "failed to find pinctrl suspend-to-usb ret:%ld\n",
			PTR_ERR(pogo_transport->susp_usb_state));
		return PTR_ERR(pogo_transport->susp_usb_state);
	}

	pogo_transport->susp_pogo_state = pinctrl_lookup_state(pogo_transport->pinctrl,
							       "suspend-to-pogo");
	if (IS_ERR(pogo_transport->susp_pogo_state)) {
		dev_err(pogo_transport->dev, "failed to find pinctrl suspend-to-pogo ret:%ld\n",
			PTR_ERR(pogo_transport->susp_pogo_state));
		return PTR_ERR(pogo_transport->susp_pogo_state);
	}

	pogo_transport->pogo_data_mux_gpio = of_get_named_gpio(pogo_transport->dev->of_node,
							       "pogo-transport-sel", 0);
	if (pogo_transport->pogo_data_mux_gpio < 0) {
		dev_err(pogo_transport->dev, "Pogo sel gpio not found ret:%d\n",
			pogo_transport->pogo_data_mux_gpio);
		ret = pogo_transport->pogo_data_mux_gpio;
		goto disable_irq;
	}

	ret = devm_gpio_request(pogo_transport->dev, pogo_transport->pogo_data_mux_gpio,
				"pogo-transport-sel");
	if (ret) {
		dev_err(pogo_transport->dev, "failed to request pogo-transport-sel gpio, ret:%d\n",
			ret);
		goto disable_irq;
	}

	ret = gpio_direction_output(pogo_transport->pogo_data_mux_gpio, 0);
	if (ret) {
		dev_err(pogo_transport->dev, "failed set pogo-transport-sel as output, ret:%d\n",
			ret);
		goto disable_irq;
	}

	pogo_transport->equal_priority = of_property_read_bool(pogo_transport->dev->of_node,
							       "equal-priority");


	if (!of_property_read_bool(pogo_transport->dev->of_node, "pogo-ovp-en")) {
		pogo_transport->pogo_ovp_en_gpio = -EINVAL;
		goto exit;
	}

	pogo_transport->pogo_ovp_en_gpio = of_get_named_gpio_flags(pogo_transport->dev->of_node,
								   "pogo-ovp-en", 0, &flags);
	if (pogo_transport->pogo_ovp_en_gpio < 0) {
		dev_err(pogo_transport->dev,
			"Pogo ovp en gpio not found. ret:%d\n",
			pogo_transport->pogo_ovp_en_gpio);
		ret = pogo_transport->pogo_ovp_en_gpio;
		goto disable_irq;
	}

	pogo_transport->pogo_ovp_en_active_state = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;

	ret = devm_gpio_request(pogo_transport->dev, pogo_transport->pogo_ovp_en_gpio,
				"pogo-ovp-en");
	if (ret) {
		dev_err(pogo_transport->dev, "failed to request pogo-ovp-en gpio, ret:%d\n",
			ret);
		goto disable_irq;
	}

	/* Default disable pogo ovp. Set to disable state for pogo_ovp_en */
	ret = gpio_direction_output(pogo_transport->pogo_ovp_en_gpio,
				    !pogo_transport->pogo_ovp_en_active_state);
	if (ret) {
		dev_err(pogo_transport->dev, "failed set pogo-ovp-en as output, ret:%d\n",
			ret);
		goto disable_irq;
	}
exit:
	return 0;

disable_irq:
	disable_irq_wake(pogo_transport->pogo_irq);
free_irq:
	devm_free_irq(pogo_transport->dev, pogo_transport->pogo_irq, pogo_transport);

	return ret;
}

static int pogo_transport_probe(struct platform_device *pdev)
{
	struct pogo_transport *pogo_transport;
	int ret = 0;
	struct device_node *data_np, *dn;
	struct i2c_client *data_client;
	struct max77759_plat *chip;
	char *pogo_psy_name;

	data_np = of_parse_phandle(pdev->dev.of_node, "data-phandle", 0);
	if (!data_np) {
		dev_err(&pdev->dev, "Failed to find tcpci node\n");
		return -ENODEV;
	}

	data_client = of_find_i2c_device_by_node(data_np);
	if (!data_client) {
		dev_err(&pdev->dev, "Failed to find tcpci client\n");
		ret = -EPROBE_DEFER;
		goto free_np;
	}

	chip = i2c_get_clientdata(data_client);
	if (!chip) {
		dev_err(&pdev->dev, "Failed to find max77759_plat\n");
		ret = -EPROBE_DEFER;
		goto put_client;
	}

	pogo_transport = devm_kzalloc(&pdev->dev, sizeof(*pogo_transport), GFP_KERNEL);
	if (!pogo_transport) {
		ret = -ENOMEM;
		goto put_client;
	}

	pogo_transport->dev = &pdev->dev;
	pogo_transport->chip = chip;

	pogo_transport->log = logbuffer_register("pogo_transport");
	if (IS_ERR_OR_NULL(pogo_transport->log)) {
		dev_err(pogo_transport->dev, "logbuffer get failed\n");
		ret = -EPROBE_DEFER;
		goto put_client;
	}
	platform_set_drvdata(pdev, pogo_transport);

	pogo_transport->wq = kthread_create_worker(0, "wq-pogo-transport");
	if (IS_ERR_OR_NULL(pogo_transport->wq)) {
		ret = PTR_ERR(pogo_transport->wq);
		goto unreg_logbuffer;
	}

	dn = dev_of_node(pogo_transport->dev);
	if (!dn) {
		dev_err(pogo_transport->dev, "of node not found\n");
		ret = -EINVAL;
		goto destroy_worker;
	}

	ret = init_regulator(pogo_transport);
	if (ret)
		goto destroy_worker;

	pogo_psy_name = (char *)of_get_property(dn, "pogo-psy-name", NULL);
	if (!pogo_psy_name) {
		dev_err(pogo_transport->dev, "pogo-psy-name not set\n");
		ret = -EINVAL;
		goto destroy_worker;
	}

	pogo_transport->pogo_psy = power_supply_get_by_name(pogo_psy_name);
	if (IS_ERR_OR_NULL(pogo_transport->pogo_psy)) {
		dev_err(pogo_transport->dev, "pogo psy not up\n");
		ret = -EPROBE_DEFER;
		goto destroy_worker;
	}

	pogo_transport->extcon = devm_extcon_dev_allocate(pogo_transport->dev, pogo_extcon_cable);
	if (IS_ERR(pogo_transport->extcon)) {
		dev_err(pogo_transport->dev, "error allocating extcon: %ld\n",
			PTR_ERR(pogo_transport->extcon));
		ret = PTR_ERR(pogo_transport->extcon);
		goto psy_put;
	}

	ret = devm_extcon_dev_register(pogo_transport->dev, pogo_transport->extcon);
	if (ret < 0) {
		dev_err(chip->dev, "failed to register extcon device:%d\n", ret);
		goto psy_put;
	}

	ret = init_pogo_gpio(pogo_transport);
	if (ret) {
		dev_err(pogo_transport->dev, "init_pogo_gpio error:%d\n", ret);
		goto psy_put;
	}

	pogo_transport->hub_embedded = of_property_read_bool(dn, "hub-embedded");
	if (pogo_transport->hub_embedded) {
		ret = init_hub_gpio(pogo_transport);
		if (ret)
			goto free_pogo_irq;
	}

	if (of_property_read_bool(dn, "pogo-acc-capable")) {
		ret = init_acc_gpio(pogo_transport);
		if (ret)
			goto free_pogo_irq;
	}

	register_data_active_callback(data_active_changed, pogo_transport);
	dev_info(&pdev->dev, "force usb:%d\n", modparam_force_usb ? 1 : 0);
	put_device(&data_client->dev);
	of_node_put(data_np);
	return 0;

free_pogo_irq:
	disable_irq_wake(pogo_transport->pogo_irq);
	devm_free_irq(pogo_transport->dev, pogo_transport->pogo_irq, pogo_transport);
psy_put:
	power_supply_put(pogo_transport->pogo_psy);
destroy_worker:
	kthread_destroy_worker(pogo_transport->wq);
unreg_logbuffer:
	logbuffer_unregister(pogo_transport->log);
put_client:
	put_device(&data_client->dev);
free_np:
	of_node_put(data_np);
	return ret;
}

static int pogo_transport_remove(struct platform_device *pdev)
{
	struct pogo_transport *pogo_transport = platform_get_drvdata(pdev);

	if (pogo_transport->hub_ldo && regulator_is_enabled(pogo_transport->hub_ldo) > 0)
		regulator_disable(pogo_transport->hub_ldo);

	if (pogo_transport->acc_detect_ldo &&
	    regulator_is_enabled(pogo_transport->acc_detect_ldo) > 0)
		regulator_disable(pogo_transport->acc_detect_ldo);

	if (pogo_transport->pogo_acc_irq > 0) {
		disable_irq_wake(pogo_transport->pogo_acc_irq);
		devm_free_irq(pogo_transport->dev, pogo_transport->pogo_acc_irq, pogo_transport);
	}
	disable_irq_wake(pogo_transport->pogo_irq);
	devm_free_irq(pogo_transport->dev, pogo_transport->pogo_irq, pogo_transport);
	power_supply_put(pogo_transport->pogo_psy);
	kthread_destroy_worker(pogo_transport->wq);
	logbuffer_unregister(pogo_transport->log);

	return 0;
}

#define POGO_TRANSPORT_RO_ATTR(_name)                                                           \
static ssize_t _name##_show(struct device *dev, struct device_attribute *attr, char *buf)       \
{                                                                                               \
	struct pogo_transport *pogo_transport  = dev_get_drvdata(dev);                          \
	return sysfs_emit(buf, "%d\n", pogo_transport->_name);                                  \
}                                                                                               \
static DEVICE_ATTR_RO(_name)
POGO_TRANSPORT_RO_ATTR(equal_priority);
POGO_TRANSPORT_RO_ATTR(pogo_usb_active);

static ssize_t move_data_to_usb_store(struct device *dev, struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct pogo_transport *pogo_transport = dev_get_drvdata(dev);
	u8 enable;

	if (kstrtou8(buf, 0, &enable))
		return -EINVAL;

	if (enable != 1)
		return -EINVAL;

	pogo_transport_event(pogo_transport, EVENT_MOVE_DATA_TO_USB, 0);

	return size;
}
static DEVICE_ATTR_WO(move_data_to_usb);

static ssize_t force_pogo_store(struct device *dev, struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct pogo_transport *pogo_transport = dev_get_drvdata(dev);
	bool force_pogo;

	if (kstrtobool(buf, &force_pogo))
		return -EINVAL;

	pogo_transport->force_pogo = force_pogo;
	if (force_pogo)
		pogo_transport_event(pogo_transport, EVENT_MOVE_DATA_TO_POGO, 0);

	return size;
}

static ssize_t force_pogo_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pogo_transport *pogo_transport  = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%u\n", pogo_transport->force_pogo);
}
static DEVICE_ATTR_RW(force_pogo);

static ssize_t enable_hub_store(struct device *dev, struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct pogo_transport *pogo_transport = dev_get_drvdata(dev);
	bool enable_hub;

	if (!pogo_transport->hub_embedded)
		return size;

	if (kstrtobool(buf, &enable_hub))
		return -EINVAL;

	if (enable_hub == pogo_transport->pogo_hub_active)
		return size;

	logbuffer_log(pogo_transport->log, "%s: %sable hub", __func__, enable_hub ? "en" : "dis");
	if (enable_hub)
		pogo_transport_event(pogo_transport, EVENT_ENABLE_HUB, 0);
	else
		pogo_transport_event(pogo_transport, EVENT_DISABLE_HUB, 0);

	return size;
}

static ssize_t enable_hub_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pogo_transport *pogo_transport  = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%u\n", pogo_transport->pogo_hub_active);
}
static DEVICE_ATTR_RW(enable_hub);

static ssize_t hall1_s_store(struct device *dev, struct device_attribute *attr, const char *buf,
			     size_t size)
{
	struct pogo_transport *pogo_transport = dev_get_drvdata(dev);
	u8 enable_acc_detect;

	if (!pogo_transport->acc_detect_ldo)
		return size;

	if (kstrtou8(buf, 0, &enable_acc_detect))
		return -EINVAL;

	if (pogo_transport->hall1_s_state == !!enable_acc_detect)
		return size;

	pogo_transport->hall1_s_state = !!enable_acc_detect;

	/*
	 * KEEP_USB_PATH is only for factory tests where the USB connection needs to stay at USB-C
	 * after the accessory is attached.
	 */
	if (enable_acc_detect == KEEP_USB_PATH)
		pogo_transport->mfg_acc_test = true;
	else
		pogo_transport->mfg_acc_test = false;

	logbuffer_log(pogo_transport->log, "%s: %sable accessory detection, mfg %u", __func__,
		      enable_acc_detect ? "en" : "dis", pogo_transport->mfg_acc_test);
	if (enable_acc_detect)
		pogo_transport_event(pogo_transport, EVENT_HALL_SENSOR_ACC_DETECTED, 0);
	else
		pogo_transport_event(pogo_transport, EVENT_HALL_SENSOR_ACC_UNDOCKED, 0);

	return size;
}
static DEVICE_ATTR_WO(hall1_s);

static ssize_t hall1_n_store(struct device *dev, struct device_attribute *attr, const char *buf,
			     size_t size)
{
	/* Reserved for HES1 Malfunction detection */
	return size;
}
static DEVICE_ATTR_WO(hall1_n);

static ssize_t hall2_s_store(struct device *dev, struct device_attribute *attr, const char *buf,
			     size_t size)
{
	/* Reserved for keyboard status detection */
	return size;
}
static DEVICE_ATTR_WO(hall2_s);

static ssize_t acc_detect_debounce_ms_store(struct device *dev, struct device_attribute *attr,
					    const char *buf, size_t size)
{
	struct pogo_transport *pogo_transport = dev_get_drvdata(dev);
	unsigned int debounce_ms;
	int ret;

	if (kstrtouint(buf, 0, &debounce_ms))
		return -EINVAL;

	ret = gpio_set_debounce(pogo_transport->pogo_acc_gpio, debounce_ms * 1000);
	if (ret < 0) {
		dev_info(pogo_transport->dev, "failed to set debounce, ret:%d\n", ret);
		pogo_transport->pogo_acc_gpio_debounce_ms = debounce_ms;
	}

	return size;
}

static ssize_t acc_detect_debounce_ms_show(struct device *dev, struct device_attribute *attr,
					   char *buf)
{
	struct pogo_transport *pogo_transport  = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%u\n", pogo_transport->pogo_acc_gpio_debounce_ms);
}
static DEVICE_ATTR_RW(acc_detect_debounce_ms);

static struct attribute *pogo_transport_attrs[] = {
	&dev_attr_move_data_to_usb.attr,
	&dev_attr_equal_priority.attr,
	&dev_attr_pogo_usb_active.attr,
	&dev_attr_force_pogo.attr,
	&dev_attr_enable_hub.attr,
	&dev_attr_hall1_s.attr,
	&dev_attr_hall1_n.attr,
	&dev_attr_hall2_s.attr,
	&dev_attr_acc_detect_debounce_ms.attr,
	NULL,
};
ATTRIBUTE_GROUPS(pogo_transport);

static const struct of_device_id pogo_transport_of_match[] = {
	{.compatible = "pogo-transport"},
	{},
};
MODULE_DEVICE_TABLE(of, pogo_transport_of_match);

static struct platform_driver pogo_transport_driver = {
	.driver = {
		   .name = "pogo-transport",
		   .owner = THIS_MODULE,
		   .of_match_table = pogo_transport_of_match,
		   .dev_groups = pogo_transport_groups,
		   },
	.probe = pogo_transport_probe,
	.remove = pogo_transport_remove,
};

module_platform_driver(pogo_transport_driver);

MODULE_DESCRIPTION("Pogo data management");
MODULE_AUTHOR("Badhri Jagan Sridharan <badhri@google.com>");
MODULE_LICENSE("GPL");
