// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2021, Google LLC
 *
 * Pogo management driver
 */

#include <linux/debugfs.h>
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
#include <linux/spinlock.h>
#include <linux/usb.h>
#include <linux/usb/tcpm.h>
#include <misc/gvotable.h>

#include "../tcpci.h"
#include "google_bms.h"
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
#define KEEP_HUB_PATH 2
#define DEFAULT_STATE_MACHINE_ENABLE false

#define POGO_VOTER "POGO"
#define SSPHY_RESTART_EL "SSPHY_RESTART"

/*
 * State Description:
 *	INVALID_STATE,
 *  (A)	STANDBY,			// Nothing attached, hub disabled
 *	DOCKING_DEBOUNCED,		// STANDBY -> DOCK_HUB, pogo gpio
 *	STANDBY_ACC_DEBOUNCED,		// STANDBY -> ACC_DIRECT, acc gpio
 *  (B)	DOCK_HUB,			// Dock online, hub enabled
 *  (C)	DOCK_DEVICE_HUB,		// Dock online, usb device online, hub enabled
 *  (H)	DOCK_AUDIO_HUB,			// Dock online, usb audio online, hub enabled
 *  (I)	AUDIO_HUB,			// Usb audio online, hub enabled
 *	AUDIO_HUB_DOCKING_DEBOUNCED,	// AUDIO_HUB -> DOCK_AUDIO_HUB, pogo gpio
 *	AUDIO_HUB_ACC_DEBOUNCED,	// AUDIO_HUB -> ACC_AUDIO_HUB, acc gpio
 *  (D)	DEVICE_HUB,			// Usb device online, hub enabled
 *      DEVICE_HUB_DOCKING_DEBOUNCED,   // DEVICE_HUB -> DOCK_DEVICE_HUB, pogo gpio
 *      DEVICE_HUB_ACC_DEBOUNCED,	// DEVICE_HUB -> ACC_DEVICE_HUB, acc gpio
 *  (E)	DEVICE_DIRECT,			// Usb device online, hub disabled
 *	DEVICE_DOCKING_DEBOUNCED,	// DEVICE_DIRECT -> DOCK_DEVICE_HUB, pogo gpio
 *	DEVICE_DIRECT_ACC_DEBOUNCED,	// DEVICE_DIRECT -> ACC_DEVICE_HUB, acc gpio
 *  (F)	AUDIO_DIRECT,			// Usb audio online, hub disabled
 *	AUDIO_DIRECT_DOCKING_DEBOUNCED,	// AUDIO_DIRECT -> AUDIO_DIRECT_DOCK_OFFLINE, pogo gpio
 *	AUDIO_DIRECT_ACC_DEBOUNCED,	// AUDIO_DIRECT -> ACC_DEVICE_HUB, acc gpio
 *  (G)	AUDIO_DIRECT_DOCK_OFFLINE,	// Usb audio online, dock offline, hub disabled
 *  (J)	HOST_DIRECT,			// Usb host online, hub disabled
 *	HOST_DIRECT_DOCKING_DEBOUNCED,	// HOST_DIRECT -> HOST_DIRECT_DOCK_OFFLINE, pogo gpio
 *  (K)	HOST_DIRECT_DOCK_OFFLINE,	// Usb host online, dock offline, hub disabled
 *      HOST_DIRECT_ACC_DEBOUNCED,	// HOST_DIRECT -> HOST_DIRECT_ACC_OFFLINE, acc gpio
 *  (L)	DOCK_HUB_HOST_OFFLINE,		// Dock online, usb host offline, hub enabled
 *  (M)	ACC_DIRECT,			// Acc online, hub disabled
 *  (N)	ACC_DEVICE_HUB,			// Acc online, usb device online, hub enabled
 *  (O)	ACC_HUB,			// Acc online, hub enabled
 *  (P)	ACC_AUDIO_HUB,			// Acc online, usb audio online, hub enabled
 *  (Q)	LID_CLOSE,
 *  (R)	HOST_DIRECT_ACC_OFFLINE,	// Usb host online, acc offline, hub disabled
 *  (S)	ACC_DIRECT_HOST_OFFLINE,	// Acc online, usb host offline
 */

#define FOREACH_STATE(S)			\
	S(INVALID_STATE),			\
	S(STANDBY),				\
	S(DOCKING_DEBOUNCED),			\
	S(STANDBY_ACC_DEBOUNCED),		\
	S(DOCK_HUB),				\
	S(DOCK_DEVICE_HUB),			\
	S(DOCK_AUDIO_HUB),			\
	S(AUDIO_HUB),				\
	S(AUDIO_HUB_DOCKING_DEBOUNCED),		\
	S(AUDIO_HUB_ACC_DEBOUNCED),		\
	S(DEVICE_HUB),				\
	S(DEVICE_HUB_DOCKING_DEBOUNCED),	\
	S(DEVICE_HUB_ACC_DEBOUNCED),		\
	S(DEVICE_DIRECT),			\
	S(DEVICE_DOCKING_DEBOUNCED),		\
	S(DEVICE_DIRECT_ACC_DEBOUNCED),		\
	S(AUDIO_DIRECT),			\
	S(AUDIO_DIRECT_DOCKING_DEBOUNCED),	\
	S(AUDIO_DIRECT_ACC_DEBOUNCED),		\
	S(AUDIO_DIRECT_DOCK_OFFLINE),		\
	S(HOST_DIRECT),				\
	S(HOST_DIRECT_DOCKING_DEBOUNCED),	\
	S(HOST_DIRECT_DOCK_OFFLINE),		\
	S(HOST_DIRECT_ACC_DEBOUNCED),		\
	S(DOCK_HUB_HOST_OFFLINE),		\
	S(ACC_DIRECT),				\
	S(ACC_DEVICE_HUB),			\
	S(ACC_HUB),				\
	S(ACC_AUDIO_HUB),			\
	S(LID_CLOSE),				\
	S(HOST_DIRECT_ACC_OFFLINE),		\
	S(ACC_DIRECT_HOST_OFFLINE)

#define GENERATE_ENUM(e)	e
#define GENERATE_STRING(s)	#s

enum pogo_state {
	FOREACH_STATE(GENERATE_ENUM)
};

static const char * const pogo_states[] = {
	FOREACH_STATE(GENERATE_STRING)
};

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
	EVENT_POGO_ACC_DEBOUNCED,
	EVENT_POGO_ACC_CONNECTED,
	/* Bypass the accessory detection and enable POGO Vout and POGO USB capability */
	/* This event is for debug only and never used in normal operations. */
	EVENT_FORCE_ACC_CONNECT,
	/* Reported when CC orientation has changed */
	EVENT_ORIENTATION_CHANGED,
};

#define EVENT_POGO_IRQ			BIT(0)
#define EVENT_USBC_DATA_CHANGE		BIT(1)
#define EVENT_ENABLE_USB_DATA		BIT(2)
#define EVENT_HES_H1S_CHANGED		BIT(3)
#define EVENT_ACC_GPIO_ACTIVE		BIT(4)
#define EVENT_ACC_CONNECTED		BIT(5)
#define EVENT_AUDIO_DEV_ATTACHED	BIT(6)
#define EVENT_USBC_ORIENTATION		BIT(7)
#define EVENT_LAST_EVENT_TYPE		BIT(63)

static bool modparam_force_usb;
module_param_named(force_usb, modparam_force_usb, bool, 0644);
MODULE_PARM_DESC(force_usb, "Force enabling usb path over pogo");

/* Overrides device tree config */
static int modparam_pogo_accessory_enable;
module_param_named(pogo_accessory_enable, modparam_pogo_accessory_enable, int, 0644);
MODULE_PARM_DESC(pogo_accessory_enable, "Enabling accessory detection over pogo");

/* Set to 1 (enable) or 2 (disable) to override the default value */
static int modparam_state_machine_enable;
module_param_named(state_machine_enable, modparam_state_machine_enable, int, 0644);
MODULE_PARM_DESC(state_machine_enable, "Enabling pogo state machine transition");

struct pogo_event {
	struct kthread_delayed_work work;
	struct pogo_transport *pogo_transport;
	enum pogo_event_type event_type;
};

enum pogo_accessory_detection {
	/* Pogo accessory detection is disabled. */
	DISABLED,
	/*
	 * Pogo accessory detection is only based on HALL output mapped to pogo-acc-hall-capable.
	 * Expected seq:
	 * EVENT_HALL_SENSOR_ACC_DETECTED -> EVENT_HALL_SENSOR_ACC_UNDOCKED
	 */
	HALL_ONLY,
	/*
	 * Pogo accessory detection POR mapped to pogo-acc-capable.
	 * Expected seq:
	 * EVENT_HALL_SENSOR_ACC_DETECTED -> EVENT_POGO_ACC_DEBOUNCED ->
	 * EVENT_POGO_ACC_CONNECTED -> EVENT_HALL_SENSOR_ACC_UNDOCKED
	 */
	ENABLED
};

struct pogo_transport_udev_ids {
	__le16 vendor;
	__le16 product;
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
	/* When true, acc irq is enabled */
	bool acc_irq_enabled;
	/* When true, hall1_s sensor reports attach event */
	bool hall1_s_state;
	/* When true, the path won't switch to pogo if accessory is attached */
	bool mfg_acc_test;
	/* When true, the hub will remain enabled after undocking */
	bool force_hub_enabled;
	/*
	 * When true, skip acc detection and POGO Vout as well as POGO USB will be enabled.
	 * Only applicable for debugfs capable builds.
	 */
	bool mock_hid_connected;

	struct kthread_worker *wq;
	struct kthread_delayed_work state_machine;
	struct kthread_work event_work;
	enum pogo_state prev_state;
	enum pogo_state state;
	enum pogo_state delayed_state;
	unsigned long delayed_runtime;
	unsigned long delay_ms;
	unsigned long event_map;
	bool state_machine_running;
	bool state_machine_enabled;
	spinlock_t pogo_event_lock;

	/* Register the notifier from USB core */
	struct notifier_block udev_nb;
	/* When true, a superspeed (or better) USB device is enumerated */
	bool ss_udev_attached;

	/* To read voltage at the pogo pins */
	struct power_supply *pogo_psy;
	/* Retry when voltage is less than POGO_USB_CAPABLE_THRESHOLD_UV */
	unsigned int retry_count;
	/* To signal userspace extcon observer */
	struct extcon_dev *extcon;
	/* When true, disable voltage based detection of pogo partners */
	bool disable_voltage_detection;
	struct gvotable_election *charger_mode_votable;
	struct gvotable_election *ssphy_restart_votable;

	/* Used for cancellable work such as pogo debouncing */
	struct kthread_delayed_work pogo_accessory_debounce_work;

	/* Pogo accessory detection status */
	enum pogo_accessory_detection accessory_detection_enabled;

	/* Orientation of USB-C, 0:TYPEC_POLARITY_CC1 1:TYPEC_POLARITY_CC2 */
	enum typec_cc_polarity polarity;

	/* Cache values from the Type-C driver */
	enum typec_data_role usbc_data_role;
	bool usbc_data_active;
};

static const unsigned int pogo_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_DOCK,
	EXTCON_NONE,
};

/*
 * list of USB VID:PID pair of udevs which are audio docks with pogo interfaces
 * Both VID and PID are required in each entry.
 */
static const struct pogo_transport_udev_ids audio_dock_ids[] = {
	{
		.vendor = cpu_to_le16(0x18d1),
		.product = cpu_to_le16(0x9480),
	},
	{ },
};

/* Return true if @vid and @pid pair is found in @list. Otherwise, return false. */
static bool pogo_transport_match_udev(const struct pogo_transport_udev_ids *list, const u16 vid,
				      const u16 pid)
{
	if (list) {
		while (list->vendor && list->product) {
			if (list->vendor == cpu_to_le16(vid) && list->product == cpu_to_le16(pid))
				return true;
			list++;
		}
	}
	return false;
}

static void pogo_transport_event(struct pogo_transport *pogo_transport,
				 enum pogo_event_type event_type, int delay_ms);

static void update_extcon_dev(struct pogo_transport *pogo_transport, bool docked, bool usb_capable)
{
	int ret;

	/* While docking, Signal EXTCON_USB before signalling EXTCON_DOCK */
	if (docked) {
		ret = extcon_set_state_sync(pogo_transport->extcon, EXTCON_USB, usb_capable ?
					    1 : 0);
		if (ret)
			dev_err(pogo_transport->dev, "%s Failed to %s EXTCON_USB\n", __func__,
				usb_capable ? "set" : "clear");
		ret = extcon_set_state_sync(pogo_transport->extcon, EXTCON_DOCK, 1);
		if (ret)
			dev_err(pogo_transport->dev, "%s Failed to set EXTCON_DOCK\n", __func__);
		return;
	}

	/* b/241919179: While undocking, Signal EXTCON_DOCK before signalling EXTCON_USB */
	ret = extcon_set_state_sync(pogo_transport->extcon, EXTCON_DOCK, 0);
	if (ret)
		dev_err(pogo_transport->dev, "%s Failed to clear EXTCON_DOCK\n", __func__);
	ret = extcon_set_state_sync(pogo_transport->extcon, EXTCON_USB, 0);
	if (ret)
		dev_err(pogo_transport->dev, "%s Failed to clear EXTCON_USB\n", __func__);
}

static void ssphy_restart_control(struct pogo_transport *pogo_transport, bool enable)
{
	if (!pogo_transport->ssphy_restart_votable)
		pogo_transport->ssphy_restart_votable =
				gvotable_election_get_handle(SSPHY_RESTART_EL);

	if (IS_ERR_OR_NULL(pogo_transport->ssphy_restart_votable)) {
		logbuffer_log(pogo_transport->log, "SSPHY_RESTART get failed %ld\n",
			      PTR_ERR(pogo_transport->ssphy_restart_votable));
		return;
	}

	logbuffer_log(pogo_transport->log, "ssphy_restart_control %u", enable);
	gvotable_cast_long_vote(pogo_transport->ssphy_restart_votable, POGO_VOTER, enable, enable);
}

/*
 * Update the polarity to EXTCON_USB_HOST. If @sync is true, use the sync version to set the
 * property.
 */
static void pogo_transport_update_polarity(struct pogo_transport *pogo_transport, int polarity,
					   bool sync)
{
	union extcon_property_value prop = {.intval = polarity};
	struct max77759_plat *chip = pogo_transport->chip;
	int ret;

	if (sync)
		ret = extcon_set_property_sync(chip->extcon, EXTCON_USB_HOST,
					       EXTCON_PROP_USB_TYPEC_POLARITY,
					       prop);
	else
		ret = extcon_set_property(chip->extcon, EXTCON_USB_HOST,
					  EXTCON_PROP_USB_TYPEC_POLARITY,
					  prop);
	logbuffer_log(pogo_transport->log, "%sset polarity to %d sync %u", ret ? "failed to " : "",
		      prop.intval, sync);
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

	/*
	 * No further action in the callback of the votable if it is disabled. Disable it here for
	 * the bookkeeping purpose in the dumpstate.
	 */
	ssphy_restart_control(pogo_transport, false);

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

	/*
	 * Calling extcon_set_state_sync to turn off the host resets the orientation of USB-C and
	 * the USB phy was also reset to the default value CC1.
	 * Update the orientation for superspeed phy if USB-C is connected and CC2 is active.
	 */
	if (pogo_transport->polarity == TYPEC_POLARITY_CC2)
		pogo_transport_update_polarity(pogo_transport, TYPEC_POLARITY_CC2, false);

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

	/*
	 * The polarity was reset to 0 when Host Mode was disabled for USB-C or POGO. If current
	 * polarity is CC2, update it to ssphy before enabling the Host Mode for hub.
	 */
	if (pogo_transport->polarity == TYPEC_POLARITY_CC2)
		pogo_transport_update_polarity(pogo_transport, pogo_transport->polarity, false);

	ret = extcon_set_state_sync(chip->extcon, EXTCON_USB_HOST, 1);
	logbuffer_log(pogo_transport->log, "%s: %s turning on host for hub", __func__, ret < 0 ?
		      "Failed" : "Succeeded");

	/* TODO: re-design the flags */
	pogo_transport->pogo_usb_active = true;
	pogo_transport->pogo_hub_active = true;
}

static void update_pogo_transport(struct pogo_transport *pogo_transport,
				  enum pogo_event_type event_type)
{
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

	if (event_type == EVENT_DOCKING || event_type == EVENT_RETRY_READ_VOLTAGE) {
		if (docked) {
			if (pogo_transport->disable_voltage_detection ||
			    voltage_now.intval >= POGO_USB_CAPABLE_THRESHOLD_UV) {
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

	if (pogo_transport->mock_hid_connected) {
		switch (event_type) {
		case EVENT_ENABLE_HUB:
		case EVENT_DISABLE_HUB:
		case EVENT_FORCE_ACC_CONNECT:
		case EVENT_HALL_SENSOR_ACC_UNDOCKED:
			break;
		default:
			logbuffer_log(pogo_transport->log, "%s: skipping mock_hid_connected set",
				      __func__);
			goto exit;
		}
	}

	switch (event_type) {
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
			if (pogo_transport->pogo_hub_active && pogo_transport->force_hub_enabled) {
				pogo_transport->pogo_usb_capable = true;
				logbuffer_log(pogo_transport->log, "%s: keep enabling the hub",
					      __func__);
			} else {
				switch_to_usbc_locked(pogo_transport);
			}
		}
		break;
	case EVENT_MOVE_DATA_TO_USB:
		if (pogo_transport->pogo_usb_active)
			switch_to_usbc_locked(pogo_transport);
		break;
	case EVENT_MOVE_DATA_TO_POGO:
		/* Currently this event is bundled to force_pogo. This case is unreachable. */
		break;
	case EVENT_DATA_ACTIVE_CHANGED:
		/* Do nothing if USB-C data becomes active or hub is enabled. */
		if ((chip->data_active && pogo_transport->equal_priority) ||
		    pogo_transport->pogo_hub_active)
			break;

		/* Switch to POGO if POGO path is available. */
		if (pogo_transport->pogo_usb_capable && !pogo_transport->pogo_usb_active)
			switch_to_pogo_locked(pogo_transport);
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

		if (pogo_transport->acc_detect_ldo &&
		    pogo_transport->accessory_detection_enabled == ENABLED) {
			ret = regulator_enable(pogo_transport->acc_detect_ldo);
			if (ret)
				logbuffer_log(pogo_transport->log, "%s: Failed to enable acc_detect %d",
					      __func__, ret);
		} else if (pogo_transport->accessory_detection_enabled == HALL_ONLY) {
			logbuffer_log(pogo_transport->log,
				      "%s: Skip enabling comparator logic, enable vout", __func__);
			if (pogo_transport->pogo_irq_enabled) {
				disable_irq_nosync(pogo_transport->pogo_irq);
				pogo_transport->pogo_irq_enabled = false;
			}
			ret = gvotable_cast_long_vote(pogo_transport->charger_mode_votable,
						      POGO_VOTER, GBMS_POGO_VOUT, 1);
			if (ret)
				logbuffer_log(pogo_transport->log,
					      "%s: Failed to vote VOUT, ret %d", __func__, ret);
			switch_to_pogo_locked(pogo_transport);
			pogo_transport->pogo_usb_capable = true;
		}
		break;
	case EVENT_HALL_SENSOR_ACC_UNDOCKED:
		pogo_transport->mock_hid_connected = 0;
		ret = gvotable_cast_long_vote(pogo_transport->charger_mode_votable, POGO_VOTER,
					      GBMS_POGO_VOUT, 0);
		if (ret)
			logbuffer_log(pogo_transport->log, "%s: Failed to unvote VOUT, ret %d",
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

		if (!pogo_transport->acc_irq_enabled) {
			enable_irq(pogo_transport->pogo_acc_irq);
			pogo_transport->acc_irq_enabled = true;
		}

		if (pogo_transport->pogo_hub_active && pogo_transport->force_hub_enabled) {
			logbuffer_log(pogo_transport->log, "%s: keep enabling the hub", __func__);
		} else {
			switch_to_usbc_locked(pogo_transport);
			pogo_transport->pogo_usb_capable = false;
		}
		break;
	case EVENT_POGO_ACC_DEBOUNCED:
		logbuffer_log(pogo_transport->log, "%s: acc detect debounce %s", __func__,
			      acc_detected ? "success, enabling pogo_vout" : "fail");
		/* Do nothing if debounce fails */
		if (!acc_detected)
			break;

		if (pogo_transport->acc_irq_enabled) {
			disable_irq(pogo_transport->pogo_acc_irq);
			pogo_transport->acc_irq_enabled = false;
		}

		ret = gvotable_cast_long_vote(pogo_transport->charger_mode_votable, POGO_VOTER,
					      GBMS_POGO_VOUT, 1);
		if (ret)
			logbuffer_log(pogo_transport->log, "%s: Failed to vote VOUT, ret %d",
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
		}
		if (pogo_transport->accessory_detection_enabled) {
			if (!pogo_transport->mfg_acc_test) {
				switch_to_pogo_locked(pogo_transport);
				pogo_transport->pogo_usb_capable = true;
			}
		}
		break;
#if IS_ENABLED(CONFIG_DEBUG_FS)
	case EVENT_FORCE_ACC_CONNECT:
		if (pogo_transport->pogo_irq_enabled) {
			disable_irq(pogo_transport->pogo_irq);
			pogo_transport->pogo_irq_enabled = false;
		}

		if (pogo_transport->acc_irq_enabled) {
			disable_irq(pogo_transport->pogo_acc_irq);
			pogo_transport->acc_irq_enabled = false;
		}

		if (pogo_transport->pogo_ovp_en_gpio >= 0)
			gpio_set_value_cansleep(pogo_transport->pogo_ovp_en_gpio,
						!pogo_transport->pogo_ovp_en_active_state);

		/* Disable, just in case when docked, if acc_detect_ldo was on */
		if (pogo_transport->acc_detect_ldo &&
		    regulator_is_enabled(pogo_transport->acc_detect_ldo)) {
			ret = regulator_disable(pogo_transport->acc_detect_ldo);
			if (ret)
				logbuffer_log(pogo_transport->log,
					      "%s: Failed to disable acc_detect %d", __func__, ret);
		}

		ret = gvotable_cast_long_vote(pogo_transport->charger_mode_votable, POGO_VOTER,
					      GBMS_POGO_VOUT, 1);
		if (ret)
			logbuffer_log(pogo_transport->log, "%s: Failed to vote VOUT, ret %d",
				      __func__, ret);

		switch_to_pogo_locked(pogo_transport);
		pogo_transport->pogo_usb_capable = true;
		break;
#endif
	case EVENT_ORIENTATION_CHANGED:
		/* Update the orientation and restart the ssphy if hub is enabled */
		if (pogo_transport->pogo_hub_active) {
			pogo_transport_update_polarity(pogo_transport, pogo_transport->polarity,
						       true);
			ssphy_restart_control(pogo_transport, true);
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
		       "ev:%u dock:%u f_u:%u f_p:%u f_h:%u p_u:%u p_act:%u hub:%u d_act:%u mock:%u v:%d",
		       event_type,
		       docked ? 1 : 0,
		       modparam_force_usb ? 1 : 0,
		       pogo_transport->force_pogo ? 1 : 0,
		       pogo_transport->force_hub_enabled ? 1 : 0,
		       pogo_transport->pogo_usb_capable ? 1 : 0,
		       pogo_transport->pogo_usb_active ? 1 : 0,
		       pogo_transport->pogo_hub_active ? 1 : 0,
		       chip->data_active ? 1 : 0,
		       pogo_transport->mock_hid_connected ? 1 : 0,
		       voltage_now.intval);
}

static void process_generic_event(struct kthread_work *work)
{
	struct pogo_event *event =
		container_of(container_of(work, struct kthread_delayed_work, work),
			     struct pogo_event, work);
	struct pogo_transport *pogo_transport = event->pogo_transport;

	update_pogo_transport(pogo_transport, event->event_type);

	devm_kfree(pogo_transport->dev, event);
}

static void process_debounce_event(struct kthread_work *work)
{
	struct pogo_transport *pogo_transport =
		container_of(container_of(work, struct kthread_delayed_work, work),
			     struct pogo_transport, pogo_accessory_debounce_work);

	update_pogo_transport(pogo_transport, EVENT_POGO_ACC_DEBOUNCED);
}

static void pogo_transport_event(struct pogo_transport *pogo_transport,
				 enum pogo_event_type event_type, int delay_ms)
{
	struct pogo_event *evt;

	if (event_type == EVENT_POGO_ACC_DEBOUNCED) {
		kthread_mod_delayed_work(pogo_transport->wq,
					 &pogo_transport->pogo_accessory_debounce_work,
					 msecs_to_jiffies(delay_ms));
		return;
	}

	evt = devm_kzalloc(pogo_transport->dev, sizeof(*evt), GFP_KERNEL);
	if (!evt) {
		logbuffer_log(pogo_transport->log, "POGO: Dropping event");
		return;
	}
	kthread_init_delayed_work(&evt->work, process_generic_event);
	evt->pogo_transport = pogo_transport;
	evt->event_type = event_type;
	kthread_mod_delayed_work(pogo_transport->wq, &evt->work, msecs_to_jiffies(delay_ms));
}

/*-------------------------------------------------------------------------*/
/* State Machine Functions                                                 */
/*-------------------------------------------------------------------------*/

/*
 * State transition
 *
 * This function is guarded by (max77759_plat)->data_path_lock
 */
static void pogo_transport_set_state(struct pogo_transport *pogo_transport, enum pogo_state state,
				     unsigned int delay_ms)
{
	if (delay_ms) {
		logbuffer_log(pogo_transport->log, "pending state change %s -> %s @ %u ms",
			      pogo_states[pogo_transport->state], pogo_states[state], delay_ms);
		pogo_transport->delayed_state = state;
		kthread_mod_delayed_work(pogo_transport->wq, &pogo_transport->state_machine,
					 msecs_to_jiffies(delay_ms));
		pogo_transport->delayed_runtime = jiffies + msecs_to_jiffies(delay_ms);
		pogo_transport->delay_ms = delay_ms;
	} else {
		logbuffer_logk(pogo_transport->log, LOGLEVEL_INFO, "state change %s -> %s",
			       pogo_states[pogo_transport->state], pogo_states[state]);
		pogo_transport->delayed_state = INVALID_STATE;
		pogo_transport->prev_state = pogo_transport->state;
		pogo_transport->state = state;

		if (!pogo_transport->state_machine_running)
			kthread_mod_delayed_work(pogo_transport->wq, &pogo_transport->state_machine,
						 0);
	}
}

/*
 * Accessory Detection regulator control
 *  - Return -ENXIO if Accessory Detection regulator does not exist
 *  - Return 0 if @enable is the same as the status of the regulator
 *  - Otherwise, return the return value from regulator_enable or regulator_disable
 */
static int pogo_transport_acc_regulator(struct pogo_transport *pogo_transport, bool enable)
{
	int ret;

	if (!pogo_transport->acc_detect_ldo)
		return -ENXIO;

	if (regulator_is_enabled(pogo_transport->acc_detect_ldo) == enable)
		return 0;

	if (enable)
		ret = regulator_enable(pogo_transport->acc_detect_ldo);
	else
		ret = regulator_disable(pogo_transport->acc_detect_ldo);

	return ret;
}

/*
 * Call this function to:
 *  - Disable POGO Vout by voting 0 to charger_mode_votable
 *  - Disable the regulator for Accessory Detection Logic
 *  - Disable Accessory Detection IRQ
 *  - Enable POGO Voltage Detection IRQ
 *
 *  This function is guarded by (max77759_plat)->data_path_lock
 */
static void pogo_transport_reset_acc_detection(struct pogo_transport *pogo_transport)
{
	int ret;

	ret = gvotable_cast_long_vote(pogo_transport->charger_mode_votable, POGO_VOTER,
				      GBMS_POGO_VOUT, 0);
	if (ret)
		logbuffer_log(pogo_transport->log, "%s: Failed to unvote VOUT, ret %d", __func__,
			      ret);

	ret = pogo_transport_acc_regulator(pogo_transport, false);
	if (ret)
		logbuffer_log(pogo_transport->log, "%s: Failed to disable acc_detect %d", __func__,
			      ret);

	if (pogo_transport->acc_irq_enabled) {
		disable_irq(pogo_transport->pogo_acc_irq);
		pogo_transport->acc_irq_enabled = false;
	}

	if (!pogo_transport->pogo_irq_enabled) {
		enable_irq(pogo_transport->pogo_irq);
		pogo_transport->pogo_irq_enabled = true;
	}
}

/*
 * This function implements the actions unon entering each state.
 *
 * This function is guarded by (max77759_plat)->data_path_lock
 */
static void pogo_transport_run_state_machine(struct pogo_transport *pogo_transport)
{
	bool acc_detected = gpio_get_value(pogo_transport->pogo_acc_gpio);
	bool docked = !gpio_get_value(pogo_transport->pogo_gpio);
	struct max77759_plat *chip = pogo_transport->chip;
	int ret;

	switch (pogo_transport->state) {
	case STANDBY:
		/* DATA_STATUS_ENABLED */
		break;
	case DOCKING_DEBOUNCED:
		if (docked) {
			update_extcon_dev(pogo_transport, true, true);
			switch_to_hub_locked(pogo_transport);
			pogo_transport_set_state(pogo_transport, DOCK_HUB, 0);
		} else {
			pogo_transport_set_state(pogo_transport, STANDBY, 0);
		}
		break;
	case DOCK_HUB:
		/* clear Dock dock detected notification */
		/* Clear accessory detected notification */
		/* DATA_STATUS_DISABLED_DEVICE_DOCK */
		break;
	case DEVICE_DOCKING_DEBOUNCED:
		if (docked) {
			update_extcon_dev(pogo_transport, true, true);
			switch_to_hub_locked(pogo_transport);
			/* switch_to_hub_locked cleared data_active, set it here */
			chip->data_active = true;
			pogo_transport_set_state(pogo_transport, DOCK_DEVICE_HUB, 0);
		} else {
			pogo_transport_set_state(pogo_transport, DEVICE_DIRECT, 0);
		}
		break;
	case DOCK_DEVICE_HUB:
		/* DATA_STATUS_DISABLED_DEVICE_DOCK */
		break;
	case DEVICE_HUB_DOCKING_DEBOUNCED:
		if (docked) {
			update_extcon_dev(pogo_transport, true, true);
			pogo_transport_set_state(pogo_transport, DOCK_DEVICE_HUB, 0);
		} else {
			pogo_transport_set_state(pogo_transport, DEVICE_HUB, 0);
		}
		break;
	case AUDIO_DIRECT_DOCKING_DEBOUNCED:
		if (docked) {
			update_extcon_dev(pogo_transport, true, true);
			pogo_transport_set_state(pogo_transport, AUDIO_DIRECT_DOCK_OFFLINE, 0);
		} else {
			pogo_transport_set_state(pogo_transport, AUDIO_DIRECT, 0);
		}
		break;
	case AUDIO_DIRECT_DOCK_OFFLINE:
		/* Push Dock dock detected notification */
		break;
	case AUDIO_HUB_DOCKING_DEBOUNCED:
		if (docked) {
			update_extcon_dev(pogo_transport, true, true);
			pogo_transport_set_state(pogo_transport, DOCK_AUDIO_HUB, 0);
		} else {
			pogo_transport_set_state(pogo_transport, AUDIO_HUB, 0);
		}
		break;
	case HOST_DIRECT:
		/* DATA_STATUS_ENABLED */
		/* Clear Pogo accessory Detected */
		/* Clear USB accessory detected notification */
		break;
	case HOST_DIRECT_DOCKING_DEBOUNCED:
		if (docked) {
			update_extcon_dev(pogo_transport, true, true);
			if (pogo_transport->force_pogo) {
				switch_to_hub_locked(pogo_transport);
				/* switch_to_hub_locked cleared data_active, set it here */
				chip->data_active = true;
				pogo_transport_set_state(pogo_transport, DOCK_HUB_HOST_OFFLINE, 0);
			} else {
				pogo_transport_set_state(pogo_transport, HOST_DIRECT_DOCK_OFFLINE,
							 0);
			}
		} else {
			pogo_transport_set_state(pogo_transport, HOST_DIRECT, 0);
		}
		break;
	case HOST_DIRECT_DOCK_OFFLINE:
		/* Push Dock dock detected notification */
		break;
	case DOCK_HUB_HOST_OFFLINE:
		/* Push accessory detected notification */
		break;
	case STANDBY_ACC_DEBOUNCED:
	case DEVICE_DIRECT_ACC_DEBOUNCED:
	case DEVICE_HUB_ACC_DEBOUNCED:
	case AUDIO_DIRECT_ACC_DEBOUNCED:
	case AUDIO_HUB_ACC_DEBOUNCED:
	case HOST_DIRECT_ACC_DEBOUNCED:
		/* debounce fail; leave the IRQ and regulator enabled and do nothing */
		if (!acc_detected)
			break;

		/*
		 * Disable the IRQ to ignore the noise after POGO Vout is enabled. It will be
		 * re-enabled when HES reports the attach event.
		 */
		if (pogo_transport->acc_irq_enabled) {
			disable_irq(pogo_transport->pogo_acc_irq);
			pogo_transport->acc_irq_enabled = false;
		}

		/* TODO: queue work for gvotable cast vote if it takes too much time */
		ret = gvotable_cast_long_vote(pogo_transport->charger_mode_votable, POGO_VOTER,
					      GBMS_POGO_VOUT, 1);
		if (ret)
			logbuffer_log(pogo_transport->log, "%s: Failed to vote VOUT, ret %d",
				      __func__, ret);
		break;
	case ACC_DIRECT:
		/* Clear Pogo accessory Detected */
		/* Clear USB accessory detected notification */
		break;
	case ACC_DEVICE_HUB:
		/* DATA_STATUS_DISABLED_DEVICE_DOCK */
		break;
	case HOST_DIRECT_ACC_OFFLINE:
		/* Push Pogo accessory Detected */
		break;
	default:
		break;
	}
}

/* Main loop of the State Machine */
static void pogo_transport_state_machine_work(struct kthread_work *work)
{
	struct pogo_transport *pogo_transport =
			container_of(container_of(work, struct kthread_delayed_work, work),
			     struct pogo_transport, state_machine);
	struct max77759_plat *chip = pogo_transport->chip;
	enum pogo_state prev_state;

	mutex_lock(&chip->data_path_lock);
	pogo_transport->state_machine_running = true;

	if (pogo_transport->delayed_state) {
		logbuffer_logk(pogo_transport->log, LOGLEVEL_INFO,
			       "state change %s -> %s [delayed %ld ms]",
			       pogo_states[pogo_transport->state],
			       pogo_states[pogo_transport->delayed_state],
			       pogo_transport->delay_ms);
		pogo_transport->prev_state = pogo_transport->state;
		pogo_transport->state = pogo_transport->delayed_state;
		pogo_transport->delayed_state = INVALID_STATE;
	}

	do {
		prev_state = pogo_transport->state;
		pogo_transport_run_state_machine(pogo_transport);
	} while (pogo_transport->state != prev_state && !pogo_transport->delayed_state);

	pogo_transport->state_machine_running = false;
	mutex_unlock(&chip->data_path_lock);
}

/*
 * Called when POGO Voltage Detection IRQ is active
 *  - Triggered from event: EVENT_POGO_IRQ
 *
 * This function is guarded by (max77759_plat)->data_path_lock
 */
static void pogo_transport_pogo_irq_active(struct pogo_transport *pogo_transport)
{
	switch (pogo_transport->state) {
	case STANDBY:
		pogo_transport_set_state(pogo_transport, DOCKING_DEBOUNCED, POGO_PSY_DEBOUNCE_MS);
		break;
	case DEVICE_HUB:
		pogo_transport_set_state(pogo_transport, DEVICE_HUB_DOCKING_DEBOUNCED,
					 POGO_PSY_DEBOUNCE_MS);
		break;
	case DEVICE_DIRECT:
		pogo_transport_set_state(pogo_transport, DEVICE_DOCKING_DEBOUNCED,
					 POGO_PSY_DEBOUNCE_MS);
		break;
	case AUDIO_DIRECT:
		pogo_transport_set_state(pogo_transport, AUDIO_DIRECT_DOCKING_DEBOUNCED,
					 POGO_PSY_DEBOUNCE_MS);
		break;
	case AUDIO_HUB:
		pogo_transport_set_state(pogo_transport, AUDIO_HUB_DOCKING_DEBOUNCED,
					 POGO_PSY_DEBOUNCE_MS);
		break;
	case HOST_DIRECT:
		pogo_transport_set_state(pogo_transport, HOST_DIRECT_DOCKING_DEBOUNCED,
					 POGO_PSY_DEBOUNCE_MS);
		break;
	default:
		break;
	}
}

/*
 * Called when POGO Voltage Detection IRQ is standby
 *  - Triggered from event: EVENT_POGO_IRQ
 *
 * This function is guarded by (max77759_plat)->data_path_lock
 */
static void pogo_transport_pogo_irq_standby(struct pogo_transport *pogo_transport)
{
	struct max77759_plat *chip = pogo_transport->chip;

	switch (pogo_transport->state) {
	case STANDBY:
		update_extcon_dev(pogo_transport, false, false);
		pogo_transport_set_state(pogo_transport, STANDBY, 0);
		break;
	case DOCK_HUB:
		update_extcon_dev(pogo_transport, false, false);
		switch_to_usbc_locked(pogo_transport);
		pogo_transport_set_state(pogo_transport, STANDBY, 0);
		break;
	case DOCK_DEVICE_HUB:
		update_extcon_dev(pogo_transport, false, false);
		pogo_transport_set_state(pogo_transport, DEVICE_HUB, 0);
		break;
	case DOCK_AUDIO_HUB:
		update_extcon_dev(pogo_transport, false, false);
		pogo_transport_set_state(pogo_transport, AUDIO_HUB, 0);
		break;
	case AUDIO_DIRECT_DOCK_OFFLINE:
		update_extcon_dev(pogo_transport, false, false);
		pogo_transport_set_state(pogo_transport, AUDIO_DIRECT, 0);
		break;
	case HOST_DIRECT_DOCK_OFFLINE:
		update_extcon_dev(pogo_transport, false, false);
		pogo_transport_set_state(pogo_transport, HOST_DIRECT, 0);
		break;
	case DOCK_HUB_HOST_OFFLINE:
		update_extcon_dev(pogo_transport, false, false);
		/* Clear data_active so that Type-C stack is able to enable the USB data later */
		chip->data_active = false;
		switch_to_usbc_locked(pogo_transport);
		pogo_transport_set_state(pogo_transport, HOST_DIRECT, 0);
		break;
	default:
		break;
	}
}

/*
 * Called when USB-C port enters Host Mode
 *  - Triggered from event: EVENT_USBC_DATA_CHANGE
 *
 * This function is guarded by (max77759_plat)->data_path_lock
 */
static void pogo_transport_usbc_host_on(struct pogo_transport *pogo_transport)
{
	struct max77759_plat *chip = pogo_transport->chip;

	switch (pogo_transport->state) {
	case STANDBY:
		pogo_transport_set_state(pogo_transport, DEVICE_DIRECT, 0);
		break;
	case DOCK_HUB:
		/* Set data_active since USB-C device is attached */
		chip->data_active = true;
		pogo_transport_set_state(pogo_transport, DOCK_DEVICE_HUB, 0);
		break;
	case ACC_DIRECT:
		switch_to_hub_locked(pogo_transport);
		/* Set data_active since USB-C device is attached */
		chip->data_active = true;
		pogo_transport_set_state(pogo_transport, ACC_DEVICE_HUB, 0);
		break;
	case ACC_HUB:
		/* Set data_active since USB-C device is attached */
		chip->data_active = true;
		pogo_transport_set_state(pogo_transport, ACC_DEVICE_HUB, 0);
		break;
	default:
		break;
	}
}

/*
 * Called when USB-C port leaves Host Mode
 *  - Triggered from event: EVENT_USBC_DATA_CHANGE
 *
 * This function is guarded by (max77759_plat)->data_path_lock
 */
static void pogo_transport_usbc_host_off(struct pogo_transport *pogo_transport)
{
	struct max77759_plat *chip = pogo_transport->chip;
	bool ss_attached = pogo_transport->ss_udev_attached;

	pogo_transport->ss_udev_attached = false;

	switch (pogo_transport->state) {
	case DOCK_DEVICE_HUB:
		/* Clear data_active since USB-C device is detached */
		chip->data_active = false;
		pogo_transport_set_state(pogo_transport, DOCK_HUB, 0);
		break;
	case DEVICE_HUB:
		switch_to_usbc_locked(pogo_transport);
		pogo_transport_set_state(pogo_transport, STANDBY, 0);
		break;
	case DEVICE_DIRECT:
	case AUDIO_DIRECT:
		pogo_transport_set_state(pogo_transport, STANDBY, 0);
		break;
	case AUDIO_DIRECT_DOCK_OFFLINE:
		switch_to_hub_locked(pogo_transport);
		pogo_transport_set_state(pogo_transport, DOCK_HUB, 0);
		break;
	case DOCK_AUDIO_HUB:
		/* Clear data_active since USB-C device is detached */
		chip->data_active = false;
		pogo_transport_set_state(pogo_transport, DOCK_HUB, 0);
		break;
	case AUDIO_HUB:
		switch_to_usbc_locked(pogo_transport);
		pogo_transport_set_state(pogo_transport, STANDBY, 0);
		break;
	case ACC_DEVICE_HUB:
	case ACC_AUDIO_HUB:
		/* b/271669059 */
		if (ss_attached) {
			/* USB_MUX_HUB_SEL set to 0 to bypass the hub */
			gpio_set_value(pogo_transport->pogo_hub_sel_gpio, 0);
			logbuffer_log(pogo_transport->log, "POGO: toggling hub-mux, hub-mux:%d",
				      gpio_get_value(pogo_transport->pogo_hub_sel_gpio));
			mdelay(10);
			/* USB_MUX_HUB_SEL set to 1 to switch the path to hub */
			gpio_set_value(pogo_transport->pogo_hub_sel_gpio, 1);
			logbuffer_log(pogo_transport->log, "POGO: hub-mux:%d",
				      gpio_get_value(pogo_transport->pogo_hub_sel_gpio));
		}

		/* Clear data_active since USB-C device is detached */
		chip->data_active = false;
		pogo_transport_set_state(pogo_transport, ACC_HUB, 0);
		break;
	default:
		break;
	}
}

/*
 * Called when USB-C port enters Device Mode
 *  - Triggered from event: EVENT_USBC_DATA_CHANGE
 *
 * This function is guarded by (max77759_plat)->data_path_lock
 */
static void pogo_transport_usbc_device_on(struct pogo_transport *pogo_transport)
{
	struct max77759_plat *chip = pogo_transport->chip;

	switch (pogo_transport->state) {
	case STANDBY:
		pogo_transport_set_state(pogo_transport, HOST_DIRECT, 0);
		break;
	case DOCK_HUB:
		/*
		 * Set data_active so that once USB-C cable is detached later, Type-C stack is able
		 * to call back for the data changed event
		 */
		chip->data_active = true;
		pogo_transport_set_state(pogo_transport, DOCK_HUB_HOST_OFFLINE, 0);
		break;
	case ACC_DIRECT:
		/*
		 * Set data_active so that once USB-C cable is detached later, Type-C stack is able
		 * to call back for the data changed event
		 */
		chip->data_active = true;
		pogo_transport_set_state(pogo_transport, ACC_DIRECT_HOST_OFFLINE, 0);
		break;
	case ACC_HUB:
		switch_to_pogo_locked(pogo_transport);
		/*
		 * Set data_active so that once USB-C cable is detached later, Type-C stack is able
		 * to call back for the data changed event
		 */
		chip->data_active = true;
		pogo_transport_set_state(pogo_transport, ACC_DIRECT_HOST_OFFLINE, 0);
		break;
	default:
		break;
	}
}

/*
 * Called when USB-C port leaves Device Mode
 *  - Triggered from event: EVENT_USBC_DATA_CHANGE
 *
 * This function is guarded by (max77759_plat)->data_path_lock
 */
static void pogo_transport_usbc_device_off(struct pogo_transport *pogo_transport)
{
	struct max77759_plat *chip = pogo_transport->chip;

	switch (pogo_transport->state) {
	case HOST_DIRECT:
		pogo_transport_set_state(pogo_transport, STANDBY, 0);
		break;
	case HOST_DIRECT_DOCK_OFFLINE:
		switch_to_hub_locked(pogo_transport);
		pogo_transport_set_state(pogo_transport, DOCK_HUB, 0);
		break;
	case DOCK_HUB_HOST_OFFLINE:
		/*
		 * Clear data_active so that Type-C stack is able to call back for the data changed
		 * event
		 */
		chip->data_active = false;
		pogo_transport_set_state(pogo_transport, DOCK_HUB, 0);
		break;
	case HOST_DIRECT_ACC_OFFLINE:
		switch_to_pogo_locked(pogo_transport);
		pogo_transport_set_state(pogo_transport, ACC_DIRECT, 0);
		break;
	case ACC_DIRECT_HOST_OFFLINE:
		/*
		 * Clear data_active so that Type-C stack is able to call back for the data changed
		 * event
		 */
		chip->data_active = false;
		pogo_transport_set_state(pogo_transport, ACC_DIRECT, 0);
		break;
	default:
		break;
	}
}

/*
 * Called when device attribute "move_data_to_usb" is written to 1
 *  - Triggered from event: EVENT_ENABLE_USB_DATA
 *
 * This function is guarded by (max77759_plat)->data_path_lock
 */
static void pogo_transport_enable_usb_data(struct pogo_transport *pogo_transport)
{
	struct max77759_plat *chip = pogo_transport->chip;

	switch (pogo_transport->state) {
	case DOCK_HUB_HOST_OFFLINE:
		/*
		 * Clear data_active so that Type-C stack is able to call back for the data changed
		 * event later
		 */
		chip->data_active = false;
		switch_to_usbc_locked(pogo_transport);
		pogo_transport_set_state(pogo_transport, HOST_DIRECT_DOCK_OFFLINE, 0);
		break;
	case ACC_DIRECT_HOST_OFFLINE:
		/*
		 * Clear data_active so that Type-C stack is able to call back for the data changed
		 * event later
		 */
		chip->data_active = false;
		switch_to_usbc_locked(pogo_transport);
		pogo_transport_set_state(pogo_transport, HOST_DIRECT_ACC_OFFLINE, 0);
		break;
	default:
		return;
	}
}

/*
 * Call this function to:
 *  - Disable Accessory Detection IRQ
 *  - Disable POGO Voltage Detection IRQ
 *  - Enable POGO Vout by voting 1 to charger_mode_votable
 *
 *  This function is guarded by (max77759_plat)->data_path_lock
 */
static void pogo_transport_skip_acc_detection(struct pogo_transport *pogo_transport)
{
	int ret;

	logbuffer_log(pogo_transport->log, "%s: Skip enabling comparator logic, enable vout",
		      __func__);

	if (pogo_transport->acc_irq_enabled) {
		disable_irq(pogo_transport->pogo_acc_irq);
		pogo_transport->acc_irq_enabled = false;
	}

	if (pogo_transport->pogo_irq_enabled) {
		disable_irq(pogo_transport->pogo_irq);
		pogo_transport->pogo_irq_enabled = false;
	}

	ret = gvotable_cast_long_vote(pogo_transport->charger_mode_votable,
				      POGO_VOTER, GBMS_POGO_VOUT, 1);
	if (ret)
		logbuffer_log(pogo_transport->log, "%s: Failed to vote VOUT, ret %d", __func__,
			      ret);
}

/*
 * Called when device attribute "hall1_s" is written to non-zero
 *  - If accessory_detection_enabled == ENABLED, it won't involve the State transition.
 *    Enable the Accessory Detection IRQ and the regulator for the later detection process.
 *  - If accessory_detection_enabled == HALL_ONLY, transition to related ACC states
 *  - Triggered from event: EVENT_HES_H1S_CHANGED
 *
 * This function is guarded by (max77759_plat)->data_path_lock
 */
static void pogo_transport_hes_acc_detected(struct pogo_transport *pogo_transport)
{
	struct max77759_plat *chip = pogo_transport->chip;
	int ret;

	/* Disable OVP to prevent the voltage going through POGO_VIN */
	if (pogo_transport->pogo_ovp_en_gpio >= 0)
		gpio_set_value_cansleep(pogo_transport->pogo_ovp_en_gpio,
					!pogo_transport->pogo_ovp_en_active_state);

	if (pogo_transport->accessory_detection_enabled == ENABLED) {
		if (!pogo_transport->acc_irq_enabled) {
			enable_irq(pogo_transport->pogo_acc_irq);
			pogo_transport->acc_irq_enabled = true;
		}

		ret = pogo_transport_acc_regulator(pogo_transport, true);
		if (ret)
			logbuffer_log(pogo_transport->log, "%s: Failed to enable acc_detect %d",
				      __func__, ret);
	} else if (pogo_transport->accessory_detection_enabled == HALL_ONLY) {
		switch (pogo_transport->state) {
		case STANDBY:
			pogo_transport_skip_acc_detection(pogo_transport);
			if (!pogo_transport->mfg_acc_test)
				switch_to_pogo_locked(pogo_transport);
			pogo_transport_set_state(pogo_transport, ACC_DIRECT, 0);
			break;
		case DEVICE_DIRECT:
		case AUDIO_DIRECT:
			pogo_transport_skip_acc_detection(pogo_transport);
			switch_to_hub_locked(pogo_transport);
			pogo_transport_set_state(pogo_transport, ACC_DEVICE_HUB, 0);
			break;
		case DEVICE_HUB:
			pogo_transport_skip_acc_detection(pogo_transport);
			pogo_transport_set_state(pogo_transport, ACC_DEVICE_HUB, 0);
			break;
		case AUDIO_HUB:
			pogo_transport_skip_acc_detection(pogo_transport);
			pogo_transport_set_state(pogo_transport, ACC_AUDIO_HUB, 0);
			break;
		case HOST_DIRECT:
			pogo_transport_skip_acc_detection(pogo_transport);
			if (pogo_transport->force_pogo) {
				switch_to_pogo_locked(pogo_transport);
				/*
				 * Set data_active so that once USB-C cable is detached later,
				 * Type-C stack is able to call back for the data changed event
				 */
				chip->data_active = true;
				pogo_transport_set_state(pogo_transport, ACC_DIRECT_HOST_OFFLINE,
							 0);
			} else {
				pogo_transport_set_state(pogo_transport, HOST_DIRECT_ACC_OFFLINE,
							 0);
			}
			break;
		default:
			break;
		}
	}
}

/*
 * Called when device attribute "hall1_s" is written to 0
 * - Triggered from event: EVENT_HES_H1S_CHANGED
 *
 * This function is guarded by (max77759_plat)->data_path_lock
 */
static void pogo_transport_hes_acc_detached(struct pogo_transport *pogo_transport)
{
	struct max77759_plat *chip = pogo_transport->chip;

	switch (pogo_transport->state) {
	case STANDBY_ACC_DEBOUNCED:
		pogo_transport_reset_acc_detection(pogo_transport);
		pogo_transport_set_state(pogo_transport, STANDBY, 0);
		break;
	case DEVICE_DIRECT_ACC_DEBOUNCED:
		pogo_transport_reset_acc_detection(pogo_transport);
		pogo_transport_set_state(pogo_transport, DEVICE_DIRECT, 0);
		break;
	case DEVICE_HUB_ACC_DEBOUNCED:
		pogo_transport_reset_acc_detection(pogo_transport);
		pogo_transport_set_state(pogo_transport, DEVICE_HUB, 0);
		break;
	case AUDIO_DIRECT_ACC_DEBOUNCED:
		pogo_transport_reset_acc_detection(pogo_transport);
		pogo_transport_set_state(pogo_transport, AUDIO_DIRECT, 0);
		break;
	case AUDIO_HUB_ACC_DEBOUNCED:
		pogo_transport_reset_acc_detection(pogo_transport);
		pogo_transport_set_state(pogo_transport, AUDIO_HUB, 0);
		break;
	case HOST_DIRECT_ACC_DEBOUNCED:
		pogo_transport_reset_acc_detection(pogo_transport);
		pogo_transport_set_state(pogo_transport, HOST_DIRECT, 0);
		break;
	case ACC_DIRECT:
	case ACC_HUB:
		pogo_transport_reset_acc_detection(pogo_transport);
		switch_to_usbc_locked(pogo_transport);
		pogo_transport_set_state(pogo_transport, STANDBY, 0);
		break;
	case ACC_DEVICE_HUB:
		pogo_transport_reset_acc_detection(pogo_transport);
		pogo_transport_set_state(pogo_transport, DEVICE_HUB, 0);
		break;
	case ACC_AUDIO_HUB:
		pogo_transport_reset_acc_detection(pogo_transport);
		pogo_transport_set_state(pogo_transport, AUDIO_HUB, 0);
		break;
	case HOST_DIRECT_ACC_OFFLINE:
		pogo_transport_reset_acc_detection(pogo_transport);
		pogo_transport_set_state(pogo_transport, HOST_DIRECT, 0);
		break;
	case ACC_DIRECT_HOST_OFFLINE:
		pogo_transport_reset_acc_detection(pogo_transport);
		/* Clear data_active so that Type-C stack is able to enable the USB data later */
		chip->data_active = false;
		switch_to_usbc_locked(pogo_transport);
		pogo_transport_set_state(pogo_transport, HOST_DIRECT, 0);
		break;
	default:
		break;
	}
}

/*
 * Called when Accessory Detection IRQ is active
 *  - Triggered from event: EVENT_ACC_GPIO_ACTIVE
 *
 * This function is guarded by (max77759_plat)->data_path_lock
 */
static void pogo_transport_acc_debouncing(struct pogo_transport *pogo_transport)
{
	switch (pogo_transport->state) {
	case STANDBY:
	case STANDBY_ACC_DEBOUNCED:
		pogo_transport_set_state(pogo_transport, STANDBY_ACC_DEBOUNCED,
					 pogo_transport->pogo_acc_gpio_debounce_ms);
		break;
	case DEVICE_DIRECT:
	case DEVICE_DIRECT_ACC_DEBOUNCED:
		pogo_transport_set_state(pogo_transport, DEVICE_DIRECT_ACC_DEBOUNCED,
					 pogo_transport->pogo_acc_gpio_debounce_ms);
		break;
	case DEVICE_HUB:
	case DEVICE_HUB_ACC_DEBOUNCED:
		pogo_transport_set_state(pogo_transport, DEVICE_HUB_ACC_DEBOUNCED,
					 pogo_transport->pogo_acc_gpio_debounce_ms);
		break;
	case AUDIO_DIRECT:
	case AUDIO_DIRECT_ACC_DEBOUNCED:
		pogo_transport_set_state(pogo_transport, AUDIO_DIRECT_ACC_DEBOUNCED,
					 pogo_transport->pogo_acc_gpio_debounce_ms);
		break;
	case AUDIO_HUB:
	case AUDIO_HUB_ACC_DEBOUNCED:
		pogo_transport_set_state(pogo_transport, AUDIO_HUB_ACC_DEBOUNCED,
					 pogo_transport->pogo_acc_gpio_debounce_ms);
		break;
	case HOST_DIRECT:
	case HOST_DIRECT_ACC_DEBOUNCED:
		pogo_transport_set_state(pogo_transport, HOST_DIRECT_ACC_DEBOUNCED,
					 pogo_transport->pogo_acc_gpio_debounce_ms);
		break;
	default:
		break;
	}
}

/*
 * Called when POGO Voltage Detection IRQ is active while Accessory Detection regulator is enabled.
 *  - Triggered from event: EVENT_ACC_CONNECTED
 *
 * This function is guarded by (max77759_plat)->data_path_lock
 */
static void pogo_transport_acc_connected(struct pogo_transport *pogo_transport)
{
	struct max77759_plat *chip = pogo_transport->chip;
	int ret;

	/*
	 * FIXME: is it possible that when acc regulator is enabled and pogo irq become active
	 * because 12V input through pogo pin? e.g. keep magnet closed to the device and then
	 * docking on korlan?
	 */

	switch (pogo_transport->state) {
	case STANDBY_ACC_DEBOUNCED:
		ret = pogo_transport_acc_regulator(pogo_transport, false);
		if (ret)
			logbuffer_log(pogo_transport->log, "%s: Failed to disable acc_detect %d",
				      __func__, ret);

		if (!pogo_transport->mfg_acc_test)
			switch_to_pogo_locked(pogo_transport);
		pogo_transport_set_state(pogo_transport, ACC_DIRECT, 0);
		break;
	case DEVICE_DIRECT_ACC_DEBOUNCED:
	case AUDIO_DIRECT_ACC_DEBOUNCED:
		ret = pogo_transport_acc_regulator(pogo_transport, false);
		if (ret)
			logbuffer_log(pogo_transport->log, "%s: Failed to disable acc_detect %d",
				      __func__, ret);

		switch_to_hub_locked(pogo_transport);
		chip->data_active = true;
		pogo_transport_set_state(pogo_transport, ACC_DEVICE_HUB, 0);
		break;
	case DEVICE_HUB_ACC_DEBOUNCED:
		ret = pogo_transport_acc_regulator(pogo_transport, false);
		if (ret)
			logbuffer_log(pogo_transport->log, "%s: Failed to disable acc_detect %d",
				      __func__, ret);

		pogo_transport_set_state(pogo_transport, ACC_DEVICE_HUB, 0);
		break;
	case AUDIO_HUB_ACC_DEBOUNCED:
		ret = pogo_transport_acc_regulator(pogo_transport, false);
		if (ret)
			logbuffer_log(pogo_transport->log, "%s: Failed to disable acc_detect %d",
				      __func__, ret);

		pogo_transport_set_state(pogo_transport, ACC_AUDIO_HUB, 0);
		break;
	case HOST_DIRECT_ACC_DEBOUNCED:
		ret = pogo_transport_acc_regulator(pogo_transport, false);
		if (ret)
			logbuffer_log(pogo_transport->log, "%s: Failed to disable acc_detect %d",
				      __func__, ret);

		if (pogo_transport->force_pogo) {
			switch_to_pogo_locked(pogo_transport);
			/*
			 * Set data_active so that once USB-C cable is detached later, Type-C stack
			 * is able to call back for the data changed event
			 */
			chip->data_active = true;
			pogo_transport_set_state(pogo_transport, ACC_DIRECT_HOST_OFFLINE, 0);
		} else {
			pogo_transport_set_state(pogo_transport, HOST_DIRECT_ACC_OFFLINE, 0);
		}
		break;
	default:
		break;
	}
}

/*
 * Called when a USB device with AUDIO Class is enumerated.
 *  - Triggered from event: EVENT_AUDIO_DEV_ATTACHED
 *
 * This function is guarded by (max77759_plat)->data_path_lock
 */
static void pogo_transport_audio_dev_attached(struct pogo_transport *pogo_transport)
{
	switch (pogo_transport->state) {
	case DOCK_DEVICE_HUB:
		pogo_transport_set_state(pogo_transport, DOCK_AUDIO_HUB, 0);
		break;
	case DEVICE_DIRECT:
		pogo_transport_set_state(pogo_transport, AUDIO_DIRECT, 0);
		break;
	case ACC_DEVICE_HUB:
		pogo_transport_set_state(pogo_transport, ACC_AUDIO_HUB, 0);
		break;
	default:
		break;
	}
}

/*
 * Called when the detected orientation on USB-C port is changed.
 *  - Triggered from event: EVENT_USBC_ORIENTATION
 *
 * This function is guarded by (max77759_plat)->data_path_lock
 */
static void pogo_transport_usbc_orientation_changed(struct pogo_transport *pogo_transport)
{
	/*
	 * TODO: It is possible that USB-C is toggling between CC2 and Open. We may need to wait for
	 * the orientation being settled and then update the ssphy.
	 */
	switch (pogo_transport->state) {
	/* usbc being connected while hub is enabled */
	case DOCK_HUB:
	case ACC_HUB:
	/* usbc being disconnected while hub is enabled */
	case DOCK_DEVICE_HUB:
	case DOCK_AUDIO_HUB:
	case DOCK_HUB_HOST_OFFLINE:
	case ACC_DEVICE_HUB:
	case ACC_AUDIO_HUB:
		pogo_transport_update_polarity(pogo_transport, (int)pogo_transport->polarity, true);
		ssphy_restart_control(pogo_transport, true);
		break;
	default:
		break;
	}
}

static void pogo_transport_event_handler(struct kthread_work *work)
{
	struct pogo_transport *pogo_transport = container_of(work, struct pogo_transport,
							     event_work);
	struct max77759_plat *chip = pogo_transport->chip;
	unsigned long events;

	mutex_lock(&chip->data_path_lock);
	spin_lock(&pogo_transport->pogo_event_lock);
	while (pogo_transport->event_map) {
		events = pogo_transport->event_map;
		pogo_transport->event_map = 0;

		spin_unlock(&pogo_transport->pogo_event_lock);

		if (events & EVENT_POGO_IRQ) {
			int pogo_gpio = gpio_get_value(pogo_transport->pogo_gpio);

			logbuffer_log(pogo_transport->log, "EV:POGO_IRQ %s", pogo_gpio ?
				      "STANDBY" : "ACTIVE");
			if (pogo_gpio)
				pogo_transport_pogo_irq_standby(pogo_transport);
			else
				pogo_transport_pogo_irq_active(pogo_transport);
		}
		if (events & EVENT_USBC_ORIENTATION) {
			logbuffer_log(pogo_transport->log, "EV:ORIENTATION %u",
				      pogo_transport->polarity);
			pogo_transport_usbc_orientation_changed(pogo_transport);
		}
		if (events & EVENT_USBC_DATA_CHANGE) {
			logbuffer_log(pogo_transport->log, "EV:DATA_CHANGE usbc-role %u usbc-active %u",
				      pogo_transport->usbc_data_role,
				      pogo_transport->usbc_data_active);
			if (pogo_transport->usbc_data_role == TYPEC_HOST) {
				if (pogo_transport->usbc_data_active)
					pogo_transport_usbc_host_on(pogo_transport);
				else
					pogo_transport_usbc_host_off(pogo_transport);
			} else {
				if (pogo_transport->usbc_data_active)
					pogo_transport_usbc_device_on(pogo_transport);
				else
					pogo_transport_usbc_device_off(pogo_transport);
			}
		}
		if (events & EVENT_ENABLE_USB_DATA) {
			logbuffer_log(pogo_transport->log, "EV:ENABLE_USB");
			pogo_transport_enable_usb_data(pogo_transport);
		}
		if (events & EVENT_HES_H1S_CHANGED) {
			logbuffer_log(pogo_transport->log, "EV:H1S state %d",
				      pogo_transport->hall1_s_state);
			if (pogo_transport->hall1_s_state)
				pogo_transport_hes_acc_detected(pogo_transport);
			else
				pogo_transport_hes_acc_detached(pogo_transport);
		}
		if (events & EVENT_ACC_GPIO_ACTIVE) {
			logbuffer_log(pogo_transport->log, "EV:ACC_GPIO_ACTIVE");
			pogo_transport_acc_debouncing(pogo_transport);
		}
		if (events & EVENT_ACC_CONNECTED) {
			logbuffer_log(pogo_transport->log, "EV:ACC_CONNECTED");
			pogo_transport_acc_connected(pogo_transport);
		}
		if (events & EVENT_AUDIO_DEV_ATTACHED) {
			logbuffer_log(pogo_transport->log, "EV:AUDIO_ATTACHED");
			pogo_transport_audio_dev_attached(pogo_transport);
		}

		spin_lock(&pogo_transport->pogo_event_lock);
	}
	spin_unlock(&pogo_transport->pogo_event_lock);
	mutex_unlock(&chip->data_path_lock);
}

static void pogo_transport_queue_event(struct pogo_transport *pogo_transport, unsigned long event)
{
	unsigned long flags;

	pm_wakeup_event(pogo_transport->dev, POGO_TIMEOUT_MS);
	/*
	 * Print the event number derived from the bit position; e.g. BIT(0) -> 0
	 * Note that ffs() only return the least significant set bit.
	 */
	logbuffer_log(pogo_transport->log, "QUEUE EVENT %d", ffs((int)event) - 1);

	spin_lock_irqsave(&pogo_transport->pogo_event_lock, flags);
	pogo_transport->event_map |= event;
	spin_unlock_irqrestore(&pogo_transport->pogo_event_lock, flags);

	kthread_queue_work(pogo_transport->wq, &pogo_transport->event_work);
}

/*-------------------------------------------------------------------------*/
/* Events triggering                                                       */
/*-------------------------------------------------------------------------*/

static irqreturn_t pogo_acc_irq(int irq, void *dev_id)
{
	struct pogo_transport *pogo_transport = dev_id;
	int pogo_acc_gpio = gpio_get_value(pogo_transport->pogo_acc_gpio);

	logbuffer_log(pogo_transport->log, "Pogo acc threaded irq running, acc_detect %u",
		      pogo_acc_gpio);

	if (pogo_transport->state_machine_enabled) {
		if (pogo_acc_gpio)
			pogo_transport_queue_event(pogo_transport, EVENT_ACC_GPIO_ACTIVE);
		return IRQ_HANDLED;
	}

	if (pogo_acc_gpio)
		pogo_transport_event(pogo_transport, EVENT_POGO_ACC_DEBOUNCED,
				     pogo_transport->pogo_acc_gpio_debounce_ms);
	else
		kthread_cancel_delayed_work_sync(&pogo_transport->pogo_accessory_debounce_work);

	return IRQ_HANDLED;
}

static irqreturn_t pogo_acc_isr(int irq, void *dev_id)
{
	struct pogo_transport *pogo_transport = dev_id;

	logbuffer_log(pogo_transport->log, "POGO ACC IRQ triggered");
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
		if (pogo_transport->pogo_irq_enabled) {
			/* disable the irq to prevent the interrupt storm after pogo 5v out */
			disable_irq_nosync(pogo_transport->pogo_irq);
			pogo_transport->pogo_irq_enabled = false;
			if (pogo_transport->state_machine_enabled)
				pogo_transport_queue_event(pogo_transport, EVENT_ACC_CONNECTED);
			else
				pogo_transport_event(pogo_transport, EVENT_POGO_ACC_CONNECTED, 0);
		}
		return IRQ_HANDLED;
	}


	if (pogo_transport->pogo_ovp_en_gpio >= 0) {
		int ret;

		/*
		 * Vote GBMS_POGO_VIN to notify BMS that there is input voltage on pogo power and
		 * it is over the threshold if pogo_gpio (ACTIVE_LOW) is in active state (0)
		 */
		ret = gvotable_cast_long_vote(pogo_transport->charger_mode_votable, POGO_VOTER,
					      GBMS_POGO_VIN, !pogo_gpio);
		if (ret)
			logbuffer_log(pogo_transport->log, "%s: Failed to vote VIN, ret %d",
				      __func__, ret);
	}

	if (pogo_transport->state_machine_enabled)
		pogo_transport_queue_event(pogo_transport, EVENT_POGO_IRQ);
	else
		pogo_transport_event(pogo_transport, EVENT_DOCKING, !pogo_gpio ?
				     POGO_PSY_DEBOUNCE_MS : 0);
	return IRQ_HANDLED;
}

static irqreturn_t pogo_isr(int irq, void *dev_id)
{
	struct pogo_transport *pogo_transport = dev_id;

	logbuffer_log(pogo_transport->log, "POGO IRQ triggered");
	pm_wakeup_event(pogo_transport->dev, POGO_TIMEOUT_MS);

	return IRQ_WAKE_THREAD;
}

static void data_active_changed(void *data, enum typec_data_role role, bool active)
{
	struct pogo_transport *pogo_transport = data;

	logbuffer_log(pogo_transport->log, "%s: role %u active %d", __func__, role, active);

	pogo_transport->usbc_data_role = role;
	pogo_transport->usbc_data_active = active;

	if (pogo_transport->state_machine_enabled)
		pogo_transport_queue_event(pogo_transport, EVENT_USBC_DATA_CHANGE);
	else
		pogo_transport_event(pogo_transport, EVENT_DATA_ACTIVE_CHANGED, 0);
}

static void orientation_changed(void *data)
{
	struct pogo_transport *pogo_transport = data;
	struct max77759_plat *chip = pogo_transport->chip;

	if (pogo_transport->polarity != chip->polarity) {
		pogo_transport->polarity = chip->polarity;
		if (pogo_transport->state_machine_enabled)
			pogo_transport_queue_event(pogo_transport, EVENT_USBC_ORIENTATION);
		else
			pogo_transport_event(pogo_transport, EVENT_ORIENTATION_CHANGED, 0);
	}
}

/* Called when a USB hub/device (exclude root hub) is enumerated */
static void pogo_transport_udev_add(struct pogo_transport *pogo_transport, struct usb_device *udev)
{
	struct usb_interface_descriptor *desc;
	struct usb_host_config *config;
	bool audio_dock = false;
	bool audio_dev = false;
	int i;

	/* Don't proceed to the event handling if the udev is an Audio Dock. Skip the check. */
	if (pogo_transport_match_udev(audio_dock_ids, le16_to_cpu(udev->descriptor.idVendor),
				      le16_to_cpu(udev->descriptor.idProduct))) {
		audio_dock = true;
		goto skip_audio_check;
	}

	if (udev->speed >= USB_SPEED_SUPER)
		pogo_transport->ss_udev_attached = true;

	config = udev->config;
	for (i = 0; i < config->desc.bNumInterfaces; i++) {
		desc = &config->intf_cache[i]->altsetting->desc;
		if (desc->bInterfaceClass == USB_CLASS_AUDIO) {
			audio_dev = true;
			break;
		}
	}

skip_audio_check:
	logbuffer_log(pogo_transport->log, "udev added %04X:%04X [%s%s%s%s]",
		      le16_to_cpu(udev->descriptor.idVendor),
		      le16_to_cpu(udev->descriptor.idProduct),
		      udev->speed >= USB_SPEED_SUPER ? "Ss" : "",
		      udev->descriptor.bDeviceClass == USB_CLASS_HUB ? "Hu" : "",
		      audio_dock ? "Do" : "",
		      audio_dev ? "Au" : "");

	if (audio_dev && pogo_transport->state_machine_enabled)
		pogo_transport_queue_event(pogo_transport, EVENT_AUDIO_DEV_ATTACHED);
}

/* notifier callback from usb core */
static int pogo_transport_udev_notify(struct notifier_block *nb, unsigned long action, void *dev)
{
	struct pogo_transport *pogo_transport = container_of(nb, struct pogo_transport, udev_nb);
	struct usb_device *udev = dev;

	switch (action) {
	case USB_DEVICE_ADD:
		/* Don't care about the root hubs. */
		if (udev->bus->root_hub == udev)
			break;

		pogo_transport_udev_add(pogo_transport, udev);
		break;
	case USB_DEVICE_REMOVE:
		/* Don't care about the root hubs. */
		if (udev->bus->root_hub == udev)
			break;

		logbuffer_log(pogo_transport->log, "udev removed %04X:%04X",
			      le16_to_cpu(udev->descriptor.idVendor),
			      le16_to_cpu(udev->descriptor.idProduct));
		break;
	}

	return NOTIFY_OK;
}

#if IS_ENABLED(CONFIG_DEBUG_FS)
static int mock_hid_connected_set(void *data, u64 val)
{
	struct pogo_transport *pogo_transport = data;

	if (pogo_transport->state_machine_enabled) {
		logbuffer_log(pogo_transport->log, "state machine enabled; ignore mock hid");
		return 0;
	}

	pogo_transport->mock_hid_connected = !!val;

	logbuffer_log(pogo_transport->log, "%s: %u", __func__, pogo_transport->mock_hid_connected);

	if (pogo_transport->mock_hid_connected)
		pogo_transport_event(pogo_transport, EVENT_FORCE_ACC_CONNECT, 0);
	else
		pogo_transport_event(pogo_transport, EVENT_HALL_SENSOR_ACC_UNDOCKED, 0);

	return 0;
}

static int mock_hid_connected_get(void *data, u64 *val)
{
	struct pogo_transport *pogo_transport = data;

	*val = (u64)pogo_transport->mock_hid_connected;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(mock_hid_connected_fops, mock_hid_connected_get, mock_hid_connected_set,
			"%llu\n");

/*-------------------------------------------------------------------------*/
/* Initialization                                                          */
/*-------------------------------------------------------------------------*/

static void pogo_transport_init_debugfs(struct pogo_transport *pogo_transport)
{
	struct dentry *dentry;

	dentry = debugfs_create_dir("pogo_transport", NULL);

	if (IS_ERR(dentry)) {
		dev_err(pogo_transport->dev, "debugfs dentry failed: %ld", PTR_ERR(dentry));
		return;
	}

	debugfs_create_file("mock_hid_connected", 0644, dentry, pogo_transport,
			    &mock_hid_connected_fops);
}
#endif

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

static int init_pogo_irqs(struct pogo_transport *pogo_transport)
{
	int ret;

	/* initialize pogo status irq */
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
		goto free_status_irq;
	}

	if (!pogo_transport->pogo_acc_gpio)
		return 0;

	/* initialize pogo accessory irq */
	pogo_transport->pogo_acc_irq = gpio_to_irq(pogo_transport->pogo_acc_gpio);
	if (pogo_transport->pogo_acc_irq <= 0) {
		dev_err(pogo_transport->dev, "Pogo acc irq not found\n");
		ret = -ENODEV;
		goto disable_status_irq_wake;
	}

	ret = devm_request_threaded_irq(pogo_transport->dev, pogo_transport->pogo_acc_irq,
					pogo_acc_isr, pogo_acc_irq,
					(IRQF_SHARED | IRQF_ONESHOT |
					 IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING),
					dev_name(pogo_transport->dev), pogo_transport);
	if (ret < 0) {
		dev_err(pogo_transport->dev, "pogo-acc-detect request irq failed ret:%d\n", ret);
		goto disable_status_irq_wake;
	}

	pogo_transport->acc_irq_enabled = true;

	ret = enable_irq_wake(pogo_transport->pogo_acc_irq);
	if (ret) {
		dev_err(pogo_transport->dev, "Enable acc irq wake failed ret:%d\n", ret);
		goto free_acc_irq;
	}

	return 0;

free_acc_irq:
	devm_free_irq(pogo_transport->dev, pogo_transport->pogo_acc_irq, pogo_transport);
disable_status_irq_wake:
	disable_irq_wake(pogo_transport->pogo_irq);
free_status_irq:
	devm_free_irq(pogo_transport->dev, pogo_transport->pogo_irq, pogo_transport);

	return ret;
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

	return 0;
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
	int ret;

	/* initialize pogo status gpio */
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

	/* initialize data mux gpio */
	pogo_transport->pogo_data_mux_gpio = of_get_named_gpio(pogo_transport->dev->of_node,
							       "pogo-transport-sel", 0);
	if (pogo_transport->pogo_data_mux_gpio < 0) {
		dev_err(pogo_transport->dev, "Pogo sel gpio not found ret:%d\n",
			pogo_transport->pogo_data_mux_gpio);
		return pogo_transport->pogo_data_mux_gpio;
	}

	ret = devm_gpio_request(pogo_transport->dev, pogo_transport->pogo_data_mux_gpio,
				"pogo-transport-sel");
	if (ret) {
		dev_err(pogo_transport->dev, "failed to request pogo-transport-sel gpio, ret:%d\n",
			ret);
		return ret;
	}

	ret = gpio_direction_output(pogo_transport->pogo_data_mux_gpio, 0);
	if (ret) {
		dev_err(pogo_transport->dev, "failed set pogo-transport-sel as output, ret:%d\n",
			ret);
		return ret;
	}

	/* pinctrl for usb-c path*/
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

	/* pinctrl for pogo path */
	pogo_transport->susp_pogo_state = pinctrl_lookup_state(pogo_transport->pinctrl,
							       "suspend-to-pogo");
	if (IS_ERR(pogo_transport->susp_pogo_state)) {
		dev_err(pogo_transport->dev, "failed to find pinctrl suspend-to-pogo ret:%ld\n",
			PTR_ERR(pogo_transport->susp_pogo_state));
		return PTR_ERR(pogo_transport->susp_pogo_state);
	}

	return 0;
}

static int init_pogo_ovp_gpio(struct pogo_transport *pogo_transport)
{
	enum of_gpio_flags flags;
	int ret;

	if (!of_property_read_bool(pogo_transport->dev->of_node, "pogo-ovp-en")) {
		pogo_transport->pogo_ovp_en_gpio = -EINVAL;
		return 0;
	}

	pogo_transport->pogo_ovp_en_gpio = of_get_named_gpio_flags(pogo_transport->dev->of_node,
								   "pogo-ovp-en", 0, &flags);
	if (pogo_transport->pogo_ovp_en_gpio < 0) {
		dev_err(pogo_transport->dev, "Pogo ovp en gpio not found. ret:%d\n",
			pogo_transport->pogo_ovp_en_gpio);
		return pogo_transport->pogo_ovp_en_gpio;
	}

	pogo_transport->pogo_ovp_en_active_state = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;

	ret = devm_gpio_request(pogo_transport->dev, pogo_transport->pogo_ovp_en_gpio,
				"pogo-ovp-en");
	if (ret) {
		dev_err(pogo_transport->dev, "failed to request pogo-ovp-en gpio, ret:%d\n", ret);
		return ret;
	}

	/* Default disable pogo ovp. Set to disable state for pogo_ovp_en */
	ret = gpio_direction_output(pogo_transport->pogo_ovp_en_gpio,
				    !pogo_transport->pogo_ovp_en_active_state);
	if (ret) {
		dev_err(pogo_transport->dev, "failed set pogo-ovp-en as output, ret:%d\n", ret);
		return ret;
	}

	return 0;
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

	spin_lock_init(&pogo_transport->pogo_event_lock);

	pogo_transport->wq = kthread_create_worker(0, "wq-pogo-transport");
	if (IS_ERR_OR_NULL(pogo_transport->wq)) {
		ret = PTR_ERR(pogo_transport->wq);
		goto unreg_logbuffer;
	}

	kthread_init_delayed_work(&pogo_transport->pogo_accessory_debounce_work,
				  process_debounce_event);
	kthread_init_delayed_work(&pogo_transport->state_machine,
				  pogo_transport_state_machine_work);
	kthread_init_work(&pogo_transport->event_work, pogo_transport_event_handler);

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

	pogo_transport->charger_mode_votable = gvotable_election_get_handle(GBMS_MODE_VOTABLE);
	if (IS_ERR_OR_NULL(pogo_transport->charger_mode_votable)) {
		dev_err(pogo_transport->dev, "GBMS_MODE_VOTABLE get failed %ld\n",
			PTR_ERR(pogo_transport->charger_mode_votable));
		ret = -EPROBE_DEFER;
		goto psy_put;
	}

	pogo_transport->equal_priority = of_property_read_bool(pogo_transport->dev->of_node,
							       "equal-priority");

	ret = init_pogo_ovp_gpio(pogo_transport);
	if (ret) {
		dev_err(pogo_transport->dev, "init_pogo_ovp_gpio error:%d\n", ret);
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
			goto psy_put;
	}

	/*
	 * modparam_state_machine_enable
	 * 0 or unset: If property "legacy-event-driven" is found in device tree, disable the state
	 *	       machine. Otherwise, enable/disable the state machine based on
	 *	       DEFAULT_STATE_MACHINE_ENABLE.
	 * 1: Enable the state machine
	 * 2: Disable the state machine
	 */
	if (modparam_state_machine_enable == 1) {
		pogo_transport->state_machine_enabled = true;
	} else if (modparam_state_machine_enable == 2) {
		pogo_transport->state_machine_enabled = false;
	} else {
		if (of_property_read_bool(pogo_transport->dev->of_node, "legacy-event-driven"))
			pogo_transport->state_machine_enabled = false;
		else
			pogo_transport->state_machine_enabled = DEFAULT_STATE_MACHINE_ENABLE;
	}

	if (pogo_transport->state_machine_enabled)
		pogo_transport_set_state(pogo_transport, STANDBY, 0);

	if (modparam_pogo_accessory_enable) {
		ret = init_acc_gpio(pogo_transport);
		if (ret)
			goto psy_put;
		pogo_transport->accessory_detection_enabled = modparam_pogo_accessory_enable;
	} else if (of_property_read_bool(dn, "pogo-acc-capable") ||
		   of_property_read_bool(dn, "pogo-acc-hall-only")) {
		ret = init_acc_gpio(pogo_transport);
		if (ret)
			goto psy_put;
		if (of_property_read_bool(dn, "pogo-acc-capable"))
			pogo_transport->accessory_detection_enabled = ENABLED;
		else
			pogo_transport->accessory_detection_enabled = HALL_ONLY;
	}

	pogo_transport->disable_voltage_detection =
		of_property_read_bool(dn, "disable-voltage-detection");

	ret = init_pogo_irqs(pogo_transport);
	if (ret) {
		dev_err(pogo_transport->dev, "init_pogo_irqs error:%d\n", ret);
		goto psy_put;
	}

#if IS_ENABLED(CONFIG_DEBUG_FS)
	pogo_transport_init_debugfs(pogo_transport);
#endif

	register_data_active_callback(data_active_changed, pogo_transport);
	register_orientation_callback(orientation_changed, pogo_transport);
	pogo_transport->udev_nb.notifier_call = pogo_transport_udev_notify;
	usb_register_notify(&pogo_transport->udev_nb);
	/* run once in case orientation has changed before registering the callback */
	orientation_changed((void *)pogo_transport);
	dev_info(&pdev->dev, "force usb:%d\n", modparam_force_usb ? 1 : 0);
	dev_info(&pdev->dev, "state machine:%u\n", pogo_transport->state_machine_enabled);
	put_device(&data_client->dev);
	of_node_put(data_np);
	return 0;

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
	struct dentry *dentry;
	int ret;

	usb_unregister_notify(&pogo_transport->udev_nb);

#if IS_ENABLED(CONFIG_DEBUG_FS)
	dentry = debugfs_lookup("pogo_transport", NULL);
	if (IS_ERR(dentry)) {
		dev_err(pogo_transport->dev, "%s: Failed to lookup debugfs dir\n", __func__);
	} else {
		debugfs_remove(dentry);
		dput(dentry);
	}
#endif

	if (pogo_transport->hub_ldo && regulator_is_enabled(pogo_transport->hub_ldo) > 0)
		regulator_disable(pogo_transport->hub_ldo);

	ret = pogo_transport_acc_regulator(pogo_transport, false);
	if (ret)
		dev_err(pogo_transport->dev, "%s: Failed to disable acc ldo %d\n", __func__, ret);

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

/*-------------------------------------------------------------------------*/
/* Event triggering part.2                                                 */
/*-------------------------------------------------------------------------*/

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

	if (pogo_transport->state_machine_enabled)
		pogo_transport_queue_event(pogo_transport, EVENT_ENABLE_USB_DATA);
	else
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
	if (force_pogo && !pogo_transport->state_machine_enabled)
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
	u8 enable_hub;

	if (pogo_transport->state_machine_enabled) {
		logbuffer_log(pogo_transport->log, "state machine enabled; ignore enable_hub");
		return size;
	}

	if (!pogo_transport->hub_embedded)
		return size;

	if (kstrtou8(buf, 0, &enable_hub))
		return -EINVAL;

	if (pogo_transport->pogo_hub_active == !!enable_hub)
		return size;

	/*
	 * KEEP_HUB_PATH is only for engineering tests where the embedded hub remains enabled after
	 * undocking.
	 */
	if (enable_hub == KEEP_HUB_PATH)
		pogo_transport->force_hub_enabled = true;
	else
		pogo_transport->force_hub_enabled = false;

	dev_info(pogo_transport->dev, "hub %u, force_hub_enabled %u\n", enable_hub,
		 pogo_transport->force_hub_enabled);
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

	if (!pogo_transport->accessory_detection_enabled) {
		logbuffer_logk(pogo_transport->log, LOGLEVEL_INFO, "%s:Accessory detection disabled",
			       __func__);
		return size;
	}

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

	logbuffer_log(pogo_transport->log, "H1S: accessory detection %u, mfg %u", enable_acc_detect,
		      pogo_transport->mfg_acc_test);

	if (pogo_transport->state_machine_enabled) {
		pogo_transport_queue_event(pogo_transport, EVENT_HES_H1S_CHANGED);
		return size;
	}

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
	struct pogo_transport *pogo_transport = dev_get_drvdata(dev);
	u8 data;

	/* Reserved for HES1 Malfunction detection */

	if (kstrtou8(buf, 0, &data))
		return -EINVAL;

	logbuffer_log(pogo_transport->log, "H1N: %u", data);
	return size;
}
static DEVICE_ATTR_WO(hall1_n);

static ssize_t hall2_s_store(struct device *dev, struct device_attribute *attr, const char *buf,
			     size_t size)
{
	struct pogo_transport *pogo_transport = dev_get_drvdata(dev);
	u8 data;

	/* Reserved for keyboard status detection */

	if (kstrtou8(buf, 0, &data))
		return -EINVAL;

	logbuffer_log(pogo_transport->log, "H2S: %u", data);
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
