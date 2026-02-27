/*
 * Copyright (c) 2025 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microcrystal_rv3032_counter

/** @file
 * @brief Microcrystal RV-3032 counter driver.
 *
 * This driver exposes the RV-3032's periodic countdown timer through the 
 * Counter API. Interrupts are configured and delegated via the parent MFD 
 * driver.
 *
 * Remarks:
 * - The duration of the first countdown after (re)starting the timer can be
 *   off by ~1 tick. See 4.8.3. FIRST PERIOD DURATION in the data sheet. This
 *   applies each time the timer is enabled (CONTROL1 TE bit set) or when a new
 *   top value is written. This means counter alarms will ALWAYS be off by up to
 *   ~1 tick.
 * - The RV-3032 does not support reading the current value/ticks of the 
 *   periodic countdown timer.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/spinlock.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/mfd/rv3032.h>

LOG_MODULE_REGISTER(rv3032_counter, CONFIG_COUNTER_LOG_LEVEL);

#define RV3032_CONTROL1_TD_4096 (RV3032_CONTROL1_TD & 0x0)
#define RV3032_CONTROL1_TD_64   (RV3032_CONTROL1_TD & 0x1)
#define RV3032_CONTROL1_TD_1    (RV3032_CONTROL1_TD & 0x2)
#define RV3032_CONTROL1_TD_1_60 (RV3032_CONTROL1_TD & 0x3)

struct rv3032_counter_config {
	struct counter_config_info counter_info;
	const struct device *mfd;
};

struct rv3032_counter_data {
	bool counter_is_enabled;
	uint16_t top_value;

	struct k_sem lock;

	bool alarm_is_pending;
	void *callback;
	void *user_data;
};

void rv3032_counter_lock_sem(const struct device *dev)
{
	struct rv3032_counter_data* data = dev->data;
	(void)k_sem_take(&data->lock, K_FOREVER);
}

void rv3032_counter_unlock_sem(const struct device *dev)
{
	struct rv3032_counter_data* data = dev->data;
	(void)k_sem_give(&data->lock);
}

int rv3032_counter_start(const struct device *dev)
{
	const struct rv3032_counter_config *config = dev->config;
	struct rv3032_counter_data *data = dev->data;
	
	rv3032_counter_lock_sem(dev);

	int err = mfd_rv3032_update_reg8(config->mfd, RV3032_REG_CONTROL1, RV3032_CONTROL1_TE,
					 RV3032_CONTROL1_TE);
	if (err) {
		goto unlock;
	}
	data->counter_is_enabled = true;

unlock:
	rv3032_counter_unlock_sem(dev);

	return err;
}

int rv3032_counter_stop(const struct device *dev)
{
	const struct rv3032_counter_config *config = dev->config;
	struct rv3032_counter_data *data = dev->data;

	rv3032_counter_lock_sem(dev);

	int err = mfd_rv3032_update_reg8(config->mfd, RV3032_REG_CONTROL1, RV3032_CONTROL1_TE, 0);
	if (err) {
		goto unlock;
	}
	data->counter_is_enabled = false;

unlock:
	rv3032_counter_unlock_sem(dev);

	return err;
}

/* The RV-3032 does not support reading the current value of the periodic
 * countdown timer.
 */
int rv3032_counter_get_value(const struct device *dev, uint32_t *ticks)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(ticks);
	return -ENOTSUP;
}

/* The RV-3032's periodic countdown timer restarts whenever a value is 
 * written to a Timer Value register, even if it is unchanged. So by rewriting 
 * the current value of the Timer Value 0 register, we can restart the timer in 
 * a single I2C write, rather than having to disable then enable the timer. 
 */
int rv3032_counter_reset(const struct device *dev)
{
	const struct rv3032_counter_config *config = dev->config;
	const struct rv3032_counter_data *data = dev->data;
	int err;

	rv3032_counter_lock_sem(dev);

	/* If the countdown timer is disabled, it will reset itself upon being 
	 * enabled. 
	 */
	if (!data->counter_is_enabled) {
		err = 0;
		goto unlock;
	}

	uint8_t timer_value_0 = data->top_value & 0xff;
	err = mfd_rv3032_write_regs(config->mfd, RV3032_REG_TIMER_VALUE_0, &timer_value_0, 
				    sizeof(timer_value_0));
unlock:
	rv3032_counter_unlock_sem(dev);

	return err;
}

void rv3032_counter_isr(const struct device *dev)
{
	const struct rv3032_counter_config *config = dev->config;
	struct rv3032_counter_data *data = dev->data;

	rv3032_counter_lock_sem(dev);

	const bool is_alarm = data->alarm_is_pending;
	const void *callback = data->callback;
	void *user_data = data->user_data;
	int err;
	
	if (is_alarm) {
		data->alarm_is_pending = false;
		data->callback = NULL;
		data->user_data = NULL;

		/* Stop the counter (alarms should be one shot) */
		err = mfd_rv3032_update_reg8(config->mfd, RV3032_REG_CONTROL1, 
			                     RV3032_CONTROL1_TE, 0);
		if (err) {
			LOG_ERR("Error stopping counter in counter alarm ISR : %d", err);
		} else {
			data->counter_is_enabled = false;
		}
	}

	rv3032_counter_unlock_sem(dev);

	if (!callback) {
		return;
	}

	if (is_alarm) {
		((counter_alarm_callback_t) callback)(dev, 0, 0, user_data);
	} else {
		((counter_top_callback_t) callback)(dev, user_data);
	}
}

int rv3032_counter_set_alarm(const struct device *dev, uint8_t chan_id,
			     const struct counter_alarm_cfg *alarm_cfg)
{
	const struct rv3032_counter_config *config = dev->config;
	struct rv3032_counter_data *data = dev->data;
	bool counter_was_running = false;
	int err;
	uint8_t time_val[2];

	/* Don't need to check channel ID, this is handled by the Counter API */

	if (alarm_cfg->ticks > config->counter_info.max_top_value) {
		LOG_ERR("alarm_cfg->ticks is %d, max value is %d", 
			alarm_cfg->ticks, config->counter_info.max_top_value);
		return -EINVAL;
	}

	if (alarm_cfg->flags & (COUNTER_ALARM_CFG_ABSOLUTE | COUNTER_ALARM_CFG_EXPIRE_WHEN_LATE)) {
		LOG_ERR("Unsupported alarm_cfg->flags: 0x%X (absolute alarms / expire when late"
			" are not supported, use relative alarms)", 
			alarm_cfg->flags);
		return -ENOTSUP;
	}

	rv3032_counter_lock_sem(dev);

	if (data->alarm_is_pending) {
		err = -EBUSY;
		goto unlock;
	}

	/* Stop the countdown timer if it is running, otherwise the alarm may 
	 * trigger based on the current top value. We'll restart it after 
	 * setting the new top value.
	 */
	if (data->counter_is_enabled) {
		err = rv3032_counter_stop(dev);
		if (err) {
			LOG_ERR("Failed to pause counter : %d", err);
			goto unlock;
		}
		counter_was_running = true;
	}

	time_val[0] = alarm_cfg->ticks & 0xff;
	time_val[1] = (alarm_cfg->ticks & 0xf00) >> 8;

	err = mfd_rv3032_write_regs(config->mfd, RV3032_REG_TIMER_VALUE_0, time_val,
				    sizeof(time_val));
	if (err) {
		LOG_ERR("TIMER register write failed : %d", err);
		goto unlock;
	}

	data->alarm_is_pending = true;
	data->callback = alarm_cfg->callback;
	data->user_data = alarm_cfg->user_data;
	data->top_value = alarm_cfg->ticks;

	/* Clear Timer Flag from status if there was something leftover */
	err = mfd_rv3032_update_reg8(config->mfd, RV3032_REG_STATUS, RV3032_STATUS_TF, 0);
	if (err) {
		LOG_ERR("Status register update failed : %d", err);
		goto unlock;
	}

	if (counter_was_running) {
		err = rv3032_counter_start(dev);
		if (err) {
			LOG_ERR("Failed to restart counter : %d", err);
			goto unlock;
		}
	}

unlock:
	rv3032_counter_unlock_sem(dev);

	return err;
}

int rv3032_counter_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
	struct rv3032_counter_data *data = dev->data;
	int err;

	/* Don't need to check channel ID, this is handled by the Counter API */

	rv3032_counter_lock_sem(dev);
	
	if (!data->alarm_is_pending) {
		err = 0;
		goto unlock;
	}

	err = rv3032_counter_stop(dev);
	if (err) {
		LOG_ERR("Failed to stop counter while cancelling alarm : %d", err);
		goto unlock;
	}

	data->alarm_is_pending = false;
	data->callback = NULL;
	data->user_data = NULL;

unlock:
	rv3032_counter_unlock_sem(dev);

	return err;
}

/* The Counter API does not specify a return value for errors, so UINT32_MAX is
 * used for errors here.
 */
uint32_t rv3032_counter_get_pending_int(const struct device *dev)
{
	const struct rv3032_counter_config *config = dev->config;
	uint8_t status;
	int err;

	err = mfd_rv3032_read_reg8(config->mfd, RV3032_REG_STATUS, &status);
	if (err) {
		LOG_ERR("Status register read failed : %d", err);
		return UINT32_MAX;
	}

	return (status & RV3032_STATUS_TF) > 0;
}

int rv3032_counter_set_top_value(const struct device *dev, const struct counter_top_cfg *cfg)
{
	const struct rv3032_counter_config *config = dev->config;
	struct rv3032_counter_data *data = dev->data;
	int err;

	/* Don't need to check cfg->ticks, this is handled by the Counter API */

	/* The RV-3032 restarts the countdown automatically when a new top value
	 * is written, so these flags are unsupported.
	 */
	if (cfg->flags & (COUNTER_TOP_CFG_RESET_WHEN_LATE | COUNTER_TOP_CFG_DONT_RESET)) {
		LOG_ERR("Unsupported cfg->flags: 0x%X (COUNTER_TOP_CFG_RESET_WHEN_LATE and"
			" COUNTER_TOP_CFG_DONT_RESET are not supported)", cfg->flags);
		return -ENOTSUP;
	}

	rv3032_counter_lock_sem(dev);

	if (data->alarm_is_pending) {
		err = -EBUSY;
		goto unlock;
	}
	
	uint8_t time_val[2] = {cfg->ticks & 0xff, (cfg->ticks >> 8) & 0xf};
	err = mfd_rv3032_write_regs(config->mfd, RV3032_REG_TIMER_VALUE_0, time_val, 
				    sizeof(time_val));
	if (err) {
		LOG_ERR("TIMER register write failed : %d", err);
		goto unlock;
	}

	data->callback = cfg->callback;
	data->user_data = cfg->user_data;
	data->top_value = cfg->ticks;

unlock:
	rv3032_counter_unlock_sem(dev);

	return err;
}

uint32_t rv3032_counter_get_top_value(const struct device *dev)
{
	const struct rv3032_counter_data *data = dev->data;
	return data->top_value;
}

static int rv3032_counter_init(const struct device *dev)
{
	const struct rv3032_counter_config *config = dev->config;
	struct rv3032_counter_data *data = dev->data;
	uint8_t freq_config;
	int err;

	if (!device_is_ready(config->mfd)) {
		return -ENODEV;
	}

	LOG_DBG("Counter [%s] mfd-parent [%s]\n", dev->name, config->mfd->name);

	/* Parent MFD driver clears status / control registers */

	switch (config->counter_info.freq) {
		case 4096: 
			freq_config = RV3032_CONTROL1_TD_4096;
			break;
		case 64:
			freq_config = RV3032_CONTROL1_TD_64;
			break;
		case 1: 
			freq_config = RV3032_CONTROL1_TD_1;
			break;
		case 0:
			freq_config = RV3032_CONTROL1_TD_1_60;
			break;
		default:
			LOG_ERR("Invalid counter frequency : %d", config->counter_info.freq);
			return -ENODEV;
	}

	/* Set the periodic countdown timer's clock frequency */
	err = mfd_rv3032_update_reg8(config->mfd, RV3032_REG_CONTROL1, RV3032_CONTROL1_TD,
				     freq_config);
	if (err) {
		LOG_ERR("Failed to set clock frequency : %d", err);
		return -ENODEV;
	}

	/* Enable the periodic countdown timer to generate interrupt signals */
	err = mfd_rv3032_update_reg8(config->mfd, RV3032_REG_CONTROL2, RV3032_CONTROL2_TIE, 
				     RV3032_CONTROL2_TIE);
	if (err) {
		LOG_ERR("Failed to enable interrupt signals : %d", err);
		return err;
	}

	k_sem_init(&data->lock, 1, 1);

	mfd_rv3032_set_irq_handler(config->mfd, dev, RV3032_DEV_COUNTER, rv3032_counter_isr);

	return 0;
}

static DEVICE_API(counter, rv3032_counter_api) = {
	.start = rv3032_counter_start,
	.stop = rv3032_counter_stop,
	.get_value = rv3032_counter_get_value,
	.reset = rv3032_counter_reset,
	.set_alarm = rv3032_counter_set_alarm,
	.cancel_alarm = rv3032_counter_cancel_alarm,
	.set_top_value = rv3032_counter_set_top_value,
	.get_pending_int = rv3032_counter_get_pending_int,
	.get_top_value = rv3032_counter_get_top_value,
};

#define RV3032_COUNTER_INIT(inst)                                                                  \
	static const struct rv3032_counter_config rv3032_counter_config_##inst = {                 \
		.counter_info = {                                                                  \
			.max_top_value = 4095,                                                     \
			.freq = DT_INST_PROP_OR(inst, frequency, 4096),                            \
			.flags = 0,                                     \
			.channels = 1,                                                             \
		},                                                                                 \
		.mfd = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                        \
	};                                                                                         \
	static struct rv3032_counter_data rv3032_counter_data_##inst = {};                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(0, &rv3032_counter_init, NULL, &rv3032_counter_data_##inst,          \
			      &rv3032_counter_config_##inst, POST_KERNEL,                          \
			      CONFIG_COUNTER_MICROCRYSTAL_RV3032_INIT_PRIORITY,                    \
			      &rv3032_counter_api);

DT_INST_FOREACH_STATUS_OKAY(RV3032_COUNTER_INIT)
