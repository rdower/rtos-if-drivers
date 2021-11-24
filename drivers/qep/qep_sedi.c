/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <device.h>
#include <drivers/qep.h>
#include "driver/sedi_driver_qep.h"

#define DT_DRV_COMPAT intel_pse_qep

/* Macro to get the QEP controller instance. */
#define GET_CONTROLLER_INSTANCE(dev)		\
	(((const struct qep_sedi_config_info *)	\
	  dev->config)->instance)

#define GET_QEP_SEM(dev)			\
	(((const struct qep_sedi_config_info *)	\
	  dev->config)->qep_sem)

#define GET_QEP_MUTEX(dev)			\
	(((const struct qep_sedi_config_info *)	\
	  dev->config)->qep_mutex)

#define  QEP_IRQ_CONFIG_FUNC_DECL(n) \
	static void irq_config_qep_##n(const struct device *dev)

/* Setting configuration function. */
#define QEP_IRQ_CONFIG_FUNC_SET(n) \
	.irq_config_func = irq_config_qep_##n

/* Defining irq config function */
#define QEP_IRQ_CONFIG_FUNC_DEFINE(n)				 \
	static void irq_config_qep_##n(const struct device *dev) \
	{							 \
		ARG_UNUSED(dev);				 \
		IRQ_CONNECT(DT_INST_IRQN(n),			 \
			    DT_INST_IRQ(n, priority), qep_isr,	 \
			    DEVICE_GET(qep_##n), 0);		 \
		irq_enable(DT_INST_IRQN(n));			 \
	}

#define QEP_SEDI_DEVICE_INIT(n)						 \
	QEP_IRQ_CONFIG_FUNC_DECL(n);					 \
	static K_MUTEX_DEFINE(qep_mutex_##n);				 \
	static K_SEM_DEFINE(qep_sem_##n, 1, 1);				 \
	static const struct qep_sedi_config_info config_info_qep_##n = { \
		.instance = DT_INST_PROP(n, peripheral_id),		 \
		.qep_mutex = &qep_mutex_##n,				 \
		.qep_sem = &qep_sem_##n,				 \
		QEP_IRQ_CONFIG_FUNC_SET(n)				 \
	};								 \
									 \
	static struct qep_sedi_drv_data drv_data_qep##n;		 \
									 \
	DEVICE_DEFINE(qep_##n, DT_INST_LABEL(n),			 \
		      &qep_sedi_init,					 \
		      &qep_sedi_device_ctrl,				 \
		      &drv_data_qep##n, &config_info_qep_##n,		 \
		      POST_KERNEL,					 \
		      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &api);	 \
	QEP_IRQ_CONFIG_FUNC_DEFINE(n)

struct qep_sedi_config_info {
	sedi_qep_t instance;
	void (*irq_config_func)(const struct device *dev);
	struct k_sem *qep_sem;
	struct k_mutex *qep_mutex;
};

struct qep_sedi_drv_data {
	qep_callback_t user_cb;
	void *user_param;
#ifdef CONFIG_PM_DEVICE
	uint32_t device_power_state;
#endif
};

#ifdef CONFIG_PM_DEVICE

static int qep_suspend_device(const struct device *dev)
{
	const struct qep_sedi_config_info *config = dev->config;
	int ret;

	if (pm_device_is_busy(dev)) {
		return -EBUSY;
	}

	ret = sedi_qep_set_power(config->instance, SEDI_POWER_SUSPEND);
	if (ret != SEDI_DRIVER_OK) {
		return -EIO;
	}

	return ret;
}

static int qep_resume_device_from_suspend(const struct device *dev)
{
	const struct qep_sedi_config_info *config = dev->config;
	int ret;

	ret = sedi_qep_set_power(config->instance, SEDI_POWER_FULL);
	if (ret != SEDI_DRIVER_OK) {
		return -EIO;
	}

	pm_device_busy_clear(dev);
	return ret;
}

static int qep_set_device_low_power(const struct device *dev)
{
	const struct qep_sedi_config_info *config = dev->config;
	int ret;

	if (pm_device_is_busy(dev)) {
		return -EBUSY;
	}

	ret = sedi_qep_set_power(config->instance, SEDI_POWER_LOW);
	if (ret != SEDI_DRIVER_OK) {
		return -EIO;
	}

	return ret;
}

static int qep_set_device_force_suspend(const struct device *dev)
{
	const struct qep_sedi_config_info *config = dev->config;
	int ret;

	ret = sedi_qep_set_power(config->instance, SEDI_POWER_FORCE_SUSPEND);
	if (ret != SEDI_DRIVER_OK) {
		return -EIO;
	}
	return ret;
}

static int qep_sedi_device_ctrl(const struct device *dev,
				enum pm_device_action action)
{
	int ret = 0;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		ret = qep_suspend_device(dev);
		break;
	case PM_DEVICE_ACTION_RESUME:
		ret = qep_resume_device_from_suspend(dev);
		break;
	case PM_DEVICE_ACTION_LOW_POWER:
		ret = qep_set_device_low_power(dev);
		break;
	case PM_DEVICE_ACTION_FORCE_SUSPEND:
		ret = qep_set_device_force_suspend(dev);
		break;
	default:
		ret = -ENOTSUP;
	}

	return ret;
}
#endif /* !CONFIG_PM_DEVICE */


void qep_isr(struct device *dev)
{
	sedi_qep_t instance = GET_CONTROLLER_INSTANCE(dev);

	sedi_qep_int_handler(instance);
}

static void default_qep_callback(const void *param,
				 uint32_t event, uint32_t cap_len)
{
	struct device *dev =  (struct device *)(param);
	struct qep_sedi_drv_data *drv_data = dev->data;
	qep_event_t zeph_evt;
	bool flag = false;

	switch (event) {
	case SEDI_QEP_WDT_DETECTED:
		zeph_evt = QEP_EVENT_WDT_TIMEOUT;
		break;

	case SEDI_QEP_DIR_CHANGE:
		zeph_evt = QEP_EVENT_DIR_CHANGE;
		break;

	case SEDI_QEP_RST_UP:
		zeph_evt = QEP_EVENT_CTR_RST_UP;
		break;

	case SEDI_QEP_RST_DN:
		zeph_evt = QEP_EVENT_CTR_RST_DN;
		break;

	case SEDI_QEP_PH_ERR:
		zeph_evt = QEP_EVENT_PHASE_ERR;
		break;

	case SEDI_QEP_CAP_DONE:
		zeph_evt = QEP_EVENT_EDGE_CAP_DONE;
		flag = true;
		break;

	case SEDI_QEP_CAP_CANCELLED:
		zeph_evt = QEP_EVENT_EDGE_CAP_CANCELLED;
		flag = true;
		break;
	default:
		zeph_evt = QEP_EVENT_UNKNOWN;
	}

	if (drv_data->user_cb) {
		drv_data->user_cb(dev, drv_data->user_param, zeph_evt, cap_len);
	}

	if (flag) {
		drv_data->user_cb = NULL;
		drv_data->user_param = NULL;
		k_sem_give(GET_QEP_SEM(dev));
		pm_device_busy_clear(dev);
	}
}

static int qep_sedi_start_decode(const struct device *dev,
				 qep_callback_t cb, void *cb_param)
{
	struct qep_sedi_drv_data *drv_data = dev->data;
	sedi_qep_t instance = GET_CONTROLLER_INSTANCE(dev);
	int ret = 0;

	if (k_sem_take(GET_QEP_SEM(dev), K_NO_WAIT)) {
		return -EBUSY;
	}
	drv_data->user_cb = cb;
	drv_data->user_param = cb_param;
	if (sedi_qep_start_decoding(instance) != SEDI_DRIVER_OK) {
		k_sem_give(GET_QEP_SEM(dev));
		drv_data->user_cb = NULL;
		drv_data->user_param = NULL;
		ret = -EPERM;
		goto exit;
	}
	pm_device_busy_set(dev);

exit:
	return ret;
}

static int qep_sedi_stop_decode(const struct device *dev)
{
	sedi_qep_t instance = GET_CONTROLLER_INSTANCE(dev);
	struct qep_sedi_drv_data *drv_data = dev->data;
	int ret = 0;

	k_mutex_lock(GET_QEP_MUTEX(dev), K_FOREVER);
	if (sedi_qep_stop_decoding(instance) != SEDI_DRIVER_OK) {
		ret = -EPERM;
		goto exit;
	}
	drv_data->user_cb = NULL;
	drv_data->user_param = NULL;
	k_sem_give(GET_QEP_SEM(dev));
	pm_device_busy_clear(dev);

exit:
	k_mutex_unlock(GET_QEP_MUTEX(dev));
	return ret;
}

static int qep_sedi_get_direction(const struct device *dev,
				  uint32_t *p_direction)
{
	sedi_qep_dir_t qep_dir;
	int ret = 0;

	__ASSERT(p_direction != NULL, "");
	sedi_qep_t instance = GET_CONTROLLER_INSTANCE(dev);

	k_mutex_lock(GET_QEP_MUTEX(dev), K_FOREVER);
	if (sedi_qep_get_direction(instance, &qep_dir) != SEDI_DRIVER_OK) {
		ret = -EPERM;
		goto exit;
	}

	if (qep_dir == SEDI_QEP_DIR_CCW) {
		*p_direction = QEP_DIRECTION_COUNTER_CLOCKWISE;
	} else if (qep_dir == SEDI_QEP_DIR_CW) {
		*p_direction = QEP_DIRECTION_CLOCKWISE;
	} else if (qep_dir == SEDI_QEP_DIR_UNKNOWN) {
		*p_direction = QEP_DIRECTION_UNKNOWN;
	}
exit:
	k_mutex_unlock(GET_QEP_MUTEX(dev));
	return ret;
}

static int qep_sedi_get_position_count(const struct device *dev,
				       uint32_t *p_cur_cnt)
{
	int ret = 0;

	__ASSERT(p_cur_cnt != NULL, "");
	sedi_qep_t instance = GET_CONTROLLER_INSTANCE(dev);

	k_mutex_lock(GET_QEP_MUTEX(dev), K_FOREVER);
	if (sedi_qep_get_position(instance, (uint32_t *)p_cur_cnt)
	    != SEDI_DRIVER_OK) {
		ret = -EINVAL;
	}

	k_mutex_unlock(GET_QEP_MUTEX(dev));
	return ret;
}

static int qep_sedi_start_edge_capture(const struct device *dev,
				       uint64_t *buffer,
				       uint32_t count, qep_callback_t cb,
				       void *cb_param)
{
	__ASSERT(buffer != NULL, "");
	struct qep_sedi_drv_data *drv_data = dev->data;
	sedi_qep_t instance = GET_CONTROLLER_INSTANCE(dev);

	if (k_sem_take(GET_QEP_SEM(dev), K_NO_WAIT)) {
		return -EBUSY;
	}
	drv_data->user_cb = cb;
	drv_data->user_param = cb_param;

	if (sedi_qep_start_cap(instance, buffer, count) != SEDI_DRIVER_OK) {
		k_sem_give(GET_QEP_SEM(dev));
		drv_data->user_cb = NULL;
		drv_data->user_param = NULL;
		return -EPERM;
	}
	pm_device_busy_set(dev);
	return 0;
}

static int qep_sedi_stop_edge_capture(const struct device *dev)
{
	int ret = 0;
	sedi_qep_t instance = GET_CONTROLLER_INSTANCE(dev);

	k_mutex_lock(GET_QEP_MUTEX(dev), K_FOREVER);
	if (sedi_qep_stop_cap(instance) != SEDI_DRIVER_OK) {
		ret = -EPERM;
	}
	k_mutex_unlock(GET_QEP_MUTEX(dev));
	pm_device_busy_clear(dev);
	return ret;
}

static int qep_sedi_enable_event(const struct device *dev,
				 qep_event_t event)
{
	sedi_qep_t instance = GET_CONTROLLER_INSTANCE(dev);
	sedi_qep_event_t bsp_event;
	int ret = 0;

	switch (event) {
	case QEP_EVENT_WDT_TIMEOUT:
		bsp_event = SEDI_QEP_WDT_DETECTED;
		break;
	case QEP_EVENT_DIR_CHANGE:
		bsp_event = SEDI_QEP_DIR_CHANGE;
		break;
	case QEP_EVENT_CTR_RST_UP:
		bsp_event = SEDI_QEP_RST_UP;
		break;
	case QEP_EVENT_CTR_RST_DN:
		bsp_event = SEDI_QEP_RST_DN;
		break;
	case QEP_EVENT_PHASE_ERR:
		bsp_event = SEDI_QEP_PH_ERR;
		break;
	default:
		return -EINVAL;
	}

	k_mutex_lock(GET_QEP_MUTEX(dev), K_FOREVER);
	if (sedi_qep_enable_event(instance, bsp_event) != SEDI_DRIVER_OK) {
		ret = -EPERM;
	}

	k_mutex_unlock(GET_QEP_MUTEX(dev));
	return ret;
}

static int qep_sedi_disable_event(const struct device *dev,
				  qep_event_t event)
{
	sedi_qep_t instance = GET_CONTROLLER_INSTANCE(dev);
	sedi_qep_event_t bsp_event;
	int ret = 0;

	switch (event) {
	case QEP_EVENT_WDT_TIMEOUT:
		bsp_event = SEDI_QEP_WDT_DETECTED;
		break;
	case QEP_EVENT_DIR_CHANGE:
		bsp_event = SEDI_QEP_DIR_CHANGE;
		break;
	case QEP_EVENT_CTR_RST_UP:
		bsp_event = SEDI_QEP_RST_UP;
		break;
	case QEP_EVENT_CTR_RST_DN:
		bsp_event = SEDI_QEP_RST_DN;
		break;
	case QEP_EVENT_PHASE_ERR:
		bsp_event = SEDI_QEP_PH_ERR;
		break;

	default:
		return -EINVAL;
	}

	k_mutex_lock(GET_QEP_MUTEX(dev), K_FOREVER);
	if (sedi_qep_disable_event(instance, bsp_event) != SEDI_DRIVER_OK) {
		ret = -EPERM;
	}
	k_mutex_unlock(GET_QEP_MUTEX(dev));
	return ret;
}

static int qep_sedi_get_phase_err_status(const struct device *dev,
					 uint32_t *p_ph_err)
{
	__ASSERT(p_ph_err != NULL, "");
	sedi_qep_t instance = GET_CONTROLLER_INSTANCE(dev);
	int ret = 0;

	k_mutex_lock(GET_QEP_MUTEX(dev), K_FOREVER);
	if (sedi_qep_get_phase_err(instance, (uint32_t *)p_ph_err)
	    != SEDI_DRIVER_OK) {
		ret = -EPERM;
	}
	k_mutex_unlock(GET_QEP_MUTEX(dev));
	return ret;
}

static int qep_sedi_config_device(const struct device *dev,
				  struct qep_config *config)
{
	__ASSERT(config != NULL, "");
	sedi_qep_t instance = GET_CONTROLLER_INSTANCE(dev);
	sedi_qep_config_t cfg = { 0 };
	int ret = 0;

	if (config->mode == QEP_MODE_QEP_DECODER) {
		cfg.mode = SEDI_QEP_MODE_QEP_DECODER;
	} else {
		cfg.mode = SEDI_QEP_MODE_CAPTURE;
	}

	cfg.swap_a_b = config->swap_a_b_input;

	if (config->edge_type == QEP_EDGE_SELECT_RISING) {
		cfg.edge_phase_a = SEDI_QEP_RISING_EDGE;
		cfg.edge_phase_b = SEDI_QEP_RISING_EDGE;
		cfg.edge_index = SEDI_QEP_RISING_EDGE;
	} else {
		cfg.edge_phase_a = SEDI_QEP_FALLING_EDGE;
		cfg.edge_phase_b = SEDI_QEP_FALLING_EDGE;
		cfg.edge_index = SEDI_QEP_FALLING_EDGE;
	}
	cfg.noise_filter_width_ns = config->filter_width_ns;
	cfg.filter_en = config->filter_en;

	if (config->pos_ctr_rst == QEP_POS_CTR_RST_ON_IDX_EVT) {
		cfg.ctr_rst_mode = SEDI_QEP_COUNTER_RESET_ON_INDEX;

		switch (config->index_gating) {
		case QEP_PH_A_LOW_PH_B_LOW:
			cfg.gating_index = SEDI_QEP_PHASE_A_LOW_PHASE_B_LOW;
			break;
		case QEP_PH_A_LOW_PH_B_HIGH:
			cfg.gating_index = SEDI_QEP_PHASE_A_LOW_PHASE_B_HIGH;
			break;
		case QEP_PH_A_HIGH_PH_B_LOW:
			cfg.gating_index = SEDI_QEP_PHASE_A_HIGH_PHASE_B_LOW;
			break;
		case QEP_PH_A_HIGH_PH_B_HIGH:
			cfg.gating_index = SEDI_QEP_PHASE_A_HIGH_PHASE_B_HIGH;
			break;
		default:
			return -EINVAL;
		}
	} else {
		cfg.ctr_rst_mode = SEDI_QEP_COUNTER_RESET_ON_MAX_COUNT;
	}

	cfg.pulses_per_rev = config->pulses_per_rev;
	cfg.wdt_en = config->wdt_en;
	cfg.wdt_usec = config->wdt_timeout_us;

	if (config->cap_edges == QEP_EDGE_CAP_SINGLE) {
		cfg.capture_edge = SEDI_QEP_CAP_SINGLE_EDGE;
	} else {
		cfg.capture_edge = SEDI_QEP_CAP_BOTH_EDGE;
	}

	k_mutex_lock(GET_QEP_MUTEX(dev), K_FOREVER);
	if (sedi_qep_config(instance, &cfg) != SEDI_DRIVER_OK) {
		ret = -EINVAL;
		goto exit;
	}
	sedi_qep_register_callback(instance, default_qep_callback,
				   (void *)(dev));

exit:
	k_mutex_unlock(GET_QEP_MUTEX(dev));
	return ret;
}

static const struct qep_driver_api api = {
	.config_device = qep_sedi_config_device,
	.start_decode = qep_sedi_start_decode,
	.stop_decode = qep_sedi_stop_decode,
	.get_direction = qep_sedi_get_direction,
	.get_position_count = qep_sedi_get_position_count,
	.start_capture = qep_sedi_start_edge_capture,
	.stop_capture = qep_sedi_stop_edge_capture,
	.enable_event = qep_sedi_enable_event,
	.disable_event = qep_sedi_disable_event,
	.get_phase_err_status = qep_sedi_get_phase_err_status
};

static int qep_sedi_init(const struct device *dev)
{
	const struct qep_sedi_config_info *config = dev->config;
	sedi_qep_t instance = GET_CONTROLLER_INSTANCE(dev);
	int ret;

	sedi_qep_set_power(instance, SEDI_POWER_FULL);
	ret = sedi_qep_init(instance);
	if (ret != SEDI_DRIVER_OK) {
		return -ENODEV;
	}
	config->irq_config_func(dev);
	return 0;
}

DT_INST_FOREACH_STATUS_OKAY(QEP_SEDI_DEVICE_INIT)
