/*
 * SmartReflex algorithms implementations
 *
 * Copyright (C) 2013 Texas Instruments, Inc.
 * Thara Gopinath <thara@ti.com>
 *
 * Copyright (C) 2013 Texas Instruments, Inc.
 * Nishanth Menon <nm@ti.com>
 *
 * Copyright (C) 2013 Texas Instruments, Inc.
 * Andrii Tseglytskyi <andrii.tseglytskyi@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/io.h>
#include <linux/pm_qos.h>
#include <linux/power/smartreflex.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/omap-pmic-regulator.h>

#define SR_PERIODIC_SAMPLING_DELAY_MS	1
#define SR_PERIODIC_STABLE_SAMPLES	10
#define SR_PERIODIC_MAX_TRIGGERS	10

/*
 * We expect events in 10uS, if we don't receive it in twice as long,
 * we stop waiting for the event and use the current value
 */
#define SR_PERIODIC_MAX_CHECK_VPTRANS_US	20

/* TODO: mutex should be defined by OPP scaling function */
static DEFINE_MUTEX(dvfs_lock);

/**
 * struct sr_periodic_calib_data - data to be used by periodic work
 * @calib_work:		calibration work
 * @sr:			SmartReflex used for calibration
 * @volt_nominal:	Nominal voltage, which needs calibration
 * @num_calib_triggers:	number of triggers from calibration loop
 * @num_osc_samples:	number of samples collected by isr
 * @u_volt_samples:	private data for collecting voltage samples in
 *			case oscillations. filled by the notifier and
 *			consumed by the work item.
 * @work_active:	have we scheduled a work item?
 * @calib_qos:		calibration work pm_qos handle
 * @recal_work:		re-calibration work
 * @recal_qos:		re-calibration work pm_qos handle
 * @next_recal_time:	time left till next re-calibration
 */
struct sr_periodic_calib_data {
	/* periodic calibration data */
	struct delayed_work calib_work;
	struct omap_sr *sr;
	unsigned long volt_nominal;
	u8 num_calib_triggers;
	u8 num_osc_samples;
	unsigned long u_volt_samples[SR_PERIODIC_STABLE_SAMPLES];
	bool work_active;
	struct pm_qos_request calib_qos;

	/* periodic recalibration data */
	struct delayed_work recal_work;
	struct pm_qos_request recal_qos;
	unsigned long next_recal_time;
};

/**
 * struct sr_sw_loop_data - data to be used by SW loop
 * @scale_work:		work which scales voltage
 * @scale_qos:		scale work pm_qos handle
 * @sr:			SmartReflex used for SW calibration
 */
struct sr_sw_loop_data {
	struct work_struct scale_work;
	struct pm_qos_request scale_qos;
	struct omap_sr *sr;
};

/**
 * sr_hw_is_tranxdone() - checks VP tranxdone status
 * @sr:		SmartReflex linked with VP
 *
 * If HW loop is used for calibration, function returns
 * Voltage Processor tranxdone status.
 */
static bool sr_hw_is_tranxdone(struct omap_sr *sr)
{
	struct omap_pmic *pmic;

	pmic = regulator_get_drvdata(sr->reg_supply);
	if (!pmic) {
		pr_err("%s: %s: PMIC is not defined\n", __func__, sr->name);
		return false;
	}

	return omap_vp_check_tranxdone(pmic->v_dev);
}

/**
 * sr_hw_clear_tranxdone() - clears VP tranxdone status
 * @sr:		SmartReflex linked with VP
 *
 * If HW loop is used for calibration, function clears
 * Voltage Processor tranxdone status.
 */
static void sr_hw_clear_tranxdone(struct omap_sr *sr)
{
	struct omap_pmic *pmic;

	pmic = regulator_get_drvdata(sr->reg_supply);
	if (!pmic) {
		pr_err("%s: %s: PMIC is not defined\n", __func__, sr->name);
		return;
	}

	omap_vp_clear_tranxdone(pmic->v_dev);
}

/**
 * sr_hw_wait_tranxdone() - waits for VP tranxdone status
 * @sr:		SmartReflex linked with VP
 *
 * If HW loop is used for calibration, function waits
 * for Voltage Processor tranxdone status till
 * timeout, specified by SR_PERIODIC_MAX_CHECK_VPTRANS_US
 */
static void sr_hw_wait_tranxdone(struct omap_sr *sr)
{
	u32 idx = 0;

	do {
		if (sr_hw_is_tranxdone(sr))
			return;
		idx++;
		/* get some constant delay */
		udelay(1);
	} while (idx < SR_PERIODIC_MAX_CHECK_VPTRANS_US);

	pr_debug("%s: %s: wait HW tranxdone timeout\n",
		 __func__, sr->name);
}

/**
 * sr_hw_force_clear_tranxdone() - force clear for VP tranxdone status
 * @sr:		SmartReflex linked with VP
 *
 * If HW loop is used for calibration, function clears
 * Voltage Processor tranxdone status.
 */
static void sr_hw_force_clear_tranxdone(struct omap_sr *sr)
{
	u32 idx = 0;

	do {
		if (!sr_hw_is_tranxdone(sr))
			return;
		idx++;
		/* get some constant delay */
		udelay(1);
		sr_hw_clear_tranxdone(sr);
	} while (idx < SR_PERIODIC_MAX_CHECK_VPTRANS_US);

	pr_err("%s: %s: clear HW tranxdone timeout\n",
	       __func__, sr->name);
}

/**
 * sr_start_hw_loop() - starts Hardware calibration loop
 * @sr:		SmartReflex used for calibration
 * @volt:	Nominal voltage, which needs to be calibrated
 *
 * Callback. Returns 0 if HW loop started successfully,
 * or error code otherwise.
 */
static int sr_start_hw_loop(struct omap_sr *sr, unsigned long volt)
{
	struct omap_pmic *pmic;
	struct omap_sr_nvalue_table *nvalue_row;
	int ret;

	nvalue_row = sr_retrieve_nvalue_row(sr, volt);
	if (!nvalue_row)
		return -ENODATA;

	pmic = regulator_get_drvdata(sr->reg_supply);
	if (!pmic) {
		pr_err("%s: %s: PMIC is not defined\n",
		       __func__, sr->name);
		return -ENODATA;
	}

	/* VP errgain should be updated for each new voltage */
	ret = omap_vp_update_errgain(pmic->v_dev, nvalue_row->errgain);
	if (ret) {
		pr_err("%s: %s: Can't update VP errorgain\n",
		       __func__, sr->name);
		return -EINVAL;
	}

	ret = omap_vp_enable(pmic->v_dev, volt);
	if (ret) {
		pr_err("%s: %s: Can't enable VP\n", __func__, sr->name);
		return -EINVAL;
	}

	return sr_enable(sr, volt);
}

/**
 * sr_stop_hw_loop() - stops Hardware calibration loop
 * @sr:		SmartReflex used for calibration
 *
 * Callback.
 */
static int sr_stop_hw_loop(struct omap_sr *sr)
{
	struct omap_pmic *pmic;

	sr_disable_errgen(sr);

	pmic = regulator_get_drvdata(sr->reg_supply);
	if (!pmic)
		pr_err("%s: %s: PMIC is not defined\n", __func__, sr->name);
	else
		omap_vp_disable(pmic->v_dev);

	sr_disable(sr);

	return 0;
}

/**
 * sr_start_sw_loop() - starts Software calibration loop
 * @sr:		SmartReflex used for calibration
 * @volt:	Nominal voltage, which needs to be calibrated
 *
 * Callback. Returns 0 if SW loop started successfully,
 * or error code otherwise.
 */
static int sr_start_sw_loop(struct omap_sr *sr, unsigned long volt)
{
	int res;

	res = sr_enable(sr, volt);
	if (res) {
		pr_err("%s: %s: can't enable, error (%d)\n",
		       __func__, sr->name, res);
		return res;
	}

	/* Start sampling */
	res = sr_notifier_control(sr, true);
	return res;
}

/**
 * sr_stop_sw_loop() - stops Software calibration loop
 * @sr:		SmartReflex used for calibration
 *
 * Callback.
 */
static int sr_stop_sw_loop(struct omap_sr *sr)
{
	struct sr_sw_loop_data *loop_data = NULL;

	if (!sr) {
		pr_err("%s: SmartReflex pointer is NULL. Caller %pF\n",
		       __func__, (void *)_RET_IP_);
		return -EINVAL;
	}

	loop_data = (struct sr_sw_loop_data *)sr->loop_data;
	if (!loop_data) {
		pr_err("%s: %s bad SW loop data\n", __func__, sr->name);
		return -EINVAL;
	}

	/* Stop sampling */
	sr_notifier_control(sr, false);

	/*
	 * work handler may acces SR registers, therefore
	 * we cancel it before SR disable
	 */
	cancel_work_sync(&loop_data->scale_work);

	sr_disable(sr);

	pm_qos_update_request(&loop_data->scale_qos, PM_QOS_DEFAULT_VALUE);
	return 0;
}

/**
 * sr_set_pmic_voltage() - sets new voltage to PMIC
 * @sr:		SmartReflex linked with PMIC
 * @volt:	target voltage
 *
 * Returns 0, or error code.
 */
static int sr_set_pmic_voltage(struct omap_sr *sr, unsigned long volt)
{
	return regulator_set_voltage(sr->reg_supply, volt, volt);
}

/**
 * sr_get_pmic_voltage() - gets current PMIC voltage
 * @sr:		SmartReflex linked with PMIC
 *
 * Returns voltage in uV, or error code.
 */
static int sr_get_pmic_voltage(struct omap_sr *sr)
{
	struct omap_pmic *pmic;

	if (AVS_SW_LOOP == sr->calibration_loop)
		return regulator_get_voltage(sr->reg_supply);

	pmic = regulator_get_drvdata(sr->reg_supply);
	if (!pmic) {
		pr_err("%s: %s: PMIC is not defined\n",
		       __func__, sr->name);
		return sr->current_nominal_voltage;
	}

	return  omap_vp_get_voltage(pmic->v_dev);
}

/**
 * sr_reset_volt_to_nominal_hwloop() - resets PMIC voltage
 * @sr:		SmartReflex linked with PMIC
 *
 * Resets voltage to nominal, when HW loop is used for calibration.
 * Typically used when SmartReflex autocompletion is disabled.
 */
static int sr_reset_volt_to_nominal_hwloop(struct omap_sr *sr)
{
	int res;

	/*
	 * In case if HW loop is used for calibration, nominal voltage
	 * is stored as current voltage in regulator framework, but
	 * due to HW loop it is changed. During reset - it is needed
	 * to scale on it one more time.
	 */
	res = regulator_sync_voltage(sr->reg_supply);

	/*
	 * In case if there were no calls of regulator_set_voltage(pmic),
	 * previous call fails, this may occur when following conditions
	 * are true:
	 * - system doesn't scale OPP during boot
	 * - SmartReflex becomes enabled during boot
	 * - SmartReflex becomes disabled after boot (userspace call)
	 *
	 * So, it is good enough to scale to nominal voltage here.
	 */
	if (res)
		res = sr_set_pmic_voltage(sr, sr->current_nominal_voltage);

	return res;
}

/**
 * sr_reset_volt_to_nominal() - resets PMIC voltage
 * @sr:		SmartReflex linked with PMIC
 *
 * Resets voltage to nominal, when HW loop is not used for calibration.
 * Typically used when SmartReflex autocompletion is disabled.
 */
static int sr_reset_volt_to_nominal(struct omap_sr *sr)
{
	return sr_set_pmic_voltage(sr, sr->current_nominal_voltage);
}

/**
 * sr_reset_volt_periodic() - resets PMIC voltage
 * @sr:		SmartReflex linked with PMIC
 *
 * Resets voltage to dynamic nominal.
 * Typically used when SmartReflex autocompletion is disabled.
 */
static int sr_reset_volt_periodic(struct omap_sr *sr)
{
	struct omap_sr_nvalue_table *nvalue_row;
	unsigned long volt_reset;

	nvalue_row = sr_retrieve_nvalue_row(sr, sr->current_nominal_voltage);
	if (!nvalue_row)
		return -ENODATA;

	volt_reset = nvalue_row->volt_dynamic_nominal;
	if (!volt_reset) {
		/*
		 * this means that re-calibration work occures more frequently
		 * than calibration work. Vdyn is still not calibrated, but
		 * re-calibration work is scheduled again.
		 */
		pr_err("%s: %s: dynamic voltage is 0 for voltage %u\n",
		       __func__, sr->name, sr->current_nominal_voltage);
		volt_reset = nvalue_row->volt_nominal;
	}

	return sr_set_pmic_voltage(sr, volt_reset);
}

/**
 * sr_start_continuous_calibration() - starts continuous calibration
 * @sr:		SmartReflex used for calibration
 * @volt:	Nominal voltage, which needs to be calibrated
 *
 * Callback. Starts continuous calibration, which is always active
 * after eech scale to OPP. Returns 0, or error code in case of failure.
 */
static int sr_start_continuous_calibration(struct omap_sr *sr,
					   unsigned long volt)
{
	if (!sr) {
		pr_err("%s: SmartReflex pointer is NULL. Caller %pF\n",
		       __func__, (void *)_RET_IP_);
		return -EINVAL;
	}

	return sr->start_loop(sr, volt);
}

/**
 * sr_stop_continuous_calibration() - stops continuous calibration
 * @sr:		SmartReflex used for calibration
 *
 * Callback. Stops continuous calibration. Returns 0,
 * or error code in case of failure.
 */
static int sr_stop_continuous_calibration(struct omap_sr *sr)
{
	if (!sr) {
		pr_err("%s: SmartReflex pointer is NULL. Caller %pF\n",
		       __func__, (void *)_RET_IP_);
		return -EINVAL;
	}

	return sr->stop_loop(sr);
}

/**
 * sr_periodic_calibration_work_is_active() - helper function
 * @sr:		SmartReflex for which we check activity
 *
 * Checks is periodic calibration work is active and returns
 * true or false
 */
static bool sr_periodic_calibration_work_is_active(struct omap_sr *sr)
{
	struct sr_periodic_calib_data *calib_data;

	calib_data = (struct sr_periodic_calib_data *)sr->alg_data;
	if (!calib_data) {
		pr_err("%s: %s: bad calibration data\n", __func__, sr->name);
		return false;
	}

	return calib_data->work_active;
}

/**
 * sr_periodic_store_sample() - helper function
 * @sr:		SmartReflex used for calibration
 *
 * Stores voltage sample for following useage in periodical
 * calibration work.
 */
static void sr_periodic_store_sample(struct omap_sr *sr, u32 volt_sample)
{
	struct sr_periodic_calib_data *calib_data;
	u32 idx;

	if (AVS_CALIBRATION_PERIODIC != sr->calibration_period)
		return;

	calib_data = (struct sr_periodic_calib_data *)sr->alg_data;

	idx = (calib_data->num_osc_samples) % SR_PERIODIC_STABLE_SAMPLES;
	calib_data->u_volt_samples[idx] = volt_sample;
	calib_data->num_osc_samples++;
}

/**
 * sr_irq_hw_notify() - isr notifier for HW loop
 *
 * This collects voltage data for the calibration work.
 */
static int sr_irq_hw_notify(struct notifier_block *nb,
			 unsigned long action, void *data)
{
	struct sr_periodic_calib_data *calib_data = NULL;
	struct omap_sr *sr = (struct omap_sr *)data;

	if (!sr) {
		pr_err("%s: SmartReflex pointer is NULL. Caller %pF\n",
		       __func__, (void *)_RET_IP_);
		return -EINVAL;
	}

	calib_data = (struct sr_periodic_calib_data *)sr->alg_data;
	if (!calib_data) {
		pr_err("%s: %s bad calibration data\n", __func__, sr->name);
		return -EINVAL;
	}

	/*
	 * Wait for transdone so that we know the voltage to read.
	 *
	 * NOTE:
	 * If we timeout, we still read the data,
	 * if we are oscillating+irq latencies are too high, we could
	 * have scenarios where we miss transdone event. since
	 * we waited long enough, it is still safe to read the voltage
	 * as we would have waited long enough - Dont warn for this.
	 */
	sr_hw_wait_tranxdone(sr);

	sr_periodic_store_sample(sr, sr_get_pmic_voltage(sr));

	sr_hw_clear_tranxdone(sr);

	return 0;
}

static struct notifier_block sr_irq_hw_notifier = {
	.notifier_call	= sr_irq_hw_notify,
};

/**
 * sr_irq_sw_notify() - isr notifier for SW loop
 *
 * This collects voltage data for the calibration work.
 */
static int sr_irq_sw_notify(struct notifier_block *nb,
			    unsigned long action, void *data)
{
	struct omap_sr *sr = (struct omap_sr *)data;
	struct sr_sw_loop_data *loop_data = NULL;
	struct omap_sr_nvalue_table *nvalue_row;

	if (!sr) {
		pr_err("%s: SmartReflex pointer is NULL. Caller %pF\n",
		       __func__, (void *)_RET_IP_);
		return -EINVAL;
	}

	loop_data = (struct sr_sw_loop_data *)sr->loop_data;
	if (!loop_data) {
		pr_err("%s: %s bad SW loop data\n", __func__, sr->name);
		return -EINVAL;
	}

	nvalue_row = sr_retrieve_nvalue_row(sr, sr->current_nominal_voltage);
	if (!nvalue_row)
		return -ENODATA;

	nvalue_row->volt_error =
			sr_get_errvoltage(sr, nvalue_row->errgain);

	schedule_work(&loop_data->scale_work);

	return 0;
}

static struct notifier_block sr_irq_sw_notifier = {
	.notifier_call	= sr_irq_sw_notify,
};

/**
 * sr_start_periodic_calibration() - starts periodic calibration
 * @sr:		SmartReflex used for calibration
 * @volt:	Nominal voltage, which needs to be calibrated
 *
 * Callback. Starts calibration loop and schedules offline calibration.
 * returns 0, or error code in case of failure.
 */
static int sr_start_periodic_calibration(struct omap_sr *sr,
					 unsigned long volt)
{
	struct sr_periodic_calib_data *calib_data;
	int res;

	if (!sr) {
		pr_err("%s: SmartReflex pointer is NULL. Caller %pF\n",
		       __func__, (void *)_RET_IP_);
		return -EINVAL;
	}

	calib_data = (struct sr_periodic_calib_data *)sr->alg_data;
	if (!calib_data) {
		pr_err("%s: %s; bad calibration data\n", __func__, sr->name);
		return -EINVAL;
	}

	if (is_idle_task(current))
		return 0;

	/* We donot expect calib_work item to be active here */
	WARN_ON(sr_periodic_calibration_work_is_active(sr));

	/* init calibration data */
	calib_data->volt_nominal = volt;
	calib_data->work_active = true;
	calib_data->num_calib_triggers = 0;
	calib_data->num_osc_samples = 0;
	memset(calib_data->u_volt_samples, 0,
	       sizeof(calib_data->u_volt_samples));

	res = sr->start_loop(sr, volt);

	/*
	 * Calibrate Voltage on first switch to OPP, don't calibrate if called
	 * from system - wide suspend/resume path - it is not allowed to deal
	 * with pm_qos and calib_work scheduling in this case
	 */
	if (res)
		return res;

	/* Dont interrupt me until calibration is complete */
	pm_qos_update_request(&calib_data->calib_qos, 0);

	/* program the workqueue and leave it to calibrate offline.. */
	schedule_delayed_work(&calib_data->calib_work,
			      msecs_to_jiffies(SR_PERIODIC_SAMPLING_DELAY_MS *
					       SR_PERIODIC_STABLE_SAMPLES));
	return res;
}

/**
 * sr_stop_periodic_calibration() - stops periodic calibration
 * @sr:		SmartReflex used for calibration
 *
 * Callback. Stops offline calibration.
 * returns 0, or error code in case of failure.
 */
static int sr_stop_periodic_calibration(struct omap_sr *sr)
{
	struct sr_periodic_calib_data *calib_data;

	if (!sr) {
		pr_err("%s: SmartReflex pointer is NULL. Caller %pF\n",
		       __func__, (void *)_RET_IP_);
		return -EINVAL;
	}

	calib_data = (struct sr_periodic_calib_data *)sr->alg_data;
	if (!calib_data) {
		pr_err("%s: %s: bad work data\n", __func__, sr->name);
		return -EINVAL;
	}

	if (is_idle_task(current)) {
		/*
		 * we should not have seen this path if calibration !complete
		 * pm_qos constraint is already released after voltage
		 * calibration work is finished
		 */
		WARN_ON(sr_periodic_calibration_work_is_active(sr));

		return 0;
	}

	/* Force disable interrupt */
	sr_notifier_control(sr, false);

	/* stop calibration loop */
	sr->stop_loop(sr);

	cancel_delayed_work_sync(&calib_data->calib_work);
	calib_data->work_active = false;
	calib_data->num_osc_samples = 0;
	calib_data->num_calib_triggers = 0;

	/* Cancelled calibration, so no more need to keep request */
	pm_qos_update_request(&calib_data->calib_qos, PM_QOS_DEFAULT_VALUE);

	return 0;
}

/**
 * sr_get_dyn_nominal() - calculates dynamic voltage
 * @nvalue_row:		per-voltage data
 */
static inline unsigned long
sr_get_dyn_nominal(struct omap_sr_nvalue_table *nvalue_row)
{
#define OMAP3PLUS_DYNAMIC_NOMINAL_MARGIN_UV       50000
	if (!nvalue_row)
		return 0;

	if (nvalue_row->volt_calibrated) {
		unsigned long v = nvalue_row->volt_calibrated +
			OMAP3PLUS_DYNAMIC_NOMINAL_MARGIN_UV;
		if (v > nvalue_row->volt_nominal)
			return nvalue_row->volt_nominal;
		return v;
	}

	return nvalue_row->volt_nominal;
}

/**
 * sr_periodic_adjust_margin() - helper to map back margins
 * @sr:		SmartReflex instance
 * @volt_margin:	margin to add
 * @volt_current:	current voltage
 *
 * If PMIC APIs exist, then convert to PMIC step size adjusted voltage
 * corresponding to margin requested.
 */
static inline u32 sr_periodic_adjust_margin(struct omap_sr *sr,
					    u32 volt_margin, u32 volt_current)
{
	struct omap_pmic *pmic;
	u32 volt;
	u8 vsel;
	int ret;

	pmic = regulator_get_drvdata(sr->reg_supply);
	if (!pmic) {
		pr_err("%s: %s: PMIC is not defined\n", __func__, sr->name);
		return 0;
	}

	if (pmic->ops->vsel_to_uv && pmic->ops->uv_to_vsel) {
		/*
		 * To ensure conversion works:
		 * use a proper base voltage - we use the current volt
		 * then convert it with pmic routine to vsel and back
		 * to voltage, and finally remove the base voltage
		 */
		volt = volt_margin + volt_current;
		ret = pmic->ops->uv_to_vsel(pmic, volt, &vsel);
		if (ret)
			return 0;

		ret = pmic->ops->vsel_to_uv(pmic, vsel, &volt);
		if (ret)
			return 0;

		volt -= volt_current;
	}
	return volt;
}

/**
 * sr_periodic_find_safe_voltage() - helper function
 * @calib_data:		Periodic calibration data
 * @volt_current:	Current PMIC voltage
 *
 * Returns safe voltage, taking in account osc samples,
 * received in isr.
 */
unsigned long sr_periodic_find_safe_voltage(
			struct sr_periodic_calib_data *calib_data,
			unsigned long volt_current)
{
	unsigned long volt_safe;
	u8 count, idx;

	/* Just in case we got more interrupts than our tiny buffer */
	if (calib_data->num_osc_samples > SR_PERIODIC_STABLE_SAMPLES)
		count = SR_PERIODIC_STABLE_SAMPLES;
	else
		count = calib_data->num_osc_samples;

	volt_safe = volt_current;
	/* Grab the max of the samples as the stable voltage */
	for (idx = 0; idx < count; idx++) {
		pr_debug("%s: %s: osc_v[%d]=%ld, safe_v=%ld\n", __func__,
			 calib_data->sr->name, idx,
			 calib_data->u_volt_samples[idx], volt_safe);
		if (calib_data->u_volt_samples[idx] > volt_safe)
			volt_safe = calib_data->u_volt_samples[idx];
	}

	return volt_safe;
}

/**
 * sr_periodic_calibration_work() - work which actually does the calibration
 * @work: pointer to the work
 *
 * calibration routine uses the following logic:
 * on the first trigger, we start the isr to collect sr voltages
 * wait for stabilization delay (reschdule self instead of sleeping)
 * after the delay, see if we collected any isr events
 * if none, we have calibrated voltage.
 * if there are any, we retry untill we giveup.
 * on retry timeout, select a voltage to use as safe voltage.
 */
static void sr_periodic_calibration_work(struct work_struct *work)
{
	struct sr_periodic_calib_data *calib_data =
	    container_of(work, struct sr_periodic_calib_data, calib_work.work);
	unsigned long u_volt_safe = 0, u_volt_current = 0, u_volt_margin = 0;
	struct omap_sr_nvalue_table *nvalue_row;
	struct omap_sr *sr;

	if (!calib_data) {
		pr_err("%s: bad work data\n", __func__);
		return;
	}

	/*
	 * Handle the case where we might have just been scheduled AND
	 * disable was called.
	 */
	if (!mutex_trylock(&dvfs_lock)) {
		schedule_delayed_work(&calib_data->calib_work,
				      msecs_to_jiffies(
					SR_PERIODIC_SAMPLING_DELAY_MS *
					SR_PERIODIC_STABLE_SAMPLES));
		return;
	}

	sr = calib_data->sr;
	nvalue_row = sr_retrieve_nvalue_row(sr, calib_data->volt_nominal);
	if (!nvalue_row)
		return;

	/*
	 * In the unlikely case that we did get through when unplanned,
	 * flag and return.
	 */
	if (unlikely(!sr_periodic_calibration_work_is_active(sr))) {
		pr_err("%s: %s: unplanned calib_work invocation!\n",
		       __func__, sr->name);
		/* No expectation of calibration, remove qos req */
		pm_qos_update_request(&calib_data->calib_qos,
				      PM_QOS_DEFAULT_VALUE);
		mutex_unlock(&dvfs_lock);
		return;
	}

	calib_data->num_calib_triggers++;
	/* if we are triggered first time, we need to start isr to sample */
	if (calib_data->num_calib_triggers == 1) {
		/* We could be interrupted many times, so, only for debug */
		pr_debug("%s: %s: Calibration start: Voltage Nominal=%lu\n",
			 __func__, sr->name, nvalue_row->volt_nominal);
		goto start_sampling;
	}

	/* Stop isr from interrupting our measurements */
	if (AVS_SW_LOOP != sr->calibration_loop)
		sr_notifier_control(sr, false);

	/*
	 * Quit sampling
	 * a) if we have oscillations
	 * b) if we have nominal voltage as the voltage
	 */
	if (calib_data->num_calib_triggers == SR_PERIODIC_MAX_TRIGGERS)
		goto stop_sampling;

	/*
	 * if there are no samples captured, between calib_work triggering.
	 * SR is silent, aka stability!
	 */
	if (!calib_data->num_osc_samples) {
		u_volt_current = sr_get_pmic_voltage(sr);
		/*
		 * If we waited enough amount of iterations, then we might be
		 * on a corner case where voltage adjusted = Vnom!
		 */
		if (calib_data->num_calib_triggers < 2 &&
		    u_volt_current >= nvalue_row->volt_nominal)
			goto start_sampling;
		u_volt_safe = u_volt_current;
		goto done_calib;
	}

	/* we have potential oscillations/first sample */
start_sampling:
	calib_data->num_osc_samples = 0;

	if (AVS_SW_LOOP != sr->calibration_loop) {
		/* Clear transdone events so that we can go on. */
		sr_hw_force_clear_tranxdone(sr);

		/* Clear pending events */
		sr_notifier_control(sr, false);
		/* trigger sampling */
		sr_notifier_control(sr, true);
	}

	schedule_delayed_work(&calib_data->calib_work,
			      msecs_to_jiffies(SR_PERIODIC_SAMPLING_DELAY_MS *
					       SR_PERIODIC_STABLE_SAMPLES));
	mutex_unlock(&dvfs_lock);
	return;

stop_sampling:
	/*
	 * We are here for Oscillations due to two scenarios:
	 * a) SR is attempting to adjust voltage lower than VLIMITO
	 *    which VP will ignore, but SR will re-attempt
	 * b) actual oscillations
	 * NOTE: For debugging, enable debug to see the samples.
	 */
	pr_warn("%s: %s Stop sampling: Vnom=%lu samples=%d triggers=%u\n",
		__func__, sr->name,
		nvalue_row->volt_nominal, calib_data->num_osc_samples,
		calib_data->num_calib_triggers);

	/* pick up current voltage */
	u_volt_current = sr_get_pmic_voltage(sr);
	u_volt_safe = sr_periodic_find_safe_voltage(calib_data, u_volt_current);

	/* Fall through to close up common stuff */
done_calib:
	sr->stop_loop(sr);

	/* Add Per OPP margin if needed */
	if (nvalue_row->volt_margin)
		u_volt_margin += sr_periodic_adjust_margin(sr,
							nvalue_row->volt_margin,
							u_volt_current);
	u_volt_safe += u_volt_margin;

	/* just warn, dont clamp down on voltage */
	if (u_volt_margin && u_volt_safe > nvalue_row->volt_nominal) {
		pr_warn("%s: %s Vsafe %ld > Vnom %lu. %ld[%d] margin on"
			"vnom %lu curr_v=%ld\n", __func__, sr->name,
			u_volt_safe, nvalue_row->volt_nominal, u_volt_margin,
			nvalue_row->volt_margin, nvalue_row->volt_nominal,
			u_volt_current);
	}

	nvalue_row->volt_calibrated = u_volt_safe;
	/* Setup my dynamic voltage for the next calibration for this opp */
	nvalue_row->volt_dynamic_nominal = sr_get_dyn_nominal(nvalue_row);

	/* scale to new calibrated voltage */
	sr_set_pmic_voltage(sr, nvalue_row->volt_calibrated);

	pr_info("%s: %s: Calibration complete: Voltage Nominal=%lu "
		"Calib=%u Dyn=%u OPP_margin=%u total_margin=%ld\n",
		__func__, sr->name, nvalue_row->volt_nominal,
		nvalue_row->volt_calibrated, nvalue_row->volt_dynamic_nominal,
		nvalue_row->volt_margin, u_volt_margin);

	calib_data->num_calib_triggers = 0;
	calib_data->num_osc_samples = 0;
	calib_data->work_active = false;

	/* Calibration done, Remove qos req */
	pm_qos_update_request(&calib_data->calib_qos, PM_QOS_DEFAULT_VALUE);
	mutex_unlock(&dvfs_lock);
}

/**
 * sr_reset_calibration_voltages() - resets calibration voltages
 * @sr:		SmartReflex used for calibration
 *
 * Resets previously calibrated voltages. Function doesn't do
 * actual scaling.
 */
static void sr_reset_calibration_voltages(struct omap_sr *sr)
{
	u32 i;

	if (!sr) {
		pr_err("%s: SmartReflex pointer is NULL. Caller %pF\n",
		       __func__, (void *)_RET_IP_);
		return;
	}

	if (!sr->nvalue_table) {
		pr_err("%s: %s: Missing ntarget value table\n",
		       __func__, sr->name);
		return;
	}

	for (i = 0; i < sr->nvalue_count; i++) {
		sr->nvalue_table[i].volt_calibrated = 0;
		sr->nvalue_table[i].volt_dynamic_nominal = 0;
	}
}

/**
 * sr_periodic_recalibration_work() - work which does the recalibration
 * @work: pointer to the work
 *
 * on a periodic basis, we come and reset our calibration setup
 * so that a recalibration of the OPPs take place. This takes
 * care of aging factor in the system.
 */
static void sr_periodic_recalibration_work(struct work_struct *work)
{
	unsigned long delay;
	struct sr_periodic_calib_data *calib_data =
	    container_of(work, struct sr_periodic_calib_data, recal_work.work);
	struct omap_sr *sr;

	if (!calib_data) {
		pr_err("%s: bad work data\n", __func__);
		return;
	}

	/* try lock only to avoid deadlock with suspend handler */
	if (!mutex_trylock(&dvfs_lock)) {
		pr_err("%s: Can't acquire lock, delay recalibration\n",
		       __func__);
		schedule_delayed_work(&calib_data->recal_work, msecs_to_jiffies
				      (SR_PERIODIC_SAMPLING_DELAY_MS *
				       SR_PERIODIC_STABLE_SAMPLES));
		return;
	}

	sr = calib_data->sr;

	/*
	 * Deny Idle during recalibration due to the following reasons:
	 * - HW loop is enabled when we enter this function
	 * - HW loop may be triggered at any moment of time from idle
	 * - As result we may have race between SmartReflex disabling/enabling
	 * from CPU Idle and from recalibration function
	 */
	pm_qos_update_request(&calib_data->recal_qos, 0);

	delay = msecs_to_jiffies(sr->recalibration_period);

	if (sr->autocomp_active) {
		sr->stop_calibration(sr);

		/* reset voltages */
		sr->volt_reset(sr);
		sr_reset_calibration_voltages(sr);

		sr->start_calibration(sr, sr->current_nominal_voltage);
		pr_info("%s: %s: calibration reset\n", __func__, sr->name);
	}

	/* Enable CPU Idle */
	pm_qos_update_request(&calib_data->recal_qos, PM_QOS_DEFAULT_VALUE);

	calib_data->next_recal_time = jiffies + delay;
	schedule_delayed_work(&calib_data->recal_work, delay);
	mutex_unlock(&dvfs_lock);
}

/**
 * sr_init_periodic_recalibration() - init routine for periodic re-calibration
 * @sr:		SmartReflex for init
 *
 * Returns 0, or error code in case of failure.
 */
static int sr_init_periodic_recalibration(struct omap_sr *sr)
{
	struct sr_periodic_calib_data *calib_data = NULL;
	unsigned long delay;

	if (!sr) {
		pr_err("%s: SmartReflex pointer is NULL. Caller %pF\n",
		       __func__, (void *)_RET_IP_);
		return -EINVAL;
	}

	if (!sr->alg_data) {
		pr_err("%s: %s: Calibration not initialized\n",
		       __func__, sr->name);
		return -EINVAL;
	}

	if (!sr->recalibration_period) {
		pr_err("%s: %s: Recalibration period is 0. Can't init!\n",
		       __func__, sr->name);
		return -EINVAL;
	}

	calib_data = (struct sr_periodic_calib_data *)sr->alg_data;

	INIT_DELAYED_WORK(&calib_data->recal_work,
			  sr_periodic_recalibration_work);
	pm_qos_add_request(&calib_data->recal_qos, PM_QOS_CPU_DMA_LATENCY,
			   PM_QOS_DEFAULT_VALUE);

	delay = msecs_to_jiffies(sr->recalibration_period);
	schedule_delayed_work(&calib_data->recal_work, delay);
	calib_data->next_recal_time = jiffies + delay;

	pr_info("%s recalibration delay = %dms\n", sr->name,
		sr->recalibration_period);

	return 0;
}

/**
 * sr_deinit_periodic_recalibration() - cleanup for periodic re-calibration
 * @sr:		SmartReflex for cleanup
 */
static void sr_deinit_periodic_recalibration(struct omap_sr *sr)
{
	struct sr_periodic_calib_data *calib_data = NULL;

	if (!sr) {
		pr_err("%s: SmartReflex pointer is NULL. Caller %pF\n",
		       __func__, (void *)_RET_IP_);
		return;
	}

	if (!sr->alg_data) {
		pr_err("%s: %s: Calibration not initialized\n",
		       __func__, sr->name);
		return;
	}

	calib_data = (struct sr_periodic_calib_data *)sr->alg_data;

	calib_data->next_recal_time = 0;
	cancel_delayed_work_sync(&calib_data->recal_work);
	pm_qos_remove_request(&calib_data->recal_qos);
}

/**
 * sr_init_periodic_calibration() - init routine for periodic calibration
 * @sr:		SmartReflex for init
 *
 * Returns 0, or error code in case of failure.
 */
static int sr_init_periodic_calibration(struct omap_sr *sr)
{
	struct sr_periodic_calib_data *calib_data = NULL;

	if (!sr) {
		pr_err("%s: SmartReflex pointer is NULL. Caller %pF\n",
		       __func__, (void *)_RET_IP_);
		return -EINVAL;
	}

	if (sr->alg_data) {
		pr_err("%s: %s: Calibration already initialized\n",
		       __func__, sr->name);
		return -EINVAL;
	}

	/* setup our work params */
	calib_data = kzalloc(sizeof(struct sr_periodic_calib_data), GFP_KERNEL);
	if (!calib_data) {
		pr_err("%s: %s: no memory to allocate work data\n",
		       __func__, sr->name);
		return -ENOMEM;
	}

	calib_data->sr = sr;
	sr->alg_data = (void *)calib_data;

	INIT_DELAYED_WORK(&calib_data->calib_work,
			  sr_periodic_calibration_work);

	pm_qos_add_request(&calib_data->calib_qos, PM_QOS_CPU_DMA_LATENCY,
			   PM_QOS_DEFAULT_VALUE);
	return 0;
}

/**
 * sr_deinit_periodic_calibration() - cleanup for periodic calibration
 * @sr:		SmartReflex for cleanup
 *
 * Stops calibration work and resets voltages. Does actual scaling.
 */
static void sr_deinit_periodic_calibration(struct omap_sr *sr)
{
	struct sr_periodic_calib_data *calib_data = NULL;

	if (!sr) {
		pr_err("%s: SmartReflex pointer is NULL. Caller %pF\n",
		       __func__, (void *)_RET_IP_);
		return;
	}

	if (!sr->alg_data) {
		pr_err("%s: %s: Calibration not initialized\n",
		       __func__, sr->name);
		return;
	}

	calib_data = (struct sr_periodic_calib_data *)sr->alg_data;

	/*
	 * we dont have SR periodic calib anymore.. so reset calibs
	 * we are already protected by appropriate locks, so no lock needed
	 * here.
	 */
	sr_stop_periodic_calibration(sr);

	sr->volt_reset(sr);
	sr_reset_calibration_voltages(sr);

	kfree(calib_data);
	sr->alg_data = NULL;
}

/**
 * sr_periodic_calibration_work() - work which does software calibration
 * @work: pointer to the work
 *
 * Typically, this work is scheduled from SR interrupt notifier handler.
 * Work does a one step of the software calibration routine.
 * Calibration routine uses the following logic:
 * - disable SR interrupt
 * - retrieve previously calculated Voltage error - delta between
 *   current PMIC voltage and optimal voltage, which is defined by
 *   corresponding NVALUE
 * - calculate target voltage:
 *	volt_target = volt_current + volt_error /2
 * - do actual voltage scaling
 * - enable SR interrupt
 *
 * Every time, when SR interrupt occurs, work is scheduled, and it scales
 * PMIC to new voltage. Every time this new voltage is closer and closer
 * to optimal voltage value, which is defined by corresponding NVALUE.
 * When target voltage is close or equal to optimal voltage, SR interrupt
 * doesn't occur. Last voltage chosen by this work is calibrated voltage.
 */
static void sr_sw_calibration_work(struct work_struct *work)
{
	struct sr_sw_loop_data *loop_data =
	    container_of(work, struct sr_sw_loop_data, scale_work);
	struct omap_sr *sr;
	struct omap_sr_nvalue_table *nvalue_row;
	s32 volt_err, volt_curr;
	const u8 volt_div = 2;

	if (!loop_data) {
		pr_err("%s: bad work data\n", __func__);
		return;
	}

	sr = loop_data->sr;

	/*
	 * Handle the case when
	 * - disable was called
	 * - recalibration is active
	 * - periodic work is active
	 */
	if (!mutex_trylock(&dvfs_lock)) {
		schedule_work(&loop_data->scale_work);
		return;
	}

	/* Disable SR interrupts */
	sr_notifier_control(sr, false);

	nvalue_row = sr_retrieve_nvalue_row(sr, sr->current_nominal_voltage);
	if (!nvalue_row)
		goto end;

	if (!nvalue_row->volt_error)
		goto end;

	/* Read current voltage */
	volt_curr = sr_get_pmic_voltage(sr);
	if (volt_curr < 0) {
		pr_err("%s: %s: error current voltage (%d)\n",
		       __func__, sr->name, volt_curr);
		goto end;
	}

	/* Deny Idle */
	pm_qos_update_request(&loop_data->scale_qos, 0);

	/*
	 * Calculate target voltage. Divide voltage error by 2 - this
	 * allows to have "calibration steps", which are similar to what VP
	 * does during HW loop calibration
	 */
	volt_err = (s32)volt_curr + nvalue_row->volt_error / volt_div;

	sr_periodic_store_sample(sr, volt_err);

	pr_debug("%s: %s Vn (%u uV) Vc (%d uV), Vset (%d uV), delta (%d)\n",
		 __func__, sr->name, sr->current_nominal_voltage, volt_curr,
		 volt_err, nvalue_row->volt_error / volt_div);

	/* Do actual scaling */
	sr_set_pmic_voltage(sr, volt_err);

	/* Enable CPU Idle */
	pm_qos_update_request(&loop_data->scale_qos, PM_QOS_DEFAULT_VALUE);

	/* Enable SR interrupts to keep calibration going */
	sr_notifier_control(sr, true);

end:
	mutex_unlock(&dvfs_lock);
}

/**
 * sr_init_sw_loop() - init routine for software calibration loop
 * @sr:		SmartReflex for init
 *
 * Returns 0, or error code in case of failure.
 */
static int sr_init_sw_loop(struct omap_sr *sr)
{
	struct sr_sw_loop_data *loop_data = NULL;

	if (!sr) {
		pr_err("%s: SmartReflex pointer is NULL. Caller %pF\n",
		       __func__, (void *)_RET_IP_);
		return -EINVAL;
	}

	/* setup our work params */
	loop_data = kzalloc(sizeof(struct sr_sw_loop_data), GFP_KERNEL);
	if (!loop_data) {
		pr_err("%s: %s: no memory to allocate work data\n",
		       __func__, sr->name);
		return -ENOMEM;
	}

	loop_data->sr = sr;
	sr->loop_data = (void *)loop_data;

	INIT_WORK(&loop_data->scale_work, sr_sw_calibration_work);
	pm_qos_add_request(&loop_data->scale_qos, PM_QOS_CPU_DMA_LATENCY,
			   PM_QOS_DEFAULT_VALUE);

	sr_irq_notifier_register(sr, &sr_irq_sw_notifier);
	return 0;
}

/**
 * sr_deinit_sw_loop() - cleanup for software calibration loop
 * @sr:		SmartReflex for cleanup
 *
 * Stops software calibration work.
 */
static int sr_deinit_sw_loop(struct omap_sr *sr)
{
	struct sr_sw_loop_data *loop_data = NULL;

	if (!sr) {
		pr_err("%s: SmartReflex pointer is NULL. Caller %pF\n",
		       __func__, (void *)_RET_IP_);
		return -EINVAL;
	}

	loop_data = (struct sr_sw_loop_data *)sr->loop_data;

	sr_irq_notifier_unregister(sr, &sr_irq_sw_notifier);

	/* Ensure worker canceled */
	cancel_work_sync(&loop_data->scale_work);
	pm_qos_remove_request(&loop_data->scale_qos);

	kfree(loop_data);
	sr->loop_data = NULL;

	return 0;
}

/**
 * sr_init_set_voltage_calibration() - init routine for "no loop" calibration
 * @sr:		SmartReflex for init
 *
 * Reads pre-calibrated voltages from EFUSE registers and store them
 * in NVALUE table. This calibration routine doesn't perform actual
 * calibration, it just passes voltages retrieved from EFUSE registers
 * to PMIC. Returns 0, or error code in case of failure.
 */
static int sr_init_set_voltage_calibration(struct omap_sr *sr)
{
	void __iomem *efuse_base;
	struct resource *mem;
	u32 i;

	if (!sr) {
		pr_err("%s: SmartReflex pointer is NULL. Caller %pF\n",
		       __func__, (void *)_RET_IP_);
		return -EINVAL;
	}

	/* pre calibrated voltages are stored in EFUSE registers */
	mem = platform_get_resource_byname(sr->pdev, IORESOURCE_MEM,
					   "efuse-address");
	efuse_base = devm_ioremap_nocache(&sr->pdev->dev, mem->start,
					  resource_size(mem));
	if (IS_ERR(efuse_base)) {
		pr_err("%s: %s: ioremap for EFUSE failed\n",
		       __func__, sr->name);
		return PTR_ERR(efuse_base);
	}

	for (i = 0; i <  sr->nvalue_count; i++) {
		sr->nvalue_table[i].volt_calibrated =
			readl(efuse_base + sr->nvalue_table[i].efuse_offs);
	}

	return 0;
}

/**
 * sr_set_efused_voltage() - sets PMIC voltage
 * @sr:		SmartReflex used for calibration
 * @volt:	Nominal voltage, which needs to be calibrated
 *
 * Callback. Passes previously retrieved pre calibrated voltage
 * to PMIC. Returns 0, or error code in case of failure.
 */
static int sr_set_efused_voltage(struct omap_sr *sr,
				 unsigned long volt)
{
	struct omap_sr_nvalue_table *nvalue_row;

	if (!sr) {
		pr_err("%s: SmartReflex pointer is NULL. Caller %pF\n",
		       __func__, (void *)_RET_IP_);
		return -EINVAL;
	}

	nvalue_row = sr_retrieve_nvalue_row(sr, volt);
	if (!nvalue_row)
		return -ENODATA;

	return sr_set_pmic_voltage(sr, nvalue_row->volt_calibrated);
}

/**
 * sr_stop_set_voltage_calibration() - stub function
 * @sr:		SmartReflex used for calibration
 *
 * Callback. Stub function which does nothing. Defined
 * for compatibility.
 */
static int sr_stop_set_voltage_calibration(struct omap_sr *sr)
{
	return 0;
}

/**
 * sr_print_settings() - prints SmartReflex settings
 * @sr:		SmartReflex used for calibration
 *
 * Prints SmartReflex settings in user friendly format.
 * Typically called when SmartRefles autocompletion starts.
 */
static void sr_print_settings(const struct omap_sr *sr)
{
	pr_info("TI SmartReflex:\n");
	switch (sr->calibration_period) {
	case AVS_CALIBRATION_CONTINUOUS:
		pr_info("\tContinuous calibration\n");
		break;
	case AVS_CALIBRATION_PERIODIC:
		pr_info("\tPeriodic calibration\n");
		pr_info("\tRe-calibration period is %u msec\n",
			sr->recalibration_period);
		break;
	case AVS_CALIBRATION_BOOT:
		pr_info("\tBoot time calibration\n");
		break;
	case AVS_CALIBRATION_SET_VOLTAGE:
		pr_info("\tEFUSED voltages\n");
		break;
	default:
		pr_err("Calibration period doesn't set properly %d\n",
		       sr->calibration_period);
		break;
	}

	switch (sr->calibration_loop) {
	case AVS_HW_LOOP:
		pr_info("\tHW loop is used for calibration (VP)\n");
		break;
	case AVS_SW_LOOP:
		pr_info("\tSW loop is used for calibration\n");
		break;
	case AVS_NO_LOOP:
		pr_info("\tNo calibration loop\n");
		break;
	default:
		break;
	}
}

/**
 * sr_init_algs() - init API for SmartReflex algorithms
 * @sr:		SmartReflex for init
 *
 * Initializes SmartReflex algorithms according to settings,
 * retrieved from device tree. Returns 0, or error code
 * in case of failure.
 */
int sr_init_algs(struct omap_sr *sr)
{
	int ret = 0;

	if (!sr) {
		pr_err("%s: SmartReflex pointer is NULL. Caller %pF\n",
		       __func__, (void *)_RET_IP_);
		return -EINVAL;
	}

	switch (sr->calibration_period) {
	case AVS_CALIBRATION_PERIODIC:
		ret = sr_init_periodic_calibration(sr);
		if (ret)
			break;
		ret = sr_init_periodic_recalibration(sr);
		break;
	case AVS_CALIBRATION_BOOT:
		ret = sr_init_periodic_calibration(sr);
		break;
	case AVS_CALIBRATION_SET_VOLTAGE:
		ret = sr_init_set_voltage_calibration(sr);
		break;
	default:
		break;
	}

	if (ret)
		return ret;

	switch (sr->calibration_loop) {
	case AVS_SW_LOOP:
		ret = sr_init_sw_loop(sr);
		break;
	case AVS_HW_LOOP:
		if ((AVS_CALIBRATION_PERIODIC == sr->calibration_period) ||
		    (AVS_CALIBRATION_BOOT == sr->calibration_period))
			ret = sr_irq_notifier_register(sr, &sr_irq_hw_notifier);
		break;
	default:
		break;
	}

	if (ret)
		return ret;

	sr_print_settings(sr);

	return ret;
}

/**
 * sr_deinit_algs() - cleanup API for SmartReflex algorithms
 * @sr:		SmartReflex for cleanup
 *
 * Returns 0, or error code in case of failure.
 */
int sr_deinit_algs(struct omap_sr *sr)
{
	if (!sr) {
		pr_err("%s: SmartReflex pointer is NULL. Caller %pF\n",
		       __func__, (void *)_RET_IP_);
		return -EINVAL;
	}

	switch (sr->calibration_period) {
	case AVS_CALIBRATION_PERIODIC:
		sr_deinit_periodic_recalibration(sr);
	case AVS_CALIBRATION_BOOT:
		sr_deinit_periodic_calibration(sr);
		break;
	case AVS_CALIBRATION_SET_VOLTAGE:
		sr_reset_calibration_voltages(sr);
		break;
	default:
		break;
	}

	switch (sr->calibration_loop) {
	case AVS_SW_LOOP:
		sr_deinit_sw_loop(sr);
		break;
	case AVS_HW_LOOP:
		if ((AVS_CALIBRATION_PERIODIC == sr->calibration_period) ||
		    (AVS_CALIBRATION_BOOT == sr->calibration_period))
			sr_irq_notifier_unregister(sr, &sr_irq_hw_notifier);
		break;

	default:
		break;
	}

	return 0;
}

/**
 * sr_get_start_calibration_func() -  API to get start calibration callback
 * @period:		type of SmartReflex calibration period
 *
 * Returns pointer to calibration callback, or NULL in case of failure.
 */
sr_start_calibration_pfunc
__init sr_get_start_calibration_func(enum omap_sr_calibration_period period)
{
	int (*pfunc) (struct omap_sr *sr, unsigned long volt) = NULL;

	switch (period) {
	case AVS_CALIBRATION_BOOT:
	case AVS_CALIBRATION_PERIODIC:
		pfunc = sr_start_periodic_calibration;
		break;
	case AVS_CALIBRATION_CONTINUOUS:
		pfunc = sr_start_continuous_calibration;
		break;
	case AVS_CALIBRATION_SET_VOLTAGE:
		pfunc = sr_set_efused_voltage;
		break;
	default:
		pr_err("%s: wrong calibration period type (%d)\n",
		       __func__, period);
		break;
	}

	return pfunc;
}

/**
 * sr_get_stop_calibration_func() -  API to get stop calibration callback
 * @period:		type of SmartReflex calibration period
 *
 * Returns pointer to calibration callback, or NULL in case of failure.
 */
sr_stop_calibration_pfunc
__init sr_get_stop_calibration_func(enum omap_sr_calibration_period period)
{
	int (*pfunc) (struct omap_sr *sr) = NULL;

	switch (period) {
	case AVS_CALIBRATION_BOOT:
	case AVS_CALIBRATION_PERIODIC:
		pfunc = sr_stop_periodic_calibration;
		break;
	case AVS_CALIBRATION_CONTINUOUS:
		pfunc = sr_stop_continuous_calibration;
		break;
	case AVS_CALIBRATION_SET_VOLTAGE:
		pfunc = sr_stop_set_voltage_calibration;
		break;
	default:
		pr_err("%s: wrong calibration period type (%d)\n",
		       __func__, period);
		break;
	}

	return pfunc;
}

/**
 * sr_get_start_loop_func() -  API to get start loop callback
 * @loop:		type of SmartReflex calibration loop
 *
 * Returns pointer to loop callback, or NULL in case of failure.
 */
sr_start_loop_pfunc
__init sr_get_start_loop_func(enum omap_sr_calibration_loop loop)
{
	int (*pfunc) (struct omap_sr *sr, unsigned long volt) = NULL;

	switch (loop) {
	case AVS_HW_LOOP:
		pfunc = sr_start_hw_loop;
		break;
	case AVS_SW_LOOP:
		pfunc = sr_start_sw_loop;
		break;
	case AVS_NO_LOOP:
		break;
	default:
		pr_err("%s: wrong loop type (%d)\n", __func__, loop);
		break;
	}

	return pfunc;
}

/**
 * sr_get_stop_loop_func() -  API to get stop loop callback
 * @loop:		type of SmartReflex calibration loop
 *
 * Returns pointer to loop callback, or NULL in case of failure.
 */
sr_stop_loop_pfunc
__init sr_get_stop_loop_func(enum omap_sr_calibration_loop loop)
{
	int (*pfunc) (struct omap_sr *sr) = NULL;

	switch (loop) {
	case AVS_HW_LOOP:
		pfunc = sr_stop_hw_loop;
		break;
	case AVS_SW_LOOP:
		pfunc = sr_stop_sw_loop;
		break;
	case AVS_NO_LOOP:
		break;
	default:
		pr_err("%s: wrong loop type (%d)\n", __func__, loop);
		break;
	}

	return pfunc;
}

/**
 * sr_get_volt_reset_func() -  API to get voltage reset callback
 * @sr:		SmartReflex for init
 *
 * Returns pointer to voltage reset callback, or NULL in case of failure.
 */
sr_volt_reset_pfunc
__init sr_get_volt_reset_func(struct omap_sr *sr)
{
	int (*pfunc) (struct omap_sr *sr) = NULL;

	if (!sr) {
		pr_err("%s: SmartReflex pointer is NULL. Caller %pF\n",
		       __func__, (void *)_RET_IP_);
		return NULL;
	}

	switch (sr->calibration_period) {
	case AVS_CALIBRATION_BOOT:
	case AVS_CALIBRATION_PERIODIC:
		pfunc = sr_reset_volt_periodic;
		break;
	case AVS_CALIBRATION_CONTINUOUS:
		if (AVS_HW_LOOP == sr->calibration_loop)
			pfunc = sr_reset_volt_to_nominal_hwloop;
		else if (AVS_SW_LOOP == sr->calibration_loop)
			pfunc = sr_reset_volt_to_nominal;
		else
			pr_err("%s: wrong calibration loop type (%d)\n",
			       __func__, sr->calibration_loop);
		break;
	case AVS_CALIBRATION_SET_VOLTAGE:
		pfunc = sr_reset_volt_to_nominal;
		break;
	default:
		pr_err("%s: wrong calibration period type (%d)\n",
		       __func__, sr->calibration_period);
		break;
	}

	return pfunc;
}
