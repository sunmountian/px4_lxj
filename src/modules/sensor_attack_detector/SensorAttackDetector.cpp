/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED.
 *
 ****************************************************************************/

#include "SensorAttackDetector.hpp"

#include <drivers/drv_pwm_output.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/log.h>

#include <float.h>
#include <math.h>

using matrix::Dcmf;
using matrix::Quatf;
using matrix::Vector3f;

SensorAttackDetector::SensorAttackDetector() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	updateParams();
}

SensorAttackDetector::~SensorAttackDetector()
{
	if (_callback_registered) {
		_vehicle_imu_sub.unregisterCallback();
	}

	ScheduleClear();
	perf_free(_cycle_perf);
	perf_free(_solver_perf);
}

bool SensorAttackDetector::init()
{
	_callback_registered = _vehicle_imu_sub.registerCallback();

	if (!_callback_registered) {
		// During boot vehicle_imu might not yet be advertised. Keep the module
		// alive and retry without requiring an external restart.
		PX4_WARN("vehicle_imu unavailable, retrying callback registration");
		ScheduleOnInterval(20_ms);
	}

	return true;
}

int SensorAttackDetector::task_spawn(int argc, char *argv[])
{
	SensorAttackDetector *instance = new SensorAttackDetector();

	if (instance != nullptr) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	PX4_ERR("allocation or initialization failed");
	return PX4_ERROR;
}

void SensorAttackDetector::request_stop()
{
	ModuleBase<SensorAttackDetector>::request_stop();
	ScheduleNow();
}

void SensorAttackDetector::parametersUpdate()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s parameter_update{};
		_parameter_update_sub.copy(&parameter_update);
		updateParams();
	}
}

void SensorAttackDetector::Run()
{
	if (should_exit()) {
		if (_callback_registered) {
			_vehicle_imu_sub.unregisterCallback();
		}

		exit_and_cleanup();
		return;
	}

	const hrt_abstime run_start = hrt_absolute_time();
	perf_begin(_cycle_perf);

	if (!_callback_registered) {
		_callback_registered = _vehicle_imu_sub.registerCallback();

		if (!_callback_registered) {
			perf_end(_cycle_perf);
			return;
		}

		ScheduleClear();
	}

	parametersUpdate();
	ingestAttitude();
	ingestActuator();
	ingestGps();
	ingestImu();
	processPendingImu();
	runPendingEvaluation(run_start);

	perf_end(_cycle_perf);
}

void SensorAttackDetector::ingestAttitude()
{
	vehicle_attitude_s attitude{};

	if (!_vehicle_attitude_sub.update(&attitude)) {
		return;
	}

	const uint64_t timestamp = attitude.timestamp_sample != 0 ? attitude.timestamp_sample : attitude.timestamp;
	float norm_squared = 0.f;

	for (float component : attitude.q) {
		if (!PX4_ISFINITE(component)) {
			_attitude_valid = false;
			return;
		}

		norm_squared += component * component;
	}

	if ((timestamp == 0) || !(norm_squared > 0.25f)) {
		_attitude_valid = false;
		return;
	}

	if (timestamp <= _last_attitude_timestamp) {
		if (timestamp < _last_attitude_timestamp) {
			_data_quality_flags |= sensor_attack_status_s::DATA_QUALITY_TIMESTAMP;
		}

		return;
	}

	AttitudeEvent event{};
	event.timestamp = timestamp;

	for (size_t i = 0; i < 4; ++i) {
		event.q[i] = attitude.q[i];
	}

	// This short history is intentionally rolling. Once full, samples older
	// than the alignment horizon are no longer needed.
	_attitude_buffer.push(event);

	_last_attitude_timestamp = timestamp;
	_attitude_valid = true;
}

void SensorAttackDetector::ingestActuator()
{
	ActuatorEvent event{};
	bool updated = false;
	bool valid = false;

	if (_param_sad_act_src.get() == sensor_attack_status_s::ACTUATOR_MOTORS) {
		actuator_motors_s motors{};

		if (_actuator_motors_sub.update(&motors)) {
			updated = true;
			event.timestamp = motors.timestamp_sample != 0 ? motors.timestamp_sample : motors.timestamp;
			float thrust_indicator = 0.f;
			valid = event.timestamp != 0;

			for (size_t i = 0; i < kMotorCount; ++i) {
				if (!PX4_ISFINITE(motors.control[i])) {
					valid = false;
					break;
				}

				thrust_indicator += fmaxf(motors.control[i], 0.f);
			}

			event.thrust_indicator = thrust_indicator;
		}

	} else {
		actuator_outputs_s outputs{};

		if (_actuator_outputs_sub.update(&outputs)) {
			updated = true;
			event.timestamp = outputs.timestamp;
			float thrust_indicator = 0.f;
			valid = (event.timestamp != 0) && (outputs.noutputs >= kMotorCount);

			for (size_t i = 0; valid && (i < kMotorCount); ++i) {
				if (!PX4_ISFINITE(outputs.output[i])) {
					valid = false;
					break;
				}

				const float normalized = (outputs.output[i] - PWM_DEFAULT_MIN) /
							 (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN);
				thrust_indicator += math::constrain(normalized, 0.f, 1.f);
			}

			event.thrust_indicator = thrust_indicator;
		}
	}

	if (!updated) {
		return;
	}

	_actuator_valid = valid;

	if (!valid) {
		return;
	}

	if (event.timestamp <= _last_actuator_timestamp) {
		if (event.timestamp < _last_actuator_timestamp) {
			_data_quality_flags |= sensor_attack_status_s::DATA_QUALITY_TIMESTAMP;
		}

		return;
	}

	// This short history is intentionally rolling. Once full, samples older
	// than the alignment horizon are no longer needed.
	_actuator_buffer.push(event);

	_last_actuator_timestamp = event.timestamp;
}

void SensorAttackDetector::ingestGps()
{
	sensor_gps_s gps{};

	if (!_sensor_gps_sub.update(&gps)) {
		return;
	}

	const uint64_t timestamp = gps.timestamp;
	const bool timestamp_valid = (timestamp != 0) && (timestamp > _last_gps_message_timestamp);

	if (!timestamp_valid) {
		if ((timestamp != 0) && (timestamp < _last_gps_message_timestamp)) {
			_data_quality_flags |= sensor_attack_status_s::DATA_QUALITY_TIMESTAMP;
		}

		return;
	}

	_last_gps_message_timestamp = timestamp;
	_gps_valid = (gps.fix_type >= 3) && gps.vel_ned_valid && PX4_ISFINITE(gps.eph)
		     && (gps.eph <= _param_sad_gps_eph.get()) && PX4_ISFINITE(gps.vel_n_m_s)
		     && PX4_ISFINITE(gps.vel_e_m_s);

	if (!_gps_valid) {
		_data_quality_flags |= sensor_attack_status_s::DATA_QUALITY_GPS;
		_pending_evaluation_timestamp = timestamp;
		_pending_gps_valid = false;
		_evaluation_pending = true;
		return;
	}

	const double latitude = static_cast<double>(gps.lat) * 1e-7;
	const double longitude = static_cast<double>(gps.lon) * 1e-7;

	if (!_gps_projection.isInitialized()) {
		_gps_projection.initReference(latitude, longitude, timestamp);
		_origin_timestamp = timestamp;
	}

	GpsEvent event{};
	event.timestamp = timestamp;
	_gps_projection.project(latitude, longitude, event.position[0], event.position[1]);
	event.velocity[0] = gps.vel_n_m_s;
	event.velocity[1] = gps.vel_e_m_s;

	if (!_gps_buffer.empty() && (event.timestamp <= _gps_buffer.back().timestamp)) {
		_data_quality_flags |= sensor_attack_status_s::DATA_QUALITY_TIMESTAMP;
		return;
	}

	if (!_gps_buffer.push(event)) {
		_data_quality_flags |= sensor_attack_status_s::DATA_QUALITY_BUFFER_OVERFLOW;
	}

	trimLongBuffers();

	const float maximum_rate_hz = fmaxf(_param_sad_rate.get(), 1.f);
	const uint64_t minimum_interval_us = static_cast<uint64_t>(1e6f / maximum_rate_hz);

	if ((_last_evaluation_timestamp == 0)
	    || (timestamp >= _last_evaluation_timestamp + minimum_interval_us)) {
		_pending_evaluation_timestamp = timestamp;
		_pending_gps_valid = true;
		_evaluation_pending = true;
	}
}

void SensorAttackDetector::ingestImu()
{
	vehicle_imu_s imu{};

	if (!_vehicle_imu_sub.update(&imu)) {
		return;
	}

	const uint64_t timestamp = imu.timestamp_sample != 0 ? imu.timestamp_sample : imu.timestamp;
	const float dt_s = static_cast<float>(imu.delta_velocity_dt) * 1e-6f;
	bool valid = (timestamp != 0) && (timestamp > _last_imu_sample_timestamp)
		     && (dt_s > 1e-5f) && (dt_s < 0.05f);

	for (float component : imu.delta_velocity) {
		valid = valid && PX4_ISFINITE(component);
	}

	if (imu.delta_velocity_clipping != 0) {
		_data_quality_flags |= sensor_attack_status_s::DATA_QUALITY_IMU_CLIPPING;
		valid = false;
	}

	if (!valid) {
		if ((timestamp != 0) && (timestamp < _last_imu_sample_timestamp)) {
			_data_quality_flags |= sensor_attack_status_s::DATA_QUALITY_TIMESTAMP;
		}

		_imu_valid = false;
		return;
	}

	PendingImuEvent event{};
	event.timestamp = timestamp;
	event.dt_s = dt_s;
	event.clipping = imu.delta_velocity_clipping;

	for (size_t i = 0; i < 3; ++i) {
		event.delta_velocity[i] = imu.delta_velocity[i];
	}

	if (!_pending_imu_buffer.push(event)) {
		_data_quality_flags |= sensor_attack_status_s::DATA_QUALITY_BUFFER_OVERFLOW;
	}

	_last_imu_sample_timestamp = timestamp;
	_imu_valid = true;
}

void SensorAttackDetector::processPendingImu()
{
	while (!_pending_imu_buffer.empty()) {
		const PendingImuEvent &imu = _pending_imu_buffer.front();
		Quatf q_nb{};
		float thrust_indicator = 0.f;
		const bool attitude_available = interpolateAttitude(imu.timestamp, q_nb);
		const bool actuator_available = findActuator(imu.timestamp, thrust_indicator);

		if (attitude_available && actuator_available) {
			aggregateAlignedImu(imu, q_nb, thrust_indicator);
			_pending_imu_buffer.pop_front();
			continue;
		}

		const bool wait_expired = (_last_imu_sample_timestamp > imu.timestamp)
					  && ((_last_imu_sample_timestamp - imu.timestamp) > kAlignmentWaitUs);

		if (!wait_expired) {
			break;
		}

		if (!attitude_available) {
			_data_quality_flags |= sensor_attack_status_s::DATA_QUALITY_ATTITUDE_ALIGNMENT;
			_attitude_valid = false;
		}

		if (!actuator_available) {
			_data_quality_flags |= sensor_attack_status_s::DATA_QUALITY_ACTUATOR_ALIGNMENT;
			_actuator_valid = false;
		}

		_pending_imu_buffer.pop_front();
	}
}

bool SensorAttackDetector::interpolateAttitude(uint64_t timestamp, Quatf &q_nb) const
{
	if (_attitude_buffer.empty() || (timestamp < _attitude_buffer.front().timestamp)
	    || (timestamp > _attitude_buffer.back().timestamp)) {
		return false;
	}

	for (size_t i = 0; i < _attitude_buffer.size(); ++i) {
		const AttitudeEvent &upper = _attitude_buffer[i];

		if (upper.timestamp == timestamp) {
			q_nb = Quatf(upper.q);
			q_nb.normalize();
			return PX4_ISFINITE(q_nb(0));
		}

		if ((upper.timestamp > timestamp) && (i > 0)) {
			const AttitudeEvent &lower = _attitude_buffer[i - 1];
			const uint64_t gap = upper.timestamp - lower.timestamp;

			if ((gap == 0) || (gap > kMaximumAttitudeGapUs)) {
				return false;
			}

			const float alpha = static_cast<float>(timestamp - lower.timestamp) / static_cast<float>(gap);
			float dot_product = 0.f;

			for (size_t j = 0; j < 4; ++j) {
				dot_product += lower.q[j] * upper.q[j];
			}

			const float upper_sign = dot_product < 0.f ? -1.f : 1.f;
			float interpolated[4] {};
			float norm_squared = 0.f;

			for (size_t j = 0; j < 4; ++j) {
				interpolated[j] = (1.f - alpha) * lower.q[j] + alpha * upper_sign * upper.q[j];
				norm_squared += interpolated[j] * interpolated[j];
			}

			if (!(norm_squared > 1e-8f)) {
				return false;
			}

			q_nb = Quatf(interpolated);
			q_nb.normalize();
			return PX4_ISFINITE(q_nb(0));
		}
	}

	return false;
}

bool SensorAttackDetector::findActuator(uint64_t timestamp, float &thrust_indicator) const
{
	for (size_t i = _actuator_buffer.size(); i > 0; --i) {
		const ActuatorEvent &event = _actuator_buffer[i - 1];

		if (event.timestamp <= timestamp) {
			const uint64_t age = timestamp - event.timestamp;

			if (age > kMaximumActuatorAgeUs) {
				return false;
			}

			thrust_indicator = event.thrust_indicator;
			return PX4_ISFINITE(thrust_indicator);
		}
	}

	return false;
}

void SensorAttackDetector::aggregateAlignedImu(const PendingImuEvent &imu, const Quatf &q_nb,
		float thrust_indicator)
{
	const Dcmf rotation_nb{q_nb};
	const Vector3f delta_velocity_body{imu.delta_velocity};
	const Vector3f delta_velocity_ned = rotation_nb * delta_velocity_body;
	const Vector3f actuator_acceleration_body{0.f, 0.f, -_param_sad_thr_gain.get() *thrust_indicator};
	const Vector3f actuator_acceleration_ned = rotation_nb * actuator_acceleration_body;
	const uint64_t bin_index = imu.timestamp / kImuBinDurationUs;

	if (_imu_bin.active && (bin_index != _imu_bin.index)) {
		flushImuBin();
	}

	if (!_imu_bin.active) {
		_imu_bin.active = true;
		_imu_bin.index = bin_index;
	}

	_imu_bin.last_timestamp = imu.timestamp;
	_imu_bin.total_dt_s += imu.dt_s;

	for (size_t axis = 0; axis < 2; ++axis) {
		_imu_bin.measured_delta_velocity[axis] += delta_velocity_ned(axis);
		_imu_bin.actuator_delta_velocity[axis] += actuator_acceleration_ned(axis) * imu.dt_s;
	}
}

void SensorAttackDetector::flushImuBin()
{
	if (!_imu_bin.active) {
		return;
	}

	if ((_imu_bin.total_dt_s >= 0.005f) && (_imu_bin.total_dt_s <= 0.05f)) {
		ImuEvent event{};
		event.timestamp = _imu_bin.last_timestamp;
		event.dt_s = _imu_bin.total_dt_s;

		for (size_t axis = 0; axis < 2; ++axis) {
			event.acceleration_measured[axis] = _imu_bin.measured_delta_velocity[axis] / _imu_bin.total_dt_s;
			event.acceleration_actuator[axis] = _imu_bin.actuator_delta_velocity[axis] / _imu_bin.total_dt_s;
		}

		if (!_imu_buffer.empty() && (event.timestamp <= _imu_buffer.back().timestamp)) {
			_data_quality_flags |= sensor_attack_status_s::DATA_QUALITY_TIMESTAMP;

		} else if (!_imu_buffer.push(event)) {
			_data_quality_flags |= sensor_attack_status_s::DATA_QUALITY_BUFFER_OVERFLOW;
		}

	} else {
		_data_quality_flags |= sensor_attack_status_s::DATA_QUALITY_WINDOW_COVERAGE;
	}

	_imu_bin = ImuBin{};
	trimLongBuffers();
}

void SensorAttackDetector::trimLongBuffers()
{
	while ((_imu_buffer.size() > 1)
	       && ((_imu_buffer.back().timestamp - _imu_buffer.front().timestamp) > kBufferRetentionUs)) {
		_imu_buffer.pop_front();
	}

	while ((_gps_buffer.size() > 1)
	       && ((_gps_buffer.back().timestamp - _gps_buffer.front().timestamp) > kBufferRetentionUs)) {
		_gps_buffer.pop_front();
	}
}

bool SensorAttackDetector::interpolateGps(uint64_t timestamp, GpsEvent &gps) const
{
	if (_gps_buffer.empty() || (timestamp < _gps_buffer.front().timestamp)
	    || (timestamp > _gps_buffer.back().timestamp)) {
		return false;
	}

	for (size_t i = 0; i < _gps_buffer.size(); ++i) {
		const GpsEvent &upper = _gps_buffer[i];

		if (upper.timestamp == timestamp) {
			gps = upper;
			return true;
		}

		if ((upper.timestamp > timestamp) && (i > 0)) {
			const GpsEvent &lower = _gps_buffer[i - 1];
			const uint64_t gap = upper.timestamp - lower.timestamp;

			if (gap == 0) {
				return false;
			}

			const float alpha = static_cast<float>(timestamp - lower.timestamp) / static_cast<float>(gap);
			gps.timestamp = timestamp;

			for (size_t axis = 0; axis < 2; ++axis) {
				gps.position[axis] = lower.position[axis] + alpha * (upper.position[axis] - lower.position[axis]);
				gps.velocity[axis] = lower.velocity[axis] + alpha * (upper.velocity[axis] - lower.velocity[axis]);
			}

			return true;
		}
	}

	return false;
}

bool SensorAttackDetector::integrateActuator(uint64_t start_timestamp, uint64_t end_timestamp,
		float (&delta_velocity)[2], float (&delta_position)[2], float &covered_time_s) const
{
	delta_velocity[0] = 0.f;
	delta_velocity[1] = 0.f;
	delta_position[0] = 0.f;
	delta_position[1] = 0.f;
	covered_time_s = 0.f;

	if (end_timestamp <= start_timestamp) {
		return false;
	}

	const double integration_end_s = static_cast<double>(end_timestamp - start_timestamp) * 1e-6;

	for (size_t i = 0; i < _imu_buffer.size(); ++i) {
		const ImuEvent &event = _imu_buffer[i];
		const double event_end_s = (static_cast<double>(event.timestamp) - static_cast<double>(start_timestamp)) * 1e-6;
		const double event_start_s = event_end_s - static_cast<double>(event.dt_s);
		const double overlap_start_s = fmax(0.0, event_start_s);
		const double overlap_end_s = fmin(integration_end_s, event_end_s);

		if (overlap_end_s <= overlap_start_s) {
			continue;
		}

		const float duration_s = static_cast<float>(overlap_end_s - overlap_start_s);
		const float time_to_end_s = static_cast<float>(integration_end_s - overlap_start_s);
		const float position_kernel = time_to_end_s * duration_s - 0.5f * duration_s * duration_s;
		covered_time_s += duration_s;

		for (size_t axis = 0; axis < 2; ++axis) {
			delta_velocity[axis] += event.acceleration_actuator[axis] * duration_s;
			delta_position[axis] += event.acceleration_actuator[axis] * position_kernel;
		}
	}

	return true;
}

bool SensorAttackDetector::interpolateActuatorAcceleration(uint64_t timestamp, float (&acceleration)[2]) const
{
	if (_imu_buffer.empty() || (timestamp < _imu_buffer.front().timestamp)
	    || (timestamp > _imu_buffer.back().timestamp)) {
		return false;
	}

	for (size_t i = 0; i < _imu_buffer.size(); ++i) {
		const ImuEvent &upper = _imu_buffer[i];

		if (upper.timestamp == timestamp) {
			acceleration[0] = upper.acceleration_actuator[0];
			acceleration[1] = upper.acceleration_actuator[1];
			return true;
		}

		if ((upper.timestamp > timestamp) && (i > 0)) {
			const ImuEvent &lower = _imu_buffer[i - 1];
			const uint64_t gap = upper.timestamp - lower.timestamp;

			if (gap == 0) {
				return false;
			}

			const float alpha = static_cast<float>(timestamp - lower.timestamp) / static_cast<float>(gap);

			for (size_t axis = 0; axis < 2; ++axis) {
				acceleration[axis] = lower.acceleration_actuator[axis]
						     + alpha * (upper.acceleration_actuator[axis] - lower.acceleration_actuator[axis]);
			}

			return PX4_ISFINITE(acceleration[0]) && PX4_ISFINITE(acceleration[1]);
		}
	}

	return false;
}


bool SensorAttackDetector::evaluateWindow(uint64_t end_timestamp,
		AxisGlrtAccumulator::Result &north_result,
		AxisGlrtAccumulator::Result &east_result,
		uint16_t &imu_sample_count, uint16_t &gps_sample_count,
		float &direction_n, float &direction_e)
{
	imu_sample_count = 0;
	gps_sample_count = 0;
	direction_n = 0.f;
	direction_e = 0.f;

	if ((end_timestamp <= kWindowDurationUs) || _imu_buffer.empty() || _gps_buffer.empty()) {
		_data_quality_flags |= sensor_attack_status_s::DATA_QUALITY_WINDOW_COVERAGE;
		return false;
	}

	const uint64_t start_timestamp = end_timestamp - kWindowDurationUs;

	if (start_timestamp <= kDynamicLagUs) {
		_data_quality_flags |= sensor_attack_status_s::DATA_QUALITY_WINDOW_COVERAGE;
		return false;
	}

	GpsEvent gps_start{};

	if (!interpolateGps(start_timestamp, gps_start)) {
		_data_quality_flags |= sensor_attack_status_s::DATA_QUALITY_WINDOW_COVERAGE;
		return false;
	}

	float full_delta_velocity[2] {};
	float full_delta_position[2] {};
	float full_coverage_s = 0.f;
	integrateActuator(start_timestamp, end_timestamp, full_delta_velocity, full_delta_position, full_coverage_s);

	if (full_coverage_s + static_cast<float>(kMaximumWindowGapUs) * 1e-6f
	    < static_cast<float>(kWindowDurationUs) * 1e-6f) {
		_data_quality_flags |= sensor_attack_status_s::DATA_QUALITY_WINDOW_COVERAGE;
		return false;
	}

	AxisGlrtAccumulator north_accumulator;
	AxisGlrtAccumulator east_accumulator;
	north_accumulator.reset();
	east_accumulator.reset();
	const float window_s = static_cast<float>(kWindowDurationUs) * 1e-6f;

	for (size_t i = 0; i < _imu_buffer.size(); ++i) {
		const ImuEvent &event = _imu_buffer[i];
		const uint64_t half_duration_us = static_cast<uint64_t>(0.5f * event.dt_s * 1e6f);
		const uint64_t center_timestamp = event.timestamp > half_duration_us ? event.timestamp - half_duration_us : 0;

		if ((center_timestamp >= start_timestamp) && (center_timestamp <= end_timestamp)
		    && (imu_sample_count < UINT16_MAX)) {
			++imu_sample_count;
		}
	}

	for (size_t i = 0; i < _gps_buffer.size(); ++i) {
		const GpsEvent &event = _gps_buffer[i];

		if ((event.timestamp > start_timestamp) && (event.timestamp <= end_timestamp)
		    && (gps_sample_count < UINT16_MAX)) {
			++gps_sample_count;
		}
	}

	if ((imu_sample_count < 100) || (gps_sample_count < 4)) {
		_data_quality_flags |= sensor_attack_status_s::DATA_QUALITY_WINDOW_COVERAGE;
		return false;
	}

	const float acceleration_weight = _param_sad_wa.get() / static_cast<float>(imu_sample_count);
	const float velocity_weight = _param_sad_wv.get() / static_cast<float>(gps_sample_count);
	const float position_weight = _param_sad_wp.get() / static_cast<float>(gps_sample_count);

	for (size_t i = 0; i < _imu_buffer.size(); ++i) {
		const ImuEvent &event = _imu_buffer[i];
		const uint64_t half_duration_us = static_cast<uint64_t>(0.5f * event.dt_s * 1e6f);
		const uint64_t center_timestamp = event.timestamp > half_duration_us ? event.timestamp - half_duration_us : 0;

		if ((center_timestamp < start_timestamp) || (center_timestamp > end_timestamp)) {
			continue;
		}

		const float tau = static_cast<float>(center_timestamp - start_timestamp)
				  / static_cast<float>(kWindowDurationUs);
		float attack_basis[AttackBasis::kSize] {};
		AttackBasis::evaluate(tau, attack_basis);
		float delayed_acceleration[2] {};

		if (!interpolateActuatorAcceleration(center_timestamp - kDynamicLagUs, delayed_acceleration)) {
			_data_quality_flags |= sensor_attack_status_s::DATA_QUALITY_WINDOW_COVERAGE;
			return false;
		}

		const float north_residual = event.acceleration_measured[0] - event.acceleration_actuator[0];
		const float east_residual = event.acceleration_measured[1] - event.acceleration_actuator[1];
		const float north_normal_basis[AxisGlrtAccumulator::kNormalDim] {
			1.f,
			event.acceleration_actuator[0],
			event.acceleration_actuator[0] - delayed_acceleration[0]
		};
		const float east_normal_basis[AxisGlrtAccumulator::kNormalDim] {
			1.f,
			event.acceleration_actuator[1],
			event.acceleration_actuator[1] - delayed_acceleration[1]
		};
		north_accumulator.addObservation(north_residual, acceleration_weight, north_normal_basis, attack_basis,\n\t\t\t\t\t\t  AxisGlrtAccumulator::kAcceleration);
		east_accumulator.addObservation(east_residual, acceleration_weight, east_normal_basis, attack_basis,\n\t\t\t\t\t       AxisGlrtAccumulator::kAcceleration);
	}

	for (size_t i = 0; i < _gps_buffer.size(); ++i) {
		const GpsEvent &event = _gps_buffer[i];

		if ((event.timestamp <= start_timestamp) || (event.timestamp > end_timestamp)) {
			continue;
		}

		const float elapsed_s = static_cast<float>(event.timestamp - start_timestamp) * 1e-6f;
		const float tau = elapsed_s / window_s;
		float delta_velocity[2] {};
		float delta_position[2] {};
		float covered_time_s = 0.f;
		integrateActuator(start_timestamp, event.timestamp, delta_velocity, delta_position, covered_time_s);

		float delayed_delta_velocity[2] {};
		float delayed_delta_position[2] {};
		float delayed_covered_time_s = 0.f;
		integrateActuator(start_timestamp - kDynamicLagUs, event.timestamp - kDynamicLagUs,
				  delayed_delta_velocity, delayed_delta_position, delayed_covered_time_s);

		if ((covered_time_s + static_cast<float>(kMaximumWindowGapUs) * 1e-6f < elapsed_s)
		    || (delayed_covered_time_s + static_cast<float>(kMaximumWindowGapUs) * 1e-6f < elapsed_s)) {
			_data_quality_flags |= sensor_attack_status_s::DATA_QUALITY_WINDOW_COVERAGE;
			return false;
		}

		float velocity_basis[AttackBasis::kSize] {};
		float position_basis[AttackBasis::kSize] {};
		AttackBasis::evaluateIntegral(tau, velocity_basis);
		AttackBasis::evaluateDoubleIntegral(tau, position_basis);

		for (size_t coefficient = 0; coefficient < AttackBasis::kSize; ++coefficient) {
			velocity_basis[coefficient] *= window_s;
			position_basis[coefficient] *= window_s * window_s;
		}

		for (size_t axis = 0; axis < 2; ++axis) {
			const float velocity_residual = event.velocity[axis] - gps_start.velocity[axis] - delta_velocity[axis];
			const float position_residual = event.position[axis] - gps_start.position[axis]
							- elapsed_s * gps_start.velocity[axis] - delta_position[axis];
			const float dynamic_velocity_basis = delta_velocity[axis] - delayed_delta_velocity[axis];
			const float dynamic_position_basis = delta_position[axis] - delayed_delta_position[axis];
			const float velocity_normal_basis[AxisGlrtAccumulator::kNormalDim] {
				elapsed_s, delta_velocity[axis], dynamic_velocity_basis
			};
			const float position_normal_basis[AxisGlrtAccumulator::kNormalDim] {
				0.5f * elapsed_s * elapsed_s, delta_position[axis], dynamic_position_basis
			};

			if (axis == 0) {
				north_accumulator.addObservation(velocity_residual, velocity_weight, velocity_normal_basis, velocity_basis,\n\t\t\t\t\t\t  AxisGlrtAccumulator::kVelocity);
				north_accumulator.addObservation(position_residual, position_weight,
								 position_normal_basis, position_basis,
								 AxisGlrtAccumulator::kPosition);

			} else {
				east_accumulator.addObservation(velocity_residual, velocity_weight, velocity_normal_basis, velocity_basis,\n\t\t\t\t\t       AxisGlrtAccumulator::kVelocity);
				east_accumulator.addObservation(position_residual, position_weight,
								position_normal_basis, position_basis,
								AxisGlrtAccumulator::kPosition);
			}
		}
	}

	AxisGlrtAccumulator::NormalPrior normal_prior{};
	const float actuator_scale_sd = fmaxf(_param_sad_as_sd.get(), 1e-3f);
	const float actuator_dynamic_sd = fmaxf(_param_sad_ad_sd.get(), 1e-3f);
	normal_prior.mean[1] = _param_sad_as_mu.get();
	normal_prior.mean[2] = _param_sad_ad_mu.get();
	normal_prior.precision[1] = 1.f / (actuator_scale_sd * actuator_scale_sd);
	normal_prior.precision[2] = 1.f / (actuator_dynamic_sd * actuator_dynamic_sd);
	north_result = north_accumulator.solve(_param_sad_reg.get(), normal_prior);
	east_result = east_accumulator.solve(_param_sad_reg.get(), normal_prior);

	if (!north_result.valid || !east_result.valid) {
		_data_quality_flags |= sensor_attack_status_s::DATA_QUALITY_SOLVER;
		return false;
	}

	float terminal_basis[AttackBasis::kSize] {};
	AttackBasis::evaluateDoubleIntegral(1.f, terminal_basis);

	for (size_t coefficient = 0; coefficient < AttackBasis::kSize; ++coefficient) {
		const float scale = window_s * window_s * terminal_basis[coefficient];
		direction_n += scale * north_result.attack_coefficients[coefficient];
		direction_e += scale * east_result.attack_coefficients[coefficient];
	}

	return PX4_ISFINITE(direction_n) && PX4_ISFINITE(direction_e);
}

void SensorAttackDetector::runPendingEvaluation(hrt_abstime run_start)
{
	if (!_evaluation_pending) {
		return;
	}

	if (!_pending_gps_valid) {
		resetSequentialDetector();
		publishStatus(_pending_evaluation_timestamp, false, nullptr, nullptr, 0, 0, 0.f, 0.f, run_start);
		_last_evaluation_timestamp = _pending_evaluation_timestamp;
		_evaluation_pending = false;
		return;
	}

	const bool imu_caught_up = !_imu_buffer.empty()
				   && (_imu_buffer.back().timestamp + kMaximumWindowGapUs >= _pending_evaluation_timestamp);
	const bool wait_expired = (_last_imu_sample_timestamp > _pending_evaluation_timestamp)
				  && ((_last_imu_sample_timestamp - _pending_evaluation_timestamp) > kAlignmentWaitUs);

	if (!imu_caught_up && !wait_expired) {
		return;
	}

	AxisGlrtAccumulator::Result north_result{};
	AxisGlrtAccumulator::Result east_result{};
	uint16_t imu_sample_count = 0;
	uint16_t gps_sample_count = 0;
	float direction_n = 0.f;
	float direction_e = 0.f;
	perf_begin(_solver_perf);
	const bool window_valid = evaluateWindow(_pending_evaluation_timestamp, north_result, east_result,
				  imu_sample_count, gps_sample_count, direction_n, direction_e);
	perf_end(_solver_perf);

	const float warmup_s = _origin_timestamp != 0
			       ? static_cast<float>(_pending_evaluation_timestamp - _origin_timestamp) * 1e-6f : 0.f;
	const bool detector_valid = window_valid && _param_sad_en.get()
				    && (warmup_s >= _param_sad_warmup.get());

	if (detector_valid) {
		const float glrt_score = north_result.glrt + east_result.glrt;
		const float standard_deviation = fmaxf(_param_sad_glrt_sd.get(), 1e-6f);
		const float normalized_score = (glrt_score - _param_sad_glrt_mu.get()) / standard_deviation;
		updateSequentialDetector(normalized_score);

	} else {
		resetSequentialDetector();
	}

	publishStatus(_pending_evaluation_timestamp, detector_valid,
		      window_valid ? &north_result : nullptr,
		      window_valid ? &east_result : nullptr,
		      imu_sample_count, gps_sample_count, direction_n, direction_e, run_start);
	_last_evaluation_timestamp = _pending_evaluation_timestamp;
	_evaluation_pending = false;
}

void SensorAttackDetector::updateSequentialDetector(float normalized_score)
{
	if (!PX4_ISFINITE(normalized_score)) {
		resetSequentialDetector();
		return;
	}

	_cusum = fmaxf(0.f, _cusum + normalized_score - _param_sad_cus_dr.get());
	_cusum = fminf(_cusum, 1e6f);

	if (_cusum > _param_sad_thresh.get()) {
		if (_consecutive_count < UINT16_MAX) {
			++_consecutive_count;
		}

	} else {
		_consecutive_count = 0;
	}
}

void SensorAttackDetector::resetSequentialDetector()
{
	_cusum = 0.f;
	_consecutive_count = 0;
}

uint32_t SensorAttackDetector::sampleAgeUs(uint64_t now, uint64_t timestamp)
{
	if ((timestamp == 0) || (now <= timestamp)) {
		return timestamp == 0 ? UINT32_MAX : 0;
	}

	const uint64_t age = now - timestamp;
	return age > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(age);
}

void SensorAttackDetector::publishStatus(uint64_t timestamp_sample, bool valid,
		const AxisGlrtAccumulator::Result *north_result,
		const AxisGlrtAccumulator::Result *east_result,
		uint16_t imu_sample_count, uint16_t gps_sample_count,
		float direction_n, float direction_e, hrt_abstime run_start)
{
	sensor_attack_status_s status{};
	status.timestamp_sample = timestamp_sample;
	status.actuator_source = static_cast<uint8_t>(_param_sad_act_src.get());
	status.valid = valid;
	status.gps_valid = _gps_valid;
	status.attitude_valid = _attitude_valid;
	status.actuator_valid = _actuator_valid;
	status.imu_valid = _imu_valid;
	status.selected_window_s = static_cast<float>(kWindowDurationUs) * 1e-6f;
	status.threshold = _param_sad_thresh.get();
	status.cusum_score = _cusum;
	status.consecutive_count = _consecutive_count;
	status.imu_sample_count = imu_sample_count;
	status.gps_sample_count = gps_sample_count;
	status.data_quality_flags = _data_quality_flags;

	if ((north_result != nullptr) && (east_result != nullptr)) {
		status.glrt_score_n = north_result->glrt;
		status.glrt_score_e = east_result->glrt;
		status.glrt_score = north_result->glrt + east_result->glrt;
		status.cost_null_n = north_result->cost_null;
		status.cost_null_e = east_result->cost_null;
		status.cost_attack_n = north_result->cost_attack;
		status.cost_attack_e = east_result->cost_attack;
		status.normalized_score = (status.glrt_score - _param_sad_glrt_mu.get())
					  / fmaxf(_param_sad_glrt_sd.get(), 1e-6f);
		status.normal_bias_n = north_result->normal_parameters[0];
		status.normal_bias_e = east_result->normal_parameters[0];
		status.normal_scale_n = north_result->normal_parameters[1];
		status.normal_scale_e = east_result->normal_parameters[1];
		status.normal_dynamic_n = north_result->normal_parameters[2];
		status.normal_dynamic_e = east_result->normal_parameters[2];
		status.normal_rms_accel_n = north_result->normal_residual_rms[AxisGlrtAccumulator::kAcceleration];
		status.normal_rms_accel_e = east_result->normal_residual_rms[AxisGlrtAccumulator::kAcceleration];
		status.normal_rms_velocity_n = north_result->normal_residual_rms[AxisGlrtAccumulator::kVelocity];
		status.normal_rms_velocity_e = east_result->normal_residual_rms[AxisGlrtAccumulator::kVelocity];
		status.normal_rms_position_n = north_result->normal_residual_rms[AxisGlrtAccumulator::kPosition];
		status.normal_rms_position_e = east_result->normal_residual_rms[AxisGlrtAccumulator::kPosition];
	}

	const float direction_norm = sqrtf(direction_n * direction_n + direction_e * direction_e);
	status.direction_norm = direction_norm;

	const uint16_t required_consecutive =
		static_cast<uint16_t>(math::max(_param_sad_consec.get(), static_cast<int32_t>(1)));
	status.attack_detected = valid && (_consecutive_count >= required_consecutive);

	if (status.attack_detected && PX4_ISFINITE(direction_norm) && (direction_norm > 1e-4f)) {
		status.direction_n = direction_n / direction_norm;
		status.direction_e = direction_e / direction_norm;
		status.direction_valid = true;
	}

	const bool inputs_valid = _gps_valid && _attitude_valid && _actuator_valid && _imu_valid;
	const bool origin_ready = (_origin_timestamp != 0) && (timestamp_sample >= _origin_timestamp);
	const bool warmup_complete = origin_ready
				     && (static_cast<float>(timestamp_sample - _origin_timestamp) * 1e-6f
					 >= _param_sad_warmup.get());

	if (!_param_sad_en.get()) {
		status.state = sensor_attack_status_s::STATE_INIT;

	} else if (!inputs_valid || !origin_ready) {
		status.state = sensor_attack_status_s::STATE_INIT;

	} else if (!valid || !warmup_complete) {
		status.state = sensor_attack_status_s::STATE_WARMUP;

	} else if (status.attack_detected) {
		status.state = sensor_attack_status_s::STATE_ALERT;

	} else {
		status.state = sensor_attack_status_s::STATE_MONITOR;
	}

	if (!_imu_buffer.empty()) {
		const ImuEvent &latest_imu = _imu_buffer.back();
		status.accel_meas_n = latest_imu.acceleration_measured[0];
		status.accel_meas_e = latest_imu.acceleration_measured[1];
		status.accel_actuator_n = latest_imu.acceleration_actuator[0];
		status.accel_actuator_e = latest_imu.acceleration_actuator[1];
	}

	const uint64_t now = hrt_absolute_time();
	status.imu_age_us = sampleAgeUs(now, _last_imu_sample_timestamp);
	status.gps_age_us = sampleAgeUs(now, _last_gps_message_timestamp);
	status.attitude_age_us = sampleAgeUs(now, _last_attitude_timestamp);
	status.actuator_age_us = sampleAgeUs(now, _last_actuator_timestamp);
	status.computation_time_us = sampleAgeUs(now, run_start);
	status.timestamp = now;

	_status_pub.publish(status);
	_last_status = status;
	_data_quality_flags = 0;
}

int SensorAttackDetector::print_status()
{
	PX4_INFO("state: %u valid: %s alert: %s", _last_status.state,
		 _last_status.valid ? "yes" : "no",
		 _last_status.attack_detected ? "yes" : "no");
	PX4_INFO("GLRT: %.3f CUSUM: %.3f, samples IMU/GPS: %u/%u",
		 (double)_last_status.glrt_score, (double)_last_status.cusum_score,
		 _last_status.imu_sample_count, _last_status.gps_sample_count);
	PX4_INFO("buffers pending/IMU/GPS/att/act: %zu/%zu/%zu/%zu/%zu",
		 _pending_imu_buffer.size(), _imu_buffer.size(), _gps_buffer.size(),
		 _attitude_buffer.size(), _actuator_buffer.size());
	perf_print_counter(_cycle_perf);
	perf_print_counter(_solver_perf);
	return 0;
}

int SensorAttackDetector::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SensorAttackDetector::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Passive actuator-command-anchored detector for asynchronous common horizontal
position, velocity, and acceleration sensor attacks.

The module only subscribes, detects, records, and publishes
`sensor_attack_status`. It does not modify EKF, control, Commander, or failsafe
behavior and does not recover state.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sensor_attack_detector", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the detector");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int sensor_attack_detector_main(int argc, char *argv[])
{
	return SensorAttackDetector::main(argc, argv);
}
