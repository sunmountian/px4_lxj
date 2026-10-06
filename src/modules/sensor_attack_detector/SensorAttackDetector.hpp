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

#pragma once

#include "EventBuffer.hpp"
#include "GlrtSolver.hpp"

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_attack_status.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_imu.h>

using namespace time_literals;

/**
 * Passive actuator-command-anchored detector for asynchronous common PVA
 * sensor attacks.
 *
 * The module has no control-path outputs: it only subscribes, detects, logs
 * through uORB, and publishes sensor_attack_status.
 */
class SensorAttackDetector final : public ModuleBase<SensorAttackDetector>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	SensorAttackDetector();
	~SensorAttackDetector() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	void request_stop() override;
	int print_status() override;

private:
	static constexpr uint64_t kImuBinDurationUs{20_ms};
	static constexpr uint64_t kWindowDurationUs{8_s};
	static constexpr uint64_t kBufferRetentionUs{12_s};
	static constexpr uint64_t kAlignmentWaitUs{100_ms};
	static constexpr uint64_t kMaximumAttitudeGapUs{100_ms};
	static constexpr uint64_t kMaximumActuatorAgeUs{100_ms};
	static constexpr uint64_t kMaximumWindowGapUs{150_ms};
	static constexpr uint64_t kDynamicLagUs{200_ms};
	static constexpr size_t kMotorCount{4};

	static constexpr size_t kPendingImuCapacity{128};
	static constexpr size_t kAttitudeCapacity{128};
	static constexpr size_t kActuatorCapacity{128};
	static constexpr size_t kProcessedImuCapacity{720};
	static constexpr size_t kGpsCapacity{160};

	struct PendingImuEvent {
		uint64_t timestamp{0};
		float delta_velocity[3] {};
		float dt_s{0.f};
		uint8_t clipping{0};
	};

	struct AttitudeEvent {
		uint64_t timestamp{0};
		float q[4] {};
	};

	struct ActuatorEvent {
		uint64_t timestamp{0};
		float thrust_indicator{0.f};
	};

	struct ImuEvent {
		uint64_t timestamp{0};
		float dt_s{0.f};
		float acceleration_measured[2] {};
		float acceleration_actuator[2] {};
	};

	struct GpsEvent {
		uint64_t timestamp{0};
		float position[2] {};
		float velocity[2] {};
	};

	struct ImuBin {
		bool active{false};
		uint64_t index{0};
		uint64_t last_timestamp{0};
		float total_dt_s{0.f};
		float measured_delta_velocity[2] {};
		float actuator_delta_velocity[2] {};
	};

	void Run() override;
	void parametersUpdate();
	void ingestAttitude();
	void ingestActuator();
	void ingestGps();
	void ingestImu();
	void processPendingImu();
	void aggregateAlignedImu(const PendingImuEvent &imu, const matrix::Quatf &q_nb,
				 float thrust_indicator);
	void flushImuBin();
	void trimLongBuffers();

	bool interpolateAttitude(uint64_t timestamp, matrix::Quatf &q_nb) const;
	bool findActuator(uint64_t timestamp, float &thrust_indicator) const;
	bool interpolateGps(uint64_t timestamp, GpsEvent &gps) const;
	bool integrateActuator(uint64_t start_timestamp, uint64_t end_timestamp,
			       float (&delta_velocity)[2], float (&delta_position)[2],
			       float &covered_time_s) const;
	bool interpolateActuatorAcceleration(uint64_t timestamp, float (&acceleration)[2]) const;

	bool evaluateWindow(uint64_t end_timestamp, AxisGlrtAccumulator::Result &north_result,
			    AxisGlrtAccumulator::Result &east_result, uint16_t &imu_sample_count,
			    uint16_t &gps_sample_count, float &direction_n, float &direction_e);
	void runPendingEvaluation(hrt_abstime run_start);
	void updateSequentialDetector(float normalized_score);
	void resetSequentialDetector();
	void publishStatus(uint64_t timestamp_sample, bool valid,
			   const AxisGlrtAccumulator::Result *north_result,
			   const AxisGlrtAccumulator::Result *east_result,
			   uint16_t imu_sample_count, uint16_t gps_sample_count,
			   float direction_n, float direction_e, hrt_abstime run_start);

	static uint32_t sampleAgeUs(uint64_t now, uint64_t timestamp);

	uORB::SubscriptionCallbackWorkItem _vehicle_imu_sub{this, ORB_ID(vehicle_imu)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _sensor_gps_sub{ORB_ID(sensor_gps)};
	uORB::Subscription _actuator_motors_sub{ORB_ID(actuator_motors)};
	uORB::Subscription _actuator_outputs_sub{ORB_ID(actuator_outputs)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Publication<sensor_attack_status_s> _status_pub{ORB_ID(sensor_attack_status)};

	EventBuffer<PendingImuEvent, kPendingImuCapacity> _pending_imu_buffer;
	EventBuffer<AttitudeEvent, kAttitudeCapacity> _attitude_buffer;
	EventBuffer<ActuatorEvent, kActuatorCapacity> _actuator_buffer;
	EventBuffer<ImuEvent, kProcessedImuCapacity> _imu_buffer;
	EventBuffer<GpsEvent, kGpsCapacity> _gps_buffer;

	MapProjection _gps_projection{};
	ImuBin _imu_bin{};
	sensor_attack_status_s _last_status{};

	uint64_t _origin_timestamp{0};
	uint64_t _last_attitude_timestamp{0};
	uint64_t _last_actuator_timestamp{0};
	uint64_t _last_gps_message_timestamp{0};
	uint64_t _last_imu_sample_timestamp{0};
	uint64_t _last_evaluation_timestamp{0};
	uint64_t _pending_evaluation_timestamp{0};

	uint32_t _data_quality_flags{0};
	float _cusum{0.f};
	uint16_t _consecutive_count{0};
	bool _evaluation_pending{false};
	bool _pending_gps_valid{false};
	bool _gps_valid{false};
	bool _attitude_valid{false};
	bool _actuator_valid{false};
	bool _imu_valid{false};
	bool _callback_registered{false};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _solver_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": GLRT")};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::SAD_EN>) _param_sad_en,
		(ParamFloat<px4::params::SAD_RATE>) _param_sad_rate,
		(ParamFloat<px4::params::SAD_WARMUP>) _param_sad_warmup,
		(ParamFloat<px4::params::SAD_WA>) _param_sad_wa,
		(ParamFloat<px4::params::SAD_WV>) _param_sad_wv,
		(ParamFloat<px4::params::SAD_WP>) _param_sad_wp,
		(ParamInt<px4::params::SAD_ACT_SRC>) _param_sad_act_src,
		(ParamFloat<px4::params::SAD_THR_GAIN>) _param_sad_thr_gain,
		(ParamFloat<px4::params::SAD_GPS_EPH>) _param_sad_gps_eph,
		(ParamFloat<px4::params::SAD_REG>) _param_sad_reg,
		(ParamFloat<px4::params::SAD_AS_MU>) _param_sad_as_mu,
		(ParamFloat<px4::params::SAD_AS_SD>) _param_sad_as_sd,
		(ParamFloat<px4::params::SAD_AD_MU>) _param_sad_ad_mu,
		(ParamFloat<px4::params::SAD_AD_SD>) _param_sad_ad_sd,
		(ParamFloat<px4::params::SAD_GLRT_MU>) _param_sad_glrt_mu,
		(ParamFloat<px4::params::SAD_GLRT_SD>) _param_sad_glrt_sd,
		(ParamFloat<px4::params::SAD_CUS_DR>) _param_sad_cus_dr,
		(ParamFloat<px4::params::SAD_THRESH>) _param_sad_thresh,
		(ParamInt<px4::params::SAD_CONSEC>) _param_sad_consec
	)
};
