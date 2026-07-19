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

#include <px4_platform_common/log.h>

using namespace time_literals;

SensorAttackDetector::SensorAttackDetector() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

SensorAttackDetector::~SensorAttackDetector()
{
	ScheduleClear();
}

int SensorAttackDetector::task_spawn(int argc, char *argv[])
{
	SensorAttackDetector *instance = new SensorAttackDetector();

	if (instance == nullptr) {
		PX4_ERR("allocation failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;
	instance->start();
	return PX4_OK;
}

void SensorAttackDetector::start()
{
	PX4_INFO("Hello PX4");
	ScheduleOnInterval(1_s);
}

void SensorAttackDetector::request_stop()
{
	ModuleBase<SensorAttackDetector>::request_stop();
	ScheduleNow();
}

void SensorAttackDetector::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// Online sensor attack detection will be implemented here.
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
Scaffold module for online sensor attack detection on PX4.

Starting the module prints `Hello PX4` and schedules a low-priority work item.
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
