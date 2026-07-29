############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name PX4 nor the names of its contributors may be used to
#    endorse or promote products derived from this software without
#    specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED.
#
############################################################################

px4_add_library(sensor_attack_detector_core
	GlrtSolver.cpp
	GlrtSolver.hpp
	AttackBasis.hpp
	EventBuffer.hpp
)

px4_add_module(
	MODULE modules__sensor_attack_detector
	MAIN sensor_attack_detector
	SRCS
		SensorAttackDetector.cpp
		SensorAttackDetector.hpp
	DEPENDS
		geo
		px4_work_queue
		sensor_attack_detector_core
)

px4_add_unit_gtest(
	SRC SensorAttackDetectorTest.cpp
	LINKLIBS sensor_attack_detector_core
)
