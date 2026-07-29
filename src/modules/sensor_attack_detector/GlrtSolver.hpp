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
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED.
 *
 ****************************************************************************/

#pragma once

#include "AttackBasis.hpp"

#include <stddef.h>
#include <stdint.h>

class AxisGlrtAccumulator
{
public:
	static constexpr size_t kAttackDim{AttackBasis::kSize};
	static constexpr size_t kJointDim{kAttackDim + 1};

	struct Result {
		bool valid{false};
		float glrt{0.f};
		float cost_null{0.f};
		float cost_attack{0.f};
		float normal_parameter{0.f};
		float attack_coefficients[kAttackDim] {};
		float minimum_cholesky_diagonal{0.f};
		uint16_t observation_count{0};
	};

	void reset();
	void addObservation(float observation, float weight, float normal_basis,
			    const float (&attack_basis)[kAttackDim]);
	Result solve(float regularization) const;
	uint16_t observationCount() const { return _observation_count; }

private:
	static bool choleskySolve(const float input[kJointDim][kJointDim],
				  const float rhs[kJointDim], float solution[kJointDim],
				  float &minimum_diagonal);

	float _normal_hessian{0.f};
	float _normal_rhs{0.f};
	float _joint_hessian[kJointDim][kJointDim] {};
	float _joint_rhs[kJointDim] {};
	float _squared_observation{0.f};
	uint16_t _observation_count{0};
};
