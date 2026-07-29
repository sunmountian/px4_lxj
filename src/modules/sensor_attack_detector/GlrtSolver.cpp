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

#include "GlrtSolver.hpp"

#include <float.h>
#include <math.h>
#include <string.h>

void AxisGlrtAccumulator::reset()
{
	_normal_hessian = 0.f;
	_normal_rhs = 0.f;
	_squared_observation = 0.f;
	_observation_count = 0;
	memset(_joint_hessian, 0, sizeof(_joint_hessian));
	memset(_joint_rhs, 0, sizeof(_joint_rhs));
}

void AxisGlrtAccumulator::addObservation(float observation, float weight, float normal_basis,
		const float (&attack_basis)[kAttackDim])
{
	if (!isfinite(observation) || !isfinite(weight) || !(weight > 0.f) || !isfinite(normal_basis)) {
		return;
	}

	float row[kJointDim] {};
	row[0] = normal_basis;

	for (size_t i = 0; i < kAttackDim; ++i) {
		if (!isfinite(attack_basis[i])) {
			return;
		}

		row[1 + i] = attack_basis[i];
	}

	_normal_hessian += weight * normal_basis * normal_basis;
	_normal_rhs += weight * normal_basis * observation;
	_squared_observation += weight * observation * observation;

	for (size_t i = 0; i < kJointDim; ++i) {
		_joint_rhs[i] += weight * row[i] * observation;

		for (size_t j = 0; j < kJointDim; ++j) {
			_joint_hessian[i][j] += weight * row[i] * row[j];
		}
	}

	if (_observation_count < UINT16_MAX) {
		++_observation_count;
	}
}

AxisGlrtAccumulator::Result AxisGlrtAccumulator::solve(float regularization) const
{
	Result result{};
	result.observation_count = _observation_count;

	if ((_observation_count < kJointDim) || !isfinite(_normal_hessian) || !(_normal_hessian > 1e-8f)
	    || !isfinite(_normal_rhs) || !isfinite(_squared_observation)) {
		return result;
	}

	float hessian[kJointDim][kJointDim] {};
	memcpy(hessian, _joint_hessian, sizeof(hessian));
	AttackBasis::addSecondDifferenceRegularizer(hessian, regularization);

	float attack_trace = 0.f;

	for (size_t i = 0; i < kAttackDim; ++i) {
		attack_trace += fabsf(hessian[1 + i][1 + i]);
	}

	const float gauge_strength = fmaxf(1e-6f, 1e-6f * attack_trace / static_cast<float>(kAttackDim));
	AttackBasis::addConstantModeGauge(hessian, gauge_strength);

	float solution[kJointDim] {};
	float minimum_diagonal = 0.f;

	if (!choleskySolve(hessian, _joint_rhs, solution, minimum_diagonal)) {
		return result;
	}

	const float normal_parameter = _normal_rhs / _normal_hessian;
	float cost_null = _squared_observation - _normal_rhs * normal_parameter;
	float joint_reduction = 0.f;

	for (size_t i = 0; i < kJointDim; ++i) {
		joint_reduction += _joint_rhs[i] * solution[i];
	}

	float cost_attack = _squared_observation - joint_reduction;

	if (!isfinite(cost_null) || !isfinite(cost_attack)) {
		return result;
	}

	const float tolerance = 1e-4f * fmaxf(1.f, _squared_observation);

	if ((cost_null < 0.f) && (cost_null > -tolerance)) {
		cost_null = 0.f;
	}

	if ((cost_attack < 0.f) && (cost_attack > -tolerance)) {
		cost_attack = 0.f;
	}

	if ((cost_null < 0.f) || (cost_attack < 0.f)) {
		return result;
	}

	result.valid = true;
	result.cost_null = cost_null;
	result.cost_attack = cost_attack;
	result.glrt = fmaxf(0.f, cost_null - cost_attack);
	result.normal_parameter = normal_parameter;
	result.minimum_cholesky_diagonal = minimum_diagonal;

	for (size_t i = 0; i < kAttackDim; ++i) {
		result.attack_coefficients[i] = solution[1 + i];
	}

	return result;
}

bool AxisGlrtAccumulator::choleskySolve(const float input[kJointDim][kJointDim],
					const float rhs[kJointDim], float solution[kJointDim],
					float &minimum_diagonal)
{
	float lower[kJointDim][kJointDim] {};
	minimum_diagonal = FLT_MAX;

	for (size_t i = 0; i < kJointDim; ++i) {
		for (size_t j = 0; j <= i; ++j) {
			float sum = input[i][j];

			for (size_t k = 0; k < j; ++k) {
				sum -= lower[i][k] * lower[j][k];
			}

			if (i == j) {
				if (!isfinite(sum) || !(sum > 1e-10f)) {
					return false;
				}

				lower[i][j] = sqrtf(sum);
				minimum_diagonal = fminf(minimum_diagonal, lower[i][j]);

			} else {
				lower[i][j] = sum / lower[j][j];
			}
		}
	}

	float intermediate[kJointDim] {};

	for (size_t i = 0; i < kJointDim; ++i) {
		float sum = rhs[i];

		for (size_t j = 0; j < i; ++j) {
			sum -= lower[i][j] * intermediate[j];
		}

		intermediate[i] = sum / lower[i][i];
	}

	for (int i = static_cast<int>(kJointDim) - 1; i >= 0; --i) {
		float sum = intermediate[i];

		for (size_t j = static_cast<size_t>(i) + 1; j < kJointDim; ++j) {
			sum -= lower[j][i] * solution[j];
		}

		solution[i] = sum / lower[i][i];

		if (!isfinite(solution[i])) {
			return false;
		}
	}

	return true;
}
