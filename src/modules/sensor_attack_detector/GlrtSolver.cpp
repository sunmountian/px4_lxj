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

#include <math.h>
#include <string.h>

void AxisGlrtAccumulator::reset()
{
	memset(_normal_hessian, 0, sizeof(_normal_hessian));
	memset(_normal_rhs, 0, sizeof(_normal_rhs));
	memset(_joint_hessian, 0, sizeof(_joint_hessian));
	memset(_joint_rhs, 0, sizeof(_joint_rhs));
	memset(_group_normal_hessian, 0, sizeof(_group_normal_hessian));
	memset(_group_normal_rhs, 0, sizeof(_group_normal_rhs));
	memset(_group_squared_observation, 0, sizeof(_group_squared_observation));
	memset(_group_observation_count, 0, sizeof(_group_observation_count));
	_squared_observation = 0.f;
	_observation_count = 0;
}

void AxisGlrtAccumulator::addObservation(float observation, float weight,
		const float (&normal_basis)[kNormalDim],
		const float (&attack_basis)[kAttackDim], ResidualGroup group)
{
	if (!isfinite(observation) || !isfinite(weight) || !(weight > 0.f)) {
		return;
	}

	float row[kJointDim] {};

	for (size_t i = 0; i < kNormalDim; ++i) {
		if (!isfinite(normal_basis[i])) {
			return;
		}

		row[i] = normal_basis[i];
	}

	for (size_t i = 0; i < kAttackDim; ++i) {
		if (!isfinite(attack_basis[i])) {
			return;
		}

		row[kNormalDim + i] = attack_basis[i];
	}

	_squared_observation += weight * observation * observation;

	const size_t group_index = static_cast<size_t>(group);

	if (group_index >= kResidualGroupCount) {
		return;
	}

	_group_squared_observation[group_index] += observation * observation;

	if (_group_observation_count[group_index] < UINT16_MAX) {
		++_group_observation_count[group_index];
	}

	for (size_t i = 0; i < kNormalDim; ++i) {
		_group_normal_rhs[group_index][i] += normal_basis[i] * observation;

		for (size_t j = 0; j < kNormalDim; ++j) {
			_group_normal_hessian[group_index][i][j] += normal_basis[i] * normal_basis[j];
		}
	}

	for (size_t i = 0; i < kNormalDim; ++i) {
		_normal_rhs[i] += weight * normal_basis[i] * observation;

		for (size_t j = 0; j < kNormalDim; ++j) {
			_normal_hessian[i][j] += weight * normal_basis[i] * normal_basis[j];
		}
	}

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

AxisGlrtAccumulator::Result AxisGlrtAccumulator::solve(float regularization, const NormalPrior &prior) const
{
	Result result{};
	result.observation_count = _observation_count;

	if ((_observation_count < kJointDim) || !isfinite(_squared_observation)) {
		return result;
	}

	float normal_hessian[kNormalDim][kNormalDim] {};
	float normal_rhs[kNormalDim] {};
	memcpy(normal_hessian, _normal_hessian, sizeof(normal_hessian));
	memcpy(normal_rhs, _normal_rhs, sizeof(normal_rhs));
	float prior_constant = 0.f;

	for (size_t i = 0; i < kNormalDim; ++i) {
		if (!isfinite(prior.mean[i]) || !isfinite(prior.precision[i]) || (prior.precision[i] < 0.f)) {
			return result;
		}

		normal_hessian[i][i] += prior.precision[i];
		normal_rhs[i] += prior.precision[i] * prior.mean[i];
		prior_constant += prior.precision[i] * prior.mean[i] * prior.mean[i];
	}

	float normal_solution[kNormalDim] {};
	float normal_minimum_diagonal = 0.f;

	if (!choleskySolve(normal_hessian, normal_rhs, normal_solution, normal_minimum_diagonal)) {
		return result;
	}

	float normal_reduction = 0.f;

	for (size_t i = 0; i < kNormalDim; ++i) {
		normal_reduction += normal_rhs[i] * normal_solution[i];
	}

	float cost_null = _squared_observation + prior_constant - normal_reduction;

	float hessian[kJointDim][kJointDim] {};
	float rhs[kJointDim] {};
	memcpy(hessian, _joint_hessian, sizeof(hessian));
	memcpy(rhs, _joint_rhs, sizeof(rhs));

	for (size_t i = 0; i < kNormalDim; ++i) {
		hessian[i][i] += prior.precision[i];
		rhs[i] += prior.precision[i] * prior.mean[i];
	}

	AttackBasis::addSecondDifferenceRegularizer(hessian, regularization, kNormalDim);

	float attack_trace = 0.f;

	for (size_t i = 0; i < kAttackDim; ++i) {
		attack_trace += fabsf(hessian[kNormalDim + i][kNormalDim + i]);
	}

	const float gauge_strength = fmaxf(1e-6f, 1e-6f * attack_trace / static_cast<float>(kAttackDim));
	AttackBasis::addConstantModeGauge(hessian, gauge_strength, kNormalDim);

	float joint_solution[kJointDim] {};
	float minimum_diagonal = 0.f;

	if (!choleskySolve(hessian, rhs, joint_solution, minimum_diagonal)) {
		return result;
	}

	float joint_reduction = 0.f;

	for (size_t i = 0; i < kJointDim; ++i) {
		joint_reduction += rhs[i] * joint_solution[i];
	}

	float cost_attack = _squared_observation + prior_constant - joint_reduction;

	if (!isfinite(cost_null) || !isfinite(cost_attack)) {
		return result;
	}

	const float tolerance = 1e-4f * fmaxf(1.f, _squared_observation + prior_constant);

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
	result.minimum_cholesky_diagonal = minimum_diagonal;

	for (size_t i = 0; i < kNormalDim; ++i) {
		result.normal_parameters[i] = normal_solution[i];
	}

	for (size_t group = 0; group < kResidualGroupCount; ++group) {
		if (_group_observation_count[group] == 0) {
			result.normal_residual_rms[group] = NAN;
			continue;
		}

		float group_cost = _group_squared_observation[group];

		for (size_t i = 0; i < kNormalDim; ++i) {
			group_cost -= 2.f * normal_solution[i] * _group_normal_rhs[group][i];

			for (size_t j = 0; j < kNormalDim; ++j) {
				group_cost += normal_solution[i]
					      * _group_normal_hessian[group][i][j]
					      * normal_solution[j];
			}
		}

		const float group_tolerance = 1e-5f * fmaxf(1.f, _group_squared_observation[group]);

		if ((group_cost < 0.f) && (group_cost > -group_tolerance)) {
			group_cost = 0.f;
		}

		result.normal_residual_rms[group] = group_cost >= 0.f
						 ? sqrtf(group_cost / static_cast<float>(_group_observation_count[group]))
						 : NAN;
	}

	for (size_t i = 0; i < kAttackDim; ++i) {
		result.attack_coefficients[i] = joint_solution[kNormalDim + i];
	}

	return result;
}
