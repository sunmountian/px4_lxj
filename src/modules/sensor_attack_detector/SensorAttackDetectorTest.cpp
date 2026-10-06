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

#include "AttackBasis.hpp"
#include "GlrtSolver.hpp"

#include <gtest/gtest.h>

#include <math.h>

namespace
{

constexpr float kWindowSeconds{8.f};

float numericalIntegral(size_t basis_index, float tau)
{
	constexpr int steps = 20000;
	const float step = tau / static_cast<float>(steps);
	float sum = 0.f;

	for (int i = 0; i < steps; ++i) {
		const float sample = (static_cast<float>(i) + 0.5f) * step;
		float basis[AttackBasis::kSize] {};
		AttackBasis::evaluate(sample, basis);
		sum += basis[basis_index] * step;
	}

	return sum;
}

float numericalDoubleIntegral(size_t basis_index, float tau)
{
	constexpr int steps = 20000;
	const float step = tau / static_cast<float>(steps);
	float sum = 0.f;

	for (int i = 0; i < steps; ++i) {
		const float sample = (static_cast<float>(i) + 0.5f) * step;
		float basis[AttackBasis::kSize] {};
		AttackBasis::evaluate(sample, basis);
		sum += (tau - sample) * basis[basis_index] * step;
	}

	return sum;
}

} // namespace

TEST(AttackBasis, PartitionOfUnity)
{
	for (int i = -10; i <= 110; ++i) {
		float basis[AttackBasis::kSize] {};
		AttackBasis::evaluate(static_cast<float>(i) / 100.f, basis);
		float sum = 0.f;

		for (float value : basis) {
			EXPECT_GE(value, 0.f);
			EXPECT_LE(value, 1.f);
			sum += value;
		}

		EXPECT_NEAR(sum, 1.f, 1e-6f);
	}
}

TEST(AttackBasis, ContinuousAtInternalKnots)
{
	constexpr float epsilon = 1e-6f;

	for (size_t knot = 1; knot < AttackBasis::kSegmentCount; ++knot) {
		const float tau = static_cast<float>(knot) / static_cast<float>(AttackBasis::kSegmentCount);
		float left[AttackBasis::kSize] {};
		float right[AttackBasis::kSize] {};
		AttackBasis::evaluate(tau - epsilon, left);
		AttackBasis::evaluate(tau + epsilon, right);

		for (size_t i = 0; i < AttackBasis::kSize; ++i) {
			EXPECT_NEAR(left[i], right[i], 2e-5f);
		}
	}
}

TEST(AttackBasis, AnalyticFirstIntegralMatchesNumericalIntegration)
{
	for (float tau : {0.07f, 0.2f, 0.53f, 0.81f, 1.f}) {
		float integral[AttackBasis::kSize] {};
		AttackBasis::evaluateIntegral(tau, integral);

		for (size_t i = 0; i < AttackBasis::kSize; ++i) {
			EXPECT_NEAR(integral[i], numericalIntegral(i, tau), 2e-5f);
		}
	}
}

TEST(AttackBasis, AnalyticDoubleIntegralMatchesNumericalIntegration)
{
	for (float tau : {0.07f, 0.2f, 0.53f, 0.81f, 1.f}) {
		float integral[AttackBasis::kSize] {};
		AttackBasis::evaluateDoubleIntegral(tau, integral);

		for (size_t i = 0; i < AttackBasis::kSize; ++i) {
			EXPECT_NEAR(integral[i], numericalDoubleIntegral(i, tau), 2e-5f);
		}
	}
}

TEST(AttackBasis, PhysicalTimeScaling)
{
	for (float tau : {0.1f, 0.4f, 0.9f, 1.f}) {
		float first[AttackBasis::kSize] {};
		float second[AttackBasis::kSize] {};
		AttackBasis::evaluateIntegral(tau, first);
		AttackBasis::evaluateDoubleIntegral(tau, second);
		float first_sum = 0.f;
		float second_sum = 0.f;

		for (size_t i = 0; i < AttackBasis::kSize; ++i) {
			first_sum += kWindowSeconds * first[i];
			second_sum += kWindowSeconds * kWindowSeconds * second[i];
		}

		const float physical_time = kWindowSeconds * tau;
		EXPECT_NEAR(first_sum, physical_time, 2e-5f);
		EXPECT_NEAR(second_sum, 0.5f * physical_time * physical_time, 2e-4f);
	}
}

TEST(AttackBasis, SecondDifferenceRegularization)
{
	float matrix[AttackBasis::kSize + 1][AttackBasis::kSize + 1] {};
	AttackBasis::addSecondDifferenceRegularizer(matrix, 1.f);
	float constant_penalty = 0.f;
	float linear_penalty = 0.f;
	float curved_penalty = 0.f;

	for (size_t i = 0; i < AttackBasis::kSize; ++i) {
		for (size_t j = 0; j < AttackBasis::kSize; ++j) {
			const float constant_i = 1.f;
			const float constant_j = 1.f;
			const float linear_i = static_cast<float>(i);
			const float linear_j = static_cast<float>(j);
			const float curved_i = static_cast<float>(i * i);
			const float curved_j = static_cast<float>(j * j);
			constant_penalty += constant_i * matrix[1 + i][1 + j] * constant_j;
			linear_penalty += linear_i * matrix[1 + i][1 + j] * linear_j;
			curved_penalty += curved_i * matrix[1 + i][1 + j] * curved_j;
		}
	}

	EXPECT_NEAR(constant_penalty, 0.f, 1e-6f);
	EXPECT_NEAR(linear_penalty, 0.f, 1e-5f);
	EXPECT_GT(curved_penalty, 1.f);
}

TEST(AxisGlrtAccumulator, PureNormalColumnHasNoAttackEvidence)
{
	AxisGlrtAccumulator accumulator;
	accumulator.reset();

	for (int sample = 0; sample <= 80; ++sample) {
		const float tau = static_cast<float>(sample) / 80.f;
		const float time = kWindowSeconds * tau;
		float acceleration_basis[AttackBasis::kSize] {};
		float velocity_basis[AttackBasis::kSize] {};
		float position_basis[AttackBasis::kSize] {};
		AttackBasis::evaluate(tau, acceleration_basis);
		AttackBasis::evaluateIntegral(tau, velocity_basis);
		AttackBasis::evaluateDoubleIntegral(tau, position_basis);

		for (size_t i = 0; i < AttackBasis::kSize; ++i) {
			velocity_basis[i] *= kWindowSeconds;
			position_basis[i] *= kWindowSeconds * kWindowSeconds;
		}

		{
			const float normal_basis[AxisGlrtAccumulator::kNormalDim] {1.f, 0.f, 0.f};
			accumulator.addObservation(2.5f, 1.f, normal_basis, acceleration_basis);
		}
		{
			const float normal_basis[AxisGlrtAccumulator::kNormalDim] {time, 0.f, 0.f};
			accumulator.addObservation(2.5f * time, 1.f, normal_basis, velocity_basis);
		}
		{
			const float normal_basis[AxisGlrtAccumulator::kNormalDim] {0.5f * time * time, 0.f, 0.f};
			accumulator.addObservation(1.25f * time * time, 1.f, normal_basis, position_basis);
		}
	}

	AxisGlrtAccumulator::NormalPrior prior{};
	prior.precision[1] = 1.f;
	prior.precision[2] = 1.f;
	const AxisGlrtAccumulator::Result result = accumulator.solve(0.1f, prior);
	ASSERT_TRUE(result.valid);
	EXPECT_NEAR(result.glrt, 0.f, 2e-2f);
	EXPECT_NEAR(result.normal_parameters[0], 2.5f, 1e-4f);
}

TEST(AxisGlrtAccumulator, StructuredAttackProducesPositiveEvidence)
{
	AxisGlrtAccumulator accumulator;
	accumulator.reset();
	const float coefficients[AttackBasis::kSize] {1.2f, -0.5f, -1.1f, 0.2f, 0.9f, -0.7f};

	for (int sample = 0; sample <= 80; ++sample) {
		const float tau = static_cast<float>(sample) / 80.f;
		const float time = kWindowSeconds * tau;
		float acceleration_basis[AttackBasis::kSize] {};
		float velocity_basis[AttackBasis::kSize] {};
		float position_basis[AttackBasis::kSize] {};
		AttackBasis::evaluate(tau, acceleration_basis);
		AttackBasis::evaluateIntegral(tau, velocity_basis);
		AttackBasis::evaluateDoubleIntegral(tau, position_basis);
		float attack_acceleration = 0.f;
		float attack_velocity = 0.f;
		float attack_position = 0.f;

		for (size_t i = 0; i < AttackBasis::kSize; ++i) {
			velocity_basis[i] *= kWindowSeconds;
			position_basis[i] *= kWindowSeconds * kWindowSeconds;
			attack_acceleration += acceleration_basis[i] * coefficients[i];
			attack_velocity += velocity_basis[i] * coefficients[i];
			attack_position += position_basis[i] * coefficients[i];
		}

		const float normal = 0.3f;
		{
			const float normal_basis[AxisGlrtAccumulator::kNormalDim] {1.f, 0.f, 0.f};
			accumulator.addObservation(normal + attack_acceleration, 1.f, normal_basis, acceleration_basis);
		}
		{
			const float normal_basis[AxisGlrtAccumulator::kNormalDim] {time, 0.f, 0.f};
			accumulator.addObservation(normal * time + attack_velocity, 1.f, normal_basis, velocity_basis);
		}
		{
			const float normal_basis[AxisGlrtAccumulator::kNormalDim] {0.5f * time * time, 0.f, 0.f};
			accumulator.addObservation(0.5f * normal * time * time + attack_position, 1.f,
					   normal_basis, position_basis);
		}
	}

	AxisGlrtAccumulator::NormalPrior prior{};
	prior.precision[1] = 1.f;
	prior.precision[2] = 1.f;
	const AxisGlrtAccumulator::Result result = accumulator.solve(0.01f, prior);
	ASSERT_TRUE(result.valid);
	EXPECT_GT(result.glrt, 10.f);
	EXPECT_LT(result.cost_attack, result.cost_null);
}

TEST(AxisGlrtAccumulator, DegenerateSystemIsRejected)
{
	AxisGlrtAccumulator accumulator;
	accumulator.reset();
	float basis[AttackBasis::kSize] {};
	AttackBasis::evaluate(0.5f, basis);

	for (int i = 0; i < 20; ++i) {
		{
			const float normal_basis[AxisGlrtAccumulator::kNormalDim] {1.f, 0.f, 0.f};
			accumulator.addObservation(1.f, 1.f, normal_basis, basis);
		}
	}

	AxisGlrtAccumulator::NormalPrior prior{};
	prior.precision[1] = 1.f;
	prior.precision[2] = 1.f;
	EXPECT_FALSE(accumulator.solve(0.1f, prior).valid);
}
