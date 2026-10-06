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

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>

class AxisGlrtAccumulator
{
public:
	static constexpr size_t kNormalDim{3};
	static constexpr size_t kAttackDim{AttackBasis::kSize};
	static constexpr size_t kJointDim{kNormalDim + kAttackDim};
	static constexpr size_t kResidualGroupCount{3};

	enum ResidualGroup : uint8_t {
		kAcceleration = 0,
		kVelocity = 1,
		kPosition = 2
	};

	struct NormalPrior {
		float mean[kNormalDim] {0.f, 0.f, 0.f};
		float precision[kNormalDim] {0.f, 0.f, 0.f};
	};

	struct Result {
		bool valid{false};
		float glrt{0.f};
		float cost_null{0.f};
		float cost_attack{0.f};
		float normal_parameters[kNormalDim] {};
		float normal_residual_rms[kResidualGroupCount] {};
		float attack_coefficients[kAttackDim] {};
		float minimum_cholesky_diagonal{0.f};
		uint16_t observation_count{0};
	};

	void reset();
	void addObservation(float observation, float weight,
			    const float (&normal_basis)[kNormalDim],
			    const float (&attack_basis)[kAttackDim],
			    ResidualGroup group = kAcceleration);
	Result solve(float regularization, const NormalPrior &prior) const;
	uint16_t observationCount() const { return _observation_count; }

private:
	template<size_t N>
	static bool choleskySolve(const float (&input)[N][N],
				  const float (&rhs)[N], float (&solution)[N],
				  float &minimum_diagonal)
	{
		float lower[N][N] {};
		minimum_diagonal = FLT_MAX;

		for (size_t i = 0; i < N; ++i) {
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

		float intermediate[N] {};

		for (size_t i = 0; i < N; ++i) {
			float sum = rhs[i];

			for (size_t j = 0; j < i; ++j) {
				sum -= lower[i][j] * intermediate[j];
			}

			intermediate[i] = sum / lower[i][i];
		}

		for (int i = static_cast<int>(N) - 1; i >= 0; --i) {
			float sum = intermediate[i];

			for (size_t j = static_cast<size_t>(i) + 1; j < N; ++j) {
				sum -= lower[j][i] * solution[j];
			}

			solution[i] = sum / lower[i][i];

			if (!isfinite(solution[i])) {
				return false;
			}
		}

		return true;
	}

	float _normal_hessian[kNormalDim][kNormalDim] {};
	float _normal_rhs[kNormalDim] {};
	float _joint_hessian[kJointDim][kJointDim] {};
	float _joint_rhs[kJointDim] {};
	float _squared_observation{0.f};
	uint16_t _observation_count{0};

	float _group_normal_hessian[kResidualGroupCount][kNormalDim][kNormalDim] {};
	float _group_normal_rhs[kResidualGroupCount][kNormalDim] {};
	float _group_squared_observation[kResidualGroupCount] {};
	uint16_t _group_observation_count[kResidualGroupCount] {};
};
