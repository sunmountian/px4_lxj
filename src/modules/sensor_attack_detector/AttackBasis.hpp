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

#include <math.h>
#include <stddef.h>

/**
 * Six equally spaced first-degree B-spline (linear hat) basis functions on
 * normalized window time tau in [0, 1].
 *
 * The integral methods return normalized-time integrals. Callers multiply
 * evaluateIntegral() by H and evaluateDoubleIntegral() by H^2 to obtain
 * physical-time velocity and position rows.
 */
class AttackBasis
{
public:
	static constexpr size_t kSize{6};
	static constexpr size_t kSegmentCount{kSize - 1};

	static void evaluate(float tau, float (&psi)[kSize])
	{
		zero(psi);
		tau = constrain(tau);

		if (tau >= 1.f) {
			psi[kSize - 1] = 1.f;
			return;
		}

		const float scaled = tau * static_cast<float>(kSegmentCount);
		size_t segment = static_cast<size_t>(scaled);

		if (segment >= kSegmentCount) {
			segment = kSegmentCount - 1;
		}

		const float local = scaled - static_cast<float>(segment);
		psi[segment] = 1.f - local;
		psi[segment + 1] = local;
	}

	static void evaluateIntegral(float tau, float (&integral)[kSize])
	{
		zero(integral);
		tau = constrain(tau);
		const float segment_width = 1.f / static_cast<float>(kSegmentCount);

		for (size_t segment = 0; segment < kSegmentCount; ++segment) {
			const float start = static_cast<float>(segment) * segment_width;

			if (tau <= start) {
				break;
			}

			const float length = fminf(segment_width, tau - start);
			const float length_sq = length * length;
			integral[segment] += length - length_sq / (2.f * segment_width);
			integral[segment + 1] += length_sq / (2.f * segment_width);
		}
	}

	static void evaluateDoubleIntegral(float tau, float (&integral)[kSize])
	{
		zero(integral);
		tau = constrain(tau);
		const float segment_width = 1.f / static_cast<float>(kSegmentCount);

		for (size_t segment = 0; segment < kSegmentCount; ++segment) {
			const float start = static_cast<float>(segment) * segment_width;

			if (tau <= start) {
				break;
			}

			const float length = fminf(segment_width, tau - start);
			const float length_sq = length * length;
			const float length_cu = length_sq * length;
			const float remaining = tau - start;

			integral[segment] += remaining * length
					     - remaining * length_sq / (2.f * segment_width)
					     - length_sq / 2.f
					     + length_cu / (3.f * segment_width);
			integral[segment + 1] += remaining * length_sq / (2.f * segment_width)
						 - length_cu / (3.f * segment_width);
		}
	}

	template<size_t N>
	static void addSecondDifferenceRegularizer(float (&matrix)[N][N], float lambda, size_t attack_offset = 1)
	{
		if (!(lambda > 0.f) || (attack_offset + kSize > N)) {
			return;
		}

		static constexpr float row[3] {1.f, -2.f, 1.f};

		for (size_t difference = 0; difference < kSize - 2; ++difference) {
			for (size_t i = 0; i < 3; ++i) {
				for (size_t j = 0; j < 3; ++j) {
					matrix[attack_offset + difference + i][attack_offset + difference + j]
						+= lambda * row[i] * row[j];
				}
			}
		}
	}

	/**
	 * Fix the single unidentifiable constant-curvature gauge.
	 *
	 * attack_offset locates the first attack coefficient in the joint normal
	 * equation. This is 1 for the legacy bias-only normal model and 3 for the
	 * V2 bias/scale/dynamic normal model.
	 */
	template<size_t N>
	static void addConstantModeGauge(float (&matrix)[N][N], float strength, size_t attack_offset = 1)
	{
		if (!(strength > 0.f) || (attack_offset + kSize > N)) {
			return;
		}

		for (size_t i = 0; i < kSize; ++i) {
			for (size_t j = 0; j < kSize; ++j) {
				matrix[attack_offset + i][attack_offset + j] += strength;
			}
		}
	}

private:
	static float constrain(float value)
	{
		return value < 0.f ? 0.f : (value > 1.f ? 1.f : value);
	}

	static void zero(float (&values)[kSize])
	{
		for (size_t i = 0; i < kSize; ++i) {
			values[i] = 0.f;
		}
	}
};
