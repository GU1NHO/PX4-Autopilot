/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file AttitudeControl.cpp
 */

#include <AttitudeControl.hpp>

#include <mathlib/math/Functions.hpp>

using namespace matrix;

matrix::Vector3f AttitudeControl::update(const Quatf &q) const
{
	const Dcmf R(q);
	const Dcmf Rd(_attitude_setpoint_q);

	// Geometric attitude tracking error eR = 1/2 * (Rd^T R - R^T Rd)∨ (Lee2010 eq. 10), body frame
	const Matrix3f R_err = Rd.transpose() * R;
	Vector3f eR(0.5f * (R_err(2, 1) - R_err(1, 2)),
		    0.5f * (R_err(0, 2) - R_err(2, 0)),
		    0.5f * (R_err(1, 0) - R_err(0, 1)));

	// Escape the 180 degree critical point where eR vanishes (tr(Rd^T R) -> -1, error function Ψ -> 2).
	// There the error rotation axis a satisfies R_err * a = a and (R_err + I) = 2*a*a^T:
	// recover a from the largest column and command a full-scale error along it (max of the sin(α) profile).
	const float trace_R_err = R_err(0, 0) + R_err(1, 1) + R_err(2, 2);

	if ((trace_R_err < -1.f + 1e-2f) && (eR.norm_squared() < 1e-4f)) {
		Matrix3f A = R_err;
		A(0, 0) += 1.f;
		A(1, 1) += 1.f;
		A(2, 2) += 1.f;

		Vector3f axis;
		float max_norm_squared = 0.f;

		for (int i = 0; i < 3; i++) {
			const Vector3f column(A(0, i), A(1, i), A(2, i));

			if (column.norm_squared() > max_norm_squared) {
				max_norm_squared = column.norm_squared();
				axis = column;
			}
		}

		axis = axis.unit_or_zero();

		// deterministic sign: first non-zero component positive
		for (int i = 0; i < 3; i++) {
			if (fabsf(axis(i)) > 1e-6f) {
				if (axis(i) < 0.f) {
					axis = -axis;
				}

				break;
			}
		}

		eR = axis;
	}

	// calculate angular rates setpoint
	Vector3f rate_setpoint = -eR.emult(_gain_kr);

	// Feed forward the yaw setpoint rate.
	// yawspeed_setpoint is the feed forward commanded rotation around the world z-axis,
	// but we need to apply it in the body frame (because _rates_sp is expressed in the body frame).
	// Therefore we infer the world z-axis (expressed in the body frame) by taking the last column of R.transposed (== q.inversed)
	// and multiply it by the yaw setpoint rate (yawspeed_setpoint).
	// This equals the transformed desired rate R^T Rd Ωd of Lee2010 eq. 11 for Ωd = yawspeed * Rd^T e3.
	if (std::isfinite(_yawspeed_setpoint)) {
		rate_setpoint += q.inversed().dcm_z() * _yawspeed_setpoint;
	}

	// limit rates
	for (int i = 0; i < 3; i++) {
		rate_setpoint(i) = math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i));
	}

	return rate_setpoint;
}
