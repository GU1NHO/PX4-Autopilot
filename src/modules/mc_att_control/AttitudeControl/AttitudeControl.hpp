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
 * @file AttitudeControl.hpp
 *
 * Geometric SO(3) attitude tracking controller, cascade form.
 *
 * Implements the attitude error of Lee, Leok, McClamroch 2010,
 * "Geometric Tracking Control of a Quadrotor UAV on SE(3)" (eqs. 8, 10):
 * eR = 1/2 (Rd^T R - R^T Rd)∨, output rate setpoint = -kR ∘ eR + feedforward.
 * The angular velocity error (eq. 11) and moment law (eq. 16) belong to the
 * rate loop (Phase 3).
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>

class AttitudeControl
{
public:
	AttitudeControl() = default;
	~AttitudeControl() = default;

	// SE(3) geometric attitude gains kR [1/s], tunable per axis (Lee2010 eq. 10, cascade form).
	// Hardcoded tuning knobs for now (no PX4 params). Defaults are the stock PX4 equivalents
	// (MC_ROLL_P, MC_PITCH_P, MC_YAW_P); the paper's moment-law gains kR = 8.81, kOmega = 2.54
	// correspond to a cascade gain of kR/kOmega ≈ 3.5.
	static constexpr float SE3_KR_X = 4.0f;
	static constexpr float SE3_KR_Y = 4.0f;
	static constexpr float SE3_KR_Z = 2.8f;

	/**
	 * Set the SE(3) attitude gains, overriding the hardcoded defaults
	 * @param kr 3D vector of attitude error gains [1/s] for the body x, y, z axes
	 */
	void setSE3AttitudeGains(const matrix::Vector3f &kr) { _gain_kr = kr; }

	/**
	 * Set hard limit for output rate setpoints
	 * @param rate_limit [rad/s] 3D vector containing limits for roll, pitch, yaw
	 */
	void setRateLimit(const matrix::Vector3f &rate_limit) { _rate_limit = rate_limit; }

	/**
	 * Set a new attitude setpoint replacing the one tracked before
	 * @param qd desired vehicle attitude setpoint
	 * @param yawspeed_setpoint [rad/s] yaw feed forward angular rate in world frame
	 */
	void setAttitudeSetpoint(const matrix::Quatf &qd, const float yawspeed_setpoint)
	{
		_attitude_setpoint_q = qd;
		_attitude_setpoint_q.normalize();
		_yawspeed_setpoint = yawspeed_setpoint;
	}

	/**
	 * Adjust last known attitude setpoint by a delta rotation
	 * Optional use to avoid glitches when attitude estimate reference e.g. heading changes.
	 * @param q_delta delta rotation to apply
	 */
	void adaptAttitudeSetpoint(const matrix::Quatf &q_delta)
	{
		_attitude_setpoint_q = q_delta * _attitude_setpoint_q;
		_attitude_setpoint_q.normalize();
	}

	/**
	 * Run one control loop cycle calculation
	 * @param q estimation of the current vehicle attitude unit quaternion
	 * @return [rad/s] body frame 3D angular rate setpoint vector to be executed by the rate controller
	 */
	matrix::Vector3f update(const matrix::Quatf &q) const;

private:
	matrix::Vector3f _gain_kr{SE3_KR_X, SE3_KR_Y, SE3_KR_Z}; ///< attitude error gain [1/s]
	matrix::Vector3f _rate_limit;

	matrix::Quatf _attitude_setpoint_q; ///< latest known attitude setpoint e.g. from position control
	float _yawspeed_setpoint{0.f}; ///< latest known yawspeed feed-forward setpoint
};
