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
 * A quaternion based attitude controller.
 *
 * @author Matthias Grob	<maetugr@gmail.com>
 *
 * Publication documenting the implemented Quaternion Attitude Control:
 * Nonlinear Quadrocopter Attitude Control (2013)
 * by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
 * Institute for Dynamic Systems and Control (IDSC), ETH Zurich
 *
 * https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf
 */

#pragma once

#include <matrix/matrix/math.hpp>

class AttitudeControl
{
public:
	/*
	 * Initialize rotation matrices as identity matrices.
	 */
	AttitudeControl()
	{
		_attitude_setpoint_R.setIdentity();
		_previous_attitude_setpoint_R.setIdentity();
	}

	/*
	 * Attitude-error gain K_R.
	 */
	void setProportionalGain(
		const matrix::Vector3f &proportional_gain,
		const float yaw_weight);

	/*
	 * Angular-velocity-error gain K_omega.
	 */
	void setAngularVelocityGain(
		const matrix::Vector3f &angular_velocity_gain);

	/*
	 * Existing PX4 body-rate limits.
	 *
	 * These limits are used only by the compatibility
	 * attitude-to-rate controller update(R).
	 */
	void setRateLimit(const matrix::Vector3f &rate_limit)
	{
		_rate_limit = rate_limit;
	}

	/*
	 * ============================================================
	 * ATTITUDE SETPOINT FUNCTIONS
	 * ============================================================
	 */

	/*
	 * Existing PX4 quaternion interface.
	 *
	 * The quaternion is immediately converted to a rotation matrix.
	 * This keeps the following existing call valid:
	 *
	 * setAttitudeSetpoint(Quatf(q_d), yaw_rate);
	 */
	void setAttitudeSetpoint(
		const matrix::Quatf &q_sp,
		const float yawspeed_setpoint)
	{
		setAttitudeSetpoint(
			matrix::Dcmf(q_sp),
			yawspeed_setpoint);
	}

	/*
	 * Existing two-input rotation-matrix interface.
	 *
	 * No numerical derivative is calculated because dt is not
	 * available. Therefore omega_d is set to zero.
	 */
	void setAttitudeSetpoint(
		const matrix::Dcmf &R_sp,
		const float yawspeed_setpoint)
	{
		_attitude_setpoint_R = R_sp;
		_yawspeed_setpoint = yawspeed_setpoint;

		/*
		 * The old interface does not provide dt, so omega_d
		 * cannot be calculated.
		 */
		_previous_attitude_setpoint_R = R_sp;
		_attitude_setpoint_initialized = true;

		_desired_angular_velocity.zero();
		_desired_angular_velocity_filtered.zero();
	}

	/*
	 * New interface used by the geometric angular-acceleration
	 * controller.
	 *
	 * It calculates:
	 *
	 *     dot(R_d) ≈ (R_d[k] - R_d[k-1]) / dt
	 *
	 *     omega_d =
	 *       vee(skew(R_d^T dot(R_d)))
	 */
	void setAttitudeSetpoint(
		const matrix::Dcmf &R_sp,
		const float yawspeed_setpoint,
		const float dt);

	/*
	 * Quaternion wrapper for the new three-input interface.
	 */
	void setAttitudeSetpoint(
		const matrix::Quatf &q_sp,
		const float yawspeed_setpoint,
		const float dt)
	{
		setAttitudeSetpoint(
			matrix::Dcmf(q_sp),
			yawspeed_setpoint,
			dt);
	}

	/*
	 * ============================================================
	 * ESTIMATOR RESET FUNCTIONS
	 * ============================================================
	 */

	void adaptAttitudeSetpoint(
		const matrix::Dcmf &R_delta)
	{
		/*
		 * Apply the estimator-frame reset:
		 *
		 *     R_d,new = R_delta R_d,old
		 */
		_attitude_setpoint_R =
			R_delta * _attitude_setpoint_R;

		/*
		 * Apply the same transformation to the previous desired
		 * attitude. This avoids an artificial numerical-derivative
		 * spike after the estimator reset.
		 */
		_previous_attitude_setpoint_R =
			R_delta * _previous_attitude_setpoint_R;
	}

	/*
	 * Existing PX4 quaternion reset interface.
	 */
	void adaptAttitudeSetpoint(
		const matrix::Quatf &q_delta)
	{
		adaptAttitudeSetpoint(
			matrix::Dcmf(q_delta));
	}

	/*
	 * ============================================================
	 * CONTROLLER UPDATE FUNCTIONS
	 * ============================================================
	 */

	/*
	 * Existing PX4 quaternion interface:
	 *
	 *     update(q)
	 *
	 * Internally converts q to R.
	 */
	matrix::Vector3f update(
		const matrix::Quatf &q) const
	{
		return update(matrix::Dcmf(q));
	}

	/*
	 * Compatibility attitude-to-rate controller.
	 *
	 * Output:
	 *
	 *     omega_sp =
	 *       K_R e_R + R^T e3 yaw_rate_sp
	 *
	 * This keeps the existing PX4 call working:
	 *
	 *     Vector3f rates_sp = update(q);
	 */
	matrix::Vector3f update(
		const matrix::Dcmf &R) const;

	/*
	 * New geometric angular-acceleration controller.
	 *
	 * Output:
	 *
	 *     dot(omega)* =
	 *       K_R e_R
	 *       + K_omega(R^T R_d omega_d - omega)
	 */
	matrix::Vector3f update(
		const matrix::Dcmf &R,
		const matrix::Vector3f &angular_velocity) const;

	/*
	 * Quaternion wrapper for the new controller.
	 */
	matrix::Vector3f update(
		const matrix::Quatf &q,
		const matrix::Vector3f &angular_velocity) const
	{
		return update(
			matrix::Dcmf(q),
			angular_velocity);
	}

private:
	/*
	 * Calculate desired angular velocity from consecutive
	 * desired rotation matrices:
	 *
	 *     dot(R_d) ≈
	 *       (R_d[k] - R_d[k-1]) / dt
	 *
	 *     omega_d =
	 *       vee(skew(R_d^T dot(R_d)))
	 */
	matrix::Vector3f calculateDesiredAngularVelocity(
		const matrix::Dcmf &R_sp,
		const float dt);

	/*
	 * Desired attitude R_d.
	 */
	matrix::Dcmf _attitude_setpoint_R;

	/*
	 * Previous desired attitude R_d[k-1], used for numerical
	 * differentiation.
	 */
	matrix::Dcmf _previous_attitude_setpoint_R;

	/*
	 * Raw desired angular velocity, expressed in the desired
	 * body frame.
	 */
	matrix::Vector3f _desired_angular_velocity{};

	/*
	 * Filtered desired angular velocity.
	 */
	matrix::Vector3f _desired_angular_velocity_filtered{};

	/*
	 * Attitude gain:
	 *
	 *     K_R = diag(k_Rx, k_Ry, k_Rz)
	 */
	matrix::Vector3f _proportional_gain{};

	/*
	 * Angular-velocity gain:
	 *
	 *     K_omega = diag(k_wx, k_wy, k_wz)
	 */
	matrix::Vector3f _angular_velocity_gain{};

	/*
	 * Existing PX4 rate limits, used only by update(R).
	 */
	matrix::Vector3f _rate_limit{};

	/*
	 * Optional yaw weighting.
	 *
	 * Use 1 for the complete geometric attitude error.
	 */
	float _yaw_w{1.f};

	/*
	 * Retained for the compatibility attitude-to-rate controller.
	 */
	float _yawspeed_setpoint{0.f};

	/*
	 * Low-pass-filter time constant for the numerically calculated
	 * desired angular velocity.
	 */
	float _omega_d_filter_time_constant{0.02f};

	/*
	 * Indicates whether R_d[k-1] is available.
	 */
	bool _attitude_setpoint_initialized{false};
};
