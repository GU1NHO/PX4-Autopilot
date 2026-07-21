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

#include <float.h>
#include <math.h>

#include <px4_platform_common/defines.h>

using namespace matrix;


/*
 * Fixed attitude gain:
 *
 *     K_R = diag(10, 10, 10)
 *
 * The parameter input is retained so the existing PX4 calls compile,
 * but it is not used.
 */
void AttitudeControl::setProportionalGain(
	const Vector3f &proportional_gain,
	const float yaw_weight)
{
	(void)proportional_gain;

	_proportional_gain =
		Vector3f(9.f, 9.f, 9.f);

	_yaw_w =
		math::constrain(yaw_weight, 0.f, 1.f);
}


/*
 * Fixed angular-velocity gain:
 *
 *     K_omega = diag(6, 6, 6)
 *
 * The parameter input is retained for interface compatibility.
 */
void AttitudeControl::setAngularVelocityGain(
	const Vector3f &angular_velocity_gain)
{
	(void)angular_velocity_gain;

	_angular_velocity_gain =
		Vector3f(6.f, 6.f, 6.f);
}


/*
 * Calculate the desired angular velocity from consecutive
 * desired rotation matrices.
 *
 *     dot(R_d) ≈ (R_d[k] - R_d[k-1]) / dt
 *
 *     omega_d =
 *       vee(skew(R_d^T dot(R_d)))
 */
Vector3f AttitudeControl::calculateDesiredAngularVelocity(
	const Dcmf &R_sp,
	const float dt)
{
	Vector3f omega_d_raw;
	omega_d_raw.zero();

	/*
	 * The first desired attitude cannot be differentiated.
	 */
	if (!_attitude_setpoint_initialized) {
		_previous_attitude_setpoint_R = R_sp;
		_attitude_setpoint_initialized = true;

		_desired_angular_velocity.zero();
		_desired_angular_velocity_filtered.zero();

		return _desired_angular_velocity_filtered;
	}

	/*
	 * Reject invalid or unreasonable time intervals.
	 */
	if (!PX4_ISFINITE(dt)
	    || dt < 1e-4f
	    || dt > 0.1f) {

		_previous_attitude_setpoint_R = R_sp;

		_desired_angular_velocity.zero();
		_desired_angular_velocity_filtered.zero();

		return _desired_angular_velocity_filtered;
	}

	/*
	 * Backward numerical derivative:
	 *
	 *     dot(R_d)[k] =
	 *       (R_d[k] - R_d[k-1]) / dt
	 */
	Dcmf R_sp_dot;

	for (int row = 0; row < 3; row++) {
		for (int column = 0; column < 3; column++) {
			R_sp_dot(row, column) =
				(R_sp(row, column)
				 - _previous_attitude_setpoint_R(row, column))
				/ dt;
		}
	}

	/*
	 * For:
	 *
	 *     dot(R_d) = R_d S(omega_d)
	 *
	 * we have:
	 *
	 *     S(omega_d) = R_d^T dot(R_d)
	 */
	const Dcmf omega_matrix =
		R_sp.transpose() * R_sp_dot;

	/*
	 * Numerical differentiation may produce a matrix that is
	 * not exactly skew-symmetric.
	 *
	 * Project it onto the skew-symmetric matrix space:
	 *
	 *     S(omega_d) =
	 *       0.5(Omega - Omega^T)
	 */
	Dcmf omega_hat;

	for (int row = 0; row < 3; row++) {
		for (int column = 0; column < 3; column++) {
			omega_hat(row, column) =
				0.5f
				* (omega_matrix(row, column)
				   - omega_matrix(column, row));
		}
	}

	/*
	 * Vee operator:
	 *
	 *             [  0   -wz   wy ]
	 * S(omega) =  [  wz   0   -wx ]
	 *             [ -wy   wx    0 ]
	 */
	omega_d_raw(0) = omega_hat(2, 1);
	omega_d_raw(1) = omega_hat(0, 2);
	omega_d_raw(2) = omega_hat(1, 0);

	_desired_angular_velocity =
		omega_d_raw;

	/*
	 * First-order low-pass filter:
	 *
	 *     alpha = dt / (tau + dt)
	 *
	 *     omega_df[k] =
	 *       omega_df[k-1]
	 *       + alpha(omega_d[k] - omega_df[k-1])
	 */
	/*const float tau =
		math::max(
			_omega_d_filter_time_constant,
			0.f);

	const float alpha =
		(tau > FLT_EPSILON)
		? math::constrain(
			dt / (tau + dt),
			0.f,
			1.f)
		: 1.f;

	_desired_angular_velocity_filtered +=
		alpha
		* (omega_d_raw
		   - _desired_angular_velocity_filtered);
	*/

	/*
	 * Save the current desired attitude for the next iteration.
	 */
	_previous_attitude_setpoint_R =
		R_sp;

	return _desired_angular_velocity;
}


/*
 * New attitude-setpoint function with dt.
 *
 * This version must be used to obtain a nonzero numerical
 * desired angular velocity.
 */
void AttitudeControl::setAttitudeSetpoint(
	const Dcmf &R_sp,
	const float yawspeed_setpoint,
	const float dt)
{
	_yawspeed_setpoint =
		yawspeed_setpoint;

	/*
	 * Calculate omega_d before replacing the current R_d.
	 */
	_desired_angular_velocity_filtered =
		calculateDesiredAngularVelocity(
			R_sp,
			dt);

	_attitude_setpoint_R =
		R_sp;
}


/*
 * Compatibility controller.
 *
 * This function is called by the existing PX4 instruction:
 *
 *     Vector3f rates_sp = _attitude_control.update(q);
 *
 * It returns a body-angular-rate setpoint:
 *
 *     omega_sp =
 *       K_R e_R + R^T e3 yaw_rate_sp
 */
Vector3f AttitudeControl::update(
	const Dcmf &R) const
{
	const Vector3f e3(0.f, 0.f, 1.f);

	const Dcmf &R_d =
		_attitude_setpoint_R;

	/*
	 * Relative attitude:
	 *
	 *     R_e = R^T R_d
	 */
	const Dcmf R_e =
		R.transpose() * R_d;

	/*
	 * Geometric attitude error:
	 *
	 *     e_R =
	 *       0.5(R^T R_d - R_d^T R)^vee
	 */
	Vector3f e_R;

	e_R(0) =
		0.5f
		* (R_e(2, 1) - R_e(1, 2));

	e_R(1) =
		0.5f
		* (R_e(0, 2) - R_e(2, 0));

	e_R(2) =
		0.5f
		* (R_e(1, 0) - R_e(0, 1));

	/*
	 * Optional yaw prioritization.
	 */
	e_R(2) *= _yaw_w;

	/*
	 * Fixed attitude gain.
	 */
	const Vector3f k_R(
		10.f,
		10.f,
		10.f);

	Vector3f rate_setpoint =
		e_R.emult(k_R);

	/*
	 * Desired yaw-rate feedforward about the inertial z-axis,
	 * expressed in the current body frame:
	 *
	 *     omega_yaw^B =
	 *       R^T e3 yaw_rate_sp
	 */
	if (PX4_ISFINITE(_yawspeed_setpoint)) {
		const Vector3f yaw_axis_body =
			R.transpose() * e3;

		rate_setpoint +=
			yaw_axis_body
			* _yawspeed_setpoint;
	}

	/*
	 * Apply the existing PX4 body-rate limits.
	 */
	for (int i = 0; i < 3; i++) {
		rate_setpoint(i) =
			math::constrain(
				rate_setpoint(i),
				-_rate_limit(i),
				_rate_limit(i));
	}

	return rate_setpoint;
}


/*
 * New geometric angular-acceleration controller.
 *
 *     dot(omega)* =
 *       K_R e_R
 *       + K_omega(R^T R_d omega_d - omega)
 */
Vector3f AttitudeControl::update(
	const Dcmf &R,
	const Vector3f &angular_velocity) const
{
	const Dcmf &R_d =
		_attitude_setpoint_R;

	/*
	 * Relative attitude:
	 *
	 *     R_e = R^T R_d
	 */
	const Dcmf R_e =
		R.transpose() * R_d;

	/*
	 * Geometric attitude error:
	 *
	 *     e_R =
	 *       0.5(R^T R_d - R_d^T R)^vee
	 */
	Vector3f e_R;

	e_R(0) =
		0.5f
		* (R_e(2, 1) - R_e(1, 2));

	e_R(1) =
		0.5f
		* (R_e(0, 2) - R_e(2, 0));

	e_R(2) =
		0.5f
		* (R_e(1, 0) - R_e(0, 1));



	/*
	 * Desired angular velocity omega_d is expressed in the
	 * desired body frame.
	 *
	 * Transform it into the current body frame:
	 *
	 *     omega_d_current =
	 *       R^T R_d omega_d
	 */
	const Vector3f omega_d_current =
		 _desired_angular_velocity;

	/*
	 * Angular-velocity correction:
	 *
	 *     omega_d_current - omega
	 */
	//const Vector3f angular_velocity_correction =
	//	omega_d_current
	//	- angular_velocity;

	/*
	 * Fixed gains:
	 *
	 *     K_R     = diag(10, 10, 10)
	 *     K_omega = diag(6, 6, 6)
	 */
	const Vector3f k_R(
		10.f,
		10.f,
		10.f);

	const Vector3f k_omega(
		0.f,
		0.f,
		0.f);

	/*
	 * Desired angular acceleration:
	 *
	 *     dot(omega)* =   K_omega*(omega_d - omega)+ K_R*e_R   */

	const Vector3f angular_acceleration_setpoint = omega_d_current
							+ e_R.emult(k_R) ;

	return angular_acceleration_setpoint;
}
