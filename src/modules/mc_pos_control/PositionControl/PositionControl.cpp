/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 * @file PositionControl.cpp
 */

#include "PositionControl.hpp"
#include "ControlMath.hpp"

#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <geo/geo.h>

using namespace matrix;

const trajectory_setpoint_s PositionControl::empty_trajectory_setpoint = {
	0,
	{NAN, NAN, NAN},
	{NAN, NAN, NAN},
	{NAN, NAN, NAN},
	{NAN, NAN, NAN},
	NAN,
	NAN
};

void PositionControl::setVelocityGains(
	const Vector3f &P,
	const Vector3f &I,
	const Vector3f &D)
{
	_gain_vel_p = P;
	_gain_vel_i = I;
	_gain_vel_d = D;
}

void PositionControl::setVelocityLimits(
	const float vel_horizontal,
	const float vel_up,
	const float vel_down)
{
	_lim_vel_horizontal = vel_horizontal;
	_lim_vel_up = vel_up;
	_lim_vel_down = vel_down;
}

void PositionControl::setThrustLimits(const float min, const float max)
{
	// Ensure that the thrust vector always has enough magnitude
	// to define an attitude.
	_lim_thr_min = math::max(min, 10e-4f);
	_lim_thr_max = max;
}

void PositionControl::setHorizontalThrustMargin(const float margin)
{
	_lim_thr_xy_margin = margin;
}

void PositionControl::updateHoverThrust(const float hover_thrust_new)
{
	/*
	 * This function is kept for compatibility with the existing PX4
	 * hover-thrust estimator.
	 *
	 * The geometric controller does not currently use _vel_int, but
	 * keeping this compensation allows integral action to be added later
	 * without changing the public interface.
	 */

	const float previous_hover_thrust = _hover_thrust;

	setHoverThrust(hover_thrust_new);

	_vel_int(2) +=
		(_acc_sp(2) - CONSTANTS_ONE_G)
		* previous_hover_thrust
		/ _hover_thrust
		+ CONSTANTS_ONE_G
		- _acc_sp(2);
}

void PositionControl::setState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
}

void PositionControl::setInputSetpoint(
	const trajectory_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(setpoint.position);
	_vel_sp = Vector3f(setpoint.velocity);
	_acc_sp = Vector3f(setpoint.acceleration);

	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
}

bool PositionControl::update(const float dt)
{
	/*
	 * The first geometric implementation is not dynamic and therefore
	 * does not require dt. It will be needed later if integral action,
	 * filters, or higher-order reference dynamics are added.
	 */
	(void)dt;

	const bool valid = _inputValid();

	if (valid) {
		_geometricTrackingControl();

		_yawspeed_sp =
			PX4_ISFINITE(_yawspeed_sp)
			? _yawspeed_sp
			: 0.f;

		_yaw_sp =
			PX4_ISFINITE(_yaw_sp)
			? _yaw_sp
			: _yaw;
	}

	/*
	 * A valid acceleration and thrust setpoint must be generated.
	 */
	return valid
	       && _acc_sp.isAllFinite()
	       && _thr_sp.isAllFinite();
}

void PositionControl::_geometricTrackingControl()
{
	Vector3f position_error;
	Vector3f velocity_error;
	Vector3f acceleration_feedforward;

	position_error.setZero();
	velocity_error.setZero();
	acceleration_feedforward.setZero();

	/*
	 * Translational tracking errors:
	 *
	 * e_p = p - p_d
	 * e_v = v - v_d
	 *
	 * Geometric acceleration command:
	 *
	 * a_c = a_d - K_p e_p - K_v e_v
	 */

	for (int axis = 0; axis < 3; axis++) {

		const bool position_setpoint_valid =
			PX4_ISFINITE(_pos_sp(axis));

		const bool velocity_setpoint_valid =
			PX4_ISFINITE(_vel_sp(axis));

		const bool acceleration_setpoint_valid =
			PX4_ISFINITE(_acc_sp(axis));

		/*
		 * Position error.
		 *
		 * When there is no position setpoint, the position-error
		 * contribution is disabled for this axis.
		 */
		if (position_setpoint_valid) {
			position_error(axis) =
				_pos(axis) - _pos_sp(axis);
		}

		/*
		 * Desired velocity.
		 *
		 * A finite velocity setpoint is treated as trajectory
		 * feed-forward.
		 *
		 * When position control is active but no velocity reference is
		 * supplied, the desired velocity is assumed to be zero.
		 */
		if (position_setpoint_valid || velocity_setpoint_valid) {

			const float desired_velocity =
				velocity_setpoint_valid
				? _vel_sp(axis)
				: 0.f;

			velocity_error(axis) =
				_vel(axis) - desired_velocity;

			/*
			 * Store the velocity reference that was actually used.
			 * This value is later published in
			 * vehicle_local_position_setpoint.
			 */
			_vel_sp(axis) = desired_velocity;
		}

		/*
		 * Desired acceleration feed-forward.
		 *
		 * When acceleration is NAN, no acceleration feed-forward
		 * is applied.
		 */
		if (acceleration_setpoint_valid) {
			acceleration_feedforward(axis) =
				_acc_sp(axis);
		}
	}

	/*
	 * PX4 parameter mapping
	 * ---------------------
	 *
	 * The original PX4 controller approximately produces:
	 *
	 * a_c =
	 *     a_d
	 *     + K_vel K_pos (p_d - p)
	 *     + K_vel (v_d - v)
	 *
	 * Therefore, for the direct geometric tracking law:
	 *
	 * K_p_geo = K_pos_PX4 .* K_vel_PX4
	 * K_v_geo = K_vel_PX4
	 *
	 * This keeps the existing PX4 parameter dimensions consistent.
	 */
	const Vector3f geometric_position_gain =
		_gain_pos_p.emult(_gain_vel_p);

	const Vector3f geometric_velocity_gain =
		_gain_vel_p;

	_acc_sp =
		acceleration_feedforward
		- position_error.emult(geometric_position_gain)
		- velocity_error.emult(geometric_velocity_gain);

	/*
	 * Convert the commanded inertial acceleration into a desired
	 * body-z direction and normalized thrust vector.
	 */
	_accelerationControl();

	/*
	 * Thrust saturation
	 * -----------------
	 *
	 * Preserve the original PX4 strategy:
	 *
	 * 1. Reserve a horizontal-thrust margin.
	 * 2. Prioritize vertical thrust.
	 * 3. Limit the remaining horizontal thrust.
	 */

	const Vector2f thrust_sp_xy(_thr_sp);
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();

	const float thrust_max_squared =
		math::sq(_lim_thr_max);

	const float allocated_horizontal_thrust =
		math::min(
			thrust_sp_xy_norm,
			_lim_thr_xy_margin);

	const float thrust_z_max_squared =
		math::max(
			0.f,
			thrust_max_squared
			- math::sq(allocated_horizontal_thrust));

	/*
	 * In PX4 NED/FRD convention, upward thrust has a negative
	 * z component.
	 */
	_thr_sp(2) =
		math::max(
			_thr_sp(2),
			-sqrtf(thrust_z_max_squared));

	const float thrust_max_xy_squared =
		math::max(
			0.f,
			thrust_max_squared
			- math::sq(_thr_sp(2)));

	const float thrust_max_xy =
		sqrtf(thrust_max_xy_squared);

	if ((thrust_sp_xy_norm > thrust_max_xy)
	    && (thrust_sp_xy_norm > FLT_EPSILON)) {

		_thr_sp.xy() =
			thrust_sp_xy
			* (thrust_max_xy / thrust_sp_xy_norm);
	}
}

void PositionControl::_accelerationControl()
{
	const Vector3f e3(0.f, 0.f, 1.f);

	/*
	 * Translational dynamics in the PX4 NED/FRD convention:
	 *
	 * m a_c = m g e3 - f b3
	 *
	 * Therefore:
	 *
	 * f b3 = m (g e3 - a_c)
	 *
	 * and the desired body-z direction is:
	 *
	 * b3_d =
	 *     (g e3 - a_c)
	 *     / ||g e3 - a_c||
	 */

	Vector3f specific_thrust_direction =
		CONSTANTS_ONE_G * e3 - _acc_sp;

	/*
	 * Protect normalization from the singular case:
	 *
	 * g e3 - a_c = 0.
	 */
	if (specific_thrust_direction.norm_squared() < 1e-6f) {
		specific_thrust_direction = e3;
	}

	Vector3f body_z =
		specific_thrust_direction.normalized();

	/*
	 * Apply PX4's maximum tilt constraint.
	 */
	ControlMath::limitTilt(
		body_z,
		e3,
		_lim_tilt);

	/*
	 * Vertical normalized thrust required to produce the requested
	 * vertical acceleration:
	 *
	 * T_z = a_z Th/g - Th
	 *
	 * where Th is the normalized hover thrust.
	 */
	const float thrust_ned_z =
		_acc_sp(2)
		* (_hover_thrust / CONSTANTS_ONE_G)
		- _hover_thrust;

	/*
	 * Project the vertical thrust requirement onto the desired
	 * body-z direction.
	 */
	const float cos_ned_body =
		math::max(
			e3.dot(body_z),
			1e-3f);

	/*
	 * PX4 thrust is negative along body z in the NED/FRD convention.
	 */
	const float collective_thrust =
		math::min(
			thrust_ned_z / cos_ned_body,
			-_lim_thr_min);

	_thr_sp = body_z * collective_thrust;
}

bool PositionControl::_inputValid()
{
	bool valid = true;

	/*
	 * Every axis must contain at least one valid setpoint:
	 *
	 * position, velocity, or acceleration.
	 */
	for (int axis = 0; axis < 3; axis++) {

		valid =
			valid
			&& (
				PX4_ISFINITE(_pos_sp(axis))
				|| PX4_ISFINITE(_vel_sp(axis))
				|| PX4_ISFINITE(_acc_sp(axis))
			);
	}

	/*
	 * Horizontal x and y setpoints must always be provided in pairs.
	 */
	valid =
		valid
		&& (
			PX4_ISFINITE(_pos_sp(0))
			== PX4_ISFINITE(_pos_sp(1))
		);

	valid =
		valid
		&& (
			PX4_ISFINITE(_vel_sp(0))
			== PX4_ISFINITE(_vel_sp(1))
		);

	valid =
		valid
		&& (
			PX4_ISFINITE(_acc_sp(0))
			== PX4_ISFINITE(_acc_sp(1))
		);

	/*
	 * Validate all states required by the geometric controller.
	 */
	for (int axis = 0; axis < 3; axis++) {

		const bool position_control_active =
			PX4_ISFINITE(_pos_sp(axis));

		const bool velocity_control_active =
			PX4_ISFINITE(_vel_sp(axis));

		if (position_control_active) {
			valid =
				valid
				&& PX4_ISFINITE(_pos(axis))
				&& PX4_ISFINITE(_vel(axis));
		}

		if (velocity_control_active) {
			valid =
				valid
				&& PX4_ISFINITE(_vel(axis));
		}
	}

	return valid;
}

void PositionControl::getLocalPositionSetpoint(
	vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);

	local_position_setpoint.yaw = _yaw_sp;
	local_position_setpoint.yawspeed = _yawspeed_sp;

	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);

	_acc_sp.copyTo(
		local_position_setpoint.acceleration);

	_thr_sp.copyTo(
		local_position_setpoint.thrust);
}

void PositionControl::getAttitudeSetpoint(
	vehicle_attitude_setpoint_s &attitude_setpoint) const
{
	ControlMath::thrustToAttitude(
		_thr_sp,
		_yaw_sp,
		attitude_setpoint);

	attitude_setpoint.yaw_sp_move_rate =
		_yawspeed_sp;
}
