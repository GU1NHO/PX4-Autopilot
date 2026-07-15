/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team.
 *
 ****************************************************************************/
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

const trajectory_setpoint_s PositionControl::empty_trajectory_setpoint = {0, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, NAN};

void PositionControl::setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_vel_p = P;
	_gain_vel_i = I;
	_gain_vel_d = D;
}

void PositionControl::setVelocityLimits(const float vel_horizontal, const float vel_up, const float vel_down)
{
	_lim_vel_horizontal = vel_horizontal;
	_lim_vel_up = vel_up;
	_lim_vel_down = vel_down;
}

void PositionControl::setThrustLimits(const float min, const float max)
{
	// make sure there's always enough thrust vector length to infer the attitude
	_lim_thr_min = math::max(min, 10e-4f);
	_lim_thr_max = max;
}

void PositionControl::setHorizontalThrustMargin(const float margin)
{
	_lim_thr_xy_margin = margin;
}

void PositionControl::updateHoverThrust(const float hover_thrust_new)
{
	// Given that the equation for thrust is T = a_sp * Th / g - Th
	// with a_sp = desired acceleration, Th = hover thrust and g = gravity constant,
	// we want to find the acceleration that needs to be added to the integrator in order obtain
	// the same thrust after replacing the current hover thrust by the new one.
	// T' = T => a_sp' * Th' / g - Th' = a_sp * Th / g - Th
	// so a_sp' = (a_sp - g) * Th / Th' + g
	// we can then add a_sp' - a_sp to the current integrator to absorb the effect of changing Th by Th'
	const float previous_hover_thrust = _hover_thrust;
	setHoverThrust(hover_thrust_new);

	_vel_int(2) += (_acc_sp(2) - CONSTANTS_ONE_G) * previous_hover_thrust / _hover_thrust
		       + CONSTANTS_ONE_G - _acc_sp(2);
}

void PositionControl::setState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
}

void PositionControl::setInputSetpoint(const trajectory_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(setpoint.position);
	_vel_sp = Vector3f(setpoint.velocity);
	_acc_sp = Vector3f(setpoint.acceleration);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
}

bool PositionControl::update(const float dt)
{
	bool valid = _inputValid();

	if (valid) {

		_customPositionVelocityControl(dt);

		_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
		_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw;
	}

	return valid && _acc_sp.isAllFinite() && _thr_sp.isAllFinite();
}

void PositionControl::_customPositionVelocityControl(const float dt)
{
	(void)dt;

	Vector3f pos_error;
	Vector3f vel_error;
	Vector3f acc_ff;
	Vector3f acc_sp_custom;
const float acc_xy_max = 5.0f;
const float acc_z_max_up = 4.0f;
const float acc_z_max_down = 3.0f;
	pos_error.setZero();
	vel_error.setZero();
	acc_ff.setZero();
	acc_sp_custom.setZero();

	for (int i = 0; i < 3; i++) {
		if (PX4_ISFINITE(_acc_sp(i))) {
			acc_ff(i) = _acc_sp(i);
		}
	}

	for (int i = 0; i < 3; i++) {
		if (PX4_ISFINITE(_pos_sp(i)) && PX4_ISFINITE(_pos(i))) {
			pos_error(i) = _pos_sp(i) - _pos(i);
		}
	}

	for (int i = 0; i < 3; i++) {
		if (PX4_ISFINITE(_vel_sp(i)) && PX4_ISFINITE(_vel(i))) {
			vel_error(i) = _vel_sp(i) - _vel(i);
		}
	}

	// Custom gains
	const Vector3f Kp_custom(2.0f, 2.0f, 3.0f);
	const Vector3f Kd_custom(1.8f, 3.0f, 2.0f);

	// a_sp = Kp*ep + Kd*ev + a_sd
	acc_sp_custom =
		pos_error.emult(Kp_custom)
		+ vel_error.emult(Kd_custom)
		+ acc_ff;


Vector2f acc_xy(acc_sp_custom(0), acc_sp_custom(1));

if (acc_xy.norm() > acc_xy_max) {
	acc_xy = acc_xy / acc_xy.norm() * acc_xy_max;
	acc_sp_custom(0) = acc_xy(0);
	acc_sp_custom(1) = acc_xy(1);
}

acc_sp_custom(2) = math::constrain(
	acc_sp_custom(2),
	-acc_z_max_up,
	acc_z_max_down
);

	_acc_sp = acc_sp_custom;

	//_accelerationControl();

	// Geometric inverse dynamics
	_geometricAccelerationControl();
	// Keep PX4 thrust saturation logic
}
void PositionControl::_geometricAccelerationControl()
{
	const Vector3f e3(0.f, 0.f, 1.f);

	/*
	 * Translational dynamics in PX4 NED/FRD:
	 *
	 * m*a_sp = m*g*e3 - f*b3
	 *
	 * Therefore:
	 *
	 * f*b3 = m*(g*e3 - a_sp)
	 */

	Vector3f specific_thrust_direction =
		CONSTANTS_ONE_G * e3 - _acc_sp;

	/*
	 * specific_thrust_direction =
	 * [-a_sp_x, -a_sp_y, g - a_sp_z]
	 */

	if (specific_thrust_direction.norm_squared() < 1e-6f) {
		specific_thrust_direction = e3;
	}

	Vector3f body_z = specific_thrust_direction.normalized();

	// Limit desired tilt
	ControlMath::limitTilt(body_z, e3, _lim_tilt);

	/*
	 * PX4 uses normalized thrust.
	 *
	 * At hover:
	 * ||g*e3 - a_sp|| = g
	 * collective_thrust = -hover_thrust
	 */

	float collective_thrust =
		-_hover_thrust *
		(specific_thrust_direction.dot(body_z) / CONSTANTS_ONE_G);

	// Minimum thrust limit
	collective_thrust = math::min(collective_thrust, -_lim_thr_min);

	// Desired normalized thrust vector
	_thr_sp = body_z * collective_thrust;

	// PX4 thrust saturation logic
	const Vector2f thrust_sp_xy(_thr_sp);
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();
	const float thrust_max_squared = math::sq(_lim_thr_max);

	const float allocated_horizontal_thrust =
		math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);

	const float thrust_z_max_squared =
		thrust_max_squared - math::sq(allocated_horizontal_thrust);

	_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));

	const float thrust_max_xy_squared =
		thrust_max_squared - math::sq(_thr_sp(2));

	float thrust_max_xy = 0.f;

	if (thrust_max_xy_squared > 0.f) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}
}


bool PositionControl::_inputValid()
{
	bool valid = true;

	// Every axis x, y, z needs to have some setpoint
	for (int i = 0; i <= 2; i++) {
		valid = valid && (PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i)) || PX4_ISFINITE(_acc_sp(i)));
	}

	// x and y input setpoints always have to come in pairs
	valid = valid && (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)));
	valid = valid && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)));
	valid = valid && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

	// For each controlled state the estimate has to be valid
	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			valid = valid && PX4_ISFINITE(_pos(i));
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
		}
	}

	return valid;
}

void PositionControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);
	local_position_setpoint.yaw = _yaw_sp;
	local_position_setpoint.yawspeed = _yawspeed_sp;
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);
	_acc_sp.copyTo(local_position_setpoint.acceleration);
	_thr_sp.copyTo(local_position_setpoint.thrust);
}

void PositionControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
{
	/*
	 * Geometric desired attitude construction:
	 *
	 * b3_d = desired body z-axis
	 * b1_c = desired yaw direction
	 *
	 * b2_d = (b3_d x b1_c) / ||b3_d x b1_c||
	 * b1_d = b2_d x b3_d
	 *
	 * R_d = [b1_d b2_d b3_d]
	 */

	const Vector3f e3(0.f, 0.f, 1.f);

	/*
	 * PX4 thrust convention:
	 *
	 * _thr_sp points opposite to the desired body z-axis.
	 *
	 * Therefore:
	 *
	 * b3_d = -_thr_sp / ||_thr_sp||
	 */

	Vector3f b3_d = -_thr_sp;

	if (b3_d.norm_squared() < 1e-6f) {
		b3_d = e3;

	} else {
		b3_d.normalize();
	}

	const float yaw_d = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : 0.f;

	/*
	 * Desired heading direction from yaw:
	 *
	 * b1_c = [cos(yaw_d), sin(yaw_d), 0]^T
	 */

	const Vector3f b1_c(cosf(yaw_d), sinf(yaw_d), 0.f);

	/*
	 * Construct orthonormal desired attitude frame
	 */

	Vector3f b2_d = b3_d.cross(b1_c);

	if (b2_d.norm_squared() < 1e-6f) {
		// fallback if b3_d and b1_c are almost parallel
		b2_d = Vector3f(-sinf(yaw_d), cosf(yaw_d), 0.f);

	} else {
		b2_d.normalize();
	}

	Vector3f b1_d = b2_d.cross(b3_d);
	b1_d.normalize();

	/*
	 * Desired rotation matrix:
	 *
	 * R_d = [b1_d b2_d b3_d]
	 */

	Dcmf R_d;

	R_d(0, 0) = b1_d(0);
	R_d(1, 0) = b1_d(1);
	R_d(2, 0) = b1_d(2);

	R_d(0, 1) = b2_d(0);
	R_d(1, 1) = b2_d(1);
	R_d(2, 1) = b2_d(2);

	R_d(0, 2) = b3_d(0);
	R_d(1, 2) = b3_d(1);
	R_d(2, 2) = b3_d(2);

	/*
	 * Convert R_d to quaternion q_d
	 */

	const Quatf q_d(R_d);
	q_d.copyTo(attitude_setpoint.q_d);

	/*
	 * Fill roll, pitch, yaw fields for logging/compatibility
	 */

	const Eulerf euler_sp(q_d);

	attitude_setpoint.roll_body = euler_sp(0);
	attitude_setpoint.pitch_body = euler_sp(1);
	attitude_setpoint.yaw_body = euler_sp(2);

	/*
	 * PX4 thrust in body frame.
	 *
	 * For multicopters, thrust is along negative body z.
	 */

	attitude_setpoint.thrust_body[0] = 0.f;
	attitude_setpoint.thrust_body[1] = 0.f;
	attitude_setpoint.thrust_body[2] = -_thr_sp.norm();

	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}
