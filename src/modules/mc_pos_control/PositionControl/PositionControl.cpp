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


void PositionControl::setAttitude(const matrix::Quatf &q)
{
	if (!q.isAllFinite()
	    || q.norm_squared() < 1e-6f) {
		return;
	}

	matrix::Quatf q_normalized = q;
	q_normalized.normalize();

	_R = matrix::Dcmf(q_normalized);
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
		_positionControl();
		_velocityControl(dt);

		_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
		_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw; // TODO: better way to disable yaw control
	}

	// There has to be a valid output acceleration and thrust setpoint otherwise something went wrong
	return valid && _acc_sp.isAllFinite() && _thr_sp.isAllFinite();
}
void PositionControl::_positionControl()
{
	// Direct PD+FF law: a_sp = ddx_d + kp*ep + kv*ev
	// No computation needed here; see _velocityControl().
}

void PositionControl::_velocityControl(const float dt)
{
	// --- Constant gains (hardcoded for testing) ---
	// Replace with tuned values; keep xy and z separate since a quadrotor's
	// vertical and horizontal dynamics are rarely symmetric.
	static constexpr float KP_XY = 4.0f;
	static constexpr float KP_Z  = 4.0f;
	static constexpr float KV_XY = 4.0f;
	static constexpr float KV_Z  = 4.0f;

	const Vector3f k_p(KP_XY, KP_XY, KP_Z);
	const Vector3f k_v(KV_XY, KV_XY, KV_Z);

	// --- Position error: ep = x_d - x ---
	Vector3f e_p = _pos_sp - _pos;
	ControlMath::setZeroIfNanVector3f(e_p);

	// --- Velocity error: ev = ẋ_d - ẋ ---
	Vector3f e_v = _vel_sp - _vel;
	ControlMath::setZeroIfNanVector3f(e_v);

	// --- Feedforward acceleration: ẍ_d ---
	Vector3f acc_ff = _acc_sp;
	ControlMath::setZeroIfNanVector3f(acc_ff);

	// a_sp = ddx_d + kp*ep + kv*ev
	_acc_sp = acc_ff + e_p.emult(k_p) + e_v.emult(k_v);

	_accelerationControl();

	// NOTE: the old post-projection saturation block that used to live here
	// (clamping _thr_sp xy/z against _lim_thr_max/_lim_thr_xy_margin) has been
	// removed. It operated on the *projected, body-frame* thrust scalar
	// (fz_normalized * e3), which is no longer computed here — see
	// _accelerationControl(). The min/max clamp is now applied downstream in
	// MulticopterRateControl::Run(), after the world-frame vector below is
	// projected onto the current body-z axis.
}
void PositionControl::_accelerationControl()
{
	const Vector3f e3(0.f, 0.f, 1.f);

/*
 * Desired specific-force direction, used only to derive the desired
 * body-z axis (b3_d) below. This is *not* used to compute a thrust
 * magnitude here anymore -- see the note further down.
 *
 *     g*e3 - a_sp
 */
const Vector3f specific_thrust_direction =
	CONSTANTS_ONE_G * e3 - _acc_sp;

/*
 * Desired body-z axis:
 *
 *     R_d*e3 = (g*e3 - a_sp) / ||g*e3 - a_sp||
 */
Vector3f R_d_e3 = e3;

if (specific_thrust_direction.norm_squared() > 1e-6f) {
	R_d_e3 = specific_thrust_direction.normalized();
}

/*
 * Limit the desired tilt.
 */
ControlMath::limitTilt(R_d_e3, e3, _lim_tilt);
b3_d = R_d_e3;

/*
 * NOTE: PositionControl no longer computes a thrust value at all. It used
 * to project (g*e3 - a_sp) onto the current body-z axis, scale by
 * hover_thrust/g, and clamp to [-thr_max, -thr_min] -- all of that (the
 * hover-thrust scaling, the projection onto R, and the clamp) has moved to
 * MulticopterRateControl::Run(), which has the freshest attitude estimate.
 *
 * _thr_sp is reused purely as the transport for _acc_sp down through the
 * existing vehicle_attitude_setpoint/vehicle_rates_setpoint.thrust_body
 * field (mc_att_control passes thrust_body through untouched, so this
 * field is a free ride to the rate controller without adding a new uORB
 * topic). MulticopterRateControl::Run() reconstructs
 * "g*e3 - a_sp", scales it by its own hover_thrust estimate, projects onto
 * the current R*e3, and clamps -- reproducing exactly the formula that
 * used to live here.
 */
_thr_sp = _acc_sp;
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
	ControlMath::thrustToAttitude(_thr_sp  ,b3_d, _yaw_sp, attitude_setpoint);
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}
