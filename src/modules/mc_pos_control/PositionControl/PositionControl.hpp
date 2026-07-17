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
 * @file PositionControl.hpp
 *
 * Geometric SE(3) translational controller (Lee, Leok, McClamroch 2010,
 * "Geometric Tracking Control of a Quadrotor UAV on SE(3)", eqs. 12 and 15).
 */

#pragma once

#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

struct PositionControlStates {
	matrix::Vector3f position;
	matrix::Vector3f velocity;
	matrix::Vector3f acceleration;
	float yaw;
	matrix::Quatf attitude; ///< vehicle attitude, used to project thrust onto the measured body z axis (eq. 15)
};

/**
 * 	Core Position-Control for MC.
 * 	Geometric SE(3) translational tracking controller (Lee2010):
 * 	pure PD feedback in acceleration space, desired body z axis from the
 * 	total specific force direction (eq. 12) and collective thrust from its
 * 	projection onto the measured body z axis (eq. 15).
 * 	Inputs:
 * 		vehicle position/velocity/yaw/attitude
 * 		desired set-point position/velocity/acceleration/yaw/yaw-speed
 * 		constraints that are stricter than global limits
 * 	Output
 * 		thrust vector and a yaw-setpoint
 *
 * 	If there is a position and a velocity set-point present, then
 * 	the velocity set-point is used as feed-forward. If feed-forward is
 * 	active, then the velocity component of the position feedback output has
 * 	priority over the feed-forward component.
 *
 * 	A setpoint that is NAN is considered as not set.
 */
class PositionControl
{
public:

	PositionControl() = default;
	~PositionControl() = default;

	// SE(3) geometric controller gains (Lee2010, mass-normalized: kx [1/s^2], kv [1/s]),
	// tunable per axis. Hardcoded tuning knobs for now (no PX4 params yet).
	// PX4 cascade equivalents: kx = MPC_*_P * MPC_*_VEL_P_ACC, kv = MPC_*_VEL_P_ACC
	// (iris: kx 1.7/1.7/4.0, kv 1.8/1.8/4.0); the paper uses kx = 16, kv = 5.6.
	static constexpr float SE3_KX_X = 4.0f;
	static constexpr float SE3_KX_Y = 4.0f;
	static constexpr float SE3_KX_Z = 4.0f;
	static constexpr float SE3_KV_X = 3.0f;
	static constexpr float SE3_KV_Y = 3.0f;
	static constexpr float SE3_KV_Z = 3.0f;


	/**
	 * Set the SE(3) controller gains, overriding the hardcoded defaults
	 * @param kx 3D vector of position error gains [1/s^2] (Lee2010 kx/m)
	 * @param kv 3D vector of velocity error gains [1/s] (Lee2010 kv/m)
	 */
	void setSE3Gains(const matrix::Vector3f &kx, const matrix::Vector3f &kv);

	/**
	 * Set the maximum velocity to execute with feed forward and position control
	 * @param vel_horizontal horizontal velocity limit
	 * @param vel_up upwards velocity limit
	 * @param vel_down downwards velocity limit
	 */
	void setVelocityLimits(const float vel_horizontal, const float vel_up, float vel_down);

	/**
	 * Set the minimum and maximum collective normalized thrust [0,1] that can be output by the controller
	 * @param min minimum thrust e.g. 0.1 or 0
	 * @param max maximum thrust e.g. 0.9 or 1
	 */
	void setThrustLimits(const float min, const float max);

	/**
	 * Set margin that is kept for horizontal control when prioritizing vertical thrust
	 * @param margin of normalized thrust that is kept for horizontal control e.g. 0.3
	 */
	void setHorizontalThrustMargin(const float margin);

	/**
	 * Set the maximum tilt angle in radians the output attitude is allowed to have
	 * @param tilt angle in radians from level orientation
	 */
	void setTiltLimit(const float tilt) { _lim_tilt = tilt; }

	/**
	 * Set the normalized hover thrust
	 * @param hover_thrust [HOVER_THRUST_MIN, HOVER_THRUST_MAX] with which the vehicle hovers not accelerating down or up with level orientation
	 */
	void setHoverThrust(const float hover_thrust) { _hover_thrust = math::constrain(hover_thrust, HOVER_THRUST_MIN, HOVER_THRUST_MAX); }

	/**
	 * Update the hover thrust from the hover thrust estimator.
	 * With the pure PD law there is no integrator to absorb the change,
	 * so the update flows to the output directly (the estimate is slow/filtered).
	 */
	void updateHoverThrust(const float hover_thrust_new) { setHoverThrust(hover_thrust_new); }

	/**
	 * Pass the current vehicle state to the controller
	 * @param PositionControlStates structure
	 */
	void setState(const PositionControlStates &states);

	/**
	 * Pass the desired setpoints
	 * Note: NAN value means no feed forward/leave state uncontrolled if there's no higher order setpoint.
	 * @param setpoint setpoints including feed-forwards to execute in update()
	 */
	void setInputSetpoint(const trajectory_setpoint_s &setpoint);

	/**
	 * Apply the SE(3) translational controller that updates the member
	 * thrust, yaw- and yawspeed-setpoints.
	 * @see _thr_sp
	 * @see _yaw_sp
	 * @see _yawspeed_sp
	 * @param dt time in seconds since last iteration (unused, kept for interface compatibility)
	 * @return true if update succeeded and output setpoint is executable, false if not
	 */
	bool update(const float dt);

	/**
	 * If set, the tilt setpoint is computed by assuming no vertical acceleration
	 */
	void decoupleHorizontalAndVecticalAcceleration(bool val) { _decouple_horizontal_and_vertical_acceleration = val; }

	/**
	 * Get the controllers output local position setpoint
	 * These setpoints are the ones which were executed on including PID output and feed-forward.
	 * The acceleration or thrust setpoints can be used for attitude control.
	 * @param local_position_setpoint reference to struct to fill up
	 */
	void getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const;

	/**
	 * Get the controllers output attitude setpoint
	 * This attitude setpoint was generated from the resulting acceleration setpoint after position and velocity control.
	 * It needs to be executed by the attitude controller to achieve velocity and position tracking.
	 * @param attitude_setpoint reference to struct to fill up
	 */
	void getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const;

	/**
	 * All setpoints are set to NAN (uncontrolled). Timestampt zero.
	 */
	static const trajectory_setpoint_s empty_trajectory_setpoint;

private:
	// The range limits of the hover thrust configuration/estimate
	static constexpr float HOVER_THRUST_MIN = 0.05f;
	static constexpr float HOVER_THRUST_MAX = 0.9f;

	bool _inputValid();

	void _se3TranslationalControl(); ///< SE(3) geometric translational control (Lee2010 eqs. 12, 15)

	// Gains
	matrix::Vector3f _gain_kx{SE3_KX_X, SE3_KX_Y, SE3_KX_Z}; ///< Position error gain [1/s^2]
	matrix::Vector3f _gain_kv{SE3_KV_X, SE3_KV_Y, SE3_KV_Z}; ///< Velocity error gain [1/s]
	matrix::Vector3f _gain_kx_over_kv{SE3_KX_X / SE3_KV_X, SE3_KX_Y / SE3_KV_Y, SE3_KX_Z / SE3_KV_Z}; ///< kx/kv, position error to implied velocity setpoint

	// Limits
	float _lim_vel_horizontal{}; ///< Horizontal velocity limit with feed forward and position control
	float _lim_vel_up{}; ///< Upwards velocity limit with feed forward and position control
	float _lim_vel_down{}; ///< Downwards velocity limit with feed forward and position control
	float _lim_thr_min{}; ///< Minimum collective thrust allowed as output [-1,0] e.g. -0.9
	float _lim_thr_max{}; ///< Maximum collective thrust allowed as output [-1,0] e.g. -0.1
	float _lim_thr_xy_margin{}; ///< Margin to keep for horizontal control when saturating prioritized vertical thrust
	float _lim_tilt{}; ///< Maximum tilt from level the output attitude is allowed to have

	float _hover_thrust{}; ///< Thrust [HOVER_THRUST_MIN, HOVER_THRUST_MAX] with which the vehicle hovers not accelerating down or up with level orientation
	bool _decouple_horizontal_and_vertical_acceleration{true}; ///< Ignore vertical acceleration setpoint to remove its effect on the tilt setpoint

	// States
	matrix::Vector3f _pos; /**< current position */
	matrix::Vector3f _vel; /**< current velocity */
	matrix::Vector3f _vel_dot; /**< velocity derivative (replacement for acceleration estimate) */
	matrix::Vector3f _body_z{0.f, 0.f, 1.f}; /**< measured body z axis R*e3 in NED, used for the eq. 15 thrust projection */
	float _yaw{}; /**< current heading */

	// Setpoints
	matrix::Vector3f _pos_sp; /**< desired position */
	matrix::Vector3f _vel_sp; /**< desired velocity */
	matrix::Vector3f _acc_sp; /**< desired acceleration */
	matrix::Vector3f _thr_sp; /**< desired thrust */
	float _yaw_sp{}; /**< desired heading */
	float _yawspeed_sp{}; /** desired yaw-speed */
};
