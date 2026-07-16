/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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

#include <gtest/gtest.h>
#include <AttitudeControl.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

TEST(AttitudeControlTest, AllZeroCase)
{
	AttitudeControl attitude_control;
	Vector3f rate_setpoint = attitude_control.update(Quatf());
	EXPECT_EQ(rate_setpoint, Vector3f());
}

class AttitudeControlConvergenceTest : public ::testing::Test
{
public:
	AttitudeControlConvergenceTest()
	{
		_attitude_control.setSE3AttitudeGains(Vector3f(.5f, .6f, .3f));
		_attitude_control.setRateLimit(Vector3f(100.f, 100.f, 100.f));
	}

	void checkConvergence()
	{
		int i; // need function scope to check how many steps

		_attitude_control.setAttitudeSetpoint(_quat_goal, 0.f);

		// Note: the geometric error norm sin(α) legitimately grows while the error angle α
		// shrinks from near 180 towards 90 degrees, so unlike the previous quaternion law
		// the output norm is not monotonic; convergence is checked on the attitude directly.
		for (i = 100; i > 0; i--) {
			// run attitude control to get rate setpoints
			const Vector3f rate_setpoint = _attitude_control.update(_quat_state);
			EXPECT_TRUE(rate_setpoint.isAllFinite());
			// rotate the simulated state quaternion according to the rate setpoint
			_quat_state = _quat_state * Quatf(AxisAnglef(rate_setpoint));
			_quat_state = -_quat_state; // antipodal flips represent the same rotation, the matrix based law is immune

			if (_quat_state.canonical() == _quat_goal.canonical()) {
				break;
			}
		}

		EXPECT_EQ(_quat_state.canonical(), _quat_goal.canonical());
		// it shouldn't have taken longer than an iteration timeout to converge
		EXPECT_GT(i, 0);
	}

	AttitudeControl _attitude_control;
	Quatf _quat_state;
	Quatf _quat_goal;
};

TEST_F(AttitudeControlConvergenceTest, AttitudeControlConvergence)
{
	const int inputs = 8;

	const Quatf QArray[inputs] = {
		Quatf(),
		Quatf(0, 1, 0, 0),
		Quatf(0, 0, 1, 0),
		Quatf(0, 0, 0, 1),
		Quatf(0.698f, 0.024f, -0.681f, -0.220f),
		Quatf(-0.820f, -0.313f, 0.225f, -0.423f),
		Quatf(0.599f, -0.172f, 0.755f, -0.204f),
		Quatf(0.216f, -0.662f, 0.290f, -0.656f)
	};

	for (int i = 0; i < inputs; i++) {
		for (int j = 0; j < inputs; j++) {
			printf("--- Input combination: %d %d\n", i, j);
			_quat_state = QArray[i];
			_quat_goal = QArray[j];
			_quat_state.normalize();
			_quat_goal.normalize();
			checkConvergence();
		}
	}
}

TEST(AttitudeControlTest, YawGain)
{
	// GIVEN: a pure yaw turn command
	AttitudeControl attitude_control;
	const float yaw_gain = 2.8f;
	const float yaw_sp = .1f;
	Quatf pure_yaw_attitude(cosf(yaw_sp / 2.f), 0, 0, sinf(yaw_sp / 2.f));
	attitude_control.setSE3AttitudeGains(Vector3f(6.5f, 6.5f, yaw_gain));
	attitude_control.setRateLimit(Vector3f(1000.f, 1000.f, 1000.f));
	attitude_control.setAttitudeSetpoint(pure_yaw_attitude, 0.f);

	// WHEN: we run one iteration of the controller
	const Vector3f rate_setpoint = attitude_control.update(Quatf());

	// THEN: no actuation in roll, pitch
	EXPECT_EQ(Vector2f(rate_setpoint), Vector2f());
	// THEN: yaw actuation sin(error) * gain towards the setpoint (eq. 10 error profile)
	EXPECT_NEAR(rate_setpoint(2), sinf(yaw_sp) * yaw_gain, 1e-4f);
}

TEST(AttitudeControlTest, GeometricErrorSmallAngle)
{
	// GIVEN: the vehicle rolled ahead of a level attitude setpoint
	AttitudeControl attitude_control;
	attitude_control.setSE3AttitudeGains(Vector3f(4.f, 4.f, 2.8f));
	attitude_control.setRateLimit(Vector3f(1000.f, 1000.f, 1000.f));
	attitude_control.setAttitudeSetpoint(Quatf(), 0.f);
	const float roll_error = .2f;
	const Quatf q_state(Eulerf(roll_error, 0.f, 0.f));

	// WHEN: we run one iteration of the controller
	const Vector3f rate_setpoint = attitude_control.update(q_state);

	// THEN: eR = sin(error) about body x, rate setpoint = -kR * eR (eq. 10 sign and magnitude)
	EXPECT_NEAR(rate_setpoint(0), -4.f * sinf(roll_error), 1e-4f);
	EXPECT_NEAR(rate_setpoint(1), 0.f, 1e-4f);
	EXPECT_NEAR(rate_setpoint(2), 0.f, 1e-4f);
}

TEST(AttitudeControlTest, CriticalPointEscape)
{
	// GIVEN: a 180 degree yaw error, where the eq. 10 error eR vanishes (critical point of Ψ)
	AttitudeControl attitude_control;
	attitude_control.setSE3AttitudeGains(Vector3f(4.f, 4.f, 2.8f));
	attitude_control.setRateLimit(Vector3f(1000.f, 1000.f, 1000.f));
	attitude_control.setAttitudeSetpoint(Quatf(0.f, 0.f, 0.f, 1.f), 0.f);

	// WHEN: we run one iteration of the controller
	const Vector3f rate_setpoint = attitude_control.update(Quatf());

	// THEN: the escape commands a finite, full-scale rotation about the error axis instead of stalling
	EXPECT_TRUE(rate_setpoint.isAllFinite());
	EXPECT_GT(rate_setpoint.norm(), 1.f);
	EXPECT_NEAR(fabsf(rate_setpoint(2)), 2.8f, 1e-4f);
}
