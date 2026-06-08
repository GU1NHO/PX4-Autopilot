/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file DynamixelServo.hpp
 *
 * Driver for Dynamixel servo motors (Protocol 2.0) over UART (write-only,
 * TTL half-duplex).  Designed for the Pixhawk 4 TELEM2 port (/dev/ttyS2)
 * connected to an OpenCM845 board driving XM430-W350-T servos.
 *
 * Only SYNC_WRITE is used so no status packets are expected from the servos.
 */

#pragma once

#include <lib/mixer_module/mixer_module.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>

class DynamixelServo : public ModuleBase, public OutputModuleInterface
{
public:
	static Descriptor desc;

	DynamixelServo(const char *device_name, const char *baud_rate_parameter);
	virtual ~DynamixelServo();

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

	void Run() override;

	/** @see OutputModuleInterface */
	bool updateOutputs(float outputs[MAX_ACTUATORS], unsigned num_outputs,
			   unsigned num_control_groups_updated) override;

	static constexpr int MAX_ACTUATORS = 6;

private:
	// Dynamixel Protocol 2.0 constants
	static constexpr uint8_t  DXL_HDR0          = 0xFF;
	static constexpr uint8_t  DXL_HDR1          = 0xFF;
	static constexpr uint8_t  DXL_HDR2          = 0xFD;
	static constexpr uint8_t  DXL_RESERVED      = 0x00;
	static constexpr uint8_t  DXL_BROADCAST_ID  = 0xFE;
	static constexpr uint8_t  DXL_INST_WRITE    = 0x03;
	static constexpr uint8_t  DXL_INST_SYNC_WRITE = 0x83;

	// XM430 control table
	static constexpr uint16_t ADDR_TORQUE_ENABLE  = 64;
	static constexpr uint16_t ADDR_GOAL_POSITION  = 116;
	static constexpr uint16_t LEN_GOAL_POSITION   = 4;

	// Maximum single SYNC_WRITE packet: header(4)+id(1)+len(2)+instr(1)+addr(2)+dlen(2)+6*(id(1)+data(4))+crc(2)
	static constexpr size_t MAX_PACKET_SIZE = 4 + 1 + 2 + 1 + 2 + 2 + MAX_ACTUATORS * (1 + 4) + 2;

	MixingOutput _mixing_output{"DXL", MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, false};

	uORB::SubscriptionData<actuator_armed_s> _actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	char _device_name[256];
	char _baud_rate_param[256];

	int  _uart_fd{-1};
	bool _uart_initialized{false};
	bool _torque_enabled{false};

	int initializeUART();

	// Enable/disable torque on all configured servos
	int setTorqueAll(bool enable);

	// Build and send a SYNC_WRITE packet for Goal Position
	int sendGoalPositions(uint8_t num_servos, const uint32_t *positions);

	// Write a single register on one servo (used for torque enable)
	int writeSingle(uint8_t servo_id, uint16_t address, const uint8_t *data, uint16_t data_len);

	// Low-level helpers
	static uint16_t calcCRC(const uint8_t *buf, size_t len);
	int  writeRaw(const uint8_t *buf, size_t len);

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::DXL_SERVO_CNT>) _param_servo_cnt,
		(ParamInt<px4::params::DXL_BASE_ID>)   _param_base_id
	)
};
