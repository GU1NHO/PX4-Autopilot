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
 * @file DynamixelServo.cpp
 *
 * Driver for Dynamixel servo motors (Protocol 2.0) via UART half-duplex.
 * Write-only: uses SYNC_WRITE to avoid status packet collisions on the
 * TTL half-duplex bus.
 */

#include "DynamixelServo.hpp"

#include <fcntl.h>
#include <mathlib/math/Limits.hpp>
#include <termios.h>
#include <px4_platform_common/log.h>

ModuleBase::Descriptor DynamixelServo::desc{task_spawn, custom_command, print_usage};

// ---------------------------------------------------------------------------
// CRC-16 IBM/ANSI as specified in the Dynamixel Protocol 2.0 documentation
// ---------------------------------------------------------------------------
static const uint16_t dxl_crc_table[256] = {
	0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
	0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
	0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
	0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
	0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
	0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
	0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
	0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
	0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
	0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
	0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
	0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
	0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
	0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
	0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
	0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
	0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
	0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
	0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
	0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
	0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
	0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
	0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
	0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
	0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
	0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
	0x02E0, 0x82E5, 0x82EF, 0x02EA, 0x82FB, 0x02FE, 0x02F4, 0x82F1,
	0x82D3, 0x02D6, 0x02DC, 0x82D9, 0x02C8, 0x82CD, 0x82C7, 0x02C2,
	0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
	0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
	0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
	0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
};

uint16_t DynamixelServo::calcCRC(const uint8_t *buf, size_t len)
{
	uint16_t crc = 0;

	for (size_t i = 0; i < len; i++) {
		uint16_t idx = ((crc >> 8) ^ buf[i]) & 0xFF;
		crc = (crc << 8) ^ dxl_crc_table[idx];
	}

	return crc;
}

// ---------------------------------------------------------------------------
// Packet helpers
// ---------------------------------------------------------------------------

int DynamixelServo::writeRaw(const uint8_t *buf, size_t len)
{
	ssize_t written = write(_uart_fd, buf, len);

	if (written < 0 || (size_t)written != len) {
		PX4_ERR("UART write failed: wrote %zd of %zu bytes", written, len);
		return PX4_ERROR;
	}

	return PX4_OK;
}

int DynamixelServo::writeSingle(uint8_t servo_id, uint16_t address,
				const uint8_t *data, uint16_t data_len)
{
	// Packet: header(4) + id(1) + len(2) + instr(1) + addr(2) + data + crc(2)
	const uint16_t param_len  = 2 + data_len;            // address bytes + data
	const uint16_t length_val = 1 + param_len + 2;       // instr + params + CRC
	const size_t   pkt_size   = 4 + 1 + 2 + 1 + param_len + 2;

	uint8_t buf[32];

	if (pkt_size > sizeof(buf)) { return PX4_ERROR; }

	size_t i = 0;
	buf[i++] = DXL_HDR0;
	buf[i++] = DXL_HDR1;
	buf[i++] = DXL_HDR2;
	buf[i++] = DXL_RESERVED;
	buf[i++] = servo_id;
	buf[i++] = (uint8_t)(length_val & 0xFF);
	buf[i++] = (uint8_t)(length_val >> 8);
	buf[i++] = DXL_INST_WRITE;
	buf[i++] = (uint8_t)(address & 0xFF);
	buf[i++] = (uint8_t)(address >> 8);

	for (uint16_t d = 0; d < data_len; d++) {
		buf[i++] = data[d];
	}

	uint16_t crc = calcCRC(buf, i);
	buf[i++] = (uint8_t)(crc & 0xFF);
	buf[i++] = (uint8_t)(crc >> 8);

	return writeRaw(buf, i);
}

int DynamixelServo::setTorqueAll(bool enable)
{
	const int n = math::constrain((int)_param_servo_cnt.get(), 1, MAX_ACTUATORS);
	const int base = (int)_param_base_id.get();
	const uint8_t val = enable ? 1 : 0;

	for (int s = 0; s < n; s++) {
		if (writeSingle((uint8_t)(base + s), ADDR_TORQUE_ENABLE, &val, 1) != PX4_OK) {
			PX4_ERR("torque %s failed on servo %d", enable ? "enable" : "disable", base + s);
			return PX4_ERROR;
		}
	}

	return PX4_OK;
}

int DynamixelServo::sendGoalPositions(uint8_t num_servos, const uint32_t *positions)
{
	// SYNC_WRITE packet layout:
	//   header(4) + id=0xFE(1) + len(2) + instr(1) +
	//   start_addr(2) + data_len_per_servo(2) +
	//   [servo_id(1) + goal_pos(4)] × N + crc(2)
	const uint16_t data_per_servo = LEN_GOAL_POSITION;
	const uint16_t params_len     = 2 + 2 + (uint16_t)num_servos * (1 + data_per_servo);
	const uint16_t length_val     = 1 + params_len + 2;
	const size_t   pkt_size       = 4 + 1 + 2 + 1 + params_len + 2;

	uint8_t buf[MAX_PACKET_SIZE];

	if (pkt_size > sizeof(buf)) { return PX4_ERROR; }

	const int base = (int)_param_base_id.get();

	size_t i = 0;
	buf[i++] = DXL_HDR0;
	buf[i++] = DXL_HDR1;
	buf[i++] = DXL_HDR2;
	buf[i++] = DXL_RESERVED;
	buf[i++] = DXL_BROADCAST_ID;
	buf[i++] = (uint8_t)(length_val & 0xFF);
	buf[i++] = (uint8_t)(length_val >> 8);
	buf[i++] = DXL_INST_SYNC_WRITE;
	buf[i++] = (uint8_t)(ADDR_GOAL_POSITION & 0xFF);
	buf[i++] = (uint8_t)(ADDR_GOAL_POSITION >> 8);
	buf[i++] = (uint8_t)(data_per_servo & 0xFF);
	buf[i++] = (uint8_t)(data_per_servo >> 8);

	for (uint8_t s = 0; s < num_servos; s++) {
		buf[i++] = (uint8_t)(base + s);
		buf[i++] = (uint8_t)(positions[s] & 0xFF);
		buf[i++] = (uint8_t)((positions[s] >> 8)  & 0xFF);
		buf[i++] = (uint8_t)((positions[s] >> 16) & 0xFF);
		buf[i++] = (uint8_t)((positions[s] >> 24) & 0xFF);
	}

	uint16_t crc = calcCRC(buf, i);
	buf[i++] = (uint8_t)(crc & 0xFF);
	buf[i++] = (uint8_t)(crc >> 8);

	return writeRaw(buf, i);
}

// ---------------------------------------------------------------------------
// UART initialisation
// ---------------------------------------------------------------------------

int DynamixelServo::initializeUART()
{
	int32_t baud_param = 0;
	param_get(param_find(_baud_rate_param), &baud_param);

	speed_t baud_posix;

	switch (baud_param) {
	case 9600:   baud_posix = B9600;   break;

	case 19200:  baud_posix = B19200;  break;

	case 38400:  baud_posix = B38400;  break;

	case 57600:  baud_posix = B57600;  break;

	case 115200: baud_posix = B115200; break;

	case 1000000: baud_posix = B1000000; break;

	default:
		PX4_ERR("Unsupported baud rate %d, defaulting to 57600", (int)baud_param);
		baud_posix = B57600;
		break;
	}

	_uart_fd = open(_device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_uart_fd < 0) {
		PX4_ERR("Failed to open %s", _device_name);
		return PX4_ERROR;
	}

	struct termios tty {};

	tcgetattr(_uart_fd, &tty);

	tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);

	tty.c_cflag |= CS8 | CREAD | CLOCAL;

	tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

	tty.c_oflag &= ~(OPOST | ONLCR);

	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

	tty.c_cc[VTIME] = 0;

	tty.c_cc[VMIN]  = 0;

	cfsetispeed(&tty, baud_posix);

	cfsetospeed(&tty, baud_posix);

	tcsetattr(_uart_fd, TCSANOW, &tty);

	PX4_INFO("Dynamixel: opened %s @ %d baud", _device_name, (int)baud_param);

	return PX4_OK;
}

// ---------------------------------------------------------------------------
// Module lifecycle
// ---------------------------------------------------------------------------

DynamixelServo::DynamixelServo(const char *device_name, const char *baud_rate_parameter)
	: OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default)
{
	strncpy(_device_name, device_name, sizeof(_device_name) - 1);
	_device_name[sizeof(_device_name) - 1] = '\0';

	strncpy(_baud_rate_param, baud_rate_parameter, sizeof(_baud_rate_param) - 1);
	_baud_rate_param[sizeof(_baud_rate_param) - 1] = '\0';
}

DynamixelServo::~DynamixelServo()
{
	if (_uart_fd >= 0) {
		if (_torque_enabled) {
			setTorqueAll(false);
		}

		close(_uart_fd);
	}
}

void DynamixelServo::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();
		exit_and_cleanup(desc);
		return;
	}

	_mixing_output.update();

	if (!_uart_initialized) {
		if (initializeUART() == PX4_OK) {
			_uart_initialized = true;
		}
	}

	if (_uart_initialized && !_torque_enabled) {
		if (setTorqueAll(true) == PX4_OK) {
			_torque_enabled = true;
			PX4_INFO("Dynamixel: torque enabled on %d servo(s) starting at ID %d",
				 (int)_param_servo_cnt.get(), (int)_param_base_id.get());
		}
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s pu;
		_parameter_update_sub.copy(&pu);
		updateParams();
	}

	_actuator_armed_sub.update();
	_mixing_output.updateSubscriptions(false);
}

bool DynamixelServo::updateOutputs(float outputs[MAX_ACTUATORS], unsigned num_outputs,
				   unsigned num_control_groups_updated)
{
	if (!_uart_initialized || !_torque_enabled) {
		return false;
	}

	const int n = math::constrain((int)_param_servo_cnt.get(), 1, (int)num_outputs);

	uint32_t positions[MAX_ACTUATORS];

	for (int s = 0; s < n; s++) {
		// outputs[s] is already in the [min, max] range configured in module.yaml (0–4095)
		positions[s] = (uint32_t)math::constrain((int)outputs[s], 0, 4095);
	}

	return sendGoalPositions((uint8_t)n, positions) == PX4_OK;
}

int DynamixelServo::print_status()
{
	PX4_INFO("Device: %s", _device_name);
	PX4_INFO("UART:   %s", _uart_initialized ? "initialized" : "not initialized");
	PX4_INFO("Torque: %s", _torque_enabled ? "enabled" : "disabled");
	PX4_INFO("Servos: %d (base ID %d)", (int)_param_servo_cnt.get(), (int)_param_base_id.get());
	_mixing_output.printStatus();
	return 0;
}

// ---------------------------------------------------------------------------
// Static module interface
// ---------------------------------------------------------------------------

int DynamixelServo::task_spawn(int argc, char *argv[])
{
	if (argc < 3) {
		print_usage("missing arguments");
		return PX4_ERROR;
	}

	const char *device_name = argv[1];
	const char *baud_param  = argv[2];

	DynamixelServo *instance = new DynamixelServo(device_name, baud_param);

	if (!instance) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;
	instance->ScheduleNow();
	return PX4_OK;
}

int DynamixelServo::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int DynamixelServo::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
### Description

Driver for Dynamixel servo motors using Protocol 2.0 over UART (half-duplex,
write-only).  Designed for the Pixhawk 4 TELEM2 port connected to Dynamixel
XM430 servos via an OpenCM845 board.

A SYNC_WRITE packet is sent on every output update so that all servos move
simultaneously without any status packet being returned.

Configure the serial port with DXL_SER_CFG, set DXL_SERVO_CNT to the number
of servos and DXL_BASE_ID to the ID of the first servo.

Start command:
  $ dynamixel_servo start <UART device> <baud rate parameter>

Example (manual start, usually handled via DXL_SER_CFG):
  $ dynamixel_servo start /dev/ttyS2 57600
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("dynamixel_servo", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}
