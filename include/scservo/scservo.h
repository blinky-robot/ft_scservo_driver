/*
 * Copyright (c) 2015, Scott K Logan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _scservo_h
#define _scservo_h

#include <stdint.h>

#define SC_MAX_DESCRIPTORS 256
#define SC_MAX_ID 0xFE
#define SC_BROADCAST_ID 0xFE
#define SC_TIMEOUT_DEFAULT 1

#ifdef __cplusplus
extern "C" {
#endif

enum SC_BAUD
{
	SC_BAUD_1M = 0,
	SC_BAUD_500K = 1,
	SC_BAUD_250K = 2,
	SC_BAUD_128K = 3,
	SC_BAUD_115200 = 4,
	SC_BAUD_76800 = 5,
	SC_BAUD_57600 = 6,
	SC_BAUD_38400 = 7,
	SC_BAUD_MAX
};

enum SC_FW_VER
{
	SC_FW_SC00 = 0,
	SC_FW_SC10 = 1,
	SC_FW_MAX
};

enum SC_ERROR
{
	SC_SUCCESS = 0,
	SC_ERROR_IO = -1,
	SC_ERROR_INVALID_PARAM = -2,
	SC_ERROR_ACCESS = -3,
	SC_ERROR_NO_DEVICE = -4,
	SC_ERROR_NOT_FOUND = -5,
	SC_ERROR_BUSY = -6,
	SC_ERROR_TIMEOUT = -7,
	SC_ERROR_NO_MEM = -11,
	SC_ERROR_OTHER = -99,
	SC_ERROR_INVALID_RESPONSE = -100,
	SC_ERROR_CHECKSUM_FAILURE = -101,
	SC_ERROR_NOT_CONNECTED = -102,
};

enum SC_FAULT
{
	SC_FAULT_NONE = 0,
	SC_FAULT_VOLTAGE = (1 << 0),
	SC_FAULT_TEMPERATURE = ( 1 << 2),
};

struct sc_info
{
	uint16_t model;
	uint16_t version;
	uint8_t reserved1;
	uint8_t id;
	uint8_t baud;
} __attribute__((packed));

struct sc_settings
{
	uint16_t min_angle_limit;
	uint16_t max_angle_limit;
	uint8_t limit_temperature;
	uint8_t max_limit_voltage;
	uint8_t min_limit_voltage;
	uint16_t max_torque;
	uint8_t alarm_led;
	uint8_t alarm_shutdown;
	uint8_t reserved1;
	uint8_t compliance_p;
	uint8_t compliance_d;
	uint8_t compliance_i;
	uint16_t punch;
	uint8_t cw_dead;
	uint8_t ccw_dead;
	uint16_t imax;
} __attribute__((packed));

struct sc_status
{
	uint16_t present_position;
	uint16_t present_speed;
	uint16_t present_load;
} __attribute__((packed));

struct sc_diag
{
	uint8_t voltage;
	uint8_t temperature;
	uint8_t error;
} __attribute__((packed));

#define SC_MAX_MSG sizeof(struct sc_settings)

/**
 * Functions
 */
void sc_close(const int scd);
int sc_open(const char *port, const enum SC_BAUD baud, const uint8_t timeout);
int sc_ping(const int scd, const uint8_t id);
int sc_read_diag(const int scd, const uint8_t id, struct sc_diag *diag);
int sc_read_goal(const int scd, const uint8_t id, uint16_t *goal_speed, uint16_t *goal_position);
int sc_read_goal_position(const int scd, const uint8_t id, uint16_t *goal_position);
int sc_read_goal_speed(const int scd, const uint8_t id, uint16_t *goal_speed);
int sc_read_info(const int scd, const uint8_t id, struct sc_info *info);
int sc_read_model(const int scd, const uint8_t id, uint16_t *model);
int sc_read_position(const int scd, const uint8_t id, uint16_t *position);
int sc_read_settings(const int scd, const uint8_t id, struct sc_settings *settings);
int sc_read_speed(const int scd, const uint8_t id, uint16_t *speed);
int sc_read_status(const int scd, const uint8_t id, struct sc_status *status);
int sc_read_temperature(const int scd, const uint8_t id, uint8_t *temperature);
int sc_read_torque_enable(const int scd, const uint8_t id, uint8_t *enable);
int sc_read_version(const int scd, const uint8_t id, uint8_t *major, uint8_t *minor);
int sc_read_vir_position(const int scd, const uint8_t id, uint16_t *vir_position);
int sc_read_voltage(const int scd, const uint8_t id, uint8_t *voltage);
const char * sc_strerror(const int error);
const char * sc_strfault(const enum SC_FAULT fault);
int sc_write_goal(const int scd, const uint8_t id, const uint16_t speed, const uint16_t position);
int sc_write_goal_position(const int scd, const uint8_t id, const uint16_t position);
int sc_write_goal_speed(const int scd, const uint8_t id, const uint16_t speed);
int sc_write_settings(const int scd, const uint8_t id, const struct sc_settings *settings);
int sc_write_torque_enable(const int scd, const uint8_t id, const uint8_t enable);

#ifdef __cplusplus
}
#endif

#endif /* _scservo_h */
