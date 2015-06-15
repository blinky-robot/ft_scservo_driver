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

#include "scservo/scservo.h"

#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

/**
 * Private Definitions
 */
struct sc_priv
{
	char *port;
	enum SC_BAUD baud;
	int fd;
	uint8_t timeout;
	int servos[SC_MAX_ID];
};

enum SC10_CMD
{
	SC10_CMD_PING = 0x01,
	SC10_CMD_READ = 0x02,
	SC10_CMD_WRITE = 0x03,
	SC10_CMD_REG_WRITE = 0x04,
	SC10_CMD_ACTION = 0x05,
	SC10_CMD_RESET = 0x06,
	SC10_CMD_SYNC_WRITE = 0x83
};

enum SC10_REG
{
	SC10_MODEL_NUMBER_L = 0,
	SC10_MODEL_NUMBER_H = 1,
	SC10_VERSION_L = 3,
	SC10_VERSION_H = 4,
	SC10_ID = 5,
	SC10_BAUD_RATE = 6,
	SC10_RETURN_DELAY_TIME = 7,
	SC10_RETURN_LEVEL = 8,
	SC10_MIN_ANGLE_LIMIT_L = 9,
	SC10_MIN_ANGLE_LIMIT_H = 10,
	SC10_MAX_ANGLE_LIMIT_L = 11,
	SC10_MAX_ANGLE_LIMIT_H = 12,
	SC10_LIMIT_TEMPERATURE = 13,
	SC10_MAX_LIMIT_VOLTAGE = 14,
	SC10_MIN_LIMIT_VOLTAGE = 15,
	SC10_MAX_TORQUE_L = 16,
	SC10_MAX_TORQUE_H = 17,
	SC10_ALARM_LED = 18,
	SC10_ALARM_SHUTDOWN = 19,
	SC10_COMPLIANCE_P = 21,
	SC10_COMPLIANCE_D = 22,
	SC10_COMPLIANCE_I = 23,
	SC10_PUNCH_H = 24,
	SC10_PUNCH_L = 25,
	SC10_CW_DEAD = 26,
	SC10_CCW_DEAD = 27,
	SC10_IMAX_L = 28,
	SC10_IMAX_H = 29,
	SC10_OFFSET_L = 30,
	SC10_OFFSET_H = 31,
	SC10_TORQUE_ENABLE = 40,
	SC10_LED = 41,
	SC10_GOAL_POSITION_L = 42,
	SC10_GOAL_POSITION_H = 43,
	SC10_GOAL_SPEED_L = 44,
	SC10_GOAL_SPEED_H = 45,
	SC10_LOCK = 48,
	SC10_PRESENT_POSITION_L = 56,
	SC10_PRESENT_POSITION_H = 57,
	SC10_PRESENT_SPEED_L = 58,
	SC10_PRESENT_SPEED_H = 59,
	SC10_PRESENT_LOAD_L = 60,
	SC10_PRESENT_LOAD_H = 61,
	SC10_PRESENT_VOLTAGE = 62,
	SC10_PRESENT_TEMPERATURE = 63,
	SC10_REGISTERED_INSTRUCTION = 64,
	SC10_ERROR = 65,
	SC10_MOVING = 66,
	SC10_VIR_POSITION_L = 67,
	SC10_VIR_POSITION_H = 68,
	SC10_CURRENT_L = 69,
	SC10_CURRENT_H = 70
};

#define SC_START_BYTE 0xFF
#define SC_ENDIAN_SWAP_16(X) (X << 8) | (X >> 8)
#define SC_ENDIAN_SWAP_16_ASSIGN(X) X = SC_ENDIAN_SWAP_16(X)

/**
 * Private Data
 */
static int sc_initialized = 0;
static struct sc_priv *scds[SC_MAX_DESCRIPTORS] = { NULL };
static const speed_t sc_baud_termios[SC_BAUD_MAX] =
{
	// SC_BAUD_1M
	B1000000,
	// SC_BAUD_500K
	B500000,
	// SC_BAUD_250K
	B0,
	// SC_BAUD_128K
	B0,
	// SC_BAUD_115200
        B115200,
	// SC_BAUD_76800
	B0,
	// SC_BAUD_57600
	B57600,
	// SC_BAUD_38400
	B38400
};

/**
 * Private Function Prototypes
 */
static uint8_t sc_checksum(const uint8_t *msg);
static int sc_errno(const int fallback);
static void sc_exit(void);
static int sc_flush(const int scd);
static int sc_init(void);
static int sc_next_descriptor(void);
static int sc_read_msg(const int scd, void *msg, const uint8_t max_len);
static int sc_read_reg(const int scd, const uint8_t id, const uint8_t reg, void *val, const uint8_t len);
static inline int sc_read_reg8(const int scd, const uint8_t id, const uint8_t reg, uint8_t *val);
static inline int sc_read_reg16(const int scd, const uint8_t id, const uint8_t reg, uint16_t *val);
static int sc_write_msg(const int scd, const uint8_t id, const void *msg, const uint8_t len);
static int sc_write_reg(const int scd, const uint8_t id, const uint8_t reg, const void *val, const uint8_t len);
static inline int sc_write_reg8(const int scd, const uint8_t id, const uint8_t reg, const uint8_t val);
static inline int sc_write_reg16(const int scd, const uint8_t id, const uint8_t reg, const uint16_t val);

/**
 * Public Functions
 */
void sc_close(const int scd)
{
	if (scd >= 0 && scd < SC_MAX_DESCRIPTORS && scds[scd] != NULL)
	{
		close(scds[scd]->fd);
		free(scds[scd]->port);
		free(scds[scd]);
		scds[scd] = NULL;
	}
}

int sc_open(const char *port, const enum SC_BAUD baud, const uint8_t timeout)
{
	int fd = -1;
	struct termios options;
	int ret = SC_SUCCESS;
	int scd = -1;

	if (port == NULL || port[0] == '\0' || baud < 0 || baud >= SC_BAUD_MAX || timeout < 1 || sc_baud_termios[baud] == B0)
	{
		ret = SC_ERROR_INVALID_PARAM;
		goto sc_pre_fail;
	}

	if (sc_initialized != 1)
	{
		ret = sc_init();
		if (ret != SC_SUCCESS)
		{
			sc_initialized = 0;
			ret = SC_ERROR_OTHER;
			goto sc_pre_fail;
		}
	}

	fd = open(port, O_NOCTTY | O_RDWR | O_SYNC);
	if (fd < 0)
	{
		ret = sc_errno(SC_ERROR_OTHER);
		goto sc_pre_fail;
	}

	scd = sc_next_descriptor();
	if (scd < 0)
	{
		ret = scd;
		goto sc_close_fail;
	}

	scds[scd] = malloc(sizeof(struct sc_priv));
	if (scds[scd] == NULL)
	{
		ret = SC_ERROR_NO_MEM;
		goto sc_close_fail;
	}

	scds[scd]->port = malloc(strlen(port) + 1);
	if (scds[scd]->port == NULL)
	{
		ret = SC_ERROR_NO_MEM;
		goto sc_free_fail;
	}

	strcpy(scds[scd]->port, port);
	scds[scd]->baud = baud;
	scds[scd]->fd = fd;
	scds[scd]->timeout = timeout;

	ret = tcgetattr(fd, &options);
	if (ret != 0)
	{
		ret = SC_ERROR_IO;
		goto sc_free_fail;
	}

	cfmakeraw(&options);
	options.c_cc[VMIN] = 0;
	options.c_cc[VTIME] = timeout;

	ret = cfsetispeed(&options, sc_baud_termios[baud]);
	if (ret != 0)
	{
		ret = SC_ERROR_IO;
		goto sc_free_fail;
	}


	ret = cfsetospeed(&options, sc_baud_termios[baud]);
	if (ret != 0)
	{
		ret = SC_ERROR_IO;
		goto sc_free_fail;
	}

	ret = tcsetattr(fd, TCSANOW, &options);
	if (ret != 0)
	{
		ret = SC_ERROR_IO;
		goto sc_free_fail;
	}

	return scd;

sc_free_fail:
	free(scds[scd]->port);
	free(scds[scd]);
	scds[scd] = NULL;

sc_close_fail:
	close(fd);

sc_pre_fail:
	return ret;
}

int sc_ping(const int scd, const uint8_t id)
{
	static const uint8_t sent_value = SC10_CMD_PING;

	uint8_t recv[6];
	int ret = SC_SUCCESS;

	if (scd < 0 || scd > SC_MAX_DESCRIPTORS || scds[scd] == NULL || id > SC_MAX_ID)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	ret = sc_flush(scd);
	if (ret != SC_SUCCESS)
	{
		return ret;
	}

	ret = sc_write_msg(scd, id, &sent_value, 1);
	if (ret != SC_SUCCESS)
	{
		return ret;
	}

	ret = sc_read_msg(scd, recv, 6);
	if (ret != SC_SUCCESS)
	{
		// If it was a timeout, that means the device isn't present
		if (ret == SC_ERROR_TIMEOUT)
		{
			return 0;
		}
		else
		{
			return ret;
		}
	}

	if (recv[0] != 0xFF || recv[1] != 0xFF || recv[3] != 2 || recv[4] != 0)
	{
		return SC_ERROR_INVALID_RESPONSE;
	}

	return 1;
}

int sc_read_diag(const int scd, const uint8_t id, uint8_t *voltage, uint8_t *temperature, uint8_t *error)
{
	int ret = SC_SUCCESS;
	uint8_t buf[4];

	if (scd < 0 || scd > SC_MAX_DESCRIPTORS || scds[scd] == NULL || id > SC_MAX_ID)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	if (voltage == NULL || temperature == NULL)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	ret = sc_read_reg(scd, id, SC10_PRESENT_VOLTAGE, buf, 4);
	if (ret == SC_SUCCESS)
	{
		*voltage = buf[0];
		*temperature = buf[1];
		*error = buf[3];
	}

	return ret;
}

int sc_read_goal(const int scd, const uint8_t id, uint16_t *goal_speed, uint16_t *goal_position)
{
	int ret = SC_SUCCESS;
	uint16_t buf[2];

	if (scd < 0 || scd > SC_MAX_DESCRIPTORS || scds[scd] == NULL || id > SC_MAX_ID)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	if (goal_speed == NULL || goal_position == NULL)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	ret = sc_read_reg(scd, id, SC10_GOAL_POSITION_L, buf, 4);
	if (ret == SC_SUCCESS)
	{
		*goal_speed = SC_ENDIAN_SWAP_16(buf[1]);
		*goal_position = SC_ENDIAN_SWAP_16(buf[0]);
	}

	return ret;
}

int sc_read_goal_position(const int scd, const uint8_t id, uint16_t *goal_position)
{
	if (scd < 0 || scd > SC_MAX_DESCRIPTORS || scds[scd] == NULL || id > SC_MAX_ID)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	if (goal_position == NULL)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	return sc_read_reg16(scd, id, SC10_GOAL_POSITION_L, goal_position);
}

int sc_read_goal_speed(const int scd, const uint8_t id, uint16_t *goal_speed)
{
	if (scd < 0 || scd > SC_MAX_DESCRIPTORS || scds[scd] == NULL || id > SC_MAX_ID)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	if (goal_speed == NULL)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	return sc_read_reg16(scd, id, SC10_GOAL_SPEED_L, goal_speed);
}

int sc_read_info(const int scd, const uint8_t id, struct sc_info *info)
{
	if (scd < 0 || scd > SC_MAX_DESCRIPTORS || scds[scd] == NULL || id > SC_MAX_ID)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	if (info == NULL)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	return sc_read_reg(scd, id, SC10_MODEL_NUMBER_L, info, sizeof(struct sc_info));
}

int sc_read_model(const int scd, const uint8_t id, uint16_t *model)
{
	if (scd < 0 || scd > SC_MAX_DESCRIPTORS || scds[scd] == NULL || id > SC_MAX_ID)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	if (model == NULL)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	return sc_read_reg16(scd, id, SC10_MODEL_NUMBER_L, model);
}

int sc_read_position(const int scd, const uint8_t id, uint16_t *position)
{
	if (scd < 0 || scd > SC_MAX_DESCRIPTORS || scds[scd] == NULL || id > SC_MAX_ID)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	if (position == NULL)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	return sc_read_reg16(scd, id, SC10_PRESENT_POSITION_L, position);
}

int sc_read_settings(const int scd, const uint8_t id, struct sc_settings *settings)
{
	int ret = SC_SUCCESS;

	if (scd < 0 || scd > SC_MAX_DESCRIPTORS || scds[scd] == NULL || id > SC_MAX_ID)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	if (settings == NULL)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	ret = sc_read_reg(scd, id, SC10_MIN_ANGLE_LIMIT_L, settings, sizeof(struct sc_settings));
	if (ret == SC_SUCCESS)
	{
		SC_ENDIAN_SWAP_16_ASSIGN(settings->min_angle_limit);
		SC_ENDIAN_SWAP_16_ASSIGN(settings->max_angle_limit);
		SC_ENDIAN_SWAP_16_ASSIGN(settings->max_torque);
		SC_ENDIAN_SWAP_16_ASSIGN(settings->imax);
	}

	return ret;
}

int sc_read_speed(const int scd, const uint8_t id, uint16_t *speed)
{
	int ret = 0;

	if (scd < 0 || scd > SC_MAX_DESCRIPTORS || scds[scd] == NULL || id > SC_MAX_ID)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	if (speed == NULL)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	ret = sc_read_reg16(scd, id, SC10_PRESENT_SPEED_L, speed);
	if (ret == SC_SUCCESS && *speed & (1 << 10))
	{
		*speed = -(*speed & 1023);
	}

	return ret;
}

int sc_read_status(const int scd, const uint8_t id, struct sc_status *status)
{
	int ret = SC_SUCCESS;

	if (scd < 0 || scd > SC_MAX_DESCRIPTORS || scds[scd] == NULL || id > SC_MAX_ID)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	if (status == NULL)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	ret = sc_read_reg(scd, id, SC10_PRESENT_POSITION_L, status, sizeof(struct sc_status));
	if (ret == SC_SUCCESS)
	{
		SC_ENDIAN_SWAP_16_ASSIGN(status->present_position);
		SC_ENDIAN_SWAP_16_ASSIGN(status->present_speed);
		SC_ENDIAN_SWAP_16_ASSIGN(status->present_load);
	}

	return ret;
}

int sc_read_temperature(const int scd, const uint8_t id, uint8_t *temperature)
{
	if (scd < 0 || scd > SC_MAX_DESCRIPTORS || scds[scd] == NULL || id > SC_MAX_ID)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	if (temperature == NULL)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	return sc_read_reg8(scd, id, SC10_PRESENT_TEMPERATURE, temperature);
}

int sc_read_torque_enable(const int scd, const uint8_t id, uint8_t *enable)
{
	if (scd < 0 || scd > SC_MAX_DESCRIPTORS || scds[scd] == NULL || id > SC_MAX_ID)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	if (enable == NULL)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	return sc_read_reg8(scd, id, SC10_TORQUE_ENABLE, enable);
}

int sc_read_version(const int scd, const uint8_t id, uint8_t *major, uint8_t *minor)
{
	int ret = 0;
	uint16_t version;

	if (scd < 0 || scd > SC_MAX_DESCRIPTORS || scds[scd] == NULL || id > SC_MAX_ID)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	if (major == NULL || minor == NULL)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	ret = sc_read_reg16(scd, id, SC10_VERSION_L, &version);
	if (ret == SC_SUCCESS)
	{
		*major = version >> 8;
		*minor = version & 0xFF;
	}

	return ret;
}

int sc_read_vir_position(const int scd, const uint8_t id, uint16_t *vir_position)
{
	if (scd < 0 || scd > SC_MAX_DESCRIPTORS || scds[scd] == NULL || id > SC_MAX_ID)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	if (vir_position == NULL)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	return sc_read_reg16(scd, id, SC10_VIR_POSITION_L, vir_position);
}

int sc_read_voltage(const int scd, const uint8_t id, uint8_t *voltage)
{
	if (scd < 0 || scd > SC_MAX_DESCRIPTORS || scds[scd] == NULL || id > SC_MAX_ID)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	if (voltage == NULL)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	return sc_read_reg8(scd, id, SC10_PRESENT_VOLTAGE, voltage);
}

int sc_write_goal_position(const int scd, const uint8_t id, const uint16_t position)
{
	if (scd < 0 || scd > SC_MAX_DESCRIPTORS || scds[scd] == NULL || id > SC_MAX_ID)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	return sc_write_reg16(scd, id, SC10_GOAL_POSITION_L, position);
}

int sc_write_goal_speed(const int scd, const uint8_t id, const uint16_t speed)
{
	if (scd < 0 || scd > SC_MAX_DESCRIPTORS || scds[scd] == NULL || id > SC_MAX_ID)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	return sc_write_reg16(scd, id, SC10_GOAL_SPEED_L, speed);
}

int sc_write_goal(const int scd, const uint8_t id, const uint16_t speed, const uint16_t position)
{
	const uint16_t buf[2] = { (position << 8) | (position >> 8), (speed << 8) | (speed >> 8) };

	if (scd < 0 || scd > SC_MAX_DESCRIPTORS || scds[scd] == NULL || id > SC_MAX_ID)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	return sc_write_reg(scd, id, SC10_GOAL_POSITION_L, buf, 4);
}

int sc_write_settings(const int scd, const uint8_t id, const struct sc_settings *settings)
{
	struct sc_settings buf;

	if (scd < 0 || scd > SC_MAX_DESCRIPTORS || scds[scd] == NULL || id > SC_MAX_ID)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	if (settings == NULL)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	buf = *settings;

	SC_ENDIAN_SWAP_16_ASSIGN(buf.min_angle_limit);
	SC_ENDIAN_SWAP_16_ASSIGN(buf.max_angle_limit);
	SC_ENDIAN_SWAP_16_ASSIGN(buf.max_torque);
	SC_ENDIAN_SWAP_16_ASSIGN(buf.imax);

	return sc_write_reg(scd, id, SC10_MIN_ANGLE_LIMIT_L, &buf, sizeof(struct sc_settings));
}

int sc_write_torque_enable(const int scd, const uint8_t id, const uint8_t enable)
{
	if (scd < 0 || scd > SC_MAX_DESCRIPTORS || scds[scd] == NULL || id > SC_MAX_ID)
	{
		return SC_ERROR_INVALID_PARAM;
	}

	return sc_write_reg(scd, id, SC10_TORQUE_ENABLE, &enable, 1);
}

/**
 * Private Functions
 */
static uint8_t sc_checksum(const uint8_t *msg)
{
	uint8_t ret = 0;
	uint8_t len = msg[3] + 3;

	while (len > 2)
	{
		ret += msg[--len];
	}

	return ~ret;
}

static int sc_errno(const int fallback)
{
	switch (errno)
	{
	case EPERM:
	case EACCES:
		return SC_ERROR_ACCESS;
	case ENOENT:
	case ENXIO:
	case ENODEV:
	case EISDIR:
		return SC_ERROR_NO_DEVICE;
	case ENOMEM:
		return SC_ERROR_NO_MEM;
	case EBUSY:
		return SC_ERROR_BUSY;
	}

	return fallback;
}

static void sc_exit(void)
{
	int scd;

	for (scd = 0; scd < SC_MAX_DESCRIPTORS; scd++)
	{
		sc_close(scd);
	}
}

static int sc_flush(const int scd)
{
	int ret;

	ret = tcflush(scds[scd]->fd, TCIOFLUSH);
	if (ret != 0)
	{
		return sc_errno(SC_ERROR_IO);
	}

	return SC_SUCCESS;
}

static int sc_init(void)
{
	sc_initialized = 1;

	// Note that this only works on glibc *nix systems (not all *BSD)
	return (atexit(sc_exit) == 0) ? SC_SUCCESS : SC_ERROR_OTHER;
}

static int sc_next_descriptor(void)
{
	int scd;

	for (scd = 0; scd < SC_MAX_DESCRIPTORS; scd++)
	{
		if (scds[scd] == NULL)
		{
			return scd;
		}
	}

	return SC_ERROR_NO_MEM;
}

static int sc_read_msg(const int scd, void *msg, const uint8_t max_len)
{
	int ret = SC_SUCCESS;
	uint8_t len;
	uint8_t *ptr = msg;

	ret = read(scds[scd]->fd, ptr++, 1);
	if (ret == 0)
	{
		return SC_ERROR_TIMEOUT;
	}
	else if (ret != 1)
	{
		return sc_errno(SC_ERROR_IO);
	}

	ret = read(scds[scd]->fd, ptr++, 1);
	if (ret != 1)
	{
		return sc_errno(SC_ERROR_IO);
	}

	ret = read(scds[scd]->fd, ptr, 2);
	if (ret != 2)
	{
		if (ret == 1)
		{
			ret = read(scds[scd]->fd, &ptr[1], 1);
			if (ret != 1)
			{
				return sc_errno(SC_ERROR_IO);
			}
		}
		else
		{
			return sc_errno(SC_ERROR_IO);
		}
	}

	len = ptr[1];

	if (len + 4 > max_len)
	{
		return SC_ERROR_NO_MEM;
	}

	ptr += 2;

	while (len > 0)
	{
		ret = read(scds[scd]->fd, ptr, len);
		if (ret < 1)
		{
			return sc_errno(SC_ERROR_IO);
		}

		ptr += ret;
		len -= ret;
	}

	if (sc_checksum(msg) != ptr[-1])
	{
		return SC_ERROR_CHECKSUM_FAILURE;
	}

	return SC_SUCCESS;
}

static int sc_read_reg(const int scd, const uint8_t id, const uint8_t reg, void *val, const uint8_t len)
{
	uint8_t buf[SC_MAX_MSG] = { SC10_CMD_READ, reg, len };
	int ret = SC_SUCCESS;

	ret = sc_flush(scd);
	if (ret != SC_SUCCESS)
	{
		return ret;
	}

	ret = sc_write_msg(scd, id, buf, 3);
	if (ret != SC_SUCCESS)
	{
		return ret;
	}

	ret = sc_read_msg(scd, buf, len + 6);
	if (ret != SC_SUCCESS)
	{
		return ret;
	}

	if (buf[0] != 0xFF || buf[1] != 0xFF || buf[3] != len + 2)
	{
		return SC_ERROR_INVALID_RESPONSE;
	}

	memcpy(val, &buf[5], len);

	return SC_SUCCESS;
}

static inline int sc_read_reg8(const int scd, const uint8_t id, const uint8_t reg, uint8_t *val)
{
	return sc_read_reg(scd, id, reg, val, 1);
}

static inline int sc_read_reg16(const int scd, const uint8_t id, const uint8_t reg, uint16_t *val)
{
	int ret = SC_SUCCESS;

	ret = sc_read_reg(scd, id, reg, val, 2);
	if (ret == SC_SUCCESS)
	{
		SC_ENDIAN_SWAP_16_ASSIGN(*val);
	}

	return ret;
}

const char * sc_strerror(const int error)
{
	switch(error)
	{
	case SC_SUCCESS:
		return "Success";
	case SC_ERROR_IO:
		return "Input/Output error";
	case SC_ERROR_INVALID_PARAM:
		return "Invalid parameter";
	case SC_ERROR_ACCESS:
		return "Access denied";
	case SC_ERROR_NO_DEVICE:
		return "No such device";
	case SC_ERROR_NOT_FOUND:
		return "Not found";
	case SC_ERROR_BUSY:
		return "Device or resource busy";
	case SC_ERROR_TIMEOUT:
		return "Input/Output timeout";
	case SC_ERROR_NO_MEM:
		return "Out of memory";
	case SC_ERROR_OTHER:
		return "General error";
	case SC_ERROR_INVALID_RESPONSE:
		return "Invalid response from device";
	case SC_ERROR_CHECKSUM_FAILURE:
		return "Checksum failure";
	case SC_ERROR_NOT_CONNECTED:
		return "Not Connected";
	default:
		return "Unknown error";
	}
}

static int sc_write_msg(const int scd, const uint8_t id, const void *msg, const uint8_t len)
{
	uint8_t buf[SC_MAX_MSG] = { SC_START_BYTE, SC_START_BYTE, id, len + 1 };
	uint8_t real_len = len + 4;
	int ret = SC_SUCCESS;

	memcpy(&buf[4], msg, len);

	buf[real_len] = sc_checksum(buf);
	real_len += 1;

	ret = write(scds[scd]->fd, buf, real_len);
	if (ret != real_len)
	{
		return sc_errno(SC_ERROR_IO);
	}

	return SC_SUCCESS;
}

static int sc_write_reg(const int scd, const uint8_t id, const uint8_t reg, const void *val, const uint8_t len)
{
	uint8_t buf[SC_MAX_MSG] = { SC10_CMD_WRITE, reg };
	int ret = SC_SUCCESS;

	memcpy(&buf[2], val, len);

	ret = sc_flush(scd);
	if (ret != SC_SUCCESS)
	{
		return ret;
	}

	ret = sc_write_msg(scd, id, buf, len + 2);
	if (ret != SC_SUCCESS)
	{
		return ret;
	}

	ret = sc_read_msg(scd, buf, 10);
	if (ret != SC_SUCCESS)
	{
		return ret;
	}

	if (buf[0] != 0xFF || buf[1] != 0xFF || buf[3] != 2 || buf[4] != 0)
	{
		return SC_ERROR_INVALID_RESPONSE;
	}

	return SC_SUCCESS;
}

static inline int sc_write_reg8(const int scd, const uint8_t id, const uint8_t reg, const uint8_t val)
{
	return sc_write_reg(scd, id, reg, &val, 1);
}

static inline int sc_write_reg16(const int scd, const uint8_t id, const uint8_t reg, const uint16_t val)
{
	const uint16_t val_tmp = SC_ENDIAN_SWAP_16(val);

	return sc_write_reg(scd, id, reg, &val_tmp, 2);
}

