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

#include <scservo/scservo.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static enum SC_BAUD strtobaud(const char *baud)
{
	if (strcmp(baud, "1000000") == 0)
	{
		return SC_BAUD_1M;
	}
	else if(strcmp(baud, "500000") == 0)
	{
		return SC_BAUD_500K;
	}
	else if(strcmp(baud, "115200") == 0)
	{
		return SC_BAUD_115200;
	}
	else if(strcmp(baud, "57600") == 0)
	{
		return SC_BAUD_57600;
	}
	else if(strcmp(baud, "38400") == 0)
	{
		return SC_BAUD_38400;
	}
	else
	{
		fprintf(stderr, "Invalid baud rate %s\n", baud);
		exit(EXIT_FAILURE);
		return SC_BAUD_MAX;
	}
}

int main(const int argc, const char *argv[])
{
	uint8_t count = 0;
	uint8_t i;
	int ret = 0;
	int scd = -1;

	if (argc != 3)
	{
		fprintf(stderr, "Usage: %s <path to device> <baud rate>\n\n", argv[0]);
		return EXIT_FAILURE;
	}

	scd = sc_open(argv[1], strtobaud(argv[2]), 1);
	if (scd < 0)
	{
		fprintf(stderr, "Failed to open port: %s\n", sc_strerror(scd));
		return scd;
	}

	for (i = 0; i <= SC_MAX_ID && i != SC_BROADCAST_ID; i++)
	{
		printf("\rPinging %d...", i);
		fflush(stdout);
		ret = sc_ping(scd, i);
		if (ret < 0)
		{
			fprintf(stderr, "\rFailed to ping device %d: %s\n", i, sc_strerror(ret));
		}
		else if (ret == 1)
		{
			count += 1;
			printf("\rFound servo at ID %d\n", i);
		}
	}
	printf("\rTotal servos: %d\n", count);

	sc_close(scd);

	return EXIT_SUCCESS;
}
