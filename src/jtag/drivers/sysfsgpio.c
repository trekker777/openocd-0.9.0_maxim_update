/***************************************************************************
 *   Copyright (C) 2012 by Creative Product Design, marc @ cpdesign.com.au *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

/* 2014-12: Addition of the SWD protocol support is based on the initial work
 * on bcm2835gpio.c by Paul Fertser and modifications by Jean-Christian de Rivaz. */

/**
 * @file
 * This driver implements a bitbang jtag interface using gpio lines via
 * sysfs.
 * The aim of this driver implementation is use system GPIOs but avoid the
 * need for a additional kernel driver.
 * (Note memory mapped IO is another option, however it doesn't mix well with
 * the kernel gpiolib driver - which makes sense I guess.)
 *
 * A gpio is required for tck, tms, tdi and tdo. One or both of srst and trst
 * must be also be specified. The required jtag gpios are specified via the
 * sysfsgpio_jtag_nums command or the relevant sysfsgpio_XXX_num commang.
 * The srst and trst gpios are set via the sysfsgpio_srst_num and
 * sysfsgpio_trst_num respectively. GPIO numbering follows the kernel
 * convention of starting from 0.
 *
 * The gpios should not be in use by another entity, and must not be requested
 * by a kernel driver without also being exported by it (otherwise they can't
 * be exported by sysfs).
 *
 * The sysfs gpio interface can only manipulate one gpio at a time, so the
 * bitbang write handler remembers the last state for tck, tms, tdi to avoid
 * superfluous writes.
 * For speed the sysfs "value" entry is opened at init and held open.
 * This results in considerable gains over open-write-close (45s vs 900s)
 *
 * Further work could address:
 *  -srst and trst open drain/ push pull
 *  -configurable active high/low for srst & trst
 */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include "bitbang.h"



























#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifndef _WIN32
#include <sys/un.h>
#include <netdb.h>
#endif
#include <jtag/interface.h>
#include "bitbang.h"

/* arbitrary limit on host name length: */
#define REMOTE_BITBANG_HOST_MAX 255

#define REMOTE_BITBANG_RAISE_ERROR(expr ...) \
	do { \
		LOG_ERROR(expr); \
		LOG_ERROR("Terminating openocd."); \
		exit(-1); \
	} while (0)

static char *remote_bitbang_host;
static char *remote_bitbang_port;

FILE *remote_bitbang_in;
FILE *remote_bitbang_out;

static void remote_bitbang_putc(int c)
{
	if (EOF == fputc(c, remote_bitbang_out))
		REMOTE_BITBANG_RAISE_ERROR("remote_bitbang_putc: %s", strerror(errno));
}

static void remote_bitbang_putbuffer(uint8_t *buffer, int count)
{
	#if 0
	int i;
	for (i = 0; i < count; i++) { 
		if (EOF == fputc(buffer[i], remote_bitbang_out)) { 
			REMOTE_BITBANG_RAISE_ERROR("remote_bitbang_putc: %s", strerror(errno));
			break;
		}
	}
	#else 
	fwrite(buffer, sizeof(uint8_t), count, remote_bitbang_out);
	#endif
}

static int remote_bitbang_quit(void)
{
	if (EOF == fputc('Q', remote_bitbang_out)) {
		LOG_ERROR("fputs: %s", strerror(errno));
		return ERROR_FAIL;
	}

	if (EOF == fflush(remote_bitbang_out)) {
		LOG_ERROR("fflush: %s", strerror(errno));
		return ERROR_FAIL;
	}

	/* We only need to close one of the FILE*s, because they both use the same */
	/* underlying file descriptor. */
	if (EOF == fclose(remote_bitbang_out)) {
		LOG_ERROR("fclose: %s", strerror(errno));
		return ERROR_FAIL;
	}

	free(remote_bitbang_host);
	free(remote_bitbang_port);

	LOG_INFO("remote_bitbang interface quit");
	return ERROR_OK;
}

/* Get the next read response. */
static int remote_bitbang_rread(void)
{
	if (EOF == fflush(remote_bitbang_out)) {
		remote_bitbang_quit();
		REMOTE_BITBANG_RAISE_ERROR("fflush: %s", strerror(errno));
	}

	int c = fgetc(remote_bitbang_in);
	switch (c) {
		case '0':
			return 0;
		case '1':
			return 1;
		default:
			remote_bitbang_quit();
			REMOTE_BITBANG_RAISE_ERROR(
					"remote_bitbang: invalid read response: %c(%i)", c, c);
	}
}

static int remote_bitbang_read_buffer(uint8_t *buffer, int count)
{
	int i;
	int val;
	if (EOF == fflush(remote_bitbang_out)) {
		remote_bitbang_quit();
		REMOTE_BITBANG_RAISE_ERROR("fflush: %s", strerror(errno));
		return -1;
	}

	for (i = 0; i < count; i++) {
		val = fgetc(remote_bitbang_in);
		buffer[i] = val;
	}
	return 0;
}

static int remote_bitbang_read(void)
{
	remote_bitbang_putc('R');
	return remote_bitbang_rread();
}

static void remote_bitbang_write(int tck, int tms, int tdi)
{
	char c = '0' + ((tck ? 0x4 : 0x0) | (tms ? 0x2 : 0x0) | (tdi ? 0x1 : 0x0));
	remote_bitbang_putc(c);
}

static void remote_bitbang_reset(int trst, int srst)
{
	char c = 'r' + ((trst ? 0x2 : 0x0) | (srst ? 0x1 : 0x0));
	remote_bitbang_putc(c);
}

static void remote_bitbang_blink(int on)
{
	char c = on ? 'B' : 'b';
	remote_bitbang_putc(c);
}
/*
static struct bitbang_interface remote_bitbang_bitbang = {
	.read = &remote_bitbang_read,
	.write = &remote_bitbang_write,
	.reset = &remote_bitbang_reset,
	.blink = &remote_bitbang_blink,
};
*/
static int remote_bitbang_init_tcp(void)
{
	struct addrinfo hints = { .ai_family = AF_UNSPEC, .ai_socktype = SOCK_STREAM };
	struct addrinfo *result, *rp;
	int fd;

	LOG_INFO("Connecting to %s:%s",
			remote_bitbang_host ? remote_bitbang_host : "localhost",
			remote_bitbang_port);

	/* Obtain address(es) matching host/port */
	int s = getaddrinfo(remote_bitbang_host, remote_bitbang_port, &hints, &result);
	if (s != 0) {
		LOG_ERROR("getaddrinfo: %s\n", gai_strerror(s));
		return ERROR_FAIL;
	}

	/* getaddrinfo() returns a list of address structures.
	 Try each address until we successfully connect(2).
	 If socket(2) (or connect(2)) fails, we (close the socket
	 and) try the next address. */

	for (rp = result; rp != NULL ; rp = rp->ai_next) {
		fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
		if (fd == -1)
			continue;

		if (connect(fd, rp->ai_addr, rp->ai_addrlen) != -1)
			break; /* Success */

		close(fd);
	}

	freeaddrinfo(result); /* No longer needed */

	if (rp == NULL) { /* No address succeeded */
		LOG_ERROR("Failed to connect: %s", strerror(errno));
		return ERROR_FAIL;
	}

	return fd;
}

static int remote_bitbang_init_unix(void)
{
	if (remote_bitbang_host == NULL) {
		LOG_ERROR("host/socket not specified");
		return ERROR_FAIL;
	}

	LOG_INFO("Connecting to unix socket %s", remote_bitbang_host);
	int fd = socket(PF_UNIX, SOCK_STREAM, 0);
	if (fd < 0) {
		LOG_ERROR("socket: %s", strerror(errno));
		return ERROR_FAIL;
	}

	struct sockaddr_un addr;
	addr.sun_family = AF_UNIX;
	strncpy(addr.sun_path, remote_bitbang_host, sizeof(addr.sun_path));
	addr.sun_path[sizeof(addr.sun_path)-1] = '\0';

	if (connect(fd, (struct sockaddr *)&addr, sizeof(struct sockaddr_un)) < 0) {
		LOG_ERROR("connect: %s", strerror(errno));
		return ERROR_FAIL;
	}

	return fd;
}


COMMAND_HANDLER(remote_bitbang_handle_remote_bitbang_port_command)
{
	if (CMD_ARGC == 1) {
		uint16_t port;
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], port);
		free(remote_bitbang_port);
		remote_bitbang_port = port == 0 ? NULL : strdup(CMD_ARGV[0]);
		LOG_INFO("art-swd driver remote port setup: %s", remote_bitbang_port);
		return ERROR_OK;
	}
	return ERROR_COMMAND_SYNTAX_ERROR;
}

COMMAND_HANDLER(remote_bitbang_handle_remote_bitbang_host_command)
{
	if (CMD_ARGC == 1) {
		free(remote_bitbang_host);
		remote_bitbang_host = strdup(CMD_ARGV[0]);
		LOG_INFO("art-swd driver remote host setup: %s", remote_bitbang_host);
		return ERROR_OK;
	}
	return ERROR_COMMAND_SYNTAX_ERROR;
}

static const struct command_registration remote_bitbang_command_handlers[] = {
	{
		.name = "art_port",
		.handler = remote_bitbang_handle_remote_bitbang_port_command,
		.mode = COMMAND_CONFIG,
		.help = "Set the port to use to connect to the remote jtag.\n"
			"  if 0 or unset, use unix sockets to connect to the remote jtag.",
		.usage = "port_number",
	},
	{
		.name = "art_host",
		.handler = remote_bitbang_handle_remote_bitbang_host_command,
		.mode = COMMAND_CONFIG,
		.help = "Set the host to use to connect to the remote jtag.\n"
			"  if port is 0 or unset, this is the name of the unix socket to use.",
		.usage = "host_name",
	},
	COMMAND_REGISTRATION_DONE,
};

static void remote_swdio_drive(bool is_output)
{
	char c = is_output ? 'D' : 'd';
	remote_bitbang_putc(c);
}

static int remote_swdio_read(void)
{
	remote_bitbang_putc('I');
	return remote_bitbang_rread();
}

//
// read command structure
//
typedef struct {
  uint8_t id;
  uint8_t cmd;
  uint32_t data;
} __attribute__ ((packed)) cmd_readreg_t;
typedef struct {
  uint8_t ack;
  uint32_t data;
  uint8_t parity;
} __attribute__ ((packed)) cmd_readreg_resp_t;

//
// write command structure
//
typedef struct {
  uint8_t id;
  uint8_t cmd;
  uint32_t data;
} __attribute__ ((packed)) cmd_writereg_t;
typedef struct {
  uint8_t ack;
} __attribute__ ((packed)) cmd_writereg_resp_t;

static void remote_swdio_write_reg(uint8_t cmd, uint8_t *ack, uint32_t data) {
	cmd_writereg_t packet;
	cmd_writereg_resp_t response;
	int result;

	packet.id = 'W';
	packet.cmd = cmd;
	packet.data = data;
	remote_bitbang_putbuffer((uint8_t *)&packet, sizeof(packet));
	fflush(remote_bitbang_out);
	
	result = remote_bitbang_read_buffer((uint8_t *)&response, sizeof(response));
	if (result == -1) {
		exit(-1);
	}
	*ack = response.ack;
}

static void remote_swdio_read_reg(uint8_t cmd, uint8_t *ack, uint32_t *data, uint8_t *parity) {
	cmd_readreg_t packet;
	cmd_readreg_resp_t response;
	int result;

	packet.id = 'E';
	packet.cmd = cmd;
	remote_bitbang_putbuffer((uint8_t *)&packet, sizeof(packet));
	fflush(remote_bitbang_out);
	
	result = remote_bitbang_read_buffer((uint8_t *)&response, sizeof(response));
	if (result == -1) {
		exit(-1);
	}
	*ack = response.ack;
	*data = response.data;
	*parity = response.parity;
}






void queue_swdio_write_reg(uint8_t cmd, uint32_t data) {
	cmd_writereg_t packet;
	//cmd_writereg_resp_t response;
	//int result;

	packet.id = 'W';
	packet.cmd = cmd;
	packet.data = data;
	remote_bitbang_putbuffer((uint8_t *)&packet, sizeof(packet));
	//fflush(remote_bitbang_out);
	
	//result = remote_bitbang_read_buffer((uint8_t *)&response, sizeof(response));
	//if (result == -1) {
	//	exit(-1);
	//}
	//*ack = response.ack;
}

void queue_swdio_read_reg(uint8_t cmd) {
	cmd_readreg_t packet;
	//cmd_readreg_resp_t response;
	//int result;

	packet.id = 'E';
	packet.cmd = cmd;
	remote_bitbang_putbuffer((uint8_t *)&packet, sizeof(packet));
	//fflush(remote_bitbang_out);
	
	//result = remote_bitbang_read_buffer((uint8_t *)&response, sizeof(response));
	//if (result == -1) {
	//	exit(-1);
	//}
	//*ack = response.ack;
	//*data = response.data;
	//*parity = response.parity;
}

void response_swdio_write_reg(uint8_t *ack) {
	cmd_writereg_resp_t response;
	int result;

	//fflush(remote_bitbang_out);
	
	result = remote_bitbang_read_buffer((uint8_t *)&response, sizeof(response));
	if (result == -1) {
		exit(-1);
	}
	*ack = response.ack;
}

void response_swdio_read_reg(uint8_t *ack, uint32_t *data, uint8_t *parity) {
	cmd_readreg_resp_t response;
	int result;

	result = remote_bitbang_read_buffer((uint8_t *)&response, sizeof(response));
	if (result == -1) {
		exit(-1);
	}
	*ack = response.ack;
	*data = response.data;
	*parity = response.parity;
}

void swdio_flush(void) {
	fflush(remote_bitbang_out);
}

static struct bitbang_interface sysfsgpio_bitbang = {
	.read = remote_bitbang_read,
	.write = remote_bitbang_write,
	.reset = remote_bitbang_reset,
	.swdio_read = remote_swdio_read,
	.swdio_drive = remote_swdio_drive,
	.blink = remote_bitbang_blink,
    .swdio_read_reg = remote_swdio_read_reg,
	.swdio_write_reg = remote_swdio_write_reg
};

static int remote_bitbang_init(void)
{
	int fd;
	//bitbang_interface = &remote_bitbang_bitbang;

	bitbang_interface = &sysfsgpio_bitbang;

	LOG_INFO("Initializing remote_bitbang driver");
	if (remote_bitbang_port == NULL)
		fd = remote_bitbang_init_unix();
	else
		fd = remote_bitbang_init_tcp();

	if (fd < 0)
		return fd;

	remote_bitbang_in = fdopen(fd, "r");
	if (remote_bitbang_in == NULL) {
		LOG_ERROR("fdopen: failed to open read stream");
		close(fd);
		return ERROR_FAIL;
	}

	remote_bitbang_out = fdopen(fd, "w");
	if (remote_bitbang_out == NULL) {
		LOG_ERROR("fdopen: failed to open write stream");
		fclose(remote_bitbang_in);
		return ERROR_FAIL;
	}

	LOG_INFO("art-swd driver initialized");
	return ERROR_OK;
}


static int art_speed(int speed)
{
	return ERROR_OK;
}

static int art_speed_div(int speed, int *khz)
{
	/* I don't think this really matters any. */
	*khz = 1;
	return ERROR_OK;
}

static int art_khz(int khz, int *jtag_speed)
{
	*jtag_speed = 0;
	return ERROR_OK;
}


#if 0
COMMAND_HANDLER(art_handle_art_port_command)
{
	if (CMD_ARGC == 1) {
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], art_port);
		return ERROR_OK;
	}
	return ERROR_COMMAND_SYNTAX_ERROR;
}

COMMAND_HANDLER(art_handle_art_host_command)
{
	if (CMD_ARGC == 1) {
		strncpy(art_host, CMD_ARGV[0], ART_HOST_MAX);
		art_host[ART_HOST_MAX-1] = '\0';
		return ERROR_OK;
	}
	return ERROR_COMMAND_SYNTAX_ERROR;
}

static const struct command_registration art_command_handlers[] = {
	{
		.name = "art_port",
		.handler = art_handle_art_port_command,
		.mode = COMMAND_CONFIG,
		.help = "Set the port to use to connect to the remote jtag.\n"
			"  if 0, use unix sockets to connect to the remote jtag.",
		.usage = "port_number",
	},
	{
		.name = "art_host",
		.handler = art_handle_art_host_command,
		.mode = COMMAND_CONFIG,
		.help = "Set the host to use to connect to the remote jtag.\n"
			"  if port is 0, this is the name of the unix socket to use.",
		.usage = "host_name",
	},
	COMMAND_REGISTRATION_DONE,
};
#endif


static const char * const sysfsgpio_transports[] = { "jtag", "swd", NULL };

struct jtag_interface sysfsgpio_interface = {
	.name = "art-swd",
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = bitbang_execute_queue,
	.transports = sysfsgpio_transports,
	.swd = &bitbang_swd,
	.commands = remote_bitbang_command_handlers,
	.init = remote_bitbang_init,
	.quit = remote_bitbang_quit,
	.speed = &art_speed,
	.speed_div = &art_speed_div,
	.khz = &art_khz,
};

