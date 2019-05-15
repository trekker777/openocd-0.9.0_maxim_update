/***************************************************************************
 *   Copyright (C) 2012 by Jeremy Brodt                                    *
 *   jeremy.brodt@maxim-ic.com                                             *
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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifndef _WIN32
#include <sys/un.h>
#include <netdb.h>
#endif
#include <jtag/interface.h>

#ifndef UNIX_PATH_LEN
#define UNIX_PATH_LEN 108
#endif

#define _DEBUG_JTAG_IO_

/* arbitrary limit on host name length: */
#define ART_HOST_MAX 255

#define ART_RAISE_ERROR(expr ...) \
	do { \
		LOG_ERROR(expr); \
		LOG_ERROR("Terminating openocd."); \
		exit(-1); \
	} while (0)


#define ART_MSG_ID		0xA5

// RESET Command
// 0xA5 0xF1 <trst> <srst>
#define CMD_RST			0xF1
#define CMD_RST_SIZE		4     // bytes

// Clock TMS Command
// 0xA5 0xF2 <bit_len_hi> <bit_len_lo> <tmsbits> ...
#define CMD_TMS_ARRAY		0xF2
#define CMD_TMS_ARRAY_HDR_SIZE	4     // bytes

// Scan Command
// 0xA5 0xF3 <type> <bit_len_hi> <bit_len_lo> <tdibits> ...
#define CMD_SCAN_ARRAY		0xF3
#define CMD_SCAN_ARRAY_HDR_SIZE	5     // bytes

// Generate TCKs Command
// 0xA5 0xF4 <num_clks_hi> <num_clks_lo> <tms>
#define CMD_TCKS		0xF4
#define CMD_TCKS_SIZE		5     // bytes

// Sleep Command
// 0xA5 0xF5 <microseconds>
#define CMD_SLEEP		0xF5
#define CMD_SLEEP_SIZE		6     // bytes

static char art_host[ART_HOST_MAX] = "openocd";
static uint16_t art_port;

FILE *art_in;
FILE *art_out;

static int art_quit(void)
{
	if (EOF == fputc('Q', art_out)) {
		LOG_ERROR("fputs: %s", strerror(errno));
		return ERROR_FAIL;
	}

	if (EOF == fflush(art_out)) {
		LOG_ERROR("fflush: %s", strerror(errno));
		return ERROR_FAIL;
	}

	/* We only need to close one of the FILE*s, because they both use the same */
	/* underlying file descriptor. */
	if (EOF == fclose(art_out)) {
		LOG_ERROR("fclose: %s", strerror(errno));
		return ERROR_FAIL;
	}

	LOG_INFO("art interface quit");
	return ERROR_OK;
}

static int art_speed(int speed)
{
	return ERROR_OK;
}

static int art_init_tcp(void)
{
	LOG_INFO("Connecting to %s:%i", art_host, art_port);
	int fd = socket(PF_INET, SOCK_STREAM, 0);
	if (fd < 0) {
		LOG_ERROR("socket: %s", strerror(errno));
		return ERROR_FAIL;
	}

	struct hostent *hent = gethostbyname(art_host);
	if (hent == NULL) {
		char *errorstr = "???";
		switch (h_errno) {
			case HOST_NOT_FOUND:
				errorstr = "host not found";
				break;
			case NO_ADDRESS:
				errorstr = "no address";
				break;
			case NO_RECOVERY:
				errorstr = "no recovery";
				break;
			case TRY_AGAIN:
				errorstr = "try again";
				break;
		}
		LOG_ERROR("gethostbyname: %s", errorstr);
		return ERROR_FAIL;
	}

	struct sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(art_port);
	addr.sin_addr = *(struct in_addr *)hent->h_addr;
	if (connect(fd, (struct sockaddr *)&addr, sizeof(struct sockaddr_in)) < 0) {
		LOG_ERROR("connect: %s", strerror(errno));
		return ERROR_FAIL;
	}

	art_in = fdopen(fd, "r");
	if (art_in == NULL) {
		LOG_ERROR("fdopen: failed to open read stream");
		return ERROR_FAIL;
	}

	art_out = fdopen(fd, "w");
	if (art_out == NULL) {
		LOG_ERROR("fdopen: failed to open write stream");
		return ERROR_FAIL;
	}

	LOG_INFO("art driver initialized");
	return ERROR_OK;
}

static int art_init_unix(void)
{
	LOG_INFO("Connecting to unix socket %s", art_host);
	int fd = socket(PF_UNIX, SOCK_STREAM, 0);
	if (fd < 0) {
		LOG_ERROR("socket: %s", strerror(errno));
		return ERROR_FAIL;
	}

	struct sockaddr_un addr;
	addr.sun_family = AF_UNIX;
	strncpy(addr.sun_path, art_host, UNIX_PATH_LEN);
	addr.sun_path[UNIX_PATH_LEN-1] = '\0';

	if (connect(fd, (struct sockaddr *)&addr, sizeof(struct sockaddr_un)) < 0) {
		LOG_ERROR("connect: %s", strerror(errno));
		return ERROR_FAIL;
	}

	art_in = fdopen(fd, "r");
	if (art_in == NULL) {
		LOG_ERROR("fdopen: failed to open read stream");
		return ERROR_FAIL;
	}

	art_out = fdopen(fd, "w");
	if (art_out == NULL) {
		LOG_ERROR("fdopen: failed to open write stream");
		return ERROR_FAIL;
	}

	LOG_INFO("art driver initialized");
	return ERROR_OK;
}

static int art_init(void)
{
	LOG_INFO("Initializing art driver");
	if (art_port == 0)
		return art_init_unix();
	return art_init_tcp();
}

static int art_khz(int khz, int *jtag_speed)
{
	*jtag_speed = 0;
	return ERROR_OK;
}

static int art_speed_div(int speed, int *khz)
{
	/* I don't think this really matters any. */
	*khz = 1;
	return ERROR_OK;
}

/* The bitbang driver leaves the TCK 0 when in idle */
static void art_end_state(tap_state_t state)
{
	if (tap_is_state_stable(state))
		tap_set_end_state(state);
	else {
		LOG_ERROR("BUG: %i is not a valid end state", state);
		exit(-1);
	}
}

static void art_state_move(int skip)
{
	uint8_t tms_scan = tap_get_tms_path(tap_get_state(), tap_get_end_state());
	int tms_count = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());
	uint8_t msg[CMD_TMS_ARRAY_HDR_SIZE + 1];

	msg[0] = ART_MSG_ID;
	msg[1] = CMD_TMS_ARRAY;
	msg[2] = (tms_count - skip);
	msg[3] = (tms_count - skip) >> 8;
	msg[4] = tms_scan >> skip;

	if (fwrite(msg, sizeof(msg), 1, art_out) != 1)
		ART_RAISE_ERROR("art_state_move: %s", strerror(errno));

	tap_set_state(tap_get_end_state());
}

/**
 * Clock a bunch of TMS (or SWDIO) transitions, to change the JTAG
 * (or SWD) state machine.
 */
static int art_execute_tms(struct jtag_command *cmd)
{
	unsigned num_bits = cmd->cmd.tms->num_bits;
	const uint8_t *bits = cmd->cmd.tms->bits;
	uint8_t msg[CMD_TMS_ARRAY_HDR_SIZE];

	DEBUG_JTAG_IO("TMS: %d bits", num_bits);

	msg[0] = ART_MSG_ID;
	msg[1] = CMD_TMS_ARRAY;
	msg[2] = num_bits;
	msg[3] = num_bits >> 8;

	if (fwrite(msg, sizeof(msg), 1, art_out) != 1)
		ART_RAISE_ERROR("art_execute_tms: %s", strerror(errno));

	if (fwrite(bits, (num_bits + 7) / 8, 1, art_out) != 1)
		ART_RAISE_ERROR("art_execute_tms: %s", strerror(errno));

	return ERROR_OK;
}

static void art_path_move(struct pathmove_command *cmd)
{
	int num_states = cmd->num_states;
	int state_count;
	int tms = 0;
	uint8_t * msg;
	int msgsize = CMD_TMS_ARRAY_HDR_SIZE + ((num_states + 7) / 8);

	msg = malloc(msgsize);
	if (msg == NULL) {
		LOG_ERROR("art_path_move: malloc failed");
		exit(-1);
	}

	// Initialize message
	memset(msg, 0, msgsize);
	msg[0] = ART_MSG_ID;
	msg[1] = CMD_TMS_ARRAY;
	msg[2] = num_states;
	msg[3] = num_states >> 8;

	state_count = 0;
	while (num_states) {
		if (tap_state_transition(tap_get_state(), false) == cmd->path[state_count])
			tms = 0;
		else if (tap_state_transition(tap_get_state(), true) == cmd->path[state_count])
			tms = 1;
		else {
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition",
				tap_state_name(tap_get_state()),
				tap_state_name(cmd->path[state_count]));
			exit(-1);
		}

		msg[CMD_TMS_ARRAY_HDR_SIZE + (state_count / 8)] |= tms << (state_count % 8);

		tap_set_state(cmd->path[state_count]);
		state_count++;
		num_states--;
	}

	if (fwrite(msg, msgsize, 1, art_out) != 1)
		ART_RAISE_ERROR("art_path_move: %s", strerror(errno));

	free(msg);

	tap_set_end_state(tap_get_state());
}

static void art_runtest(int num_cycles)
{
	uint8_t msg[CMD_TCKS_SIZE];
	tap_state_t saved_end_state = tap_get_end_state();

	/* only do a state_move when we're not already in IDLE */
	if (tap_get_state() != TAP_IDLE) {
		art_end_state(TAP_IDLE);
		art_state_move(0);
	}

	msg[0] = ART_MSG_ID;
	msg[1] = CMD_TCKS;
	msg[2] = num_cycles;
	msg[3] = num_cycles >> 8;
	msg[4] = 0;

	if (fwrite(msg, sizeof(msg), 1, art_out) != 1)
		ART_RAISE_ERROR("art_runtest: %s", strerror(errno));

	/* finish in end_state */
	art_end_state(saved_end_state);
	if (tap_get_state() != tap_get_end_state())
		art_state_move(0);
}

static void art_stableclocks(int num_cycles)
{
	uint8_t msg[CMD_TCKS_SIZE];

	msg[0] = ART_MSG_ID;
	msg[1] = CMD_TCKS;
	msg[2] = num_cycles;
	msg[3] = num_cycles >> 8;
	msg[4] = (tap_get_state() == TAP_RESET ? 1 : 0);

	if (fwrite(msg, sizeof(msg), 1, art_out) != 1)
		ART_RAISE_ERROR("art_stableclocks: %s", strerror(errno));
}

static void art_scan(bool ir_scan, enum scan_type type, uint8_t *buffer, int scan_size)
{
	tap_state_t saved_end_state = tap_get_end_state();
	uint8_t msg[CMD_SCAN_ARRAY_HDR_SIZE];
	int scan_bytes = (scan_size + 7) / 8;

	if (!((!ir_scan &&
			(tap_get_state() == TAP_DRSHIFT)) ||
			(ir_scan && (tap_get_state() == TAP_IRSHIFT)))) {
		if (ir_scan)
			art_end_state(TAP_IRSHIFT);
		else
			art_end_state(TAP_DRSHIFT);

		art_state_move(0);
		art_end_state(saved_end_state);
	}

	// Initialize message
	msg[0] = ART_MSG_ID;
	msg[1] = CMD_SCAN_ARRAY;
	msg[2] = type;
	msg[3] = scan_size;
	msg[4] = scan_size >> 8;

	if (fwrite(msg, sizeof(msg), 1, art_out) != 1)
		ART_RAISE_ERROR("art_scan: %s", strerror(errno));

	/* if we're just reading the scan, but don't care about the output
	 * default to outputting 'low', this also makes valgrind traces more readable,
	 * as it removes the dependency on an uninitialised value
	 */
	if (type == SCAN_IN)
		memset(buffer, 0, scan_bytes);

	if (fwrite(buffer, scan_bytes, 1, art_out) != 1)
		ART_RAISE_ERROR("art_scan: %s", strerror(errno));

	if (type != SCAN_OUT) {
		fflush(art_out);

#ifdef _DEBUG_JTAG_IO_
		LOG_DEBUG("art_scan: waiting for response...");
#endif

		if (fread(msg, sizeof(msg), 1, art_in) != 1)
			ART_RAISE_ERROR("art_scan: %s", strerror(errno));

		if (fread(buffer, scan_bytes, 1, art_in) != 1)
			ART_RAISE_ERROR("art_scan: %s", strerror(errno));
	}

	if (tap_get_state() != tap_get_end_state()) {
		/* we *KNOW* the above loop transitioned out of
		 * the shift state, so we skip the first state
		 * and move directly to the end state.
		 */
		art_state_move(1);
	}
}

int art_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;	/* currently processed command */
	int scan_size;
	enum scan_type type;
	uint8_t *buffer;
	int retval;
	uint8_t msg[CMD_SLEEP_SIZE];

	/* return ERROR_OK, unless a jtag_read_buffer returns a failed check
	 * that wasn't handled by a caller-provided error handler
	 */
	retval = ERROR_OK;

	while (cmd) {
		switch (cmd->type) {
//
// JTAG_RESET
//			
			case JTAG_RESET:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("reset trst: %i srst %i",
				cmd->cmd.reset->trst,
				cmd->cmd.reset->srst);
#endif
				if ((cmd->cmd.reset->trst == 1) ||
						(cmd->cmd.reset->srst && (jtag_get_reset_config() & RESET_SRST_PULLS_TRST)))
					tap_set_state(TAP_RESET);

				msg[0] = ART_MSG_ID;
				msg[1] = CMD_RST;
				msg[2] = cmd->cmd.reset->trst;
				msg[3] = cmd->cmd.reset->srst;

				if (fwrite(msg, CMD_RST_SIZE, 1, art_out) != 1)
					ART_RAISE_ERROR("art_execute_queue: %s", strerror(errno));
				break;
//
// JTAG_RUNTEST
//			
			case JTAG_RUNTEST:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("runtest %i cycles, end in %s",
						cmd->cmd.runtest->num_cycles,
						tap_state_name(cmd->cmd.runtest->end_state));
#endif
				art_end_state(cmd->cmd.runtest->end_state);
				art_runtest(cmd->cmd.runtest->num_cycles);
				break;

//
// JTAG_STABLECLOCKS
//			
			case JTAG_STABLECLOCKS:
				/* this is only allowed while in a stable state.  A check for a stable
				 * state was done in jtag_add_clocks()
				 */
				art_stableclocks(cmd->cmd.stableclocks->num_cycles);
				break;

//
// JTAG_TLR_RESET
//			
			case JTAG_TLR_RESET:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("statemove end in %s",
						tap_state_name(cmd->cmd.statemove->end_state));
#endif
				art_end_state(cmd->cmd.statemove->end_state);
				art_state_move(0);
				break;
//
// JTAG_PATHMOVE
//			
			case JTAG_PATHMOVE:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("pathmove: %i states, end in %s",
						cmd->cmd.pathmove->num_states,
						tap_state_name(cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]));
#endif
				art_path_move(cmd->cmd.pathmove);
				break;
//
// JTAG_SCAN
//			
			case JTAG_SCAN:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("%s scan end in %s",
						(cmd->cmd.scan->ir_scan) ? "IR" : "DR",
					tap_state_name(cmd->cmd.scan->end_state));
#endif
				art_end_state(cmd->cmd.scan->end_state);
				scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);
				type = jtag_scan_type(cmd->cmd.scan);
				art_scan(cmd->cmd.scan->ir_scan, type, buffer, scan_size);
				if (jtag_read_buffer(buffer, cmd->cmd.scan) != ERROR_OK)
					retval = ERROR_JTAG_QUEUE_FAILED;
				if (buffer)
					free(buffer);
				break;
//
// JTAG_SLEEP
//
			case JTAG_SLEEP:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("sleep %" PRIi32, cmd->cmd.sleep->us);
#endif

				msg[0] = ART_MSG_ID;
				msg[1] = CMD_SLEEP;
				msg[2] = cmd->cmd.sleep->us;
				msg[3] = cmd->cmd.sleep->us >> 8;
				msg[4] = cmd->cmd.sleep->us >> 16;
				msg[5] = cmd->cmd.sleep->us >> 24;

				if (fwrite(msg, CMD_SLEEP_SIZE, 1, art_out) != 1)
					ART_RAISE_ERROR("art_execute_queue: %s", strerror(errno));
				break;
			case JTAG_TMS:
				retval = art_execute_tms(cmd);
				break;
			default:
				LOG_ERROR("BUG: unknown JTAG command type encountered");
				exit(-1);
		}
		cmd = cmd->next;
	}

	return retval;
}

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

struct jtag_interface art_interface = {
	.name = "art",

	.supported = DEBUG_CAP_TMS_SEQ,
	.transports = jtag_only,
	
	.execute_queue = &art_execute_queue,

	.speed = &art_speed,

	.commands = art_command_handlers,

	.init = &art_init,
	.quit = &art_quit,
	.khz = &art_khz,
	.speed_div = &art_speed_div,
};
