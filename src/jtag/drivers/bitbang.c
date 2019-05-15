/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
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
 * by Paul Fertser and modifications by Jean-Christian de Rivaz. */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "bitbang.h"
#include <jtag/interface.h>
#include <jtag/commands.h>

/* YUK! - but this is currently a global.... */
extern struct jtag_interface *jtag_interface;

/**
 * Function bitbang_stableclocks
 * issues a number of clock cycles while staying in a stable state.
 * Because the TMS value required to stay in the RESET state is a 1, whereas
 * the TMS value required to stay in any of the other stable states is a 0,
 * this function checks the current stable state to decide on the value of TMS
 * to use.
 */
static void bitbang_stableclocks(int num_cycles);

struct bitbang_interface *bitbang_interface;

/* DANGER!!!! clock absolutely *MUST* be 0 in idle or reset won't work!
 *
 * Set this to 1 and str912 reset halt will fail.
 *
 * If someone can submit a patch with an explanation it will be greatly
 * appreciated, but as far as I can tell (ØH) DCLK is generated upon
 * clk = 0 in TAP_IDLE. Good luck deducing that from the ARM documentation!
 * The ARM documentation uses the term "DCLK is asserted while in the TAP_IDLE
 * state". With hardware there is no such thing as *while* in a state. There
 * are only edges. So clk => 0 is in fact a very subtle state transition that
 * happens *while* in the TAP_IDLE state. "#&¤"#¤&"#&"#&
 *
 * For "reset halt" the last thing that happens before srst is asserted
 * is that the breakpoint is set up. If DCLK is not wiggled one last
 * time before the reset, then the breakpoint is not set up and
 * "reset halt" will fail to halt.
 *
 */
#define CLOCK_IDLE() 0

/* The bitbang driver leaves the TCK 0 when in idle */
static void bitbang_end_state(tap_state_t state)
{
	if (tap_is_state_stable(state))
		tap_set_end_state(state);
	else {
		LOG_ERROR("BUG: %i is not a valid end state", state);
		exit(-1);
	}
}

//
// move the jtag state with tms transition
//
static void bitbang_state_move(int skip)
{
	int i = 0, tms = 0;
	uint8_t tms_scan = tap_get_tms_path(tap_get_state(), tap_get_end_state());
	int tms_count = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());

	for (i = skip; i < tms_count; i++) {
		tms = (tms_scan >> i) & 1;
		bitbang_interface->write(0, tms, 0);
		bitbang_interface->write(1, tms, 0);
	}
	bitbang_interface->write(CLOCK_IDLE(), tms, 0);

	tap_set_state(tap_get_end_state());
}

/**
 * Clock a bunch of TMS (or SWDIO) transitions, to change the JTAG
 * (or SWD) state machine.
 */
static int bitbang_execute_tms(struct jtag_command *cmd)
{
	unsigned num_bits = cmd->cmd.tms->num_bits;
	const uint8_t *bits = cmd->cmd.tms->bits;

	DEBUG_JTAG_IO("TMS: %d bits", num_bits);

	int tms = 0;
	for (unsigned i = 0; i < num_bits; i++) {
		tms = ((bits[i/8] >> (i % 8)) & 1);
		bitbang_interface->write(0, tms, 0);
		bitbang_interface->write(1, tms, 0);
	}
	bitbang_interface->write(CLOCK_IDLE(), tms, 0);

	return ERROR_OK;
}

static void bitbang_path_move(struct pathmove_command *cmd)
{
	int num_states = cmd->num_states;
	int state_count;
	int tms = 0;

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

		bitbang_interface->write(0, tms, 0);
		bitbang_interface->write(1, tms, 0);

		tap_set_state(cmd->path[state_count]);
		state_count++;
		num_states--;
	}

	bitbang_interface->write(CLOCK_IDLE(), tms, 0);

	tap_set_end_state(tap_get_state());
}

static void bitbang_runtest(int num_cycles)
{
	int i;

	tap_state_t saved_end_state = tap_get_end_state();

	/* only do a state_move when we're not already in IDLE */
	if (tap_get_state() != TAP_IDLE) {
		bitbang_end_state(TAP_IDLE);
		bitbang_state_move(0);
	}

	/* execute num_cycles */
	for (i = 0; i < num_cycles; i++) {
		bitbang_interface->write(0, 0, 0);
		bitbang_interface->write(1, 0, 0);
	}
	bitbang_interface->write(CLOCK_IDLE(), 0, 0);

	/* finish in end_state */
	bitbang_end_state(saved_end_state);
	if (tap_get_state() != tap_get_end_state())
		bitbang_state_move(0);
}

static void bitbang_stableclocks(int num_cycles)
{
	int tms = (tap_get_state() == TAP_RESET ? 1 : 0);
	int i;

	/* send num_cycles clocks onto the cable */
	for (i = 0; i < num_cycles; i++) {
		bitbang_interface->write(1, tms, 0);
		bitbang_interface->write(0, tms, 0);
	}
}

static void bitbang_scan(bool ir_scan, enum scan_type type, uint8_t *buffer, int scan_size)
{
	tap_state_t saved_end_state = tap_get_end_state();
	int bit_cnt;

	if (!((!ir_scan &&
			(tap_get_state() == TAP_DRSHIFT)) ||
			(ir_scan && (tap_get_state() == TAP_IRSHIFT)))) {
		if (ir_scan)
			bitbang_end_state(TAP_IRSHIFT);
		else
			bitbang_end_state(TAP_DRSHIFT);

		bitbang_state_move(0);
		bitbang_end_state(saved_end_state);
	}

	for (bit_cnt = 0; bit_cnt < scan_size; bit_cnt++) {
		int val = 0;
		int tms = (bit_cnt == scan_size-1) ? 1 : 0;
		int tdi;
		int bytec = bit_cnt/8;
		int bcval = 1 << (bit_cnt % 8);

		/* if we're just reading the scan, but don't care about the output
		 * default to outputting 'low', this also makes valgrind traces more readable,
		 * as it removes the dependency on an uninitialised value
		 */
		tdi = 0;
		if ((type != SCAN_IN) && (buffer[bytec] & bcval))
			tdi = 1;

		bitbang_interface->write(0, tms, tdi);

		if (type != SCAN_OUT)
			val = bitbang_interface->read();

		bitbang_interface->write(1, tms, tdi);

		if (type != SCAN_OUT) {
			if (val)
				buffer[bytec] |= bcval;
			else
				buffer[bytec] &= ~bcval;
		}
	}

	if (tap_get_state() != tap_get_end_state()) {
		/* we *KNOW* the above loop transitioned out of
		 * the shift state, so we skip the first state
		 * and move directly to the end state.
		 */
		bitbang_state_move(1);
	}
}

int bitbang_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;	/* currently processed command */
	int scan_size;
	enum scan_type type;
	uint8_t *buffer;
	int retval;

	if (!bitbang_interface) {
		LOG_ERROR("BUG: Bitbang interface called, but not yet initialized");
		exit(-1);
	}

	/* return ERROR_OK, unless a jtag_read_buffer returns a failed check
	 * that wasn't handled by a caller-provided error handler
	 */
	retval = ERROR_OK;

	if (bitbang_interface->blink)
		bitbang_interface->blink(1);

	while (cmd) {
		switch (cmd->type) {
			case JTAG_RESET:
#ifdef _DEBUG_JTAG_IO_
				//LOG_DEBUG("reset trst: %i srst %i",
				cmd->cmd.reset->trst,
				cmd->cmd.reset->srst);
#endif
				if ((cmd->cmd.reset->trst == 1) ||
						(cmd->cmd.reset->srst && (jtag_get_reset_config() & RESET_SRST_PULLS_TRST)))
					tap_set_state(TAP_RESET);
				bitbang_interface->reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
				break;
			case JTAG_RUNTEST:
#ifdef _DEBUG_JTAG_IO_
				//LOG_DEBUG("runtest %i cycles, end in %s",
				//		cmd->cmd.runtest->num_cycles,
				//		tap_state_name(cmd->cmd.runtest->end_state));
#endif
				bitbang_end_state(cmd->cmd.runtest->end_state);
				bitbang_runtest(cmd->cmd.runtest->num_cycles);
				break;

			case JTAG_STABLECLOCKS:
				/* this is only allowed while in a stable state.  A check for a stable
				 * state was done in jtag_add_clocks()
				 */
				bitbang_stableclocks(cmd->cmd.stableclocks->num_cycles);
				break;

			case JTAG_TLR_RESET:
#ifdef _DEBUG_JTAG_IO_
				//LOG_DEBUG("statemove end in %s",
				//		tap_state_name(cmd->cmd.statemove->end_state));
#endif
				bitbang_end_state(cmd->cmd.statemove->end_state);
				bitbang_state_move(0);
				break;
			case JTAG_PATHMOVE:
#ifdef _DEBUG_JTAG_IO_
				//LOG_DEBUG("pathmove: %i states, end in %s",
				//		cmd->cmd.pathmove->num_states,
				//		tap_state_name(cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]));
#endif
				bitbang_path_move(cmd->cmd.pathmove);
				break;
			case JTAG_SCAN:
#ifdef _DEBUG_JTAG_IO_
				//LOG_DEBUG("%s scan end in %s",
				//		(cmd->cmd.scan->ir_scan) ? "IR" : "DR",
				//	tap_state_name(cmd->cmd.scan->end_state));
#endif
				bitbang_end_state(cmd->cmd.scan->end_state);
				scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);
				type = jtag_scan_type(cmd->cmd.scan);
				bitbang_scan(cmd->cmd.scan->ir_scan, type, buffer, scan_size);
				if (jtag_read_buffer(buffer, cmd->cmd.scan) != ERROR_OK)
					retval = ERROR_JTAG_QUEUE_FAILED;
				if (buffer)
					free(buffer);
				break;
			case JTAG_SLEEP:
#ifdef _DEBUG_JTAG_IO_
				//LOG_DEBUG("sleep %" PRIi32, cmd->cmd.sleep->us);
#endif
				jtag_sleep(cmd->cmd.sleep->us);
				break;
			case JTAG_TMS:
				retval = bitbang_execute_tms(cmd);
				break;
			default:
				LOG_ERROR("BUG: unknown JTAG command type encountered");
				exit(-1);
		}
		cmd = cmd->next;
	}
	if (bitbang_interface->blink)
		bitbang_interface->blink(0);

	return retval;
}


bool swd_mode;
static int queued_retval;
void transaction_queue_init(void);

static int bitbang_swd_init(void)
{
	//LOG_DEBUG("bitbang_swd_init");
	LOG_INFO("bitbang_swd_init");
	transaction_queue_init();
	swd_mode = true;
	return ERROR_OK;
}

// 
// rnw: true for read
// buf: buffer to transfer
// offset: offset in the buffer
// bit_cnt: number of bits to transfer
//
static void bitbang_exchange(bool rnw, uint8_t buf[], unsigned int offset, unsigned int bit_cnt)
{
	//LOG_DEBUG("bitbang_exchange");
	//LOG_INFO("bitbang_exchange");

	int tdi;

	for (unsigned int i = offset; i < bit_cnt + offset; i++) {
		int bytec = i/8;
		int bcval = 1 << (i % 8);
		tdi = !rnw && (buf[bytec] & bcval);

		bitbang_interface->write(0, 0, tdi); // tck, tms, tdi

		if (rnw && buf) {
			if (bitbang_interface->swdio_read())
				buf[bytec] |= bcval;
			else
				buf[bytec] &= ~bcval;
		}

		bitbang_interface->write(1, 0, tdi); // tck, tms, tdi
	}
}

int bitbang_swd_switch_seq(struct adiv5_dap *dap, enum swd_special_seq seq)
{
	//LOG_DEBUG("bitbang_swd_switch_seq");
	//LOG_INFO("bitbang_swd_switch_seq");

	switch (seq) {
	case LINE_RESET:
		//LOG_DEBUG("SWD line reset");
		bitbang_exchange(false, (uint8_t *)swd_seq_line_reset, 0, swd_seq_line_reset_len);
		break;
	case JTAG_TO_SWD:
		//LOG_DEBUG("JTAG-to-SWD");
		bitbang_exchange(false, (uint8_t *)swd_seq_jtag_to_swd, 0, swd_seq_jtag_to_swd_len);
		break;
	case SWD_TO_JTAG:
		//LOG_DEBUG("SWD-to-JTAG");
		bitbang_exchange(false, (uint8_t *)swd_seq_swd_to_jtag, 0, swd_seq_swd_to_jtag_len);
		break;
	default:
		LOG_ERROR("Sequence %d not supported", seq);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

void bitbang_switch_to_swd(void)
{
	//LOG_DEBUG("bitbang_switch_to_swd");
	bitbang_exchange(false, (uint8_t *)swd_seq_jtag_to_swd, 0, swd_seq_jtag_to_swd_len);
}
/*
static void swd_clear_sticky_errors(struct adiv5_dap *dap)
{
	const struct swd_driver *swd = jtag_interface->swd;
	assert(swd);

	swd->write_reg(dap, swd_cmd(false,  false, DP_ABORT),
		STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR);
}*/
//
#if 0
static void bitbang_swd_read_reg(struct adiv5_dap *dap, uint8_t cmd, uint32_t *value)
{
	//LOG_INFO("bitbang_swd_read_reg");
	assert(cmd & SWD_CMD_RnW);

	if (queued_retval != ERROR_OK) {
		LOG_INFO("Skip bitbang_swd_read_reg because queued_retval=%d", queued_retval);
		return;
	}

	for (;;) {
		uint8_t trn_ack_data_parity_trn[DIV_ROUND_UP(4 + 3 + 32 + 1 + 4, 8)];

		cmd |= SWD_CMD_START | (1 << 7);
		bitbang_exchange(false, &cmd, 0, 8);

		bitbang_interface->swdio_drive(false);
		bitbang_exchange(true, trn_ack_data_parity_trn, 0, 1 + 3 + 32 + 1 + 1);
		bitbang_interface->swdio_drive(true);

		int ack = buf_get_u32(trn_ack_data_parity_trn, 1, 3);
		uint32_t data = buf_get_u32(trn_ack_data_parity_trn, 1 + 3, 32);
		int parity = buf_get_u32(trn_ack_data_parity_trn, 1 + 3 + 32, 1);

		/*LOG_INFO("%s %s %s reg %X = %08"PRIx32,
			  ack == SWD_ACK_OK ? "OK" : ack == SWD_ACK_WAIT ? "WAIT" : ack == SWD_ACK_FAULT ? "FAULT" : "JUNK",
			  cmd & SWD_CMD_APnDP ? "AP" : "DP",
			  cmd & SWD_CMD_RnW ? "read" : "write",
			  (cmd & SWD_CMD_A32) >> 1,
			  data);*/

		switch (ack) {
		 case SWD_ACK_OK:
			if (parity != parity_u32(data)) {
				LOG_INFO("Wrong parity detected");
				queued_retval = ERROR_FAIL;
				return;
			}
			if (value)
				*value = data;
			if (cmd & SWD_CMD_APnDP)
				bitbang_exchange(true, NULL, 0, dap->memaccess_tck);
			return;
		 case SWD_ACK_WAIT:
			//LOG_INFO("SWD_ACK_WAIT");
			swd_clear_sticky_errors(dap);
			break;
		 case SWD_ACK_FAULT:
			//LOG_INFO("SWD_ACK_FAULT");
			queued_retval = ack;
			return;
		 default:
			//LOG_INFO("No valid acknowledge: ack=%d", ack);
			queued_retval = ack;
			return;
		}
	}
}
#else

#if 0
static void art_swd_read_reg(struct adiv5_dap *dap, uint8_t cmd, uint32_t *value)
{
  uint8_t ack;
  uint32_t data;
  uint8_t parity;
	//LOG_INFO("----read");
	assert(cmd & SWD_CMD_RnW);

	if (queued_retval != ERROR_OK) {
		LOG_INFO("Skip bitbang_swd_read_reg because queued_retval=%d", queued_retval);
		return;
	}

	for (;;) {
		//uint8_t trn_ack_data_parity_trn[DIV_ROUND_UP(4 + 3 + 32 + 1 + 4, 8)];

		cmd |= SWD_CMD_START | (1 << 7);

		#if 0
			bitbang_exchange(false, &cmd, 0, 8);

			bitbang_interface->swdio_drive(false);
			bitbang_exchange(true, trn_ack_data_parity_trn, 0, 1 + 3 + 32 + 1 + 1);
			bitbang_interface->swdio_drive(true);

			ack = buf_get_u32(trn_ack_data_parity_trn, 1, 3);
			data = buf_get_u32(trn_ack_data_parity_trn, 1 + 3, 32);
			parity = buf_get_u32(trn_ack_data_parity_trn, 1 + 3 + 32, 1);
		#else
			bitbang_interface->swdio_read_reg(cmd, &ack, &data, &parity);
		#endif

		/*LOG_INFO("%s %s %s reg %X = %08"PRIx32,
			  ack == SWD_ACK_OK ? "OK" : ack == SWD_ACK_WAIT ? "WAIT" : ack == SWD_ACK_FAULT ? "FAULT" : "JUNK",
			  cmd & SWD_CMD_APnDP ? "AP" : "DP",
			  cmd & SWD_CMD_RnW ? "read" : "write",
			  (cmd & SWD_CMD_A32) >> 1,
			  data);*/

		switch (ack) {
		 case SWD_ACK_OK:
			if (parity != parity_u32(data)) {
				LOG_INFO("Wrong parity detected");
				queued_retval = ERROR_FAIL;
				return;
			}
			if (value)
				*value = data;
			if (cmd & SWD_CMD_APnDP) { 
				//LOG_INFO("dap->memaccess_tck=%d", dap->memaccess_tck);
				//bitbang_exchange(true, NULL, 0, dap->memaccess_tck);
		    }
			return;
		 case SWD_ACK_WAIT:
			//LOG_INFO("SWD_ACK_WAIT");
			//swd_clear_sticky_errors(dap);
			break;
		 case SWD_ACK_FAULT:
			//LOG_INFO("SWD_ACK_FAULT");
			queued_retval = ack;
			return;
		 default:
			//LOG_INFO("No valid acknowledge: ack=%d", ack);
			queued_retval = ack;
			return;
		}
	}
}
#endif


#endif

#if 0
static void bitbang_swd_write_reg(struct adiv5_dap *dap, uint8_t cmd, uint32_t value)
{
	LOG_DEBUG("bitbang_swd_write_reg");
	assert(!(cmd & SWD_CMD_RnW));

	if (queued_retval != ERROR_OK) {
		LOG_DEBUG("Skip bitbang_swd_write_reg because queued_retval=%d", queued_retval);
		return;
	}

	for (;;) {
		uint8_t trn_ack_data_parity_trn[DIV_ROUND_UP(4 + 3 + 32 + 1 + 4, 8)];
		buf_set_u32(trn_ack_data_parity_trn, 1 + 3 + 1, 32, value);
		buf_set_u32(trn_ack_data_parity_trn, 1 + 3 + 1 + 32, 1, parity_u32(value));

		// prepend the start bit to the command byte
		cmd |= SWD_CMD_START | (1 << 7);
		// send the command byte
		bitbang_exchange(false, &cmd, 0, 8);

		// turn the sdio bus around to read
		bitbang_interface->swdio_drive(false);
		// read 5 bits with 0 buffer offset
		bitbang_exchange(true, trn_ack_data_parity_trn, 0, 1 + 3 + 1);
		// turn bus around to write
		bitbang_interface->swdio_drive(true);
		// write 33 bits with 5 bit offset into buffer
		bitbang_exchange(false, trn_ack_data_parity_trn, 1 + 3 + 1, 32 + 1);

		// interpret the results
		int ack = buf_get_u32(trn_ack_data_parity_trn, 1, 3);
		LOG_DEBUG("%s %s %s reg %X = %08"PRIx32,
			  ack == SWD_ACK_OK ? "OK" : ack == SWD_ACK_WAIT ? "WAIT" : ack == SWD_ACK_FAULT ? "FAULT" : "JUNK",
			  cmd & SWD_CMD_APnDP ? "AP" : "DP",
			  cmd & SWD_CMD_RnW ? "read" : "write",
			  (cmd & SWD_CMD_A32) >> 1,
			  buf_get_u32(trn_ack_data_parity_trn, 1 + 3 + 1, 32));

		switch (ack) {
		 case SWD_ACK_OK:
			if (cmd & SWD_CMD_APnDP)
				bitbang_exchange(true, NULL, 0, dap->memaccess_tck);
			return;
		 case SWD_ACK_WAIT:
			LOG_DEBUG("SWD_ACK_WAIT");
			swd_clear_sticky_errors(dap);
			break;
		 case SWD_ACK_FAULT:
			LOG_DEBUG("SWD_ACK_FAULT");
			queued_retval = ack;
			return;
		 default:
			LOG_DEBUG("No valid acknowledge: ack=%d", ack);
			queued_retval = ack;
			return;
		}
	}
}
#else

//bitbang_interface
#if 0
static void art_swd_write_reg(struct adiv5_dap *dap, uint8_t cmd, uint32_t data)
{
	uint8_t ack;
	LOG_INFO("----write");
	assert(!(cmd & SWD_CMD_RnW));

	if (queued_retval != ERROR_OK) {
		//LOG_DEBUG("Skip bitbang_swd_write_reg because queued_retval=%d", queued_retval);
		return;
	}

	for (;;) {
		uint8_t trn_ack_data_parity_trn[DIV_ROUND_UP(4 + 3 + 32 + 1 + 4, 8)];
		buf_set_u32(trn_ack_data_parity_trn, 1 + 3 + 1, 32, data);
		buf_set_u32(trn_ack_data_parity_trn, 1 + 3 + 1 + 32, 1, parity_u32(data));

		// or the start bit and park bit to the command byte
		cmd |= SWD_CMD_START | (1 << 7);

		#if 0
			// send the command byte
			bitbang_exchange(false, &cmd, 0, 8);
			// turn the sdio bus around to read
			bitbang_interface->swdio_drive(false);
			// read 5 bits with 0 buffer offset
			bitbang_exchange(true, trn_ack_data_parity_trn, 0, 1 + 3 + 1);
			// turn bus around to write
			bitbang_interface->swdio_drive(true);
			// write 33 bits with 5 bit offset into buffer
			bitbang_exchange(false, trn_ack_data_parity_trn, 1 + 3 + 1, 32 + 1);

			ack = buf_get_u32(trn_ack_data_parity_trn, 1, 3);
		#else
			bitbang_interface->swdio_write_reg(cmd, &ack, data);
		#endif

		// interpret the results
		//int ack = buf_get_u32(trn_ack_data_parity_trn, 1, 3);
/*		LOG_DEBUG("%s %s %s reg %X = %08"PRIx32,
			  ack == SWD_ACK_OK ? "OK" : ack == SWD_ACK_WAIT ? "WAIT" : ack == SWD_ACK_FAULT ? "FAULT" : "JUNK",
			  cmd & SWD_CMD_APnDP ? "AP" : "DP",
			  cmd & SWD_CMD_RnW ? "read" (: "write",
			  (cmd & SWD_CMD_A32) >> 1,
			  buf_get_u32(trn_ack_data_parity_trn, 1 + 3 + 1, 32));*/

		switch (ack) {
		 case SWD_ACK_OK:
			//if (cmd & SWD_CMD_APnDP)
			//	bitbang_exchange(true, NULL, 0, dap->memaccess_tck);
			return;
		 case SWD_ACK_WAIT:
			//LOG_DEBUG("SWD_ACK_WAIT");
			//swd_clear_sticky_errors(dap);
			break;
		 case SWD_ACK_FAULT:
			//LOG_DEBUG("SWD_ACK_FAULT");
			queued_retval = ack;
			return;
		 default:
			//LOG_DEBUG("No valid acknowledge: ack=%d", ack);
			queued_retval = ack;
			return;
		}
	}
}
#endif

#endif




//
//
// transaction_queue
//
//
static struct transaction_queue_entry {
	uint8_t read;
	uint32_t *dst;
	//uint8_t trn_ack_data_parity_trn[DIV_ROUND_UP(4 + 3 + 32 + 1 + 4, 8)];
} *transaction_queue;
static int transaction_queue_length;
static int transaction_queue_allocated;

void transaction_queue_init(void) {
	transaction_queue_allocated = 10000;
	transaction_queue = malloc(transaction_queue_allocated * sizeof(*transaction_queue));
	transaction_queue_length = 0;
}

void transaction_queue_quit(void) {
	free(transaction_queue);	
}

void swdio_flush(void);
void response_swdio_write_reg(uint8_t *ack);
void response_swdio_read_reg(uint8_t *ack, uint32_t *data, uint8_t *parity);

static int bitbang_swd_run_queue(struct adiv5_dap *dap)
{
	int i;
	uint8_t ack;
	uint32_t data;
	uint8_t parity;
	//LOG_INFO("++++++++run_queue, %d", transaction_queue_length);
	/* A transaction must be followed by another transaction or at least 8 idle cycles to
	 * ensure that data is clocked through the AP. */
	bitbang_exchange(true, NULL, 0, 8);

	queued_retval = ERROR_OK;
	swdio_flush();

	for (i = 0; i < transaction_queue_length; i++) {
		//if ((i % 10) == 0) LOG_INFO("run %d", i);
		if (transaction_queue[i].read) {
			response_swdio_read_reg(&ack, &data, &parity);
			if (transaction_queue[i].dst != NULL)
				*transaction_queue[i].dst = data;
			if (ack != SWD_ACK_OK) {
				//LOG_INFO("!!!ack != SWD_ACK_OK1");
				queued_retval = ack == SWD_ACK_WAIT ? ERROR_WAIT : ERROR_FAIL;
				goto skip;
			}
		} else {
			response_swdio_write_reg(&ack);
			if (ack != SWD_ACK_OK) {
				//LOG_INFO("!!!ack != SWD_ACK_OK2");
				queued_retval = ack == SWD_ACK_WAIT ? ERROR_WAIT : ERROR_FAIL;
				goto skip;
			}
		}
	}

skip:
	//LOG_INFO("ending");
	transaction_queue_length = 0;

	int retval = queued_retval;
	queued_retval = ERROR_OK;
	//LOG_DEBUG("SWD queue return value: %02x", retval);
	return retval;
}

void queue_swdio_write_reg(uint8_t cmd, uint32_t data);
void queue_swdio_read_reg(uint8_t cmd);

static void art_queue_swd_read_reg(struct adiv5_dap *dap, uint8_t cmd, uint32_t *value) {
	transaction_queue[transaction_queue_length].read = 1;
	transaction_queue[transaction_queue_length].dst = value;
	transaction_queue_length++;
	queue_swdio_read_reg(cmd);
}

static void art_queue_swd_write_reg(struct adiv5_dap *dap, uint8_t cmd, uint32_t data) {
	transaction_queue[transaction_queue_length].read = 0;
	transaction_queue[transaction_queue_length].dst = NULL;
	transaction_queue_length++;
	queue_swdio_write_reg(cmd, data);
}

const struct swd_driver bitbang_swd = {
	.init = bitbang_swd_init,
	.switch_seq = bitbang_swd_switch_seq,
	.read_reg = art_queue_swd_read_reg, //bitbang_swd_read_reg, //art_swd_read_reg
	.write_reg = art_queue_swd_write_reg, //bitbang_swd_write_reg, //art_swd_write_reg
	.run = bitbang_swd_run_queue,
};
