/***************************************************************************
 *   Copyright (C) 2015 by Maxim Integrated                                *
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

/***************************************************************************
* Maxim XIP Flash
***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <target/algorithm.h>
#include <target/armv7m.h>

#define TIMEOUT_CNT         0x2FFF
#define WRITE_BLOCK         0x100

#define SPIM_CFG            0x00
#define SPIM_SS_POL         0x04
#define SPIM_GEN_CTRL       0x08
#define SPIM_FIFO_CTRL      0x0C
#define SPIM_SPCL_CTRL      0x10
#define SPIM_INTFL          0x14
#define SPIM_INTEN          0x18

#define SPIX_CFG            0x00
#define SPIX_FETCH          0x04
#define SPIX_MODE           0x08
#define SPIX_FB             0x10
#define SPIX_CRYPTO         0x20

#define SPIM_GEN_CTRL_INIT  0x07
#define SPIM_CFG_INIT       0x4400
#define SPIM_INTFL_RES_DONE 0x08
#define SPIM_TXFIFO_FULL    0x1F00
#define SPIM_INTFL_TXAE     0x10

#define SPIX_CFG_INIT       0x4404
#define SPIX_MODE_INIT      0x000
#define SPIX_CRYPTO_INIT    0x03

#define SPIM_IO             0x1100110
#define SPIM_CLK            0x01
#define SPIX_IO             0x10110
#define SPIX_CLK            0x01

#define CMD_WE              0x06
#define CMD_SR              0x05
#define CMD_SR_BUSY         0x01
#define CMD_EQPI0           0xFF
#define CMD_EQPI1           0xF5

static int maxim_xip_mass_erase(struct flash_bank *bank);
static int maxim_xip_setup_spim(struct flash_bank *bank);
static int maxim_xip_setup_spix(struct flash_bank *bank);
static int maxim_xip_exit_qpi(struct flash_bank *bank);
static int maxim_xip_probe(struct flash_bank *bank);

struct maxim_xip_flash_bank
{
    int probed;
    unsigned int flash_size;
    unsigned int flash_block;
    unsigned int flash_dummy;
    unsigned int flash_read;
    unsigned int flash_write;
    unsigned int flash_erase;
    unsigned int flash_mass_erase;
    unsigned int spim_base;
    unsigned int spim_txfifo;
    unsigned int spim_rxfifo;
    unsigned int spim_io;
    unsigned int spim_clk;
    unsigned int spix_base;
    unsigned int spix_io;
    unsigned int spix_clk;
    unsigned int crypto_base;
    unsigned int integrity;
};

static uint8_t maxim_xip_write_code[] = {
  0x0b, 0x68, 0x03, 0x60, 0x4b, 0x68, 0x43, 0x60, 0x8b, 0x68, 0x83, 0x60, 0xcb, 0x68, 0xc3, 0x60, 0x70, 0x47, 0xf8, 0xb5, 
  0x06, 0x9c, 0x03, 0xf1, 0x00, 0x63, 0x16, 0x46, 0x86, 0x46, 0x0f, 0x46, 0x85, 0x1c, 0x03, 0xf1, 0x10, 0x02, 0x15, 0xf8, 
  0x02, 0x1c, 0x04, 0x35, 0x59, 0x40, 0x05, 0xf8, 0x06, 0x1c, 0x15, 0xf8, 0x05, 0x1c, 0x81, 0xea, 0x13, 0x21, 0x05, 0xf8, 
  0x05, 0x1c, 0x15, 0xf8, 0x04, 0x1c, 0x81, 0xea, 0x13, 0x41, 0x05, 0xf8, 0x04, 0x1c, 0x15, 0xf8, 0x03, 0x1c, 0x81, 0xea, 
  0x13, 0x61, 0x04, 0x33, 0x93, 0x42, 0x05, 0xf8, 0x03, 0x1c, 0xe4, 0xd1, 0x23, 0x68, 0x04, 0xf1, 0x20, 0x05, 0x43, 0xf0, 
  0xf8, 0x53, 0x23, 0x60, 0x10, 0x23, 0x63, 0x60, 0x63, 0x68, 0x28, 0x46, 0x23, 0xf4, 0x00, 0x63, 0x23, 0xf0, 0x0c, 0x03, 
  0x43, 0xf0, 0x0c, 0x03, 0x63, 0x60, 0x71, 0x46, 0xff, 0xf7, 0xbe, 0xff, 0x23, 0x68, 0x1a, 0x01, 0xfc, 0xd5, 0x38, 0x46, 
  0x04, 0xf1, 0x30, 0x01, 0xff, 0xf7, 0xb6, 0xff, 0x23, 0x68, 0x28, 0x46, 0x43, 0xf0, 0xf8, 0x53, 0x23, 0x60, 0x10, 0x23, 
  0x63, 0x60, 0x63, 0x68, 0x39, 0x46, 0x43, 0xf4, 0x00, 0x63, 0x43, 0xf0, 0x0c, 0x03, 0x63, 0x60, 0xff, 0xf7, 0xa6, 0xff, 
  0x23, 0x68, 0x1b, 0x01, 0xfc, 0xd5, 0xe3, 0x6b, 0x33, 0x70, 0xc3, 0xf3, 0x07, 0x23, 0x73, 0x70, 0xf8, 0xbd, 0x0c, 0x46, 
  0x09, 0x68, 0x05, 0x46, 0x92, 0x46, 0x08, 0x30, 0x05, 0x93, 0x13, 0x46, 0xe2, 0x68, 0x04, 0x91, 0x08, 0x90, 0x04, 0xf1, 
  0x14, 0x00, 0x0a, 0x92, 0x0b, 0x90, 0x04, 0xf1, 0x34, 0x02, 0x04, 0x98, 0x09, 0x92, 0x00, 0x22, 0xd4, 0xf8, 0x04, 0xb0, 
  0xa7, 0x68, 0x82, 0x60, 0x07, 0x22, 0x82, 0x60, 0x62, 0x68, 0x04, 0xf1, 0x24, 0x01, 0x06, 0x91, 0x0a, 0xb3, 0x01, 0x22, 
  0xcb, 0xf8, 0x00, 0x20, 0xdb, 0xf8, 0x00, 0x20, 0x42, 0xf4, 0x80, 0x42, 0xcb, 0xf8, 0x00, 0x20, 0xdb, 0xf8, 0x00, 0x20, 
  0x42, 0xf0, 0x10, 0x02, 0xcb, 0xf8, 0x00, 0x20, 0xdb, 0xf8, 0x00, 0x20, 0x42, 0xf0, 0x20, 0x02, 0xcb, 0xf8, 0x00, 0x20, 
  0x22, 0x69, 0x52, 0xb1, 0x2a, 0xf0, 0x7f, 0x03, 0x4f, 0xea, 0xda, 0x12, 0x05, 0x99, 0x03, 0xeb, 0x02, 0x13, 0x90, 0x33, 
  0xc1, 0xf3, 0x02, 0x1c, 0x00, 0xe0, 0x94, 0x46, 0xdd, 0xf8, 0x14, 0x90, 0x00, 0x2b, 0x40, 0xf3, 0xee, 0x80, 0x42, 0xf2, 
  0x15, 0x02, 0x3a, 0x80, 0x04, 0x98, 0x06, 0x22, 0x3a, 0x80, 0x04, 0x22, 0x42, 0x61, 0x45, 0x22, 0x3a, 0x80, 0xc9, 0xf3, 
  0x07, 0x42, 0x12, 0x02, 0x42, 0xf0, 0x02, 0x02, 0x3a, 0x80, 0x99, 0xfa, 0x99, 0xf2, 0x3a, 0x80, 0x5f, 0xfa, 0x89, 0xf2, 
  0xc2, 0xf5, 0x80, 0x72, 0x9a, 0x42, 0x98, 0x46, 0xc0, 0xf0, 0xae, 0x80, 0xff, 0x2b, 0x00, 0xf3, 0xad, 0x80, 0x62, 0x68, 
  0x2a, 0xb1, 0x1e, 0x07, 0x1c, 0xbf, 0x23, 0xf0, 0x0f, 0x08, 0x08, 0xf1, 0x10, 0x08, 0xb8, 0xf1, 0x0f, 0x0f, 0x0c, 0xd9, 
  0xb8, 0xf1, 0x10, 0x0f, 0x42, 0xf2, 0x05, 0x02, 0x18, 0xbf, 0x05, 0x22, 0x42, 0xf4, 0x80, 0x72, 0x3a, 0x80, 0xa8, 0xf1, 
  0x10, 0x08, 0x10, 0x26, 0x0a, 0xe0, 0x4f, 0xea, 0x08, 0x12, 0x42, 0xf4, 0x00, 0x52, 0x42, 0xf0, 0x05, 0x02, 0x92, 0xb2, 
  0x46, 0x46, 0x3a, 0x80, 0x4f, 0xf0, 0x00, 0x08, 0x62, 0x68, 0x00, 0x2a, 0x4b, 0xd0, 0xbc, 0xf1, 0x08, 0x0f, 0x0a, 0xd1, 
  0x06, 0x98, 0x09, 0x99, 0x03, 0x93, 0xff, 0xf7, 0x0b, 0xff, 0x03, 0x9b, 0xb1, 0x44, 0x10, 0x3b, 0x4f, 0xf0, 0x00, 0x0c, 
  0x58, 0xe0, 0x01, 0x22, 0xba, 0xf1, 0x00, 0x0f, 0x14, 0xd0, 0x68, 0x68, 0x29, 0x68, 0x88, 0x42, 0xfb, 0xd0, 0x69, 0x68, 
  0x0a, 0xf1, 0xff, 0x3a, 0x08, 0x78, 0xa1, 0x18, 0xc8, 0x74, 0x69, 0x68, 0x98, 0x1a, 0x01, 0x31, 0x8c, 0x42, 0x8a, 0xbf, 
  0x69, 0x68, 0x08, 0x99, 0x01, 0x31, 0x69, 0x60, 0x07, 0x90, 0x04, 0xe0, 0xa1, 0x18, 0xff, 0x20, 0xc8, 0x74, 0x99, 0x1a, 
  0x07, 0x91, 0xb2, 0x42, 0x02, 0xf1, 0x01, 0x01, 0x01, 0xd2, 0x0a, 0x46, 0xdc, 0xe7, 0x09, 0x9b, 0xcd, 0xf8, 0x00, 0xb0, 
  0x03, 0xeb, 0x4c, 0x02, 0x0b, 0x98, 0x05, 0x9b, 0x06, 0x99, 0xcd, 0xf8, 0x0c, 0xc0, 0xff, 0xf7, 0xde, 0xfe, 0x23, 0x69, 
  0xdd, 0xf8, 0x0c, 0xc0, 0x1b, 0xb1, 0x0c, 0xf1, 0x01, 0x03, 0x5f, 0xfa, 0x83, 0xfc, 0x05, 0x98, 0xb1, 0x44, 0x30, 0x44, 
  0x05, 0x90, 0x07, 0x9b, 0x1a, 0xe0, 0xba, 0xf1, 0x00, 0x0f, 0x13, 0xd0, 0x68, 0x68, 0x29, 0x68, 0x88, 0x42, 0xfb, 0xd0, 
  0x69, 0x68, 0x0a, 0xf1, 0xff, 0x3a, 0x08, 0x78, 0xa1, 0x18, 0x81, 0xf8, 0x24, 0x00, 0x69, 0x68, 0x01, 0x3b, 0x01, 0x31, 
  0x8c, 0x42, 0x8a, 0xbf, 0x69, 0x68, 0x08, 0x99, 0x01, 0x31, 0x69, 0x60, 0x01, 0x32, 0xb2, 0x42, 0xe5, 0xd3, 0xb1, 0x44, 
  0x00, 0x22, 0x06, 0x98, 0x01, 0x2e, 0x80, 0x5c, 0x02, 0xf1, 0x01, 0x01, 0x07, 0x90, 0x02, 0xd1, 0x02, 0x90, 0x00, 0x26, 
  0x07, 0xe0, 0x06, 0x98, 0x02, 0x3e, 0x41, 0x5c, 0x07, 0x98, 0x40, 0xea, 0x01, 0x21, 0x02, 0x91, 0x91, 0x1c, 0x04, 0x9a, 
  0xd0, 0x68, 0xc0, 0xf3, 0x04, 0x20, 0x0e, 0x28, 0xf9, 0xd8, 0x02, 0x98, 0x38, 0x80, 0x36, 0xb1, 0x0a, 0x46, 0xe2, 0xe7, 
  0x90, 0x46, 0x5a, 0xe7, 0x4f, 0xf4, 0x80, 0x78, 0x57, 0xe7, 0xb8, 0xf1, 0x00, 0x0f, 0x7f, 0xf4, 0x54, 0xaf, 0x04, 0x99, 
  0x4a, 0x69, 0x50, 0x07, 0xfb, 0xd5, 0x04, 0x98, 0x04, 0x22, 0x42, 0x61, 0x15, 0x22, 0x3a, 0x80, 0x05, 0x22, 0x3a, 0x80, 
  0x42, 0xf2, 0x16, 0x02, 0x3a, 0x80, 0x04, 0x99, 0xca, 0x68, 0x12, 0xf0, 0x7c, 0x5f, 0xfa, 0xd0, 0x04, 0x98, 0x42, 0x69, 
  0x51, 0x07, 0xfb, 0xd5, 0x0a, 0x99, 0x0a, 0x78, 0xd2, 0x07, 0xe8, 0xd4, 0x0e, 0xe7, 0x00, 0xbe, 
};

/***************************************************************************
*   openocd command interface                                              *
***************************************************************************/

/* Config Command: flash bank name driver base size chip_width bus_width target [driver_options]
      flash bank maxim_xip <base> <size> 0 0 <target> <flash_block> <flash_dummy> <flash_read> 
      <flash_write> <flash_erase> <flash_mass_erase> <spim_base> <spim_txfifo> <spim_rxfifo> 
      <spim_io> <spim_clk> <spix_base> <spix_io> <spix_clk> <crypto_base> <integrity>
 */
FLASH_BANK_COMMAND_HANDLER(maxim_xip_flash_bank_command)
{
    struct maxim_xip_flash_bank *maxim_info;

    LOG_INFO("--------------- FLASH_BANK_COMMAND_HANDLER\n");
    if (CMD_ARGC < 21) {
        LOG_WARNING("incomplete flash bank maxim configuration");
        return ERROR_FLASH_BANK_INVALID;
    }

    maxim_info = calloc(sizeof(struct maxim_xip_flash_bank), 1);
    COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], maxim_info->flash_size);
    COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], maxim_info->flash_block);
    COMMAND_PARSE_NUMBER(u32, CMD_ARGV[7], maxim_info->flash_dummy);
    COMMAND_PARSE_NUMBER(u32, CMD_ARGV[8], maxim_info->flash_read);
    COMMAND_PARSE_NUMBER(u32, CMD_ARGV[9], maxim_info->flash_write);
    COMMAND_PARSE_NUMBER(u32, CMD_ARGV[10], maxim_info->flash_erase);
    COMMAND_PARSE_NUMBER(u32, CMD_ARGV[11], maxim_info->flash_mass_erase);
    COMMAND_PARSE_NUMBER(u32, CMD_ARGV[12], maxim_info->spim_base);
    COMMAND_PARSE_NUMBER(u32, CMD_ARGV[13], maxim_info->spim_txfifo);
    COMMAND_PARSE_NUMBER(u32, CMD_ARGV[14], maxim_info->spim_rxfifo);
    COMMAND_PARSE_NUMBER(u32, CMD_ARGV[15], maxim_info->spim_io);
    COMMAND_PARSE_NUMBER(u32, CMD_ARGV[16], maxim_info->spim_clk);
    COMMAND_PARSE_NUMBER(u32, CMD_ARGV[17], maxim_info->spix_base);
    COMMAND_PARSE_NUMBER(u32, CMD_ARGV[18], maxim_info->spix_io);
    COMMAND_PARSE_NUMBER(u32, CMD_ARGV[19], maxim_info->spix_clk);
    COMMAND_PARSE_NUMBER(u32, CMD_ARGV[20], maxim_info->crypto_base);
    COMMAND_PARSE_NUMBER(u32, CMD_ARGV[21], maxim_info->integrity);

    bank->driver_priv = maxim_info;

    return ERROR_OK;
}

static int get_maxim_info(struct flash_bank *bank, char *buf, int buf_size)
{
    int printed;
    struct maxim_xip_flash_bank *maxim_info = bank->driver_priv;

    LOG_INFO("--------------- FLASH_BANK_COMMAND_HANDLER\n");
    if (maxim_info->probed == 0)
        return ERROR_FLASH_BANK_NOT_PROBED;

    printed = snprintf(buf, buf_size, "\nMaxim Integrated\n");
    buf += printed;
    buf_size -= printed;

    return ERROR_OK;
}

static int maxim_xip_setup_spim(struct flash_bank *bank)
{
    struct target *target = bank->target;
    struct maxim_xip_flash_bank *maxim_xip_info = bank->driver_priv;
    int retval;
    uint32_t tempvalue;

    LOG_INFO("--------------- maxim_xip_setup_spim\n");
    if (target->state != TARGET_HALTED)
    {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    if (maxim_xip_info->probed == 0) {
        LOG_ERROR("Target not probed");
        return ERROR_FLASH_BANK_NOT_PROBED;
    }

    if((maxim_xip_info->spim_io != 0) && (maxim_xip_info->spim_clk != 0)) {

        // Disable I/O and clocking for SPIX
        target_write_u32(target, maxim_xip_info->spix_io, 0);
        target_write_u32(target, maxim_xip_info->spix_clk, 0);

        // Enable the I/O for the SPIM
        target_write_u32(target, maxim_xip_info->spim_io, SPIM_IO);

        // Check the I/O response
        retval = target_read_u32(target, maxim_xip_info->spim_io + 0x4, &tempvalue);
        if (retval != ERROR_OK) {
            LOG_WARNING("target_read_u32() failed with %d", retval);
            return retval;
        } else if ((tempvalue & SPIM_IO) != SPIM_IO) {
            LOG_WARNING("SPI Master I/O request failure 0x%x", tempvalue);
            return ERROR_FAIL;
        }

        // Enable the clock for the SPIM
        target_write_u32(target, maxim_xip_info->spim_clk, SPIM_CLK);

    }

    // Initialize the SPIM
    target_write_u32(target, (maxim_xip_info->spim_base + SPIM_GEN_CTRL), SPIM_GEN_CTRL_INIT);
    target_write_u32(target, (maxim_xip_info->spim_base + SPIM_CFG), SPIM_CFG_INIT);

    maxim_xip_exit_qpi(bank);

    return ERROR_OK;
}

static int maxim_xip_setup_spix(struct flash_bank *bank)
{
    struct target *target = bank->target;
    struct maxim_xip_flash_bank *maxim_xip_info = bank->driver_priv;
    int retval;
    uint32_t tempvalue;

    LOG_INFO("--------------- maxim_xip_setup_spix\n");
    if (target->state != TARGET_HALTED)
    {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    if (maxim_xip_info->probed == 0) {
        return ERROR_FLASH_BANK_NOT_PROBED;
    }

    if((maxim_xip_info->spix_io != 0) && (maxim_xip_info->spix_clk != 0)) {

        // Disable I/O and clocking for SPIM
        target_write_u32(target, maxim_xip_info->spim_io, 0);
        target_write_u32(target, maxim_xip_info->spim_clk, 0);

        // Enable the I/O for the SPIX
        target_write_u32(target, maxim_xip_info->spix_io, SPIX_IO);

        // Check the I/O response
        retval = target_read_u32(target, (maxim_xip_info->spix_io + 0x4), &tempvalue);
        if (retval != ERROR_OK) {
            LOG_WARNING("target_read_u32() failed with %d", retval);
            return retval;
        } else if ((tempvalue & SPIX_IO) != SPIX_IO) {
            LOG_WARNING("SPIX I/O request failure");
            return ERROR_FAIL;
        }

        // Enable the clock or the SPIX
        target_write_u32(target, maxim_xip_info->spix_clk, SPIX_CLK);
    }

    // Disable the SPIM
    target_write_u32(target, (maxim_xip_info->spim_base + SPIM_GEN_CTRL), 0);

    // Initialize the SPIX
    target_write_u32(target, maxim_xip_info->spix_base + SPIX_CFG, SPIX_CFG_INIT);
    target_write_u32(target, maxim_xip_info->spix_base + SPIX_MODE, 
        (SPIX_MODE_INIT | (maxim_xip_info->flash_dummy & 0xF)));

    target_write_u32(target, maxim_xip_info->spix_base + SPIX_FETCH,
        (maxim_xip_info->flash_read & 0xFF));

      // Enable or disable the decryption
    if(maxim_xip_info->crypto_base != 0) {
      target_write_u32(target, maxim_xip_info->spix_base + SPIX_CRYPTO, SPIX_CRYPTO_INIT);
    } else {
      target_write_u32(target, maxim_xip_info->spix_base + SPIX_CRYPTO, 0x0);
    }

    return ERROR_OK;
}

static int maxim_xip_protect_check(struct flash_bank *bank)
{
    struct maxim_xip_flash_bank *maxim_info = bank->driver_priv;
    int i;

    LOG_INFO("--------------- maxim_xip_protect_check\n");
    if (maxim_info->probed == 0)
        return ERROR_FLASH_BANK_NOT_PROBED;

    for (i = 0; i < bank->num_sectors; i++)
        bank->sectors[i].is_protected = -1;

    /* TODO */
    return ERROR_FLASH_OPER_UNSUPPORTED;
}

static int maxim_xip_busy(struct flash_bank *bank)
{
    struct target *target = bank->target;
    struct maxim_xip_flash_bank *maxim_xip_info = bank->driver_priv;
    int retval;
    uint32_t temp32, timeout;
    uint8_t temp8;

    LOG_INFO("--------------- maxim_xip_busy\n");
    //               write | bytes      | 1 byte
    uint32_t header = (0x1 | (0x1 << 2) | (0x1 << 4));

    // Send the read SR command
    target_write_u16(target, maxim_xip_info->spim_txfifo, header);
    target_write_u16(target, maxim_xip_info->spim_txfifo, CMD_SR);

    //             read  | bytes      | 1 byte     | deassert SS
    header = ((0x1 << 1) | (0x1 << 2) | (0x1 << 4) | (0x1 << 13));

    // Clear the Results done flag
    target_write_u32(target, maxim_xip_info->spim_base + SPIM_INTFL, 
        SPIM_INTFL_RES_DONE);

    // Write the header to read 1 byte
    target_write_u16(target, maxim_xip_info->spim_txfifo, header);

    // Wait for the Results done flag
    timeout = TIMEOUT_CNT;
    while(1) {
        retval = target_read_u32(target, maxim_xip_info->spim_base + SPIM_INTFL, &temp32);
        if (retval != ERROR_OK) {
            LOG_WARNING("target_read_u32() failed with %d", retval);
            return retval;
        } else if ((temp32 & SPIM_INTFL_RES_DONE)) {
            break;
        }

        if(!(--timeout)) {
            LOG_WARNING("SPIM timeout");
            return ERROR_FAIL;
        }
    }

    // Read the results
    retval = target_read_u8(target, maxim_xip_info->spim_rxfifo, &temp8);
    if (retval != ERROR_OK) {
        LOG_WARNING("target_read_u8() failed with %d", retval);
        return retval;
    }

    if(temp8& CMD_SR_BUSY) {
        return ERROR_WAIT;
    }

    return ERROR_OK;
}

static void maxim_xip_cmd(struct flash_bank *bank, uint32_t cmd, uint8_t deass)
{
    struct target *target = bank->target;
    struct maxim_xip_flash_bank *maxim_xip_info = bank->driver_priv;

    // Create the header value

    LOG_INFO("--------------- maxim_xip_cmd\n");
    //               write | bytes      | 1 byte     | deassert SS
    uint32_t header = (0x1 | (0x1 << 2) | (0x1 << 4) | (0x1 << 13));

    // Send the write enable command
    target_write_u16(target, maxim_xip_info->spim_txfifo, header);
    target_write_u16(target, maxim_xip_info->spim_txfifo, CMD_WE);

    // Send the command
    //               write | bytes      | 1 byte     | deassert SS
    header = (0x1 | (0x1 << 2) | (0x1 << 4) | (deass << 13));
    target_write_u16(target, maxim_xip_info->spim_txfifo, header);
    target_write_u16(target, maxim_xip_info->spim_txfifo, cmd);
}

static int maxim_xip_exit_qpi(struct flash_bank *bank)
{
    struct target *target = bank->target;
    struct maxim_xip_flash_bank *maxim_xip_info = bank->driver_priv;

    LOG_INFO("--------------- maxim_xip_exit_qpi\n");
    // Exit the no command mode
    uint32_t header = (0x1 | (0x1 << 2) | (0x4 << 4) | (0x1 << 13) | (0x2 << 9));
    target_write_u16(target, maxim_xip_info->spim_txfifo, header);
    target_write_u16(target, maxim_xip_info->spim_txfifo, 0xFFFF);
    target_write_u16(target, maxim_xip_info->spim_txfifo, 0xFFFF);

    // Create the header value
    header = (0x1 | (0x1 << 2) | (0x1 << 4) | (0x1 << 13) | (0x2 << 9));

    // Send the EXIT QPI command
    target_write_u16(target, maxim_xip_info->spim_txfifo, header);
    target_write_u16(target, maxim_xip_info->spim_txfifo, CMD_EQPI0);

    // Create the header value
    header = (0x1 | (0x1 << 2) | (0x1 << 4) | (0x1 << 13) | (0x2 << 9));

    // Send the EXIT QPI command
    target_write_u16(target, maxim_xip_info->spim_txfifo, header);
    target_write_u16(target, maxim_xip_info->spim_txfifo, CMD_EQPI1);

    return ERROR_OK;
}

static int maxim_xip_erase(struct flash_bank *bank, int first, int last)
{
    struct maxim_xip_flash_bank *maxim_xip_info = bank->driver_priv;
    struct target *target = bank->target;
    int retval, banknr;
    unsigned total_len;

    LOG_INFO("--------------- maxim_xip_erase\n");
    if (bank->target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    if (maxim_xip_info->probed == 0) {
        return ERROR_FLASH_BANK_NOT_PROBED;
    }

    if ((first < 0) || (last < first) || (last >= bank->num_sectors)) {
        return ERROR_FLASH_SECTOR_INVALID;
    }

    // Adjust the number of banks if we are using integrity
    if(maxim_xip_info->integrity) {
      total_len = (last + 1) * maxim_xip_info->flash_block;
      total_len += ((total_len/0x80)+1)*0x10;
      last = (total_len / maxim_xip_info->flash_block) + 1;
    }

    if ((first == 0) && (last == (bank->num_sectors - 1))) {
        return maxim_xip_mass_erase(bank);
    }

    if((retval = maxim_xip_setup_spim(bank)) != ERROR_OK) {
        LOG_ERROR("Failed to initialize SPIM peripheral");
        maxim_xip_setup_spix(bank);
        return retval;
    }

    for (banknr = first; banknr <= last; banknr++) {

        // Send the sector erase command
        LOG_INFO("--------------- Send the  sector erase command\n");
        maxim_xip_cmd(bank, maxim_xip_info->flash_erase, 0);

        // Send the address
        //               write | bytes      | 3 bytes    | deassert SS
        uint32_t header = (0x1 | (0x1 << 2) | (0x3 << 4) | (0x1 << 13));
        
        target_write_u16(target, maxim_xip_info->spim_txfifo, header);

        uint16_t addr_temp = ((banknr * maxim_xip_info->flash_block >> 16) & 0xFF);
        addr_temp |= ((banknr * maxim_xip_info->flash_block) & 0xFF00);
        target_write_u16(target, maxim_xip_info->spim_txfifo, addr_temp);

        addr_temp = 0xF000;
        addr_temp |= ((banknr * maxim_xip_info->flash_block) & 0xFF);
        target_write_u16(target, maxim_xip_info->spim_txfifo, addr_temp);

        // Wait for the flash to not be busy
        int timeout = 0xFFF;
        while((retval = maxim_xip_busy(bank)) == ERROR_WAIT) {
            if(timeout-- == 1) {
				LOG_ERROR("Timeout waiting for external flash");
                return retval;
            }
        }

        if(retval != ERROR_OK) {
            LOG_ERROR("XIP busy error");
            return retval;
        }

        bank->sectors[banknr].is_erased = 1;
    }
       
    return maxim_xip_setup_spix(bank);
}

static int maxim_xip_protect(struct flash_bank *bank, int set, int first, int last)
{
    struct maxim_xip_flash_bank *maxim_info = bank->driver_priv;

    LOG_INFO("--------------- maxim_xip_protect\n");
    if (bank->target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    if (maxim_info->probed == 0)
        return ERROR_FLASH_BANK_NOT_PROBED;

/* TODO */
    return ERROR_FLASH_OPER_UNSUPPORTED;
}

static int maxim_xip_write_blob(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
    struct maxim_xip_flash_bank *maxim_xip_info = bank->driver_priv;
    struct target *target = bank->target;
    struct working_area *source;
    struct working_area *write_algorithm;
    uint32_t buffer_size = 0x4000;
    struct reg_param reg_params[5];
    struct mem_param mem_param[5];
    struct armv7m_algorithm armv7m_info;
    int retval = ERROR_OK;
    static const unsigned buf_min = 1024;

    LOG_INFO("--------------- maxim_xip_write_blob\n");
    LOG_WARNING("(bank=%p buffer=%p offset=0x%x, count=0x%x",
            bank, buffer, offset, count);

    // Allocate the blob
    if (target_alloc_working_area(target, sizeof(maxim_xip_write_code),
            &write_algorithm) != ERROR_OK) {

        LOG_DEBUG("no working area for block memory writes");
        return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
    }

    if (count < buffer_size) {
        // data plus wp, rp, addreses, enc_buffers, and stack
        buffer_size = count + ((7*4)+(16*3)+512);
    }

    // Allocate the memory buffer
    while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
        buffer_size /= 2;
        if (buffer_size <= buf_min) {
            target_free_working_area(target, write_algorithm);
            return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
        }
        LOG_DEBUG("retry target_alloc_working_area(%s, size=%u)",
                target_name(target), (unsigned) buffer_size);
    }

    // Write the algorithm
    target_write_buffer(target, write_algorithm->address,
        sizeof(maxim_xip_write_code), (uint8_t *) maxim_xip_write_code);

    LOG_WARNING("Working area addr = 0x%x size = 0x%x", write_algorithm->address,
      (unsigned int)(sizeof(maxim_xip_write_code) + buffer_size));

    init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
    init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
    init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
    init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);
    init_reg_param(&reg_params[4], "sp", 32, PARAM_OUT);

    buf_set_u32(reg_params[0].value, 0, 32, source->address);
    buf_set_u32(reg_params[1].value, 0, 32, (source->address + (source->size - ((5*4)+(16*3)+512))));
    buf_set_u32(reg_params[2].value, 0, 32, count);
    buf_set_u32(reg_params[3].value, 0, 32, offset);
    buf_set_u32(reg_params[4].value, 0, 32, (source->address + (source->size)));

    init_mem_param(&mem_param[0], source->address + (source->size - ((5*4)+(16*3)+512) + 0), 4, PARAM_OUT);
    init_mem_param(&mem_param[1], source->address + (source->size - ((5*4)+(16*3)+512) + 4), 4, PARAM_OUT);
    init_mem_param(&mem_param[2], source->address + (source->size - ((5*4)+(16*3)+512) + 8), 4, PARAM_OUT);
    init_mem_param(&mem_param[3], source->address + (source->size - ((5*4)+(16*3)+512) + 12), 4, PARAM_OUT);
    init_mem_param(&mem_param[4], source->address + (source->size - ((5*4)+(16*3)+512) + 16), 4, PARAM_OUT);

    buf_set_u32(mem_param[0].value, 0, 32, maxim_xip_info->spim_base);
    buf_set_u32(mem_param[1].value, 0, 32, maxim_xip_info->crypto_base);
    buf_set_u32(mem_param[2].value, 0, 32, maxim_xip_info->spim_txfifo);
    buf_set_u32(mem_param[3].value, 0, 32, maxim_xip_info->spim_rxfifo);
    buf_set_u32(mem_param[4].value, 0, 32, maxim_xip_info->integrity);

    armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
    armv7m_info.core_mode = ARM_MODE_THREAD;

    // Leave room for the encryption buffers
    // Set the starting address to the start of algo write
    retval = target_run_flash_async_algorithm(target, buffer, count, 1,
            5, mem_param,
            5, reg_params,
            source->address, (source->size - ((5*4)+(16*3)+512)),
            (write_algorithm->address + 0xc6), 0,
            &armv7m_info);

    if (retval == ERROR_FLASH_OPERATION_FAILED) {
        LOG_ERROR("error %d executing maxim flash write algorithm", retval);
    }

    target_free_working_area(target, write_algorithm);
    target_free_working_area(target, source);

    destroy_reg_param(&reg_params[0]);
    destroy_reg_param(&reg_params[1]);
    destroy_reg_param(&reg_params[2]);
    destroy_reg_param(&reg_params[3]);
    destroy_reg_param(&reg_params[4]);

    destroy_mem_param(&mem_param[0]);
    destroy_mem_param(&mem_param[1]);
    destroy_mem_param(&mem_param[2]);
    destroy_mem_param(&mem_param[3]);
    destroy_mem_param(&mem_param[4]);

    return retval;
}

static int maxim_xip_write_block(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
    struct maxim_xip_flash_bank *maxim_xip_info = bank->driver_priv;
    struct target *target = bank->target;
    uint32_t timeout, header_count, num, temp32;
    int retval;
    num = 0;

    LOG_INFO("--------------- maxim_xip_write_block\n");
    // Wait for the flash busy bit
    timeout = 0xFFF;
    while((retval = maxim_xip_busy(bank)) == ERROR_WAIT) {
        if(timeout-- == 1) {
			LOG_ERROR("Timeout waiting for external flash");
            return retval;
        }
    } 

    // Send the write command
    maxim_xip_cmd(bank, maxim_xip_info->flash_write, 0);

    // Send the address
    //               write | bytes      | 3 bytes 
    uint32_t header = (0x1 | (0x1 << 2) | (0x3 << 4));
    
    target_write_u16(target, maxim_xip_info->spim_txfifo, header);

    uint16_t addr_temp = (((offset >> 16) & 0xFF) | (offset & 0xFF00));
    target_write_u16(target, maxim_xip_info->spim_txfifo, addr_temp);

    addr_temp = 0xF000;
    addr_temp |= ((offset) & 0xFF);
    target_write_u16(target, maxim_xip_info->spim_txfifo, addr_temp);

    while(count > 0) {

        if(count >= 32) {
            header_count = 32;
            count -= 32;
        }
        else {
            header_count = count;
            count -= count;
        }

        // Send the header
        if(header_count == 32) {
            if(count == 0) {
                //      write | bytes      | 32 bytes    | deassert SS
                header = (0x1 | (0x1 << 2) | (0x0 << 4) | (0x1 << 13));
            } else {
                //      write | bytes      | 32 bytes
                header = (0x1 | (0x1 << 2) | (0x0 << 4));
            }
        } else {
            //      write | bytes      | header_count bytes  | deassert SS
            header = (0x1 | (0x1 << 2) | (header_count << 4) | (0x1 << 13));
        }

        target_write_u16(target, maxim_xip_info->spim_txfifo, header);

        while(header_count) {
            // Wait for the TXFIFO to have 2 bytes spaces free
            timeout = TIMEOUT_CNT;

            while(1) {
                // Clear the TX_FIFO almost empty flag
                target_write_u32(target, maxim_xip_info->spim_base + SPIM_INTFL, SPIM_INTFL_TXAE);

                // Wait for the AE flag to be set
                retval = target_read_u32(target, maxim_xip_info->spim_base + SPIM_INTFL, &temp32);

                if (retval != ERROR_OK) {
                    LOG_WARNING("target_read_u32() failed with %d", retval);
                    return retval;
                } else if (temp32 & SPIM_INTFL_TXAE) {
                    break;
                }

                if(!(--timeout)) {
                    LOG_WARNING("SPIM timeout SPIM_TXFIFO");
                    return ERROR_FAIL;
                }
            }

            // Put 2 bytes into the TXFIFO
            if(header_count > 1) {
                target_write_u16(target, maxim_xip_info->spim_txfifo, ((buffer[num]) | (buffer[num+1] << 8)));
                header_count -= 2;
                num += 2;
            } else if(header_count){
                target_write_u16(target, maxim_xip_info->spim_txfifo, (0xF000 | buffer[num]));
                header_count -= 1;
                num += 1;
            }
        }
    }

    return ERROR_OK;
}

static int maxim_xip_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
    struct maxim_xip_flash_bank *maxim_xip_info = bank->driver_priv;
    uint32_t blocks_remaining = count / WRITE_BLOCK;
    uint32_t remaining;
    int retval;

    LOG_INFO("--------------- maxim_xip_write\n");
    if (bank->target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    LOG_DEBUG("bank=%p buffer=%p offset=%08" PRIx32 " count=%08" PRIx32 "",
            bank, buffer, offset, count);

    if (maxim_xip_info->probed == 0)
        return ERROR_FLASH_BANK_NOT_PROBED;

    if (offset & 0x3) {
        LOG_WARNING("offset size must be word aligned");
        return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
    }

    if (offset + count > bank->size)
        return ERROR_FLASH_DST_OUT_OF_BANK;

    if((retval = maxim_xip_setup_spim(bank)) != ERROR_OK) {
        LOG_ERROR("Failed to initialize SPIM peripheral");
        maxim_xip_setup_spix(bank);
        return retval;
    }

    // Attempt to use the blob
    if(count >= 0x100) {
        retval = maxim_xip_write_blob(bank, buffer, offset, count);
        if(retval != ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
            if(retval == ERROR_OK) {
                return maxim_xip_setup_spix(bank); 
            } else {
                maxim_xip_setup_spix(bank);
                return retval;
            }
        }
    }

    // Write less than one block if the address is not aligned to the block boundary
    remaining = offset % WRITE_BLOCK;
    if(remaining) {
        if((retval = maxim_xip_write_block(bank, buffer, offset, remaining)) != ERROR_OK) {
            maxim_xip_setup_spix(bank);
            return retval;
        }
        buffer += remaining;
        offset += remaining;
        count -= remaining;
    }

    blocks_remaining = (count / WRITE_BLOCK);

    // Write the data in blocks
    while(blocks_remaining) {
        if((retval = maxim_xip_write_block(bank, buffer, offset, WRITE_BLOCK)) != ERROR_OK) {
            maxim_xip_setup_spix(bank);
            return retval;
        }
        buffer += WRITE_BLOCK;
        offset += WRITE_BLOCK;
        blocks_remaining--;
        count -= WRITE_BLOCK;
    }

    // Write the remainder length first to get on the page boundary
    if(count) {
        if((retval = maxim_xip_write_block(bank, buffer, offset, count)) != ERROR_OK) {
            maxim_xip_setup_spix(bank);
            return retval;
        }
    }

    return maxim_xip_setup_spix(bank);
}

static int maxim_xip_probe(struct flash_bank *bank)
{
    struct maxim_xip_flash_bank *maxim_info = bank->driver_priv;

    LOG_INFO("--------------- maxim_xip_probe\n");
    if (bank->sectors) {
        free(bank->sectors);
        bank->sectors = NULL;
    }

    /* provide this for the benefit of the NOR flash framework */
    bank->size = maxim_info->flash_size;
    bank->num_sectors = maxim_info->flash_size / maxim_info->flash_block;

    bank->sectors = calloc(bank->num_sectors, sizeof(struct flash_sector));
    for (int i = 0; i < bank->num_sectors; i++)
    {
        bank->sectors[i].offset = i * maxim_info->flash_block;
        bank->sectors[i].size = maxim_info->flash_block;
        bank->sectors[i].is_erased = -1;
        bank->sectors[i].is_protected = -1;
    }

    maxim_info->probed = 1;

    return ERROR_OK;
}

static int maxim_xip_mass_erase(struct flash_bank *bank)
{
    struct target *target = bank->target;
    struct maxim_xip_flash_bank *maxim_xip_info = bank->driver_priv;
    int retval;

    LOG_INFO("--------------- maxim_xip_mass_erase\n");
    if (target->state != TARGET_HALTED)
    {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    if (maxim_xip_info->probed == 0)
        return ERROR_FLASH_BANK_NOT_PROBED;

    if((retval = maxim_xip_setup_spim(bank)) == ERROR_OK) {
           
        maxim_xip_cmd(bank, maxim_xip_info->flash_mass_erase, 1);

        // Wait for the flash busy to clear
        LOG_WARNING("Mass erase is a long operation. On the order of 100 seconds");

        // Wait for the flash to not be busy
        int timeout = 0xFFFF;
        while((retval = maxim_xip_busy(bank)) == ERROR_WAIT) {
            if(timeout-- == 1) {
				LOG_ERROR("Timeout waiting for external flash");
                return retval;
            }
        }

        if(retval != ERROR_OK) {
            LOG_ERROR("XIP busy error");
            return retval;
        }

        maxim_xip_setup_spix(bank);

        return ERROR_OK;
    }

    LOG_ERROR("setup_spim fail"); 
    maxim_xip_setup_spix(bank);
    return retval;
}

static int maxim_xip_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
    int retval;

    LOG_INFO("--------------- maxim_xip_read\n");
    if((retval = maxim_xip_setup_spix(bank)) != ERROR_OK) {
        return retval;
    }

    return default_flash_read(bank, buffer, offset, count);
}

COMMAND_HANDLER(maxim_xip_handle_mass_erase_command)
{
    int i;

    LOG_INFO("--------------- COMMAND_HANDLER\n");
    if (CMD_ARGC < 1) {
        command_print(CMD_CTX, "maxim_xip mass_erase <bank>");
        return ERROR_OK;
    }

    struct flash_bank *bank;
    int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
    if (ERROR_OK != retval)
        return retval;

    if (maxim_xip_mass_erase(bank) == ERROR_OK) {
        /* set all sectors as erased */
        for (i = 0; i < bank->num_sectors; i++)
            bank->sectors[i].is_erased = 1;

        command_print(CMD_CTX, "maxim_xip mass erase complete");
    }
    else {
        command_print(CMD_CTX, "maxim_xip mass erase failed");
    }

    return ERROR_OK;
}

COMMAND_HANDLER(maxim_xip_handle_setup_spix_command)
{
    if (CMD_ARGC < 1) {
        command_print(CMD_CTX, "maxim_xip setup_spix <bank>");
        return ERROR_OK;
    }

    struct flash_bank *bank;
    int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
    if (ERROR_OK != retval)
        return retval;

    if (maxim_xip_setup_spix(bank) == ERROR_OK) {

        command_print(CMD_CTX, "maxim_xip setup_spix complete");
    }
    else {
        command_print(CMD_CTX, "maxim_xip setup_spix failed");
    }

    return ERROR_OK;
}

COMMAND_HANDLER(maxim_xip_handle_setup_spim_command)
{
    if (CMD_ARGC < 1) {
        command_print(CMD_CTX, "maxim_xip setup_spim <bank>");
        return ERROR_OK;
    }

    struct flash_bank *bank;
    int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
    if (ERROR_OK != retval)
        return retval;

    if (maxim_xip_setup_spim(bank) == ERROR_OK) {

        command_print(CMD_CTX, "maxim_xip setup_spim complete");
    }
    else {
        command_print(CMD_CTX, "maxim_xip setup_spim failed");
    }

    return ERROR_OK;
}

static const struct command_registration maxim_xip_exec_command_handlers[] = {
    {
        .name = "mass_erase",
        .handler = maxim_xip_handle_mass_erase_command,
        .mode = COMMAND_EXEC,
        .usage = "bank_id",
        .help = "erase entire external flash device",
    },
    {
        .name = "setup_spix",
        .handler = maxim_xip_handle_setup_spix_command,
        .mode = COMMAND_EXEC,
        .usage = "bank_id",
        .help = "configure the SPIX peripheral",
    },
    {
        .name = "setup_spim",
        .handler = maxim_xip_handle_setup_spim_command,
        .mode = COMMAND_EXEC,
        .usage = "bank_id",
        .help = "configure the SPIM peripheral",
    },
    COMMAND_REGISTRATION_DONE
};

static const struct command_registration maxim_xip_command_handlers[] = {
    {
        .name = "maxim_xip",
        .mode = COMMAND_EXEC,
        .help = "Maxim XIP flash command group",
        .chain = maxim_xip_exec_command_handlers,
    },
    COMMAND_REGISTRATION_DONE
};

struct flash_driver maxim_xip_flash = {
    .name = "maxim_xip",
    .commands = maxim_xip_command_handlers,
    .flash_bank_command = maxim_xip_flash_bank_command,
    .erase = maxim_xip_erase,
    .protect = maxim_xip_protect,
    .write = maxim_xip_write,
    .read = maxim_xip_read,
    .probe = maxim_xip_probe,
    .auto_probe = maxim_xip_probe,
    .erase_check = default_flash_blank_check,
    .protect_check = maxim_xip_protect_check,
    .info = get_maxim_info,
};
