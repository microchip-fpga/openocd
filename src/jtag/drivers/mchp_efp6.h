// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2024 by Microchip Inc.                                  *
 *   matteo.bonicolini@microchip.com                                       *
 ***************************************************************************/

#ifndef MCHP_EFP6
#define MCHP_EFP6

#include "mchp_fp.h"

#define EFP6_VID	0x1514
#define EFP6_REV_A_PID	0x200a
#define EFP6_REV_B_PID	0x200b

#define EFP6_MAX_SERIAL_STRING		126  // https://stackoverflow.com/questions/7193645/how-long-is-the-string-of-manufacturer-of-a-usb-device

/* Discrete JTAG pins */
#define EFP6_TRSTB_BIT			0x4

struct __attribute__((__packed__)) efp6_packet_atomic {
	uint16_t packet_start;
	uint16_t packet_type;
	uint32_t address;
	uint16_t len_high;
	uint16_t len_low;
	uint8_t data[2];
	uint16_t crc;
};

struct efp6_ir_scan {
	struct efp6_packet_atomic set_len;
	struct efp6_packet_atomic data;
	struct efp6_packet_atomic shift;
	struct efp6_packet_atomic end_state;
};

#define EFP6_PACKET_OFFSET			4
#define EFP6_PACKET_HEADER_OPCODE_OFFSET	2
#define EFP6_PACKET_HEADER_PREAMBLE_LENGTH	(10 + EFP6_PACKET_HEADER_OPCODE_OFFSET)
#define EFP6_PACKET_HEADER_POSTAMBLE_LENGTH	2
#define EFP6_PACKET_HEADER_TOTAL_LENGTH		(EFP6_PACKET_HEADER_PREAMBLE_LENGTH + EFP6_PACKET_HEADER_POSTAMBLE_LENGTH)

#define EFP6_RESPONSE_PACKET_SIZE_ERROR		-100
#define EFP6_RESPONSE_DEVICE_OPEN_ERROR		-101
#define EFP6_RESPONSE_BITS_SIZE_ERROR		-102

enum efp6_color {
	EFP6_OFF,
	EFP6_SOLID_GREEN,
	EFP6_SOLID_RED,
	EFP6_BLINK_GREEN = 5,
	EFP6_BLINK_RED
};

static int mchp_efp6_get_cm3_version(void);
static int mchp_efp6_enable_jtag_port(void);
static int mchp_efp6_disable_jtag_port(void);
static int mchp_efp6_set_led(uint8_t status);

#endif
