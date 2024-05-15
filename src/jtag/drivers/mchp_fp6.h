// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2024 by Microchip Inc.                                  *
 *   matteo.bonicolini@microchip.com                                       *
 ***************************************************************************/

#ifndef MCHP_FP6
#define MCHP_FP6

#include "mchp_fp.h"

#define FP6_VID 0x1514
#define FP6_PID 0x2009

#define PAYLOAD_PACKET_START_ADDRESS			12
#define PAYLOAD_READ_PACKET_START_ADDRESS		8

#define PACKET_CRC_BYTE_SIZE				2

#define FP6_TRSTB_BIT					0x4

enum op_status {
	OP_UNDEF,
	OP_OFF,
	OP_PASS,
	OP_FAIL,
	OP_ACTIVE,
	OP_MAX
};

#endif
