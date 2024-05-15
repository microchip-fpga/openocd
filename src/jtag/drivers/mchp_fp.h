// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2024 by Microchip Inc.                                  *
 *   matteo.bonicolini@microchip.com                                       *
 ***************************************************************************/

#ifndef MCHP_FP
#define MCHP_FP

#define HZ_PER_KHZ					1000

#define FP_MAX_USB_BUFFER_BYTE_SIZE 			1024

#define FP_READ_TIMEOUT					3500

#define FP_PACKET_START_CODE				0x01

/* sub commands */
#define FP_SET_TCK_FREQUENCY				0
#define FP_FREQUENCY_READ_BYTE_LOCATION			13

#define FP_TARGET_FREQUENCY_ADDRESS			0xa0fffff0

/* Opcodes */
#define SEND_PACKET_OPCODE				0x11
#define DDR_REMAP_OPCODE				0x12
#define ENVM_REMAP_OPCODE				0x13
#define READ_PRINT_BUFFER_OPCODE			0x21
#define M3_IN_RESET_OPCODE				0x23
#define M3_OUT_OF_RESET_OPCODE				0x24
#define FP6_READ_TCK_OPCODE				0x25
#define READ_PROGRESS_OPCODE				0x26
#define MSS_IN_RESET_OPCODE				0x27
#define MSS_OUT_OF_RESET_OPCODE				0x28
#define INTERRUPT_OPCODE				0x29
#define JTAG_ATOMIC_OPCODE				0x2a
#define SET_SHIFT_IR_DR_BIT_LENGTH_OPCODE		0x2c
#define SHIFT_DATA_FROM_REGISTER_OPCODE			0x2d
#define SHIFT_DATA_FROM_DDR_OPCODE			0x2e
#define READ_TDO_BUFFER_COMMAND_OPCODE			0x2f
#define FREQUENCY_INTERRUPT_OPCODE			0x30
#define INSERT_WAIT_CYCLES_OPCODE			0x31
#define ENABLE_JTAG_PORT_OPCODE				0x32
#define DISABLE_JTAG_PORT_OPCODE			0x33
#define SET_JTAG_PINS_OPCODE				0x34
#define GET_JTAG_PINS_OPCODE				0x35
#define GET_CM3_STATUS_OPCODE				0x36
#define SET_CHAIN_PARAMETER_OPCODE			0x37
#define READ_CHAIN_PARAMETER_OPCODE			0x38
#define READ_INBOX_OPCODE				0x39
#define TURN_ACTIVITY_LED_ON_OPCODE			0x41
#define TURN_ACTIVITY_LED_OFF_OPCODE			0x42
/* eFP6 Vendor Commands */
#define EFP6_CM3_FW_VERSION_OPCODE		        0xc0
/* FP6 Vendor Commands */
#define FP6_PROGRAM_I2C_VENDOR_OPCODE			0xba
#define FP6_SET_PROGRAMMER_SERIAL_NUMBER_VENDOR_OPCODE	0xba
#define FP6_GET_PROGRAMMER_SERIAL_NUMBER_VENDOR_OPCODE	0xbb
#define FP6_GET_FX3_FIRMWARE_VERSION_VENDOR_OPCODE	0xc0
#define FP6_SET_LED_OFF_VENDOR_OPCODE			0xc1
#define FP6_SET_LED_PASS_VENDOR_OPCODE			0xc2
#define FP6_SET_LED_FAIL_VENDOR_OPCODE			0xc3
#define FP6_SET_VPUMP_LOW_VENDOR_OPCODE			0xc4
#define FP6_SET_VPUMP_HIGH_VENDOR_OPCODE		0xc5
#define FP6_SET_PROG_MODE_LOW_VENDOR_OPCODE		0xc6
#define FP6_SET_PROG_MODE_HIGH_VENDOR_OPCODE		0xc7
#define FP6_FX3_RESET_OPCODE				0xc8
#define FP6_SF2_RESET_OPCODE				0xc9
#define FP6_READ_ADC_VPUMP_VENDOR_OPCODE		0xee
#define FP6_READ_ADC_VJTAG_VENDOR_OPCODE		0xef

#define COMMAND_PACKET_LENGTH				14
#define ACTION_PACKET_LENGTH				16
#define READ_PACKET_LENGTH				128
#define PROGRESS_PACKET_LENGTH				128
#define PAYLOAD_ATOMIC_PACKET_LENGTH			16
#define ATOMIC_JTAG_OPERATION_PACKET_LENGTH		2

#endif
