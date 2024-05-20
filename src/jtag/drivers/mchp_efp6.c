// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2024 by Microchip Inc.                                  *
 *   matteo.bonicolini@microchip.com                                       *
 ***************************************************************************/

#include <hidapi.h>
#include <jtag/interface.h>

#include "mchp_efp6.h"
#include "helper/log.h"

static int quitting;

static hid_device *efp6_handle;

static wchar_t serial_number[EFP6_MAX_SERIAL_STRING];
static char specific_serial_number[EFP6_MAX_SERIAL_STRING];
static bool specified_serial_number = 0;

__attribute__((aligned(8)))
static uint8_t usb_in_buffer[FP_MAX_USB_BUFFER_BYTE_SIZE];

__attribute__((aligned(8)))
static uint8_t usb_out_buffer[FP_MAX_USB_BUFFER_BYTE_SIZE];

static uint8_t packets_in_buffer;
static uint32_t bytes_in_buffer = EFP6_PACKET_OFFSET;

__attribute__((always_inline)) inline
static void write16(uint8_t *buf, uint16_t val)
{
	uint8_t *ptr = buf;

	*ptr++ = val & 0xff;
	*ptr = val >> 8;
}

__attribute__((always_inline)) inline
static void add_usb_packet_header(uint16_t packet_type, uint32_t target_address, uint32_t packet_length)
{
	uint8_t *ptr = &usb_in_buffer[bytes_in_buffer];

	write16(ptr, FP_PACKET_START_CODE);
	ptr += 2;

	write16(ptr, packet_type);
	ptr += 2;

	write16(ptr, target_address >> 16);
	ptr += 2;
	write16(ptr, target_address);
	ptr += 2;

	write16(ptr, packet_length >> 16);
	ptr += 2;
	write16(ptr, packet_length);

	/* Skipping CRC - the firmware/IP ignores it */
}

__attribute__((always_inline)) inline
static void add_usb_packet_header_simpler(uint16_t packet_type, uint32_t packet_length)
{
	uint8_t *ptr = &usb_in_buffer[bytes_in_buffer];

	/*
	 * Similar as add_usb_packet_header, but even more simplified, it assumes:
	 * - the target address is 0
	 * - the length is under 65536
	 * - the CRC is ignored in the firmware/IP and doesn't have to be set
	 * - that we correctly zeroized the buffer previously and all the skipped writes will have 0 in the buffer anyway
	 */
	write16(ptr, FP_PACKET_START_CODE);
	ptr += 2;

	write16(ptr, packet_type);
	ptr += 8;

	/* Skipping high 16-bits of the 32-bit target address */
	/* Skipping low 16-bits of the 32-bit target address */

	/* Skipping High 16-bits of the 32-bit packet length */
	write16(ptr, packet_length);

	/* Skipping CRC - the firmware/IP ignores it */
}

__attribute__((always_inline)) inline
static int flush_usb_buffer(void)
{
	int err;

	/*
	 * __MINGW32__ is detecting the Windows builds, for Windows it uses separate slightly bigger buffer
	 * so it can prepone the payload with one 0. In essence the REPORT ID is optional, but not on Windows
	 * so it has to be set to 0 to skip it and we need to send 1024 payload on Windows we have to send 1025
	 * instead. Cleaner it would be to put it into hidapi and then treat the API the same and I did it for
	 * a moment and the build can be changed to use different fork, but then having to maintain it could
	 * be hassle so I reverted to a ugly fix on the host side done here, instead of having it the library.
	 * There are numerous issues around the 0 on Windows on the signal11 repository, and it looks like
	 * the libusb (maintained fork) shares the same problem
	 * https://stackoverflow.com/a/31668028/4535300
	 */
	#ifdef __MINGW32__
	static uint8_t usb_in_buffer_reportid[FP_MAX_USB_BUFFER_BYTE_SIZE + 1];

	usb_in_buffer_reportid[0] = 0;
	#endif

	usb_in_buffer[0] = packets_in_buffer;

	if (quitting) {
		return ERROR_OK;
	}

	#ifdef __MINGW32__
	memcpy(usb_in_buffer_reportid + 1, usb_in_buffer, FP_MAX_USB_BUFFER_BYTE_SIZE);
	err = hid_write(efp6_handle, usb_in_buffer_reportid, FP_MAX_USB_BUFFER_BYTE_SIZE + 1);
	#else
	err = hid_write(efp6_handle, usb_in_buffer, FP_MAX_USB_BUFFER_BYTE_SIZE);
	#endif

	if (err < 0) {
		LOG_ERROR("Embedded FlashPro6 failed to send the data. Programmer device reset is required.  Err = %d (%ls)", err, hid_error(efp6_handle));
		exit(0);
	}
	memset(usb_in_buffer, 0, bytes_in_buffer);

	err = hid_read_timeout(efp6_handle, usb_out_buffer, FP_MAX_USB_BUFFER_BYTE_SIZE, FP_READ_TIMEOUT);
	if (err < (int)FP_MAX_USB_BUFFER_BYTE_SIZE) {
		LOG_ERROR("Failed to read data from USB buffer.  Programmer reset is required.  Err = %d (%ls)", err, hid_error(efp6_handle));
		exit(0);
	}

	bytes_in_buffer = EFP6_PACKET_OFFSET;
	packets_in_buffer = 0;

	return ERROR_OK;
}

__attribute__((always_inline)) inline
static void add_packet_to_usb_buffer(uint16_t opcode, const uint8_t *buf, uint32_t packet_length, uint32_t num_padding_bytes, uint32_t target_address)
{
	if (buf != NULL)
		memcpy(&usb_in_buffer[bytes_in_buffer + EFP6_PACKET_HEADER_PREAMBLE_LENGTH], buf, packet_length);

	if (target_address != 0)
		add_usb_packet_header(opcode, target_address, packet_length + num_padding_bytes);
	else
		add_usb_packet_header_simpler(opcode, packet_length + num_padding_bytes);

	bytes_in_buffer += EFP6_PACKET_HEADER_TOTAL_LENGTH + packet_length + num_padding_bytes;

	packets_in_buffer++;
}

__attribute__((always_inline)) inline
static int construct_and_send_packet(uint16_t opcode, uint8_t const *buf, uint32_t packet_length, uint32_t target_address)
{
	add_packet_to_usb_buffer(opcode, buf, packet_length, 0, target_address);
	return flush_usb_buffer();
}

__attribute__((always_inline)) inline
static int retrieve_data(uint8_t *buf, uint32_t bit_length)
{
	int err;
	uint32_t byte_length;

	byte_length = (bit_length + 7) / 8;

	err = construct_and_send_packet(READ_TDO_BUFFER_COMMAND_OPCODE, NULL, 0, 0);

	if (err)
		return err;

	memcpy(buf, usb_out_buffer, byte_length);

	return err;
}

static int mchp_efp6_enumerate(const char *partial_port_name)
{
	struct hid_device_info *devs;
	uint32_t num_found_devices = 0;
	size_t port_string_size = 0;
	wchar_t efp6port[EFP6_MAX_SERIAL_STRING];

	if (partial_port_name) {
		port_string_size = strlen(partial_port_name);
		mbstowcs(efp6port, partial_port_name, port_string_size + 1);
	}

	if (hid_init() != 0) {
		LOG_ERROR("unable to open HIDAPI");
		return ERROR_FAIL;
	}

	devs = hid_enumerate(EFP6_VID, EFP6_REV_A_PID);
	for (struct hid_device_info *cur_dev = devs; cur_dev; cur_dev = cur_dev->next) {
		LOG_INFO("Embedded FlashPro6 Rev B found (%04hx:%04hx %ls %ls %ls)",
			cur_dev->vendor_id, cur_dev->product_id,
			cur_dev->manufacturer_string, cur_dev->product_string,
			cur_dev->serial_number);
		LOG_WARNING("Please upgrade firmware to Rev B");
		exit(0);
	}
	hid_free_enumeration(devs);

	devs = hid_enumerate(EFP6_VID, EFP6_REV_B_PID);
	for (struct hid_device_info *cur_dev = devs; cur_dev; cur_dev = cur_dev->next) {
		LOG_INFO("Embedded FlashPro6 Rev B found (%04hx:%04hx %ls %ls %ls)",
			cur_dev->vendor_id, cur_dev->product_id,
			cur_dev->manufacturer_string, cur_dev->product_string,
			cur_dev->serial_number);

		if (port_string_size == 0) {
			num_found_devices++;
			wcsncpy(serial_number, cur_dev->serial_number, EFP6_MAX_SERIAL_STRING);
		} else {
			if (0 == wcsncmp(cur_dev->serial_number, efp6port, port_string_size)) {
				num_found_devices++;
				wcsncpy(serial_number, cur_dev->serial_number, EFP6_MAX_SERIAL_STRING);
				LOG_DEBUG("The device at %s with port %d matched with the port checker", cur_dev->path, *(cur_dev->serial_number));
			} else {
				LOG_DEBUG("The device at %s with port %d will be ignored because it didn't match the port checker", cur_dev->path, *(cur_dev->serial_number));
			}
		}
	}

	hid_free_enumeration(devs);

	if (num_found_devices == 0)
		LOG_INFO("No Embedded FlashPro6 devices found");

	return num_found_devices;
}

static int mchp_efp6_open(void)
{
	int err;

	memset(usb_in_buffer, 0, FP_MAX_USB_BUFFER_BYTE_SIZE);
	memset(usb_out_buffer, 0, FP_MAX_USB_BUFFER_BYTE_SIZE);

	efp6_handle = hid_open(EFP6_VID, EFP6_REV_B_PID, serial_number);
	if (efp6_handle == NULL) {
		LOG_ERROR("Unable to open Embedded FlashPro6 device");
		return EFP6_RESPONSE_DEVICE_OPEN_ERROR;
	}

	err = hid_set_nonblocking(efp6_handle, 0);
	if (err) {
		LOG_ERROR("Embedded FlashPro6 hid_set_nonblocking failed: %d (%ls)", err, hid_error(efp6_handle));
		return err;
	}

	err = mchp_efp6_get_cm3_version();
	if (err) {
		LOG_ERROR("Embedded FlashPro6 get_cm3_version failed (%d)", err);
		return err;
	}

	err = mchp_efp6_enable_jtag_port();
	if (err) {
		LOG_ERROR("Embedded FlashPro6 enable Jtag port failed (%d)", err);
		return err;
	}

	mchp_efp6_set_led(EFP6_SOLID_GREEN);

	return ERROR_OK;
}

static int mchp_efp6_quit(void)
{
	quitting = 1;
	LOG_INFO("Embedded FlashPro6 closing the device");

	mchp_efp6_disable_jtag_port();
	mchp_efp6_set_led(EFP6_OFF);

	hid_close(efp6_handle);
	hid_exit();

	return ERROR_OK;
}

static int mchp_efp6_get_cm3_version(void)
{
	int err;

	err = construct_and_send_packet(EFP6_CM3_FW_VERSION_OPCODE, NULL, 0, 0);
	if (err)
		LOG_ERROR("Embedded FlashPro6 failed to read firmware version number (%d)", err);
	else
		LOG_INFO("Embedded FlashPro6 CM3 firmware version: %X.%X", usb_out_buffer[1], usb_out_buffer[0]);

	return err;
}

static int mchp_efp6_speed(int32_t tck_freq)
{
	uint8_t payload_buf[PAYLOAD_ATOMIC_PACKET_LENGTH] = { 0 };
	const int default_tck_mhz = 12;
	int err;

	if (tck_freq <= 0) {
		LOG_WARNING("Embedded FlashPro6 TCK frequency is set to 0, use valid frequency of %dMHz instead", default_tck_mhz);
		tck_freq = default_tck_mhz * 1000000;
	}

	tck_freq = tck_freq / 1000000;

	if (tck_freq < 4 || tck_freq > 20) {
		LOG_WARNING("Embedded FlashPro6 frequency range is 4 MHz - 20 MHz. Setting frequency to %dMHz", default_tck_mhz);
		tck_freq = default_tck_mhz;
	}

	payload_buf[0] = FP_SET_TCK_FREQUENCY;
	payload_buf[1] = tck_freq;

	err = construct_and_send_packet(SEND_PACKET_OPCODE, payload_buf, PAYLOAD_ATOMIC_PACKET_LENGTH, FP_TARGET_FREQUENCY_ADDRESS);
	if (err)
		return err;

	return construct_and_send_packet(FREQUENCY_INTERRUPT_OPCODE, NULL, 0, 0);
}

static int mchp_efp6_enable_jtag_port(void)
{
	return construct_and_send_packet(ENABLE_JTAG_PORT_OPCODE, NULL, 0, 0);
}

static int mchp_efp6_disable_jtag_port(void)
{
	return construct_and_send_packet(DISABLE_JTAG_PORT_OPCODE, NULL, 0, 0);
}

static int mchp_efp6_execute_reset(struct reset_command const *cmd)
{
	uint8_t payload_buf[2] = { 0 };

	if (!cmd->trst)
		payload_buf[0] = EFP6_TRSTB_BIT;

	return construct_and_send_packet(SET_JTAG_PINS_OPCODE, payload_buf, ATOMIC_JTAG_OPERATION_PACKET_LENGTH, 0);
}

static int mchp_efp6_set_led(uint8_t status)
{
	int err;
	uint8_t payload_buf[2] = { 0 };

	payload_buf[0] = status;

	err = construct_and_send_packet(TURN_ACTIVITY_LED_OFF_OPCODE, NULL, 0, 0);
	if (err)
		return err;

	if (status != EFP6_OFF)
		err = construct_and_send_packet(TURN_ACTIVITY_LED_ON_OPCODE, payload_buf, ATOMIC_JTAG_OPERATION_PACKET_LENGTH, 0);

	return err;
}

static int mchp_efp6_jtag_go_to_state_raw(int state)
{
	int err = ERROR_OK;
	uint8_t payload_buf[ATOMIC_JTAG_OPERATION_PACKET_LENGTH];

	write16(payload_buf, state);

	add_packet_to_usb_buffer(JTAG_ATOMIC_OPCODE, payload_buf, ATOMIC_JTAG_OPERATION_PACKET_LENGTH, 0, 0);

	if (state == TAP_RESET)
		err = flush_usb_buffer();

	return err;
}

static int mchp_efp6_execute_statemove(struct statemove_command const *cmd)
{
	int err = ERROR_OK;
	uint8_t payload_buf[ATOMIC_JTAG_OPERATION_PACKET_LENGTH];

	write16(payload_buf, cmd->end_state);

	add_packet_to_usb_buffer(JTAG_ATOMIC_OPCODE, payload_buf, ATOMIC_JTAG_OPERATION_PACKET_LENGTH, 0, 0);

	if (cmd->end_state == TAP_RESET)
		err = flush_usb_buffer();

	return err;
}

static int mchp_efp6_execute_runtest(struct runtest_command const *cmd)
{
	if (cmd->end_state != TAP_IDLE)
		LOG_ERROR("Embedded FlashPro6 execute run-test state only implemented with TAP_IDLE end-state");

	for (int i = 0; i < cmd->num_cycles; i++)
		mchp_efp6_jtag_go_to_state_raw(TAP_IDLE);

	return mchp_efp6_jtag_go_to_state_raw(TAP_IDLE);
}

static int mchp_efp6_ir_scan(uint32_t bit_length, const uint8_t *out, uint8_t *in, uint16_t end_state)
{
	int err = ERROR_OK;

	__attribute__((aligned(8)))
	static struct efp6_ir_scan full_scan = {
		.set_len = {
			.packet_start = FP_PACKET_START_CODE,
			.packet_type = SET_SHIFT_IR_DR_BIT_LENGTH_OPCODE,
			.address = 0,
			.len_high = 0,
			.len_low = ATOMIC_JTAG_OPERATION_PACKET_LENGTH,
			.data[0] = 0,
			.data[1] = 0,
			.crc = 0
		},
		.data = {
			.packet_start = FP_PACKET_START_CODE,
			.packet_type = SHIFT_DATA_FROM_REGISTER_OPCODE,
			.address = 0,
			.len_high = 0,
			.len_low = ATOMIC_JTAG_OPERATION_PACKET_LENGTH,
			.data[0] = 0,
			.data[1] = 0,
			.crc = 0
		},
		.shift = {
			.packet_start = FP_PACKET_START_CODE,
			.packet_type = JTAG_ATOMIC_OPCODE,
			.address = 0,
			.len_high = 0,
			.len_low = ATOMIC_JTAG_OPERATION_PACKET_LENGTH,
			.data[0] = TAP_IRSHIFT,
			.data[1] = 0,
			.crc = 0
		},
		.end_state = {
			.packet_start = FP_PACKET_START_CODE,
			.packet_type = JTAG_ATOMIC_OPCODE,
			.address = 0,
			.len_high = 0,
			.len_low = ATOMIC_JTAG_OPERATION_PACKET_LENGTH,
			.data[0] = 0,
			.data[1] = 0,
			.crc = 0
		}
	};

	full_scan.set_len.data[0] = bit_length;
	full_scan.data.data[0] = out[0];
	full_scan.data.data[1] = out[1];
	full_scan.end_state.data[0] = end_state;

	memcpy(&usb_in_buffer[bytes_in_buffer], (uint8_t *)&full_scan, sizeof(full_scan));

	bytes_in_buffer += sizeof(full_scan);
	packets_in_buffer += 4;

	if (end_state == TAP_RESET)
		err = flush_usb_buffer();

	if (in && err == ERROR_OK)
		err = retrieve_data(in, bit_length);

	return err;
}

static int mchp_efp6_dr_scan(uint32_t bit_length, const uint8_t *out, uint8_t *in, uint16_t end_state)
{
	int err = ERROR_OK;
	uint8_t payload_buf[ATOMIC_JTAG_OPERATION_PACKET_LENGTH];
	uint32_t bytes_to_transmit;
	uint32_t padding_bytes;

	write16(payload_buf, bit_length);
	add_packet_to_usb_buffer(SET_SHIFT_IR_DR_BIT_LENGTH_OPCODE, payload_buf, ATOMIC_JTAG_OPERATION_PACKET_LENGTH, 0, 0);

	padding_bytes = 0;
	if (bit_length > 16) {
		bytes_to_transmit = (uint32_t)((bit_length + 7) / 8);
		if (bytes_to_transmit % 16)
			padding_bytes = (16 - bytes_to_transmit % 16);

		add_packet_to_usb_buffer(SHIFT_DATA_FROM_DDR_OPCODE, out, bytes_to_transmit, padding_bytes, 0);

		if (bytes_in_buffer % EFP6_PACKET_OFFSET)
			bytes_in_buffer += (EFP6_PACKET_OFFSET - bytes_in_buffer % EFP6_PACKET_OFFSET);
	} else {
		add_packet_to_usb_buffer(SHIFT_DATA_FROM_REGISTER_OPCODE, out, 2, 0, 0);
	}

	payload_buf[0] = TAP_DRSHIFT;
	payload_buf[1] = 0;
	add_packet_to_usb_buffer(JTAG_ATOMIC_OPCODE, payload_buf, ATOMIC_JTAG_OPERATION_PACKET_LENGTH, 0, 0);

	err = mchp_efp6_jtag_go_to_state_raw(end_state);

	if (in && err == ERROR_OK)
		err = retrieve_data(in, bit_length);

	return err;
}

static int mchp_efp6_execute_scan(struct scan_command *cmd)
{
	int err;

	if (cmd->num_fields > 1) {
		LOG_ERROR("Execute scan is implemented only with one field but this request has %d fields", cmd->num_fields);
		return ERROR_FAIL;
	}

	if (cmd->ir_scan)
		err = mchp_efp6_ir_scan(cmd->fields[0].num_bits, cmd->fields[0].out_value, cmd->fields[0].in_value, cmd->end_state);
	else
		err = mchp_efp6_dr_scan(cmd->fields[0].num_bits, cmd->fields[0].out_value, cmd->fields[0].in_value, cmd->end_state);

	return err;
}

static int mchp_efp6_speed_div(int speed, int *khz)
{
	*khz = speed / HZ_PER_KHZ;

	return ERROR_OK;
}

static int mchp_efp6_khz(int khz, int *jtag_speed)
{
	*jtag_speed = khz * HZ_PER_KHZ;

	return ERROR_OK;
}

static int mchp_efp6_execute_command(struct jtag_command *cmd)
{
	switch (cmd->type) {
	case JTAG_SCAN:
		return mchp_efp6_execute_scan(cmd->cmd.scan);

	case JTAG_RUNTEST:
		return mchp_efp6_execute_runtest(cmd->cmd.runtest);

	case JTAG_RESET:
		return mchp_efp6_execute_reset(cmd->cmd.reset);

	case JTAG_TLR_RESET:
		return mchp_efp6_execute_statemove(cmd->cmd.statemove);

	default:
		break;
	}

	return ERROR_OK;
}

static int mchp_efp6_execute_queue(struct jtag_command *cmd_queue)
{
	struct jtag_command *cmd = cmd_queue;

    while (cmd) {
        if (mchp_efp6_execute_command(cmd))
            return ERROR_COMMAND_CLOSE_CONNECTION;

        cmd = cmd->next;
    }

	return ERROR_OK;
}

static int mchp_efp6_init(void)
{
	int err;

	if (specified_serial_number)
		err = mchp_efp6_enumerate(specific_serial_number);
	else
		err = mchp_efp6_enumerate(0);

	if (err < 0)
		return ERROR_FAIL;

	return mchp_efp6_open();
}

static int mchp_efp6_reset(int trst, int srst)
{
	uint8_t payload_buf[2] = { 0x0, 0x0 };

	if (!trst)
		payload_buf[0] = EFP6_TRSTB_BIT;

	return construct_and_send_packet(SET_JTAG_PINS_OPCODE, payload_buf, ATOMIC_JTAG_OPERATION_PACKET_LENGTH, 0);
}

COMMAND_HANDLER(mchp_efp6_handle_serial_num)
{
	if (CMD_ARGC == 1) {
		memcpy(specific_serial_number, CMD_ARGV[0], 126);
		specified_serial_number = 1;
	}

	return ERROR_OK;
}

static const struct command_registration mchp_efp6_subcommand_handlers[] = {
	{
		.name = "serial",
		.handler = mchp_efp6_handle_serial_num,
		.mode = COMMAND_CONFIG,
		.help = "SERIAL_NUM to locate unique Embedded FP6",
		.usage = "[SERIAL_NUM]",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration mchp_efp6_command_handlers[] = {
	{
		.name = "mchp_efp6",
		.mode = COMMAND_ANY,
		.help = "perform mchp_efp6 management",
		.chain = mchp_efp6_subcommand_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static const char * const mchp_efp6_transports[] = { "jtag", NULL };

static struct jtag_interface mchp_efp6_interface = {
	.supported = 0, /* Don't support DEBUG_CAP_TMS_SEQ */
	.execute_queue = mchp_efp6_execute_queue,
};

struct adapter_driver mchp_efp6_adapter_driver = {
	.name = "microchip-efp6",
	.commands = mchp_efp6_command_handlers,
	.transports = mchp_efp6_transports,
	.init = mchp_efp6_init,
	.quit = mchp_efp6_quit,
	.reset = mchp_efp6_reset,
	.speed = mchp_efp6_speed,
	.khz = mchp_efp6_khz,
	.speed_div = mchp_efp6_speed_div,
	.jtag_ops = &mchp_efp6_interface,
};
