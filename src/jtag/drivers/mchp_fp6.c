// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2024 by Microchip Inc.                                  *
 *   matteo.bonicolini@microchip.com                                       *
 ***************************************************************************/

#include <jtag/interface.h>
#include <libusb.h>
#include "libusb_helper.h"

#include "helper/log.h"
#include "helper/time_support.h"
#include "mchp_fp6.h"

/* Compatibility define for older libusb-1.0 */
#ifndef LIBUSB_CALL
#define LIBUSB_CALL
#endif

struct fp6_ctx {
	struct libusb_context *usb_ctx;
	struct libusb_device_handle *usb_dev;
	uint32_t usb_write_timeout;
	uint32_t usb_read_timeout;
	uint8_t in_ep;
	uint8_t out_ep;
	uint16_t max_packet_size;
	uint16_t index;
	uint16_t value;
	uint8_t interface;
	uint8_t *write_buffer;
	uint32_t write_size;
	uint32_t write_count;
	uint8_t *read_buffer;
	uint32_t read_size;
	uint32_t read_count;
	uint8_t *read_chunk;
	uint32_t read_chunk_size;
	struct bit_copy_queue read_queue;
	int retval;
};

static struct fp6_ctx *fp6_ctx;
static char serial_num[126];
static bool select_serial_num = 0;

static int mchp_fp6_quit(void)
{
	int err;

	err = libusb_release_interface(fp6_ctx->usb_dev, fp6_ctx->interface);
	if (err)
		return err;

	libusb_close(fp6_ctx->usb_dev);
	libusb_exit(fp6_ctx->usb_ctx);

	return ERROR_OK;
}

/* Returns true if the string descriptor indexed by str_index in device matches string */
static bool string_descriptor_equal(struct libusb_device_handle *device, uint8_t str_index,
	const char *string)
{
	int retval;
	char desc_string[256]; /* Max size of string descriptor */

	retval = libusb_get_string_descriptor_ascii(device, str_index, (uint8_t *)desc_string,
			sizeof(desc_string));
	if (retval < 0) {
		LOG_ERROR("libusb_get_string_descriptor_ascii() failed with %s", libusb_error_name(retval));
		return false;
	}

	return strncmp(string, desc_string, sizeof(desc_string)) == 0;
}

static bool device_location_equal(struct libusb_device *device, const char *location)
{
	bool result = false;
#ifdef HAVE_LIBUSB_GET_PORT_NUMBERS
	char *loc = strdup(location);
	uint8_t port_path[7];
	int path_step, path_len;
	uint8_t dev_bus = libusb_get_bus_number(device);
	char const *ptr;

	path_len = libusb_get_port_numbers(device, port_path, 7);
	if (path_len == LIBUSB_ERROR_OVERFLOW) {
		LOG_ERROR("cannot determine path to usb device! (more than 7 ports in path)");
		goto done;
	}

	ptr = strtok(loc, "-:");
	if (!ptr) {
		LOG_ERROR("no ':' in path");
		goto done;
	}
	if (atoi(ptr) != dev_bus) {
		LOG_ERROR("bus mismatch");
		goto done;
	}

	path_step = 0;
	while (path_step < 7) {
		ptr = strtok(NULL, ".,");
		if (!ptr) {
			LOG_ERROR("no more tokens in path at step %i", path_step);
			break;
		}

		if (path_step < path_len && atoi(ptr) != port_path[path_step]) {
			LOG_ERROR("path mismatch at step %i", path_step);
			break;
		}

		path_step++;
	}

	/* walked the full path, all elements match */
	if (path_step == path_len)
		result = true;

done:
	free(loc);
#endif
	return result;
}

static void add_usb_packet_header(struct fp6_ctx *ctx, uint16_t start_code, uint16_t packet_type, uint32_t target_address, uint32_t packet_length, uint16_t packet_crc)
{
	int offset = ctx->write_count;

	ctx->write_buffer[offset + 0] = start_code;
	ctx->write_buffer[offset + 1] = start_code >> 8;

	ctx->write_buffer[offset + 2] = packet_type;
	ctx->write_buffer[offset + 3] = packet_type >> 8;

	ctx->write_buffer[offset + 4] = target_address >> 16;
	ctx->write_buffer[offset + 5] = target_address >> 24;
	ctx->write_buffer[offset + 6] = target_address;
	ctx->write_buffer[offset + 7] = target_address >> 8;

	ctx->write_buffer[offset + 8] = packet_length >> 16;
	ctx->write_buffer[offset + 9] = packet_length >> 24;
	ctx->write_buffer[offset + 10] = packet_length;
	ctx->write_buffer[offset + 11] = packet_length >> 8;

	ctx->write_buffer[offset + packet_length + COMMAND_PACKET_LENGTH - 2] = packet_crc;
	ctx->write_buffer[offset + packet_length + COMMAND_PACKET_LENGTH - 1] = packet_crc >> 8;
}

/* Context needed by the write and read callbacks */
struct transfer_result {
	struct fp6_ctx *ctx;
	bool done;
	uint32_t transferred;
};

static LIBUSB_CALL void read_cb(struct libusb_transfer *transfer)
{
	struct transfer_result *res = transfer->user_data;
	struct fp6_ctx *ctx = res->ctx;
	uint32_t packet_size = ctx->max_packet_size;
	/*
	 * Strip the two status bytes sent at the beginning of each USB packet
	 * while copying the chunk buffer to the read buffer
	 */
	uint32_t num_packets = DIV_ROUND_UP(transfer->actual_length, packet_size);
	uint32_t chunk_remains = transfer->actual_length;

	for (uint32_t i = 0; i < num_packets && chunk_remains > 2; i++) {
		uint32_t this_size = packet_size - 2;

		if (this_size > chunk_remains - 2)
			this_size = chunk_remains - 2;
		if (this_size > ctx->read_count - res->transferred)
			this_size = ctx->read_count - res->transferred;
		memcpy(ctx->read_buffer + res->transferred,
			ctx->read_chunk + packet_size * i + 2,
			this_size);
		res->transferred += this_size;
		chunk_remains -= this_size + 2;
		if (res->transferred == ctx->read_count) {
			res->done = true;
			break;
		}
	}

	if (!res->done)
		if (libusb_submit_transfer(transfer) != LIBUSB_SUCCESS)
			res->done = true;
}

static LIBUSB_CALL void write_cb(struct libusb_transfer *transfer)
{
	struct transfer_result *res = transfer->user_data;
	struct fp6_ctx *ctx = res->ctx;

	res->transferred += transfer->actual_length;

	if (res->transferred == ctx->write_count) {
		res->done = true;
	} else {
		transfer->length = ctx->write_count - res->transferred;
		transfer->buffer = ctx->write_buffer + res->transferred;
		if (libusb_submit_transfer(transfer) != LIBUSB_SUCCESS)
			res->done = true;
	}
}

static void purge_usb_buffer(struct fp6_ctx *ctx)
{
	int err = ERROR_OK;

	LOG_DEBUG("-");
	ctx->write_count = 0;
	ctx->read_count = 0;
	ctx->retval = ERROR_OK;
	bit_copy_discard(&ctx->read_queue);

	LOG_WARNING("unable to purge fp6 rx buffers: %s", libusb_error_name(err));
	LOG_WARNING("unable to purge fp6 tx buffers: %s", libusb_error_name(err));
}

static void buffer_write_byte(struct fp6_ctx *ctx, uint8_t data)
{
	assert(ctx->write_count < ctx->write_size);
	ctx->write_buffer[ctx->write_count++] = data;
}

static int flush_usb_buffer(struct fp6_ctx *ctx)
{
	int retval = ctx->retval;

	if (retval != ERROR_OK) {
		LOG_ERROR("Ignoring flush due to previous error");
		assert(ctx->write_count == 0 && ctx->read_count == 0);
		ctx->retval = ERROR_OK;
		return retval;
	}

	assert(ctx->write_count > 0 || ctx->read_count == 0); /* No read data without write data */

	if (ctx->write_count == 0)
		return retval;

	struct libusb_transfer *read_transfer = NULL;
	struct transfer_result read_result = { .ctx = ctx, .done = true };

	if (ctx->read_count) {
		buffer_write_byte(ctx, 0x87); /* SEND_IMMEDIATE */
		read_result.done = false;
		/*
		 * delay read transaction to ensure the FP6 chip can support us with data
		 * immediately after processing the commands in the write transaction
		 */
	}

	struct transfer_result write_result = { .ctx = ctx, .done = false };
	struct libusb_transfer *write_transfer = libusb_alloc_transfer(0);

	libusb_fill_bulk_transfer(write_transfer, ctx->usb_dev, ctx->out_ep, ctx->write_buffer,
		ctx->write_count, write_cb, &write_result, ctx->usb_write_timeout);
	retval = libusb_submit_transfer(write_transfer);
	if (retval != LIBUSB_SUCCESS)
		goto error_check;

	if (ctx->read_count) {
		read_transfer = libusb_alloc_transfer(0);
		libusb_fill_bulk_transfer(read_transfer, ctx->usb_dev, ctx->in_ep, ctx->read_chunk,
			ctx->read_chunk_size, read_cb, &read_result,
			ctx->usb_read_timeout);
		retval = libusb_submit_transfer(read_transfer);
		if (retval != LIBUSB_SUCCESS)
			goto error_check;
	}

	/* Polling loop, more or less taken from libftdi */
	int64_t start = timeval_ms();
	int64_t warn_after = 2000;

	while (!write_result.done || !read_result.done) {
		struct timeval timeout_usb;

		timeout_usb.tv_sec = 1;
		timeout_usb.tv_usec = 0;

		retval = libusb_handle_events_timeout_completed(ctx->usb_ctx, &timeout_usb, NULL);
		keep_alive();
		if (retval == LIBUSB_ERROR_NO_DEVICE || retval == LIBUSB_ERROR_INTERRUPTED)
			break;

		if (retval != LIBUSB_SUCCESS) {
			libusb_cancel_transfer(write_transfer);
			if (read_transfer)
				libusb_cancel_transfer(read_transfer);
			while (!write_result.done || !read_result.done) {
				retval = libusb_handle_events_timeout_completed(ctx->usb_ctx,
								&timeout_usb, NULL);
				if (retval != LIBUSB_SUCCESS)
					break;
			}
		}

		int64_t now = timeval_ms();

		if (now - start > warn_after) {
			LOG_WARNING("Haven't made progress in flush_usb_buffer() for %" PRId64
					"ms.", now - start);
			warn_after *= 2;
		}
	}

error_check:
	if (retval != LIBUSB_SUCCESS) {
		LOG_ERROR("libusb_handle_events() failed with %s", libusb_error_name(retval));
		retval = ERROR_FAIL;
	} else if (write_result.transferred < ctx->write_count) {
		LOG_ERROR("fp6 device did not accept all data: %u, tried %u",
			write_result.transferred,
			ctx->write_count);
		retval = ERROR_FAIL;
	} else if (read_result.transferred < ctx->read_count) {
		LOG_ERROR("fp6 device did not return all data: %u, expected %u",
			read_result.transferred,
			ctx->read_count);
		retval = ERROR_FAIL;
	} else if (ctx->read_count) {
		ctx->write_count = 0;
		ctx->read_count = 0;
		bit_copy_execute(&ctx->read_queue);
		retval = ERROR_OK;
	} else {
		ctx->write_count = 0;
		bit_copy_discard(&ctx->read_queue);
		retval = ERROR_OK;
	}

	if (retval != ERROR_OK)
		purge_usb_buffer(ctx);

	libusb_free_transfer(write_transfer);
	if (read_transfer)
		libusb_free_transfer(read_transfer);

	return retval;
}

static int add_packet_to_usb_buffer(struct fp6_ctx *ctx, uint16_t opcode, uint8_t const *buf, uint32_t buf_len, uint32_t pad_len, uint32_t target_address)
{
	int err = ERROR_OK;

	if ((COMMAND_PACKET_LENGTH + buf_len + pad_len) > ctx->write_size) {
		LOG_ERROR("Error: Requested data size is larger than USB buffer");
		return ERROR_FAIL;
	}

	if ((ctx->write_count + COMMAND_PACKET_LENGTH + buf_len + pad_len) > ctx->write_size) {
		err = flush_usb_buffer(ctx);
		if (err)
			return err;
	}

	if (buf)
		memcpy(&ctx->write_buffer[ctx->write_count + PAYLOAD_PACKET_START_ADDRESS], buf, buf_len);
	else
		memset(&ctx->write_buffer[ctx->write_count + PAYLOAD_PACKET_START_ADDRESS], 0, buf_len);

	memset(&ctx->write_buffer[ctx->write_count + PAYLOAD_PACKET_START_ADDRESS + buf_len], 0, pad_len);

	add_usb_packet_header(ctx, FP_PACKET_START_CODE, opcode, target_address, buf_len + pad_len, 0xDEAD);

	ctx->write_count += COMMAND_PACKET_LENGTH + buf_len + pad_len;

	return err;
}

__attribute__((always_inline)) inline
static int construct_and_send_packet(struct fp6_ctx *ctx, uint16_t opcode, uint8_t const *buf, uint32_t buf_len, uint32_t target_address)
{
	int err;

	err = add_packet_to_usb_buffer(ctx, opcode, buf, buf_len, 0, target_address);
	if (err)
		return err;

	return flush_usb_buffer(ctx);
}

static int sf2_reset(struct fp6_ctx *ctx)
{
	char resp[1] = { 0 };
	int err;

	err = jtag_libusb_control_transfer(ctx->usb_dev,
				 LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN,
				 FP6_SF2_RESET_OPCODE,
				 ctx->value,
				 ctx->index,
				 resp,
				 sizeof(resp),
				 ctx->usb_write_timeout,
				 NULL);

	if (resp[0] != 0)
		return (int)resp[0];

	return err;
}

static int frequency_interrupt(struct fp6_ctx *ctx)
{
	int err;

	usleep(10000);
	err = construct_and_send_packet(ctx, FREQUENCY_INTERRUPT_OPCODE, NULL, 0, 0);
	usleep(10000);

	return err;
}

static int get_hw_tck_frequency(struct fp6_ctx *ctx, uint8_t *freq_mhz)
{
	uint8_t tck_frq = 0;
	uint8_t read_buffer[PROGRESS_PACKET_LENGTH];
	int err;

	err = construct_and_send_packet(ctx, READ_PROGRESS_OPCODE, NULL, 0, FP_TARGET_FREQUENCY_ADDRESS);
	if (err)
		return err;

	err = libusb_bulk_transfer(ctx->usb_dev, ctx->in_ep, read_buffer, sizeof(read_buffer), 0, ctx->usb_read_timeout);
	if (err)
		return err;

	tck_frq = read_buffer[FP_FREQUENCY_READ_BYTE_LOCATION];
	*freq_mhz = tck_frq;

	return ERROR_OK;
}

static int set_operation_status(struct fp6_ctx *ctx, enum op_status op_status)
{
	char resp[1] = { 0 };
	uint16_t opcode = TURN_ACTIVITY_LED_OFF_OPCODE;
	uint16_t request =  FP6_SET_LED_OFF_VENDOR_OPCODE;
	int err;

	switch (op_status) {
	case OP_OFF:
		opcode = TURN_ACTIVITY_LED_OFF_OPCODE;
		request = FP6_SET_LED_OFF_VENDOR_OPCODE;
		break;

	case OP_PASS:
		opcode = TURN_ACTIVITY_LED_OFF_OPCODE;
		request = FP6_SET_LED_PASS_VENDOR_OPCODE;
		break;

	case OP_FAIL:
		opcode = TURN_ACTIVITY_LED_OFF_OPCODE;
		request = FP6_SET_LED_FAIL_VENDOR_OPCODE;
		break;

	case OP_ACTIVE:
		opcode = TURN_ACTIVITY_LED_ON_OPCODE;
		request = FP6_SET_LED_OFF_VENDOR_OPCODE;
		break;

	default:
		break;
	}

	err =  construct_and_send_packet(ctx, opcode, NULL, 0, 0);
	if (err)
		return err;

	err = jtag_libusb_control_transfer(ctx->usb_dev,
					 LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN,
					 request,
					 ctx->value,
					 ctx->index,
					 resp,
					 sizeof(resp),
					 ctx->usb_write_timeout,
					 NULL);

	if (err)
		return err;

	if (resp[0] != 0)
		return (int)resp[0];

	return ERROR_OK;
}

#define MAX_ATTEMPTS (2)

static int set_tck_frequency(struct fp6_ctx *ctx, const uint32_t hz)
{
	uint8_t payload_buffer[ACTION_PACKET_LENGTH] = { 0 };
	uint8_t mhz;
	uint8_t hw_mhz = 0;
	int err;
	int i;

	mhz = (uint8_t)(hz / 1000000);

	payload_buffer[0] = FP_SET_TCK_FREQUENCY;
	payload_buffer[1] = mhz;

	for (i = 0; i < MAX_ATTEMPTS; i++) {
		err = construct_and_send_packet(ctx, SEND_PACKET_OPCODE, payload_buffer, ACTION_PACKET_LENGTH, FP_TARGET_FREQUENCY_ADDRESS);
		if (err)
			return err;

		err = frequency_interrupt(ctx);
		if (err)
			return err;

		err = get_hw_tck_frequency(ctx, &hw_mhz);
		if (err)
			return err;

		if (hw_mhz != mhz)
			LOG_WARNING("failed to set tck frequency .. retrying");
		else
			break;
	}

	if (hw_mhz != mhz) {
		LOG_ERROR("error: failed to set tck frequency");
		return ERROR_FAIL;
	}

	err = construct_and_send_packet(ctx, ENABLE_JTAG_PORT_OPCODE, NULL, 0, 0);
	if (err)
		return err;

	return set_operation_status(ctx, OP_ACTIVE);
}

static int m3_reset(struct fp6_ctx *ctx, bool state)
{
	/* False is hold in reset */
	if (!state)
		return construct_and_send_packet(ctx, M3_IN_RESET_OPCODE, NULL, 0, 0);
	else
		return construct_and_send_packet(ctx, M3_OUT_OF_RESET_OPCODE, NULL, 0, 0);
}

static int set_trst(struct fp6_ctx *ctx, bool state)
{
	uint8_t payload_buffer[ATOMIC_JTAG_OPERATION_PACKET_LENGTH];
	uint8_t frame[READ_PACKET_LENGTH];
	int err;

	/* read the existing values of JTAG pins before setting trst pin */
	err = construct_and_send_packet(ctx, GET_JTAG_PINS_OPCODE, NULL, 0, 0);
	if (err)
		return err;

	err = libusb_bulk_transfer(ctx->usb_dev, ctx->in_ep, frame, sizeof(frame), 0, ctx->usb_read_timeout);
	if (err)
		return err;

	payload_buffer[0] = frame[8];
	payload_buffer[0] &= ~(FP6_TRSTB_BIT);
	payload_buffer[1] = 0;

	if (state == 1)
		payload_buffer[0] |= FP6_TRSTB_BIT;

	return construct_and_send_packet(ctx, SET_JTAG_PINS_OPCODE, payload_buffer, ATOMIC_JTAG_OPERATION_PACKET_LENGTH, 0);
}

static int set_prog_mode_pin(struct fp6_ctx *ctx, bool state)
{
	char resp[1] = { 0 };
	uint16_t opcode = FP6_SET_PROG_MODE_HIGH_VENDOR_OPCODE;
	int err;

	if (state == 0)
		opcode = FP6_SET_PROG_MODE_LOW_VENDOR_OPCODE;

	err = jtag_libusb_control_transfer(ctx->usb_dev,
							 LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN,
							 opcode,
							 ctx->value,
							 ctx->index,
							 resp,
							 sizeof(resp),
							 ctx->usb_write_timeout,
							 NULL);

	if (err)
		return err;

	if (resp[0] != 0)
		return (int)resp[0];

	return ERROR_OK;
}

static int enable_programming_port(struct fp6_ctx *ctx, bool enable)
{
	if (enable) {
		construct_and_send_packet(ctx, ENABLE_JTAG_PORT_OPCODE, NULL, 0, 0);

		m3_reset(ctx, false);
		m3_reset(ctx, true);

		/* Only set TRST to HIGH when enable == true */
		set_trst(ctx, 1);
		set_prog_mode_pin(ctx, 1);
	} else {
		set_prog_mode_pin(ctx, 0);
		construct_and_send_packet(ctx, DISABLE_JTAG_PORT_OPCODE, NULL, 0, 0);
	}

	return ERROR_OK;
}

static int execute_delay(struct fp6_ctx *ctx, uint32_t ticks)
{
	uint8_t payload[4] = { 0 };

	payload[0] = ticks >> 16;
	payload[1] = ticks >> 24;
	payload[2] = ticks;
	payload[3] = ticks >> 8;

	return add_packet_to_usb_buffer(ctx, INSERT_WAIT_CYCLES_OPCODE, payload, sizeof(payload), 0, 0);
}

static int jtag_delay(struct fp6_ctx *ctx, uint32_t tck, uint32_t ticks)
{
	return execute_delay(ctx, tck + ticks);
}

static int jtag_goto_state(struct fp6_ctx *ctx, uint16_t jtag_state)
{
	uint8_t payload[ATOMIC_JTAG_OPERATION_PACKET_LENGTH] = { 0 };
	int err;

	payload[0] = jtag_state;

	err = add_packet_to_usb_buffer(ctx, JTAG_ATOMIC_OPCODE, payload, sizeof(payload), 0, 0);
	flush_usb_buffer(ctx);

	return err;
}

static inline uint32_t ROUND_UP8(uint32_t num)
{
	num += 7;
	num /= 8;
	num *= 8;

	return num;
}

#define MAX_FRAME_DATA (READ_PACKET_LENGTH - (PAYLOAD_READ_PACKET_START_ADDRESS + PACKET_CRC_BYTE_SIZE))

static int retrieve_data(struct fp6_ctx *ctx, uint8_t *in, uint32_t bitlen)
{
	uint8_t buffer[READ_PACKET_LENGTH];
	uint32_t byte_len;
	uint32_t num_frames;
	int buf_offset = 0;
	int frame_len;
	uint32_t i;
	int err;

	/* convert bit_len to byte_len - round up partial bits to a byte */
	byte_len = (bitlen + 7) / 8;

	/* byte_len is rounded to a multiple of 8 bytes */
	byte_len = ROUND_UP8(byte_len);

	num_frames = (byte_len) / MAX_FRAME_DATA;
	if ((num_frames * MAX_FRAME_DATA) < byte_len)
		num_frames++;

	err = construct_and_send_packet(ctx, READ_TDO_BUFFER_COMMAND_OPCODE, NULL, 0, 0);
	if (err)
		return err;

	for (i = 0; i < num_frames; i++) {
		err = libusb_bulk_transfer(ctx->usb_dev, ctx->in_ep, buffer, sizeof(buffer), 0, ctx->usb_read_timeout);
		if (err)
			return err;

		frame_len = byte_len > MAX_FRAME_DATA ? MAX_FRAME_DATA : byte_len;

		memcpy(in, &buffer[PAYLOAD_READ_PACKET_START_ADDRESS + buf_offset], frame_len);
		buf_offset += frame_len;
		byte_len -= frame_len;
	}

	return ERROR_OK;
}

static int jtag_common_scan(struct fp6_ctx *ctx, int bitlen, uint8_t const *out, uint8_t *in, bool is_ir)
{
	uint8_t payload[ATOMIC_JTAG_OPERATION_PACKET_LENGTH];
	uint32_t bytes_to_transmit;
	uint32_t padding_bytes = 0;
	uint8_t *dummy = NULL;
	int err;

	if (bitlen > 8064) {
		LOG_ERROR("Error: Number of bits to shifted exceeded supported maximum of 8064 bits");
		return ERROR_FAIL;
	}

	jtag_goto_state(ctx, TAP_IDLE);

	payload[0] = bitlen & 0xff;
	payload[1] = bitlen >> 8;

	err = add_packet_to_usb_buffer(ctx, SET_SHIFT_IR_DR_BIT_LENGTH_OPCODE, payload, sizeof(payload), 0, 0);
	if (err)
		return err;

	if (!out) {
		bytes_to_transmit = (uint32_t)((bitlen + 7) / 8);
		dummy = calloc(bytes_to_transmit, 1);
		if (!dummy)
			return ERROR_FAIL;
		out = dummy;
	}

	if (bitlen > 16) {
		bytes_to_transmit = (uint32_t)((bitlen + 7) / 8);
		if (bytes_to_transmit % 16)
			padding_bytes = 16 - (bytes_to_transmit % 16);
		add_packet_to_usb_buffer(ctx, SHIFT_DATA_FROM_DDR_OPCODE, out, bytes_to_transmit, padding_bytes, 0);
	} else {
		add_packet_to_usb_buffer(ctx, SHIFT_DATA_FROM_REGISTER_OPCODE, out, 2, 0, 0);
	}

	if (is_ir) {
		jtag_goto_state(ctx, TAP_IRSHIFT);
		jtag_goto_state(ctx, TAP_IRPAUSE);
	} else {
		jtag_goto_state(ctx, TAP_DRSHIFT);
		jtag_goto_state(ctx, TAP_DRPAUSE);
	}

	if (in)
		retrieve_data(ctx, in, bitlen);

	if (dummy)
		free(dummy);

	return err;
}

static int jtag_ir_scan(struct fp6_ctx *ctx, int bitlen, uint8_t const *out, uint8_t *in)
{
	return jtag_common_scan(ctx, bitlen, out, in, 1);
}

static int jtag_dr_scan(struct fp6_ctx *ctx, int bitlen, uint8_t const *out, uint8_t *in)
{
	return jtag_common_scan(ctx, bitlen, out, in, 0);
}

int fp6_get_serial_number(struct fp6_ctx *ctx, char *desc_serial, size_t buflen)
{
	uint8_t buffer[4] = { 0 };
	int err;

	err = jtag_libusb_control_transfer(ctx->usb_dev,
				 LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN,
				 FP6_GET_PROGRAMMER_SERIAL_NUMBER_VENDOR_OPCODE,
				 ctx->value,
				 ctx->index,
				 (char *)buffer,
				 sizeof(buffer),
				 ctx->usb_read_timeout,
				 NULL);
	if (err)
		return err;

	snprintf(desc_serial, buflen, "%02X%02X%02X%02X", buffer[3], buffer[2], buffer[1], buffer[0]);

	return ERROR_OK;
}

/*
 * Helper to open a libusb device that matches vid, pid, product string and/or serial string.
 * Set any field to 0 as a wildcard. If the device is found true is returned, with ctx containing
 * the already opened handle. ctx->interface must be set to the desired interface (channel) number
 * prior to calling this function
 */
static bool open_matching_device(struct fp6_ctx *ctx, const uint16_t vids[], const uint16_t pids[],
	const char *product, const char *serial, const char *location)
{
	struct libusb_device **list;
	struct libusb_device_descriptor desc;
	struct libusb_config_descriptor *config0;
	int err;
	bool found = false;
	ssize_t cnt = libusb_get_device_list(ctx->usb_ctx, &list);
	char desc_man[32] = { 0 }; /* Max size of FP6 descriptor */
	char desc_prod[16] = { 0 }; /* Max size of FP6 descriptor */
	char desc_serial[9] = { 0 };

	if (cnt < 0)
		LOG_ERROR("libusb_get_device_list() failed with %s", libusb_error_name(cnt));

	for (ssize_t i = 0; i < cnt; i++) {
		struct libusb_device *device = list[i];

		err = libusb_get_device_descriptor(device, &desc);
		if (err != LIBUSB_SUCCESS) {
			LOG_ERROR("libusb_get_device_descriptor() failed with %s", libusb_error_name(err));
			continue;
		}

		if (!jtag_libusb_match_ids(&desc, vids, pids))
			continue;

		err = libusb_open(device, &ctx->usb_dev);
		if (err != LIBUSB_SUCCESS) {
			LOG_ERROR("libusb_open() failed with %s",
				  libusb_error_name(err));
			continue;
		}

		err = fp6_get_serial_number(ctx, desc_serial, sizeof(desc_serial));
		if (err) {
			LOG_ERROR("failed to get serial number");
			return false;
		}

		if (location && !device_location_equal(device, location)) {
			libusb_close(ctx->usb_dev);
			continue;
		}

		if (product && !string_descriptor_equal(ctx->usb_dev, desc.iProduct, product)) {
			libusb_close(ctx->usb_dev);
			continue;
		}

		if (serial && strncmp(desc_serial, serial, 126)) {
			libusb_close(ctx->usb_dev);
			continue;
		}

		err = libusb_get_string_descriptor_ascii(ctx->usb_dev, desc.iProduct, (uint8_t *)desc_prod,
							sizeof(desc_prod));
		if (err < 0) {
			LOG_ERROR("libusb_get_string_descriptor_ascii() failed with %s", libusb_error_name(err));
			return false;
		}

		err = libusb_get_string_descriptor_ascii(ctx->usb_dev, desc.iManufacturer, (uint8_t *)desc_man,
							sizeof(desc_man));
		if (err < 0) {
			LOG_ERROR("libusb_get_string_descriptor_ascii() failed with %s", libusb_error_name(err));
			return false;
		}

		LOG_INFO("%s %s %s", desc_man, desc_prod, desc_serial);

		found = true;
		break;
	}

	libusb_free_device_list(list, 1);

	if (!found) {
		/* The caller reports detailed error desc */
		return false;
	}

	err = libusb_get_config_descriptor(libusb_get_device(ctx->usb_dev), 0, &config0);
	if (err != LIBUSB_SUCCESS) {
		LOG_ERROR("libusb_get_config_descriptor() failed with %s", libusb_error_name(err));
		libusb_close(ctx->usb_dev);
		return false;
	}

	/* Make sure the first configuration is selected */
	int cfg;

	err = libusb_get_configuration(ctx->usb_dev, &cfg);
	if (err != LIBUSB_SUCCESS) {
		LOG_ERROR("libusb_get_configuration() failed with %s", libusb_error_name(err));
		goto error;
	}

	if (desc.bNumConfigurations > 0 && cfg != config0->bConfigurationValue) {
		err = libusb_set_configuration(ctx->usb_dev, config0->bConfigurationValue);
		if (err != LIBUSB_SUCCESS) {
			LOG_ERROR("libusb_set_configuration() failed with %s", libusb_error_name(err));
			goto error;
		}
	}

	err = libusb_claim_interface(ctx->usb_dev, ctx->interface);
	if (err != LIBUSB_SUCCESS) {
		LOG_ERROR("libusb_claim_interface() failed with %s", libusb_error_name(err));
		goto error;
	}

	/* Determine maximum packet size and endpoint addresses */
	if (!(desc.bNumConfigurations > 0 && ctx->interface < config0->bNumInterfaces &&
	   config0->interface[ctx->interface].num_altsetting > 0))
		goto desc_error;

	const struct libusb_interface_descriptor *descriptor;

	descriptor = &config0->interface[ctx->interface].altsetting[0];
	if (descriptor->bNumEndpoints != 2)
		goto desc_error;

	ctx->in_ep = 0;
	ctx->out_ep = 0;
	for (int i = 0; i < descriptor->bNumEndpoints; i++) {
		if (descriptor->endpoint[i].bEndpointAddress & 0x80) {
			ctx->in_ep = descriptor->endpoint[i].bEndpointAddress;
			ctx->max_packet_size =
					descriptor->endpoint[i].wMaxPacketSize;
		} else {
			ctx->out_ep = descriptor->endpoint[i].bEndpointAddress;
		}
	}

	if (ctx->in_ep == 0 || ctx->out_ep == 0)
		goto desc_error;

	libusb_free_config_descriptor(config0);
	return true;

desc_error:
	LOG_ERROR("unrecognized USB device descriptor");
error:
	libusb_free_config_descriptor(config0);
	libusb_close(ctx->usb_dev);
	return false;
}

static int mchp_fp6_init(void)
{
	const uint16_t vids[1] = { FP6_VID };
	const uint16_t pids[1] = { FP6_PID };
	struct fp6_ctx *ctx = calloc(1, sizeof(*ctx));
	char const *desc = NULL;
	char const *ser = NULL;
	char const *loc = NULL;
	int err;

	if (!ctx)
		return ERROR_FAIL;

	if (select_serial_num)
		ser = serial_num;

	bit_copy_queue_init(&ctx->read_queue);
	ctx->read_chunk_size = FP_MAX_USB_BUFFER_BYTE_SIZE;
	ctx->read_size = FP_MAX_USB_BUFFER_BYTE_SIZE;
	ctx->write_size = FP_MAX_USB_BUFFER_BYTE_SIZE;
	ctx->read_chunk = malloc(ctx->read_chunk_size);
	ctx->read_buffer = malloc(ctx->read_size);

	ctx->write_buffer = calloc(1, ctx->write_size);

	if (!ctx->read_chunk || !ctx->read_buffer || !ctx->write_buffer)
		goto error;

	ctx->interface = 0;
	ctx->index = 0xfff0;
	ctx->value = 3;
	ctx->usb_read_timeout = FP_READ_TIMEOUT;
	ctx->usb_write_timeout = FP_READ_TIMEOUT;

	err = libusb_init(&ctx->usb_ctx);
	if (err != LIBUSB_SUCCESS) {
		LOG_ERROR("libusb_init() failed with %s", libusb_error_name(err));
		goto error;
	}

	if (!open_matching_device(ctx, vids, pids, desc, ser, loc)) {
		LOG_ERROR("unable to open device with description '%s', "
				  "serial '%s' at bus location '%s'",
				  desc ? desc : "*",
				  ser ? ser : "*",
				  loc ? loc : "*");
		ctx->usb_dev = NULL;
		goto error;
	}

	fp6_ctx = ctx;

	sf2_reset(ctx);

	enable_programming_port(ctx, 1);

	return ERROR_OK;
error:
	return ERROR_FAIL;
}

static int mchp_fp6_execute_statemove(struct fp6_ctx *ctx, struct statemove_command const *cmd)
{
	return jtag_goto_state(ctx, cmd->end_state);
}

static int mchp_fp6_execute_runtest(struct fp6_ctx *ctx, struct runtest_command const *cmd)
{
	int err;

	err = jtag_goto_state(ctx, TAP_IDLE);
	if (err)
		return err;

	err = jtag_delay(ctx, cmd->num_cycles, 0);
	if (err)
		return err;

	err = jtag_goto_state(ctx, cmd->end_state);
	if (err)
		return err;

	return ERROR_OK;
}

static int mchp_fp6_execute_reset(struct fp6_ctx *ctx, struct reset_command const *cmd)
{
	return jtag_goto_state(ctx, TAP_RESET);
}

static int mchp_fp6_ir_scan(struct fp6_ctx *ctx, uint32_t bitlen, uint8_t const *out, uint8_t *in, uint16_t end_state)
{
	return jtag_ir_scan(ctx, bitlen, out, in);
}

static int mchp_fp6_dr_scan(struct fp6_ctx *ctx, uint32_t bitlen, uint8_t const *out, uint8_t *in, uint16_t end_state)
{
	return jtag_dr_scan(ctx, bitlen, out, in);
}

static int mchp_fp6_execute_scan(struct fp6_ctx *ctx, struct scan_command const *cmd)
{
	int error;

	/*
	 * Experimentally noted that rarely the end state is something else than IDLE
	 * Then num_fields is 1 even when the base structure support bigger scans
	 */
	if (cmd->num_fields > 1) {
		LOG_ERROR("Execute scan is implemented only with one field but this request has %d fields\r", cmd->num_fields);
		return ERROR_FAIL;
	}

	if (cmd->ir_scan)
		error = mchp_fp6_ir_scan(ctx, cmd->fields[0].num_bits, (uint8_t *)(cmd->fields[0].out_value), cmd->fields[0].in_value, cmd->end_state);
	else
		error = mchp_fp6_dr_scan(ctx, cmd->fields[0].num_bits, (uint8_t *)(cmd->fields[0].out_value), cmd->fields[0].in_value, cmd->end_state);

	return error;
}

static int mchp_fp6_speed_div(int speed, int *khz)
{
	uint32_t actual_speed = 0;
	int err;

	err = get_hw_tck_frequency(fp6_ctx, (uint8_t *)&actual_speed);
	if (err)
		return err;

	/* hw_tck_freq is in MHz */
	*khz = actual_speed * HZ_PER_KHZ;

	return ERROR_OK;
}

static int mchp_fp6_khz(int khz, int *jtag_speed)
{
	*jtag_speed = khz * HZ_PER_KHZ;

	return ERROR_OK;
}

static int mchp_fp6_execute_command(struct fp6_ctx *ctx, struct jtag_command const *cmd)
{
	switch (cmd->type) {
	case JTAG_SCAN:
		return mchp_fp6_execute_scan(ctx, cmd->cmd.scan);
		break;

	case JTAG_RUNTEST:
		return mchp_fp6_execute_runtest(ctx, cmd->cmd.runtest);
		break;

	case JTAG_RESET:
		return mchp_fp6_execute_reset(ctx, cmd->cmd.reset);
		break;

	case JTAG_TLR_RESET:
		return mchp_fp6_execute_statemove(ctx, cmd->cmd.statemove);
		break;

	default:
		return ERROR_FAIL;
		break;
	}

	return ERROR_FAIL;
}

static int mchp_fp6_execute_queue(struct jtag_command *cmd_queue)
{
    struct jtag_command *cmd = cmd_queue;

    while (cmd) {
        if (mchp_fp6_execute_command(fp6_ctx, cmd))
            return ERROR_COMMAND_CLOSE_CONNECTION;

        cmd = cmd->next;
    }

    return ERROR_OK;
}

static int mchp_fp6_reset(int trst, int srst)
{
	int err;

	if (trst == 0)
		err = set_trst(fp6_ctx, 1);
	else
		err = set_trst(fp6_ctx, 0);

	return err;
}

static int mchp_fp6_speed(int32_t tck_freq)
{
	return set_tck_frequency(fp6_ctx, tck_freq);
}

COMMAND_HANDLER(mchp_fp6_handle_serial_num)
{
	if (CMD_ARGC == 1) {
		memcpy(serial_num, CMD_ARGV[0], 126);
		select_serial_num = 1;
	}

	return ERROR_OK;
}

static const struct command_registration mchp_fp6_subcommand_handlers[] = {
	{
		.name = "serial",
		.handler = mchp_fp6_handle_serial_num,
		.mode = COMMAND_CONFIG,
		.help = "SERIAL_NUM to locate unique FP6",
		.usage = "[SERIAL_NUM]",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration mchp_fp6_command_handlers[] = {
	{
		.name = "mchp_fp6",
		.mode = COMMAND_ANY,
		.help = "perform mchp_fp6 management",
		.chain = mchp_fp6_subcommand_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static const char * const mchp_fp6_transports[] = { "jtag", NULL };

static struct jtag_interface mchp_fp6_interface = {
	.supported = 0, /* Don't support DEBUG_CAP_TMS_SEQ */
	.execute_queue = mchp_fp6_execute_queue,
};

struct adapter_driver mchp_fp6_adapter_driver = {
	.name = "microchip-fp6",
	.commands = mchp_fp6_command_handlers,
	.transports = mchp_fp6_transports,
	.init = mchp_fp6_init,
	.quit = mchp_fp6_quit,
	.reset = mchp_fp6_reset,
	.speed = mchp_fp6_speed,
	.khz = mchp_fp6_khz,
	.speed_div = mchp_fp6_speed_div,
	.jtag_ops = &mchp_fp6_interface,
};
