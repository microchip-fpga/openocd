#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdint.h>
#include "imp.h"
#include "flash/nor/core.h"
#include <helper/binarybuffer.h>
#include "target/riscv/riscv.h"
#include <target/algorithm.h>
#include <target/image.h>


#define LOADER_BASE_ADDRESS 0x8000000
#define PAYLOAD_SIZE_ADDR 0x8020000
#define PAYLOAD_WORKING_AREA 0x8021000
#define ENVM_BASE_ADDR  0x20220000U
#define ENVM_SIZE  0x20000U

struct mchp_flash_bank {
	const char *family_name;
	bool probed;
	uint32_t addr;
	uint32_t algo_size;
	const uint8_t * algo_code;
	uint32_t envm_size;
};

static const uint8_t mchp_algo[] = {
#include "../../../contrib/loaders/flash/mchp_flash/loader_algo.inc"
};

static int mchp_riscv_flash_auto_probe(struct flash_bank *bank);

static int mchp_riscv_flash_probe(struct flash_bank *bank)
{
	struct mchp_flash_bank *mchp_flash_bank = bank->driver_priv;

	int retval = ERROR_OK;

	if (!mchp_flash_bank->probed)
		retval = mchp_riscv_flash_auto_probe(bank);

	return retval;
}

static int mchp_riscv_flash_auto_probe(struct flash_bank *bank)
{
	struct mchp_flash_bank *mchp_flash_bank = bank->driver_priv;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if(!mchp_flash_bank->probed) {
		mchp_flash_bank->probed = true;
	}

	mchp_flash_bank->probed = true;

	return ERROR_OK;
}

static int mchp_riscv_flash_erase(struct flash_bank *bank,
									uint32_t first, uint32_t last)
{
	LOG_INFO("Not implemented");
	return ERROR_OK;
}

static int mchp_riscv_flash_write(struct flash_bank *bank,
				const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t retval;

	retval = target_write_buffer(target, LOADER_BASE_ADDRESS,
				sizeof(mchp_algo), mchp_algo);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write eNVM algorithm to target working area");
		return retval;
	}

	retval = target_write_buffer(target,
			PAYLOAD_WORKING_AREA, count, buffer);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write payload to target working area");
	}

	retval = target_write_u32(target, PAYLOAD_SIZE_ADDR, count);

	if (retval != ERROR_OK) {
		LOG_ERROR("could not write the size into memory");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	retval = target_run_algorithm(target, 0, NULL, 0, NULL,
				LOADER_BASE_ADDRESS, 0,20000, NULL);

	if(retval != ERROR_OK) {
		LOG_ERROR("could not start the loader algorithm");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(mchp_flash_bank_command)
{
	struct flash_bank *t_bank = bank;
	struct mchp_flash_bank *mchp_flash_bank;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	mchp_flash_bank = malloc(sizeof(struct mchp_flash_bank));
	t_bank->driver_priv = mchp_flash_bank;

	if (!mchp_flash_bank) {
		LOG_ERROR("No memory for flash bank chip info");
		return ERROR_FAIL;
	}

	mchp_flash_bank->probed = false;
	mchp_flash_bank->family_name = "Microchip_eNVM";
	mchp_flash_bank->envm_size = ENVM_SIZE;

	t_bank->bank_number = 0;
	t_bank->base = ENVM_BASE_ADDR;
	t_bank->size = ENVM_SIZE;
	t_bank->num_sectors = 4;

	t_bank->sectors = malloc(
					t_bank->num_sectors * sizeof(struct flash_sector));

	t_bank->sectors[0].offset = 0;
	t_bank->sectors[0].size = 0x2000;
	t_bank->sectors[0].is_erased = -1;
	t_bank->sectors[0].is_protected = -1;

	t_bank->sectors[1].offset = 0x2000;
	t_bank->sectors[1].size = 0xE000;
	t_bank->sectors[1].is_erased = -1;
	t_bank->sectors[1].is_protected = -1;

	t_bank->sectors[2].offset = 0x10000;
	t_bank->sectors[2].size = 0xE000;
	t_bank->sectors[2].is_erased = -1;
	t_bank->sectors[2].is_protected = -1;

	t_bank->sectors[3].offset = 0x1E000;
	t_bank->sectors[3].size = 0x2000;
	t_bank->sectors[3].is_erased = -1;
	t_bank->sectors[3].is_protected = -1;

	bank->driver_priv = mchp_flash_bank;

	return ERROR_OK;
}

static int get_mchp_info(struct flash_bank *bank,
							struct command_invocation *cmd)
{
	struct mchp_flash_bank *t_bank = bank->driver_priv;
	command_print_sameline(cmd, "%s device", t_bank->family_name);

	return ERROR_OK;
}

const struct flash_driver mchp_flash = {
	.name = "mchp",
	.usage = "flash bank envm mchp 0x20220000 0x020000 0 0 $_TARGETNAME_0",
	.flash_bank_command = mchp_flash_bank_command,
	.erase = mchp_riscv_flash_erase,
	.write = mchp_riscv_flash_write,
	.read = default_flash_read,
	.probe = mchp_riscv_flash_probe,
	.auto_probe = mchp_riscv_flash_auto_probe,
	.erase_check =  default_flash_blank_check,
	.info = get_mchp_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
