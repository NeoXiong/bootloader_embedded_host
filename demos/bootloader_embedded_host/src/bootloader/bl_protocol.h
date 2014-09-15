#ifndef __BL_PROTOCOL_H__
#define __BL_PROTOCOL_H__

#include <stdint.h>

#define MAKE_STATUS(group,code) 	((((group) * 100) + (code)))

#define kStatusGroup_Host			500

enum _generic_status
{
    kStatus_Success         = 0,
    kStatus_hLinkCorrupt    = MAKE_STATUS(kStatusGroup_Host, 1),
    kStatus_hExpectedResp  = MAKE_STATUS(kStatusGroup_Host, 2),
    kStatus_hUnexpectedChar = MAKE_STATUS(kStatusGroup_Host, 3),
    kStatus_hDataAbort      = MAKE_STATUS(kStatusGroup_Host, 4),
    kStatus_hInvalidParam   = MAKE_STATUS(kStatusGroup_Host, 5),
    kStatus_hCommTimeOut    = MAKE_STATUS(kStatusGroup_Host, 6),
    kStatus_hCRCError       = MAKE_STATUS(kStatusGroup_Host, 7),
    kStatus_hRetryFail      = MAKE_STATUS(kStatusGroup_Host, 8),
    kStatus_hUnexpectedPacketType = MAKE_STATUS(kStatusGroup_Host, 9),
    kStatus_hUnexpectedCommand = MAKE_STATUS(kStatusGroup_Host, 10),
};

uint32_t bl_ping(uint8_t *p_version);

/*
	DESCRIPTION:
		perform an erase of one or more sectors of FLASH.
	INPUT:
		addr:	target FLASH memory start address, must be CONFIG_BYTES_ALIGN aligned, it's 4 or 8 bytes depends on device. 
		len:	num of bytes to erase, must be CONFIG_BYTES_ALIGN aligned, it's 4 or 8 bytes depends on device.
	RETURN:
		kStatus_Success:	command succeed.
		kStatus_???:		command failed. Refer to Code2Str.c and 'Kinetis Bootloader Status Error Codes' section for details.
*/
uint32_t bl_flash_erase_region(uint32_t addr, uint32_t len);

/* 
	DESCRIPTION:
		write a specified range of bytes in memory (FLASH or RAM) by provided data source.
	INPUT:
		addr:	target memory start address. 
				For FLASH memory, address must be CONFIG_BYTES_ALIGN aligned, it's 4 or 8 bytes depends on device.
		p_data:	data source provided by user.
		len:	num of bytes to write. 
				For FLASH memory, it will be rounded up to a multiple of 4, trailing bytes will be filled with 0xFF.
	RETURN:
		kStatus_Success:	command succeed.
		kStatus_???:		command failed. Refer to Code2Str.c and 'Kinetis Bootloader Status Error Codes' section for details.
*/
uint32_t bl_write_memory(uint32_t addr, const uint8_t *p_data, uint32_t len);
uint32_t bl_read_memory(uint32_t addr, uint8_t *p_data, uint32_t len);

#endif