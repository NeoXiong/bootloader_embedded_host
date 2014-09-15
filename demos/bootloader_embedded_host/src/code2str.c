#include "code2str.h"

/* This file keeps all status code return by Response command and Host itself in 
   ASCII string format for debug purpose, don't touch it ! */

/* 0 - 99 */
const char *generic_status_codes[] =
{
	"kStatus_Success",
	"kStatus_Fail",
	"kStatus_ReadOnly",
	"kStatus_OutOfRange",
	"kStatus_InvalidArgument",
	"kStatus_Timeout",
};

/* 100 - 199 */
const char *flash_driver_status_codes[] =
{
	"kStatus_FlashSizeError",
	"kStatus_FlashAlignmentError",
	"kStatus_FlashAddressError",
	"kStatus_FlashAccessError",
	"kStatus_FlashProtectionViolation",
	"kStatus_FlashCommandFailure",
	"kStatus_FlashUnknownProperty",
	"kStatus_FlashEraseKeyError",
	"kStatus_FlashRegionExecuteOnly",
};

/* 200 - 299 */
const char *i2c_driver_status_codes[] =
{
	"kStatus_I2C_SlaveTxUnderrun",
	"kStatus_I2C_SlaveRxOverrun",
	"kStatus_I2C_AribtrationLost",
};

/* 300 - 399 */
const char *spi_driver_status_codes[] =
{
	"kStatus_I2C_SlaveTxUnderrun",
	"kStatus_I2C_SlaveRxOverrun",
	"kStatus_I2C_AribtrationLost",
};

/* 10000 - 10099 */
const char *bootloader_status_codes[] =
{
	"kStatus_UnknownCommand",
	"kStatus_SecurityViolation",
	"kStatus_AbortDataPhase",
	"kStatus_Ping",
	"kStatus_NoResponse",
	"kStatus_NoResponseExpected",
};

/* 10100 - 10199 */
const char *sbloader_status_codes[] =
{
	"kStatusRomLdrSectionOverrun",
	"kStatusRomLdrSignature",
	"kStatusRomLdrSectionLength",
	"kStatusRomLdrUnencryptedOnly",
	"kStatusRomLdrEOFReached",
	"kStatusRomLdrChecksum",
	"kStatusRomLdrCrc32Error",
	"kStatusRomLdrUnknownCommand",
	"kStatusRomLdrIdNotFound",
	"kStatusRomLdrDataUnderrun",
	"kStatusRomLdrJumpReturned",
	"kStatusRomLdrCallFailed",
	"kStatusRomLdrKeyNotFound",
};

/* 10200 - 10299 */
const char *memory_status_codes[] =
{
	"kStatusMemoryRangeInvalid",
	"kStatusMemoryReadFailed",
	"kStatusMemoryWriteFailed",
};

/* 10300 - 10399 */
const char *property_status_codes[] =
{
	"kStatus_UnknownProperty",
	"kStatus_ReadOnlyProperty",
	"kStatus_InvalidPropertyValue",
};

/* 50000 - 50099 */
const char *host_status_codes[] =
{
	"N/A",
	"kStatus_hLinkCorrupt",
	"kStatus_hExpectedResp",
	"kStatus_hUnexpectedChar",
	"kStatus_hDataAbort",
	"kStatus_hInvalidParam",
	"kStatus_hCommTimeOut",
	"kStatus_hCRCError",
};

/* Map code to strings */
const char *code_2_str(uint32_t status_code)
{
	if ((status_code >= GENERIC_STATUS_BEGIN) && (status_code <= GENERIC_STATUS_END))
	{
		return generic_status_codes[status_code - GENERIC_STATUS_BEGIN];
	}
	else if ((status_code >= FLASH_DRIVER_STATUS_BEGIN) && (status_code <= FLASH_DRIVER_STATUS_END))
	{
		return flash_driver_status_codes[status_code - FLASH_DRIVER_STATUS_BEGIN];
	}
	else if ((status_code >= I2C_DRIVER_STATUS_BEGIN) && (status_code <= I2C_DRIVER_STATUS_END))
	{
		return i2c_driver_status_codes[status_code - I2C_DRIVER_STATUS_BEGIN];
	}
	else if ((status_code >= SPI_DRIVER_STATUS_BEGIN) && (status_code <= SPI_DRIVER_STATUS_END))
	{
		return spi_driver_status_codes[status_code - SPI_DRIVER_STATUS_BEGIN];
	}
	else if ((status_code >= BOOTLOADER_STATUS_BEGIN) && (status_code <= BOOTLOADER_STATUS_END))
	{
		return bootloader_status_codes[status_code - BOOTLOADER_STATUS_BEGIN];
	}
	else if ((status_code >= SBLOADER_STATUS_BEGIN) && (status_code <= SBLOADER_STATUS_END))
	{
		return sbloader_status_codes[status_code - SBLOADER_STATUS_BEGIN];
	}
	else if ((status_code >= MEMORY_STATUS_BEGIN) && (status_code <= MEMORY_STATUS_END))
	{
		return memory_status_codes[status_code - MEMORY_STATUS_BEGIN];
	}
	else if ((status_code >= PROPERTY_STATUS_BEGIN) && (status_code <= PROPERTY_STATUS_END))
	{
		return property_status_codes[status_code - PROPERTY_STATUS_BEGIN];
	}
	else if ((status_code >= HOST_STATUS_BEGIN) && (status_code <= HOST_STATUS_END))
	{
		return host_status_codes[status_code - HOST_STATUS_BEGIN];
	}
	else
	{
		return "Unknown Error code";
	}
}
