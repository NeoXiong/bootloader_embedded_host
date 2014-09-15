#ifndef __BL_PRIVATE_H__
#define __BL_PRIVATE_H__

#include <string.h>
#include <stdint.h>
#include <assert.h>
#include "bl_config.h"
#include "port_me.h"

#define MIN(a, b)   ((a) < (b) ? (a) : (b))

enum _command_tags {
    kCommandTag_GenericResponse             = 0xa0,
    kCommandTag_FlashEraseAll               = 0x01,
    kCommandTag_FlashEraseRegion            = 0x02,
    kCommandTag_ReadMemory                  = 0x03,
    kCommandTag_ReadMemoryResponse          = 0xa3,
    kCommandTag_WriteMemory                 = 0x04,
    kCommandTag_FillMemory                  = 0x05,
    kCommandTag_FlashSecurityDisable        = 0x06,
    kCommandTag_GetProperty                 = 0x07,
    kCommandTag_GetPropertyResponse         = 0xa7,
    kCommandTag_ReceiveSbFile               = 0x08,
    kCommandTag_Execute                     = 0x09,
    kCommandTag_Call                        = 0x0a,
    kCommandTag_Reset                       = 0x0b,
    kCommandTag_SetProperty                 = 0x0c,
    kCommandTag_FlashEraseAllUnsecure       = 0x0d,
    kCommandTag_FlashProgramOnce            = 0x0e,
    kCommandTag_FlashReadOnce               = 0x0f,
    kCommandTag_FlashReadOnceResponse       = 0xaf,
    kCommandTag_FlashReadResource           = 0x10,
    kCommandTag_FlashReadResourceResponse   = 0xb0,

    kFirstCommandTag                    = kCommandTag_FlashEraseAll,

    //! Maximum linearly incrementing command tag value, excluding the response commands.
    kLastCommandTag                     = kCommandTag_FlashReadResource,

    kResponseCommandHighNibbleMask = 0xa0           //!< Mask for the high nibble of a command tag that identifies it as a response command.
};

enum _command_packet_flags
{
    kCommandFlag_HasNoDataPhase = 0,	
    kCommandFlag_HasDataPhase   = 1,
};

//! @brief Serial framing packet constants.
enum _framing_packet_constants
{
    kFramingPacketStartByte         = 0x5a,
    kFramingPacketType_Ack          = 0xa1,
    kFramingPacketType_Nak          = 0xa2,
    kFramingPacketType_AckAbort     = 0xa3,
    kFramingPacketType_Command      = 0xa4,
    kFramingPacketType_Data         = 0xa5,
    kFramingPacketType_Ping         = 0xa6,
    kFramingPacketType_PingResponse = 0xa7
};

#pragma pack(1)
typedef struct
{
    uint8_t start_byte;
    uint8_t packet_type;
} framing_header_t;

typedef struct
{
    framing_header_t header;
} framing_sync_packet_t;

typedef struct
{
    framing_header_t header;
    uint16_t         length;
    uint16_t         crc16;
} framing_payload_packet_t;

typedef struct
{
	framing_payload_packet_t framing_packet;
    uint8_t data_payload[CONFIG_MAX_PAYLOAD];
} data_packet_t;

typedef struct
{
    framing_header_t header;
    uint8_t version[4];
    uint16_t options;
    uint16_t crc16;
} framing_ping_response_t;

typedef struct
{
    uint8_t command_tag;
    uint8_t flags;
    uint8_t reserved;
    uint8_t parameter_count;
} command_header_t;

typedef struct
{
    command_header_t header;
	uint32_t		 args[4];
} command_packet_with_args_t;

typedef struct
{
    framing_payload_packet_t   framing_packet;
    command_packet_with_args_t command_packet;
} command_packet_t;

typedef struct
{
    command_header_t command_packet;
    uint32_t status;
    uint32_t command;
} generic_response_packet_t;

typedef struct
{
    command_header_t command_packet;
    uint32_t status;
    uint32_t byte_count;
} read_memory_response_packet_t;

#pragma pack()

/* 
	DESCRIPTION:
		construct a data packet by provided data.
	INPUT:	
		p_data:	data source provided by user.
		len:	num of bytes to packet
	RETURN:
		NULL:					NULL pointer due to invalid parameters.
		const data_packet_t *:	point to a ready to send data packet.
*/
const data_packet_t *_construct_data_packet(const uint8_t *p_data, uint32_t len);

/* 
	DESCRIPTION:
		construct a command packet without parameter.
	INPUT:
		command_tag:	command tags, refer to 'enum _command_tags' in bl_prviate.h.
		flag:			kCommandFlag_HasDataPhase or kCommandFlag_HasNoDataPhase. 
	RETURN:
		const command_packet_t *:	point to a ready to send command packet.
*/
const command_packet_t *_construct_command_packet_woArg(uint32_t command_tag, uint32_t flag);

/* 
	DESCRIPTION:
		construct a command packet with 1 32-bit width parameters.
	INPUT:
		command_tag:	command tags, refer to 'enum _command_tags' in bl_prviate.h.
		flag:			kCommandFlag_HasDataPhase or kCommandFlag_HasNoDataPhase.
		arg1:			argument 1.
	RETURN:
		const command_packet_t *:	point to a ready to send command packet.
*/
const command_packet_t *_construct_command_packet_w1Arg(uint32_t command_tag, uint32_t flag, uint32_t arg);

/* 
	DESCRIPTION:
		construct a command packet with 2 32-bit width parameters.
	INPUT:
		command_tag:	command tags, refer to 'enum _command_tags' in bl_prviate.h.
		flag:			kCommandFlag_HasDataPhase or kCommandFlag_HasNoDataPhase.
		arg1:			argument 1.
		arg2:			argument 2.
	RETURN:
		const command_packet_t* :	point to a ready to send command packet.
*/
const command_packet_t *_construct_command_packet_w2Args(uint8_t command_tag, uint8_t flag, uint32_t arg1, uint32_t arg2);

/* 
	DESCRIPTION:
		construct a command packet with 3 32-bit width parameters.
	INPUT:
		command_tag:	command tags, refer to 'enum _command_tags' in bl_prviate.h.
		flag:			kCommandFlag_HasDataPhase or kCommandFlag_HasNoDataPhase.
		arg1:			argument 1.
		arg2:			argument 2.
		arg3:			argument 3.
	RETURN:
		const command_packet_t* :	point to a ready to send command packet.
*/
const command_packet_t *_construct_command_packet_w3Args(uint32_t command_tag, uint32_t flag, uint32_t arg1, uint32_t arg2, uint32_t arg3);

/* 
	DESCRIPTION:
		construct a command packet with 4 32-bit width parameters.
	INPUT:
		command_tag:	command tags, refer to 'enum _command_tags' in bl_prviate.h.
		flag:			kCommandFlag_HasDataPhase or kCommandFlag_HasNoDataPhase.
		arg1:			argument 1.
		arg2:			argument 2.
		arg3:			argument 3.
		arg4:			argument 4.
	RETURN:
		const command_packet_t *:	point to a ready to send command packet.
*/
const command_packet_t *_construct_command_packet_w4Args(uint32_t command_tag, uint32_t flag, uint32_t arg1, uint32_t arg2, uint32_t arg3, uint32_t arg4);

uint16_t _crc16_update(uint32_t crc, const uint8_t *p_src, uint32_t len);

uint32_t _send_sync(uint32_t sync);

/* 
	DESCRIPTION:
		send a payload packet (either command or data) and read sync (ACK/NAK/ACKABORT). 
		it handles re-transmission of NAKed packet for CONFIG_MAX_RETRIES times.
	INPUT:
		p_data:	point to a payload packet.
		len:	num of bytes for the payload packet.
	RETURN:
		kStatus_Success:		transfer complete and get ACK signal
		kStatus_hInvalidParam:	Invalid NULL p_data or zero len.
		kStatus_hLinkCorrupt:	comm link corrupt or can't find start_byte|0x5A after CONFIG_RECV_TIMEOUT for UART.
		kStatus_hDataAbort: 	get ACKABORT signal
		kStatus_hRetryFail: 	still failed after retry CONFIG_MAX_RETRIES times for NAKed packet.
		kStatus_hUnexpectedChar:get char other than ACK/NAK/ACKABORT after kFramingPacketStartByte.
		kStatus_hCommTimeOut:	can't find start_byte|0x5A after CONFIG_RECV_TIMEOUT for SPI and I2C.
*/
uint32_t _send_packet_and_read_sync(const uint8_t *p_data, uint32_t len);

/* 
	DESCRIPTION:
		read a payload packet (either command or data) and send sync (ACK/NAK/ACKABORT). 
		it handles re-reception of CRC corrupted packet for CONFIG_MAX_RETRIES times.
	INPUT:
		p_p_packet:				point to a pointer, which point to payload packet
		p_packet_len:			point to num of bytes of the payload packet.
		expected_packet_type:	expected payload packet type, kFramingPacketType_Command or kFramingPacketType_Data
	RETURN:
		kStatus_Success:				transfer complete and get ACK signal
		kStatus_hInvalidParam:			Invalid NULL p_p_packet or p_packet_len.
		kStatus_hLinkCorrupt			comm link corrupt or can't find start_byte|0x5A after CONFIG_RECV_TIMEOUT for UART.
		kStatus_hRetryFail: 			still failed after retry CONFIG_MAX_RETRIES times for NAKed packet.
		kStatus_hUnexpectedPacketType: 	get expected payload packet type.
		kStatus_hCommTimeOut			can't find start_byte|0x5A after CONFIG_RECV_TIMEOUT for SPI and I2C.
*/
uint32_t _read_packet_and_send_sync(uint8_t **p_p_packet, uint32_t *p_packet_len, uint32_t expected_packet_type);

/* 
	DESCRIPTION:
		Handle general response and return status to Application layer. 
	INPUT:
		p_generic_response:		point to a generic response packet without framing header.
		expected_command_tag:	expected command tag.
	RETURN:
		kStatus_Success:			Response indicate no error.
		kStatus_hInvalidParam:		Invalid NULL input. 
		kStatus_hUnexpectedCommand:	get a command other than kCommandTag_GenericResponse.
		kStatus_hExpectedResp:		the command tag in generic response packet is not expected one.
		kStatus_???:				other status codes by target. Refer to Code2Str.c and 'Kinetis Bootloader Status Error Codes' section for details.
*/
uint32_t _handle_general_response(const generic_response_packet_t *p_generic_response, uint32_t expected_command_tag);

#endif
