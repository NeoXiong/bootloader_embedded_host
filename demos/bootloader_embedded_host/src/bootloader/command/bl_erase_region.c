/*
	Author:		Neo	(b44513@freescale.com)
	Data:		2014/9/14
	Revision:	v0.1
*/

#include "bl_private.h"
#include "bl_protocol.h"

/* perform an erase of one or more sectors of FLASH. */
uint32_t bl_flash_erase_region(uint32_t addr, uint32_t len)
{
	/* There is no need to check addr and len, target will find if it's validated and feedback in GeneralResponse */
    uint32_t rt_code;

	/* send flash erase region command */
	const command_packet_t *p_cmd = _construct_command_packet_w2Args(kCommandTag_FlashEraseRegion, kCommandFlag_HasNoDataPhase, addr, len);
	rt_code = _send_packet_and_read_sync((const uint8_t *)p_cmd, sizeof(framing_payload_packet_t) + p_cmd->framing_packet.length);
	if (rt_code != kStatus_Success)
	{
		return rt_code;
	}

	/* generic response */
	uint8_t *p_response_packet	= NULL;
	uint32_t response_len		= 0;
	rt_code = _read_packet_and_send_sync(&p_response_packet, &response_len, kFramingPacketType_Command);
	if (rt_code != kStatus_Success)
	{
		return rt_code;
	}

	return _handle_general_response((generic_response_packet_t *)p_response_packet, kCommandTag_FlashEraseRegion);
}

