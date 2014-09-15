#include "bl_private.h"
#include "bl_protocol.h"

/* write a specified range of bytes in memory (FLASH or RAM) by provided data source. */
uint32_t bl_write_memory(uint32_t addr, const uint8_t *p_data, uint32_t len)
{
	/* no need to check addr and len, target will check if it's validate and feedback in response packet */
	if (p_data == NULL)
	{
		return kStatus_hInvalidParam;
	}
	
	uint32_t rt_code;

	/* send write memory command */
	const command_packet_t *p_cmd_payload = _construct_command_packet_w2Args(kCommandTag_WriteMemory, kCommandFlag_HasDataPhase, addr, len);
	rt_code = _send_packet_and_read_sync((const uint8_t *)p_cmd_payload, sizeof(framing_payload_packet_t) + p_cmd_payload->framing_packet.length);
	if (rt_code != kStatus_Success)
	{
		return rt_code;
	}

	/* initial generic response */
	uint8_t *p_response_packet	= NULL;
	uint32_t response_len		= 0;
	rt_code = _read_packet_and_send_sync(&p_response_packet, &response_len, kFramingPacketType_Command);
	if (rt_code != kStatus_Success)
	{
		return rt_code;
	}

	rt_code = _handle_general_response((const generic_response_packet_t *)p_response_packet, kCommandTag_WriteMemory);
	if (rt_code != kStatus_Success)
	{
		return rt_code;
	}

	const data_packet_t *p_data_payload;

	/* split data into small data payload packet */
	while (len >= CONFIG_MAX_PAYLOAD)
	{
		len -= CONFIG_MAX_PAYLOAD;

		p_data_payload = _construct_data_packet(p_data, CONFIG_MAX_PAYLOAD);
		rt_code = _send_packet_and_read_sync((const uint8_t *)p_data_payload, sizeof(framing_payload_packet_t) + p_data_payload->framing_packet.length);
		if (rt_code != kStatus_Success)
		{
			return rt_code;
		}

		p_data += CONFIG_MAX_PAYLOAD;
	}

	/* handle bytes less than CONFIG_MAX_PAYLOAD payload size */
	if (len > 0)
	{
		p_data_payload = _construct_data_packet(p_data, len);
		rt_code = _send_packet_and_read_sync((const uint8_t *)p_data_payload, sizeof(framing_payload_packet_t) + p_data_payload->framing_packet.length);
		if (rt_code != kStatus_Success)
		{
			return rt_code;
		}
	}	

	/* final generic response */
	p_response_packet	= NULL;
	response_len		= 0;
	rt_code = _read_packet_and_send_sync(&p_response_packet, &response_len, kFramingPacketType_Command);
	if (rt_code != kStatus_Success)
	{
		return rt_code;
	}

	return _handle_general_response((const generic_response_packet_t *)p_response_packet, kCommandTag_WriteMemory);
}

