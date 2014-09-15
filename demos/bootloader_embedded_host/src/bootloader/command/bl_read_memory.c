#include "bl_private.h"
#include "bl_protocol.h"

static uint32_t _handle_read_memory_response(const read_memory_response_packet_t *p_read_memory_response, uint32_t *p_byte_to_read);

/* read the contents of memory at given address for a specified number of bytes. */
uint32_t bl_read_memory(uint32_t addr, uint8_t *p_data, uint32_t len)	/* TO DO: handle 0 length packet to abort */
{
	/* no need to check addr and len, target will check if it's validate and feedback in response packet */
	if (p_data == NULL)
	{
		return kStatus_hInvalidParam;
	}
	
	uint32_t rt_code;

	/* send read memory command */
	const command_packet_t *p_cmd_payload = _construct_command_packet_w2Args(kCommandTag_ReadMemory, kCommandFlag_HasNoDataPhase, addr, len);
	rt_code = _send_packet_and_read_sync((const uint8_t *)p_cmd_payload, sizeof(framing_payload_packet_t) + p_cmd_payload->framing_packet.length);
	if (rt_code != kStatus_Success)
	{
		return rt_code;
	}

	/* read memory response */
	uint8_t *p_response_packet	= NULL;
	uint32_t response_len		= 0;
	rt_code = _read_packet_and_send_sync(&p_response_packet, &response_len, kFramingPacketType_Command);
	if (rt_code != kStatus_Success)
	{
		return rt_code;
	}

	rt_code = _handle_read_memory_response((const read_memory_response_packet_t *)p_response_packet, &len);
	if (rt_code != kStatus_Success)
	{
		return rt_code;
	}

	uint8_t *p_data_payload = NULL;
	uint32_t len_read       = 0;
	uint32_t offset 		= 0;

	do {
		rt_code = _read_packet_and_send_sync(&p_data_payload, &len_read, kFramingPacketType_Data);
		if (rt_code != kStatus_Success)
		{
			return rt_code;
		}

		memcpy(&p_data[offset], p_data_payload, len_read);
		
		offset	+= len_read;
		len 	-= len_read;
	} while (len > 0);

	/* final response is a general response for read memory command */
	p_response_packet	= NULL;
	response_len		= 0;
	rt_code = _read_packet_and_send_sync(&p_response_packet, &response_len, kFramingPacketType_Command);
	if (rt_code != kStatus_Success)
	{
		return rt_code;
	}

	return _handle_general_response((const generic_response_packet_t *)p_response_packet, kCommandTag_ReadMemory);
}

/* Handle read memory response and return status and how many bytes target will send back. */
static uint32_t _handle_read_memory_response(const read_memory_response_packet_t *p_read_memory_response, uint32_t *p_byte_to_read)
{
    if ((p_read_memory_response == NULL) || (p_byte_to_read == NULL))
    {
		return kStatus_hInvalidParam;
    }

	*p_byte_to_read = 0;

	/* handle generic response which would be returned if command is not supported */
    if (p_read_memory_response->command_packet.command_tag == kCommandTag_GenericResponse)
    {
		return _handle_general_response((const generic_response_packet_t *)p_read_memory_response, kCommandTag_ReadMemory);
    }
	
    if (p_read_memory_response->command_packet.command_tag != kCommandTag_ReadMemoryResponse)
    {
		return kStatus_hUnexpectedCommand;
    }

	if (p_read_memory_response->status != kStatus_Success)
	{
		return p_read_memory_response->status;
	}

	*p_byte_to_read = p_read_memory_response->byte_count;
    return kStatus_Success;
}


