#include "bl_private.h"
#include "bl_protocol.h"

/* Send Ping packet and wait PingResponse */
uint32_t bl_ping(uint8_t *p_version)
{
	if (p_version == NULL)
	{
		return kStatus_hInvalidParam;
	}
    
    uint32_t rt_code;
	framing_ping_response_t ping_response;

    rt_code = _send_sync(kFramingPacketType_Ping);
    if (rt_code != kStatus_Success)
    {
        return rt_code;
    }
	
    sys_delay_in_ms(1);	/* give target 1ms to prepare response */
	
    if (DATA_RECV_BLOCKING((uint8_t *)&ping_response, sizeof(ping_response), WAIT_FOREVER) != LINK_COMM_SUCCESS)
    {
        return kStatus_hLinkCorrupt;
    }
    
    if (_crc16_update(0, (uint8_t *)&ping_response, sizeof(framing_ping_response_t) - sizeof(ping_response.crc16)) != ping_response.crc16)
    {
        return kStatus_hCRCError;
    }

	memcpy(p_version, ping_response.version, 4);
    
    return kStatus_Success;
}


