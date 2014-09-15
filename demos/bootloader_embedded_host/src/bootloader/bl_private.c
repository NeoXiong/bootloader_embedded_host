#include "port_me.h"
#include "bl_config.h"
#include "bl_protocol.h"
#include "bl_private.h"

static command_packet_t g_command_packet;

static data_packet_t    g_data_packet;


static uint32_t _read_sync(uint32_t timeout_in_ms);
static uint32_t _read_packet(framing_payload_packet_t *p_framing, uint8_t *p_payload, uint32_t expected_packet_type, uint32_t timeout_in_ms);
static uint16_t _calcuate_framing_crc16(framing_payload_packet_t *p_framing, const uint8_t *p_payload);

static uint16_t _calcuate_framing_crc16(framing_payload_packet_t *p_framing, const uint8_t *p_payload)
{
	uint16_t crc;
	crc = _crc16_update(0, (uint8_t *)p_framing, sizeof(framing_payload_packet_t) - sizeof(p_framing->crc16));
	crc = _crc16_update(crc, p_payload, p_framing->length);
	return crc;
}

/* read a payload packet (either command or data) and send sync (ACK/NAK/ACKABORT). */
static uint32_t _read_packet(framing_payload_packet_t *p_framing, uint8_t *p_payload, uint32_t expected_packet_type, uint32_t timeout_in_ms)
{
	uint32_t rt_code;

    uint32_t start_time_in_ms;
    uint32_t elapsed_time_in_ms;

    timeout_in_ms += 1;	/* +1 to ensure the actual delay >= setting delay */
    start_time_in_ms = sys_current_time_in_ms();

	rt_code = kStatus_hCommTimeOut;
    do {
		/* timeout_in_ms used here for UART only */
        if (DATA_RECV_BLOCKING(&(p_framing->header.start_byte), sizeof(p_framing->header.start_byte), timeout_in_ms) != LINK_COMM_SUCCESS)
        {
			return kStatus_hLinkCorrupt;
        }
        
        if (p_framing->header.start_byte == kFramingPacketStartByte)
        {
			rt_code = kStatus_Success;
			
			if (DATA_RECV_BLOCKING(&(p_framing->header.packet_type), 1, timeout_in_ms) != LINK_COMM_SUCCESS)
			{
				return kStatus_hLinkCorrupt;
			}

            break;
        }

		/* go to here only for I2C and SPI master operation, get 0x00 for no data */
		sys_delay_in_ms(1);	/* check every 1ms */
        
        elapsed_time_in_ms = sys_current_time_in_ms() - start_time_in_ms;
    } while (elapsed_time_in_ms < timeout_in_ms);

	/* return kStatus_hCommTimeOut if no start_byte|0x5A found */
	if (rt_code != kStatus_Success)
	{
		return rt_code;
	}

    if (p_framing->header.packet_type != expected_packet_type)
    {
		return kStatus_hUnexpectedPacketType;
    }

    /* Read length bytes */
    if (DATA_RECV_BLOCKING((uint8_t *)&p_framing->length, sizeof(p_framing->length), timeout_in_ms) != LINK_COMM_SUCCESS)
    {
		return kStatus_hLinkCorrupt;
    }
	
    /* Read CRC bytes */
    if (DATA_RECV_BLOCKING((uint8_t *)&p_framing->crc16, sizeof(p_framing->crc16), timeout_in_ms) != LINK_COMM_SUCCESS)
    {
		return kStatus_hLinkCorrupt;
    }

    /* Make sure the packet doesn't exceed the allocated buffer size */
    p_framing->length = MIN(CONFIG_MAX_PAYLOAD, p_framing->length);

    /* read the data or command content */
    if (p_framing->length > 0)
    {
        if (DATA_RECV_BLOCKING(p_payload, p_framing->length, timeout_in_ms) != LINK_COMM_SUCCESS)
        {
			rt_code = kStatus_hLinkCorrupt;
        }
    }

    return rt_code;
}

/* construct a data packet by provided data. */
const data_packet_t *_construct_data_packet(const uint8_t *p_data, uint32_t len)
{
	if (p_data == NULL)
	{
		return NULL;
	}

	memset(&g_data_packet, 0, sizeof(g_data_packet));
	
	g_data_packet.framing_packet.header.start_byte      = kFramingPacketStartByte;
	g_data_packet.framing_packet.header.packet_type     = kFramingPacketType_Data;
	g_data_packet.framing_packet.length                 = len;
	memcpy(g_data_packet.data_payload, p_data, MIN(CONFIG_MAX_PAYLOAD, len));
	g_data_packet.framing_packet.crc16 = 
		_calcuate_framing_crc16(&g_data_packet.framing_packet, (const uint8_t *)g_data_packet.data_payload);

	return &g_data_packet;
}

/* construct a command packet without parameter. */
const command_packet_t *_construct_command_packet_woArg(uint32_t command_tag, uint32_t flag)
{
	/* no need to check command_tag, target will check it and return in response packet */
	memset(&g_command_packet, 0, sizeof(g_command_packet));
	
	g_command_packet.framing_packet.header.start_byte      = kFramingPacketStartByte;
	g_command_packet.framing_packet.header.packet_type     = kFramingPacketType_Command;
	g_command_packet.framing_packet.length                 = sizeof(command_header_t) + 0 * sizeof(uint32_t);
	g_command_packet.command_packet.header.command_tag     = command_tag;
	g_command_packet.command_packet.header.flags           = flag;
	g_command_packet.command_packet.header.reserved        = 0;
	g_command_packet.command_packet.header.parameter_count = 0;

	g_command_packet.framing_packet.crc16 = 
		_calcuate_framing_crc16(&g_command_packet.framing_packet, (const uint8_t *)&g_command_packet.command_packet);

	return &g_command_packet;
}

/* construct a command packet with 1 32-bit width parameters. */
const command_packet_t *_construct_command_packet_w1Arg(uint32_t command_tag, uint32_t flag, uint32_t arg1)
{
	/* no need to check command_tag, target will check it and return in response packet */
	memset(&g_command_packet, 0, sizeof(g_command_packet));
	
	g_command_packet.framing_packet.header.start_byte      = kFramingPacketStartByte;
	g_command_packet.framing_packet.header.packet_type     = kFramingPacketType_Command;
	g_command_packet.framing_packet.length                 = sizeof(command_header_t) + 1 * sizeof(uint32_t);
	g_command_packet.command_packet.header.command_tag     = command_tag;
	g_command_packet.command_packet.header.flags           = flag;
	g_command_packet.command_packet.header.reserved        = 0;
	g_command_packet.command_packet.header.parameter_count = 1;
	g_command_packet.command_packet.args[0]                = arg1;
	g_command_packet.framing_packet.crc16 = 
		_calcuate_framing_crc16(&g_command_packet.framing_packet, (const uint8_t *)&g_command_packet.command_packet);

	return &g_command_packet;
}

/* construct a command packet with 2 32-bit width parameters. */
const command_packet_t *_construct_command_packet_w2Args(uint8_t command_tag, uint8_t flag, uint32_t arg1, uint32_t arg2)
{
	/* no need to check command_tag, target will check it and return in response packet */
	memset(&g_command_packet, 0, sizeof(g_command_packet));
	
	g_command_packet.framing_packet.header.start_byte      = kFramingPacketStartByte;
	g_command_packet.framing_packet.header.packet_type     = kFramingPacketType_Command;
	g_command_packet.framing_packet.length                 = sizeof(command_header_t) + 2 * sizeof(uint32_t);
	g_command_packet.command_packet.header.command_tag     = command_tag;
	g_command_packet.command_packet.header.flags           = flag;
	g_command_packet.command_packet.header.reserved        = 0;
	g_command_packet.command_packet.header.parameter_count = 2;
	g_command_packet.command_packet.args[0]                = arg1;
	g_command_packet.command_packet.args[1]                = arg2;
	g_command_packet.framing_packet.crc16 = 
		_calcuate_framing_crc16(&g_command_packet.framing_packet, (const uint8_t *)&g_command_packet.command_packet);

	return &g_command_packet;
}

/* construct a command packet with 3 32-bit width parameters. */
const command_packet_t *_construct_command_packet_w3Args(uint32_t command_tag, uint32_t flag, uint32_t arg1, uint32_t arg2, uint32_t arg3)
{
	/* no need to check command_tag, target will check it and return in response packet */
	memset(&g_command_packet, 0, sizeof(g_command_packet));
	
	g_command_packet.framing_packet.header.start_byte      = kFramingPacketStartByte;
	g_command_packet.framing_packet.header.packet_type     = kFramingPacketType_Command;
	g_command_packet.framing_packet.length                 = sizeof(command_header_t) + 3 * sizeof(uint32_t);
	g_command_packet.command_packet.header.command_tag     = command_tag;
	g_command_packet.command_packet.header.flags           = flag;
	g_command_packet.command_packet.header.reserved        = 0;
	g_command_packet.command_packet.header.parameter_count = 3;
	g_command_packet.command_packet.args[0]                = arg1;
	g_command_packet.command_packet.args[1]                = arg2;
	g_command_packet.command_packet.args[2]                = arg3;
	g_command_packet.framing_packet.crc16 = 
		_calcuate_framing_crc16(&g_command_packet.framing_packet, (const uint8_t *)&g_command_packet.command_packet);

	return &g_command_packet;
}

/* construct a command packet with 4 32-bit width parameters. */
const command_packet_t *_construct_command_packet_w4Args(uint32_t command_tag, uint32_t flag, uint32_t arg1, uint32_t arg2, uint32_t arg3, uint32_t arg4)
{
	/* no need to check command_tag, target will check it and return in response packet */
	memset(&g_command_packet, 0, sizeof(g_command_packet));
	
	g_command_packet.framing_packet.header.start_byte      = kFramingPacketStartByte;
	g_command_packet.framing_packet.header.packet_type     = kFramingPacketType_Command;
	g_command_packet.framing_packet.length                 = sizeof(command_header_t) + 4 * sizeof(uint32_t);
	g_command_packet.command_packet.header.command_tag     = command_tag;
	g_command_packet.command_packet.header.flags           = flag;
	g_command_packet.command_packet.header.reserved        = 0;
	g_command_packet.command_packet.header.parameter_count = 4;
	g_command_packet.command_packet.args[0]                = arg1;
	g_command_packet.command_packet.args[1]                = arg2;
	g_command_packet.command_packet.args[2]                = arg3;
	g_command_packet.command_packet.args[3] 			   = arg4;
	g_command_packet.framing_packet.crc16 = 
		_calcuate_framing_crc16(&g_command_packet.framing_packet, (const uint8_t *)&g_command_packet.command_packet);

	return &g_command_packet;
}


/* crc16 algorithm */
uint16_t _crc16_update(uint32_t crc, const uint8_t *p_src, uint32_t len)
{
    uint32_t j;
    for (j = 0; j < len; j++)
    {
        uint32_t i;
        uint32_t byte = p_src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; i++)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    }
    return crc;
}

/* Send all 2 bytes sync packet, include ACK, NAK, ACKABORT and PING */
uint32_t _send_sync(uint32_t sync)
{
	uint32_t rt_code;
	framing_sync_packet_t sync_to_target;

	sync_to_target.header.start_byte  = kFramingPacketStartByte;
	sync_to_target.header.packet_type = sync;

	rt_code = DATA_SEND_BLOCKING((uint8_t *)&sync_to_target, sizeof(sync_to_target));
	return (rt_code != LINK_COMM_SUCCESS) ? kStatus_hLinkCorrupt : kStatus_Success;
}

/* wait ACK, NAK or ACKABORT sync packet from target with a ms level timeout
   Target always excute command (erase, program) before sending ACK. Adopt byte by byte checking to ensure right sequence (NO |00|5A|, |A1|00|)*/
static uint32_t _read_sync(uint32_t timeout_in_ms)
{
    uint32_t start_time_in_ms;
    uint32_t elapsed_time_in_ms;
	uint8_t  c;
    uint32_t rt_code = kStatus_hCommTimeOut;

    timeout_in_ms += 1;	/* +1 to ensure the actual delay >= setting delay */
    start_time_in_ms = sys_current_time_in_ms();
    
    do {
		/* timeout_in_ms used here for UART only */
        if (DATA_RECV_BLOCKING(&c, 1, timeout_in_ms) != LINK_COMM_SUCCESS)
        {
			return kStatus_hLinkCorrupt;
        }
        
        if (c == kFramingPacketStartByte)
        {
			if (DATA_RECV_BLOCKING(&c, 1, timeout_in_ms) != LINK_COMM_SUCCESS)
			{
				return kStatus_hLinkCorrupt;
			}

            switch (c)
            {
            case kFramingPacketType_Ack:
            case kFramingPacketType_Nak:
            case kFramingPacketType_AckAbort:
                rt_code = c;
                break;
            default:
                rt_code = kStatus_hUnexpectedChar;
                break;
            }

			return rt_code;
        }

		/* go to here only for I2C and SPI master operation, get 0x00 for no data */
		sys_delay_in_ms(1);	/* check every 1ms */
        
        elapsed_time_in_ms = sys_current_time_in_ms() - start_time_in_ms;
    } while (elapsed_time_in_ms < timeout_in_ms);

	/* time out occur */
    return rt_code;
}

/* send a payload packet (either command or data) and read sync (ACK/NAK/ACKABORT). */
uint32_t _send_packet_and_read_sync(const uint8_t *p_data, uint32_t len)
{
	if ((p_data == NULL) || (len == 0))
	{
		return kStatus_hInvalidParam;
	}
	
	int32_t retries = CONFIG_MAX_RETRIES;
	uint32_t rt_code;
	
	do {
		/* send payload packet */
		if (DATA_SEND_BLOCKING(p_data, len) != LINK_COMM_SUCCESS)
		{
			return kStatus_hLinkCorrupt;
		}

		/* read sync packet within CONFIG_RECV_TIMEOUT ms */
		rt_code = _read_sync(CONFIG_RECV_TIMEOUT);
		if (rt_code == kFramingPacketType_Ack)
		{
			return kStatus_Success;
		}
		else if (rt_code == kFramingPacketType_Nak)
		{
			retries--;
		}
		else if (rt_code == kFramingPacketType_AckAbort)
		{
			return kStatus_hDataAbort;
		}
		else /* kStatus_hLinkCorrupt or kStatus_hUnexpectedChar or kStatus_hCommTimeOut */
		{
			return rt_code;
		}
	} while (retries > 0);

	/* still failed after CONFIG_MAX_RETRIES retries */
	return kStatus_hRetryFail;
}

uint8_t InCommingData[CONFIG_MAX_PAYLOAD];
uint32_t _read_packet_and_send_sync(uint8_t **p_p_packet, uint32_t *p_packet_len, uint32_t expected_packet_type)
{
	if ((p_p_packet == NULL) || (p_packet_len == NULL))
	{
		return kStatus_hInvalidParam;
	}

	framing_payload_packet_t framing_packet;
	
	uint32_t rt_code;
	uint16_t crc;
	int32_t retries = CONFIG_MAX_RETRIES;

	do {
		rt_code = _read_packet(&framing_packet, InCommingData, expected_packet_type, CONFIG_RECV_TIMEOUT);
		if (rt_code != kStatus_Success)
		{
			return rt_code;
		}

		crc = _calcuate_framing_crc16(&framing_packet, InCommingData);
		if (crc == framing_packet.crc16)
		{
			rt_code = _send_sync(kFramingPacketType_Ack);
			if (rt_code != kStatus_Success)
			{
				return rt_code;
			}

			*p_p_packet   = InCommingData;
			*p_packet_len = framing_packet.length;

			return kStatus_Success;
		}
		else
		{
			retries--;
			
			rt_code = _send_sync(kFramingPacketType_Nak);
			if (rt_code != kStatus_Success)
			{
				return rt_code;
			}
		}
	} while (retries > 0);

	return kStatus_hRetryFail;
}

/* Handle general response and return status to Application layer. */
uint32_t _handle_general_response(const generic_response_packet_t *p_generic_response, uint32_t expected_command_tag)
{
    if (p_generic_response == NULL)
    {
		return kStatus_hInvalidParam;
    }

    if (p_generic_response->command_packet.command_tag != kCommandTag_GenericResponse)
    {
		return kStatus_hUnexpectedCommand;
    }
	
    if (p_generic_response->command != expected_command_tag)
    {
        return kStatus_hExpectedResp;
    }

    return p_generic_response->status;
}

