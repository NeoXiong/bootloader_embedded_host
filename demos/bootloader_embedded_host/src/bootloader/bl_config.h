#ifndef __BL_CONFIG_H__
#define __BL_CONFIG_H__

/* Max. payload in packet, should be align with target */
#define CONFIG_MAX_PAYLOAD					32

/* Max. retry times for occurpt data*/
#define CONFIG_MAX_RETRIES					3

/* Timeout in milliseconds for sync packet: ACK, NAK, ACKABORT */
#define CONFIG_RECV_TIMEOUT					500

#define CONFIG_APP_ADDRESS					0x0000A000

#define CONFIG_BYTES_ALIGN					8

#define ROOT_DIR    						"1:"

#define I2C									0
#define SPI									1
#define UART								2

#define CONFIG_LINKLAYER					I2C

#endif

