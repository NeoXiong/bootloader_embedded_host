#ifndef __PORT_ME_H__
#define __PORT_ME_H__

#include <stdint.h>

#define LINK_COMM_SUCCESS		0
#define LINK_COMM_FAIL			1

/* hw init and ctrl */
void sys_hw_init(void);
void sys_enter_bootloader_mode();
void sys_enter_app_mode();
void sys_reset_device(void);

/* dynamic memory allocation */
void *sys_malloc(uint32_t size);
void sys_free(void *p);

/* file system */
int32_t sys_fopen_readonly(const char *p_name);
int32_t sys_fclose(int32_t fd);
int32_t sys_fread(int32_t fd, void *p_buffer, uint32_t size);
int32_t sys_fsize(int32_t fd);
void *sys_fopendir(const char *p_filename);
int32_t sys_fclosedir(void *p_dir);
int32_t sys_freaddir_nameonly(void *p_dir, char *p_filename, uint32_t len);

/* time */
void sys_delay_in_ms(uint32_t ms);
uint32_t sys_current_time_in_ms(void);

#define WAIT_FOREVER		0xFFFFFFFF

#if (CONFIG_LINKLAYER == I2C)

uint32_t i2c_send_blocking(const uint8_t *p_send_buffer, uint32_t len);
uint32_t i2c_recv_blocking(uint8_t *p_recv_buffer, uint32_t len, uint32_t timeout);

#define DATA_SEND_BLOCKING(p1, p2)			i2c_send_blocking(p1, p2)
#define DATA_RECV_BLOCKING(p1, p2, p3)		i2c_recv_blocking(p1, p2, p3)

#elif (CONFIG_LINKLAYER == SPI)

uint32_t spi_send_blocking(const uint8_t *p_send_buffer, uint32_t len);
uint32_t spi_recv_blocking(uint8_t *p_recv_buffer, uint32_t len, uint32_t timeout);

#define DATA_SEND_BLOCKING(p1, p2)			spi_send_blocking(p1, p2)
#define DATA_RECV_BLOCKING(p1, p2, p3)		spi_recv_blocking(p1, p2, p3)

#elif (CONFIG_LINKLAYER == UART)

uint32_t uart_send_blocking(const uint8_t *p_send_buffer, uint32_t len);
uint32_t uart_recv_blocking(uint8_t *p_recv_buffer, uint32_t len, uint32_t timeout);

#define DATA_SEND_BLOCKING(p1, p2)			uart_send_blocking(p1, p2)
#define DATA_RECV_BLOCKING(p1, p2, p3)		uart_recv_blocking(p1, p2, p3)

#else

#error "ERR! CONFIG_LINKLAYER must be set to either of I2C, SPI or UART"

#endif

#endif