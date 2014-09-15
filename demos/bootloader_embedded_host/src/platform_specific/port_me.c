#include <string.h>
#include "fsl_os_abstraction.h"
#include "fsl_gpio_driver.h"
#include "board.h"
#include "ff.h"
#include "bl_config.h"
#include "port_me.h"

static FATFS my_fatfs;

#define RESET_PIN				GPIO_MAKE_PIN(HW_GPIOB, 2)
#define BOOT_PIN				GPIO_MAKE_PIN(HW_GPIOB, 3)

const gpio_output_pin_user_config_t output_pin[] = {
	{
		.pinName = RESET_PIN,
		.config.outputLogic = 1,
		.config.slewRate = kPortFastSlewRate,
		.config.driveStrength = kPortHighDriveStrength,
		.config.isOpenDrainEnabled = false,
	},
	{
		.pinName = BOOT_PIN,
		.config.outputLogic = 0,
		.config.slewRate = kPortFastSlewRate,
		.config.driveStrength = kPortHighDriveStrength,
		.config.isOpenDrainEnabled = false,
	},
	{
		// Note: This pinName must be defined here to indicate the end of the array.
		.pinName = GPIO_PINS_OUT_OF_RANGE,
	}
};

void sys_enter_bootloader_mode(void)
{
    GPIO_DRV_SetPinOutput(BOOT_PIN);
}

void sys_enter_app_mode(void)
{
    GPIO_DRV_ClearPinOutput(BOOT_PIN);
}

void sys_reset_device(void)
{
	GPIO_DRV_ClearPinOutput(RESET_PIN);
	OSA_TimeDelay(100);
	GPIO_DRV_SetPinOutput(RESET_PIN);
	OSA_TimeDelay(100);
}

void sys_delay_in_ms(uint32_t ms)
{
	ms = ms + 1;
	OSA_TimeDelay(ms);
}

uint32_t sys_current_time_in_ms(void)
{
	return OSA_TimeGetMsec();
}

void *sys_malloc(uint32_t size)
{
	return malloc(size);
}

void sys_free(void *p)
{
	free(p);
}

int32_t sys_fopen_readonly(const char *p_filename)
{
	FIL *fd = sys_malloc(sizeof(FIL));
	if (f_open(fd, p_filename, FA_READ) == FR_OK)
	{
		return (int32_t)fd;
	}
	else
	{
		sys_free(fd);
		return -1;
	}
}

int32_t sys_fclose(int32_t fd)
{
	uint32_t result;
	result = f_close((FIL *)fd);
	sys_free((FIL *)fd);
	if (result == FR_OK) 
	{
		return 0;
	}
	else
	{
		return -1;
	}
}

int32_t sys_fread(int32_t fd, void *p_buffer, uint32_t size)
{
	uint32_t bytes_get;
	
	if (f_read((FIL *)fd, p_buffer, size, &bytes_get) == FR_OK) 
	{
		return bytes_get;
	}
	else
	{
		return -1;
	}
}

void *sys_fopendir(const char *p_filename)
{
	DIR *p_dir = sys_malloc(sizeof(DIR));
	if (p_dir == NULL)
	{
		return NULL;
	}
	
	if (f_opendir(p_dir, p_filename) == FR_OK)
	{
		return p_dir;
	}
	else
	{
		sys_free(p_dir);
		return NULL;
	}
}

int32_t sys_fclosedir(void *p_dir)
{
	sys_free(p_dir);
	return 0;
}

int32_t sys_freaddir_nameonly(void *p_dir, char *p_filename, uint32_t len)
{
	FILINFO tmpt;
	if (f_readdir(p_dir, &tmpt) == FR_OK)
	{
		strncpy(p_filename, tmpt.fname, len - 1);
		p_filename[len - 1] = '\0';
		return 0;
	}
	else
	{
		return -1;
	}
}


int32_t sys_fsize(int32_t fd)
{
	return f_size((FIL *)fd);
}

#if (CONFIG_LINKLAYER == I2C)

#include "fsl_i2c_master_driver.h"

#define I2C_INSTANCE					1

static i2c_master_state_t i2c_master_state;
static const i2c_device_t i2c_device = 
{
	.address	   = 0x10, /* 7-bit address, 0x20 for w, 0x21 for r */
	.baudRate_kbps = 400,
};

uint32_t i2c_send_blocking(const uint8_t *p_send_buffer, uint32_t len)
{
    uint32_t status;
    
    if (I2C_DRV_MasterSendDataBlocking(I2C_INSTANCE,
                                       &i2c_device,
                                       NULL,
                                       0,
                                       (uint8_t *)p_send_buffer,
                                       len,
                                       OSA_WAIT_FOREVER) == kStatus_I2C_Success)
    {
        status = LINK_COMM_SUCCESS;
    }
    else
    {
        status = LINK_COMM_FAIL;
    }
    
    return status;
}

uint32_t i2c_recv_blocking(uint8_t *p_recv_buffer, uint32_t len, uint32_t timeout)
{
    uint32_t status;
	(void)timeout;
    
    if (I2C_DRV_MasterReceiveDataBlocking(I2C_INSTANCE,
                                          &i2c_device,
                                          NULL,
                                          0,
                                          p_recv_buffer,
                                          len,
                                          OSA_WAIT_FOREVER) == kStatus_I2C_Success)
    {
        status = LINK_COMM_SUCCESS;
    }
    else
    {
        status = LINK_COMM_FAIL;
    }
    
    return status;
}

#elif (CONFIG_LINKLAYER == SPI)

#include "fsl_dspi_master_driver.h"

#define SPI_INSTANCE			0

static dspi_master_state_t spi_master_state;
static const dspi_device_t spi_device = 
{
	.bitsPerSec 				= 500 * 1000,
	.dataBusConfig.bitsPerFrame = 8,
	.dataBusConfig.clkPolarity  = kDspiClockPolarity_ActiveHigh,
	.dataBusConfig.clkPhase     = kDspiClockPhase_FirstEdge,
	.dataBusConfig.direction    = kDspiMsbFirst,
};

uint32_t spi_send_blocking(const uint8_t *p_send_buffer, uint32_t len)
{
    uint32_t status;
    
    if (DSPI_DRV_MasterTransferDataBlocking(SPI_INSTANCE,
                                       		&spi_device,
                                       		p_send_buffer,
                                       		NULL,	/* ignore incoming data */
                                       		len,
                                       		OSA_WAIT_FOREVER) == kStatus_DSPI_Success)
    {
        status = LINK_COMM_SUCCESS;
    }
    else
    {
        status = LINK_COMM_FAIL;
    }
    
    return status;
}

uint32_t spi_recv_blocking(uint8_t *p_recv_buffer, uint32_t len, uint32_t timeout)
{
    uint32_t status;
	(void)timeout;
    
    if (DSPI_DRV_MasterTransferDataBlocking(SPI_INSTANCE,
                                            &spi_device,
                                            NULL,	/* send 0x00 */
                                            p_recv_buffer,
                                            len,
                                            OSA_WAIT_FOREVER) == kStatus_DSPI_Success)
    {
        status = LINK_COMM_SUCCESS;
    }
    else
    {
        status = LINK_COMM_FAIL;
    }
    
    return status;
}

#elif (CONFIG_LINKLAYER == UART)

#include "fsl_uart_driver.h"

#define	UART_INSTANCE						1

static uart_state_t uart_state;

uint32_t uart_send_blocking(const uint8_t *p_send_buffer, uint32_t len)
{
    uint32_t status;
    
    if (UART_DRV_SendDataBlocking(UART_INSTANCE,
                                  p_send_buffer,
                                  len,
                                  OSA_WAIT_FOREVER) == kStatus_UART_Success)
    {
        status = LINK_COMM_SUCCESS;
    }
    else
    {
        status = LINK_COMM_FAIL;
    }
    
    return status;
}


uint32_t uart_recv_blocking(uint8_t *p_recv_buffer, uint32_t len, uint32_t timeout)
{
	uint32_t status;
	
	if (UART_DRV_ReceiveDataBlocking(UART_INSTANCE,
									 p_recv_buffer,
									 len,
									 timeout) == kStatus_UART_Success)
	{
		status = LINK_COMM_SUCCESS;
	}
	else
	{
		status = LINK_COMM_FAIL;
	}
	
	return status;
}

#endif

void sys_hw_init(void)
{
    hardware_init();
    dbg_uart_init();
    OSA_Init();

#if (CONFIG_LINKLAYER == I2C)
	I2C_DRV_MasterInit(I2C_INSTANCE, &i2c_master_state);
#elif (CONFIG_LINKLAYER == SPI)
	dspi_master_user_config_t user_config;
	user_config.isChipSelectContinuous = false;
	user_config.isSckContinuous		   = false;
	user_config.pcsPolarity 		   = kDspiPcs_ActiveLow;
	user_config.whichCtar 		       = kDspiCtar0;
	user_config.whichPcs			   = kDspiPcs0;
	DSPI_DRV_MasterInit(SPI_INSTANCE, &spi_master_state, &user_config);
#elif (CONFIG_LINKLAYER == UART)
	uart_user_config_t uart_config;
	uart_config.baudRate 		= 9600;
	uart_config.bitCountPerChar = kUart8BitsPerChar;
	uart_config.parityMode 	    = kUartParityDisabled;
	uart_config.stopBitCount    = kUartOneStopBit;
	UART_DRV_Init(UART_INSTANCE, &uart_state, &uart_config);
#endif

	PORT_HAL_SetMuxMode(PORTB_BASE, 2, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAsGpio);
	GPIO_DRV_OutputPinInit(output_pin);

	f_mount(1, &my_fatfs); 
}

