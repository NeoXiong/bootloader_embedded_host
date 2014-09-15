/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "code2str.h"
#include "port_me.h"
#include "bl_config.h"
#include "bl_protocol.h"

typedef struct
{
	int32_t    fd;
    uint32_t   baseaddr;
	uint32_t   size;
	uint32_t   offset;
	uint8_t   *p_buffer;
} firmware_info_t;

#define MIN(a, b)   		((a) < (b) ? (a) : (b))
#define ALIGN_DOWN(x, a)    ((x) & -(a))
#define ALIGN_UP(x, a)      (-(-(x) & -(a)))

static uint32_t get_user_choice(uint32_t max)
{
    uint32_t key;

	do {
    	key = getchar() - 0x30;
	} while ((key == 0) || (key > max));

    return key;
}

static void wait_for_enter(void)
{
	char key;
	
	do {
    	key = getchar();
	} while (key != '\r');
}

static int32_t show_all_binary(char *p_target_filename)
{
	void *p_dir;
	char filename[10][13]; /* max support 10 files */
	uint32_t num = 0;
	uint32_t choice;
	p_dir = sys_fopendir(ROOT_DIR);

	if (p_dir != NULL)
	{
		printf("\f- All binary files in root folder are listed below:\r\n");
		
		while (1)
		{
			if (sys_freaddir_nameonly(p_dir, &filename[num][0], 13) == 0)
			{
				if (filename[num][0] == '\0')
				{
					break;
				}	

				if ((strstr(&filename[num][0], ".BIN") != NULL) || (strstr(&filename[num][0], ".bin") != NULL))
				{
                    printf("<%d> %s\r\n", num + 1, &filename[num][0]);
                    ++num;
				}
			}
			else
			{
				printf("");
				return -1;
			}
		}

		if (num > 0)
		{
			choice = get_user_choice(num);

			strcpy(p_target_filename, &filename[choice - 1][0]);
		}
		else
		{
			printf("No binary file exist\r\n");
			p_target_filename[0] = '\0';
			return -1;
		}

		sys_fclosedir(p_dir);
		return 0;
	}
    
    return -1;
}

static void show_menu(const char *p_filename)
{
    printf("\f====== Welcome to bootloader embedded host demo ======\r\n");
	printf("- Press digital key to select:\r\n");
	printf("<1> Select the target binary in SDcard (%s)\r\n", p_filename);
	printf("<2> Go bootloader mode to do update\r\n");
}

static int32_t do_update(const char *p_filename)
{
	uint32_t rt_code;
	int32_t  fd;
	uint32_t len;
	uint32_t sector_size;
    uint32_t erase_size;
	firmware_info_t *p_fw_info;
    uint8_t bl_version[4];
    char file_path[100];
    
    memset(file_path, 0, 100);
    strcpy(file_path, ROOT_DIR);
    strcat(file_path, p_filename);

	/* Open file and retrieve info. */
	p_fw_info = sys_malloc(sizeof(firmware_info_t));
	if (p_fw_info == NULL)
	{
		printf("sys_malloc(firmware_info_t) Error: get NULL\r\n");
		goto ERROR_EXIT3; /* goto EXIT3 because .fd and .p_buffer are not ready */
	}

	p_fw_info->baseaddr = CONFIG_APP_ADDRESS;
	p_fw_info->offset   = 0;
	p_fw_info->p_buffer = NULL;
	p_fw_info->fd       = sys_fopen_readonly(file_path);
	if (p_fw_info->fd == -1)
	{
		printf("sys_fopen_readonly() Error:%d", p_fw_info->fd);
		goto ERROR_EXIT2; /* goto EXIT2 because .fd is not ready */
	}
	p_fw_info->size = sys_fsize(p_fw_info->fd);
	
	/* force device to enter bootloader mode */
	sys_enter_bootloader_mode();
	sys_reset_device();

	/* try to ping bootloader */
	rt_code = bl_ping(bl_version);
	if (rt_code != kStatus_Success) 
	{
		printf("bl_ping() Error: %s\r\n", code_2_str(rt_code));
		goto ERROR_EXIT1;
	}
	printf("\fConnected to bootloader, version: %c.%d.%d.%d\r\n", bl_version[3], bl_version[2], bl_version[1], bl_version[0]);
	
	/* get property */
	//rt_code = bl_getproerty(SECTOR_SIZE, &sector_size);
	//rt_code = bl_getproerty(RESEVED_REGION, &flash_addr);
	sector_size = 2048;
	p_fw_info->p_buffer = (uint8_t *)sys_malloc(sector_size);
	if (p_fw_info->p_buffer == NULL) 
	{
		printf("sys_malloc(p_fw_info->p_buffer) Error: get NULL\r\n");
		goto ERROR_EXIT1; /* goto EXIT2 because p_fw_info->p_buffer is not ready to support free() */
	}

	printf("Start downloading...");
	/* erase memory, size must be 4 or 8 bytes aligned */
	rt_code = bl_flash_erase_region(p_fw_info->baseaddr, ALIGN_UP(p_fw_info->size, CONFIG_BYTES_ALIGN));
	if (rt_code != kStatus_Success) 
	{
		printf("bl_flash_erase_region() Error: %s\r\n", code_2_str(rt_code));
		goto ERROR_EXIT1;
	}
	
	/* write memory */
	len = 0;
	while (p_fw_info->size > 0) 
	{
		len = MIN(sector_size, p_fw_info->size);

		/* assume we can get exact bytes as expect to make things simple */
		if (sys_fread(p_fw_info->fd, p_fw_info->p_buffer, len) == -1)
		{
			printf("sys_fread() Error: -1\r\n");
			goto ERROR_EXIT1;
		}

		rt_code = bl_write_memory(p_fw_info->baseaddr + p_fw_info->offset, p_fw_info->p_buffer, len);
		if (rt_code != kStatus_Success) 
		{
			printf("bl_write_memory() Error: %s\r\n", code_2_str(rt_code));
			goto ERROR_EXIT1;
		}

		p_fw_info->offset += len;
		p_fw_info->size   -= len;
	}
	
	printf("Done.\r\n");
	sys_fclose(p_fw_info->fd);
	sys_free(p_fw_info->p_buffer);
	sys_free(p_fw_info);
	printf("Press Enter to continue...");
	wait_for_enter();
	return 0; /* success */

ERROR_EXIT1:
	sys_fclose(p_fw_info->fd);
ERROR_EXIT2:
	sys_free(p_fw_info->p_buffer);
ERROR_EXIT3:
	sys_free(p_fw_info);
	printf("Press Enter to exit...");
	wait_for_enter();
	return -1;
}

int main(int argc, char *argv[])
{
	int32_t rt_code = 0;
	char target_filename[13] = {'N', 'O', 'N', 'E','\0'};		/* only support 8.3 format */

	/* platform dependent initialization, clock, I/O, SD card etc */
    sys_hw_init();

	while (1)
	{
	    show_menu(&target_filename[0]);

		switch (get_user_choice(2))
		{
		case 1:
			{
				rt_code = show_all_binary(&target_filename[0]);
			}
			break;

		case 2:
			{
				rt_code = do_update(&target_filename[0]);
			}
			break;

		default:
            rt_code = -1;
			break;
		}
        
        if (rt_code == -1)
        {
            break;
        }
	}

	return 0;
}

