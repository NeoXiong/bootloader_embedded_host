--- HW setup ------------------------------------------------------------------------------------------------------------------------------

1) Prepare 2x FRDM-K64F board, 1 is Host running 'bootloader_embedded_host', the other is target running 'FSL_Kinetis_Bootloader_1_0_2'. 
2) Connect two boards by 4 wires:
   > [HOST J4-12] PTC10/I2C1_SCL   <--->   [TARGET J2-20] PTE24/I2C0_SCL
   > [HOST J4-10] PTC11/I2C1_SDa   <--->   [TARGET J2-18] PTE25/I2C0_SDA
   > [HOST J4-2]  PTB2             <--->   [TARGET J3-6]  RST_TGTMCU_B
   > [HOST J3-16] GND              <--->   [HOST   J3-16] GND   
3) Power two boards by USB-Micro.

--- SW setup ------------------------------------------------------------------------------------------------------------------------------

General:
1) Download Install KSDK 1.0.0GA and Kinetis Bootloader 1.0.2 to default path. 

Host:
1) Copy 'demos' and 'misc' folder to C:\Freescale\KSDK_1.0.0\
2) Open C:\Freescale\KSDK_1.0.0\demos\bootloader_embedded_host\iar\frdmk64f120m\bootloader_embedded_host.eww
3) Rebuild 'ksdk_platform_lib-debug' and 'bootloader_embedded_host-debug' target.
4) Download .out to host FRDM-K64F board.

Target:
1) Copy and paste '\misc\peripherals_MK64F12.c and hardware_init_MK64F12.c' to 
   'C:\Freescale\FSL_Kinetis_Bootloader_1_0_2\targets\k64f12\src' to enable I2C0 PTE24/25 as communication interface. The default I2C1 PTC10/11 doesn't have on-board pull-up. 
2) Open 'C:\Freescale\FSL_Kinetis_Bootloader_1_0_2\targets\k64f12\bootloader.eww' and Rebuild all.
4) Download .out to target FRDM-K64F board.

--- Woking flow ------------------------------------------------------------------------------------------------------------------------------

1) Format a microSD to FAT32 format and copy '\misc\led_demo_MK64F12_Freedom_FTF.bin' to the root path.
2) insert microSD card to host FRDM-K64F board.
3) Open a terminal connected to host FRDM-K64F board's CDC UART ith 115200N81. 
4) Reset host FRDM-K64F board, user will see a prompt as below:


====== Welcome to bootloader embedded host demo ======
- Press digital key to select:
<1> Select the target binary in SDcard (LED_DE~1.BIN)
<2> Go bootloader mode to do update


5) Press '1' and then 'n' to select led_demo_MK64F12_Freedom_FTF.bin (aka LED_DE~1.BIN, cutted to 8.3 format)
6) Press '2' to reset board and download firmware, user will get prompt as below if succeed.

Connected to bootloader, version: P.1.1.0
Start downloading...Done.
Press Enter to continue...

7) Reset target FRDM-K64F board by reset button, user will see the RGB LED changes.
