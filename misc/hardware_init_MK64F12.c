/*
 * Copyright (c) 2013, Freescale Semiconductor, Inc.
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

#include "bootloader_common.h"
#include "bootloader/context.h"
#include "device/fsl_device_registers.h"
#include "drivers/uart/scuart.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

#ifdef TOWER
#define UART1_RX_GPIO_PIN_NUM 3  // PIN 3 in the PTC group
#define UART1_RX_ALT_MODE 3      // ALT mode for UART1 functionality for pin 3
#define UART1_RX_GPIO_ALT_MODE 1 // ALT mdoe for GPIO functionality for pin 3

#define UART1_TX_GPIO_PIN_NUM 4  // PIN 4 in the PTC group
#define UART1_TX_ALT_MODE 3      // ALT mode for UART1 TX functionality for pin 4

// UART 5 is the uart from the tower serial board
#define UART5_RX_GPIO_PIN_NUM 9  // PIN 9 in the PTE group
#define UART5_RX_ALT_MODE 3      // ALT mode for UART5 functionality for pin 9
#define UART5_RX_GPIO_ALT_MODE 1 // ALT mdoe for GPIO functionality for pin 9

#define UART5_TX_GPIO_PIN_NUM 8  // PIN 8 in the PTE group
#define UART5_TX_ALT_MODE 3      // ALT mode for UART1 TX functionality for pin 4
#endif //TOWER

#ifdef FREEDOM
#define UART0_RX_GPIO_PIN_NUM 16  // PIN 3 in the PTB group
#define UART0_RX_ALT_MODE 3       // ALT mode for UART0 functionality for pin 16
#define UART0_RX_GPIO_ALT_MODE 1  // ALT mdoe for GPIO functionality for pin 16

#define UART0_TX_GPIO_PIN_NUM 17  // PIN 17 in the PTB group
#define UART0_TX_ALT_MODE 3       // ALT mode for UART0 TX functionality for pin 17
#endif //FREEDOM

#define PORT_IRQC_INTERRUPT_FALLING_EDGE 0xA
#define PORT_IRQC_INTERRUPT_DISABLE 0

#define BOOT_PIN_DEBOUNCE_READ_COUNT 500

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////
//! this is to store the function pointer for calling back to the function that wants
//! the UART RX instance pin that triggered the interrupt.
static pin_irq_callback_t s_pin_irq_func[HW_UART_INSTANCE_COUNT] = {0};

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

/* This function is called for configurating pinmux for uart module
 * This function only support switching default or gpio or fixed-ALTx mode on fixed pins
 * (Although there are many ALTx-pinmux configuration choices on various pins for the same
 * peripheral module) */
void uart_pinmux_config(unsigned int instance, pinmux_type_t pinmux)
{
    switch(instance)
    {
        case 0:
#ifdef FREEDOM
            switch(pinmux)
            {
                case kPinmuxType_Default:
                    BW_PORT_PCRn_MUX(HW_PORTB, UART0_RX_GPIO_PIN_NUM, 0);
                    BW_PORT_PCRn_MUX(HW_PORTB, UART0_TX_GPIO_PIN_NUM, 0);
                    break;
                case kPinmuxType_GPIO:
                    BW_PORT_PCRn_MUX(HW_PORTB, UART0_RX_GPIO_PIN_NUM, UART0_RX_GPIO_ALT_MODE); // Set UART0_RX pin in GPIO mode
                    HW_GPIO_PDDR_CLR(HW_GPIOB, 1 << UART0_RX_GPIO_PIN_NUM);                    // Set UART0_RX pin as an input
                    break;
                case kPinmuxType_Peripheral:
                    BW_PORT_PCRn_MUX(HW_PORTB, UART0_RX_GPIO_PIN_NUM, UART0_RX_ALT_MODE);   // Set UART0_RX pin to UART0_RX functionality
                    BW_PORT_PCRn_MUX(HW_PORTB, UART0_TX_GPIO_PIN_NUM, UART0_TX_ALT_MODE);   // Set UART0_TX pin to UART0_TX functionality
                    break;
                default:
                    break;
            }
#endif // FREEDOM
            break;
        case 1:
#ifdef TOWER
            switch(pinmux)
            {
                case kPinmuxType_Default:
                    BW_PORT_PCRn_MUX(HW_PORTC, UART1_RX_GPIO_PIN_NUM, 0);
                    BW_PORT_PCRn_MUX(HW_PORTC, UART1_TX_GPIO_PIN_NUM, 0);
                    break;
                case kPinmuxType_GPIO:
                    BW_PORT_PCRn_MUX(HW_PORTC, UART1_RX_GPIO_PIN_NUM, UART1_RX_GPIO_ALT_MODE); // Set UART1_RX pin in GPIO mode
                    HW_GPIO_PDDR_CLR(HW_GPIOC, 1 << UART1_RX_GPIO_PIN_NUM);                    // Set UART1_RX pin as an input
                    break;
                case kPinmuxType_Peripheral:
                    BW_PORT_PCRn_MUX(HW_PORTC, UART1_RX_GPIO_PIN_NUM, UART1_RX_ALT_MODE);   // Set UART1_RX pin to UART1_RX functionality
                    BW_PORT_PCRn_MUX(HW_PORTC, UART1_TX_GPIO_PIN_NUM, UART1_TX_ALT_MODE);   // Set UART1_TX pin to UART1_TX functionality
                    break;
                default:
                    break;
            }
#endif // TOWER
            break;
        case 2:
        case 3:
        case 4:
            break;
        case 5:
#ifdef TOWER
            switch(pinmux)
            {
                case kPinmuxType_Default:
                    BW_PORT_PCRn_MUX(HW_PORTE, UART5_RX_GPIO_PIN_NUM, 0);
                    BW_PORT_PCRn_MUX(HW_PORTE, UART5_TX_GPIO_PIN_NUM, 0);
                    break;
                case kPinmuxType_GPIO:
                    BW_PORT_PCRn_MUX(HW_PORTE, UART5_RX_GPIO_PIN_NUM, UART5_RX_GPIO_ALT_MODE); // Set UART5_RX pin in GPIO mode
                    HW_GPIO_PDDR_CLR(HW_GPIOE, 1 << UART5_RX_GPIO_PIN_NUM);                    // Set UART5_RX pin as an input
                    break;
                case kPinmuxType_Peripheral:
                    BW_PORT_PCRn_MUX(HW_PORTE, UART5_RX_GPIO_PIN_NUM, UART5_RX_ALT_MODE);   // Set UART5_RX pin to UART5_RX functionality
                    BW_PORT_PCRn_MUX(HW_PORTE, UART5_TX_GPIO_PIN_NUM, UART5_TX_ALT_MODE);   // Set UART5_TX pin to UART5_TX functionality
                    break;
                default:
                    break;
            }
#endif // TOWER
            break;
        default:
            break;
    }
}

/* This function is called for configurating pinmux for i2c module
 * This function only support switching default or gpio or fixed-ALTx mode on fixed pins
 * (Although there are many ALTx-pinmux configuration choices on various pins for the same
 * peripheral module) */
void i2c_pinmux_config(unsigned int instance, pinmux_type_t pinmux)
{
    switch(instance)
    {
        case 0:
#ifdef TOWER
            switch(pinmux)
            {
                case kPinmuxType_Default:
                    BW_PORT_PCRn_MUX(HW_PORTD, 8, 0);
                    BW_PORT_PCRn_MUX(HW_PORTD, 9, 0);
                    break;
                case kPinmuxType_Peripheral:
                    // Enable pins for I2C0.
                    BW_PORT_PCRn_MUX(HW_PORTD, 8, 2);  // I2C0_SCL is ALT2 for pin PTD8
                    BW_PORT_PCRn_MUX(HW_PORTD, 9, 2);  // I2C0_SDA is ALT2 for pin PTD9
                    break;
                default:
                    break;
            }
#endif // TOWER
            switch(pinmux)
            {
                case kPinmuxType_Default:
                    BW_PORT_PCRn_MUX(HW_PORTE, 24, 0);
                    BW_PORT_PCRn_MUX(HW_PORTE, 25, 0);
                    break;
                case kPinmuxType_Peripheral:
                    // Enable pins for I2C0.
                    BW_PORT_PCRn_MUX(HW_PORTE, 24, 5);  // I2C0_SCL is ALT5 for pin PTE24
                    BW_PORT_PCRn_ODE(HW_PORTE, 24, true);
                    BW_PORT_PCRn_MUX(HW_PORTE, 25, 5);  // I2C0_SDA is ALT5 for pin PTE25
                    BW_PORT_PCRn_ODE(HW_PORTE, 25, true);
                    break;
                default:
                    break;
            }
            break;
        case 1:
#ifdef FREEDOM
            switch(pinmux)
            {
                case kPinmuxType_Default:
                    BW_PORT_PCRn_MUX(HW_PORTC, 10, 0);
                    BW_PORT_PCRn_MUX(HW_PORTC, 11, 0);
                    break;
                case kPinmuxType_Peripheral:
                    // Enable pins for I2C1.
                    BW_PORT_PCRn_MUX(HW_PORTC, 10, 2);  // I2C1_SCL is ALT2 for pin PTC10
                    BW_PORT_PCRn_MUX(HW_PORTC, 11, 2);  // I2C1_SDA is ALT2 for pin PTC11
                    break;
                default:
                    break;
            }
#endif // FREEDOM
            break;
        case 2:
            break;
        default:
            break;
    }
}

/* This function is called for configurating pinmux for spi module
 * This function only support switching default or gpio or fixed-ALTx mode on fixed pins
 * (Although there are many ALTx-pinmux configuration choices on various pins for the same
 * peripheral module) */
void spi_pinmux_config(unsigned int instance, pinmux_type_t pinmux)
{
    switch(instance)
    {
        case 0:
            switch(pinmux)
            {
                case kPinmuxType_Default:
                    BW_PORT_PCRn_MUX(HW_PORTD, 0, 0);
                    BW_PORT_PCRn_MUX(HW_PORTD, 1, 0);
                    BW_PORT_PCRn_MUX(HW_PORTD, 2, 0);
                    BW_PORT_PCRn_MUX(HW_PORTD, 3, 0);
                    break;
                case kPinmuxType_Peripheral:
                    // Enable pins for SPI0 on PTD0~3
                    BW_PORT_PCRn_MUX(HW_PORTD, 0, 2);  // SPI0_PCS0 is ALT2 for pin PTD0
                    BW_PORT_PCRn_MUX(HW_PORTD, 1, 2);  // SPI0_SCK is ALT2 for pin PTD1
                    BW_PORT_PCRn_MUX(HW_PORTD, 2, 2);  // SPI0_SOUT is ALT2 for pin PTD2
                    BW_PORT_PCRn_MUX(HW_PORTD, 3, 2);  // SPI0_SIN is ALT2 for pin PTD3
                    break;
                default:
                    break;
            }
            break;
        case 1:
            break;
        case 2:
            break;
        default:
            break;
    }
}

void init_hardware(void)
{
    // Disable the MPU otherwise USB cannot access the bus
    MPU->CESR = 0;

    // Enable all the ports
    SIM->SCGC5 |= ( SIM_SCGC5_PORTA_MASK
                  | SIM_SCGC5_PORTB_MASK
                  | SIM_SCGC5_PORTC_MASK
                  | SIM_SCGC5_PORTD_MASK
                  | SIM_SCGC5_PORTE_MASK );

#if DEBUG && defined(TOWER)
    // Enable the pins for the debug UART0
    BW_PORT_PCRn_MUX(HW_PORTA, 15, 3);   // UART0_RX is PTA15 in ALT3
    BW_PORT_PCRn_MUX(HW_PORTA, 14, 3);   // UART0_TX is PTA14 in ALT3
#endif // DEBUG && defined(TOWER)

    // Update SystemCoreClock. FOPT bits set the OUTDIV1 value.
    SystemCoreClock /= (HW_SIM_CLKDIV1.B.OUTDIV1 + 1);
}

void deinit_hardware(void)
{
    SIM->SCGC5 &= (uint32_t)~( SIM_SCGC5_PORTA_MASK
                  | SIM_SCGC5_PORTB_MASK
                  | SIM_SCGC5_PORTC_MASK
                  | SIM_SCGC5_PORTD_MASK
                  | SIM_SCGC5_PORTE_MASK );
}

bool usb_clock_init(void)
{
    // Select PLL clock
    SIM_SOPT2 |= (SIM_SOPT2_USBSRC_MASK);

    // Enable USB-OTG IP clocking
    SIM_SCGC4 |= (SIM_SCGC4_USBOTG_MASK);

    // Configure enable USB regulator for device
    SIM_SOPT1 |= SIM_SOPT1_USBREGEN_MASK;

    return true;
}

uint32_t get_bus_clock(void)
{
    uint32_t busClockDivider = ((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV2_MASK) >> SIM_CLKDIV1_OUTDIV2_SHIFT) + 1;
    return (SystemCoreClock / busClockDivider);
}

uint32_t get_uart_clock( unsigned int instance )
{
    switch(instance)
    {
        case 0:
        case 1:
            // UART0 and UART1 always use the system clock
            return SystemCoreClock;
        case 2:
        case 3:
        case 4:
        case 5:
            // UART2, UART3, UART4, and UART5 always use the bus clock
            return get_bus_clock();
        default:
            return 0;
    }
}

unsigned int read_autobaud_pin( unsigned int instance )
{
    switch(instance)
    {
        case 0:
#ifdef FREEDOM
            return (HW_GPIO_PDIR_RD(HW_GPIOB) >> UART0_RX_GPIO_PIN_NUM) & 1;
#endif // FREEDOM
        case 1:
#ifdef TOWER
            return (HW_GPIO_PDIR_RD(HW_GPIOC) >> UART1_RX_GPIO_PIN_NUM) & 1;
#endif // TOWER
        case 2:
        case 3:
        case 4:
            return 0;
        case 5:
#ifdef TOWER
            return (HW_GPIO_PDIR_RD(HW_GPIOE) >> UART5_RX_GPIO_PIN_NUM) & 1;
#endif // TOWER
        default:
            return 0;
    }
}

bool is_boot_pin_asserted(void)
{
#ifdef BL_TARGET_FLASH
    // Initialize PTC6 to Mode 1 for GPIO
    BW_PORT_PCRn_MUX(HW_PORTC, 6, 1);
    // Set PTC6 pin as an input
    HW_GPIO_PDDR_CLR(HW_GPIOC, 1 << 6);
    // Set PTC6 pin pullup enabled, pullup select, filter enable
    HW_PORT_PCRn_SET(HW_PORTC, 6, PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_PFE_MASK);

    unsigned int readCount = 0;

    // Sample the pin a number of times
    for (unsigned int i = 0; i < BOOT_PIN_DEBOUNCE_READ_COUNT; i++)
    {
        readCount += (HW_GPIO_PDIR_RD(HW_GPIOC) >> 6) & 1;
    }

    // PTC6 is pulled high so we are measuring lows, make sure most of our measurements
    // registered as low
    return (readCount < (BOOT_PIN_DEBOUNCE_READ_COUNT/2));
#else
    // Boot pin for Flash only target
    return false;
#endif
}

#ifdef FREEDOM
//! @brief this is going to be used for autobaud IRQ handling for UART0
void PORTB_IRQHandler(void)
{
    // Check if the pin for UART1 is what triggered the PORT C interrupt
    if (HW_PORT_PCRn(HW_PORTB, UART0_RX_GPIO_PIN_NUM).B.ISF && s_pin_irq_func[0])
    {
        s_pin_irq_func[0](0);
        HW_PORT_ISFR_WR(HW_PORTB, ~0U);
    }
}
#endif // FREEDOM

#ifdef TOWER
//! @brief this is going to be used for autobaud IRQ handling for UART1
void PORTC_IRQHandler(void)
{
    // Check if the pin for UART1 is what triggered the PORT C interrupt
    if (HW_PORT_PCRn(HW_PORTC, UART1_RX_GPIO_PIN_NUM).B.ISF && s_pin_irq_func[1])
    {
        s_pin_irq_func[1](1);
        HW_PORT_ISFR_WR(HW_PORTC, ~0U);
    }

}

//! @brief this is going to be used for autobaud IRQ handling for UART5
void PORTE_IRQHandler(void)
{
    // Check if the pin for UART5 is what triggered the PORT E interrupt
    if (HW_PORT_PCRn(HW_PORTE, UART5_RX_GPIO_PIN_NUM).B.ISF && s_pin_irq_func[5])
    {
        s_pin_irq_func[5](5);
        HW_PORT_ISFR_WR(HW_PORTE, ~0U);
    }
}
#endif // TOWER

void enable_autobaud_pin_irq(unsigned int instance, pin_irq_callback_t func)
{
    switch(instance)
    {
#ifdef FREEDOM
        case 0:
            NVIC_EnableIRQ(PORTB_IRQn);
            // Only look for a falling edge for our interrupts
            HW_PORT_PCRn(HW_PORTB, UART0_RX_GPIO_PIN_NUM).B.IRQC = PORT_IRQC_INTERRUPT_FALLING_EDGE;
            s_pin_irq_func[0] = func;
            break;
#endif // FREEDOM
#ifdef TOWER
        case 1:
            NVIC_EnableIRQ(PORTC_IRQn);
            // Only look for a falling edge for our interrupts
            HW_PORT_PCRn(HW_PORTC, UART1_RX_GPIO_PIN_NUM).B.IRQC = PORT_IRQC_INTERRUPT_FALLING_EDGE;
            s_pin_irq_func[1] = func;
            break;
        case 5:
            NVIC_EnableIRQ(PORTE_IRQn);
            // Only look for a falling edge for our interrupts
            HW_PORT_PCRn(HW_PORTE, UART5_RX_GPIO_PIN_NUM).B.IRQC = PORT_IRQC_INTERRUPT_FALLING_EDGE;
            s_pin_irq_func[5] = func;
            break;
#endif // TOWER
    }
}

void disable_autobaud_pin_irq(unsigned int instance)
{
    switch(instance)
    {
#ifdef FREEDOM
        case 0:
            NVIC_DisableIRQ(PORTB_IRQn);
            HW_PORT_PCRn(HW_PORTB, UART0_RX_GPIO_PIN_NUM).B.IRQC = PORT_IRQC_INTERRUPT_DISABLE;
            s_pin_irq_func[0] = 0;
            break;
#endif // FREEDOM
#ifdef TOWER
        case 1:
            NVIC_DisableIRQ(PORTC_IRQn);
            HW_PORT_PCRn(HW_PORTC, UART1_RX_GPIO_PIN_NUM).B.IRQC = PORT_IRQC_INTERRUPT_DISABLE;
            s_pin_irq_func[1] = 0;
            break;
        case 5:
            NVIC_DisableIRQ(PORTE_IRQn);
            HW_PORT_PCRn(HW_PORTE, UART5_RX_GPIO_PIN_NUM).B.IRQC = PORT_IRQC_INTERRUPT_DISABLE;
            s_pin_irq_func[5] = 0;
            break;
#endif // TOWER
    }
}

void dummy_byte_callback(uint8_t byte)
{
    (void)byte;
}

void debug_init(void)
{
#ifdef TOWER
    scuart_init(UART0, get_uart_clock(0), TERMINAL_BAUD, dummy_byte_callback);
#endif // TOWER
}

#if __ICCARM__

size_t __write(int handle, const unsigned char *buf, size_t size)
{
#ifdef TOWER
    while (size--)
    {
        scuart_putchar(UART0, *buf++);
    }
#endif // TOWER

    return size;
}

#endif // __ICCARM__

void update_available_peripherals()
{
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

