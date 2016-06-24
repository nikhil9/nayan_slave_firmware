/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

                                      ---

    A special exception to the GPL can be applied should you wish to distribute
    a combined work that includes ChibiOS/RT, without being obliged to provide
    the source code for any proprietary components. See the file exception.txt
    for full details of how and when the exception can be applied.
*/

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for STMicroelectronics STM32F4-Discovery board.
 */

/*
 * Board identifier.
 */
#define BOARD_NAYAN_AP_HLP
#define BOARD_NAME              "Nayan_AP_SLAVE"

#define NAYAN	0
#define V10		1
#define BOARD	V10 //'NAYAN' or 'V10'

#if BOARD == NAYAN
/*
 * Board frequencies.
 * NOTE: The LSE crystal is not fitted by default on the board.
 */
#define STM32_LSECLK            0
#define STM32_HSECLK            16000000

/*
 * Board voltages.
 * Required for performance limi1ts calculation.
 */
#define STM32_VDD               330

/*
 * MCU type as defined in the ST header file stm32f4xx.h.
 */
#define STM32F40_41xxx
/*#define STM32F4XX
*/
/*
 * IO pins assignments.
 */



#define GPIOB_SWO               3
#define GPIOB_SCL               6
#define GPIOB_SDA               9
#define GPIOB_SCK               10

#define GPIOH_OSC_IN            0
#define GPIOH_OSC_OUT           1

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_2M(n)            (0U << ((n) * 2))
#define PIN_OSPEED_25M(n)           (1U << ((n) * 2))
#define PIN_OSPEED_50M(n)           (2U << ((n) * 2))
#define PIN_OSPEED_100M(n)          (3U << ((n) * 2))
#define PIN_PUDR_FLOATING(n)        (0U << ((n) * 2))
#define PIN_PUDR_PULLUP(n)          (1U << ((n) * 2))
#define PIN_PUDR_PULLDOWN(n)        (2U << ((n) * 2))
#define PIN_AFIO_AF(n, v)           ((v##U) << ((n % 8) * 4))




/*
 * Port A setup.
 * PA0	- SERVO1-T2CH1		(AF1)
 * PA1	- SERVO6-T2CH2		(AF1)
 * PA2	- SD2-TX			(AF7)
 * PA3	- SD2-RX			(AF7)
 * PA4	- SPI1-NSS			(AF0)
 * PA5	- SPI1-SCK			(AF5)
 * PA6	- SPI1-MISO			(AF5)
 * PA7	- SPI1-MOSI			(AF5)
 * PA8	- I2C3-SCL			(AF4)
 * PA9	- OTG_FS_VBUS		(AF10)
 * PA10	- SERVO7-T1CH3		(AF1)
 * PA11	- OTG_FS_DM			(AF10)
 * PA12	- OTG_FS_DP			(AF10)
 * PA13 - SWDIO				(AF0)
 * PA14 - SWCLK				(AF0)
 * PA15 - JTDI				(AF0)
 */

#define VAL_GPIOA_MODER     (PIN_MODE_ALTERNATE(0) | \
                             PIN_MODE_ALTERNATE(1) | \
                             PIN_MODE_ALTERNATE(2) | \
                             PIN_MODE_ALTERNATE(3) | \
                             PIN_MODE_ALTERNATE(4) | \
                             PIN_MODE_ALTERNATE(5) | \
                             PIN_MODE_ALTERNATE(6) | \
                             PIN_MODE_ALTERNATE(7) | \
                             PIN_MODE_ALTERNATE(8) | \
                             PIN_MODE_ALTERNATE(9) | \
                             PIN_MODE_INPUT(10) | \
                             PIN_MODE_ALTERNATE(11) | \
                             PIN_MODE_ALTERNATE(12) | \
                             PIN_MODE_ALTERNATE(13) | \
                             PIN_MODE_ALTERNATE(14) | \
                             PIN_MODE_ALTERNATE(15))
#define VAL_GPIOA_OTYPER    (PIN_OTYPE_PUSHPULL(0) | \
							 PIN_OTYPE_PUSHPULL(1) | \
							 PIN_OTYPE_PUSHPULL(2) | \
							 PIN_OTYPE_PUSHPULL(3) | \
							 PIN_OTYPE_PUSHPULL(4) | \
							 PIN_OTYPE_PUSHPULL(5) | \
							 PIN_OTYPE_PUSHPULL(6) | \
							 PIN_OTYPE_PUSHPULL(7) | \
							 PIN_OTYPE_OPENDRAIN(8) | \
							 PIN_OTYPE_PUSHPULL(9) | \
							 PIN_OTYPE_PUSHPULL(10) | \
							 PIN_OTYPE_PUSHPULL(11) | \
							 PIN_OTYPE_PUSHPULL(12) | \
							 PIN_OTYPE_PUSHPULL(13) | \
							 PIN_OTYPE_PUSHPULL(14) | \
							 PIN_OTYPE_PUSHPULL(15))
#define VAL_GPIOA_OSPEEDR    (PIN_OSPEED_100M(0) | \
							 PIN_OSPEED_100M(1) | \
							 PIN_OSPEED_100M(2) | \
							 PIN_OSPEED_100M(3) | \
							 PIN_OSPEED_100M(4) | \
							 PIN_OSPEED_100M(5) | \
							 PIN_OSPEED_100M(6) | \
							 PIN_OSPEED_100M(7) | \
							 PIN_OSPEED_100M(8) | \
							 PIN_OSPEED_100M(9) | \
							 PIN_OSPEED_100M(10) | \
							 PIN_OSPEED_100M(11) | \
							 PIN_OSPEED_100M(12) | \
							 PIN_OSPEED_100M(13) | \
							 PIN_OSPEED_100M(14) | \
							 PIN_OSPEED_100M(15))

#define VAL_GPIOA_PUPDR     (PIN_PUDR_PULLDOWN(0) | \
                             PIN_PUDR_PULLDOWN(1) | \
                             PIN_PUDR_FLOATING(2) | \
                             PIN_PUDR_FLOATING(3) | \
                             PIN_PUDR_PULLUP(4) | \
                             PIN_PUDR_PULLUP(5) | \
                             PIN_PUDR_PULLUP(6) | \
                             PIN_PUDR_PULLUP(7) | \
                             PIN_PUDR_FLOATING(8) | \
                             PIN_PUDR_FLOATING(9) | \
                             PIN_PUDR_PULLDOWN(10) | \
                             PIN_PUDR_FLOATING(11) | \
                             PIN_PUDR_FLOATING(12) | \
                             PIN_PUDR_PULLUP(13) | \
                             PIN_PUDR_PULLDOWN(14) | \
                             PIN_PUDR_FLOATING(15))
#define VAL_GPIOA_ODR       (PIN_ODR_HIGH(0) | \
							 PIN_ODR_HIGH(1) | \
							 PIN_ODR_HIGH(2) | \
							 PIN_ODR_HIGH(3) | \
							 PIN_ODR_HIGH(4) | \
							 PIN_ODR_HIGH(5) | \
							 PIN_ODR_HIGH(6) | \
							 PIN_ODR_HIGH(7) | \
							 PIN_ODR_HIGH(8) | \
							 PIN_ODR_HIGH(9) | \
							 PIN_ODR_HIGH(10) | \
							 PIN_ODR_HIGH(11) | \
							 PIN_ODR_HIGH(12) | \
							 PIN_ODR_HIGH(13) | \
							 PIN_ODR_HIGH(14) | \
							 PIN_ODR_HIGH(15))
#define VAL_GPIOA_AFRL      (PIN_AFIO_AF(0, 1) | \
							 PIN_AFIO_AF(1, 1) | \
							 PIN_AFIO_AF(2, 7) | \
                             PIN_AFIO_AF(3, 7) | \
                             PIN_AFIO_AF(4, 0) | \
                             PIN_AFIO_AF(5, 5) | \
                             PIN_AFIO_AF(6, 5) | \
                             PIN_AFIO_AF(7, 5))
#define VAL_GPIOA_AFRH      (PIN_AFIO_AF(8, 4) | \
							 PIN_AFIO_AF(9, 1) | \
							 PIN_AFIO_AF(10, 0) | \
							 PIN_AFIO_AF(11, 10) | \
                             PIN_AFIO_AF(12, 10) | \
                             PIN_AFIO_AF(13, 0) | \
                             PIN_AFIO_AF(14, 0) | \
							 PIN_AFIO_AF(15, 0))



/*
 * Port B setup.
 * All input with pull-up except:
 * PB0	- RC6				(AF0)
 * PB1	- RC5				(AF0)
 * PB2	- BOOT-1			(AF0)
 * PB3  - SWO		        (AF0)
 * PB4  - NJRST		        (AF0)
 * PB5	- SERVO5-T3CH2		(AF2)
 * PB6	- SD1-TX			(AF7)
 * PB7	- SD1_RX			(AF7)
 * PB8  - I2C1-SCL    		(AF4)
 * PB9  - I2C1-SDA       	(AF4)
 * PB10 - I2C2-SCL			(AF4)
 * PB11	- I2C2-SDA			(AF4)
 * PB12	- RC1				(AF0)
 * PB13	- RC2				(AF0)
 * PB14	- RC3				(AF0)
 * PB15	- RC4				(AF0)
 *
 */
#define VAL_GPIOB_MODER     (PIN_MODE_INPUT(0) | \
                             PIN_MODE_INPUT(1) | \
                             PIN_MODE_INPUT(2) | \
                             PIN_MODE_ALTERNATE(3) | \
                             PIN_MODE_ALTERNATE(4) | \
                             PIN_MODE_ALTERNATE(5) | \
                             PIN_MODE_ALTERNATE(6) | \
                             PIN_MODE_ALTERNATE(7) | \
                             PIN_MODE_ALTERNATE(8) | \
                             PIN_MODE_ALTERNATE(9) | \
                             PIN_MODE_ALTERNATE(10) | \
                             PIN_MODE_ALTERNATE(11) | \
                             PIN_MODE_OUTPUT(12) | \
                             PIN_MODE_INPUT(13) | \
                             PIN_MODE_INPUT(14) | \
                             PIN_MODE_INPUT(15))
#define VAL_GPIOB_OTYPER    (PIN_OTYPE_PUSHPULL(0) | \
                             PIN_OTYPE_PUSHPULL(1) | \
							 PIN_OTYPE_OPENDRAIN(2) | \
							 PIN_OTYPE_PUSHPULL(3) | \
							 PIN_OTYPE_PUSHPULL(4) | \
							 PIN_OTYPE_PUSHPULL(5) | \
							 PIN_OTYPE_PUSHPULL(6) | \
							 PIN_OTYPE_PUSHPULL(7) | \
							 PIN_OTYPE_OPENDRAIN(8) | \
							 PIN_OTYPE_OPENDRAIN(9) | \
							 PIN_OTYPE_OPENDRAIN(10) | \
							 PIN_OTYPE_OPENDRAIN(11) | \
							 PIN_OTYPE_PUSHPULL(12) | \
							 PIN_OTYPE_PUSHPULL(13) | \
							 PIN_OTYPE_PUSHPULL(14) | \
							 PIN_OTYPE_PUSHPULL(15))
#define VAL_GPIOB_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOB_PUPDR     (PIN_PUDR_PULLDOWN(0) | \
                             PIN_PUDR_PULLDOWN(1) | \
                             PIN_PUDR_PULLUP(2) | \
                             PIN_PUDR_FLOATING(3) | \
                             PIN_PUDR_FLOATING(4) | \
                             PIN_PUDR_PULLDOWN(5) | \
                             PIN_PUDR_FLOATING(6) | \
                             PIN_PUDR_FLOATING(7) | \
                             PIN_PUDR_FLOATING(8) | \
                             PIN_PUDR_FLOATING(9) | \
                             PIN_PUDR_FLOATING(10) | \
                             PIN_PUDR_FLOATING(11) | \
                             PIN_PUDR_FLOATING(12) | \
                             PIN_PUDR_PULLDOWN(13) | \
                             PIN_PUDR_PULLDOWN(14) | \
                             PIN_PUDR_PULLDOWN(15))
#define VAL_GPIOB_ODR       0xFFFFFFFF
#define VAL_GPIOB_AFRL      (PIN_AFIO_AF(0, 0) | \
							 PIN_AFIO_AF(1, 0) | \
							 PIN_AFIO_AF(2, 0) | \
							 PIN_AFIO_AF(3, 0) | \
							 PIN_AFIO_AF(4, 0) | \
							 PIN_AFIO_AF(5, 2) | \
							 PIN_AFIO_AF(6, 7) | \
							 PIN_AFIO_AF(7, 7))
#define VAL_GPIOB_AFRH      (PIN_AFIO_AF(8, 4) | \
							 PIN_AFIO_AF(9, 4) | \
							 PIN_AFIO_AF(10, 4) | \
							 PIN_AFIO_AF(11, 4) | \
							 PIN_AFIO_AF(12, 0) | \
							 PIN_AFIO_AF(13, 0) | \
							 PIN_AFIO_AF(14, 0) | \
		                     PIN_AFIO_AF(15, 0))



/*
 * Port C setup.
 * PC0	- LED			(AF0)
 * PC1	- LED			(AF0)
 * PC2	- A1			(AF0)
 * PC3	- A2			(AF0)
 * PC4	- A3			(AF0)
 * PC5	- A4			(AF0)
 * PC6	- SERVO3-T8CH1	(AF3)
 * PC7	- SERVO4-T8CH2	(AF3)
 * PC8	- SERVO5-T8CH3	(AF3)
 * PC9	- I2C3-SDA		(AF4)
 * PC10	- SD4-TX		(AF7)
 * PC11	- SD4-RX		(AF7)
 * PC12	- LED			(AF0)
 * PC13	- LED			(AF0)
 * PC14	- NA			(AF0)
 * PC15	- NA			(AF0)
 *
 */

#define VAL_GPIOC_MODER     (PIN_MODE_OUTPUT(0) | \
                             PIN_MODE_OUTPUT(1) | \
                             PIN_MODE_ALTERNATE(2) | \
                             PIN_MODE_ALTERNATE(3) | \
                             PIN_MODE_ANALOG(4) | \
                             PIN_MODE_ANALOG(5) | \
                             PIN_MODE_ALTERNATE(6) | \
                             PIN_MODE_ALTERNATE(7) | \
                             PIN_MODE_ALTERNATE(8) | \
                             PIN_MODE_ALTERNATE(9) | \
                             PIN_MODE_ALTERNATE(10) | \
                             PIN_MODE_ALTERNATE(11) | \
                             PIN_MODE_ALTERNATE(12) | \
                             PIN_MODE_OUTPUT(13) | \
                             PIN_MODE_ALTERNATE(14) | \
                             PIN_MODE_ALTERNATE(15))
#define VAL_GPIOC_OTYPER    0x00000000
#define VAL_GPIOC_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOC_PUPDR     (PIN_PUDR_PULLDOWN(0) | \
                             PIN_PUDR_PULLDOWN(1) | \
                             PIN_PUDR_FLOATING(2) | \
                             PIN_PUDR_FLOATING(3) | \
                             PIN_PUDR_FLOATING(4) | \
                             PIN_PUDR_FLOATING(5) | \
                             PIN_PUDR_PULLDOWN(6) | \
                             PIN_PUDR_PULLDOWN(7) | \
                             PIN_PUDR_PULLDOWN(8) | \
                             PIN_PUDR_FLOATING(9) | \
                             PIN_PUDR_FLOATING(10) | \
                             PIN_PUDR_FLOATING(11) | \
                             PIN_PUDR_FLOATING(12) | \
                             PIN_PUDR_PULLDOWN(13) | \
                             PIN_PUDR_FLOATING(14) | \
                             PIN_PUDR_FLOATING(15))
#define VAL_GPIOC_ODR       0xFFFFFFFF
#define VAL_GPIOC_AFRL      (PIN_AFIO_AF(1, 0) | \
							 PIN_AFIO_AF(2, 0) | \
							 PIN_AFIO_AF(3, 0) | \
							 PIN_AFIO_AF(4, 0) | \
							 PIN_AFIO_AF(5, 0) | \
							 PIN_AFIO_AF(6, 3) | \
							 PIN_AFIO_AF(7, 3))
#define VAL_GPIOC_AFRH		(PIN_AFIO_AF(8, 3) | \
							 PIN_AFIO_AF(9, 4) | \
							 PIN_AFIO_AF(10, 7) | \
							 PIN_AFIO_AF(11, 7) | \
							 PIN_AFIO_AF(12, 8) | \
							 PIN_AFIO_AF(13, 0) | \
							 PIN_AFIO_AF(14, 0) | \
							 PIN_AFIO_AF(15, 0) )


/*
 * Port D setup.
 */
#define GPIOD_TELE_RX             2 /* Telemetry UART5 RX (AF8) */

/* All input with pull-up
 */
#define VAL_GPIOD_MODER             PIN_MODE_ALTERNATE(GPIOD_TELE_RX)
#define VAL_GPIOD_OTYPER            0x00000000
#define VAL_GPIOD_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOD_PUPDR             PIN_PUDR_FLOATING(GPIOD_TELE_RX)
#define VAL_GPIOD_ODR               0xFFFFFFFF
#define VAL_GPIOD_AFRL              PIN_AFIO_AF(GPIOD_TELE_RX, 8)
#define VAL_GPIOD_AFRH              0x00000000


/*
 * Port E setup.
 * All input with pull-up
 */
#define VAL_GPIOE_MODER             0x00000000
#define VAL_GPIOE_OTYPER            0x00000000
#define VAL_GPIOE_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOE_PUPDR             0xFFFFFFFF
#define VAL_GPIOE_ODR               0xFFFFFFFF
#define VAL_GPIOE_AFRL              0x00000000
#define VAL_GPIOE_AFRH              0x00000000

/*
 * Port F setup.
 * All input with pull-up.
 */
#define VAL_GPIOF_MODER             0x00000000
#define VAL_GPIOF_OTYPER            0x00000000
#define VAL_GPIOF_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOF_PUPDR             0xFFFFFFFF
#define VAL_GPIOF_ODR               0xFFFFFFFF
#define VAL_GPIOF_AFRL              0x00000000
#define VAL_GPIOF_AFRH              0x00000000

/*
 * Port G setup.
 * All input with pull-up.
 */
#define VAL_GPIOG_MODER             0x00000000
#define VAL_GPIOG_OTYPER            0x00000000
#define VAL_GPIOG_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOG_PUPDR             0xFFFFFFFF
#define VAL_GPIOG_ODR               0xFFFFFFFF
#define VAL_GPIOG_AFRL              0x00000000
#define VAL_GPIOG_AFRH              0x00000000


/*
 * Port H setup.
 * All input with pull-up except:
 * PH0  - GPIOH_OSC_IN          (input floating).
 * PH1  - GPIOH_OSC_OUT         (input floating).
 */
#define VAL_GPIOH_MODER     (PIN_MODE_INPUT(GPIOH_OSC_IN) |                 \
                             PIN_MODE_INPUT(GPIOH_OSC_OUT) |                \
                             PIN_MODE_INPUT(2) |                            \
                             PIN_MODE_INPUT(3) |                            \
                             PIN_MODE_INPUT(4) |                            \
                             PIN_MODE_INPUT(5) |                            \
                             PIN_MODE_INPUT(6) |                            \
                             PIN_MODE_INPUT(7) |                            \
                             PIN_MODE_INPUT(8) |                            \
                             PIN_MODE_INPUT(9) |                            \
                             PIN_MODE_INPUT(10) |                           \
                             PIN_MODE_INPUT(11) |                           \
                             PIN_MODE_INPUT(12) |                           \
                             PIN_MODE_INPUT(13) |                           \
                             PIN_MODE_INPUT(14) |                           \
                             PIN_MODE_INPUT(15))
#define VAL_GPIOH_OTYPER    0x00000000
#define VAL_GPIOH_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOH_PUPDR     (PIN_PUDR_FLOATING(GPIOH_OSC_IN) |              \
                             PIN_PUDR_FLOATING(GPIOH_OSC_OUT) |             \
                             PIN_PUDR_PULLUP(2) |                           \
                             PIN_PUDR_PULLUP(3) |                           \
                             PIN_PUDR_PULLUP(4) |                           \
                             PIN_PUDR_PULLUP(5) |                           \
                             PIN_PUDR_PULLUP(6) |                           \
                             PIN_PUDR_PULLUP(7) |                           \
                             PIN_PUDR_PULLUP(8) |                           \
                             PIN_PUDR_PULLUP(9) |                           \
                             PIN_PUDR_PULLUP(10) |                          \
                             PIN_PUDR_PULLUP(11) |                          \
                             PIN_PUDR_PULLUP(12) |                          \
                             PIN_PUDR_PULLUP(13) |                          \
                             PIN_PUDR_PULLUP(14) |                          \
                             PIN_PUDR_PULLUP(15))
#define VAL_GPIOH_ODR       0xFFFFFFFF
#define VAL_GPIOH_AFRL      0x00000000
#define VAL_GPIOH_AFRH      0x00000000
/*
 * Port I setup.
 * All input with pull-up.
 */
#define VAL_GPIOI_MODER             0x00000000
#define VAL_GPIOI_OTYPER            0x00000000
#define VAL_GPIOI_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOI_PUPDR             0xFFFFFFFF
#define VAL_GPIOI_ODR               0xFFFFFFFF
#define VAL_GPIOI_AFRL              0x00000000
#define VAL_GPIOI_AFRH              0x00000000

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */
#elif BOARD == V10

/*
 * Setup for STMicroelectronics STM32F4-Discovery board.
 */

/*
 * Board identifier.
 */

#define BOARD_NAME              "V10 Master"

/*
 * Board frequencies.
 * NOTE: The LSE crystal is not fitted by default on the board.
 */
#define STM32_LSECLK            0


#define STM32_HSECLK            16000000


/*
 * Board voltages.
 * Required for performance limi1ts calculation.
 */
#define STM32_VDD               330

/*
 * MCU type as defined in the ST header file stm32f4xx.h.
 */
//#define STM32F40_41xxx

#define STM32F427_437xx

/*
*/
/*
 * IO pins assignments.
 */


/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_2M(n)            (0U << ((n) * 2))
#define PIN_OSPEED_25M(n)           (1U << ((n) * 2))
#define PIN_OSPEED_50M(n)           (2U << ((n) * 2))
#define PIN_OSPEED_100M(n)          (3U << ((n) * 2))
#define PIN_PUDR_FLOATING(n)        (0U << ((n) * 2))
#define PIN_PUDR_PULLUP(n)          (1U << ((n) * 2))
#define PIN_PUDR_PULLDOWN(n)        (2U << ((n) * 2))
#define PIN_AFIO_AF(n, v)           ((v##U) << ((n % 8) * 4))




/*
 * Port A setup.
 * PA0	- USART4_TX			(AF8)
 * PA1	- USART4_RX			(AF8)
 * PA2	- TIM2_CH3			(AF1)
 * PA3	- TIM2_CH4			(AF1)
 * PA4	- SPI1-NSS			(AF5) OUTPUT PUSHPULL PULLUP
 * PA5	- SPI1-SCK			(AF5)
 * PA6	- SPI1-MISO			(AF5)
 * PA7	- SPI1-MOSI			(AF5)
 * PA8	- NA
 * PA9	- OTG_FS_VBUS		(AF10)
 * PA10	- OTG_FS_ID			(AF10)
 * PA11	- OTG_FS_DM			(AF10)
 * PA12	- OTG_FS_DP			(AF10)
 * PA13 - JTMS-SWDIO		(AF0)
 * PA14 - JTCK-SWCLK		(AF0)
 * PA15 - JTDI				(AF0)
 */

#define VAL_GPIOA_MODER     (PIN_MODE_ALTERNATE(0) | \
                             PIN_MODE_ALTERNATE(1) | \
                             PIN_MODE_INPUT(2) | \
                             PIN_MODE_INPUT(3) | \
                             PIN_MODE_OUTPUT(4) | \
                             PIN_MODE_ALTERNATE(5) | \
                             PIN_MODE_ALTERNATE(6) | \
                             PIN_MODE_ALTERNATE(7) | \
                             PIN_MODE_ALTERNATE(8) | \
                             PIN_MODE_INPUT(9) | \
                             PIN_MODE_ALTERNATE(10) | \
                             PIN_MODE_ALTERNATE(11) | \
                             PIN_MODE_ALTERNATE(12) | \
                             PIN_MODE_ALTERNATE(13) | \
                             PIN_MODE_ALTERNATE(14) | \
                             PIN_MODE_ALTERNATE(15))
#define VAL_GPIOA_OTYPER    (PIN_OTYPE_PUSHPULL(0) | \
							 PIN_OTYPE_PUSHPULL(1) | \
							 PIN_OTYPE_PUSHPULL(2) | \
							 PIN_OTYPE_PUSHPULL(3) | \
							 PIN_OTYPE_PUSHPULL(4) | \
							 PIN_OTYPE_PUSHPULL(5) | \
							 PIN_OTYPE_PUSHPULL(6) | \
							 PIN_OTYPE_PUSHPULL(7) | \
							 PIN_OTYPE_PUSHPULL(8) | \
							 PIN_OTYPE_PUSHPULL(9) | \
							 PIN_OTYPE_PUSHPULL(10) | \
							 PIN_OTYPE_PUSHPULL(11) | \
							 PIN_OTYPE_PUSHPULL(12) | \
							 PIN_OTYPE_PUSHPULL(13) | \
							 PIN_OTYPE_PUSHPULL(14) | \
							 PIN_OTYPE_PUSHPULL(15))
#define VAL_GPIOA_OSPEEDR    (PIN_OSPEED_100M(0) | \
							 PIN_OSPEED_100M(1) | \
							 PIN_OSPEED_100M(2) | \
							 PIN_OSPEED_100M(3) | \
							 PIN_OSPEED_100M(4) | \
							 PIN_OSPEED_100M(5) | \
							 PIN_OSPEED_100M(6) | \
							 PIN_OSPEED_100M(7) | \
							 PIN_OSPEED_100M(8) | \
							 PIN_OSPEED_100M(9) | \
							 PIN_OSPEED_100M(10) | \
							 PIN_OSPEED_100M(11) | \
							 PIN_OSPEED_100M(12) | \
							 PIN_OSPEED_100M(13) | \
							 PIN_OSPEED_100M(14) | \
							 PIN_OSPEED_100M(15))

#define VAL_GPIOA_PUPDR     (PIN_PUDR_FLOATING(0) | \
                             PIN_PUDR_FLOATING(1) | \
                             PIN_PUDR_PULLDOWN(2) | \
                             PIN_PUDR_PULLDOWN(3) | \
                             PIN_PUDR_PULLUP(4) | \
                             PIN_PUDR_PULLUP(5) | \
                             PIN_PUDR_PULLUP(6) | \
                             PIN_PUDR_PULLUP(7) | \
                             PIN_PUDR_FLOATING(8) | \
                             PIN_PUDR_FLOATING(9) | \
                             PIN_PUDR_FLOATING(10) | \
                             PIN_PUDR_FLOATING(11) | \
                             PIN_PUDR_FLOATING(12) | \
                             PIN_PUDR_PULLUP(13) | \
                             PIN_PUDR_PULLDOWN(14) | \
                             PIN_PUDR_FLOATING(15))
#define VAL_GPIOA_ODR       (PIN_ODR_HIGH(0) | \
							 PIN_ODR_HIGH(1) | \
							 PIN_ODR_HIGH(2) | \
							 PIN_ODR_HIGH(3) | \
							 PIN_ODR_HIGH(4) | \
							 PIN_ODR_HIGH(5) | \
							 PIN_ODR_HIGH(6) | \
							 PIN_ODR_HIGH(7) | \
							 PIN_ODR_HIGH(8) | \
							 PIN_ODR_HIGH(9) | \
							 PIN_ODR_HIGH(10) | \
							 PIN_ODR_HIGH(11) | \
							 PIN_ODR_HIGH(12) | \
							 PIN_ODR_HIGH(13) | \
							 PIN_ODR_HIGH(14) | \
							 PIN_ODR_HIGH(15))
#define VAL_GPIOA_AFRL      (PIN_AFIO_AF(0, 8) | \
							 PIN_AFIO_AF(1, 8) | \
							 PIN_AFIO_AF(2, 0) | \
                             PIN_AFIO_AF(3, 0) | \
                             PIN_AFIO_AF(4, 0) | \
                             PIN_AFIO_AF(5, 5) | \
                             PIN_AFIO_AF(6, 5) | \
                             PIN_AFIO_AF(7, 5))
#define VAL_GPIOA_AFRH      (PIN_AFIO_AF(8, 0) | \
							 PIN_AFIO_AF(9, 10) | \
							 PIN_AFIO_AF(10, 10) | \
							 PIN_AFIO_AF(11, 10) | \
                             PIN_AFIO_AF(12, 10) | \
                             PIN_AFIO_AF(13, 0) | \
                             PIN_AFIO_AF(14, 0) | \
							 PIN_AFIO_AF(15, 0))



/*
 * Port B setup.
 * All input with pull-up except:
 * PB0	- TIM3-CH3			(AF2)
 * PB1	- TIM3-CH4			(AF2)
 * PB2	- BOOT-1			(AF0)
 * PB3  - JTDO-SWO		    (AF0)
 * PB4  - NJRST		        (AF0)
 * PB5	- NA
 * PB6	- USART1-TX			(AF7)
 * PB7	- USART1-RX			(AF7)
 * PB8  - I2C1-SCL    		(AF4)
 * PB9  - I2C1-SDA       	(AF4)
 * PB10 - I2C2-SCL			(AF4)
 * PB11	- I2C2-SDA			(AF4)
 * PB12	- SPI2_MSS			(AF5)
 * PB13	- SPI2_SCK			(AF5)
 * PB14	- SPI2_MISO			(AF5)
 * PB15	- SPI2_MOSI			(AF5)
 *
 */
#define VAL_GPIOB_MODER     (PIN_MODE_INPUT(0) | \
                             PIN_MODE_INPUT(1) | \
                             PIN_MODE_ALTERNATE(2) | \
                             PIN_MODE_ALTERNATE(3) | \
                             PIN_MODE_ALTERNATE(4) | \
                             PIN_MODE_ALTERNATE(5) | \
                             PIN_MODE_ALTERNATE(6) | \
                             PIN_MODE_ALTERNATE(7) | \
                             PIN_MODE_ALTERNATE(8) | \
                             PIN_MODE_ALTERNATE(9) | \
                             PIN_MODE_ALTERNATE(10) | \
                             PIN_MODE_ALTERNATE(11) | \
                             PIN_MODE_ALTERNATE(12) | \
                             PIN_MODE_ALTERNATE(13) | \
                             PIN_MODE_ALTERNATE(14) | \
                             PIN_MODE_ALTERNATE(15))
#define VAL_GPIOB_OTYPER    (PIN_OTYPE_PUSHPULL(0) | \
                             PIN_OTYPE_PUSHPULL(1) | \
							 PIN_OTYPE_PUSHPULL(2) | \
							 PIN_OTYPE_PUSHPULL(3) | \
							 PIN_OTYPE_PUSHPULL(4) | \
							 PIN_OTYPE_PUSHPULL(5) | \
							 PIN_OTYPE_PUSHPULL(6) | \
							 PIN_OTYPE_PUSHPULL(7) | \
							 PIN_OTYPE_OPENDRAIN(8) | \
							 PIN_OTYPE_OPENDRAIN(9) | \
							 PIN_OTYPE_OPENDRAIN(10) | \
							 PIN_OTYPE_OPENDRAIN(11) | \
							 PIN_OTYPE_PUSHPULL(12) | \
							 PIN_OTYPE_PUSHPULL(13) | \
							 PIN_OTYPE_PUSHPULL(14) | \
							 PIN_OTYPE_PUSHPULL(15))
#define VAL_GPIOB_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOB_PUPDR     (PIN_PUDR_PULLDOWN(0) | \
                             PIN_PUDR_PULLDOWN(1) | \
                             PIN_PUDR_PULLDOWN(2) | \
                             PIN_PUDR_FLOATING(3) | \
                             PIN_PUDR_FLOATING(4) | \
                             PIN_PUDR_FLOATING(5) | \
                             PIN_PUDR_FLOATING(6) | \
                             PIN_PUDR_FLOATING(7) | \
                             PIN_PUDR_FLOATING(8) | \
                             PIN_PUDR_FLOATING(9) | \
                             PIN_PUDR_FLOATING(10) | \
                             PIN_PUDR_FLOATING(11) | \
                             PIN_PUDR_PULLUP(12) | \
                             PIN_PUDR_PULLUP(13) | \
                             PIN_PUDR_PULLUP(14) | \
                             PIN_PUDR_PULLUP(15))
#define VAL_GPIOB_ODR       0xFFFFFFFF
#define VAL_GPIOB_AFRL      (PIN_AFIO_AF(0, 0) | \
							 PIN_AFIO_AF(1, 0) | \
							 PIN_AFIO_AF(2, 0) | \
							 PIN_AFIO_AF(3, 0) | \
							 PIN_AFIO_AF(4, 0) | \
							 PIN_AFIO_AF(5, 0) | \
							 PIN_AFIO_AF(6, 7) | \
							 PIN_AFIO_AF(7, 7))
#define VAL_GPIOB_AFRH      (PIN_AFIO_AF(8, 4) | \
							 PIN_AFIO_AF(9, 4) | \
							 PIN_AFIO_AF(10, 4) | \
							 PIN_AFIO_AF(11, 4) | \
							 PIN_AFIO_AF(12, 5) | \
							 PIN_AFIO_AF(13, 5) | \
							 PIN_AFIO_AF(14, 5) | \
		                     PIN_AFIO_AF(15, 5))



/*
 * Port C setup.
 * PC0	- ADC1-IN10		(AF0)
 * PC1	- ADC1-IN11		(AF0)
 * PC2	- ADC1-IN12		(AF0)
 * PC3	- ADC1-IN13		(AF0)
 * PC4	- ADC1-IN14		(AF0)
 * PC5	- NA			(AF0)
 * PC6	- TIM3-CH1		(AF2)RC5
 * PC7	- TIM3-CH2		(AF2)RC6
 * PC8	- SDIO			(AF12)
 * PC9	- SDIO			(AF12)
 * PC10	- SDIO-D2		(AF12)
 * PC11	- SDIO-D3		(AF12)
 * PC12	- SDIO-CK		(AF12)
 * PC13	- NA			(AF0)
 * PC14	- NA			(AF0)
 * PC15	- NA			(AF0)
 *
 */

#define VAL_GPIOC_MODER     (PIN_MODE_ALTERNATE(0) | \
                             PIN_MODE_ALTERNATE(1) | \
                             PIN_MODE_ALTERNATE(2) | \
                             PIN_MODE_ALTERNATE(3) | \
                             PIN_MODE_ALTERNATE(4) | \
                             PIN_MODE_ALTERNATE(5) | \
							 PIN_MODE_ALTERNATE(6) | \
							 PIN_MODE_ALTERNATE(7) | \
                             PIN_MODE_ALTERNATE(8) | \
                             PIN_MODE_ALTERNATE(9) | \
                             PIN_MODE_ALTERNATE(10) | \
                             PIN_MODE_ALTERNATE(11) | \
                             PIN_MODE_ALTERNATE(12) | \
                             PIN_MODE_ALTERNATE(13) | \
                             PIN_MODE_ALTERNATE(14) | \
                             PIN_MODE_ALTERNATE(15))
#define VAL_GPIOC_OTYPER    0x00000000
#define VAL_GPIOC_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOC_PUPDR     (PIN_PUDR_FLOATING(0) | \
                             PIN_PUDR_FLOATING(1) | \
                             PIN_PUDR_FLOATING(2) | \
                             PIN_PUDR_FLOATING(3) | \
                             PIN_PUDR_FLOATING(4) | \
                             PIN_PUDR_FLOATING(5) | \
                             PIN_PUDR_PULLDOWN(6) | \
                             PIN_PUDR_PULLDOWN(7) | \
                             PIN_PUDR_FLOATING(8) | \
                             PIN_PUDR_FLOATING(9) | \
                             PIN_PUDR_FLOATING(10) | \
                             PIN_PUDR_FLOATING(11) | \
                             PIN_PUDR_FLOATING(12) | \
                             PIN_PUDR_FLOATING(13) | \
                             PIN_PUDR_FLOATING(14) | \
                             PIN_PUDR_FLOATING(15))
#define VAL_GPIOC_ODR       0xFFFFFFFF
#define VAL_GPIOC_AFRL      (PIN_AFIO_AF(0, 0) | \
							 PIN_AFIO_AF(1, 0) | \
							 PIN_AFIO_AF(2, 0) | \
							 PIN_AFIO_AF(3, 0) | \
							 PIN_AFIO_AF(4, 0) | \
							 PIN_AFIO_AF(5, 0) | \
							 PIN_AFIO_AF(6, 2) | \
							 PIN_AFIO_AF(7, 2))
#define VAL_GPIOC_AFRH		(PIN_AFIO_AF(8, 12) | \
							 PIN_AFIO_AF(9, 12) | \
							 PIN_AFIO_AF(10, 12) | \
							 PIN_AFIO_AF(11, 12) | \
							 PIN_AFIO_AF(12, 12) | \
							 PIN_AFIO_AF(13, 0) | \
							 PIN_AFIO_AF(14, 0) | \
							 PIN_AFIO_AF(15, 0) )



/*
 * Port D setup.
 * PD0	- LED1		(AF0)
 * PD1	- LED2		(AF0)
 * PD2	- SDIO-CMD		(AF12)
 * PD3	- LED			(AF0)
 * PD4	- LED			(AF0)
 * PD5	- USART2-TX		(AF7)
 * PD6	- USART2-RX		(AF7)
 * PD7	- NA
 * PD8	- USART3-TX		(AF7)
 * PD9	- USART3-RX		(AF7)
 * PD10	- LSM_CS_G		(AF0)
 * PD11	- LSM_CS_X		(AF0)
 * PD12	- TIM4-CH1		(AF2)
 * PD13	- TIM4-CH2		(AF2)
 * PD14	- TIM4-CH3		(AF2)
 * PD15	- TIM4-CH4		(AF2)
 *
 */

/* All input with pull-up
 */
#define VAL_GPIOD_MODER     (PIN_MODE_OUTPUT(0) | \
                             PIN_MODE_OUTPUT(1) | \
                             PIN_MODE_ALTERNATE(2) | \
                             PIN_MODE_OUTPUT(3) | \
                             PIN_MODE_OUTPUT(4) | \
                             PIN_MODE_ALTERNATE(5) | \
                             PIN_MODE_ALTERNATE(6) | \
                             PIN_MODE_ALTERNATE(7) | \
                             PIN_MODE_ALTERNATE(8) | \
                             PIN_MODE_ALTERNATE(9) | \
                             PIN_MODE_ALTERNATE(10) | \
                             PIN_MODE_ALTERNATE(11) | \
							 PIN_MODE_INPUT(12) | \
							 PIN_MODE_INPUT(13) | \
							 PIN_MODE_INPUT(14) | \
                             PIN_MODE_INPUT(15))
#define VAL_GPIOD_OTYPER    0x00000000
#define VAL_GPIOD_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOD_PUPDR     (PIN_PUDR_PULLDOWN(0) | \
                             PIN_PUDR_PULLDOWN(1) | \
                             PIN_PUDR_FLOATING(2) | \
                             PIN_PUDR_PULLDOWN(3) | \
                             PIN_PUDR_PULLDOWN(4) | \
                             PIN_PUDR_FLOATING(5) | \
                             PIN_PUDR_FLOATING(6) | \
                             PIN_PUDR_FLOATING(7) | \
                             PIN_PUDR_FLOATING(8) | \
                             PIN_PUDR_FLOATING(9) | \
                             PIN_PUDR_FLOATING(10) | \
                             PIN_PUDR_FLOATING(11) | \
							 PIN_PUDR_PULLDOWN(12) | \
							 PIN_PUDR_PULLDOWN(13) | \
							 PIN_PUDR_PULLDOWN(14) | \
							 PIN_PUDR_PULLDOWN(15))
#define VAL_GPIOD_ODR       0xFFFFFFFF
#define VAL_GPIOD_AFRL      (PIN_AFIO_AF(0, 0) | \
							 PIN_AFIO_AF(1, 0) | \
							 PIN_AFIO_AF(2, 12) | \
							 PIN_AFIO_AF(3, 0) | \
							 PIN_AFIO_AF(4, 0) | \
							 PIN_AFIO_AF(5, 7) | \
							 PIN_AFIO_AF(6, 7) | \
							 PIN_AFIO_AF(7, 0))
#define VAL_GPIOD_AFRH		(PIN_AFIO_AF(8, 7) | \
							 PIN_AFIO_AF(9, 7) | \
							 PIN_AFIO_AF(10, 0) | \
							 PIN_AFIO_AF(11, 0) | \
							 PIN_AFIO_AF(12, 0) | \
							 PIN_AFIO_AF(13, 0) | \
							 PIN_AFIO_AF(14, 0) | \
							 PIN_AFIO_AF(15, 0) )


/*
 * Port E setup.
 * PE0	- LED			(AF0)
 * PE1	- LED			(AF0)
 * PE2	- SPI4-SCK		(AF5)
 * PE3	- NA			(AF5)
 * PE4	- SPI4-NSS		(AF5)
 * PE5	- SPI4-MISO		(AF5)
 * PE6	- SPI4-MOSI		(AF5)
 * PE7	- SBUS_SWITCH	(AF0)
 * PE8	- ST-INT		(AF0)
 * PE9	- TIM1-CH1		(AF0)RC1
 * PE10	- NA			(AF0)
 * PE11	- TIM1-CH2		(AF0)RC2
 * PE12	- NA			(AF0)
 * PE13	- TIM1-CH3		(AF0)RC3
 * PE14	- TIM1-CH4		(AF0)RC4
 * PE15	- NA			(AF0)
 *
 */

/* All input with pull-up
 */
#define VAL_GPIOE_MODER     (PIN_MODE_OUTPUT(0) | \
                             PIN_MODE_OUTPUT(1) | \
                             PIN_MODE_ALTERNATE(2) | \
                             PIN_MODE_OUTPUT(3) | \
                             PIN_MODE_OUTPUT(4) | \
                             PIN_MODE_ALTERNATE(5) | \
                             PIN_MODE_ALTERNATE(6) | \
                             PIN_MODE_OUTPUT(7) | \
                             PIN_MODE_ALTERNATE(8) | \
							 PIN_MODE_ALTERNATE(9) | \
                             PIN_MODE_ALTERNATE(10) | \
							 PIN_MODE_ALTERNATE(11) | \
							 PIN_MODE_OUTPUT(12) | \
							 PIN_MODE_ALTERNATE(13) | \
							 PIN_MODE_ALTERNATE(14) | \
                             PIN_MODE_ALTERNATE(15))
#define VAL_GPIOE_OTYPER    0x00000000
#define VAL_GPIOE_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOE_PUPDR     (PIN_PUDR_PULLDOWN(0) | \
                             PIN_PUDR_PULLDOWN(1) | \
                             PIN_PUDR_PULLUP(2) | \
                             PIN_PUDR_PULLUP(3) | \
                             PIN_PUDR_PULLUP(4) | \
                             PIN_PUDR_PULLUP(5) | \
                             PIN_PUDR_PULLUP(6) | \
                             PIN_PUDR_FLOATING(7) | \
                             PIN_PUDR_FLOATING(8) | \
                             PIN_PUDR_PULLDOWN(9) | \
                             PIN_PUDR_FLOATING(10) | \
                             PIN_PUDR_PULLDOWN(11) | \
                             PIN_PUDR_FLOATING(12) | \
                             PIN_PUDR_PULLDOWN(13) | \
                             PIN_PUDR_PULLDOWN(14) | \
                             PIN_PUDR_FLOATING(15))
#define VAL_GPIOE_ODR       0xFFFFFFFF
#define VAL_GPIOE_AFRL      (PIN_AFIO_AF(0, 0) | \
							 PIN_AFIO_AF(1, 0) | \
							 PIN_AFIO_AF(2, 5) | \
							 PIN_AFIO_AF(3, 0) | \
							 PIN_AFIO_AF(4, 0) | \
							 PIN_AFIO_AF(5, 5) | \
							 PIN_AFIO_AF(6, 5) | \
							 PIN_AFIO_AF(7, 0))
#define VAL_GPIOE_AFRH		(PIN_AFIO_AF(8, 0) | \
							 PIN_AFIO_AF(9, 1) | \
							 PIN_AFIO_AF(10, 0) | \
							 PIN_AFIO_AF(11, 1) | \
							 PIN_AFIO_AF(12, 0) | \
							 PIN_AFIO_AF(13, 1) | \
							 PIN_AFIO_AF(14, 1) | \
							 PIN_AFIO_AF(15, 0) )

/*
 * Port F setup.
 * All input with pull-up.
 */
#define VAL_GPIOF_MODER             0x00000000
#define VAL_GPIOF_OTYPER            0x00000000
#define VAL_GPIOF_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOF_PUPDR             0xFFFFFFFF
#define VAL_GPIOF_ODR               0xFFFFFFFF
#define VAL_GPIOF_AFRL              0x00000000
#define VAL_GPIOF_AFRH              0x00000000

/*
 * Port G setup.
 * All input with pull-up.
 */
#define VAL_GPIOG_MODER             0x00000000
#define VAL_GPIOG_OTYPER            0x00000000
#define VAL_GPIOG_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOG_PUPDR             0xFFFFFFFF
#define VAL_GPIOG_ODR               0xFFFFFFFF
#define VAL_GPIOG_AFRL              0x00000000
#define VAL_GPIOG_AFRH              0x00000000


/*
 * Port H setup.
 * All input with pull-up except:
 * PH0  - GPIOH_OSC_IN          (input floating).
 * PH1  - GPIOH_OSC_OUT         (input floating).
 */
#define VAL_GPIOH_MODER     (PIN_MODE_INPUT(0) |                 \
                             PIN_MODE_INPUT(1) |                \
                             PIN_MODE_INPUT(2) |                            \
                             PIN_MODE_INPUT(3) |                            \
                             PIN_MODE_INPUT(4) |                            \
                             PIN_MODE_INPUT(5) |                            \
                             PIN_MODE_INPUT(6) |                            \
                             PIN_MODE_INPUT(7) |                            \
                             PIN_MODE_INPUT(8) |                            \
                             PIN_MODE_INPUT(9) |                            \
                             PIN_MODE_INPUT(10) |                           \
                             PIN_MODE_INPUT(11) |                           \
                             PIN_MODE_INPUT(12) |                           \
                             PIN_MODE_INPUT(13) |                           \
                             PIN_MODE_INPUT(14) |                           \
                             PIN_MODE_INPUT(15))
#define VAL_GPIOH_OTYPER    0x00000000
#define VAL_GPIOH_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOH_PUPDR     (PIN_PUDR_FLOATING(0) |              \
                             PIN_PUDR_FLOATING(1) |             \
                             PIN_PUDR_PULLUP(2) |                           \
                             PIN_PUDR_PULLUP(3) |                           \
                             PIN_PUDR_PULLUP(4) |                           \
                             PIN_PUDR_PULLUP(5) |                           \
                             PIN_PUDR_PULLUP(6) |                           \
                             PIN_PUDR_PULLUP(7) |                           \
                             PIN_PUDR_PULLUP(8) |                           \
                             PIN_PUDR_PULLUP(9) |                           \
                             PIN_PUDR_PULLUP(10) |                          \
                             PIN_PUDR_PULLUP(11) |                          \
                             PIN_PUDR_PULLUP(12) |                          \
                             PIN_PUDR_PULLUP(13) |                          \
                             PIN_PUDR_PULLUP(14) |                          \
                             PIN_PUDR_PULLUP(15))
#define VAL_GPIOH_ODR       0xFFFFFFFF
#define VAL_GPIOH_AFRL      0x00000000
#define VAL_GPIOH_AFRH      0x00000000
/*
 * Port I setup.
 * All input with pull-up.
 */
#define VAL_GPIOI_MODER             0x00000000
#define VAL_GPIOI_OTYPER            0x00000000
#define VAL_GPIOI_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOI_PUPDR             0xFFFFFFFF
#define VAL_GPIOI_ODR               0xFFFFFFFF
#define VAL_GPIOI_AFRL              0x00000000
#define VAL_GPIOI_AFRH              0x00000000

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif

#endif /* _BOARD_H_ */
