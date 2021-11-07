/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
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
 * o Neither the name of the copyright holder nor the names of its
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

/*
 * TEXT BELOW IS USED AS SETTING FOR THE PINS TOOL *****************************
PinsProfile:
- !!product 'Pins v2.0'
- !!processor 'MKL25Z128xxx4'
- !!package 'MKL25Z128VLK4'
- !!mcu_data 'ksdk2_0'
- !!processor_version '1.1.0'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR THE PINS TOOL ***
 */

#include "fsl_common.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "pin_mux.h"

#define PIN1_IDX                         1u   /*!< Pin number for pin 1 in a port */
#define PIN2_IDX                         2u   /*!< Pin number for pin 2 in a port */
#define SOPT5_UART0RXSRC_UART_RX      0x00u   /*!< UART0 receive data source select: UART0_RX pin */
#define SOPT5_UART0TXSRC_UART_TX      0x00u   /*!< UART0 transmit data source select: UART0_TX pin */

/*
 * TEXT BELOW IS USED AS SETTING FOR THE PINS TOOL *****************************
BOARD_InitPins:
- options: {coreID: singlecore, enableClock: 'true'}
- pin_list:
  - {pin_num: '28', peripheral: UART0, signal: TX, pin_signal: TSI0_CH3/PTA2/UART0_TX/TPM2_CH1}
  - {pin_num: '27', peripheral: UART0, signal: RX, pin_signal: TSI0_CH2/PTA1/UART0_RX/TPM2_CH0}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR THE PINS TOOL ***
*/

/*FUNCTION**********************************************************************
 * 
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 * 
 *END**************************************************************************/
void BOARD_InitBootPins(void) {
  BOARD_InitPins();
}


/*FUNCTION**********************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/
void BOARD_InitPins(void) {
  /* Port A Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortA);
    /* Port B Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortB);
    /* Port C Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortC);
    /* Port D Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortD);
    /* Port E Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortE);

    gpio_pin_config_t WSense_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTA1 (pin 27)  */
    GPIO_PinInit(BOARD_INITPINS_OESTE_SENSOR_GPIO, BOARD_INITPINS_OESTE_SENSOR_PIN, &WSense_config);

    gpio_pin_config_t WBlue_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTA2 (pin 28)  */
    GPIO_PinInit(BOARD_INITPINS_OESTE_VUELTA_GPIO, BOARD_INITPINS_OESTE_VUELTA_PIN, &WBlue_config);

    gpio_pin_config_t WGreen_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTA4 (pin 30)  */
    GPIO_PinInit(BOARD_INITPINS_OESTE_VERDE_GPIO, BOARD_INITPINS_OESTE_VERDE_PIN, &WGreen_config);

    gpio_pin_config_t WYellow_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTA12 (pin 32)  */
    GPIO_PinInit(BOARD_INITPINS_OESTE_AMARILLO_GPIO, BOARD_INITPINS_OESTE_AMARILLO_PIN, &WYellow_config);

    // gpio_pin_config_t PeopleSense_config = {
    //  .pinDirection = kGPIO_DigitalInput,
    //  .outputLogic = 0U
    //};
    /* Initialize GPIO functionality on pin PTA13 (pin 33)  */
    //GPIO_PinInit(BOARD_INITPINS_PeopleSense_GPIO, BOARD_INITPINS_PeopleSense_PIN, &PeopleSense_config);

    gpio_pin_config_t NGreen_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTB0 (pin 43)  */
    GPIO_PinInit(BOARD_INITPINS_NORTE_VERDE_GPIO, BOARD_INITPINS_NORTE_VERDE_PIN, &NGreen_config);

    gpio_pin_config_t NYellow_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTB1 (pin 44)  */
    GPIO_PinInit(BOARD_INITPINS_NORTE_AMARILLO_GPIO, BOARD_INITPINS_NORTE_AMARILLO_PIN, &NYellow_config);

    gpio_pin_config_t NRed_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTB2 (pin 45)  */
    GPIO_PinInit(BOARD_INITPINS_NORTE_ROJO_GPIO, BOARD_INITPINS_NORTE_ROJO_PIN, &NRed_config);

    gpio_pin_config_t NBlue_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTB3 (pin 46)  */
    GPIO_PinInit(BOARD_INITPINS_NORTE_VUELTA_GPIO, BOARD_INITPINS_NORTE_VUELTA_PIN, &NBlue_config);

    gpio_pin_config_t EBlue_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTC0 (pin 55)  */
    GPIO_PinInit(BOARD_INITPINS_ESTE_VUELTA_GPIO, BOARD_INITPINS_ESTE_VUELTA_PIN, &EBlue_config);

    gpio_pin_config_t NSense_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTC2 (pin 57)  */
    GPIO_PinInit(BOARD_INITPINS_NORTE_SENSOR_GPIO, BOARD_INITPINS_NORTE_SENSOR_PIN, &NSense_config);

    gpio_pin_config_t ERed_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTC3 (pin 58)  */
    GPIO_PinInit(BOARD_INITPINS_ESTE_ROJO_GPIO, BOARD_INITPINS_ESTE_ROJO_PIN, &ERed_config);

    gpio_pin_config_t EYellow_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTC4 (pin 61)  */
    GPIO_PinInit(BOARD_INITPINS_ESTE_AMARILLO_GPIO, BOARD_INITPINS_ESTE_AMARILLO_PIN, &EYellow_config);

    gpio_pin_config_t EGreen_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTC5 (pin 62)  */
    GPIO_PinInit(BOARD_INITPINS_ESTE_VERDE_GPIO, BOARD_INITPINS_ESTE_VERDE_PIN, &EGreen_config);

    gpio_pin_config_t ESense_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTC7 (pin 64)  */
    GPIO_PinInit(BOARD_INITPINS_ESTE_SENSOR_GPIO, BOARD_INITPINS_ESTE_SENSOR_PIN, &ESense_config);

    gpio_pin_config_t WRed_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTD4 (pin 77)  */
    GPIO_PinInit(BOARD_INITPINS_OESTE_ROJO_GPIO, BOARD_INITPINS_OESTE_ROJO_PIN, &WRed_config);

    gpio_pin_config_t SGreen_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTE20 (pin 13)  */
    GPIO_PinInit(BOARD_INITPINS_SUR_VERDE_GPIO, BOARD_INITPINS_SUR_VERDE_PIN, &SGreen_config);

    gpio_pin_config_t SYellow_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTE21 (pin 14)  */
    GPIO_PinInit(BOARD_INITPINS_SUR_AMARILLO_GPIO, BOARD_INITPINS_SUR_AMARILLO_PIN, &SYellow_config);

    gpio_pin_config_t SRed_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTE22 (pin 15)  */
    GPIO_PinInit(BOARD_INITPINS_SUR_ROJO_GPIO, BOARD_INITPINS_SUR_ROJO_PIN, &SRed_config);

    gpio_pin_config_t SBlue_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTE23 (pin 16)  */
    GPIO_PinInit(BOARD_INITPINS_SUR_VUELTA_GPIO, BOARD_INITPINS_SUR_VUELTA_PIN, &SBlue_config);

    gpio_pin_config_t SSense_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTE29 (pin 21)  */
    GPIO_PinInit(BOARD_INITPINS_SUR_SENSOR_GPIO, BOARD_INITPINS_SUR_SENSOR_PIN, &SSense_config);

    const port_pin_config_t WSense = {/* Internal pull-up resistor is enabled */
                                      kPORT_PullUp,
                                      /* Slow slew rate is configured */
                                      kPORT_SlowSlewRate,
                                      /* Passive filter is disabled */
                                      kPORT_PassiveFilterDisable,
                                      /* Low drive strength is configured */
                                      kPORT_LowDriveStrength,
                                      /* Pin is configured as PTA1 */
                                      kPORT_MuxAsGpio};
    /* PORTA1 (pin 27) is configured as PTA1 */
    PORT_SetPinConfig(BOARD_INITPINS_OESTE_SENSOR_PORT, BOARD_INITPINS_OESTE_SENSOR_PIN, &WSense);

    /* PORTA12 (pin 32) is configured as PTA12 */
    PORT_SetPinMux(BOARD_INITPINS_OESTE_AMARILLO_PORT, BOARD_INITPINS_OESTE_AMARILLO_PIN, kPORT_MuxAsGpio);

    //    const port_pin_config_t PeopleSense = {/* Internal pull-up resistor is enabled */
    //                                     kPORT_PullUp,
                                           /* Slow slew rate is configured */
    //                                     kPORT_SlowSlewRate,
                                           /* Passive filter is disabled */
    //                                     kPORT_PassiveFilterDisable,
                                           /* Low drive strength is configured */
    //                                     kPORT_LowDriveStrength,
                                           /* Pin is configured as PTA13 */
    //                                     kPORT_MuxAsGpio};
    /* PORTA13 (pin 33) is configured as PTA13 */
    //PORT_SetPinConfig(BOARD_INITPINS_PeopleSense_PORT, BOARD_INITPINS_PeopleSense_PIN, &PeopleSense);

    /* PORTA2 (pin 28) is configured as PTA2 */
    PORT_SetPinMux(BOARD_INITPINS_OESTE_VUELTA_PORT, BOARD_INITPINS_OESTE_VUELTA_PIN, kPORT_MuxAsGpio);

    /* PORTA4 (pin 30) is configured as PTA4 */
    PORT_SetPinMux(BOARD_INITPINS_OESTE_VERDE_PORT, BOARD_INITPINS_OESTE_VERDE_PIN, kPORT_MuxAsGpio);

    PORTA->PCR[4] = ((PORTA->PCR[4] &
                      /* Mask bits to zero which are setting */
                      (~(PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK)))

                     /* Pull Enable: Internal pullup or pulldown resistor is not enabled on the corresponding pin. */
                     | PORT_PCR_PE(kPORT_PullDisable));

    /* PORTB0 (pin 43) is configured as PTB0 */
    PORT_SetPinMux(BOARD_INITPINS_NORTE_VERDE_PORT, BOARD_INITPINS_NORTE_VERDE_PIN, kPORT_MuxAsGpio);

    /* PORTB1 (pin 44) is configured as PTB1 */
    PORT_SetPinMux(BOARD_INITPINS_NORTE_AMARILLO_PORT, BOARD_INITPINS_NORTE_AMARILLO_PIN, kPORT_MuxAsGpio);

    /* PORTB2 (pin 45) is configured as PTB2 */
    PORT_SetPinMux(BOARD_INITPINS_NORTE_ROJO_PORT, BOARD_INITPINS_NORTE_ROJO_PIN, kPORT_MuxAsGpio);

    /* PORTB3 (pin 46) is configured as PTB3 */
    PORT_SetPinMux(BOARD_INITPINS_NORTE_VUELTA_PORT, BOARD_INITPINS_NORTE_VUELTA_PIN, kPORT_MuxAsGpio);

    /* PORTC0 (pin 55) is configured as PTC0 */
    PORT_SetPinMux(BOARD_INITPINS_ESTE_VUELTA_PORT, BOARD_INITPINS_ESTE_VUELTA_PIN, kPORT_MuxAsGpio);

    const port_pin_config_t NSense = {/* Internal pull-up resistor is enabled */
                                      kPORT_PullUp,
                                      /* Slow slew rate is configured */
                                      kPORT_SlowSlewRate,
                                      /* Passive filter is disabled */
                                      kPORT_PassiveFilterDisable,
                                      /* Low drive strength is configured */
                                      kPORT_LowDriveStrength,
                                      /* Pin is configured as PTC2 */
                                      kPORT_MuxAsGpio};
    /* PORTC2 (pin 57) is configured as PTC2 */
    PORT_SetPinConfig(BOARD_INITPINS_NORTE_SENSOR_PORT, BOARD_INITPINS_NORTE_SENSOR_PIN, &NSense);

    /* PORTC3 (pin 58) is configured as PTC3 */
    PORT_SetPinMux(BOARD_INITPINS_ESTE_ROJO_PORT, BOARD_INITPINS_ESTE_ROJO_PIN, kPORT_MuxAsGpio);

    /* PORTC4 (pin 61) is configured as PTC4 */
    PORT_SetPinMux(BOARD_INITPINS_ESTE_AMARILLO_PORT, BOARD_INITPINS_ESTE_AMARILLO_PIN, kPORT_MuxAsGpio);

    /* PORTC5 (pin 62) is configured as PTC5 */
    PORT_SetPinMux(BOARD_INITPINS_ESTE_VERDE_PORT, BOARD_INITPINS_ESTE_VERDE_PIN, kPORT_MuxAsGpio);

    const port_pin_config_t ESense = {/* Internal pull-up resistor is enabled */
                                      kPORT_PullUp,
                                      /* Fast slew rate is configured */
                                      kPORT_FastSlewRate,
                                      /* Passive filter is disabled */
                                      kPORT_PassiveFilterDisable,
                                      /* Low drive strength is configured */
                                      kPORT_LowDriveStrength,
                                      /* Pin is configured as PTC7 */
                                      kPORT_MuxAsGpio};
    /* PORTC7 (pin 64) is configured as PTC7 */
    PORT_SetPinConfig(BOARD_INITPINS_ESTE_SENSOR_PORT, BOARD_INITPINS_ESTE_SENSOR_PIN, &ESense);

    /* PORTD4 (pin 77) is configured as PTD4 */
    PORT_SetPinMux(BOARD_INITPINS_OESTE_ROJO_PORT, BOARD_INITPINS_OESTE_ROJO_PIN, kPORT_MuxAsGpio);

    /* PORTE20 (pin 13) is configured as PTE20 */
    PORT_SetPinMux(BOARD_INITPINS_SUR_VERDE_PORT, BOARD_INITPINS_SUR_VERDE_PIN, kPORT_MuxAsGpio);

    /* PORTE21 (pin 14) is configured as PTE21 */
    PORT_SetPinMux(BOARD_INITPINS_SUR_AMARILLO_PORT, BOARD_INITPINS_SUR_AMARILLO_PIN, kPORT_MuxAsGpio);

    /* PORTE22 (pin 15) is configured as PTE22 */
    PORT_SetPinMux(BOARD_INITPINS_SUR_ROJO_PORT, BOARD_INITPINS_SUR_ROJO_PIN, kPORT_MuxAsGpio);

    /* PORTE23 (pin 16) is configured as PTE23 */
    PORT_SetPinMux(BOARD_INITPINS_SUR_VUELTA_PORT, BOARD_INITPINS_SUR_VUELTA_PIN, kPORT_MuxAsGpio);

    const port_pin_config_t SSense = {/* Internal pull-up resistor is enabled */
                                      kPORT_PullUp,
                                      /* Slow slew rate is configured */
                                      kPORT_SlowSlewRate,
                                      /* Passive filter is disabled */
                                      kPORT_PassiveFilterDisable,
                                      /* Low drive strength is configured */
                                      kPORT_LowDriveStrength,
                                      /* Pin is configured as PTE29 */
                                      kPORT_MuxAsGpio};
    /* PORTE29 (pin 21) is configured as PTE29 */
    PORT_SetPinConfig(BOARD_INITPINS_SUR_SENSOR_PORT, BOARD_INITPINS_SUR_SENSOR_PIN, &SSense);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
