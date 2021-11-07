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

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Direction type  */
typedef enum _pin_mux_direction
{
  kPIN_MUX_DirectionInput = 0U,         /* Input direction */
  kPIN_MUX_DirectionOutput = 1U,        /* Output direction */
  kPIN_MUX_DirectionInputOrOutput = 2U  /* Input or output direction */
} pin_mux_direction_t;

/*!
 * @addtogroup pin_mux
 * @{
 */

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPins(void);

/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

#define BOARD_INITPINS_NORTE_VERDE_GPIO GPIOB
#define BOARD_INITPINS_NORTE_VERDE_PORT PORTB
#define BOARD_INITPINS_NORTE_VERDE_PIN 0U
#define BOARD_INITPINS_NORTE_AMARILLO_GPIO GPIOB
#define BOARD_INITPINS_NORTE_AMARILLO_PORT PORTB
#define BOARD_INITPINS_NORTE_AMARILLO_PIN 1U
#define BOARD_INITPINS_NORTE_ROJO_GPIO GPIOB
#define BOARD_INITPINS_NORTE_ROJO_PORT PORTB
#define BOARD_INITPINS_NORTE_ROJO_PIN 2U
#define BOARD_INITPINS_NORTE_VUELTA_GPIO GPIOB
#define BOARD_INITPINS_NORTE_VUELTA_PORT PORTB
#define BOARD_INITPINS_NORTE_VUELTA_PIN 3U
#define BOARD_INITPINS_NORTE_SENSOR_GPIO GPIOC
#define BOARD_INITPINS_NORTE_SENSOR_PORT PORTC
#define BOARD_INITPINS_NORTE_SENSOR_PIN 2U

#define BOARD_INITPINS_ESTE_VERDE_GPIO GPIOC
#define BOARD_INITPINS_ESTE_VERDE_PORT PORTC
#define BOARD_INITPINS_ESTE_VERDE_PIN 5U
#define BOARD_INITPINS_ESTE_AMARILLO_GPIO GPIOC
#define BOARD_INITPINS_ESTE_AMARILLO_PORT PORTC
#define BOARD_INITPINS_ESTE_AMARILLO_PIN 4U
#define BOARD_INITPINS_ESTE_ROJO_GPIO GPIOC
#define BOARD_INITPINS_ESTE_ROJO_PORT PORTC
#define BOARD_INITPINS_ESTE_ROJO_PIN 3U
#define BOARD_INITPINS_ESTE_VUELTA_GPIO GPIOC
#define BOARD_INITPINS_ESTE_VUELTA_PORT PORTC
#define BOARD_INITPINS_ESTE_VUELTA_PIN 0U
#define BOARD_INITPINS_ESTE_SENSOR_GPIO GPIOC
#define BOARD_INITPINS_ESTE_SENSOR_PORT PORTC
#define BOARD_INITPINS_ESTE_SENSOR_PIN 7U

#define BOARD_INITPINS_SUR_VERDE_GPIO GPIOE
#define BOARD_INITPINS_SUR_VERDE_PORT PORTE
#define BOARD_INITPINS_SUR_VERDE_PIN 20U
#define BOARD_INITPINS_SUR_AMARILLO_GPIO GPIOE
#define BOARD_INITPINS_SUR_AMARILLO_PORT PORTE
#define BOARD_INITPINS_SUR_AMARILLO_PIN 21U
#define BOARD_INITPINS_SUR_ROJO_GPIO GPIOE
#define BOARD_INITPINS_SUR_ROJO_PORT PORTE
#define BOARD_INITPINS_SUR_ROJO_PIN 22U
#define BOARD_INITPINS_SUR_VUELTA_GPIO GPIOE
#define BOARD_INITPINS_SUR_VUELTA_PORT PORTE
#define BOARD_INITPINS_SUR_VUELTA_PIN 23U
#define BOARD_INITPINS_SUR_SENSOR_GPIO GPIOE
#define BOARD_INITPINS_SUR_SENSOR_PORT PORTE
#define BOARD_INITPINS_SUR_SENSOR_PIN 29U

#define BOARD_INITPINS_OESTE_VERDE_GPIO GPIOA
#define BOARD_INITPINS_OESTE_VERDE_PORT PORTA
#define BOARD_INITPINS_OESTE_VERDE_PIN 4U
#define BOARD_INITPINS_OESTE_AMARILLO_GPIO GPIOA
#define BOARD_INITPINS_OESTE_AMARILLO_PORT PORTA
#define BOARD_INITPINS_OESTE_AMARILLO_PIN 12U
#define BOARD_INITPINS_OESTE_ROJO_GPIO GPIOD
#define BOARD_INITPINS_OESTE_ROJO_PORT PORTD
#define BOARD_INITPINS_OESTE_ROJO_PIN 4U
#define BOARD_INITPINS_OESTE_VUELTA_GPIO GPIOA
#define BOARD_INITPINS_OESTE_VUELTA_PORT PORTA
#define BOARD_INITPINS_OESTE_VUELTA_PIN 2U
#define BOARD_INITPINS_OESTE_SENSOR_GPIO GPIOA
#define BOARD_INITPINS_OESTE_SENSOR_PORT PORTA
#define BOARD_INITPINS_OESTE_SENSOR_PIN 1U

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/*******************************************************************************
 * EOF
 ******************************************************************************/
