/******************************************************************************
* Project: Sync_CAM
* Repository: https://github.com/sammack/Sync_cam
* File: leds.h
* Author: Sam MacKenzie - samtmackenzie@gmail.com
* File Summary : header for LED 
******************************************************************************
*
* COPYRIGHT(c) 2014 Sam MacKenzie, All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*******************************************************************************/

#ifndef __LEDS_H
#define __LEDS_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* constants --------------------------------------------------------*/

#ifdef PX4FLOW

#define LED_RED         GPIO_PIN_7
#define LED_ORANGE      GPIO_PIN_2
#define LED_BLUE        GPIO_PIN_3
#define LED_PORT        GPIOE

#else

#define LED_RED         GPIO_PIN_14
#define LED_ORANGE      GPIO_PIN_13
#define LED_BLUE        GPIO_PIN_14
#define LED_PORT        GPIOD

#endif

/* Exported functions ------------------------------------------------------- */
/* Set up the GPIO ports for the 3 LEDs */
void LEDS_InitializeLeds(void);
/* turn on the LED, argument is LED_RED, LED_BLUE, or LED_ORANGE */
void LEDS_TurnOnLed(uint16_t ledToTurnOn);
void LEDS_TurnOffLed(uint16_t ledToTurnOff);

#endif /* __LEDS_H */
