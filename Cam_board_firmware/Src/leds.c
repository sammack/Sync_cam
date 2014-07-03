/******************************************************************************
* Project: Sync_CAM
* Repository: https://github.com/sammack/Sync_cam
* File: leds.c
* Author: Sam MacKenzie - samtmackenzie@gmail.com
* File Summary : functions for turning on and off status LEDs 
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

/* Includes ------------------------------------------------------------------*/
#include "leds.h"

/* Public functions ----------------------------------------------------------*/   
void LEDS_InitializeLeds(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;
  
#ifdef PX4FLOW
  /* Configure LED pins as output push pull */
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin =  LED_RED | LED_ORANGE | LED_BLUE;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStructure);
  HAL_GPIO_WritePin(LED_PORT, LED_RED, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_PORT, LED_ORANGE, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_PORT, LED_BLUE, GPIO_PIN_SET);
#else   
  /* Configure LED pins output open drain, PX4 uses GPIO pins as
  low side switch for LED */
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin =  LED_RED | LED_ORANGE | LED_BLUE;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStructure);
#endif
}

void LEDS_TurnOnLed(uint16_t ledToTurnOn)
{
#ifdef PX4FLOW
   HAL_GPIO_WritePin(LED_PORT, ledToTurnOn, GPIO_PIN_RESET);
#else   
   HAL_GPIO_WritePin(LED_PORT, ledToTurnOn, GPIO_PIN_SET);
#endif
}

void LEDS_TurnOffLed(uint16_t ledToTurnOff)
{
#ifdef PX4FLOW
   HAL_GPIO_WritePin(LED_PORT, ledToTurnOff, GPIO_PIN_SET);
#else   
   HAL_GPIO_WritePin(LED_PORT, ledToTurnOff, GPIO_PIN_SESET);
#endif
}
