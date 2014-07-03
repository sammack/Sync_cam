/******************************************************************************
* Project: Sync_CAM
* Repository: https://github.com/sammack/Sync_cam
* File: 
* Author: Sam MacKenzie - samtmackenzie@gmail.com
* File Summary : 
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

#ifndef __DCMI_H
#define __DCMI_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Exported functions ------------------------------------------------------- */

/* Configure the STM32F4 Digital Camera port (DCMI) to read data from the camera
first configure the GPIO's used by the DCMI, then configure the DCMI port itself
and finally set up the DMA (DMA2_CHANNEL1). */
void DCMI_Configuration(void);

/* Interrupt handlers names are specific to the STM32F4 and are defined in
startup_stm32f407xx.s */
/* Camera Interrupts are just for dealing with errors */
void DCMI_IRQHandler(void);
/* Camera DMA interrupt handler TODO: Write better summary of what DMA interupts
are used */
void DMA2_Stream1_IRQHandler(void);

#endif /* __DCMI_H */
