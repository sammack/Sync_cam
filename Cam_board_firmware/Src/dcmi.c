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

#include "dcmi.h"

/* Private define ------------------------------------------------------------*/
DCMI_HandleTypeDef DCMI_HandleStructure;  
#define CAMERA_FRAME_BUFFER               0xC0260000

/* Public functions ----------------------------------------------------------*/   
/* Configure the STM32F4's Digital Camera port (DCMI) 
first configure the GPIO's used by the DCMI, then configure the DCMI port itself
and finally set up the DMA (DMA2_CHANNEL1). */
void DCMI_Configuration(void)
{
  static DMA_HandleTypeDef hdma_eval;
  DCMI_HandleTypeDef *hdcmi = &DCMI_HandleStructure;
  GPIO_InitTypeDef   GPIO_InitStruct;  
  GPIO_InitStruct.Pin       = GPIO_PIN_4 | GPIO_PIN_6;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
  GPIO_InitStruct.Pin       = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
  GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_10 | 
                              GPIO_PIN_12;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
  GPIO_InitStruct.Pin       = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    
  GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5
                              | GPIO_PIN_6;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  
  __DMA2_CLK_ENABLE();
  __DCMI_CLK_ENABLE();
  DCMI_HandleTypeDef *phdcmi;
  
  /* Get the DCMI handle structure */
  phdcmi = &DCMI_HandleStructure;

  phdcmi->Init.CaptureRate = DCMI_CR_ALL_FRAME;
  phdcmi->Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  phdcmi->Init.HSPolarity = DCMI_HSPOLARITY_HIGH;
  phdcmi->Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
  phdcmi->Init.JPEGMode = DCMI_JPEG_DISABLE;
  phdcmi->Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  phdcmi->Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  //phdcmi->Init.SyncroCode = ;
  phdcmi->Instance              = DCMI;

  /*** Configure the DMA ***/
  /* Set the parameters to be configured */
  hdma_eval.Init.Channel             = DMA_CHANNEL_1;
  hdma_eval.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_eval.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_eval.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_eval.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_eval.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  hdma_eval.Init.Mode                = DMA_CIRCULAR;
  hdma_eval.Init.Priority            = DMA_PRIORITY_HIGH;
  hdma_eval.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;         
  hdma_eval.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  hdma_eval.Init.MemBurst            = DMA_MBURST_SINGLE;
  hdma_eval.Init.PeriphBurst         = DMA_PBURST_SINGLE; 

  hdma_eval.Instance = DMA2_Stream1;

  /* Associate the initialized DMA handle to the DCMI handle */
  __HAL_LINKDMA(hdcmi, DMA_Handle, hdma_eval);
  
  /*** Configure the NVIC for DCMI and DMA ***/
  /* NVIC configuration for DCMI transfer complete interrupt */
  HAL_NVIC_SetPriority(DCMI_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DCMI_IRQn);  
  
  /* NVIC configuration for DMA2D transfer complete interrupt */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  
  /* Configure the DMA stream */
  HAL_DMA_Init(hdcmi->DMA_Handle);  

  
  HAL_DCMI_Init(phdcmi);
  /* Start the camera capture */
  HAL_DCMI_Start_DMA(&DCMI_HandleStructure, DCMI_MODE_CONTINUOUS, (uint32_t)CAMERA_FRAME_BUFFER, (480 * 752));  
}


/* Interrupt handlers --------------------------------------------------------*/   
/* DCMI port Interrupt handler routine */
void DCMI_IRQHandler(void)
{
  HAL_DCMI_IRQHandler(&DCMI_HandleStructure);
}

/* Camera DMA interrupt handler TODO: Write better summary of what DMA interupts
are used */
void DMA2_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(DCMI_HandleStructure.DMA_Handle);
}
