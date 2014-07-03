/******************************************************************************
* Project: Sync_CAM
* Repository: https://github.com/sammack/Sync_cam
* File: mt9v034.c
* Author: Sam MacKenzie - samtmackenzie@gmail.com
* File Summary : functions for mt9034 image sensor configuration and use
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
#include "mt9v034.h"

/* Private Function Prototypes -----------------------------------------------*/
void I2cConfiguration(void);
void ErrorHandler(void);
uint16_t ReadRegister(uint8_t address);
uint8_t WriteRegister(uint8_t address, uint16_t Data);
void ClockConfiguration(void);

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef   I2cHandle;
TIM_HandleTypeDef   TimHandle;

/* Private functions ---------------------------------------------------------*/
void ClockConfiguration(void)
{  
  GPIO_InitTypeDef   GPIO_InitStruct;
  GPIO_InitStruct.Pin           = GPIO_PIN_8;
  GPIO_InitStruct.Mode          = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull          = GPIO_NOPULL;
  GPIO_InitStruct.Speed         = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate     = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  __TIM3_CLK_ENABLE(); 
  /* Set TIM3 instance */
  TimHandle.Instance            = TIM3;
  TimHandle.Init.Period         = 3;
  TimHandle.Init.Prescaler      = 0; 
  TimHandle.Init.ClockDivision  = 0;
  TimHandle.Init.CounterMode    = TIM_COUNTERMODE_UP;
  HAL_TIM_Base_Init(&TimHandle);
  
  TIM_OC_InitTypeDef sConfig;
  sConfig.OCMode                = TIM_OCMODE_PWM1;
  sConfig.Pulse                 = 2;
  sConfig.OCPolarity            = TIM_OCPOLARITY_HIGH;
  sConfig.OCFastMode            = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_3);
    
}

void I2cConfiguration(void)
{
  GPIO_InitTypeDef   GPIO_InitStruct;
  /* I2C SCL GPIO pin configuration  */
  GPIO_InitStruct.Pin           = MTV_I2C_SCL_PIN;
  GPIO_InitStruct.Mode          = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull          = GPIO_PULLUP;
  GPIO_InitStruct.Speed         = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate     = MTV_I2C_AF;
  HAL_GPIO_Init(MTV_I2C_SCL_PORT, &GPIO_InitStruct);
    
  /* I2C SDA GPIO pin configuration  */
  GPIO_InitStruct.Pin = MTV_I2C_SDA_PIN;
  GPIO_InitStruct.Alternate = MTV_I2C_AF;
  HAL_GPIO_Init(MTV_I2C_SDA_PORT, &GPIO_InitStruct);
  
  __I2C2_CLK_ENABLE(); 
  /* I2C port configuration */
  I2cHandle.Instance             = MTV_I2C;
  I2cHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  I2cHandle.Init.ClockSpeed      = 100000;
  I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  I2cHandle.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLED;
  I2cHandle.Init.OwnAddress1     = 0xFE;
  HAL_I2C_Init(&I2cHandle);
}

/* TODO: Write and Error Handler */
void ErrorHandler(void)
{
}

/* Reads from a specific 16bit camera register. Send it an 8bit register address 
and it will return a 16-bit value starting at that address. Returns 0xff on
failure */
uint16_t ReadRegister(uint8_t reg_address)
{  
  uint8_t aTxBuffer[2];
  aTxBuffer[0] = reg_address;
  while(HAL_I2C_Master_Transmit(&I2cHandle, (uint16_t)MTV_DEVICE_WRITE_ADDRESS, (uint8_t*)aTxBuffer, 1, 10000)!= HAL_OK)
  {
    if (HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
    {
      ErrorHandler();
      return 0xff;
    }
  }
  
  uint8_t RxBuffer[2];
  
  while(HAL_I2C_Master_Receive(&I2cHandle, (uint16_t)MTV_DEVICE_WRITE_ADDRESS, (uint8_t *)RxBuffer, 2, 10000) != HAL_OK)
  {
    if (HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
    {
      ErrorHandler();
      return 0xff;
    }   
  }
    uint16_t result = RxBuffer[0] << 8; // read upper byte
    result |= RxBuffer[1]; // read lower byte
    return result;
}

/* Writes to a specific 16 bit Camera register. Arguments are an 8bit register
address and the 16 bit data you wish to write into that address. Returns 0x00 
for success and 0xff for failure */
uint8_t WriteRegister(uint8_t reg_address, uint16_t data)
{
  uint8_t TxBuffer[3];
  TxBuffer[0] = reg_address;
  TxBuffer[1] = (uint8_t)data;
  TxBuffer[2] = (uint8_t)(data >> 8);
  
  /* Timeout is set to 10S */
  while(HAL_I2C_Master_Transmit(&I2cHandle, (uint16_t)MTV_DEVICE_WRITE_ADDRESS, (uint8_t*)TxBuffer, 2, 10000)!= HAL_OK)
  {
    if (HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
    {
      ErrorHandler();
      return 0xff;
    }
  }
  return 0x00;
}

/* Public functions ----------------------------------------------------------*/  

/* Configures the mt9v034 camera for just one contex (A) 
values that are hardware dependant and may be modifed for differnt hardware
setups are defined in the header file. The Register addresses are defined
in mt9v032registermap.h. */
void MTV_Configuration(void)
{
  ClockConfiguration();
  I2cConfiguration();
  uint16_t new_control = MTV_CHIP_CTRL_SCAN_PROG | MTV_CHIP_CTRL_SOM_MASTER |
                         MTV_CHIP_CTRL_STEREO_DISABLED | 
                         MTV_CHIP_CTRL_PAR_OUT_EN | MTV_CHIP_CTRL_SIMO_OUT | 
                         MTV_CHIP_CTRL_CONTEX_A;
  /* check the version of the image sensor to make sure we are using the
  corect one, if so, continue configuring */
  uint16_t version = ReadRegister(MTV_CHIP_VERSION_REG);
  if (version == 0x1324)
  {
    /* Contex A setup */
    WriteRegister(MTV_CHIP_CONTROL_REG, new_control);
    WriteRegister(MTV_COLUMN_START_REG_A, 1);
    WriteRegister(MTV_ROW_START_REG_A, 1);
    WriteRegister(MTV_WINDOW_HEIGHT_REG_A, MTV_SENSOR_HEIGHT);
    WriteRegister(MTV_WINDOW_WIDTH_REG_A, MTV_SENSOR_WIDTH);
    WriteRegister(MTV_HOR_BLANKING_REG_A, MTV_MINIMUM_HORIZONTAL_BLANKING);
    WriteRegister(MTV_VER_BLANKING_REG_A, 10);
    WriteRegister(MTV_COARSE_SW_1_REG_A, 0x01BB);
    WriteRegister(MTV_COARSE_SW_2_REG_A, 0x01D9);
    WriteRegister(MTV_COARSE_SW_CTRL_REG_A, 0x0164);
    WriteRegister(MTV_READ_MODE_REG_A, MTV_READ_MODE);
    WriteRegister(MTV_V2_CTRL_REG_A, 0x01E0);
    
    /* General Settings */
    WriteRegister(MTV_ROW_NOISE_CORR_CTRL_REG, 0x0000);
    WriteRegister(MTV_AEC_AGC_ENABLE_REG, 0x0303);
    WriteRegister(MTV_HDR_ENABLE_REG, 0x0000);
    WriteRegister(MTV_MIN_EXPOSURE_REG, MTV_MIN_EXPOSURE);
    WriteRegister(MTV_MAX_EXPOSURE_REG, MTV_MAX_EXPOSURE);
    WriteRegister(MTV_MAX_GAIN_REG, MTV_MAX_GAIN);
    WriteRegister(MTV_AGC_AEC_PIXEL_COUNT_REG, MTV_AGC_AEC_PIXIL_COUNT);
    WriteRegister(MTV_AGC_AEC_DESIRED_BIN_REG, MTV_DESIRED_BRIGHTNESS);
    WriteRegister(MTV_ADC_RES_CTRL_REG, 0x0203);
    WriteRegister(MTV_DIGITAL_TEST_REG, MTV_GREY_SCALE_PATTERN);
    WriteRegister(MTV_AEC_UPDATE_REG, MTV_AUTO_EXPOSE_UPDATE_FREQUENCY);
    WriteRegister(MTV_AEC_LOWPASS_REG, MTV_AUTO_EXPOSE_LOWPASS_FREQUENCY);
    WriteRegister(MTV_AGC_UPDATE_REG, MTV_AUTO_GAIN_UPDATE_FREQUENCY);
    WriteRegister(MTV_AGC_LOWPASS_REG, MTV_AUTO_GAIN_LOWPASS_FREQUENCY);
    /* Reset the image sensor so that the changes take effect*/
    WriteRegister(MTV_SOFT_RESET_REG, 0x01);
  }
}





