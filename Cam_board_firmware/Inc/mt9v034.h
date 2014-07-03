/******************************************************************************
* Project: Sync_CAM
* Repository: https://github.com/sammack/Sync_cam
* File: mt9v032.h
* Author: Sam MacKenzie - samtmackenzie@gmail.com
* File Summary : Header file for mt9034 image sensor configuration and use
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

#ifndef MT9V034_H_
#define MT9V034_H_

#include "stm32f4xx_hal.h"
#include "mt9v034_registermap.h"

/* Image Sensor Setup Constants */
#define MTV_TIMEOUT_MAX                     10000
#define MTV_COLUMN_START                    1
#define MTV_ROW_START                       4
#define MTV_SENSOR_HEIGHT                   480
#define MTV_SENSOR_WIDTH                    752
#define MTV_MINIMUM_HORIZONTAL_BLANKING     91
#define MTV_BINNING_ROW_A                   2
#define MTV_BINNING_COLUMN_A                2
#define MTV_GREY_SCALE_PATTERN              0x3000
#define MTV_MIN_EXPOSURE                    0x0001
#define MTV_MAX_EXPOSURE                    0x01E0
#define MTV_MAX_GAIN                        64
#define MTV_AGC_AEC_PIXIL_COUNT             4096
#define MTV_DESIRED_BRIGHTNESS              58
#define MTV_AUTO_EXPOSE_UPDATE_FREQUENCY    0x02
#define MTV_AUTO_EXPOSE_LOWPASS_FREQUENCY   0x01
#define MTV_AUTO_GAIN_UPDATE_FREQUENCY      0x02
#define MTV_AUTO_GAIN_LOWPASS_FREQUENCY     0x02
#define MTV_READ_MODE                       0x30A

/* Chip Control Reg 0x07 */
#define MTV_CHIP_CTRL_SCAN_PROG             0x0000
#define MTV_CHIP_CTRL_SCAN_TFI              0x0002
#define MTV_CHIP_CTRL_SCAN_SFI              0x0003
#define MTV_CHIP_CTRL_SOM_SLAVE             0x0000
#define MTV_CHIP_CTRL_SOM_MASTER            0x0009
#define MTV_CHIP_CTRL_SOM_SNAPHOT           0x0000
#define MTV_CHIP_CTRL_STEREO_DISABLED       0x0000
#define MTV_CHIP_CTRL_STEREO_ENABLED        0x0020
#define MTV_CHIP_CTRL_STEREO_MASTER         0x0000
#define MTV_CHIP_CTRL_STEREO_SLAVE          0x0040
#define MTV_CHIP_CTRL_PAR_OUT_EN            0x0080
#define MTV_CHIP_CTRL_SEQ_OUT               0x0100
#define MTV_CHIP_CTRL_SIMO_OUT              0x0200
#define MTV_CHIP_CTRL_CONTEX_A              0x0000
#define MTV_CHIP_CTRL_CONTEX_B              0x8000

/* Camera I2C setup Constants */
#define MTV_I2C_SDA_PIN                     GPIO_PIN_11
#define MTV_I2C_SCL_PIN                     GPIO_PIN_10     
#define MTV_I2C_SDA_PORT                    GPIOB
#define MTV_I2C_SCL_PORT                    GPIOB
#define MTV_I2C                             I2C2
#define MTV_I2C_AF                          GPIO_AF4_I2C2

/* Functions */

/* Configures the mt9v034 camera for just one contex (A) 
values that are hardware dependant and may be modifed for differnt hardware
setups are defined in this header file. The Register addresses are defined
in mt9v032registermap.h. */
void MTV_Configuration(void);

#endif /* MT9V034_H_ */
