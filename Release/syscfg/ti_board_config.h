/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Auto generated file 
 */

#ifndef TI_BOARD_CONFIG_H
#define TI_BOARD_CONFIG_H

#include "ti_drivers_config.h"

#ifdef __cplusplus
extern "C" {
#endif

void Board_init(void);
void Board_deinit(void);

/*
 * FLASH
 */
#include <board/flash.h>

/* FLASH Instance Macros */
#define CONFIG_FLASH0 (0U)
#define CONFIG_FLASH_NUM_INSTANCES (1U)


void mmwDemo_PowerMeasurement(I2C_Handle i2cHandle, uint16_t *ptrPwrMeasured);

void SensorConfig(I2C_Handle i2cHandle);

/*
 * INA
 */
#define INA228
#define INA_CONFIG0_DIE_ID_REG_VALUE         INA228_DIE_ID_REG_VALUE
#define INA_CONFIG0_CURRENT_LSB              1
#define INA_CONFIG0_CALIB_REG_VALUE          524
#define INA_CONFIG0_SHUNT_TEMP_REG_VALUE     75
#define INA_CONFIG0_TARGET_ADDRESS           64
#define INA_CONFIG0_REG0_VALUE               0x0020
#define INA_CONFIG0_REG1_VALUE               65535
#define INA_CONFIG1_DIE_ID_REG_VALUE         INA228_DIE_ID_REG_VALUE
#define INA_CONFIG1_CURRENT_LSB              1
#define INA_CONFIG1_CALIB_REG_VALUE          524
#define INA_CONFIG1_SHUNT_TEMP_REG_VALUE     75
#define INA_CONFIG1_TARGET_ADDRESS           65
#define INA_CONFIG1_REG0_VALUE               0x0020
#define INA_CONFIG1_REG1_VALUE               65535
#define INA_CONFIG2_DIE_ID_REG_VALUE         INA228_DIE_ID_REG_VALUE
#define INA_CONFIG2_CURRENT_LSB              1
#define INA_CONFIG2_CALIB_REG_VALUE          262
#define INA_CONFIG2_SHUNT_TEMP_REG_VALUE     75
#define INA_CONFIG2_TARGET_ADDRESS           68
#define INA_CONFIG2_REG0_VALUE               0x0020
#define INA_CONFIG2_REG1_VALUE               65535



#ifdef __cplusplus
}
#endif

#endif /* TI_BOARD_CONFIG_H */
