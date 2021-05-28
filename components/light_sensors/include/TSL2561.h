/**
 * Copyright 2021 Lucas Orsi (lorsi)
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this 
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors 
 *    may be used to endorse or promote products derived from this software without 
 *    specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 * 
 * @brief TSL2561 Light Sensor Imlpementation.
 */
#ifndef __LIGHT_SENSOR_TSL_H__
#define __LIGHT_SENSOR_TSL_H__

#include <stdint.h>
#include <stdbool.h>
#include "LightSensorInterface.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "TSL2561_defs.h"
#include "driver/i2c.h"

/*****************************************************************************************/
/* Types Definitions                                                                     */
/*****************************************************************************************/

typedef struct TSL2561_Config_ {
    TSL2561_Address_t address;
    i2c_port_t port;
    struct {
        uint8_t sda;
        uint8_t scl;
        uint8_t isr;
    } io;
} TSL2561_Config_t;


typedef struct TSL2561_ {
    TSL2561_Config_t config;
    SemaphoreHandle_t outOfRangeSemaphore;
    IrradianceConsumer_t interruptCallback;
    LightSensorInterface lightSensorInterfaceView;
} TSL2561;

/*****************************************************************************************/
/* Public Functions                                                                      */
/*****************************************************************************************/

void TSL2561_new(TSL2561* instance, TSL2561_Config_t config);

LightSensorInterface* TSL2561_viewAsLightSensorInterface(TSL2561* tsl2561);


#endif // __LIGHT_SENSOR_TSL_H__