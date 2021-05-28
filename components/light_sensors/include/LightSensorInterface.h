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
 * @brief Plaform Agnostic Sensor Interface.
 */
#ifndef __LIGHT_SENSOR_INTERFACE_H__
#define __LIGHT_SENSOR_INTERFACE_H__

#include <stdint.h>
#include <stdbool.h>

/*****************************************************************************************/
/* Types Definitions                                                                     */
/*****************************************************************************************/

typedef void(*IrradianceConsumer_t)(uint16_t);

typedef struct LightSensorType {
    void* instance;
    void(*init)(void* this);
    void(*readIrradianceLevel)(void* this, uint16_t* irradiance);
    void(*enableThresholdInterrupts)(
        void* this, uint16_t minLightDn, uint16_t maxLightDn, IrradianceConsumer_t callback
    );
} LightSensorInterface;

/*****************************************************************************************/
/* Interface Abstract Methods                                                            */
/*****************************************************************************************/

/**
 * @brief Initializes and configures the Light sensor.
 * 
 * @param this 
 */
static inline void LightSensorInterface_init(LightSensorInterface* this) {
    return this->init(this->instance);
}


/**
 * @brief Gets light irradiance level.
 * 
 * @param[in] this
 * @param[out] irradiance in digital number format (0 min - 0xFFFF max)
 */
static inline void LightSensorInterface_readIrradianceLevel(LightSensorInterface* this, uint16_t* irradiance) {
    this->readIrradianceLevel(this->instance, irradiance);
}


/**
 * @brief Registers a callback to be called whenever light values get out of bounds.
 * 
 * @param this 
 * @param minLightDn if an irradiance value is lower than this parameter, the callback is triggered.
 * @param maxLightDn if an irradiance value is higher than this parameter, the callback is triggered.
 * @param callback to be run if irradiance goes higher or lower than the thresholds. 
 *  - The irradiance value is passed as a parameter.
 * 
 */
static inline void LightSensorInterface_enableThresholdInterrupts(
    LightSensorInterface* this, uint16_t minLightDn, uint16_t maxLightDn, IrradianceConsumer_t callback) {
  return this->enableThresholdInterrupts(this->instance, minLightDn, maxLightDn, callback);
}


#endif // __LIGHT_SENSOR_INTERFACE_H__