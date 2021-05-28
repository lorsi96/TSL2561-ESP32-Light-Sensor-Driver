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
 * @brief Example application showcasing TSL2561 I2C/Interrupt communication using an ESP32.
 */
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "LightSensorInterface.h"

#include "LightSensorInterface.h"
#include "TSL2561_defs.h"
#include "TSL2561.h"

static const char *TAG = "TSL2561_DEMO_APP";

/*****************************************************************************************/
/* IO and Peripheral Asignment Constants                                                 */
/*****************************************************************************************/
#define I2C_PORT            I2C_NUM_1
#define I2C_MASTER_SCL_IO   25               
#define I2C_MASTER_SDA_IO   26
#define I2C_INTERRUPT_IO    12

#define LED_BUILT_IN_IO     2

/*****************************************************************************************/
/* Application Constants                                                                 */
/*****************************************************************************************/
#define LIGHT_MAX_TH 0xFFFF
#define LIGHT_MIN_TH 0x20


/*****************************************************************************************/
/* Light Out of Bounds Callback                                                          */
/*****************************************************************************************/
static void onLightOutOfBounds(uint16_t val) {
    if(LIGHT_MIN_TH) {
        gpio_set_level(LED_BUILT_IN_IO, true);
        printf(TAG, "It's getting too dark!");
    }
}

static void buildInLedInit() {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = 1ULL << LED_BUILT_IN_IO;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);
}

/*****************************************************************************************/
/* Private  Constants -> TSL2561 Config                                                  */
/*****************************************************************************************/
const TSL2561_Config_t tslConfig = {
    .address = TSL2561_ADDR_FLOAT,
    .port = I2C_PORT,
    .io = {
        .scl = I2C_MASTER_SCL_IO,
        .sda = I2C_MASTER_SDA_IO,
        .isr = I2C_INTERRUPT_IO
    }
};

/*****************************************************************************************/
/* Private Variables                                                                     */
/*****************************************************************************************/
static TSL2561 concreteSensor;
static LightSensorInterface* genericLightSensor;


/*****************************************************************************************/
/* Application                                                                           */
/*****************************************************************************************/
void app_main(void){
    TSL2561_new(&concreteSensor, tslConfig);
    genericLightSensor = TSL2561_viewAsLightSensorInterface(&concreteSensor);
    
    buildInLedInit();

    LightSensorInterface_init(genericLightSensor);
    LightSensorInterface_enableThresholdInterrupts(
        genericLightSensor, 
        LIGHT_MIN_TH, 
        LIGHT_MAX_TH, 
        onLightOutOfBounds
    );

    uint16_t irradiance;
    for(;;) {
        LightSensorInterface_readIrradianceLevel(genericLightSensor , &irradiance);
        if(irradiance > LIGHT_MIN_TH) {
            gpio_set_level(LED_BUILT_IN_IO, false);
        }            
        ESP_LOGI(TAG, "Read: %x", irradiance);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
