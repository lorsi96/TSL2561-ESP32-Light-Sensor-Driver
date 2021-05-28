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
 */
#include "TSL2561.h"
#include "TSL2561_defs.h"

#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#define TSL2561_WR_BIT              (0b0)
#define TSL2561_R_BIT               (0b1)
#define TSL2561_ACK                 (0b1)

#define TSL2561_I2C_TIMEOUT_MS      (1000)
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0   
#define ESP_INTR_FLAG_DEFAULT 0    


/*****************************************************************************************/
/* Configuration Constants                                                               */
/*****************************************************************************************/

static const TSL2561_Timing_t defaultTimingConfig = {
    .field = {
        .intgTime = TSL2561_INTEGRATIONTIME_402MS,
        .gain = TSL2561_GAIN_1X
    }
};

static const TSL2561_InterruptControl_t thInterruptConfig = {
    .field = {
        .intr = TSL2561_INT_ENABLED,
        .persist = TSL2561_INT_VALUE_OUT_OF_RANGE
    }
};

/*****************************************************************************************/
/* Private Methods Declarations  - Helper Functions                                      */
/*****************************************************************************************/
static esp_err_t TSL2561_writeByte(TSL2561* tsl2561, uint8_t command, uint8_t value);
static esp_err_t TSL2561_readByte(TSL2561* tsl2561, uint8_t command, uint8_t* value);

static inline esp_err_t TSL2561_writeRegisterByte_clr_int(TSL2561* self, TSL2561_RegAdd_t reg, uint8_t value, bool clrIntr);
static inline esp_err_t TSL2561_writeRegisterByte(TSL2561* self, TSL2561_RegAdd_t reg, uint8_t value);
static inline esp_err_t TSL2561_readRegisterByte(TSL2561* self, TSL2561_RegAdd_t reg, uint8_t* value);


static esp_err_t TSL2561_i2cInit(TSL2561* self);
static void TSL2561_readIrradianceLevel_should_clr_int(void* self, uint16_t* irradiance, bool clrInterrupt);


/*****************************************************************************************/
/* Private Methods Declarations  - Interface Definitions                                 */
/*****************************************************************************************/
static void TSL2561_init(void* self);
static void TSL2561_initBase(TSL2561* tsl2561);
static void TSL2561_enableThresholdInterrupts(
    void* this, uint16_t minLightDn, uint16_t maxLightDn, IrradianceConsumer_t callback);


/*****************************************************************************************/
/* Light Out-of-bounds ISR                                                               */
/*****************************************************************************************/
static void IRAM_ATTR TSL2561_outOfRangeIntrHandler(void* outOfRangeSemaphore){
    BaseType_t na = pdFALSE;
    SemaphoreHandle_t smphr = *(SemaphoreHandle_t*)outOfRangeSemaphore;
    xSemaphoreGiveFromISR(smphr, &na);
    portYIELD_FROM_ISR(na);
}

/*****************************************************************************************/
/* Light Out-of-bounds ISR Task                                                          */
/*****************************************************************************************/
static void outOfRangeCheckerTask(void* pvTSL2561) {
    TSL2561* tsl = pvTSL2561;
    uint16_t irradiance;
    for(;;) {
        xSemaphoreTake(tsl->outOfRangeSemaphore, portMAX_DELAY);
        TSL2561_readIrradianceLevel_should_clr_int(tsl, &irradiance, /*clrInterrupt=*/true);
        tsl->interruptCallback(irradiance);
    }
}

/*****************************************************************************************/
/* Private Methods Definitions  - Helper Functions                                       */
/*****************************************************************************************/
static esp_err_t TSL2561_writeByte(TSL2561* tsl2561, uint8_t command, uint8_t value) {
    esp_err_t err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, tsl2561->config.address << 1 | TSL2561_WR_BIT, TSL2561_ACK);
    i2c_master_write_byte(cmd, command, TSL2561_ACK);
    i2c_master_write_byte(cmd, value, TSL2561_ACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(tsl2561->config.port, cmd, TSL2561_I2C_TIMEOUT_MS / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

static esp_err_t TSL2561_readByte(TSL2561* tsl2561, uint8_t command, uint8_t* value) {
    esp_err_t err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, tsl2561->config.address << 1 | TSL2561_WR_BIT, TSL2561_ACK);
    i2c_master_write_byte(cmd, command, TSL2561_ACK);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, tsl2561->config.address << 1 | TSL2561_R_BIT, TSL2561_ACK);
    i2c_master_read_byte(cmd, value, TSL2561_ACK);

    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(tsl2561->config.port, cmd, TSL2561_I2C_TIMEOUT_MS / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

static inline esp_err_t TSL2561_writeRegisterByte_clr_int(TSL2561* self, TSL2561_RegAdd_t reg, uint8_t value, bool clrIntr) {
    TSL2561_Command_t command = { .field = { .cmdBit = 0b1, .reg = reg, .clear = clrIntr}};
    return TSL2561_writeByte(self, command.value, value);
}

static inline esp_err_t TSL2561_writeRegisterByte(TSL2561* self, TSL2561_RegAdd_t reg, uint8_t value) {
    TSL2561_Command_t command = { .field = { .cmdBit = 0b1, .reg = reg}};
    return TSL2561_writeByte(self, command.value, value);
}

static inline esp_err_t TSL2561_readRegisterByte(TSL2561* self, TSL2561_RegAdd_t reg, uint8_t* value) {
    TSL2561_Command_t command = { .field = { .cmdBit = 0b1, .reg = reg}};
    return TSL2561_readByte(self, command.value, value);
}

static esp_err_t TSL2561_i2cInit(TSL2561* self) {
    int i2c_master_port = self->config.port;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = self->config.io.sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = self->config.io.scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static void TSL2561_enableOutOfBoundsInterrupt(TSL2561* this) {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = 1ULL << (this->config.io.isr) ;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);

    xTaskCreate(
        outOfRangeCheckerTask,
        "TSL2561_Interrupt_Task",
        configMINIMAL_STACK_SIZE*4, 
        this,                          
        tskIDLE_PRIORITY+1,  
        0
    );

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(this->config.io.isr, TSL2561_outOfRangeIntrHandler, &this->outOfRangeSemaphore);
}

static void TSL2561_readIrradianceLevel_should_clr_int(void* self, uint16_t* irradiance, bool clrInterrupt) {
    TSL2561* self_ = self;
    TSL2561_Command_t commandClrIntr = { .field = { .cmdBit = 0b1, .reg = TSL2561_REGISTER_CHAN0_HIGH, .clear=clrInterrupt}};
    uint8_t irad[2];
    TSL2561_readRegisterByte(self_, TSL2561_REGISTER_CHAN0_LOW, &irad[0]);
    TSL2561_readByte(self_, commandClrIntr.value, &irad[1]);
    *irradiance = *(uint16_t*)irad;
}

/*****************************************************************************************/
/* Private Methods Definitions  - Interface Impls                                        */
/*****************************************************************************************/
static void TSL2561_init(void* self) {
    TSL2561* self_ = self;
    TSL2561_i2cInit(self_);
    TSL2561_writeRegisterByte_clr_int(self_, TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWERON, /*clrIntr=*/true);
    TSL2561_writeRegisterByte(self_, TSL2561_REGISTER_TIMING, defaultTimingConfig.value);
}

void TSL2561_enableThresholdInterrupts(void* this, uint16_t minLightDn, uint16_t maxLightDn, IrradianceConsumer_t callback) {
    TSL2561* this_ = this;
    this_->interruptCallback = callback;
    TSL2561_writeRegisterByte(this_, TSL2561_REGISTER_THRESHHOLDL_LOW, ((uint8_t*)(&minLightDn))[0]);
    TSL2561_writeRegisterByte(this_, TSL2561_REGISTER_THRESHHOLDL_HIGH, ((uint8_t*)(&minLightDn))[1]);
    TSL2561_writeRegisterByte(this_, TSL2561_REGISTER_THRESHHOLDH_LOW, ((uint8_t*)(&maxLightDn))[0]);
    TSL2561_writeRegisterByte(this_, TSL2561_REGISTER_THRESHHOLDH_HIGH, ((uint8_t*)(&maxLightDn))[1]);
    TSL2561_writeRegisterByte(this_, TSL2561_REGISTER_INTERRUPT, thInterruptConfig.value); 
    TSL2561_enableOutOfBoundsInterrupt(this_);

}

static void TSL2561_readIrradianceLevel(void* self, uint16_t* irradiance) {
    TSL2561_readIrradianceLevel_should_clr_int(self, irradiance, /*clrInterrupt=*/false);
}

static void TSL2561_initBase(TSL2561* tsl2561) {
    tsl2561->lightSensorInterfaceView.instance = tsl2561;
    tsl2561->lightSensorInterfaceView.readIrradianceLevel = TSL2561_readIrradianceLevel;
    tsl2561->lightSensorInterfaceView.enableThresholdInterrupts = TSL2561_enableThresholdInterrupts;
    tsl2561->lightSensorInterfaceView.init = TSL2561_init;
}  

/*****************************************************************************************/
/* Public Methods Definitions  - Interface Impls                                         */
/*****************************************************************************************/

void TSL2561_new(TSL2561* this, TSL2561_Config_t config){
    this->outOfRangeSemaphore = xSemaphoreCreateBinary();
    this->config = config;
    TSL2561_initBase(this);
}

LightSensorInterface* TSL2561_viewAsLightSensorInterface(TSL2561* tsl2561) {
    return &tsl2561->lightSensorInterfaceView;
}
