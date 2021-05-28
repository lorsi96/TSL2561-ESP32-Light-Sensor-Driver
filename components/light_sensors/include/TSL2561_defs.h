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
 * @brief TSL2561 Definitions and Register Structures.
 */
#ifndef __TSL2561_REFS_H_
#define __TSL2561_REFS_H_

#include <stdint.h>

typedef uint8_t __reserved__;

// I2C address options
typedef enum {
  TSL2561_ADDR_LOW =    0x29, ///< Default address (pin pulled low)
  TSL2561_ADDR_FLOAT =  0x39, ///< Default address (pin left floating)
  TSL2561_ADDR_HIGH =   0x49, ///< Default address (pin pulled high)
} TSL2561_Address_t;


#define TSL2561_CONTROL_POWERON     (0x03) ///< Control register setting to turn on
#define TSL2561_CONTROL_POWEROFF    (0x00) ///< Control register setting to turn off

/** TSL2561 I2C Registers */
typedef enum {
  TSL2561_REGISTER_CONTROL = 0x00,          // Control/power register
  TSL2561_REGISTER_TIMING = 0x01,           // Set integration time register
  TSL2561_REGISTER_THRESHHOLDL_LOW = 0x02,  // Interrupt low threshold low-byte
  TSL2561_REGISTER_THRESHHOLDL_HIGH = 0x03, // Interrupt low threshold high-byte
  TSL2561_REGISTER_THRESHHOLDH_LOW = 0x04,  // Interrupt high threshold low-byte
  TSL2561_REGISTER_THRESHHOLDH_HIGH =0x05,                           // Interrupt high threshold high-byte
  TSL2561_REGISTER_INTERRUPT = 0x06,  // Interrupt settings
  TSL2561_REGISTER_CRC = 0x08,        // Factory use only
  TSL2561_REGISTER_ID = 0x0A,         // TSL2561 identification setting
  TSL2561_REGISTER_CHAN0_LOW = 0x0C,  // Light data channel 0, low byte
  TSL2561_REGISTER_CHAN0_HIGH = 0x0D, // Light data channel 0, high byte
  TSL2561_REGISTER_CHAN1_LOW = 0x0E,  // Light data channel 1, low byte
  TSL2561_REGISTER_CHAN1_HIGH = 0x0F  // Light data channel 1, high byte
} TSL2561_RegAdd_t;

typedef enum {
  TSL2561_INT_DISABLED =  0b00,
  TSL2561_INT_ENABLED =   0b01,
} TSL2561_IntrCtrl_t;

typedef enum {
  TSL2561_INT_EVERY_CYCLE =           0b0000,
  TSL2561_INT_VALUE_OUT_OF_RANGE =    0b0001,
  TSL2561_INT_5_OUT_OF_RANGE_CYCLES = 0b0101
} TSL2561_InterruptPersistence_t;


/** Three options for how long to integrate readings for */
typedef enum {
  TSL2561_INTEGRATIONTIME_13MS = 0x00,  // 13.7ms
  TSL2561_INTEGRATIONTIME_101MS = 0x01, // 101ms
  TSL2561_INTEGRATIONTIME_402MS = 0x02, // 402ms
  TSL2561_INTEGRATIONTIME_MANUAL = 0x03 // Manual trigger
} TSL2561_IntegrationTime_t;

/** TSL2561 offers 2 gain settings */
typedef enum {
  TSL2561_GAIN_1X =     0b0,    // No gain
  TSL2561_GAIN_16X =    0b1,    // 16x gain
} TSL2561_Gain_t;


// Command Structure.
typedef union TSL2561_Command_ {
    uint8_t value;
    struct {
        TSL2561_RegAdd_t reg    :4; /*< Register */
        bool block              :1; /*< Block R/W mode */
        bool word               :1; /*< Word R/W mode */
        bool clear              :1; /*< Clears interrupt */
        bool cmdBit             :1; /*< Always 1 */
    } field;
} TSL2561_Command_t;


typedef union TSL2561_Timing_ {
    uint8_t value;
    struct {
        TSL2561_IntegrationTime_t intgTime  :2; 
        __reserved__                        :1;
        bool manual                         :1;
        TSL2561_Gain_t gain                 :1;
        __reserved__                        :3; 
    } field;
} TSL2561_Timing_t;


typedef union TSL2561_Interrupt_ {
    uint8_t value;
    struct {
        TSL2561_InterruptPersistence_t persist  :4; 
        TSL2561_IntrCtrl_t intr                 :2;
        __reserved__                            :2;
    } field;
} TSL2561_InterruptControl_t;



#endif