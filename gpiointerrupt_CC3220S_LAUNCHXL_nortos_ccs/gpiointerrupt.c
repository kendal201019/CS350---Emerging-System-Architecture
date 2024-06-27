/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Timer.h>
#include "ti_drivers_config.h"

// Global variables
volatile uint8_t setPoint = 25;  // Default set-point temperature
volatile int16_t temperature = 0;
volatile unsigned char TimerFlag = 0;
uint32_t seconds = 0;

// I2C variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
} sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};
uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;
I2C_Handle i2c;

// UART variables
char output[64];
UART_Handle uart;

// Timer variables
Timer_Handle timer0;

// Function declarations
void initUART(void);
void initI2C(void);
int16_t readTemp(void);
void initTimer(void);
void gpioButtonFxn0(uint_least8_t index);
void gpioButtonFxn1(uint_least8_t index);
void timerCallback(Timer_Handle myHandle, int_fast16_t status);

// UART Initialization
void initUART(void) {
    UART_Params uartParams;
    UART_init();
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;
    uart = UART_open(CONFIG_UART_0, &uartParams);
    if (uart == NULL) {
        while (1);  // UART_open() failed
    }
}

// I2C Initialization
void initI2C(void) {
    int8_t i, found;
    I2C_Params i2cParams;
    snprintf(output, 64, "Initializing I2C Driver - ");
    UART_write(uart, output, strlen(output));
    I2C_init();
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL) {
        snprintf(output, 64, "Failed\n\r");
        UART_write(uart, output, strlen(output));
        while (1);
    }
    snprintf(output, 32, "Passed\n\r");
    UART_write(uart, output, strlen(output));
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    found = false;
    for (i = 0; i < 3; ++i) {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        snprintf(output, 64, "Is this %s? ", sensors[i].id);
        UART_write(uart, output, strlen(output));
        if (I2C_transfer(i2c, &i2cTransaction)) {
            snprintf(output, 64, "Found\n\r");
            UART_write(uart, output, strlen(output));
            found = true;
            break;
        }
        snprintf(output, 64, "No\n\r");
        UART_write(uart, output, strlen(output));
    }
    if (found) {
        snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.slaveAddress);
        UART_write(uart, output, strlen(output));
    } else {
        snprintf(output, 64, "Temperature sensor not found, contact professor\n\r");
        UART_write(uart, output, strlen(output));
    }
}

// Read Temperature from Sensor
int16_t readTemp(void) {
    int16_t temperature = 0;
    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction)) {
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;
        if (rxBuffer[0] & 0x80) {
            temperature |= 0xF000;
        }
    } else {
        snprintf(output, 64, "Error reading temperature sensor (%d)\n\r", i2cTransaction.status);
        UART_write(uart, output, strlen(output));
        snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r");
        UART_write(uart, output, strlen(output));
    }
    return temperature;
}

// Timer Initialization
void initTimer(void) {
    Timer_Params params;
    Timer_init();
    Timer_Params_init(&params);
    params.period = 1000000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;
    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL) {
        while (1) {}
    }
    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        while (1) {}
    }
}

// Timer Callback Function
void timerCallback(Timer_Handle myHandle, int_fast16_t status) {
    TimerFlag = 1;
}

// Button 0 Callback Function
void gpioButtonFxn0(uint_least8_t index) {
    setPoint++;
}

// Button 1 Callback Function
void gpioButtonFxn1(uint_least8_t index) {
    setPoint--;
}

// Main Thread
void *mainThread(void *arg0) {
    GPIO_init();
    initUART();
    initI2C();
    initTimer();

    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_1);

    uint32_t buttonCheckCounter = 0;
    uint32_t tempCheckCounter = 0;

    while (1) {
        if (TimerFlag) {
            TimerFlag = 0;
            seconds++;

            // Check buttons every 200 ms
            if (buttonCheckCounter >= 200) {
                buttonCheckCounter = 0;
                // Button handling is done in interrupt callbacks
            }

            // Check temperature every 500 ms
            if (tempCheckCounter >= 500) {
                tempCheckCounter = 0;
                temperature = readTemp();
                // Control LED based on temperature
                if (temperature < setPoint) {
                    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
                } else {
                    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
                }
            }

            // Send data to UART every second
            snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", temperature, setPoint, (temperature < setPoint), seconds);
            UART_write(uart, output, strlen(output));

            buttonCheckCounter += 1000;
            tempCheckCounter += 1000;
        }
    }

    return NULL;
}

