/*
 * Driver for QM5883 compass
 */
#define DEBUG_MODULE "COMPASS"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stm32fxxx.h"
#include "config.h"
#include "console.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"
#include "log.h"
#include "param.h"
#include "system.h"
#include "queue.h"
#include "deck.h"
#include "i2cdev.h"

static bool isInit = false;
static TaskHandle_t xHandle = NULL;
static int16_t x_data;
static int16_t y_data;
static int16_t z_data;


void compassTask(void *param)
{
    uint8_t devAddress = 0x0D;
    uint8_t compass_data[6] = {0};

    systemWaitStart();
    
    i2cdevWriteByte(&deckBus, devAddress, 0x0B, 0x01);
    vTaskDelay(M2T(50));
    //i2cdevWriteByte(&deckBus, devAddress, 0x09, 0x1D); //set continuous mode, 200hz, 8G range 512 sample ratio
    i2cdevWriteByte(&deckBus, devAddress, 0x09, 0x0D); //set continuous mode, 200hz, 2G range 512 sample ratio
    vTaskDelay(M2T(50));

    while (1)
    {
        i2cdevReadReg8(&deckBus, devAddress, 0x00, 6, compass_data);
        x_data = (int)(int16_t)(compass_data[0] | compass_data[1] << 8);
        y_data = (int)(int16_t)(compass_data[2] | compass_data[3] << 8);
        z_data = (int)(int16_t)(compass_data[4] | compass_data[5] << 8);
        vTaskDelay(M2T(5));
    }


}

static void compassInit(DeckInfo *info)
{
    if (isInit)
        return;

    DEBUG_PRINT("Initialize.\n");

    xTaskCreate(compassTask, "COMPASS_TASK",
                configMINIMAL_STACK_SIZE, NULL, 1, &xHandle);

    isInit = true;
}

static bool compassTest()
{
    if (!isInit)
        return false;

    DEBUG_PRINT("Test passed.\n");

    return true;
}

static const DeckDriver compass_deck = {
    .vid = 0,
    .pid = 0,
    .name = "compassdec",
    .usedGpio = 0,
    .usedPeriph = DECK_USING_I2C,
    .init = compassInit,
    .test = compassTest,
};

DECK_DRIVER(compass_deck);

LOG_GROUP_START(compass)
LOG_ADD(LOG_INT16, compass_x, &x_data)
LOG_ADD(LOG_INT16, compass_y, &y_data)
LOG_ADD(LOG_INT16, compass_z, &z_data)
LOG_GROUP_STOP(compass)