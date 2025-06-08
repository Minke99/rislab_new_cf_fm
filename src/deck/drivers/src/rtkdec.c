/*
 * Driver for RTK
 */
#define DEBUG_MODULE "RTK"

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
#include "uart1.h"

#define PACKET_SIZE 85

static bool isInit = false;
static TaskHandle_t xHandle = NULL;
static float pos_x = 0.0;//cm
static float pos_y = 0.0;//cm
static float pos_z = 0.0;//cm
static uint8_t fix_mode = 0;

static uint8_t first_data;

void uart1Read(uint8_t *packet, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++)
    {
        uart1GetDataWithDefaultTimeout(packet + i);
        // uart1Getchar(packet + i);
    }
}

// Custom function to convert a string to a double
double_t custom_atof(char *str) {
    double_t result = 0.0;
    int i = 0;
    int sign = 1;
    double_t fraction = 0.1;

    // Handling sign
    if (str[i] == '-') {
        sign = -1;
        i++;
    }

    // Parsing the integer part
    while (str[i] >= '0' && str[i] <= '9') {
        result = result * 10.0 + (str[i] - '0');
        i++;
    }

    // Handling fractional part
    if (str[i] == '.') {
        i++;
        while (str[i] >= '0' && str[i] <= '9') {
            result = result + fraction * (str[i] - '0');
            fraction *= 0.1;
            i++;
        }
    }

    return sign * result;
}

void rtkTask(void *param)
{
    const uint32_t baudrate = 115200;
    uint8_t data_vec[PACKET_SIZE] = {0};
    uint8_t current_index = 0;
    char char_vec[PACKET_SIZE + 1] = {'\0'};
    char char_GGA[7];
    //char real_GGA[7] = "$GNGGA";
    int8_t latIndex;
    int8_t lonIndex;
    int8_t altIndex;
    int8_t modIndex;
    char lat_deg[3];
    char lat_min[11];
    char lon_deg[4];
    char lon_min[11];
    char alt_char[7];
    char mod_char[2];
    double_t latitude = 0.0;
    double_t longitude = 0.0;
    double_t altitude = 0.0;
    double_t latitude_0 = 22.33760453;
    double_t longitude_0 = 114.17156506;

    uart1Init(baudrate);
    systemWaitStart();
    DEBUG_PRINT("Baud rate: %d\n", (int)baudrate);
    
    while (1)
    {
        uart1Read(data_vec, 1);
        //first_data = data_vec[0];
        if (data_vec[0] == 0x24) // find '$'
        {
            char_vec[0] = (char)data_vec[0];
            while (1)
            {
                current_index++;
                if (current_index >= PACKET_SIZE) // exceed maximum number
                {
                    break;
                }
                uart1Read(data_vec + current_index, 1);
                char_vec[current_index] = (char)data_vec[current_index];
                if (data_vec[current_index] == 0x2A) // find '*'
                {
                    //first_data = data_vec[current_index];
                    break;
                }
            }
            // calculate data
            for (int i = 0; i < 6; i++) // get first 6 characters
            {
                char_GGA[i]=char_vec[i];
            }
            //if (strcmp(char_GGA, real_GGA) == 0) // if is GGA
            if ((uint8_t)char_GGA[5] == 65) // if is A
            {
                // indicate the number of ',' before the data
                latIndex = 2; 
                lonIndex = 4;
                altIndex = 9;
                modIndex = 6;

                current_index = 6;
                while (char_vec[current_index] != '\0') // not reach the end
                {
                    if (latIndex == 0 && char_vec[current_index] != ',')
                    {
                        first_data = current_index;
                        for (int i = 0; i < 2; i++)
                        {
                            lat_deg[i] = char_vec[i+current_index];
                        }
                        for (int i = 0; i < 10; i++)
                        {
                            lat_min[i] = char_vec[i+2+current_index];
                        }
                        latitude = (double_t)(custom_atof(lat_deg) + custom_atof(lat_min) / 60.0);
                        pos_y = (float)((latitude - latitude_0) * 111.32 * 1e5); // cm
                        latIndex--;
                    }
                    if (lonIndex == 0 && char_vec[current_index] != ',')
                    {
                        for (int i = 0; i < 3; i++)
                        {
                            lon_deg[i] = char_vec[i+current_index];
                        }
                        for (int i = 0; i < 10; i++)
                        {
                            lon_min[i] = char_vec[i+3+current_index];
                        }
                        longitude = (double_t)(custom_atof(lon_deg) + custom_atof(lon_min) / 60.0);
                        pos_x = (float)((longitude - longitude_0) * 102.52 * 1e5); // cm
                        lonIndex--;
                    }
                    if (altIndex == 0 && char_vec[current_index] != ',')
                    {
                        for (int i = 0; i < 6; i++)
                        {
                            alt_char[i] = char_vec[i+current_index];
                        }
                        altitude = (double_t)custom_atof(alt_char);
                        pos_z = (float)(altitude * 1e2); // cm
                        altIndex--;
                    }
                    if (modIndex == 0 && char_vec[current_index] != ',')
                    {
                        for (int i = 0; i < 1; i++)
                        {
                            mod_char[i] = char_vec[i+current_index];
                        }
                        fix_mode = (uint8_t)custom_atof(mod_char);
                        modIndex--;
                    }

                    if (char_vec[current_index] == ',') // find next ','
                    {
                        latIndex--;
                        lonIndex--;
                        altIndex--;
                        modIndex--;
                    }                    
                    current_index++;
                }
            }
            current_index = 0;
            memset(data_vec, 0, PACKET_SIZE);
            memset(char_vec, '\0', PACKET_SIZE);
        }
    }


}

static void rtkInit(DeckInfo *info)
{
    if (isInit)
        return;

    DEBUG_PRINT("Initialize.\n");

    xTaskCreate(rtkTask, "RTK_TASK",
                configMINIMAL_STACK_SIZE, NULL, 1, &xHandle);

    isInit = true;
}

static bool rtkTest()
{
    if (!isInit)
        return false;

    DEBUG_PRINT("Test passed.\n");

    return true;
}

static const DeckDriver rtk_deck = {
    .vid = 0,
    .pid = 0,
    .name = "rtkdec",
    .usedGpio = 0,
    .usedPeriph = DECK_USING_UART1,
    .init = rtkInit,
    .test = rtkTest,
};

DECK_DRIVER(rtk_deck);

LOG_GROUP_START(rtk)
LOG_ADD(LOG_FLOAT, rtk_x, &pos_x)
LOG_ADD(LOG_FLOAT, rtk_y, &pos_y)
LOG_ADD(LOG_FLOAT, rtk_z, &pos_z)
LOG_ADD(LOG_UINT8, rtk_mod, &fix_mode)
LOG_ADD(LOG_UINT8, rtk_1data, &first_data)
LOG_GROUP_STOP(rtk)