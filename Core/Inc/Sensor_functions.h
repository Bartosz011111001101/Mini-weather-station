/*
 * Sensor_functions.h
 *
 *  Created on: Feb 8, 2025
 *      Author: Bartosz
MIT License

Copyright (c) 2025 [Bartosz]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */
#ifndef INC_SENSOR_FUNCTIONS_H_
#define INC_SENSOR_FUNCTIONS_H_

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "bme280.h"
#include "stm32g4xx_hal.h"


extern TIM_HandleTypeDef htim1;
extern struct bme280_dev dev;
extern struct bme280_data comp_data;
extern int8_t rslt;
extern char hum_string[50], temp_string[50],  press_string[50];
extern I2C_HandleTypeDef hi2c3;

int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
void user_delay_ms(uint32_t period);
int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);



struct DHT_Reading {
    GPIO_TypeDef* GPIOx;
    uint16_t GPIO_Pin;
    void (*Set_Pin_Output_struct)(struct DHT_Reading*, GPIO_TypeDef*, uint16_t);
    void (*Set_Pin_Input_struct)(struct DHT_Reading*, GPIO_TypeDef*, uint16_t);
    void (*delay_struct)(struct DHT_Reading*, uint16_t);
    void (*DHT11_Start_struct)(struct DHT_Reading*);
    uint8_t (*Check_Response_struct)(struct DHT_Reading*);
    uint8_t (*DHT11_Read_struct)(struct DHT_Reading*);
};


void DHT11_Start_struct(struct DHT_Reading* self);
uint8_t Check_Response_struct(struct DHT_Reading* self);
void Set_Pin_Input_struct(struct DHT_Reading* self, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Output_struct(struct DHT_Reading* self, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void delay_struct(struct DHT_Reading* self, uint16_t us);
uint8_t DHT11_Read_struct(struct DHT_Reading* self);




#endif /* INC_SENSOR_FUNCTIONS_H_ */
