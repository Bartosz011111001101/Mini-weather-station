/*
 * Sensor_functions.c
 *
 *  Created on: Feb 8, 2025
 *      Author: Bartosz
 *      MIT License

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


#include "Sensor_functions.h"
#include "main.h"
#include <stdlib.h>


int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  if(HAL_I2C_Master_Transmit(&hi2c3, (id << 1), &reg_addr, 1, 10) != HAL_OK) return -1;
  if(HAL_I2C_Master_Receive(&hi2c3, (id << 1) | 0x01, data, len, 10) != HAL_OK) return -1;

  return 0;
}
void user_delay_ms(uint32_t period)
{
  HAL_Delay(period);
}
int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  int8_t *buf;
  buf = malloc(len +1);
  buf[0] = reg_addr;
  memcpy(buf +1, data, len);

  if(HAL_I2C_Master_Transmit(&hi2c3, (id << 1), (uint8_t*)buf, len + 1, HAL_MAX_DELAY) != HAL_OK) return -1;

  free(buf);
  return 0;
}


void DHT11_Start_struct(struct DHT_Reading* self) {
    self->Set_Pin_Output_struct(self, self->GPIOx, self->GPIO_Pin);
    HAL_GPIO_WritePin(self->GPIOx, self->GPIO_Pin, GPIO_PIN_RESET);
    self->delay_struct(self, 18000);   // wait for 18ms
    self->Set_Pin_Input_struct(self, self->GPIOx, self->GPIO_Pin);
}

uint8_t Check_Response_struct(struct DHT_Reading* self) {
    uint8_t Response = 0;
    self->delay_struct(self, 40);
    if (!(HAL_GPIO_ReadPin(self->GPIOx, self->GPIO_Pin))) {
        self->delay_struct(self, 80);
        if ((HAL_GPIO_ReadPin(self->GPIOx, self->GPIO_Pin))) Response = 1;
        else Response = -1;
    }
    while ((HAL_GPIO_ReadPin(self->GPIOx, self->GPIO_Pin)));
    return Response;
}

void Set_Pin_Input_struct(struct DHT_Reading* self, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Output_struct(struct DHT_Reading* self, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void delay_struct(struct DHT_Reading* self, uint16_t us) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

uint8_t DHT11_Read_struct(struct DHT_Reading* self) {
    uint8_t i, j;
    for (j = 0; j < 8; j++) {
        while (!(HAL_GPIO_ReadPin(self->GPIOx, self->GPIO_Pin)));
        self->delay_struct(self, 40);   // wait for 40 us
        if (!(HAL_GPIO_ReadPin(self->GPIOx, self->GPIO_Pin)))
        {
            i &= ~(1 << (7 - j));
        }
        else i |= (1 << (7 - j));
        while ((HAL_GPIO_ReadPin(self->GPIOx, self->GPIO_Pin)));
    }
    return i;
}
