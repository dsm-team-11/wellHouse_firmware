/*=========================================================
 * File : soft_i2c.h
 * MCU  : STM32F446RET6
 * HAL  : STM32Cube HAL
 * SCL  : PB4
 * SDA  : PB5
 =========================================================*/

#ifndef __SOFT_I2C_H
#define __SOFT_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/*---------------------------------------------------------
    I2C PIN
 ---------------------------------------------------------*/
#define SOFT_I2C_PORT      GPIOB

#define SOFT_I2C_SCL_PIN   GPIO_PIN_4
#define SOFT_I2C_SDA_PIN   GPIO_PIN_5

/*---------------------------------------------------------
    PIN CONTROL
 ---------------------------------------------------------*/
#define SCL_HIGH() HAL_GPIO_WritePin(SOFT_I2C_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_SET)
#define SCL_LOW()  HAL_GPIO_WritePin(SOFT_I2C_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_RESET)

#define SDA_HIGH() HAL_GPIO_WritePin(SOFT_I2C_PORT, SOFT_I2C_SDA_PIN, GPIO_PIN_SET)
#define SDA_LOW()  HAL_GPIO_WritePin(SOFT_I2C_PORT, SOFT_I2C_SDA_PIN, GPIO_PIN_RESET)

#define SDA_READ() HAL_GPIO_ReadPin(SOFT_I2C_PORT, SOFT_I2C_SDA_PIN)

/*---------------------------------------------------------
    FUNCTION
 ---------------------------------------------------------*/

/* Delay */
void Soft_I2C_Delay(void);

/* Start / Stop */
void Soft_I2C_Start(void);
void Soft_I2C_Stop(void);

/* Write */
void Soft_I2C_WriteByte(uint8_t data);

/* Read */
uint8_t Soft_I2C_ReadByte(uint8_t ack);

/* ACK */
void Soft_I2C_SendACK(void);
void Soft_I2C_SendNACK(void);
uint8_t Soft_I2C_WaitACK(void);

/* SDA Direction */
void SDA_Output(void);
void SDA_Input(void);

#ifdef __cplusplus
}
#endif

#endif
