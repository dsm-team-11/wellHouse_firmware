/*=========================================================
 * File : lcd_i2c.h
 * MCU  : STM32F446RET6
 * LCD  : 1602 I2C (PCF8574)
 =========================================================*/

#ifndef __LCD_I2C_H
#define __LCD_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "soft_i2c.h"

/*---------------------------------------------------------
    PCF8574 I2C ADDRESS

    일반적으로 아래 둘 중 하나입니다.
    0x27 : 가장 많이 사용
    0x3F : 일부 LCD 모듈
---------------------------------------------------------*/
#define LCD_ADDR        (0x27 << 1)

/*---------------------------------------------------------
    LCD CONTROL BIT
---------------------------------------------------------*/
#define LCD_BACKLIGHT   0x08
#define LCD_ENABLE      0x04
#define LCD_RW          0x02
#define LCD_RS          0x01

/*---------------------------------------------------------
    FUNCTION
---------------------------------------------------------*/

/* LCD 초기화 */
void lcd_init(void);

/* LCD 화면 지우기 */
void lcd_clear(void);

/* 커서 위치 설정 */
void lcd_setCursor(uint8_t col, uint8_t row);

/* 문자 1개 출력 */
void lcd_sendChar(char data);

/* 문자열 출력 */
void lcd_print(char *str);

/* 명령 전송 */
void lcd_sendCommand(uint8_t cmd);

/* 데이터 전송 */
void lcd_sendData(uint8_t data);

#ifdef __cplusplus
}
#endif

#endif
