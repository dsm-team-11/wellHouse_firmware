/*=========================================================
 * File : lcd_i2c.c (1/2)
 * MCU  : STM32F446RET6
 * LCD  : 1602 I2C (PCF8574)
 =========================================================*/

#include "lcd_i2c.h"

/*---------------------------------------------------------
    내부 함수 선언
---------------------------------------------------------*/
static void lcd_write4bits(uint8_t data);
static void lcd_expanderWrite(uint8_t data);
static void lcd_pulseEnable(uint8_t data);

/*---------------------------------------------------------
    PCF8574에 1Byte 전송
---------------------------------------------------------*/
static void lcd_expanderWrite(uint8_t data)
{
    Soft_I2C_Start();

    Soft_I2C_WriteByte(LCD_ADDR);

    Soft_I2C_WriteByte(data | LCD_BACKLIGHT);

    Soft_I2C_Stop();
}

/*---------------------------------------------------------
    Enable Pulse
---------------------------------------------------------*/
static void lcd_pulseEnable(uint8_t data)
{
    lcd_expanderWrite(data | LCD_ENABLE);

    HAL_Delay(1);

    lcd_expanderWrite(data & ~LCD_ENABLE);

    HAL_Delay(1);
}

/*---------------------------------------------------------
    LCD 4Bit Write
---------------------------------------------------------*/
static void lcd_write4bits(uint8_t data)
{
    lcd_expanderWrite(data);

    lcd_pulseEnable(data);
}

/*---------------------------------------------------------
    Command 전송
---------------------------------------------------------*/
void lcd_sendCommand(uint8_t cmd)
{
    uint8_t high;
    uint8_t low;

    high = cmd & 0xF0;
    low  = (cmd << 4) & 0xF0;

    lcd_write4bits(high);

    lcd_write4bits(low);
}

/*---------------------------------------------------------
    Data 전송
---------------------------------------------------------*/
void lcd_sendData(uint8_t data)
{
    uint8_t high;
    uint8_t low;

    high = (data & 0xF0) | LCD_RS;

    low = ((data << 4) & 0xF0) | LCD_RS;

    lcd_write4bits(high);

    lcd_write4bits(low);
}

/*---------------------------------------------------------
    문자 출력
---------------------------------------------------------*/
void lcd_sendChar(char data)
{
    lcd_sendData((uint8_t)data);
}

/*---------------------------------------------------------
    LCD 초기화
---------------------------------------------------------*/
void lcd_init(void)
{
    HAL_Delay(50);

    lcd_write4bits(0x30);
    HAL_Delay(5);

    lcd_write4bits(0x30);
    HAL_Delay(5);

    lcd_write4bits(0x30);
    HAL_Delay(5);

    lcd_write4bits(0x20);
    HAL_Delay(5);

    lcd_sendCommand(0x28);   // 4Bit, 2Line
    lcd_sendCommand(0x08);   // Display OFF
    lcd_sendCommand(0x01);   // Clear
    HAL_Delay(5);

    lcd_sendCommand(0x06);   // Entry Mode
    lcd_sendCommand(0x0C);   // Display ON
}

/*---------------------------------------------------------
    LCD Clear
---------------------------------------------------------*/
void lcd_clear(void)
{
    lcd_sendCommand(0x01);
    HAL_Delay(2);
}

/*---------------------------------------------------------
    Cursor Position
---------------------------------------------------------*/
void lcd_setCursor(uint8_t col, uint8_t row)
{
    uint8_t address;

    switch(row)
    {
        case 0:
            address = 0x00;
            break;

        case 1:
            address = 0x40;
            break;

        default:
            address = 0x00;
            break;
    }

    lcd_sendCommand(0x80 | (address + col));
}

/*---------------------------------------------------------
    Print String
---------------------------------------------------------*/
void lcd_print(char *str)
{
    while(*str)
    {
        lcd_sendData((uint8_t)(*str));
        str++;
    }
}
