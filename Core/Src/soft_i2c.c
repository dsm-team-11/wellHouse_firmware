#include "soft_i2c.h"

/*---------------------------------------------
    Delay
---------------------------------------------*/
void Soft_I2C_Delay(void)
{
    for(volatile int i=0;i<40;i++);
}

/*---------------------------------------------
    SDA INPUT
---------------------------------------------*/
void SDA_Input(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = SOFT_I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;

    HAL_GPIO_Init(SOFT_I2C_PORT, &GPIO_InitStruct);
}

/*---------------------------------------------
    SDA OUTPUT
---------------------------------------------*/
void SDA_Output(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = SOFT_I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    HAL_GPIO_Init(SOFT_I2C_PORT, &GPIO_InitStruct);
}

/*---------------------------------------------
    START
---------------------------------------------*/
void Soft_I2C_Start(void)
{
    SDA_Output();

    SDA_HIGH();
    SCL_HIGH();
    Soft_I2C_Delay();

    SDA_LOW();
    Soft_I2C_Delay();

    SCL_LOW();
}

/*---------------------------------------------
    STOP
---------------------------------------------*/
void Soft_I2C_Stop(void)
{
    SDA_Output();

    SCL_LOW();
    SDA_LOW();
    Soft_I2C_Delay();

    SCL_HIGH();
    Soft_I2C_Delay();

    SDA_HIGH();
    Soft_I2C_Delay();
}

/*---------------------------------------------
    WRITE BYTE
---------------------------------------------*/
void Soft_I2C_WriteByte(uint8_t data)
{
    SDA_Output();

    for(int i=0;i<8;i++)
    {
        if(data & 0x80)
            SDA_HIGH();
        else
            SDA_LOW();

        Soft_I2C_Delay();

        SCL_HIGH();
        Soft_I2C_Delay();

        SCL_LOW();
        Soft_I2C_Delay();

        data <<= 1;
    }

    Soft_I2C_WaitACK();
}

/*---------------------------------------------
    WAIT ACK
---------------------------------------------*/
uint8_t Soft_I2C_WaitACK(void)
{
    uint16_t timeout = 0;

    SDA_Input();

    SDA_HIGH();

    Soft_I2C_Delay();

    SCL_HIGH();

    while(SDA_READ())
    {
        timeout++;

        if(timeout > 200)
        {
            Soft_I2C_Stop();
            return 1;
        }
    }

    SCL_LOW();

    SDA_Output();

    return 0;
}

/*---------------------------------------------
    ACK
---------------------------------------------*/
void Soft_I2C_SendACK(void)
{
    SDA_Output();

    SDA_LOW();

    Soft_I2C_Delay();

    SCL_HIGH();
    Soft_I2C_Delay();

    SCL_LOW();
}

/*---------------------------------------------
    NACK
---------------------------------------------*/
void Soft_I2C_SendNACK(void)
{
    SDA_Output();

    SDA_HIGH();

    Soft_I2C_Delay();

    SCL_HIGH();
    Soft_I2C_Delay();

    SCL_LOW();
}

/*---------------------------------------------
    READ BYTE
---------------------------------------------*/
uint8_t Soft_I2C_ReadByte(uint8_t ack)
{
    uint8_t data = 0;

    SDA_Input();

    for(int i=0;i<8;i++)
    {
        data <<= 1;

        SCL_HIGH();

        Soft_I2C_Delay();

        if(SDA_READ())
            data |= 0x01;

        SCL_LOW();

        Soft_I2C_Delay();
    }

    if(ack)
        Soft_I2C_SendACK();
    else
        Soft_I2C_SendNACK();

    return data;
}
