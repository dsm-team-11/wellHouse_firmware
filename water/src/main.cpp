#include <Arduino.h>
#include <SPI.h>

#define PIN_MOSI 23
#define PIN_MISO 19
#define PIN_SCK 18
#define PIN_CS 5

SPIClass spi(VSPI);

uint8_t txData[3];
uint8_t rxData[3];

uint16_t waterValue = 0;
uint8_t state = 0;

void setup()
{
    Serial.begin(115200);

    spi.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

    pinMode(PIN_CS, OUTPUT);
    digitalWrite(PIN_CS, HIGH);
}

void loop()
{
    // 테스트용 상태값
    state++;

    if(state > 3)
        state = 0;

    txData[0] = state;
    txData[1] = 0;
    txData[2] = 0;

    digitalWrite(PIN_CS, LOW);

    spi.beginTransaction(
        SPISettings(
            500000,
            MSBFIRST,
            SPI_MODE0));

    for(int i=0;i<3;i++)
    {
        rxData[i] = spi.transfer(txData[i]);
    }

    spi.endTransaction();

    digitalWrite(PIN_CS, HIGH);

    waterValue =
        ((uint16_t)rxData[1] << 8) |
        rxData[2];

    Serial.print("State : ");
    Serial.print(state);

    Serial.print("   ADC : ");
    Serial.println(waterValue);

    delay(1000);
}