# WSPRTransmitter
[![Compile Sketch status](https://github.com/generationmake/WSPRTransmitter/workflows/Compile%20Sketch/badge.svg)](https://github.com/generationmake/WSPRTransmitter/actions?workflow=Compile+Sketch)
[![General Formatting Checks](https://github.com/generationmake/WSPRTransmitter/workflows/General%20Formatting%20Checks/badge.svg)](https://github.com/generationmake/WSPRTransmitter/actions?workflow=General+Formatting+Checks)

Arduino Sketch for transmitting WSPR messages with a Si5351

## used hardware

  * Adafruit Feather M0 https://learn.adafruit.com/adafruit-feather-m0-basic-proto
  * Adafruit Si5351 Clock Generator Breakout https://learn.adafruit.com/adafruit-si5351-clock-generator-breakout
  * Low Pass Filter for clock outputs
  * GPS receiver
  * HMIFeatherWing https://github.com/generationmake/HMIFeatherWing
  * battery for Feather or USB power bank

## used libraries

  * ArduinoNmeaParser https://github.com/107-systems/107-Arduino-NMEA-Parser
  * DogGraphicDisplay https://github.com/generationmake/DogGraphicDisplay
  * TimeLib https://github.com/PaulStoffregen/Time
  * si5351 https://github.com/etherkit/Si5351Arduino
  * JTEncode https://github.com/etherkit/JTEncode
  * SAMDTimerInterrupt https://github.com/khoih-prog/SAMD_TimerInterrupt

## pin usage

| **Pin** | **Pin Name** | **Signal**    | **Description**                            |
|:-------:|:------------:|:-------------:|:------------------------------------------:|
| 1       | RESET        | RESET         |                                            |
| 2       | 3V3          | 3V3-rail      | supply voltage for GPS and clock generator |
| 3       | AREF         | not connected |                                            |
| 4       | GND          | GND           |                                            |
| 5       | A0           | KEYPAD_A0     | Joystick                                   |
| 6       | A1           | DIS_CS        | display chip select                        |
| 7       | A2           | DIS_RESET     | display reset                              |
| 8       | A3           | DIS_A0        | display A0                                 |
| 9       | A4           |               |                                            |
| 10      | A5           |               |                                            |
| 11      | 15/SCK       | SCK           | SPI for display                            |
| 12      | 16/MOSI      | MOSI          | SPI for display                            |
| 13      | 14/MISO      | MISO          | SPI for display                            |
| 14      | 0/RX         | RX            | serial for GPS                             |
| 15      | 1/TX         | TX            | serial for GPS                             |
| 16      | D7           |               |                                            |
| 17      | D11/SDA      | SDA           | I2C for clock generator                    |
| 18      | D12/SCL      | SCL           | I2C for clock generator                    |
| 19      | 5            |               |                                            |
| 20      | 6            |               |                                            |
| 21      | 9            |               |                                            |
| 22      | 10           |               |                                            |
| 23      | 11           |               |                                            |
| 24      | 12           |               |                                            |
| 25      | 13           | DIS_BL        | Display Backlight                          |
| 26      | VBUS         |               |                                            |
| 27      | EN           |               |                                            |
| 28      | BAT          |               |                                            |
