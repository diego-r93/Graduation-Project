#ifndef _COMMUNICATION_H_
#define _COMMUNICATION_H_

#include <stdint.h>

#include "SPI.h"

/******************************************************************************/
/* GPIO Definitions                                                           */
/******************************************************************************/

// CS_AD7793
#define CS_AD7793_PIN 38  // Pin connected to the ADC Chip Select pin (not the SPI library 10 pin, which is not used here)

#define CS_AD7793_PIN_OUT pinMode(CS_AD7793_PIN, OUTPUT)

#define CS_AD7793_PIN_LOW digitalWrite(CS_AD7793_PIN, LOW)
#define CS_AD7793_PIN_HIGH digitalWrite(CS_AD7793_PIN, HIGH)

// CS_AD7124
#define CS_AD7124_PIN 10

// CS_AD5683
#define CS_AD5683_PIN 8

#define CS_AD7124_PIN_OUT pinMode(CS_AD7124_PIN, OUTPUT)
#define CS_AD5683_PIN_OUT pinMode(CS_AD5683_PIN, OUTPUT)

#define CS_AD7124_PIN_LOW digitalWrite(CS_AD7124_PIN, LOW)
#define CS_AD7124_PIN_HIGH digitalWrite(CS_AD7124_PIN, HIGH)

#define CS_AD5683_PIN_LOW digitalWrite(CS_AD5683_PIN, LOW)
#define CS_AD5683_PIN_HIGH digitalWrite(CS_AD5683_PIN, HIGH)

// SPI Pins
#define SDO_PIN 13  // Pin connected to the ADC Dout/Data Ready pin
#define SDI_PIN 11  // Pin connected to the ADC Din pin
#define SCK_PIN 12  // Pin connected to the ADC SCLK pin

#define SDO_PIN_IN pinMode(SDO_PIN, INPUT)

#define GPIO1_STATE digitalRead(SDO_PIN)

/* SPI Functions */
void SPI_Init(void);

int32_t SPI_Write(uint8_t ui8_slave_id, uint8_t ui8_buffer[], uint8_t ui8_nr_bytes);

int32_t SPI_Read(uint8_t ui8_slave_id, uint8_t ui8_buffer[], uint8_t ui8_nr_bytes);

#endif  // _COMMUNICATION_H_
