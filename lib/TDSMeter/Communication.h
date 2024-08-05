#ifndef _COMMUNICATION_H_
#define _COMMUNICATION_H_

#include "SPI.h"

/******************************************************************************/
/* GPIO Definitions                                                           */
/******************************************************************************/

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

#define SDO_PIN 13  // Pin connected to the ADC Dout pin
#define SDI_PIN 11  // Pin connected to the ADC Din pin
#define SCK_PIN 12  // Pin connected to the ADC SCLK pin

#define SDO_PIN_IN pinMode(SDO_PIN, INPUT)

// SPI Functions
void SPI_Init(void);
int32_t SPI_Write(uint8_t ui8_slave_id, uint8_t ui8_buffer[], uint8_t ui8_nr_bytes);
int32_t SPI_Read(uint8_t ui8_slave_id, uint8_t ui8_buffer[], uint8_t ui8_nr_bytes);

#endif  // INCLUDE_COMMUNICATION_H_