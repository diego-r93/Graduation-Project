#ifndef _COMMUNICATION_H_
#define _COMMUNICATION_H_

#include "SPI.h"

/******************************************************************************/
/* GPIO Definitions                                                           */
/******************************************************************************/

// CS_AD7793
#define CS_AD7793_PIN 38  // Pin connected to the ADC Chip Select pin (not the SPI library 10 pin, which is not used here)

#define CS_AD7793_PIN_OUT pinMode(CS_AD7793_PIN, OUTPUT)

#define CS_AD7793_PIN_LOW digitalWrite(CS_AD7793_PIN, LOW)
#define CS_AD7793_PIN_HIGH digitalWrite(CS_AD7793_PIN, HIGH)

#define SDO_PIN 13  // Pin connected to the ADC Dout/Data Ready pin
#define SDI_PIN 11  // Pin connected to the ADC Din pin
#define SCK_PIN 12  // Pin connected to the ADC SCLK pin

#define SDO_PIN_IN pinMode(SDO_PIN, INPUT)

#define GPIO1_STATE digitalRead(SDO_PIN)

/*************************** Functions prototypes *****************************/
void SPI_Init(void);
void SPI_Write(uint8_t ui8address, uint32_t ui32data, uint8_t ui8bytes);
uint32_t SPI_Read(uint8_t ui8address, uint8_t ui8bytes);

#endif  // _COMMUNICATION_H_
