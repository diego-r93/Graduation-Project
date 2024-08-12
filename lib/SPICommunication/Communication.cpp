/***************************** Include Files **********************************/

#include "Communication.h"

#include <Arduino.h>
#include <SPI.h>

// #include "AD7793.h"

/**
   @brief Initializes the SPI communication peripheral.

   @param lsbFirst - Transfer format (0 or 1).
                     Example: 0x0 - MSB first.
                              0x1 - LSB first.
   @param clockFreq - SPI clock frequency (Hz).
                      Example: 1000 - SPI clock frequency is 1 kHz.
   @param clockPol - SPI clock polarity (0 or 1).
                     Example: 0x0 - idle state for SPI clock is low.
                                  0x1 - idle state for SPI clock is high.
   @param clockPha - SPI clock phase (0 or 1).
                     Example: 0x0 - data is latched on the leading edge of SPI
                                    clock and data changes on trailing edge.
                              0x1 - data is latched on the trailing edge of SPI
                                    clock and data changes on the leading edge.
   @return none
**/
void SPI_Init() {
   CS_AD7793_PIN;
   CS_AD7793_PIN_OUT;
   CS_AD7793_PIN_HIGH;

   CS_AD7124_PIN_OUT;
   CS_AD7124_PIN_HIGH;

   CS_AD5683_PIN_OUT;
   CS_AD5683_PIN_HIGH;

   SDO_PIN;
   SDO_PIN_IN;

   SPI.begin();
}

/**
   @brief Writes a register to the Converter via SPI.

   @param ui8_slave_id - slave id for chip select
   @param ui8_address - ACC register address
   @param ui32_data - value to be written
   @param ui8bytes - nr of bytes to be written

   @return none

**/
int32_t SPI_Write(uint8_t ui8_slave_id, uint8_t ui8_buffer[], uint8_t ui8bytes) {
   int32_t ret = 0;

   if (ui8bytes > 4) {
      ui8bytes = 4;
   }

   /*Clear Slave based on ID */
   switch (ui8_slave_id) {
      case 0:

         digitalWrite(CS_AD7124_PIN, LOW);
         SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));
         break;
      case 1:

         digitalWrite(CS_AD5683_PIN, LOW);
         SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE2));
         break;
   }

   SPI.transfer(ui8_buffer, ui8bytes);
   SPI.endTransaction();

   /*Set Slave based on ID */
   switch (ui8_slave_id) {
      case 0:
         digitalWrite(CS_AD7124_PIN, HIGH);
         break;
      case 1:
         digitalWrite(CS_AD5683_PIN, HIGH);
         break;
   }

   if (ui8bytes == 0)
      ret = -1;

   return ret;
}

/**
   @brief Reads a specified register address in the converter via SPI.

   @param ui8_slave_id - slave id for chip select
   @param ui8_address - register address
   @param ui8bytes - register number of bytes

   @return reading result

**/
int32_t SPI_Read(uint8_t ui8_slave_id, uint8_t ui8_buffer[], uint8_t ui8bytes) {
   int32_t ret = 0;
   /*Clear Slave based on ID */

   switch (ui8_slave_id) {
      case 0:
         digitalWrite(CS_AD7124_PIN, LOW);
         SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));
         break;
      case 1:
         digitalWrite(CS_AD5683_PIN, LOW);
         SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE2));
         break;
   }

   SPI.transfer(ui8_buffer, ui8bytes);
   SPI.endTransaction();

   /*Set Slave based on ID */
   switch (ui8_slave_id) {
      case 0:
         digitalWrite(CS_AD7124_PIN, HIGH);
         break;
      case 1:
         digitalWrite(CS_AD5683_PIN, HIGH);
         break;
   }

   if (ui8bytes == 0)
      ret = -1;

   return ret;
}