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
int32_t SPI_Write(uint8_t ui8_slave_id, uint8_t ui8_buffer[], uint8_t ui8_nr_bytes) {
   int32_t ret = 0;

   if (ui8_nr_bytes > 4) {
      ui8_nr_bytes = 4;
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

   SPI.transfer(ui8_buffer, ui8_nr_bytes);
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

   if (ui8_nr_bytes == 0)
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
int32_t SPI_Read(uint8_t ui8_slave_id, uint8_t ui8_buffer[], uint8_t ui8_nr_bytes) {
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

   SPI.transfer(ui8_buffer, ui8_nr_bytes);
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

   if (ui8_nr_bytes == 0)
      ret = -1;

   return ret;
}

/***************************************************************************/ /**

   * @brief Writes data to SPI.
  *
  * @param data - Write data buffer:
  *
                - first byte is the chip select number;
  *               - from
   the second byte onwards are located data bytes to write.
  * @param bytesNumber
   - Number of bytes to write.
  *
  * @return Number of written bytes.
 *******************************************************************************/
unsigned char SPI_Write_PH(unsigned char* data,
                        unsigned char bytesNumber) {
   int
       SCNumber = data[0];

   CS_AD7793_PIN_LOW;
   SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));
   SPI.transfer(&data[1], bytesNumber);
   SPI.endTransaction();
   if (SCNumber == 0x1) {
      CS_AD7793_PIN_HIGH;
   }
   return (bytesNumber);
}

/***************************************************************************/ /**

   * @brief Reads data from SPI.
  *
  * @param data - As an input parameter, data
   represents the write buffer:
  *               - first byte is the chip select
   number;
  *               - from the second byte onwards are located data bytes
   to write.
  *               As an output parameter, data represents the read buffer:

   *               - from the first byte onwards are located the read data bytes.

   * @param bytesNumber - Number of bytes to write.
  *
  * @return Number of written
   bytes.
 *******************************************************************************/
unsigned char SPI_Read_PH(unsigned char* data,
                       unsigned char bytesNumber) {
   int i = 0;

   CS_AD7793_PIN_LOW;
   SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));
   int SCNumber = data[0];
   for (i = 1; i < bytesNumber + 1; i++) {
      data[i - 1] = SPI.transfer(data[i]);
   }
   SPI.endTransaction();

   if (SCNumber == 0x1) {
      CS_AD7793_PIN_HIGH;
   }
   return (bytesNumber);
}