/*
 * Copyright (c) 2019, Christian Llanes, ADCL, Embry-Riddle Aeronautical University
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <bcm2835.h>
#include <stdio.h>

void makeBuffer(char *buffer, uint16_t data);

int main(int argc, char **argv)
{
    // If you call this, it will not actually access the GPIO
// Use for testing
//        bcm2835_set_debug(1);

    if (!bcm2835_init())
    {
      printf("bcm2835_init failed. Are you running as root??\n");
      return 1;
    }
    if (!bcm2835_spi_begin())
    {
      printf("bcm2835_spi_begin failed. Are you running as root??\n");
      return 1;
    }
    bcm2835_spi_begin();
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);                   // The default
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_4096); // The default
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      // The default
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      // the default

    // Send a some bytes to the slave and simultaneously read
    // some bytes back from the slave
    // Most SPI devices expect one or 2 bytes of command, after which they will send back
    // some data. In such a case you will have the command bytes first in the buffer,
    // followed by as many 0 bytes as you expect returned data bytes. After the transfer, you
    // Can the read the reply bytes from the buffer.
    // If you tie MISO to MOSI, you should read back what was sent.

    bcm2835_delay(50);
    char buf[3]; // Data to send
    makeBuffer(buf,0x0011);
    bcm2835_spi_transfern(buf, sizeof(buf));
    printf("Read from SPI: %02X  %02X  %02X \n", buf[0], buf[1], buf[2]);
    bcm2835_delay(50);

    char buf2[3]; // Data to send
    makeBuffer(buf2,0x0011);
    bcm2835_spi_transfern(buf2, sizeof(buf2));
    // buf will now be filled with the data that was read from the slave
    printf("Read from SPI: %02X  %02X  %02X \n", buf2[0], buf2[1], buf2[2]);

    bcm2835_spi_end();
    bcm2835_close();
    return 0;
}

void makeBuffer(char *buffer, uint16_t data){
  for(int i = 0;i < sizeof(buffer);i++){
    buffer[i] &= 0x00;
  }
  buffer[1] = data & 0xFF;
  buffer[0] = data >> 8;
}
