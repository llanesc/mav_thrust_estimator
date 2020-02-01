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

#include "ros/ros.h"
#include <bcm2835.h>

void makeBuffer(char *buffer, uint16_t data);


int main(int argc, char** argv)
{
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

  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);                   // The default
  bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_32);    // The default
  bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      // The default
  bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      // the default

  uint16_t deviceWord = 0x0655;
  uint16_t null = 0x0000;
  char buffer[3];

  makeBuffer(buffer,null);
  printf("%02X %02X %02X \n",buffer[0],buffer[1],buffer[2]);
  bcm2835_spi_transfern(buffer,sizeof(buffer));
  printf("%02X %02X %02X \n\n",buffer[0],buffer[1],buffer[2]);

  makeBuffer(buffer,deviceWord);
  printf("%02X %02X %02X \n",buffer[0],buffer[1],buffer[2]);
  bcm2835_spi_transfern(buffer,sizeof(buffer));
  printf("%02X %02X %02X \n\n",buffer[0],buffer[1],buffer[2]);

  makeBuffer(buffer,null);
  printf("%02X %02X %02X \n",buffer[0],buffer[1],buffer[2]);
  bcm2835_spi_transfern(buffer,sizeof(buffer));
  printf("%02X %02X %02X \n\n",buffer[0],buffer[1],buffer[2]);

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
