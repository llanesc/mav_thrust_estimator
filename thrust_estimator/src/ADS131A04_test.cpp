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

#include <stdio.h>
#include <unistd.h>
#include <spilib.h>
#include <gpiolib.h>

void makeBuffer(char *buffer, uint16_t data);
uint8_t bits;
uint32_t speed;
uint16_t SPIdelay;
uint8_t deselect_cs;
uint8_t mode;
int spifd;
int gpiofd;
int gpioPin;

int main(int argc, char **argv)
{
  const char* fileName = "/dev/spidev0.0";
  bits = 8;
  speed = 1000000;
  SPIdelay = 1;
  deselect_cs = 1;
  mode = SPI_MODE_1;
  gpioPin = 14;

  if ((spifd = spi_init(fileName,bits,speed,SPIdelay,deselect_cs,mode)) < 0)
  {
    perror("spi_init error.");
    return 0;
  }

  if (gpio_export(gpioPin) < 0)
  {
    perror("gpio_export error.");
    return 0;
  }


  if (gpio_set_direction(gpioPin,0) < 0)
  {
    perror("gpio_direction error.");
    return 0;
  }

  if (gpio_set_edge(gpioPin,"falling") < 0)
  {
    perror("gpio_edge error.");
        return 0;
  }



  if ((gpiofd = gpio_pin_open(gpioPin)) < 0)
  {
    perror("gpio_open error.");
        return 0;
  }

}

void makeBuffer(char *buffer, uint16_t data){
  for(int i = 0;i < sizeof(buffer);i++){
    buffer[i] &= 0x00;
  }
  buffer[1] = data & 0xFF;
  buffer[0] = data >> 8;
}
