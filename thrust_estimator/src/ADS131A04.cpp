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


#include "../include/thrust_estimator_headers/ADS131A04.hpp"

namespace ADS131A04_ADC {
ADS131A04::ADS131A04(){
  classPtr = this;
  DRDY = false;

  const char* fileName = "/dev/spidev0.0";

  if ((fd = spi_init(fileName)) < 0)
  {
    ROS_ERROR("spi_init error.");
  }

  sendSystemCommand(CMD_NULL);
}

ADS131A04::~ADS131A04()
{
  close(fd);
}

int ADS131A04::spi_init(const char* fileDir)
{

  int fd;
  unsigned int mode, lsb, bits;
  unsigned long speed = 2000000;

  if ((fd = open(fileDir,O_RDWR)) < 0)
  {
    printf("Failed to open the bus.");
    com_serial = 0;
    exit(1);
  }

  mode = SPI_MODE_2 | SPI_CS_HIGH | SPI_READY;

  if (ioctl(fd, SPI_IOC_WR_MODE, &mode)<0)
  {
    perror("can't set spi mode");
    return -1;
  }

  if (ioctl(fd, SPI_IOC_RD_MODE, &mode) < 0)
  {
    perror("SPI rd_mode");
    return -1;
  }

  if (ioctl(fd, SPI_IOC_RD_LSB_FIRST, &lsb) < 0)
  {
    perror("SPI rd_lsb_fist");
    return -1;
  }

  if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, new (__u8[1]){8}) < 0)
  {
    perror("can't set bits per word");
    return -1;
  }

  if (ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0)
  {
    perror("SPI bits_per_word");
    return -1;
  }

  if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed)<0)
  {
    perror("can't set max speed hz");
    return -1;
  }

  if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0)
  {
    perror("SPI max_speed_hz");
    return -1;
  }

  //xfer[0].tx_buf = (unsigned long)buf;
  //xfer[0].len = 3; /* Length of  command to write*/
  xfer[0].cs_change = 0; /* Keep CS activated */
  xfer[0].delay_usecs = 0, //delay in us
  xfer[0].speed_hz = 2000000, //speed
  xfer[0].bits_per_word = 8, // bites per word 8

  //xfer[1].rx_buf = (unsigned long) buf2;
  //xfer[1].len = 4; /* Length of Data to read */
  xfer[1].cs_change = 0; /* Keep CS activated */
  xfer[1].delay_usecs = 0;
  xfer[1].speed_hz = 2000000;
  xfer[1].bits_per_word = 8;

  return fd;

}

void ADS131A04::spi_read(char * rbuffer,int nbytes,int fd)
{
  int status;

  char tbuffer[nbytes];
  memset(tbuffer,0,sizeof tbuffer);

  xfer[0].tx_buf = (unsigned long)tbuffer;
  xfer[0].len = nbytes; /* Length of  command to write*/
  xfer[1].rx_buf = (unsigned long)rbuffer;
  xfer[1].len = nbytes; /* Length of Data to read */
  status = ioctl(fd, SPI_IOC_MESSAGE(2), xfer);
  if (status < 0)
  {
    perror("SPI_IOC_MESSAGE");
    return;
  }

  com_serial=1;
  failcount=0;

}

void ADS131A04::spi_write(char * buf,int nbytes,int fd)
{
  int status;

  xfer[0].tx_buf = (unsigned long)buf;
  xfer[0].len = nbytes; /* Length of  command to write*/

  status = ioctl(fd, SPI_IOC_MESSAGE(2), xfer);
  if (status < 0)
  {
    perror("SPI_IOC_MESSAGE");
    return;
  }

  com_serial=1;
  failcount=0;
}


void ADS131A04::readChannelsExt()
{
    ADS131A04::classPtr->readChannels();
}

bool ADS131A04::sendSystemCommand(systemCommands cmd)
{
  if (ADC_ENA_) {
    char tbuffer[15];
    char rbuffer[15];

    uint16_t deviceWord = cmd;
     makeBuffer_(tbuffer,deviceWord);
     printf("%02X %02X %02X \n",tbuffer[0],tbuffer[1],tbuffer[2]);
     spi_write(tbuffer,sizeof tbuffer,fd);
     char responseMask[3];

     if (cmd == CMD_RESET) {
       do {
       spi_read(rbuffer,sizeof rbuffer,fd);
       printf("%02X %02X %02X \n",rbuffer[0],rbuffer[1],rbuffer[2]);
       uint16_t DeviceWordResponse = READY;
       makeBuffer_(responseMask,DeviceWordResponse);
       } while(rbuffer[0] != responseMask[0] | rbuffer[1] != responseMask[1] | rbuffer[2] != responseMask[2]);
     } else {
       spi_read(rbuffer,sizeof rbuffer,fd);
       printf("%02X %02X %02X \n",rbuffer[0],rbuffer[1],rbuffer[2]);
       uint16_t DeviceWordResponse = cmd;
       makeBuffer_(responseMask,DeviceWordResponse);
     }

     if (rbuffer[0] == responseMask[0] & rbuffer[1] == responseMask[1] & rbuffer[2] == responseMask[2]) {
       return true;
     } else {
       return false;
     }
  } else {
    char tbuffer[3];
    char rbuffer[3];

    uint16_t deviceWord = cmd;
     makeBuffer_(tbuffer,deviceWord);
     printf("%02X %02X %02X \n",tbuffer[0],tbuffer[1],tbuffer[2]);
     spi_write(tbuffer,sizeof tbuffer,fd);
     char responseMask[3];

     if (cmd == CMD_RESET) {
       do {
       spi_read(rbuffer,sizeof rbuffer,fd);
       printf("%02X %02X %02X \n",rbuffer[0],rbuffer[1],rbuffer[2]);
       uint16_t DeviceWordResponse = READY;
       makeBuffer_(responseMask,DeviceWordResponse);
       } while(rbuffer[0] != responseMask[0] | rbuffer[1] != responseMask[1] | rbuffer[2] != responseMask[2]);
     } else {
       spi_read(rbuffer,sizeof rbuffer,fd);
       printf("%02X %02X %02X \n",rbuffer[0],rbuffer[1],rbuffer[2]);
       uint16_t DeviceWordResponse = cmd;
       makeBuffer_(responseMask,cmd);
     }

     if (rbuffer[0] == responseMask[0] & rbuffer[1] == responseMask[1] & rbuffer[2] == responseMask[2]) {
       return true;
     } else {
       return false;
     }
  }

}

uint32_t ADS131A04::readRegister(statusRegisterAddress statusADDR)
{
  char tbuffer[3];
  char rbuffer[3];
  uint16_t deviceWord = RREG | statusADDR;
  makeBuffer_(tbuffer,deviceWord);
  spi_write(tbuffer,sizeof tbuffer,fd);

  spi_read(rbuffer,sizeof rbuffer,fd);

  return (rbuffer[2] | uint32_t(rbuffer[1]) << 8 | uint32_t(rbuffer[0]) << 16);
}

uint32_t ADS131A04::readRegister(configRegisterAddress configADDR)
{
  char tbuffer[3];
  char rbuffer[3];
  uint16_t deviceWord = RREG | configADDR;
  makeBuffer_(tbuffer,deviceWord);
  spi_write(tbuffer,sizeof tbuffer,fd);

  spi_read(rbuffer,sizeof rbuffer,fd);

  return (rbuffer[2] | uint32_t(rbuffer[1]) << 8 | uint32_t(rbuffer[0]) << 16);
}

bool ADS131A04::writeRegister(configRegisterAddress configADDR, uint16_t data)
{
  char tbuffer[3];
  char rbuffer[3];
  uint16_t deviceWord = WREG | configADDR | data;
  makeBuffer_(tbuffer,deviceWord);
  printf("%02X %02X %02X \n",tbuffer[0],tbuffer[1],tbuffer[2]);
  spi_write(tbuffer,sizeof tbuffer,fd);

  spi_read(rbuffer,sizeof rbuffer,fd);
  printf("%02X %02X %02X \n",rbuffer[0],rbuffer[1],rbuffer[2]);

  char responseMask[3];
  uint16_t DeviceWordResponse = RREG | configADDR | data;
  makeBuffer_(responseMask,DeviceWordResponse);

  if (rbuffer[0] == responseMask[0] & rbuffer[1] == responseMask[1] & rbuffer[2] == responseMask[2]) {
    return true;
  } else {
    return false;
  }
}

bool ADS131A04::enableADC()
{
  if (writeRegister(ADDR_ADC_ENA,0x000F)) {
    ADC_ENA_ = true;
    return true;
  } else {
    return false;
  }
}

void ADS131A04::update()
{
}

void ADS131A04::readChannels()
{
  uint32_t channels[4];

  char outputBuffer[15];
  spi_read(outputBuffer,sizeof outputBuffer,fd);

//  printf("%02X %02X %02X %02X %02X %02X\n", outputBuffer[3],outputBuffer[4],outputBuffer[5],outputBuffer[6],outputBuffer[7],outputBuffer[8]);
  channels[0] = (outputBuffer[5] | uint32_t(outputBuffer[4]) << 8 | uint32_t(outputBuffer[3]) << 16);
  channels[1] = (outputBuffer[8] | uint32_t(outputBuffer[7]) << 8 | uint32_t(outputBuffer[6]) << 16);
  channels[2] = (outputBuffer[11] | uint32_t(outputBuffer[10]) << 8 | uint32_t(outputBuffer[9]) << 16);
  channels[3] = (outputBuffer[14] | uint32_t(outputBuffer[13]) << 8 | uint32_t(outputBuffer[12]) << 16);
  channels_[0] = channels[0];
  channels_[1] = channels[1];
  channels_[2] = channels[2];
  channels_[3] = channels[3];

  DRDY = true;
}

bool ADS131A04::isDRDY()
{
 return DRDY;
}

uint32_t* ADS131A04::getChannels()
{
  DRDY = false;
  return channels_;
}

void ADS131A04::makeBuffer_(char *buffer, uint16_t data){
  for(int i = 0;i < sizeof(buffer);i++){
    buffer[i] &= 0x00;
  }
  buffer[1] = data & 0xFF;
  buffer[0] = data >> 8;
}

} /*namespace ADS131A04_ADC*/
