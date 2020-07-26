/*  ADS131A04
 *  A driver for the ADS131A04 ADC.
 *  Copyright (C) 2019-2020 Christian Llanes
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *
 */


#include "../include/thrust_estimator_headers/ADS131A04.hpp"

namespace ADS131A04_ADC {
ADS131A04::ADS131A04(){
  DRDY = false;
  ADC_ENA_ = false;
  const char* fileName = "/dev/spidev0.0";

  bits = 8;
  speed = 1000000;
  SPIdelay = 1;
  deselect_cs = 1;
  mode = SPI_MODE_1;
  gpioPin = 14;

  if ((spifd = spi_init(fileName,bits,speed,SPIdelay,deselect_cs,mode)) < 0)
  {
    ROS_ERROR("spi_init error.");
  }

  sleep(1);

  if (gpio_export(gpioPin) < 0)
  {
    ROS_ERROR("gpio_export error.");
  }

  sleep(1);

  if (gpio_set_direction(gpioPin,0) < 0)
  {
    ROS_ERROR("gpio_set_direction error.");
  }

  sleep(1);

  if (gpio_set_edge(gpioPin,"falling") < 0)
  {
    ROS_ERROR("gpio_set_edge error.");
  }

  sleep(1);


  if ((gpiofd = gpio_pin_open(gpioPin)) < 0)
  {
    ROS_ERROR("gpio_pin_open error.");
  }

  sleep(1);

  pfd.fd = gpiofd;
  pfd.events = POLLPRI | POLLERR;


  sendSystemCommand(CMD_NULL);
}

ADS131A04::~ADS131A04()
{
  close(spifd);
  //gpio_fd_close(gpiofd);
  //gpio_unexport(gpioPin);
}

//int ADS131A04::spi_init(const char* fileDir)
//{
//
//  int spifd;
//  bits = 8;
//  speed = 1000000;
//  SPIdelay = 1;
//  deselect_cs = 1;
//  mode = SPI_MODE_1;
//
//  if ((spifd = open(fileDir,O_RDWR)) < 0)
//  {
//    printf("Failed to open the bus.");
//    com_serial = 0;
//    exit(1);
//  }
//
//  if (ioctl(spifd, SPI_IOC_WR_MODE, &mode) < 0)
//  {
//    perror("can't set spi mode");
//    return -1;
//  }
//
//  if (ioctl(spifd, SPI_IOC_RD_MODE, &mode) < 0)
//  {
//    perror("SPI rd_mode");
//    return -1;
//  }
//
//
//  if (ioctl(spifd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0)
//  {
//    perror("can't set bits per word");
//    return -1;
//  }
//
//  if (ioctl(spifd, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0)
//  {
//    perror("SPI bits_per_word");
//    return -1;
//  }
//
//  if (ioctl(spifd, SPI_IOC_WR_MAX_SPEED_HZ, &speed)<0)
//  {
//    perror("can't set max speed hz");
//    return -1;
//  }
//
//  if (ioctl(spifd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0)
//  {
//    perror("SPI max_speed_hz");
//    return -1;
//  }
//
//  return spifd;
//
//}
//
//int ADS131A04::gpio_init(int pin)
//{
//  int gpiofd;
//
//  gpiofd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
//
//  if (gpiofd == -1) {
//      perror("Unable to open /sys/class/gpio/export");
//      exit(1);
//  }
//
//  if (write(gpiofd, "24", 2) != 2) {
//      perror("Error writing to /sys/class/gpio/export");
//      exit(1);
//  }
//
//  close(gpiofd);
//
//  gpiofd = open("/sys/class/gpio/gpio24/direction", O_WRONLY);
//  if (gpiofd == -1) {
//      perror("Unable to open /sys/class/gpio/gpio24/direction");
//      exit(1);
//  }
//
//  if (write(gpiofd, "in", 3) != 3) {
//      perror("Error writing to /sys/class/gpio/gpio24/direction");
//      exit(1);
//  }
//
//  close(gpiofd);
//
//  gpiofd = open("/sys/")
//
//
//  gpiofd = open("/sys/class/gpio/gpio24/value", O_RDONLY);
//
//  if (gpiofd == -1) {
//      perror("Unable to open /sys/class/gpio/gpio24/value");
//      exit(1);
//  }
//
//  return gpiofd;
//}

//int ADS131A04::gpio_read()
//{
//  int* gpiovalue;
//  if (read(gpiofd,gpiovalue,1) != 1) {
//    perror("Error reading from gpio.");
//  }
//  return &gpiovalue;
//}

//void ADS131A04::spi_read(char buffer,int spifd)
//{
//  uint32_t nbytes = sizeof(buffer);
//
//  struct spi_ioc_transfer xfer[1];
//  memset(xfer,0,sizeof xfer);
//
//  char nulldata[nbytes];
//  memset(nulldata,0,sizeof nulldata);
//
//
//  xfer[0].tx_buf = (unsigned long)(nulldata);
//  xfer[0].rx_buf = (unsigned long)(buffer);
//  xfer[0].len = nbytes;
////xfer[0].cs_change = deselect_cs; /* Keep CS activated */
//  xfer[0].delay_usecs = SPIdelay; //delay in us
//  xfer[0].speed_hz = speed; //speed
//  xfer[0].bits_per_word = bits; // bites per word 8
//
//  int status = ioctl(spifd, SPI_IOC_MESSAGE(1), xfer);
//
//  if (status < 0)
//  {
//    perror("SPI_IOC_MESSAGE");
//    return;
//  }
//
//  com_serial=1;
//  failcount=0;
//
//}

//void ADS131A04::spi_write(char buffer,int fd)
//{
//
//  int nbytes = sizeof(buffer);
//
//  struct spi_ioc_transfer xfer[1];
//  memset(xfer,0,sizeof xfer);
//  xfer[0].tx_buf = (unsigned long)(buffer);
//  xfer[0].rx_buf = (unsigned long)(buffer);
//  xfer[0].len = nbytes; /* Length of  command to write*/
////xfer[0].cs_change = deselect_cs; /* Keep CS activated */
//  xfer[0].delay_usecs = SPIdelay; //delay in us
//  xfer[0].speed_hz = speed; //speed
//  xfer[0].bits_per_word = bits; // bites per word 8
//
//  int status = ioctl(spifd, SPI_IOC_MESSAGE(1), xfer);
//
//  if (status < 0)
//  {
//    perror("SPI_IOC_MESSAGE");
//    return;
//  }
//
//  com_serial=1;
//  failcount=0;
//}


bool ADS131A04::sendSystemCommand(systemCommands cmd)
{
  if (ADC_ENA_) {
    char tbuffer[15];
    char rbuffer[15];

    uint16_t deviceWord = cmd;
    makeBuffer_(tbuffer,deviceWord);
    printf("%02X %02X %02X \n",tbuffer[0],tbuffer[1],tbuffer[2]);
    spi_write(tbuffer,spifd,SPIdelay,speed,bits);
    char responseMask[3];

     if (cmd == CMD_RESET) {
       do {
       spi_read(rbuffer,spifd,SPIdelay,speed,bits);
       printf("%02X %02X %02X \n",rbuffer[0],rbuffer[1],rbuffer[2]);
       uint16_t DeviceWordResponse = READY;
       makeBuffer_(responseMask,DeviceWordResponse);
       } while(rbuffer[0] != responseMask[0] | rbuffer[1] != responseMask[1] | rbuffer[2] != responseMask[2]);
     } else {
       spi_read(rbuffer,spifd,SPIdelay,speed,bits);
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
     printf("tbuffer b4: %02X %02X %02X \n",tbuffer[0],tbuffer[1],tbuffer[2]);
     spi_write(tbuffer,spifd,SPIdelay,speed,bits);
     printf("tbuffer af: %02X %02X %02X \n",tbuffer[0],tbuffer[1],tbuffer[2]);
     char responseMask[3];

     if (cmd == CMD_RESET) {
       do {
       spi_read(rbuffer,spifd,SPIdelay,speed,bits);
       printf("%02X %02X %02X \n",rbuffer[0],rbuffer[1],rbuffer[2]);
       uint16_t DeviceWordResponse = READY;
       makeBuffer_(responseMask,DeviceWordResponse);
       } while(rbuffer[0] != responseMask[0] | rbuffer[1] != responseMask[1] | rbuffer[2] != responseMask[2]);
     } else {
       spi_read(rbuffer,spifd,SPIdelay,speed,bits);
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
  spi_write(tbuffer,spifd,SPIdelay,speed,bits);

  spi_read(rbuffer,spifd,SPIdelay,speed,bits);

  return (rbuffer[2] | uint32_t(rbuffer[1]) << 8 | uint32_t(rbuffer[0]) << 16);
}

uint32_t ADS131A04::readRegister(configRegisterAddress configADDR)
{
  char tbuffer[3];
  char rbuffer[3];
  uint16_t deviceWord = RREG | configADDR;
  makeBuffer_(tbuffer,deviceWord);
  spi_write(tbuffer,spifd,SPIdelay,speed,bits);

  spi_read(rbuffer,spifd,SPIdelay,speed,bits);

  return (rbuffer[2] | uint32_t(rbuffer[1]) << 8 | uint32_t(rbuffer[0]) << 16);
}

bool ADS131A04::writeRegister(configRegisterAddress configADDR, uint16_t data)
{
  char tbuffer[3];
  char rbuffer[3];
  uint16_t deviceWord = WREG | configADDR | data;
  makeBuffer_(tbuffer,deviceWord);
  printf("%02X %02X %02X \n",tbuffer[0],tbuffer[1],tbuffer[2]);
  spi_write(tbuffer,spifd,SPIdelay,speed,bits);

  spi_read(rbuffer,spifd,SPIdelay,speed,bits);
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

int ADS131A04::pollRead()
{
    lseek(pfd.fd, 0, SEEK_SET);
    char buffer[32] = {0};
    int len = read(pfd.fd, buffer, 32);
    return atoi(buffer);
}

void ADS131A04::update()
{
  if (pfd.revents & POLLPRI) {
    int value = pollRead();
    DRDY = (bool)value;
  }
}

void ADS131A04::readChannels()
{
  uint32_t channels[4];

  char outputBuffer[15];
  spi_read(outputBuffer,spifd,SPIdelay,speed,bits);
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

void ADS131A04::makeBuffer_(char buffer[], uint16_t data)
{
  memset(buffer, 0, sizeof *buffer);
  buffer[1] = data & 0xFF;
  buffer[0] = data >> 8;
}

} /*namespace ADS131A04_ADC*/
