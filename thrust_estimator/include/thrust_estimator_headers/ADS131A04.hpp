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

#ifndef INCLUDE_THRUST_ESTIMATOR_HEADERS_ADS131A04_HPP_
#define INCLUDE_THRUST_ESTIMATOR_HEADERS_ADS131A04_HPP_

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include "ros/ros.h"
#include <poll.h>
#include <spilib.h>
#include <gpiolib.h>

namespace ADS131A04_ADC {


  /* STATUS REGISTER ADDRESSES */
  typedef enum {
    ADDR_STAT_1        =  0x0200,
    ADDR_STAT_P        =  0x0300,
    ADDR_STAT_N        =  0x0400,
    ADDR_STAT_S        =  0x0500,
    ADDR_ERROR_CNT     =  0x0600,
    ADDR_STAT_M2       =  0x0700,
  } statusRegisterAddress;

  /* CONFIGURATION REGISTER ADDRESSES */
  typedef enum {
    ADDR_A_SYS_CFG     =  0x0B00,
    ADDR_D_SYS_CFG     =  0x0C00,
    ADDR_CLK1          =  0x0D00,
    ADDR_CLK2          =  0x0E00,
    ADDR_ADC_ENA       =  0x0F00,
    ADDR_ADC1          =  0x1100,
    ADDR_ADC2          =  0x1200,
    ADDR_ADC3          =  0x1300,
    ADDR_ADC4          =  0x1400
  } configRegisterAddress;

  /* SYSTEM COMMANDS */
  typedef enum {
    CMD_NULL          =  0x0000,
    CMD_RESET         =  0x0011,
    CMD_STANDBY       =  0x0022,
    CMD_WAKEUP        =  0x0033,
    CMD_LOCK          =  0x0555,
    CMD_UNLOCK        =  0x0655
  } systemCommands;

  /* REGISTER WRITE AND READ PREFIX MASK */
  #define RREG              0x2000
  #define WREG              0x4000
  #define WREGS             0x6000

  /* SYSTEM RESPONSE */
  #define READY             0xFF04

  #define CHANNEL  0

  static int myFd;

  class ADS131A04 {
   public:
    ADS131A04();
    virtual ~ADS131A04();

    bool sendSystemCommand(systemCommands cmd);
    uint32_t readRegister(statusRegisterAddress statusADDR);
    uint32_t readRegister(configRegisterAddress configADDR);
    bool writeRegister(configRegisterAddress, uint16_t data);
    bool enableADC();
    void update();
    bool isDRDY();
    void readChannels();
    uint32_t* getChannels();

   private:
    bool ADC_ENA_;
    bool DRDY;
    uint32_t channels_[4];
    void makeBuffer_(char *buffer, uint16_t data);
    int pollRead();
//    int spi_init(const char* fileDir);
//    int gpio_init(int pin);
//    int gpio_read();
//    void spi_read(std::vector<uint8_t> &data,int fd);
//    void spi_write(std::vector<uint8_t> &data,int fd);
    int spifd;
    int gpiofd;
    int gpioPin;
    struct pollfd pfd;
    uint8_t bits;
    uint32_t speed;
    uint16_t SPIdelay;
    uint8_t deselect_cs;
    uint8_t mode;

  }; // end of class ADS131A04

} // end of namespace ADS131A04_ADC



#endif /* INCLUDE_THRUST_ESTIMATOR_HEADERS_ADS131A04_HPP_ */
