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

#ifndef INCLUDE_THRUST_ESTIMATOR_HEADERS_ADS131A04_HPP_
#define INCLUDE_THRUST_ESTIMATOR_HEADERS_ADS131A04_HPP_

#include <wiringPiSPI.h>
#include <wiringPi.h>
#include "ros/ros.h"

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

  #define CHANNEL  1
  static int myFd ;

  class ADS131A04 {
    volatile bool DRDY;

    static ADS131A04 * classPtr;

    static void readChannelsExt();

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
    uint32_t channels_[4];
    void makeBuffer_(unsigned char *buffer, uint16_t data);

  }; // end of class ADS131A04

} // end of namespace ADS131A04_ADC



#endif /* INCLUDE_THRUST_ESTIMATOR_HEADERS_ADS131A04_HPP_ */
