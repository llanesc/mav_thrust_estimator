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

  wiringPiSetup();
  if ((myFd = wiringPiSPISetupMode(CHANNEL,8000000,1)) < 0)
  {
    fprintf (stderr, "Can't open the SPI bus: %s\n", strerror (errno)) ;
    exit (EXIT_FAILURE) ;
  }

  wiringPiISR(3,INT_EDGE_FALLING, &readChannelsExt);

  sendSystemCommand(CMD_NULL);

}

ADS131A04::~ADS131A04()
{

}

void ADS131A04::readChannelsExt()
{
    ADS131A04::classPtr ->readChannels();
}

bool ADS131A04::sendSystemCommand(systemCommands cmd)
{
  if (ADC_ENA_) {
    unsigned char tbuffer[15];
    unsigned char rbuffer[15];

    uint16_t deviceWord = cmd;
     makeBuffer_(tbuffer,deviceWord);
     printf("%02X %02X %02X \n",tbuffer[0],tbuffer[1],tbuffer[2]);
     wiringPiSPIDataRW(CHANNEL,tbuffer,sizeof(tbuffer)) ;
     unsigned char responseMask[3];

     if (cmd == CMD_RESET) {
       do {
       makeBuffer_(rbuffer,CMD_NULL);
       wiringPiSPIDataRW(CHANNEL,rbuffer,sizeof(rbuffer));
       printf("%02X %02X %02X \n",rbuffer[0],rbuffer[1],rbuffer[2]);
       uint16_t DeviceWordResponse = READY;
       makeBuffer_(responseMask,DeviceWordResponse);
       } while(rbuffer[0] != responseMask[0] & rbuffer[1] != responseMask[1] & rbuffer[2] != responseMask[2]);
     } else {
       makeBuffer_(rbuffer,CMD_NULL);
       wiringPiSPIDataRW(CHANNEL,rbuffer,sizeof(rbuffer)) ;
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
    unsigned char tbuffer[3];
    unsigned char rbuffer[3];

    uint16_t deviceWord = cmd;
     makeBuffer_(tbuffer,deviceWord);
     printf("%02X %02X %02X \n",tbuffer[0],tbuffer[1],tbuffer[2]);
     wiringPiSPIDataRW(CHANNEL,tbuffer,sizeof(tbuffer)) ;
     unsigned char responseMask[3];

     if (cmd == CMD_RESET) {
       do {
       makeBuffer_(rbuffer,CMD_NULL);
       wiringPiSPIDataRW(CHANNEL,rbuffer,sizeof(rbuffer));
       printf("%02X %02X %02X \n",rbuffer[0],rbuffer[1],rbuffer[2]);
       uint16_t DeviceWordResponse = READY;
       makeBuffer_(responseMask,DeviceWordResponse);
       } while(rbuffer[0] != responseMask[0] & rbuffer[1] != responseMask[1] & rbuffer[2] != responseMask[2]);
     } else {
       makeBuffer_(rbuffer,CMD_NULL);
       wiringPiSPIDataRW(CHANNEL,rbuffer,sizeof(rbuffer)) ;
       printf("%02X %02X %02X \n",rbuffer[0],rbuffer[1],rbuffer[2]);
       uint16_t DeviceWordResponse = cmd;
       makeBuffer_(responseMask,DeviceWordResponse);
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
  unsigned char tbuffer[3];
  unsigned char rbuffer[3];
  uint16_t deviceWord = RREG | statusADDR;
  makeBuffer_(tbuffer,deviceWord);
  makeBuffer_(rbuffer,CMD_NULL);
  wiringPiSPIDataRW(CHANNEL,rbuffer,sizeof(rbuffer)) ;
  return (rbuffer[0] | uint32_t(rbuffer[1]) << 8 | uint32_t(rbuffer[2]) << 16);
}

uint32_t ADS131A04::readRegister(configRegisterAddress configADDR)
{
  unsigned char tbuffer[3];
  unsigned char rbuffer[3];
  uint16_t deviceWord = RREG | configADDR;
  makeBuffer_(tbuffer,deviceWord);
  wiringPiSPIDataRW(CHANNEL,tbuffer,sizeof(tbuffer)) ;

  makeBuffer_(rbuffer,CMD_NULL);
  wiringPiSPIDataRW(CHANNEL,rbuffer,sizeof(rbuffer)) ;

  return (rbuffer[0] | uint32_t(rbuffer[1]) << 8 | uint32_t(rbuffer[2]) << 16);
}

bool ADS131A04::writeRegister(configRegisterAddress configADDR, uint16_t data)
{
  unsigned char tbuffer[3];
  unsigned char rbuffer[3];
  uint16_t deviceWord = WREG | configADDR | data;
  makeBuffer_(tbuffer,deviceWord);
  printf("%02X %02X %02X \n",tbuffer[0],tbuffer[1],tbuffer[2]);
  wiringPiSPIDataRW(CHANNEL,tbuffer,sizeof(tbuffer)) ;

  makeBuffer_(rbuffer,CMD_NULL);
  wiringPiSPIDataRW(CHANNEL,rbuffer,sizeof(rbuffer)) ;
  printf("%02X %02X %02X \n",rbuffer[0],rbuffer[1],rbuffer[2]);

  unsigned char responseMask[3];
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

  unsigned char outputBuffer[15];
  makeBuffer_(outputBuffer,CMD_NULL);
  wiringPiSPIDataRW(CHANNEL,outputBuffer,sizeof(outputBuffer)) ;
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

void ADS131A04::makeBuffer_(unsigned char *buffer, uint16_t data){
  for(int i = 0;i < sizeof(buffer);i++){
    buffer[i] &= 0x00;
  }
  buffer[1] = data & 0xFF;
  buffer[0] = data >> 8;
}

} /*namespace ADS131A04_ADC*/
