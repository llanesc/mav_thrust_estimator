/*  thrust_estimator
 *  A ROS implementation of a multirotor thrust estimator using strain gauges and the ADS131A04 ADC.
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

#include "ros/ros.h"

#include "../include/thrust_estimator_headers/ADS131A04.hpp"
#include "thrust_msgs/ThrustStrainGauge.h"

bool readParameters(ros::NodeHandle& nodeHandle, int* ForceConversion) {
  if (!nodeHandle.getParam("thrust_estimator_node/Channel1/ForceConversion", ForceConversion[0]))
    return false;
  if (!nodeHandle.getParam("thrust_estimator_node/Channel2/ForceConversion", ForceConversion[1]))
    return false;
  if (!nodeHandle.getParam("thrust_estimator_node/Channel3/ForceConversion", ForceConversion[2]))
      return false;
  if (!nodeHandle.getParam("thrust_estimator_node/Channel4/ForceConversion", ForceConversion[3]))
    return false;

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"thrust_estimator");
  ros::NodeHandle nh;

  ros::Publisher thrust_pub = nh.advertise<thrust_msgs::ThrustStrainGauge>("thrust_estimator/thrust", 10);

  thrust_msgs::ThrustStrainGauge thrust_msg;

  int ForceConvert[4];

  if (!readParameters(nh,ForceConvert)){
  ROS_ERROR("Could not read parameters.");
  ros::requestShutdown();
  }

  ros::Rate rate(100);

  ADS131A04_ADC::ADS131A04 ADC;

  std::cout << ADC.sendSystemCommand(ADS131A04_ADC::CMD_UNLOCK) << std::endl;

  std::cout << ADC.writeRegister(ADS131A04_ADC::ADDR_CLK1,0x02) << std::endl;

  std::cout << ADC.writeRegister(ADS131A04_ADC::ADDR_CLK2,0x20) << std::endl;


  if (ADC.enableADC() & ADC.sendSystemCommand(ADS131A04_ADC::CMD_WAKEUP)){
    while(ros::ok()) {
      ADC.pollDRDY();
      if (ADC.isDRDY()) {

        uint32_t* motors = ADC.getChannels();
        thrust_msg.thrust[0] = ((float)motors[0] - (float)(0x400000))/(float)ForceConvert[0];
        thrust_msg.thrust[1] = ((float)motors[1]);
        thrust_msg.thrust[2] = ((float)motors[2]);
        thrust_msg.thrust[3] = ((float)motors[3]);

        thrust_pub.publish(thrust_msg);

      } // if DRDY

      ros::spinOnce();
      rate.sleep();
    } // while loop
  } // if enableADC & wakeup

  return 1;
}
