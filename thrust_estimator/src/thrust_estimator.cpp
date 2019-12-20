#include "ros/ros.h"

#include "../include/thrust_estimator_headers/ADS131A04.hpp"
#include "thrust_msgs/ThrustStrainGauge.h"

ADS131A04_ADC::ADS131A04 * ADS131A04_ADC::ADS131A04::classPtr;

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

  thrust_msg.thrust.reserve(4);

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
      if (ADC.isDRDY()) {


        uint32_t* motors = ADC.getChannels();
        thrust_msg.thrust[0] = ((float)motors[1] - float(0x400000))/ForceConvert[0];
        thrust_msg.thrust[1] = ((float)motors[1] - float(0x400000))/ForceConvert[1];
        thrust_msg.thrust[2] = ((float)motors[2] - float(0x400000))/ForceConvert[2];
        thrust_msg.thrust[3] = ((float)motors[3] - float(0x400000))/ForceConvert[3];

        thrust_pub.publish(thrust_msg);

        ros::spinOnce();
        rate.sleep();
      } // if DRDY
    } // while loop
  } // if enableADC & wakeup
  return 1;
}
