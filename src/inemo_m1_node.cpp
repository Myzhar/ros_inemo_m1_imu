#include <stdlib.h>
#include <ros/ros.h>

#include <inemo_m1_driver.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "st_inemo_node");

  inemo::CInemoDriver inemoDrv;

  ROS_INFO_STREAM("-----------------------------------\r");
  ROS_INFO_STREAM("    ST iNemo-M1 Discovery Node     \r");
  ROS_INFO_STREAM("-----------------------------------\r");

  if( !inemoDrv.startIMU() )
    return(EXIT_FAILURE);

  ros::spin();

  return(EXIT_SUCCESS);
}
