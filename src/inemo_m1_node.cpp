#include <stdlib.h>
#include <ros/ros.h>
#include <ros/console.h>


#include "driver/include/inemo_m1_driver.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "st_inemo_node");

  inemo::CInemoDriver inemoDrv;

  ROS_INFO_STREAM("-----------------------------------\r");
  ROS_INFO_STREAM("    ST iNemo-M1 Discovery Node     \r");
  ROS_INFO_STREAM("-----------------------------------\r");

  /*if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
  {
     ros::console::notifyLoggerLevelsChanged();
  }//*/

  if( !inemoDrv.startIMU() )
    return(EXIT_FAILURE);

  ros::spin();

  ROS_INFO_STREAM("... stopped!");

  return(EXIT_SUCCESS);
}
