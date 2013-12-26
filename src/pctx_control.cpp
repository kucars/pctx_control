#include "ros/ros.h"
#include "pctx_control/Control.h"
#include "std_msgs/String.h"
#include "pctx_control/pctx.h" 

pctx_control::Control controlMsg;
Controller pctxController;
void sendPCTXControl(const pctx_control::Control& msg)
{
  ROS_INFO("PTCX Control forwarding request to: [%d %d]", msg.channel,msg.value);
  pctxController.updateChannel(msg.channel, msg.value);
}

int main(int argc, char **argv)
{
    ROS_INFO("Try to connect to the R/C Radio ");
    while(!pctxController.connect())
    {
        sleep(1);
        ROS_INFO(".");
    }
  ros::init(argc, argv, "PCTXController");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("sendPCTXControl", 1000, sendPCTXControl);
  ros::spin();
  return 0;
}


