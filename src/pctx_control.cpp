/***************************************************************************
* Copyright (C) 2013 - 2014 by                                             *
* Tarek Taha, Khalifa University Robotics Institute KURI                   *
*                     <tarek.taha@kustar.ac.ae>                            *
*                                                                          *
*                                                                          *
* This program is free software; you can redistribute it and/or modify     *
* it under the terms of the GNU General Public License as published by     *
* the Free Software Foundation; either version 2 of the License, or        *
* (at your option) any later version.                                      *
*                                                                          *
* This program is distributed in the hope that it will be useful,          *
* but WITHOUT ANY WARRANTY; without even the implied warranty of           *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
* GNU General Public License for more details.                             *
*                                                                          *
* You should have received a copy of the GNU General Public License        *
* along with this program; if not, write to the                            *
* Free Software Foundation, Inc.,                                          *
* 51 Franklin Steet, Fifth Floor, Boston, MA 02111-1307, USA.              *
***************************************************************************/
#include "ros/ros.h"
#include "pctx_control/Control.h"
#include "std_msgs/String.h"
#include "pctx_control/pctx.h" 

pctx_control::Control controlMsg;
Controller pctxController;

// Send control signal through the USB PCTX device to all the channels
void sendPCTXControl(const pctx_control::Control& msg)
{
    for(std::vector<int16_t>::const_iterator it = msg.values.begin(); it != msg.values.end(); ++it)
    {
        std::cout <<*it <<" ";
    }
    std::cout <<"\n";
    if(msg.values.size() == 9)
    {
        pctxController.transmit(msg.values[0],msg.values[1],msg.values[2],msg.values[3],msg.values[4],msg.values[5],msg.values[6],msg.values[7],msg.values[8]);
    }
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


