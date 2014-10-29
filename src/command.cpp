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
#include "std_msgs/String.h"
#include "pctx_control/Control.h"
#include <dynamic_reconfigure/server.h>
#include <pctx_control/comm_dynConfig.h>
#include <sstream>

std::vector<int16_t> controlValues(9,0);

void paramsCallback(pctx_control::comm_dynConfig &config, uint32_t level)
{
    if (!config.bool_param)
    {
    controlValues[0] = config.Throttle ;
    std::cout << "controlValues[0]" << controlValues[0] << std::endl ;
    controlValues[1] = config.Roll_Aileron ;
    //std::cout << "controlValues[1]" << controlValues[1] << std::endl ;
    controlValues[2] = config.Pitch_Elevator ;
    controlValues[3] = config.Yaw_Rudder ;
    controlValues[4] = config.Posctl_switch ;
    controlValues[5] = config.loiter_switch ;
    controlValues[6] = config.main_mode_switch ;
    controlValues[7] = config.Return_Switch ;
  //  controlValues[8] = config.Channel_9 ;

    }
    else
    {
        controlValues[0] = 0;
        controlValues[1] = 0 ;
        controlValues[2] = 0 ;
        controlValues[3] = 0 ;
    }

}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_commander");
    ros::NodeHandle n;
    ros::Publisher uav_commands = n.advertise<pctx_control::Control>("sendPCTXControl", 1000);
    ros::Rate loop_rate(50);
    // int count = 0;

    dynamic_reconfigure::Server<pctx_control::comm_dynConfig> param_server;
    dynamic_reconfigure::Server<pctx_control::comm_dynConfig>::CallbackType param_callback_type;
    param_callback_type = boost::bind(&paramsCallback,  _1, _2);
    param_server.setCallback(param_callback_type);
    while (ros::ok())
    {


        pctx_control::Control controlMessage;
        // Arming values
        //        controlValues[0] = 0 ;
        //        controlValues[1] = 444;
        //        controlValues[2] = 454 ;
        //        controlValues[3] = 1000 ;

        //trim valuses
        //        controlValues[0] = 0 ;// throttole
        //        controlValues[1] = 444; // ROll in calculation 508
        //        controlValues[2] = 454 ;// Pitch
        //        controlValues[3] = 446 ;// Yaw

        // controlValues[4] =controlValues[5] =controlValues[6] =controlValues[7] =controlValues[8] =0;
        //        std::cout << "controlValues[0]" << controlValues[0] << std::endl ;
        //        std::cout << "controlValues[1]" << controlValues[1] << std::endl ;
        //        std::cout << "controlValues[2]" << controlValues[2] << std::endl ;
        //        std::cout << "controlValues[3]" << controlValues[3] << std::endl ;
        //        std::cout << "controlValues[4]" << controlValues[4] << std::endl ;
        //        std::cout << "controlValues[5]" << controlValues[5] << std::endl ;
        //        std::cout << "controlValues[6]" << controlValues[6] << std::endl ;
        //        std::cout << "controlValues[7]" << controlValues[7] << std::endl ;
        //        std::cout << "controlValues[8]" << controlValues[8] << std::endl ;

        controlMessage.values  = controlValues;
        //   ROS_INFO("UAV_Commander broadcasting to all channels value:%d",controlValues[0]);
        uav_commands.publish(controlMessage);
        ros::spinOnce();
        loop_rate.sleep();
        //count+=20;
    }
    return 0;
}
