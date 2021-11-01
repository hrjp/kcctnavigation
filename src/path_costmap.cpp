/**
* @file path_costmap.cpp
* @brief path_costmap
* @author Shunya Hara
* @date 2021.11.1
* @details waypoint/path上が最小になるようなコストマップ
*/

#include <ros/ros.h>

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

using namespace std;

int main(int argc, char **argv){
    
    ros::init(argc, argv, "path_costmap");
    ros::NodeHandle n;
    //制御周期10Hz
    ros::Rate loop_rate(10);

    //param setting
    ros::NodeHandle pn("~");
    double width;
    pn.param<double>("width", width, 30.0);
    double height;
    pn.param<double>("height", height, 30.0);
    double resolution;
    pn.param<double>("resolution", resolution, 0.1);
    double update_frequency;
    pn.param<double>("update_frequency", update_frequency, 20.0);
    double publish_frequency;
    pn.param<double>("resolutionpublish_frequency", publish_frequency, 20.0);
    string global_frame;
    pn.param<string>("global_frame", global_frame, "map");
    string robot_base_frame;
    pn.param<string>("robot_base_frame", robot_base_frame, "base_link");

    while (n.ok())  {

        ros::spinOnce();//subsucriberの割り込み関数はこの段階で実装される
        loop_rate.sleep();

    }

    return 0;
}
