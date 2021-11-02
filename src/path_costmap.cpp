/**
* @file path_costmap.cpp
* @brief path_costmap
* @author Shunya Hara
* @date 2021.11.1
* @details waypoint/path上が最小になるようなコストマップ
*/

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/OccupancyGrid.h>

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include <kcctnavigation/TFtoPose.h>

using namespace std;

std_msgs::Int32 now_wp;
void now_wp_callback(const std_msgs::Int32& now_wp_message){
    now_wp = now_wp_message;
}
nav_msgs::Path path;
void path_callback(const nav_msgs::Path& path_message){
    path = path_message;
}

class PathCostmap{
public:
    PathCostmap();
    void update(nav_msgs::OccupancyGrid& costmap, const nav_msgs::Path& path, const std_msgs::Int32& now_wp, const geometry_msgs::Pose& now_pose);
private:

};

void PathCostmap::update(nav_msgs::OccupancyGrid& costmap, const nav_msgs::Path& path, const std_msgs::Int32& now_wp, const geometry_msgs::Pose& now_pose){

}

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

    ros::NodeHandle lSubscriber("");

    //Path subscliber
    ros::Subscriber path_sub = lSubscriber.subscribe("waypoint/path", 50, path_callback);
    //Now subscliber
    ros::Subscriber now_wp_sub = lSubscriber.subscribe("waypoint/now", 50, now_wp_callback);

    //costmap publisher
    ros::Publisher costmap_pub=n.advertise<nav_msgs::OccupancyGrid>("", 1);
    
    //Now pose from TF
    TFtoPose now_pose(global_frame,robot_base_frame);

    while (n.ok())  {

        ros::spinOnce();//subsucriberの割り込み関数はこの段階で実装される
        loop_rate.sleep();

    }

    return 0;
}
