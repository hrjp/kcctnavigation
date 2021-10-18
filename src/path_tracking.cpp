/**
* @file path_tracking.cpp
* @brief path
* @author Shunya Hara
* @date 2021.3.8
* @details pathを受けっとってそれをなぞって走行する
*/


#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include <kcctnavigation/TFtoPose.h>
#include <kcctnavigation/FlexPose.h>
#include <kcctnavigation/button_status.h>
#include <kcctnavigation/PurePursuit.h>

using namespace std;

int now_wp=0;
void set_wp_callback(const std_msgs::Int32& now_wp_){
    now_wp=now_wp_.data;
}

nav_msgs::Path path;
void path_callback(const nav_msgs::Path& path_){
    path=path_;
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "path_tracking_node");
    ros::NodeHandle n;
    //制御周期10Hz
    ros::Rate loop_rate(100);

    //param setting
    ros::NodeHandle pn("~");
    string map_id,baselink_id;
    double lookahead_distance=1.5;
    double looprate=10.0;
    double vel_max=2.0/3.6;
    double angular_vel_max=0.5;
    pn.param<string>("map_frame_id",map_id,"map");
    pn.param<string>("base_link_frame_id",baselink_id,"base_link");
    pn.param<double>("lookahead_distance",lookahead_distance,1.5);
    pn.param<double>("loop_rate",looprate,100.0);
    pn.param<double>("vel_max",vel_max,2.0/3.6);
    pn.param<double>("angular_vel_max",angular_vel_max,0.5);

    //map->base_link のTFを取得
    TFtoPose now_position(map_id,baselink_id,looprate);
    PurePursuit purepursuit(vel_max,angular_vel_max);

    ros::NodeHandle lSubscriber("");

    //waypoint/set subscliber
    ros::Subscriber set_wp_sub = lSubscriber.subscribe("waypoint/set", 50, set_wp_callback);
    //waypoint/path subscliber
    ros::Subscriber path_sub = lSubscriber.subscribe("waypoint/path", 50, path_callback);
    

    //2D_POSE_ESTIMATE publisher
    //ros::Publisher initial_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    //cmd_vel publisher
    ros::Publisher cmd_pub=n.advertise<geometry_msgs::Twist>("mcl_cmd_vel", 1);
    //wp now publisher
    ros::Publisher wpnow_pub=n.advertise<std_msgs::Int32>("waypoint/now", 1);

    geometry_msgs::Twist cmd_vel;
    geometry_msgs::Twist zero_vel;
    zero_vel.linear.x=0.0;
    zero_vel.angular.z=0.0;

    while (n.ok())  {
        if(path.poses.size()>0){
            auto nowpos=FlexPose(now_position.toPoseStamped());
            auto lookpos=FlexPose(path.poses.at(now_wp));

            if(now_wp<path.poses.size()-1){
                cmd_vel=purepursuit.getVelocity(nowpos,lookpos);
                if((lookpos-nowpos).size()<lookahead_distance){
                    now_wp++;
                }
            }
            else{
                cmd_vel=zero_vel;
            }
        }

        cmd_pub.publish(cmd_vel);

        std_msgs::Int32 now_wp_msg;
        now_wp_msg.data=now_wp;
        wpnow_pub.publish(now_wp_msg);

        ros::spinOnce();//subsucriberの割り込み関数はこの段階で実装される
        loop_rate.sleep();
        
    }
    
    return 0;
}
