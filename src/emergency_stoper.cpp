/**
* @file emergency_stoper.cpp
* @brief 
* @author Shunya Hara
* @date 2021.3.10
* @details 前方に障害物がある時に停止する
*/



#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>


#include <iostream>
#include <string>
#include <fstream>
#include <sstream>


using namespace std;


ros::Publisher linear_vel;
ros::Publisher angular_vel;

double pcl_dis;
void pcl_callback(const std_msgs::Float32& pcl_dis_){
    pcl_dis=pcl_dis_.data;
}

double double_constrain(double val,double down_limit,double up_limit){
  if(val>up_limit){
    return up_limit;
  }
  if(val<down_limit){
    return down_limit;
  }
  return val;
}

double max_vel=0.5;
double max_dis=5.0;
double min_dis=0.8;
ros::Publisher cmd_pub;

void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel_){
  static int stop_cou=0;
    geometry_msgs::Twist cmd_vel=cmd_vel_;
    double vel_limit=(pcl_dis-min_dis)/(max_dis-min_dis)*max_vel;
    cmd_vel.linear.x=double_constrain(cmd_vel.linear.x,0.0,vel_limit);

  if(cmd_vel.linear.x==0.0){
    stop_cou=10;
  }
  else{
    if(stop_cou>0){
      stop_cou--;
    }
  }
  if(stop_cou>0){
    cmd_vel.linear.x=0.0;
  }
    cmd_pub.publish(cmd_vel);

    //直進速度表示
        std_msgs::Float32 linear_vel_data;
        linear_vel_data.data=cmd_vel.linear.x;
        linear_vel.publish(linear_vel_data);

        // 旋回速度表示
        std_msgs::Float32 angular_vel_data;
        angular_vel_data.data=cmd_vel.angular.z;
        angular_vel.publish(angular_vel_data);

}



int main(int argc, char **argv){
    
    ros::init(argc, argv, "emergency_stoper_node");
    ros::NodeHandle n;
    
    double looprate=10.0;

    //param setting
    ros::NodeHandle pn("~");
    pn.param<double>("max_vel",max_vel,1.0);
    pn.param<double>("loop_rate",looprate,10.0);
    pn.param<double>("max_dis",max_dis,5.0);
    pn.param<double>("min_dis",min_dis,0.5);


    //制御周期 Hz
    ros::Rate loop_rate(looprate);

    ros::NodeHandle lSubscriber("");

    //pcl_handler subscliber
    ros::Subscriber pcl_sub = lSubscriber.subscribe("/pcl_handler/front_dist", 50, pcl_callback);

    //cmd_vel subscliber
    ros::Subscriber cmd_vel_sub = lSubscriber.subscribe("selected_cmd_vel", 50, cmd_vel_callback);


    //cmd_vel publisher
    cmd_pub=n.advertise<geometry_msgs::Twist>("final_cmd_vel", 1);

    //直進速度　publisher
    linear_vel = n.advertise<std_msgs::Float32>("linear_vel", 10);

    //旋回速度　publisher
    angular_vel = n.advertise<std_msgs::Float32>("angular_vel", 10);


    ros::spin();
    
    return 0;
}