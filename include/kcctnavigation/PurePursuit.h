/**
* @file PurePursuit.h
* @brief PurePursuit calc
* @author Shunya Hara
* @date 2021.3.8
* @details pathを受けっとってそれをなぞって走行する
*/

#pragma once
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>

#include <kcctnavigation/FlexPose.h>


class PurePursuit{
    public:
    PurePursuit(double max_vel_,double max_anglular_vel_);
    geometry_msgs::Twist getVelocity(FlexPose nowpos,FlexPose wppos);
    
    private:
    double double_constrain(double val,double down_limit,double up_limit);
    double max_vel;
    double max_angular_vel;
};

PurePursuit::PurePursuit(double max_vel_,double max_anglular_vel_){
    max_vel=max_vel_;
    max_angular_vel=max_anglular_vel_;
}

geometry_msgs::Twist PurePursuit::getVelocity(FlexPose nowpos,FlexPose wppos){
    using namespace std;
    geometry_msgs::Twist calc_vel;
    //steering
    double l_dis=(nowpos-wppos).size();
    double angle1=(wppos-nowpos).getPoseYaw()-nowpos.getYaw()+M_PI*2.0;
    double angle2=(wppos-nowpos).getPoseYaw()-nowpos.getYaw();
    double angle3=(wppos-nowpos).getPoseYaw()-nowpos.getYaw()-M_PI*2.0;
    
    double alfa=0.0;
    if(abs(angle1)<abs(angle2)&&abs(angle1)<abs(angle3)){
        alfa=angle1;
    }
    else if(abs(angle2)<abs(angle1)&&abs(angle2)<abs(angle3)){
        alfa=angle2;
    }
    else if(abs(angle3)<abs(angle1)&&abs(angle3)<abs(angle2)){
        alfa=angle3;
    }
    
    alfa=double_constrain(alfa,-M_PI/2.0,M_PI/2.0);
    //cout<<"alfa"<<alfa;

    double limit_v=(max_angular_vel*l_dis)/(2.0*std::abs(sin(alfa)));
    //std::cout<<"v="<<limit_v<<std::endl;
    calc_vel.linear.x=double_constrain(max_vel,0,limit_v);
    calc_vel.angular.z=2.0*calc_vel.linear.x*sin(alfa)/l_dis;
    calc_vel.angular.z=double_constrain(calc_vel.angular.z,-max_angular_vel,max_angular_vel);
    //std::cout<<"v="<<calc_vel.angular.z<<std::endl;
    return calc_vel;
}

double PurePursuit::double_constrain(double val,double down_limit,double up_limit){
  if(val>up_limit){
    return up_limit;
  }
  if(val<down_limit){
    return down_limit;
  }
  return val;
}