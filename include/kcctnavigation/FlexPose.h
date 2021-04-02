/**
* @file FlexPose.h
* @brief Flexible pose calculating class
* @author Shunya Hara
* @date 2021.1.31
* @details General purpose class for calculating coordinates and posture in ros.
*          Mutual conversion between tf, Pose, and PoseStamped
*          tf listen/broadcast
*/

#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <string>


class FlexPose{
    public:
    
    /// @brief constructor
    FlexPose();
    FlexPose(double x,double y,double z=0.0);
    FlexPose(geometry_msgs::Pose pose);
    FlexPose(geometry_msgs::PoseStamped pose);
    //FlexPose(std::string parent_id,std::string child_id);

    void setPosition(double x,double y,double z=0.0);
    void setPosition(geometry_msgs::Point position);
    void setOrientation(double x,double y,double z,double w);
    void setOrientation(geometry_msgs::Quaternion orientation);
    
    void setYaw(double angle);
    void setRoll(double angle);
    void setPitch(double angle);
    void setRPY(double roll,double pitch,double yaw);

    void setPose(geometry_msgs::Pose pose);
    void setPose(geometry_msgs::PoseStamped pose);
    //void setPose(std::string parent_id,std::string child_id);

    void setHeader(std_msgs::Header header);
    void setChildid(std::string child_frame_id);

    double getYaw();
    double getRoll();
    double getPitch();

    //Pose vector direction
    double getPoseYaw();

    geometry_msgs::Pose toPose();
    geometry_msgs::PoseStamped toPoseStamped();
    
    //void updateTF();
    //void sendTF();

    FlexPose operator+(const FlexPose& pose);
    FlexPose operator-(const FlexPose& pose);
    FlexPose operator*(double gain);

    FlexPose operator+=(const FlexPose& pose);
    FlexPose operator-=(const FlexPose& pose);
    FlexPose operator*=(double gain);

    double size();






    private:
    //tf::TransformListener listener;
    ///tf::TransformBroadcaster br;
    //tf::TransformListener listener1;
    geometry_msgs::PoseStamped pos;
    std::string child_id;
    

    //geometry_msgs::PoseStamped TFtoPoseStamped(std::string parent_id,std::string child_id);
    void EulerAnglesToQuaternion(double roll, double pitch, double yaw,double& q0, double& q1, double& q2, double& q3);
    geometry_msgs::Quaternion EulerAnglesToQuaternion(double roll, double pitch, double yaw);
    void geometry_quat_to_rpy(double& roll, double& pitch, double& yaw, geometry_msgs::Quaternion geometry_quat);
};










FlexPose::FlexPose(){}

FlexPose::FlexPose(double x,double y,double z){
    pos.pose.position.x=x;
    pos.pose.position.y=y;
    pos.pose.position.z=z;
}

FlexPose::FlexPose(geometry_msgs::Pose pose){
    pos.pose=pose;
}

FlexPose::FlexPose(geometry_msgs::PoseStamped pose){
    pos=pose;
}
/*
FlexPose::FlexPose(std::string parent_id,std::string child_id){
    pos=TFtoPoseStamped(parent_id,child_id);
}*/

void FlexPose::setPosition(double x,double y,double z){
    pos.pose.position.x=x;
    pos.pose.position.y=y;
    pos.pose.position.z=z;
}

void FlexPose::setPosition(geometry_msgs::Point position){
    pos.pose.position=position;
}

void FlexPose::setOrientation(double x,double y,double z,double w){
    pos.pose.orientation.x=x;
    pos.pose.orientation.y=y;
    pos.pose.orientation.z=z;
    pos.pose.orientation.w=w;
}

void FlexPose::setOrientation(geometry_msgs::Quaternion orientation){
    pos.pose.orientation=orientation;
}

void FlexPose::setYaw(double angle){
    pos.pose.orientation=EulerAnglesToQuaternion(0,0,angle);
}
void FlexPose::setRoll(double angle){
    pos.pose.orientation=EulerAnglesToQuaternion(angle,0,0);
}
void FlexPose::setPitch(double angle){
    pos.pose.orientation=EulerAnglesToQuaternion(0,angle,0);
}
void FlexPose::setRPY(double roll,double pitch,double yaw){
    pos.pose.orientation=EulerAnglesToQuaternion(roll,pitch,yaw);
}

void FlexPose::setPose(geometry_msgs::Pose pose){
    pos.pose=pose;
}
void FlexPose::setPose(geometry_msgs::PoseStamped pose){
    pos=pose;
}
/*
void FlexPose::setPose(std::string parent_id,std::string child_id){
    pos=TFtoPoseStamped(parent_id,child_id);
}*/

void FlexPose::setHeader(std_msgs::Header header){
    pos.header=header;
}
void FlexPose::setChildid(std::string child_frame_id){
    child_id=child_frame_id;
}

double FlexPose::getYaw(){
    double roll,pitch,yaw;
    geometry_quat_to_rpy(roll, pitch, yaw, pos.pose.orientation);
    
    return yaw;
}

double FlexPose::getRoll(){
    double roll,pitch,yaw;
    geometry_quat_to_rpy(roll, pitch, yaw, pos.pose.orientation);
    return roll;
}

double FlexPose::getPitch(){
    double roll,pitch,yaw;
    geometry_quat_to_rpy(roll, pitch, yaw, pos.pose.orientation);
    return pitch;
}

double FlexPose::getPoseYaw(){
    //std::cout<<"yaw="<<atan2(pos.pose.position.y,pos.pose.position.x)<<std::endl;
    return atan2(pos.pose.position.y,pos.pose.position.x);
}

geometry_msgs::Pose FlexPose::toPose(){
    return pos.pose;
}
geometry_msgs::PoseStamped FlexPose::toPoseStamped(){
    return pos;
}
/*
void FlexPose::updateTF(){
    TFtoPoseStamped(pos.header.frame_id,child_id);
}
/*
void FlexPose::sendTF(){
   tf::Transform transform;
   tf::poseMsgToTF(pos.pose,transform);
   br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),pos.header.frame_id, child_id));
}*/

FlexPose FlexPose::operator+(const FlexPose& pose){
    FlexPose calc;
    calc.pos=this->pos;
    calc.child_id=this->child_id;
    calc.pos.pose.position.x+=pose.pos.pose.position.x;
    calc.pos.pose.position.y+=pose.pos.pose.position.y;
    calc.pos.pose.position.z+=pose.pos.pose.position.z;
    return calc;
}

FlexPose FlexPose::operator-(const FlexPose& pose){
    FlexPose calc;
    calc.pos=this->pos;
    calc.child_id=this->child_id;
    calc.pos.pose.position.x-=pose.pos.pose.position.x;
    calc.pos.pose.position.y-=pose.pos.pose.position.y;
    calc.pos.pose.position.z-=pose.pos.pose.position.z;
    return calc;
}
FlexPose FlexPose::operator*(double gain){
    FlexPose calc;
    calc.pos=this->pos;
    calc.child_id=this->child_id;
    calc.pos.pose.position.x*=gain;
    calc.pos.pose.position.y*=gain;
    calc.pos.pose.position.z*=gain;
    return calc;
}

FlexPose FlexPose::operator+=(const FlexPose& pose){
    this->pos.pose.position.x+=pose.pos.pose.position.x;
    this->pos.pose.position.y+=pose.pos.pose.position.y;
    this->pos.pose.position.z+=pose.pos.pose.position.z;
    return *this;
}
FlexPose FlexPose::operator-=(const FlexPose& pose){
    this->pos.pose.position.x-=pose.pos.pose.position.x;
    this->pos.pose.position.y-=pose.pos.pose.position.y;
    this->pos.pose.position.z-=pose.pos.pose.position.z;
    return *this;
}
FlexPose FlexPose::operator*=(double gain){
    this->pos.pose.position.x*=gain;
    this->pos.pose.position.y*=gain;
    this->pos.pose.position.z*=gain;
    return *this;
}

double FlexPose::size(){
    double x2=this->pos.pose.position.x*this->pos.pose.position.x;
    double y2=this->pos.pose.position.y*this->pos.pose.position.y;
    double z2=this->pos.pose.position.z*this->pos.pose.position.z;
    
    return sqrt(x2+y2+z2);
}

void FlexPose::EulerAnglesToQuaternion(double roll, double pitch, double yaw,double& q0, double& q1, double& q2, double& q3){
            //q0:w q1:x q2:y q3:z
            double cosRoll = cos(roll / 2.0);
            double sinRoll = sin(roll / 2.0);
            double cosPitch = cos(pitch / 2.0);
            double sinPitch = sin(pitch / 2.0);
            double cosYaw = cos(yaw / 2.0);
            double sinYaw = sin(yaw / 2.0);

            q0 = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
            q1 = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
            q2 = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
            q3 = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
}

geometry_msgs::Quaternion FlexPose::EulerAnglesToQuaternion(double roll, double pitch, double yaw){
    //q0:w q1:x q2:y q3:z
    double cosRoll = cos(roll / 2.0);
    double sinRoll = sin(roll / 2.0);
    double cosPitch = cos(pitch / 2.0);
    double sinPitch = sin(pitch / 2.0);
    double cosYaw = cos(yaw / 2.0);
    double sinYaw = sin(yaw / 2.0);
    geometry_msgs::Quaternion quat;
    quat.w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
    quat.x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
    quat.y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
    quat.z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
    return quat;
}

void FlexPose::geometry_quat_to_rpy(double& roll, double& pitch, double& yaw, geometry_msgs::Quaternion geometry_quat){
    tf::Quaternion quat;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
}