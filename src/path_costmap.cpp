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
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include <kcctnavigation/TFtoPose.h>

using namespace std;


//callback functions
std_msgs::Int32 now_wp;
void now_wp_callback(const std_msgs::Int32& now_wp_message){
    now_wp = now_wp_message;
}
nav_msgs::Path path;
void path_callback(const nav_msgs::Path& path_message){
    path = path_message;
}

//calc functions

//2つのposeの２次元距離を計算
double pose_dist2d(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2){
    double dist_x=pose1.position.x-pose2.position.x;
    double dist_y=pose1.position.y-pose2.position.y;
    return std::sqrt(dist_x*dist_x+dist_y*dist_y);
}
//vectorのrangeoutを判定する
template <class T> bool vector_rangeout(const T& vec, const int range){
    if(0<=range && range<vec.size()){
        return true;
    }
    return false;
}

//最大最小でクリップする
template <class T> T clip(const T& n, const T& lower, const T& upper){
    T number = std::max(lower, std::min(n, upper));
    return number;
}

void geometry_quat_to_rpy(double& roll, double& pitch, double& yaw, geometry_msgs::Quaternion& geometry_quat){
    tf::Quaternion quat;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
}


int main(int argc, char **argv){
    
    ros::init(argc, argv, "path_costmap");
    ros::NodeHandle n;
    

    //param setting
    ros::NodeHandle pn("~");
    double width;
    pn.param<double>("width", width, 20.0);
    double height;
    pn.param<double>("height", height, 20.0);
    double resolution;
    pn.param<double>("resolution", resolution, 0.1);
    //double update_frequency;
    //pn.param<double>("update_frequency", update_frequency, 20.0);
    double publish_frequency;
    pn.param<double>("publish_frequency", publish_frequency, 1.0);
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
    ros::Publisher costmap_pub=n.advertise<nav_msgs::OccupancyGrid>("/costmap_node/path_costmap", 1);
    
    //Now pose from TF
    TFtoPose now_pose(global_frame,robot_base_frame);

    //制御周期
    ros::Rate loop_rate(publish_frequency);

    //grid_size
    int gw=width/resolution;//y
    int gh=height/resolution;//x

    //map 原点
    double map_x0=-height/2.0;
    double map_y0=-width/2.0;

    //mapの中心から四隅までの距離
    double map_radius=std::sqrt(width*width/4.0+height*height/4.0);

    //costmap init
    nav_msgs::OccupancyGrid costmap;
    costmap.header.frame_id=robot_base_frame;
    costmap.data.resize(gw*gh);
    costmap.info.width=gw;
    costmap.info.height=gh;
    costmap.info.resolution=resolution;

    //tf listener
    tf::TransformListener tflistener;

    while (n.ok())  {
        static int costmap_seq=0;
        geometry_msgs::Pose robotpose=now_pose.toPose();
        geometry_msgs::PoseStamped robotposestamped=now_pose.toPoseStamped();

        //pathの探索範囲を決定
        //上限
        int uplimit=now_wp.data;
        while(vector_rangeout(path.poses,uplimit)){
            if(pose_dist2d(robotpose,path.poses[uplimit].pose)>map_radius){
                break;
            }
            uplimit++;
        }
        uplimit=clip(uplimit,0,int(path.poses.size())-1);
        //下限
        int downlimit=now_wp.data;
        while(vector_rangeout(path.poses,downlimit)){
            if(pose_dist2d(robotpose,path.poses[downlimit].pose)>map_radius){
                break;
            }
            downlimit--;
        }
        downlimit=clip(downlimit,0,int(path.poses.size())-1);

        //
        double roll,pitch,yaw;
        geometry_quat_to_rpy(roll,pitch,yaw,robotpose.orientation);
        double sinsita=sin(-yaw);
        double cossita=cos(-yaw);
        //各gridのコストを計算
        int j=0;
        geometry_msgs::PoseStamped costmap_pose,global_costmap_pose;
        for(int gx=0;gx<gh;gx++){
            for(int gy=0;gy<gw;gy++){
                //double a=127.0+127.0*sin(2.0*M_PI*double(j)/148.0);
                //costmap.data[j]=int(a);
                //std::cout<<"AAAAAAAAAAAAAA"<<std::endl;
                //ロボットからみたgridの座標
                double dgx=double(gx)*resolution+map_x0;
                double dgy=double(gy)*resolution+map_y0;
                /*costmap_pose.pose.position.x=double(gx)*resolution+map_x0;
                costmap_pose.pose.position.y=double(gy)*resolution+map_y0;
                costmap_pose.pose.orientation.w=1.0;
                costmap_pose.header.frame_id=robot_base_frame;
                costmap_pose.header.stamp=ros::Time::now();
                costmap_pose.header.seq=costmap_seq;*/
                //std::cout<<"BB"<<std::endl;

                //mapからみたgridの座標に変換
                global_costmap_pose.pose.position.x=robotpose.position.x+dgx*cossita-dgy*sinsita;
                global_costmap_pose.pose.position.y=robotpose.position.y+dgx*sinsita+dgy*cossita;
                global_costmap_pose.pose.orientation.w=1.0;
                global_costmap_pose.header.frame_id=global_frame;
                global_costmap_pose.header.stamp=ros::Time::now();
                global_costmap_pose.header.seq=costmap_seq;

                //try {
                //    tflistener.transformPose(global_frame,costmap_pose,global_costmap_pose);
                //} catch(tf::TransformException& ex) {
                    //ROS_ERROR_STREAM(" error is " << ex.what());
                //}
                //tflistener.transformPose(robot_base_frame,costmap_pose,global_costmap_pose);
                //std::cout<<"up"<<uplimit<<"down"<<downlimit<<std::endl;
                //最近傍点探索
                double nb_dis_min=1000.0;
                int nb_dis_num=0;
                for(int i=downlimit;i<uplimit;i++){
                    double nb_dis=pose_dist2d(path.poses[i].pose,global_costmap_pose.pose);
                    if(nb_dis<nb_dis_min){
                        nb_dis_min=nb_dis;
                        nb_dis_num=i;
                    }
                }
                costmap.data[j]=clip(int(nb_dis_min/5.0*255.0),0,255);


                j++;
            }
            
        }
        
        //costmap publish 
        
        costmap.header.seq=costmap_seq;
        costmap_seq++;
        costmap.header.stamp=ros::Time::now();
        costmap.info.origin.position.x=map_x0;
        costmap.info.origin.position.y=map_y0;
        costmap_pub.publish(costmap);

        ros::spinOnce();//subsucriberの割り込み関数はこの段階で実装される
        loop_rate.sleep();

    }

    return 0;
}
