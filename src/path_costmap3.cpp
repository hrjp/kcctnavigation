/**
* @file path_costmap.cpp
* @brief path_costmap
* @author Shunya Hara
* @date 2021.11.1
* @details waypoint/path上が最小になるようなコストマップ 差分計算による軽量化Ver.
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
//#include <omp.h>
#include <time.h>

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
inline double pose_dist2d(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2){
    double dist_x=pose1.position.x-pose2.position.x;
    double dist_y=pose1.position.y-pose2.position.y;
    return std::sqrt(dist_x*dist_x+dist_y*dist_y);
}

//2つのposeの２次元距離の2乗を計算
inline double pose_dist2dx2(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2){
    double dist_x=pose1.position.x-pose2.position.x;
    double dist_y=pose1.position.y-pose2.position.y;
    return dist_x*dist_x+dist_y*dist_y;
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

//quaternion->rpy convert function
inline void geometry_quat_to_rpy(double& roll, double& pitch, double& yaw, geometry_msgs::Quaternion& geometry_quat){
    tf::Quaternion quat;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
}

//pose1とpose2を通る直線とpose3が垂直に交わる点pose4を求める(直線上でpose3に一番近い点)
inline double line_point_nn_pose(const geometry_msgs::Pose& pose1,const geometry_msgs::Pose& pose2,const geometry_msgs::Pose& pose3,geometry_msgs::Pose& pose4){
    //pathの座標取得
    double x0=pose1.position.x;
    double y0=pose1.position.y;
    double x1=pose2.position.x;
    double y1=pose2.position.y;
    double x2=pose3.position.x;
    double y2=pose3.position.y;
    //pathの方程式を求める
    double a0=(y1-y0)/(x1-x0);
    double b0=-a0*x0+y0;
    //自己位置を通りpathに垂直な直線の方程式を求める
    double a1=-(1/a0);
    double b1=y2-a1*x2;
    //2直線の交点を求める
    pose4.position.x=(b1-b0)/(a0-a1);
    pose4.position.y=(a0*b1-b0*a1)/(a0-a1);

    //pose1->pose2 * pose1->pose3 内積
    double ip1=(x1-x0)*(x2-x0)+(y1-y0)*(y2-y0);
    //pose2->pose1 * pose2->pose3 内積
    double ip2=(x0-x1)*(x2-x1)+(y0-y1)*(y2-y1);
    if(ip1<0){
        pose4=pose1;
        return pose_dist2d(pose1,pose3);
    }
    if(ip2<0){
        pose4=pose2;
        return pose_dist2d(pose2,pose3);
    }
    return pose_dist2d(pose4,pose3);

}


int main(int argc, char **argv){
    
    ros::init(argc, argv, "path_costmap");
    ros::NodeHandle n;
    

    //param setting
    ros::NodeHandle pn("~/my_costmap");
    double width;
    pn.param<double>("width", width, 30.0);
    double height;
    pn.param<double>("height", height, 30.0);
    double resolution;
    pn.param<double>("resolution", resolution, 0.1);
    //double update_frequency;
    //pn.param<double>("update_frequency", update_frequency, 20.0);
    double publish_frequency;
    pn.param<double>("publish_frequency", publish_frequency, 5.0);
    string global_frame;
    pn.param<string>("global_frame", global_frame, "map");
    string robot_base_frame;
    pn.param<string>("robot_base_frame", robot_base_frame, "base_link");

    //固有パラメータ
    //costの傾き costが0から100まで変化する幅[m]
    double cost_path_width;
    pn.param<double>("path_costmap/cost_path_width", cost_path_width, 10.0);
    
    //このパラメータ以上経路から外れた領域のコストを100にする 実質的にロボットが動ける道幅を指定できる[m]
    double cost_wall_width;
    pn.param<double>("path_costmap/cost_wall_width", cost_wall_width, cost_path_width);


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
    costmap.header.frame_id=global_frame;
    costmap.data.resize(gw*gh);
    costmap.info.width=gw;
    costmap.info.height=gh;
    costmap.info.resolution=resolution;

    //tf listener
    //tf::TransformListener tflistener;

    static geometry_msgs::Pose init_pose;
    init_pose.position.x=2.0*height;
    init_pose.position.y=2.0*width;

    //初回の計算flag
    bool init_calc_flag=true;

    while (n.ok())  {
        if(path.poses.size()>2){
            ros::Time start_time=ros::Time::now();
            static int costmap_seq=0;
            geometry_msgs::Pose robotpose=now_pose.toPose();
            geometry_msgs::PoseStamped robotposestamped=now_pose.toPoseStamped();
            static geometry_msgs::Pose pre_robotpose=init_pose;
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

            //各gridのコストを計算
            //int j=0;

            //一つ前のフレームからの変化量（移動量）を計算
            int intpose_x=int(robotpose.position.x/resolution);
            int intpose_y=int(robotpose.position.y/resolution);
            static int pre_intpose_x,pre_intpose_y;
            int diff_x=intpose_x-pre_intpose_x;
            int diff_y=intpose_y-pre_intpose_y;
            if(init_calc_flag){
                diff_x=gh-1;
                diff_y=gw-1;
                init_calc_flag=false;
            }
            else{
                diff_x=clip(diff_x,-gh+1,gh-1);
                diff_y=clip(diff_y,-gw+1,gw-1);
            }
            
            pre_intpose_x=intpose_x;
            pre_intpose_y=intpose_y;
            

            //前回からの移動量だけ配列をシフト
            for(int gy=0;gy<gw-std::abs(diff_y);gy++){
                for(int gx=0;gx<gh-std::abs(diff_x);gx++){
                    int dx=diff_x>0?gx:gh-gx;
                    int dy=diff_y>0?gy:gw-gy;

                    int before_addr=(dy+diff_y)*gw+(dx+diff_x);
                    int after_addr=dy*gw+dx;

                    //std::cout<<"size"<<costmap.data.size()<<" dx="<<diff_x<<" dy="<<diff_y<<" after="<<after_addr<<" before="<<before_addr<<std::endl;
                    costmap.data[after_addr]=costmap.data[before_addr];
                }
            }
            

            //新しい範囲だけコスト計算
            //diff_y*dhの長方形のコスト計算
            
            for(int gy=0;gy<std::abs(diff_y);gy++){
                for(int gx=0;gx<gh;gx++){
                    int dx=gx;
                    int dy=diff_y<0?gy:gw-gy-1;
                    geometry_msgs::Pose costmap_pose;
                    //mapからみたgridの座標を計算
                    costmap_pose.position.x=robotpose.position.x+double(dx)*resolution+map_x0;
                    costmap_pose.position.y=robotpose.position.y+double(dy)*resolution+map_y0;
                    costmap_pose.orientation.w=1.0;

                    //現在の選択しているグリッドから一番近いwaypointpath上の点を選び距離を求める
                    int nb_dis_num=0;
                    double nn_online_dis_min=cost_wall_width+100.0;
                    geometry_msgs::Pose nn_online_pose;
                    for(int i=downlimit;i<uplimit;i++){
                        double nn_online_dis=line_point_nn_pose(path.poses[i].pose,path.poses[i+1].pose,costmap_pose,nn_online_pose);
                        if(nn_online_dis<nn_online_dis_min){
                            nn_online_dis_min=nn_online_dis;
                            nb_dis_num=i;
                        }
                    }
                    
                    costmap.data[dy*gw+dx]=(nn_online_dis_min>cost_wall_width)?100:clip(int(nn_online_dis_min/cost_path_width*100.0),0,100);

                    //j++;
                }
                
            }

            //diff_x*dwの長方形のコスト計算
            //diff_x*diff_yの範囲は上とかぶっているので２回目だが微小なので無視
            /*
            for(int gx=0;gx<std::abs(diff_x);gx++){
                for(int gy=0;gy<gw;gy++){
                    int dx=diff_x<0?gx:gh-gx-1;
                    int dy=gy;
                    geometry_msgs::Pose costmap_pose;
                    //mapからみたgridの座標を計算
                    costmap_pose.position.x=robotpose.position.x+double(dx)*resolution+map_x0;
                    costmap_pose.position.y=robotpose.position.y+double(dy)*resolution+map_y0;
                    costmap_pose.orientation.w=1.0;

                    //現在の選択しているグリッドから一番近いwaypointpath上の点を選び距離を求める
                    int nb_dis_num=0;
                    double nn_online_dis_min=cost_wall_width+100.0;
                    geometry_msgs::Pose nn_online_pose;
                    for(int i=downlimit;i<uplimit;i++){
                        double nn_online_dis=line_point_nn_pose(path.poses[i].pose,path.poses[i+1].pose,costmap_pose,nn_online_pose);
                        if(nn_online_dis<nn_online_dis_min){
                            nn_online_dis_min=nn_online_dis;
                            nb_dis_num=i;
                        }
                    }
                    
                    costmap.data[dy*gw+dx]=(nn_online_dis_min>cost_wall_width)?100:clip(int(nn_online_dis_min/cost_path_width*100.0),0,100);
                    //j++;
                }
                
            }*/
        
        
            //costmap publish 
            
            costmap.header.seq=costmap_seq;
            costmap_seq++;
            costmap.header.stamp=ros::Time::now();
            costmap.info.origin.position.x=map_x0+robotpose.position.x;
            costmap.info.origin.position.y=map_y0+robotpose.position.y;
            costmap.info.origin.position.z=robotpose.position.z;
            costmap_pub.publish(costmap);
            //std::cout<<"CALC TIME : "<<(ros::Time::now()-start_time).toSec()<<std::endl;
            
        }
        ros::spinOnce();//subsucriberの割り込み関数はこの段階で実装される
        loop_rate.sleep();

    }

    return 0;
}
