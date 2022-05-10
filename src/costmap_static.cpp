/**
* @file costmap_static.cpp
* @brief
* @author Akiro Harada
* @date
* @details 静的なオブジェクトのコストマップ
*/

// ROS
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2/utils.h>

// ROS <-> PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

#include <omp.h>

ros::Publisher grid_pub;
ros::Publisher pc_pub;

std::string map_frameId = "map";
std::string base_link_frameId = "base_link";

double resolution = 0.5;
double width = 20.0;
double occupancyThreshold = 0.5;
double beam_num = 360;
double logOdds_inc = 0.25;
double logOdds_dec = 0.65;

int grid_width = width/resolution;
int grid_num = grid_width*grid_width;
double width_2 = width/2;
int grid_width_2 = grid_width/2;

std::vector<bool> occupancyGridMap;

std::string remove_firstSlash(std::string frame_id){
    const int slash_pos = frame_id.find('/');
    if(slash_pos == 0)
        frame_id.erase(0, 1);
    return frame_id;
} // remove_firstSlash()

pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leaf_size){
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
    
    voxel_grid_filter.setInputCloud(cloud);
    voxel_grid_filter.setLeafSize (leaf_size, leaf_size, leaf_size);
    voxel_grid_filter.filter(*cloud_filtered); 

    return cloud_filtered;
} // downsample_cloud()

int get_x_index_from_index(const int index){
    return index % grid_width;
} // get_x_index_from_index()

int get_y_index_from_index(const int index){
    return index / grid_width;
} // get_y_index_from_index()

int get_index_from_xy(const double x, const double y)
{
    const int _x = floor(x / resolution + 0.5) + grid_width_2;
    const int _y = floor(y / resolution + 0.5) + grid_width_2;
    return _y * grid_width + _x;
} // get_index_fromxy()

bool is_validPoint(double x, double y)
{
    const int index = get_index_from_xy(x, y);
    if(x < -width_2 || x > width_2 || y < -width_2 || y > width_2){
        return false;
    }else if(index < 0 || grid_num <= index){
        return false;
    }else{
        return true;
    }
} // is_validPoint()

void transform_pointCloud(nav_msgs::Odometry odom, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, bool inv){
    Eigen::Matrix3f mat3 = Eigen::Quaternionf(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z).toRotationMatrix();
    Eigen::Matrix4f mat4 = Eigen::Matrix4f::Identity();
    mat4.block(0,0,3,3) = mat3;
    if(inv)
        pcl::transformPointCloud( *cloud, *cloud, mat4.inverse());
    else
        pcl::transformPointCloud( *cloud, *cloud, mat4);
}

void set_clear_GridCells(const std::vector<double>& beam_list, const std::vector<bool>& obstacle_indices, std::vector<bool>& map){
    std::vector<bool> clear_indices(grid_num, false);
    const double beam_angle_resolution = 2.0 * M_PI / (double)beam_num;

    for(int i=0;i<beam_num;i++){
        double direction = i * beam_angle_resolution - M_PI;
        direction = atan2(sin(direction), cos(direction));
        const double c = cos(direction);
        const double s = sin(direction);

        for(double range=0.0;range<beam_list[i];range+=resolution){
            const double x = range * c;
            const double y = range * s;
            if(is_validPoint(x, y)){
                const int index = get_index_from_xy(x, y);
                if(!obstacle_indices[index]){
                    clear_indices[index] = true;
                }else{
                    break;
                }
            } // if
            else
            {
                break;
            } // else
        } // for rage
    } // for i
    for(int i = 0; i < grid_num; ++i){
        if(clear_indices[i]){
            map[i] = false;
        }
    } // for i
} // set_clear_GridCells()


void input_cloud_to_occupancyGridMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_ptr){
    std::vector<double>beam_list(beam_num, sqrt(2) * width_2);
    const double beam_angle_resolution = 2.0 * M_PI / (double)beam_num;

    const int cloud_size = cloud_ptr->points.size();
    std::vector<bool> obstacle_indices(grid_num, false);

    #pragma omp parallel for
    for(int i=0;i<cloud_size;i++){
        const auto& p = cloud_ptr->points[i];
        if(!is_validPoint(p.x, p.y)){
            continue;
        } // if
        // occupancyGridmap[get_index_from_xy(p.x, p.y)].add_logOdds(0.01);
        const double distance = sqrt(p.x * p.x + p.y * p.y);
        const double direction = atan2(p.y, p.x);
        const int beam_index = (direction + M_PI) / beam_angle_resolution;
        if(0 <= beam_index && beam_index < beam_num){
            beam_list[beam_index] = std::min(beam_list[beam_index], distance);
        } // if
        const int index = get_index_from_xy(p.x, p.y);
        if(index < 0 || grid_num <= index){
            continue;
        } // if
        obstacle_indices[get_index_from_xy(p.x, p.y)] = true;
    } // for i

    for(int i=0;i<grid_num;i++){
        if(obstacle_indices[i]){
            occupancyGridMap[i] = true;
        } // if
    } // for i

    set_clear_GridCells(beam_list, obstacle_indices, occupancyGridMap);
} // input_cloud_to_occupancyGridMap()

void publish_occupancyGridMap(const ros::Time& stamp, const std::string& frame_id, nav_msgs::Odometry odom)
{
    nav_msgs::OccupancyGrid og;
    og.header.stamp = stamp;
    og.header.frame_id = frame_id;
    og.info.resolution = resolution;
    og.info.width = grid_width;
    og.info.height = grid_width;
    og.info.origin.position.x = -width_2+odom.pose.pose.position.x;
    og.info.origin.position.y = -width_2+odom.pose.pose.position.y;
    og.info.origin.position.z = +odom.pose.pose.position.z;
    og.info.origin.orientation.w = 1.0;
    og.data.resize(grid_num);
    for(int i=0;i<grid_num;i++){
        og.data[i] = occupancyGridMap[i]?100:1;
    } // for i
    og.header.stamp = ros::Time::now();
    grid_pub.publish(og);
} // public_OccupancyGridMap()

void callback(const sensor_msgs::PointCloud2ConstPtr& ros_pc_input, const nav_msgs::OdometryConstPtr& odom_input){
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_input (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*ros_pc_input, *pc_input);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    
    pc_filtered = downsample_cloud(pc_input, 0.5f);
    
    static Eigen::Vector2d last_odom_pos(odom_input->pose.pose.position.x, odom_input->pose.pose.position.y);
    static double last_odom_yaw = tf2::getYaw(odom_input->pose.pose.orientation);

    const Eigen::Vector2d odom_pos(odom_input->pose.pose.position.x, odom_input->pose.pose.position.y);
    const double odom_yaw = tf2::getYaw(odom_input->pose.pose.orientation);
    const Eigen::Vector2d diff_odom_pos = Eigen::Rotation2Dd(-last_odom_yaw).toRotationMatrix() * (odom_pos - last_odom_pos);
    double diff_odom_yaw = odom_yaw - last_odom_yaw;
    diff_odom_yaw = atan2(sin(diff_odom_yaw), cos(diff_odom_yaw));

    occupancyGridMap.clear();
    occupancyGridMap.resize(grid_num);

    transform_pointCloud(*odom_input, pc_filtered, false);
    input_cloud_to_occupancyGridMap(pc_filtered);
    publish_occupancyGridMap(odom_input->header.stamp, map_frameId, *odom_input);
    pc_pub.publish(ros_pc_input);

    last_odom_pos = odom_pos;
    last_odom_yaw = odom_yaw;
} // callback()

int main(int argc, char **argv){
    
    ros::init(argc, argv, "costmap_static");
    
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry>> sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry>(10), cloud_sub, odom_sub);

    cloud_sub.subscribe(nh, "/map2pc/pc_map_obstacle", 1);
    odom_sub.subscribe(nh, "/hdl_localization/odom", 1);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("/costmap_node/marged_costmap", 1);
    pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud/dynamic", 1);

    occupancyGridMap.resize(grid_num);

    ros::spin();

    return 0;
} // main()
