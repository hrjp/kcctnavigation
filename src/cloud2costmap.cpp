#ifndef CLOUD_2_COSTMAP_CPP
#define CLOUD_2_COSTMAP_CPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

#include <omp.h>

nav_msgs::OccupancyGrid og_total;

double resolution = 0.25;
double width = 20.0;

int grid_width = width/resolution;
int grid_num = grid_width*grid_width;
double width_2 = width/2.0;
int grid_width_2 = grid_width/2.0;

pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leaf_size){
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
    
    voxel_grid_filter.setInputCloud(cloud);
    voxel_grid_filter.setLeafSize (leaf_size, leaf_size, leaf_size);
    voxel_grid_filter.filter(*cloud_filtered); 

    return cloud_filtered;
} // downsample_cloud()

pcl::PointCloud<pcl::PointXYZ>::Ptr crop_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint){

    pcl::CropBox<pcl::PointXYZ> crop_box;
    crop_box.setInputCloud(cloud);

    crop_box.setMin(minPoint);
    crop_box.setMax(maxPoint);

    crop_box.filter(*cloud);    

    return cloud;
} // crop_cloud()

pcl::PointCloud<pcl::PointXYZ>::Ptr remove_roof(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint){

    std::vector<int> indices;

    pcl::CropBox<pcl::PointXYZ> roof(true);

    roof.setMin(minPoint);
    roof.setMax(maxPoint);
    roof.setInputCloud(cloud);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    for(int point:indices){
    	inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud);

    return cloud;
} // remove_roof()

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

bool is_validIndex(int index)
{
    return (0 <= index && index < grid_num)?true:false;
} // is_validIndex()

void input_cloud_to_occupancyGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_ptr, nav_msgs::OccupancyGrid& og, int radius){
    const int cloud_size = cloud_ptr->points.size();
    for(int i=0;i<grid_num;i++)og.data[i]=0;
    #pragma omp parallel for
    for(int i=0;i<cloud_size;i++){
        const auto& p = cloud_ptr->points[i];
        if(!is_validPoint(p.x, p.y)){
            continue;
        } // if
        const int index = get_index_from_xy(p.x, p.y);
        if(index < 0 || grid_num <= index){
            continue;
        } // if
        og.data[get_index_from_xy(p.x, p.y)]++;
        for(int j = -radius; j <= radius; j++)
        {
            for(int k = -radius; k <= radius; k++)
            {
                if(j*j+k*k > radius*radius) continue;
                if(0 <= get_index_from_xy(p.x, p.y)+j*grid_width+k && get_index_from_xy(p.x, p.y)+j*grid_width+k < grid_num)
                    og.data[get_index_from_xy(p.x, p.y)+j*grid_width+k] = 1;
            }
        }
    } // for i
} // input_cloud_to_occupancyGridMap()

void makeRoof_to_roadOccupancyGrid(nav_msgs::OccupancyGrid&og, int size)
{
    for(int i = grid_width_2-size; i < grid_width_2+size; i++)
    {
        for(int j = grid_width_2-size; j < grid_width_2+size; j++)
        {
            og.data[i*grid_width+j] = 1;
        }
    }
}

void dilation_roadOccupancyGrid(nav_msgs::OccupancyGrid& og, int size)
{
    nav_msgs::OccupancyGrid og_tmp;
    og_tmp.data.resize(grid_num);

    for(int i=0;i<grid_num;i++)og_tmp.data[i]=0;

    for(int y = 0; y < grid_width; y++)
    {
        for(int x = 0; x < grid_width; x++)
        {
            if(og.data[y*grid_width+x] == 0) continue;
            
            double tan = (double)(y - grid_width_2) / (double)(x - grid_width_2);
            
            int x_, y_;
            for(int i = -size; i <= size; i++)
            {
                x_ = x+i;
                y_ = y+(int)(i*tan);
                if(is_validIndex(y_*grid_width+x_))og_tmp.data[y_*grid_width+x_] = 1;
                x_ = x+i;
                y_ = y+(int)(i*tan+1);
                if(is_validIndex(y_*grid_width+x_))og_tmp.data[y_*grid_width+x_] = 1;
                x_ = x+(int)(i/tan);
                y_ = y+i;
                if(is_validIndex(y_*grid_width+x_))og_tmp.data[y_*grid_width+x_] = 1;
                x_ = x+(int)(i/tan)+1;
                y_ = y+i;
                if(is_validIndex(y_*grid_width+x_))og_tmp.data[y_*grid_width+x_] = 1;
            }
            
        }
    }

    for(int i=0;i<grid_num;i++)og.data[i]=og_tmp.data[i];
}

nav_msgs::Odometry odom;
void odom_cb(const nav_msgs::Odometry& odom_message)
{
    odom = odom_message;
}

ros::Publisher costmap_pub;
void pc_cb(const sensor_msgs::PointCloud2ConstPtr& ros_pc_ground, const sensor_msgs::PointCloud2ConstPtr& ros_pc_obstacle)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ground (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_obstacle (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*ros_pc_ground, *pc_ground);
    pcl::fromROSMsg(*ros_pc_obstacle, *pc_obstacle);

    pc_ground = crop_cloud(pc_ground, Eigen::Vector4f(-15, -15, -1, 1), Eigen::Vector4f(15, 15, 0.5, 1));
    pc_obstacle = crop_cloud(pc_obstacle, Eigen::Vector4f(-15, -15, -1, 1), Eigen::Vector4f(15, 15, 0.5, 1));

    Eigen::Matrix3f mat3 = Eigen::Quaternionf(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z).toRotationMatrix();
    Eigen::Matrix4f mat4 = Eigen::Matrix4f::Identity();
    mat4.block(0,0,3,3) = mat3;
    pcl::transformPointCloud( *pc_ground, *pc_ground, mat4);
    pcl::transformPointCloud( *pc_obstacle, *pc_obstacle, mat4);

    nav_msgs::OccupancyGrid og_ground, og_obstacle;
    og_ground.data.resize(grid_num);
    og_obstacle.data.resize(grid_num);

    input_cloud_to_occupancyGrid(pc_ground, og_ground, 1);
    input_cloud_to_occupancyGrid(pc_obstacle, og_obstacle, 2);

    dilation_roadOccupancyGrid(og_ground, 1);
    makeRoof_to_roadOccupancyGrid(og_ground, 10);

    for(int i=0;i<grid_num;i++)og_total.data[i] = (og_obstacle.data[i]==0&&og_ground.data[i]==1)?1:100;

    og_total.header.stamp = ros_pc_ground->header.stamp;
    og_total.header.frame_id = "map";
    og_total.info.resolution = resolution;
    og_total.info.width = grid_width;
    og_total.info.height = grid_width;
    og_total.info.origin.position.x = -width_2 + odom.pose.pose.position.x;
    og_total.info.origin.position.y = -width_2 + odom.pose.pose.position.y;
    og_total.info.origin.position.z = +odom.pose.pose.position.z;
    og_total.info.origin.orientation.w = 1.0;
    og_total.header.stamp = ros::Time::now();

    costmap_pub.publish(og_total);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cloud2costmap");
    
    ros::NodeHandle nh;
    ros::NodeHandle lSubscriber("");

    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_ground_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_obstacle_sub;
    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>> sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>(10), pc_ground_sub, pc_obstacle_sub);

    pc_ground_sub.subscribe(lSubscriber, "/velodyne_points_ground", 10);
    pc_obstacle_sub.subscribe(lSubscriber, "/velodyne_points_obstacle", 10);
    sync.registerCallback(boost::bind(&pc_cb, _1, _2));

    ros::Subscriber odom_sub = lSubscriber.subscribe("/hdl_localization/odom", 10, odom_cb);

    costmap_pub = nh.advertise<nav_msgs::OccupancyGrid>("/cloud2costmap/cloudCostmap", 1);

    og_total.data.resize(grid_num);

    ros::spin();

    return 0;
}

#endif