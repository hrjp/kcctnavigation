/**
* @file pcl_diff.cpp
* @brief
* @author Akiro Harada
* @date
* @details PCL差分
*/

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>

// PCL
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

#include <nav_msgs/Odometry.h>

#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/filters/voxel_grid.h>

#include <iostream>
#include <string>

ros::Publisher pc_diff_pub;

pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leaf_size){
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
    
    voxel_grid_filter.setInputCloud(cloud);
    voxel_grid_filter.setLeafSize (leaf_size, leaf_size, leaf_size);
    voxel_grid_filter.filter(*cloud_filtered); 

    return cloud_filtered;
} // downsample_cloud()

pcl::PointCloud<pcl::PointXYZ>::Ptr _pc_map (new pcl::PointCloud<pcl::PointXYZ>);
void pc_map_cb(const sensor_msgs::PointCloud2ConstPtr& ros_pc_map)
{
    pcl::fromROSMsg(*ros_pc_map, *_pc_map);
}

void pc_road_cb(const sensor_msgs::PointCloud2ConstPtr& ros_pc_road)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_road (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*ros_pc_road, *pc_road);

    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (1.f);
    octree.setInputCloud (pc_road);
    octree.addPointsFromInputCloud ();
    octree.switchBuffers();
    octree.setInputCloud (_pc_map);
    octree.addPointsFromInputCloud ();

    std::vector<int> newPointIdxVector;

    octree.getPointIndicesFromNewVoxels (newPointIdxVector);
   
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_diff (new pcl::PointCloud<pcl::PointXYZ> );

    pc_diff->width = _pc_map->points.size() + pc_road->points.size();
    pc_diff->height = 1;
    pc_diff->points.resize (pc_diff->width * pc_diff->height);   

    int n = 0;
    for(size_t i = 0; i < newPointIdxVector.size (); i++)
    {
        pc_diff->points[i].x = _pc_map->points[newPointIdxVector[i]].x;
        pc_diff->points[i].y = _pc_map->points[newPointIdxVector[i]].y;
        pc_diff->points[i].z = _pc_map->points[newPointIdxVector[i]].z;
        n++;
    }

    pc_diff->width = n;
    pc_diff->height = 1;
    pc_diff->points.resize (pc_diff->width * pc_diff->height);

    // Output
    sensor_msgs::PointCloud2 ros_pc_output;
    pcl::toROSMsg(*pc_diff, ros_pc_output);
    ros_pc_output.header.frame_id = "base_link";
    pc_diff_pub.publish(ros_pc_output);
    return;

}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "pcl_diff");
    
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber pc_map_sub = nh.subscribe<sensor_msgs::PointCloud2>("/map2pc/pc_map", 1, pc_map_cb);
    ros::Subscriber pc_road_sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_ground", 1, pc_road_cb);
    
    // Publishers
    pc_diff_pub = nh.advertise<sensor_msgs::PointCloud2>("/pcl_diff/", 10);

    ros::spin();

    return 0;
} // main