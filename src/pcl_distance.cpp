/**
 * @brief
 * @author Toshihiko Shimizu
 * @updated 2019/09/18 10:50:16 @
 * @ref https://ppdr.softether.net/ros-pointcloud-zatxy
 */
#include <numeric> 		//  to calculate average
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

// publisher
// typedef pcl::PointCloud<pcl::PointXYZ> FilteredPointCloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr gfiltered;
double g_xmin, g_xmax;
double g_ymin, g_ymax;

// ROS param configuration, here is default value
double g_radius = 0.1; //半径r
int g_max_nn = 1; //何点見つかったら探索を打ち切るか。0にすると打ち切らない
int g_loop_rate = 10;

// ROS
std_msgs::Float32 g_vel;//navigation stack の速度指令

// pass through filter
pcl::PointCloud<pcl::PointXYZ>::Ptr pass_through(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string iaxis, double imin, double imax)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName (iaxis);
    // pass.setFilterFieldName ("x");    
    pass.setFilterLimits (imin, imax);
    pass.filter(*cloud_filtered);
    // printf("%f\n",cloud->points.size());
    return cloud_filtered;
}

// callback
void callback_knn(sensor_msgs::PointCloud2 pc2){
	g_vel.data = g_xmax;
    // 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nan (new pcl::PointCloud<pcl::PointXYZ>); // NaN値あり
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); // NaN値なし

    // 
    pcl::fromROSMsg(pc2, *cloud_nan);    

    // NaN値が入ってるといろいろ面倒なので除去
    std::vector<int> nan_index;
    pcl::removeNaNFromPointCloud(*cloud_nan, *cloud, nan_index);

    // Filtered by each axis"
    cloud = pass_through(cloud, "x", g_xmin, g_xmax);
    cloud = pass_through(cloud, "y", g_ymin, g_ymax);    
    gfiltered = cloud;		
    
    //近傍点探索 and get average distance
    if(cloud->size()>0){
	//x座標とy座標だけをコピーしたPointCloudを作る
	pcl::PointCloud<pcl::PointXY>::Ptr cloud2d (new pcl::PointCloud<pcl::PointXY>); // NaN値なし
	cloud2d->points.resize(cloud->size());
	for(int i=0; i<cloud->points.size(); i++){
	    cloud2d->points[i].x = cloud->points[i].x;
	    cloud2d->points[i].y = cloud->points[i].y;
	}
    
	//treeも2Dで作る
	pcl::KdTreeFLANN<pcl::PointXY>::Ptr tree2d (new pcl::KdTreeFLANN<pcl::PointXY>);
	tree2d->setInputCloud(cloud2d);

	//近傍点探索に使うパラメータと結果が入る変数
	double radius	= g_radius; //半径r
	unsigned int max_nn = g_max_nn; //何点見つかったら探索を打ち切るか。0にすると打ち切らない
	std::vector<int> k_indices; //範囲内の点のインデックスが入る
	std::vector<float> k_distances; //範囲内の点の距離が入る
  
	//中心座標
	pcl::PointXY p;
	p.x = 0.0;
	p.y = 0.0;

	//2Dで近傍点探索
	tree2d->radiusSearch(p, radius, k_indices, k_distances, max_nn);

	// error handling
	if(k_indices.size() == 0) {
		g_vel.data = g_xmax;
	
	return;
	}

	// get center of each nearest neighbors
	if(max_nn > 1){
	    //**CAUTION, distance is radiul distance, 
	    // float tsum = std::accumulate(k_distances.begin(),k_distances.end(), 0.0); // get sum
	    // float tave = tsum / k_distances.size(); // get average
	    // std::cout << "平均：" << tave << std::endl;

	    //**Accepted! Get average x axis distance, which is the front distances of each points.
	    float tsum = 0.0;
	    for(int i=0; i<k_indices.size();i++) tsum += cloud->points[k_indices[i]].x;
	    float tave = tsum / k_indices.size(); // get average

	    //
	    g_vel.data = tave;

	    ROS_INFO("Ave. distance of %i NN in rad. %lf \t :%lf", max_nn, radius, tave);	
	}
	else if(max_nn == 1){
	    //求めたインデックスでもとのPointCloudの点を見る
	    ROS_INFO("A nearest point of (0.5, 0.5) is...\nx: %lf, y:%lf, z:%lf", cloud->points[k_indices[0]].x, cloud->points[k_indices[0]].y, cloud->points[k_indices[0]].z);
		 g_vel.data = g_xmax;
	}else{
	    ROS_INFO("No neighbor exist.");
	    g_vel.data = g_xmax;
	}
    }
}

int main(int argc, char** argv){
    ros::init(argc,argv,"pcl_handler");
    ros::NodeHandle nh("~"); 	//  load rosparam

    // publisher
    ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/pcl_handler/filtered", 1);
    // Average distance of front points
    ros::Publisher cmd_pub = nh.advertise<std_msgs::Float32>("/pcl_handler/front_dist", 10);
    
    //** set parameter from ros command 
    nh.getParam("x_max", g_xmax);
    nh.getParam("x_min", g_xmin);            
    nh.getParam("y_max", g_ymax);
    nh.getParam("y_min", g_ymin);
    nh.getParam("max_nn", g_max_nn);
    nh.getParam("radius", g_radius);
    nh.getParam("loop_rate", g_loop_rate);    
   ROS_INFO("[%s], [%f < x < %f], [%f < y < %f], [max_nn:%i], [radius:%f], [loop_rate:%i]", ros::this_node::getName().c_str(), g_xmin, g_xmax, g_ymin, g_ymax, g_max_nn, g_radius, g_loop_rate);    

    // 
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/laser2pc/pc2", 1, callback_knn);
    // ros::spin(); // If you just need subscriber, use spin instead.

    // Instead of ros::spin, use spinOnce to publish.
    ros::Rate loop_rate(g_loop_rate);
    while (nh.ok()){
	
	//pub.publish(gfiltered);
	cmd_pub.publish(g_vel);
	ros::spinOnce ();
	loop_rate.sleep();
    }
	
    return 0;
}
