# kcctnavigation
## emergency_stoper
前方に障害物がある時に停止する
### publisher
* final_cmd_vel [geometry_msgs::Twist]
* linear_vel [std_msgs::Float32]
> for rviz visualization
* angular_vel [std_msgs::Float32]
> for rviz visualization
### subscriber
* /pcl_handler/front_dist [std_msgs::Float32]
* selected_cmd_vel [geometry_msgs::Twist]
### parameter
* max_vel [double] (default : 1.0)
* loop_rate [double] (default : 10.0)
* max_dis [double] (default : 5.0)
* min_dis [double] (default : 0.5)


## path_tracking
pathを受けっとってそれをなぞって走行する
### publisher
* /initialpose [geometry_msgs::PoseWithCovarianceStamped]
* mcl_cmd_vel [geometry_msgs::Twist]
* waypoint/now [std_msgs::Int32]
### subscriber
* waypoint/set [std_msgs::Int32]
* waypoint/path [nav_msgs::Path]
### parameter
* map_frame_id [string] (default : map)
* base_link_frame_id [string] (default : base_link)
* lookahead_distance [double] (default : 1.5)
* loop_rate [double] (default : 100)
* vel_max [double] (default : 2.0/3.6)
* angular_vel_max [double] (default : 0.5)


## pcl_distance
周囲の点群の中心位置を求める（障害物の距離を求める）
### publisher
* /pcl_handler/filtered [pcl::PointCloud<pcl::PointXYZ]
* /pcl_handler/front_dist [std_msgs::Float32]
### subscriber
* /laser2pc/pc2 [sensor_msgs::PointCloud2]
### parameter
* x_max [double]
* x_min [double]
* y_max [double]
* y_min [double]
* max_nn [int] (default : 1)
> sample cloud number
* radius [double] (default : 0.1)
> robot stop radius
* loop_rate [int] (default : 10)


### person_tracking
人間を追跡する
### publisher
* camera_cmd_vel [geometry_msgs::Twist]
### subscriber
* result [std_msgs::Float32MultiArray]
* waypoint/now [std_msgs::Int32]
* waypoint/type [std_msgs::Int32MultiArray]
* /pcl_handler/front_dist [std_msgs::Float32]
### parameter
* loop_rate [double] (default : 100.0)