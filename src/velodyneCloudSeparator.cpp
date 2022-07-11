#ifndef VELODYNE_CLOUD_SEPARATOR_CPP
#define VELODYNE_CLOUD_SEPARATOR_CPP

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/common/common.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>

#include <queue>

#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

ros::Publisher pc_ground_pub;
ros::Publisher pc_obstacle_pub;

struct PointXYZIR{
PCL_ADD_POINT4D;
float intensity;
uint16_t ring;
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
(float,x,x)
(float,y,y)
(float,z,z)
(float,intensity,intensity)
(uint16_t,ring,ring)
)

struct IndexXY{
	int x = 0;
	int y = 0;
};

class GroundRemover{

public:
	GroundRemover()
    {
        pcl::PointCloud<PointXYZIR>::Ptr tempc(new pcl::PointCloud<PointXYZIR>);
        pc      = tempc;
        length_ = int((max_range_ - min_range_)/deltaR_);
        region_minz_.assign(width_ * length_, 100);
        cloud_index_.assign(width_ * height_, -1);
        range_image_ = cv::Mat::zeros(height_, width_,  CV_8UC3);
        region_ = cv::Mat::zeros(height_, width_,  CV_8UC1);
    }
	~GroundRemover(){}

	void RangeProjection(){
    	for(int i=0; i<pc->points.size(); ++i){
		    float u(0), range(0);
		    calAngle(pc->points[i].x, pc->points[i].y, u);
		    calRange(pc->points[i], range);
		    if(range<min_range_ || range>max_range_)
			    continue;

    		int col = round((width_-1)*(u *180.0/M_PI)/360.0);
	    	int ind = pc->points[i].ring;
		    int region = int((range-min_range_)/deltaR_);

    		int region_index = col * length_ + region;
	    	int index = col * height_ + ind;
		    range_image_.at<cv::Vec3b>(ind, col) = cv::Vec3b(0,255,0);
		    region_minz_[region_index] = std::min(region_minz_[region_index], pc->points[i].z);
		    region_.at<uchar>(ind, col) = region;
		    cloud_index_[index] = i;
	    }
	}

	void RECM()
    {
        float pre_th = 0.;
	    for(int i=0; i<region_minz_.size(); ++i){
		    if( i%length_ == 0){
			    pre_th = region_minz_[i];
		    }else{
			    region_minz_[i] = std::min(region_minz_[i], pre_th + deltaR_ * (float)tan(sigma_*M_PI/180));
			    pre_th = region_minz_[i] ;
		    }
	    }

	    for(int i=0; i<width_; ++i){
		    for(int j= 0; j<height_; ++j){
			    int index = i * height_ +j;
			    int region_i = region_.at<uchar>(j, i);
			    float th_height = region_minz_[i*length_ + region_i];
			    int id = cloud_index_[index];
			    if(id == -1)
				    continue;
			    if(pc->points[id].z >= (th_height+th_g_)){
				    range_image_.at<cv::Vec3b>(j, i) = cv::Vec3b(0,0,255);
			    }
		    }
	    }
    }
	void JCP()
    {
        std::vector<cv::Mat> channels;
        cv::split(range_image_, channels);
        cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(5,5));
        cv::dilate(channels[2],channels[2],element);
        cv::merge(channels, range_image_);

        std::queue<IndexXY> qt;
        for(int i=0; i<width_; ++i){
            for(int j= 0; j<height_; ++j){
                if(range_image_.at<cv::Vec3b>(j, i) == cv::Vec3b(0,255,255)){
                    IndexXY id;
                    id.x = i;
                    id.y = j;
                    if(cloud_index_[j * height_ + i] != -1){
                        qt.push(id);
                        range_image_.at<cv::Vec3b>(j, i) = cv::Vec3b(255,0,0);
                    }else{
                        range_image_.at<cv::Vec3b>(j, i) = cv::Vec3b(0,0,255);
                    }
                }
            }
        }

        while(!qt.empty()){
            IndexXY id = qt.front();
            qt.pop();
            int cloud_id = id.x * height_ + id.y;
            Eigen::VectorXf D(24);
            int mask[24];
            float sumD(0);
            for(int i=0; i<24; ++i){
                int nx = neighborx_[i] + id.x;
                int ny = neighbory_[i] + id.y;

                int ncloud_id = nx * height_ + ny;
                float range_diff(0);
                
                if(nx < 0 || nx >= width_ || ny < 0 ||ny >= height_ || cloud_index_[ncloud_id] == -1){
                    D(i) = 0;
                    sumD += D(i) ;
                    mask[i] = -1;		
                    continue;
                }

                calRangeDiff(pc->points[cloud_index_[cloud_id]], pc->points[cloud_index_[ncloud_id]], range_diff);
                if(range_diff > 0){
                    D(i) = 0;
                    sumD += D(i);
                }else{
                    D(i) = (exp(-5 * range_diff));
                    sumD += D(i);
                }
                if(range_image_.at<cv::Vec3b>(ny, nx) == cv::Vec3b(255,0,0)){
                    mask[i] = 2;
                }else if(range_image_.at<cv::Vec3b>(ny, nx) == cv::Vec3b(0,255,0)){
                    mask[i] = 1;
                }else if(range_image_.at<cv::Vec3b>(ny, nx) == cv::Vec3b(0,0,255)){
                    mask[i] = 0;
                }
            }

            Eigen::VectorXf W(24);
            W = D / sumD;

            float score_r(0), score_g(0);
            for(int i=0; i<D.size(); ++i){
                if(mask[i] == 0){
                    score_r += W(i);
                }else if(mask[i] == 1){
                    score_g += W(i);
                }
            }
            
            if(score_r > score_g){
                range_image_.at<cv::Vec3b>(id.y, id.x) = cv::Vec3b(0,0,255);
            }else{
                range_image_.at<cv::Vec3b>(id.y, id.x) = cv::Vec3b(0,255,0);
            }
        }
    }

	void GroundRemove(pcl::PointCloud<PointXYZIR>& pc_in, 
				pcl::PointCloud<PointXYZIR>& pc_ground, 
				pcl::PointCloud<PointXYZIR>& pc_obstacle)
    {
        for(auto p:pc_in.points){
            pc->points.push_back(p);
        }
        RangeProjection();
        RECM();
        JCP();

        for(int i=0; i<width_; ++i){
            for(int j=0; j<height_; ++j){
                int index = cloud_index_[i*height_ + j];
                if(index == -1)
                    continue;
                if(range_image_.at<cv::Vec3b>(j, i) == cv::Vec3b(0,255,0)){	
                    pc_ground.push_back(pc->points[index]);
                }else if(range_image_.at<cv::Vec3b>(j, i) == cv::Vec3b(0,0,255)){	
                    pc_obstacle.push_back(pc->points[index]);
                }
            }
        }
    }
	
    void calAngle(float x, float y, float &temp_tangle)
    {
        if (x == 0 && y == 0) {
		    temp_tangle = 0;
	    } else if (y >= 0) {
		    temp_tangle = (float) atan2(y, x);
	    } else if (y <= 0) {
		    temp_tangle = (float) atan2(y, x) + 2 * M_PI;
	    }
    }

	void calRange(const PointXYZIR& p, float& range)
    {
        range = sqrt(p.x*p.x + p.y*p.y);
    }
	void calRangeDiff(const PointXYZIR& p1, const PointXYZIR& p2, float& range)
    {
        range = sqrt((p1.x- p2.x)*(p1.x- p2.x)+(p1.y- p2.y)*(p1.y- p2.y) +(p1.z- p2.z)*(p1.z- p2.z));
    }

private:

	pcl::PointCloud<PointXYZIR>::Ptr pc;

	int width_  = 2083;
	int height_ = 64;
	float max_range_ = 70.0;
	float min_range_ = 2.0;
	int length_ = 0;

	int neighborx_[24] = {-2, -1,  0,  1,  2,-2, -1,  0,  1,  2,-2,-1, 1, 2, -2,-1,0, 1, 2,-2,-1, 0, 1, 2};
	int neighbory_[24] = {-2, -2, -2, -2, -2,-1, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2};

	std::vector<float> region_minz_;
	
	std::vector<int> cloud_index_;
	cv::Mat range_image_;
	cv::Mat region_;
	

	float th_g_ = 0.3;
	float sigma_ = 7.;
	float deltaR_ = 2.;
	
};

void callback(const sensor_msgs::PointCloud2ConstPtr& ros_pc_input)
{
    pcl::PointCloud<PointXYZIR>::Ptr pc_input (new pcl::PointCloud<PointXYZIR>);
    pcl::PointCloud<PointXYZIR>::Ptr pc_ground(new pcl::PointCloud<PointXYZIR>);
	pcl::PointCloud<PointXYZIR>::Ptr pc_obstacle(new pcl::PointCloud<PointXYZIR>);

    pcl::fromROSMsg(*ros_pc_input, *pc_input);

    GroundRemover groundremover;
	groundremover.GroundRemove(*pc_input, *pc_ground, *pc_obstacle);

    sensor_msgs::PointCloud2 ros_pc_output_ground;
    sensor_msgs::PointCloud2 ros_pc_output_obstacle;
    pcl::toROSMsg(*pc_ground, ros_pc_output_ground);
    pcl::toROSMsg(*pc_obstacle, ros_pc_output_obstacle);
    ros_pc_output_ground.header.frame_id = ros_pc_input->header.frame_id;
    ros_pc_output_obstacle.header.frame_id = ros_pc_input->header.frame_id;

    pc_ground_pub.publish(ros_pc_output_ground);
    pc_obstacle_pub.publish(ros_pc_output_obstacle);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velodyneCloudSeparator");
    ros::NodeHandle nh;
    ros::NodeHandle lSubscriber("");

    ros::Subscriber robotState_sub = lSubscriber.subscribe("/velodyne_points", 10 , callback);
    
    pc_ground_pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_ground", 10);
    pc_obstacle_pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_obstacle", 10);

    ros::spin();
    return 0;

}

#endif