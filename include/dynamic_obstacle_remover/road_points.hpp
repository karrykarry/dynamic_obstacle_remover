#ifndef _SAVE_POINTS_HPP_
#define _SAVE_POINTS_HPP_

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>


#include"tool.hpp"

using namespace std;

const float height_diff_threshold_ = 0.15;

class Road_points
{
	private:
		tf::TransformBroadcaster br;
		tf::TransformListener listener;
		tf::Transform broad_transform;
		tf::StampedTransform buffer_transform;

		sensor_msgs::PointCloud save_point;

		vector<sensor_msgs::PointCloud2> save_point2;		//保存用
		vector<sensor_msgs::PointCloud2> swap_save_point2;	//次の保存用のswap

		vector< vector<float> > prob;	//gridの確率
		vector< vector<float> > grid2count;	//点群が入った数を格納
		vector< vector<bool> > init_point;	//outputの点を抑える用

		int step_num;
		int grid_dim_;
		int grid_dim_ex_;				//prevent_segfault
		float m_per_cell_;
		float road_threshold;

	public:

	Road_points();

	void say();

	void prepare(int step_num,int r_length,float per_celli,float r_threshold);
	bool first_process(int step_num);

	void return_globalxy(double x, double y, double yaw, double& return_x, double& return_y);
	
	void listen_tf(sensor_msgs::PointCloud buffer_point, string Child_id, string Parent_id);
	
	void withprob_method(sensor_msgs::PointCloud2 s_points, pcl::PointCloud<pcl::PointXYZI>::Ptr clear_cloud);//height_map

	void road_or_notroad(int step_num, pcl::PointCloud<pcl::PointXYZI>::Ptr clear_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr road_cloud);//
	
	void save_points2pcl(int step_num, std_msgs::Float32& world2lidar, pcl::PointCloud<pcl::PointXYZI>::Ptr road_cloud);


};

#endif

