#ifndef _SAVE_POINTS4LOCAL_HPP_
#define _SAVE_POINTS4LOCAL_HPP_

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string.h>
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

struct data{
	float prob;
	int count;
	data(){
	prob=0;
	count=0;
	}
};


struct angle_data{
	int x;
	int y;
	angle_data(){
	x=0;
	y=0;
	}
};


class Save_points
{
	private:
		tf::TransformBroadcaster br;
		tf::TransformListener listener;
		tf::Transform broad_transform;
		tf::StampedTransform buffer_transform;

		sensor_msgs::PointCloud save_point;

		vector<sensor_msgs::PointCloud2> save_point2;		//保存用
		vector<sensor_msgs::PointCloud2> swap_save_point2;	//次の保存用のswap

		// vector< vector<float> > prob;	//gridの確率
		vector< vector<data> > grid;	//gridの確率
		vector< vector<angle_data> > angle;
		// vector< vector<float> > prob;	//gridの確率
		// vector< vector<int> > count;	//gridの確率

		//後param
		int M_degree;
		int theta_dim_;

		//param
		int step_num;
		int grid_dim_;			//gridの
		int grid_dim_ex_;		//prevent_segfault
		float m_per_cell_;		//cellの大きさ
		float static_threshold;	//静的である確率

		int count_rs;

	public:

	Save_points();

	void say();

	void prepare(int step_num,int r_length,float per_celli,float s_threshold);
	
	void precasting();
	
	bool first_process(int step_num);

	void return_globalxy(double x, double y, double yaw, double& return_x, double& return_y);
	
	void listen_tf(sensor_msgs::PointCloud buffer_point, string Child_id, string Parent_id);

	void withprob_method(sensor_msgs::PointCloud2 s_points, pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud);//height_map

	void dynamic_or_static(int step_num, pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr static_cloud);//
	
	void save_points2pcl(int step_num, pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr static_cloud);


};

#endif
