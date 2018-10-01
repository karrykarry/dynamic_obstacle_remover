#ifndef _SAVE_POINTS_HPP_
#define _SAVE_POINTS_HPP_

#include <ros/ros.h>
#include<iostream>
#include<vector>
#include<string.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include"tool.hpp"

using namespace std;


pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZI>);

class Save_points
{
	private:
		tf::TransformBroadcaster br;
		tf::TransformListener listener;
		tf::Transform broad_transform;
		tf::StampedTransform buffer_transform;

		sensor_msgs::PointCloud save_point;

		vector<sensor_msgs::PointCloud2> save_point2;
		vector<sensor_msgs::PointCloud2> swap_save_point2;	//保存用
	
		int distance_threshold;	
		
	public:

	Save_points();
	void prepare(int step_num,int threshold);
	bool first_process(int step_num);
	void listen_tf(sensor_msgs::PointCloud buffer_point, string Child_id, string Parent_id);

	void save_points2pcl(int step_num,pcl::PointCloud<pcl::PointXYZI>::Ptr save_cloud);

	void points_clear(pcl::PointCloud<pcl::PointXYZI>::Ptr save_cloud);

};

#endif
