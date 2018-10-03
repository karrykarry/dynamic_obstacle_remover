#ifndef _TOOL_HPP_
#define _TOOL_HPP_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
// #include <Eigen/Core>
// #include <Eigen/Geometry>
// #include <Eigen/LU>
// #include <Eigen/Eigenvalues>
#include <cmath>

#include<iostream>

float distance(float x,float y);

// void return_globalxy(double x, double y, double yaw, double& return_x, double& return_y);

void voxel_grid(float size,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,pcl::PointCloud<pcl::PointXYZI>::Ptr &filtered_cloud);

void point_pub(ros::Publisher pub,const pcl::PointCloud<pcl::PointXYZI>& cloud, std::string frame,ros::Time time);


#endif

