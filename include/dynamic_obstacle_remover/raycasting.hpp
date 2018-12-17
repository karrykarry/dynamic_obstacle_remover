#ifndef _RAYCASTING_HPP_
#define _RAYCASTING_HPP_
#include <ros/ros.h>
#include <iostream>
#include <pcl/poiny_type.h>

class Raycasting
{
	private:
		ros::NodeHandle n;
		
		float theta_dim_;
		float r_dim_; 
  		int radius_length;
  		int c_grid_length;//直交座標のgridの数
		
		vector< vector<float> > occupy;
		
	public:
		Raycasting();


};


#endif
