/* voxel_points.cpp
 *
 * 単なるbuffer
 *
*/

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <iostream>
#include <vector>

#include "tool.hpp"

typedef pcl::PointXYZ PointX;
typedef pcl::PointXYZI PointI;
//typedef pcl::PointXYZRGB PointI;
typedef pcl::PointCloud<PointX> CloudX;
typedef pcl::PointCloud<PointI> CloudI;
typedef pcl::PointCloud<PointX>::Ptr CloudXPtr;
typedef pcl::PointCloud<PointI>::Ptr CloudIPtr;

CloudIPtr input_cloud (new pcl::PointCloud<PointI>);
CloudIPtr filtered_vg_cloud (new pcl::PointCloud<PointI>);
CloudIPtr output_cloud (new pcl::PointCloud<PointI>);

using namespace std;

class Buffer
{
	private:
	
		ros::Rate r;
		ros::Subscriber laser_sub;

		ros::Publisher velodyne_pub;

		sensor_msgs::PointCloud2 buffer_points;

		pcl::VoxelGrid<PointI> vg;

		bool flag_callback;
		bool first_flag;
		size_t point_size;

		ros::Time time_v;

		//param
		float voxel_size;	
	public:

		Buffer(ros::NodeHandle n, ros::NodeHandle priv_nh);
		void laserCallback(const sensor_msgs::PointCloud2 input);

};

Buffer::Buffer(ros::NodeHandle n, ros::NodeHandle priv_nh):
	r(20)
{
	laser_sub = n.subscribe("velodyne_points", 10, &Buffer::laserCallback, this);
	
	// velodyne_pub = n.advertise<sensor_msgs::PointCloud2>("voxel_point", 10);
	velodyne_pub = n.advertise<sensor_msgs::PointCloud2>("buffer5", 10);

	priv_nh.getParam("voxel_size", voxel_size);
}


void
Buffer::laserCallback(const sensor_msgs::PointCloud2 input){

	pcl::fromROSMsg(input,*input_cloud);
	voxel_grid(voxel_size,input_cloud,filtered_vg_cloud);

	size_t point_size = filtered_vg_cloud->points.size();

	output_cloud->points.clear();

	for(size_t i = 0; i < point_size; i++){
		
		float d = distance(filtered_vg_cloud->points[i].x,filtered_vg_cloud->points[i].y);
		
		if(d<18){

			pcl::PointXYZI temp_point;
			temp_point.x = filtered_vg_cloud->points[i].x; 
			temp_point.y = filtered_vg_cloud->points[i].y;
			temp_point.z = filtered_vg_cloud->points[i].z;
			temp_point.intensity = filtered_vg_cloud->points[i].intensity;

			output_cloud->points.push_back(temp_point);
		}
	}
	
	pcl::toROSMsg(*filtered_vg_cloud,buffer_points);

	point_pub(velodyne_pub,*output_cloud,"velodyne",input.header.stamp);
	
}



int main(int argc, char** argv){
	ros::init(argc, argv, "buffer_points");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");

	cout<<"-------buffer points ok--------"<<endl;

	Buffer buffer(n,priv_nh);
	
	ros::spin();
	
	return 0;
}


