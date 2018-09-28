/* points_buffer.cpp
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

const int N = 5;

class Buffer
{
	private:
	
		ros::Rate r;
		ros::Subscriber laser_sub;

		ros::Publisher buffer_pub1;
		ros::Publisher buffer_pub2;
		ros::Publisher buffer_pub3;
		ros::Publisher buffer_pub4;
		ros::Publisher buffer_pub5;

		sensor_msgs::PointCloud2 buffer_points[N];
		sensor_msgs::PointCloud2 new_buffer_points;

		pcl::VoxelGrid<PointI> vg;

		bool flag_callback;
		bool first_flag;

		//param
		size_t point_size;
		// ros::Time time_v;
	public:

		Buffer(ros::NodeHandle n, ros::NodeHandle priv_nh);
		void laserCallback(const sensor_msgs::PointCloud2 input);
		void buffer2point();
		void change_point(sensor_msgs::PointCloud2 vg_buffer_point,ros::Publisher pub);

		bool spin()
		{
			ros::Rate loop_rate(r);

			while(ros::ok()){
				if(flag_callback){
					buffer2point();
					change_point(buffer_points[0],buffer_pub1);             
					change_point(buffer_points[1],buffer_pub2);             
					change_point(buffer_points[2],buffer_pub3);             
					change_point(buffer_points[3],buffer_pub4);             
					change_point(buffer_points[4],buffer_pub5);             
				}

				ros::spinOnce();
				loop_rate.sleep();		
			}

			return true;
		}   
};

Buffer::Buffer(ros::NodeHandle n, ros::NodeHandle priv_nh):
	r(1),
	flag_callback(false),
	first_flag(true)
{
	laser_sub = n.subscribe("velodyne_points", 10, &Buffer::laserCallback, this);
	
	buffer_pub1 = n.advertise<sensor_msgs::PointCloud2>("buffer1", 10);
	buffer_pub2 = n.advertise<sensor_msgs::PointCloud2>("buffer2", 10);
	buffer_pub3 = n.advertise<sensor_msgs::PointCloud2>("buffer3", 10);
	buffer_pub4 = n.advertise<sensor_msgs::PointCloud2>("buffer4", 10);
	buffer_pub5 = n.advertise<sensor_msgs::PointCloud2>("buffer5", 10);
}


void
Buffer::laserCallback(const sensor_msgs::PointCloud2 input){

	new_buffer_points  = input;

	flag_callback = true;
}

void
Buffer::buffer2point(){

	if(!first_flag){
		for(int i=N-1;i>0;i--){
			buffer_points[i] = buffer_points[i-1];
		}
	}

	pcl::fromROSMsg(new_buffer_points,*input_cloud);
	
	voxel_grid(0.3,input_cloud,filtered_vg_cloud);
	
	pcl::toROSMsg(*filtered_vg_cloud,buffer_points[0]);

	if(first_flag){
		for(int i=0;i<N;i++){
			buffer_points[i] = buffer_points[0];
		}

		first_flag = false;
	}

}

void
Buffer::change_point(sensor_msgs::PointCloud2 vg_buffer_point,ros::Publisher pub_){

	pcl::PointCloud<pcl::PointXYZI>::Ptr change_input (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr change_output (new pcl::PointCloud<pcl::PointXYZI>);

	pcl::fromROSMsg(vg_buffer_point,*change_input);
	// time_v = buffer_points[0].header.stamp;

	point_size = change_input->points.size();

	output_cloud->points.clear();

	for(size_t i = 0; i < point_size; i++){
		
		float d = distance(change_input->points[i].x,change_input->points[i].y);
		
		if(d<20){

			pcl::PointXYZI temp_point;
			temp_point.x = change_input->points[i].x; 
			temp_point.y = change_input->points[i].y;
			temp_point.z = change_input->points[i].z;
			temp_point.intensity = change_input->points[i].intensity;

			change_output->points.push_back(temp_point);
		}
	}

	point_pub(pub_,*change_output,"/velodyne",ros::Time::now());

}



int main(int argc, char** argv){
	ros::init(argc, argv, "points_buffer");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");

	cout<<"-------points buffer ok--------"<<endl;

	Buffer buffer(n,priv_nh);
	
	buffer.spin();
	
	return 0;
}

