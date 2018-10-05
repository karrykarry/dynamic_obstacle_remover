/* probability_points.cpp:動的の点・静的の点・地面を抽出
 *
 * skip_time 分の[hz]で貯めて，step分の点群を貯める
 * 障害物判定にgrid_dim_でグリッドの大きさ・per_cellで解像度を表している．
 *
 * subscribe:obstacle_points
*/ 
#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "tool.hpp"
#include "road_points.hpp"

using namespace std;

pcl::PointCloud<pcl::PointXYZI>::Ptr road_cloud (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr notroad_cloud (new pcl::PointCloud<pcl::PointXYZI>);

class BufferTF
{
	private:
		ros::Rate r;
		ros::Subscriber laser_sub;
		ros::Publisher road_pub;
		ros::Publisher notroad_pub;
	
		sensor_msgs::PointCloud buffer_point;
		sensor_msgs::PointCloud2 dynamic_points;
		sensor_msgs::PointCloud2 static_points;
		ros::Time time_now;
			
		int count;
		bool flag;
		//param
		string Parent_id;
		string Child_id;
		int skip_time;
		int step_num;
		int r_length;
		float per_cell;
		float static_threshold;
	
	public:
		BufferTF(ros::NodeHandle n, ros::NodeHandle priv_nh);
		Road_points road_points;
		void laserCallback(const sensor_msgs::PointCloud2 input);
};

BufferTF::BufferTF(ros::NodeHandle n, ros::NodeHandle priv_nh):
	r(20),
	count(0),
	flag(true)
{
	laser_sub = n.subscribe("road_vis", 10, &BufferTF::laserCallback, this);
	
	road_pub = n.advertise<sensor_msgs::PointCloud2>("road_points_pub", 10);
	notroad_pub = n.advertise<sensor_msgs::PointCloud2>("notroad_points_pub", 10);
	
	priv_nh.getParam("Parent_id", Parent_id);
	priv_nh.getParam("Child_id", Child_id);
	priv_nh.getParam("skip_time", skip_time);
	priv_nh.getParam("step_num", step_num);
	priv_nh.getParam("radius_length", r_length);
	priv_nh.getParam("per_cell_", per_cell);
	priv_nh.getParam("static_threshold", static_threshold);

	road_points.prepare(step_num,r_length,per_cell,static_threshold);
}

void
BufferTF::laserCallback(const sensor_msgs::PointCloud2 input){

	if(count%skip_time==0){

		sensor_msgs::convertPointCloud2ToPointCloud(input, buffer_point);
		ros::Time time_now = buffer_point.header.stamp;
		road_points.listen_tf(buffer_point, Child_id, Parent_id);
	
		if(flag) flag = road_points.first_process(step_num);//はじめの処理
		road_points.save_points2pcl(step_num,road_cloud,notroad_cloud);//main処理

		//pub
		point_pub(road_pub,*road_cloud,"/velodyne",time_now);
		point_pub(notroad_pub,*notroad_cloud,"/velodyne",time_now);

		road_cloud->points.clear();	
		notroad_cloud->points.clear();	
		// road_points.say();

		count=0;
	}
	count++;
}


int main(int argc, char **argv){

	ros::init(argc, argv, "probabilty_points");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");

	cout<<"-------probability points ok--------"<<endl;

	BufferTF buffer_tf(n,priv_nh);

	ros::spin();

	return 0;
}

