/* 
 *
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
#include "save_points.hpp"

using namespace std;

pcl::PointCloud<pcl::PointXYZI>::Ptr save_cloud (new pcl::PointCloud<pcl::PointXYZI>);

class BufferTF
{
	private:
		ros::Rate r;
		ros::Subscriber laser_sub;
		ros::Publisher save_pub;
	
		sensor_msgs::PointCloud buffer_point;
		sensor_msgs::PointCloud2 pub_points;
		ros::Time time_now;
			
		int count;
		bool flag;
		//param
		int skip_time;
		int step_num;
		int threshold;
		string Parent_id;
		string Child_id;
	
	public:
		BufferTF(ros::NodeHandle n, ros::NodeHandle priv_nh);
		Save_points save_points;
		void laserCallback(const sensor_msgs::PointCloud2 input);
};

BufferTF::BufferTF(ros::NodeHandle n, ros::NodeHandle priv_nh):
	r(20),
	count(0),
	flag(true)
{
	laser_sub = n.subscribe("voxel_points", 10, &BufferTF::laserCallback, this);
	
	save_pub = n.advertise<sensor_msgs::PointCloud2>("save_points_pub", 10);
	
	priv_nh.getParam("Parent_id", Parent_id);
	priv_nh.getParam("Child_id", Child_id);
	priv_nh.getParam("skip_time", skip_time);
	priv_nh.getParam("step_num", step_num);
	priv_nh.getParam("distance_threshold", threshold);

	save_points.prepare(step_num,threshold);
}

void
BufferTF::laserCallback(const sensor_msgs::PointCloud2 input){

	if(count%skip_time==0){

		sensor_msgs::convertPointCloud2ToPointCloud(input, buffer_point);
		ros::Time time_now = buffer_point.header.stamp;
		save_points.listen_tf(buffer_point, Child_id, Parent_id);//tfつかっていれている
	
		if(flag) flag = save_points.first_process(step_num);
		save_points.save_points2pcl(step_num,save_cloud);//main処理

		//pub
		pcl::toROSMsg(*save_cloud,pub_points);
		pub_points.header.frame_id = "/odom"; //odomとかに変更
		pub_points.header.stamp = time_now;
		save_pub.publish(pub_points);

		save_points.points_clear(save_cloud) ;

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
