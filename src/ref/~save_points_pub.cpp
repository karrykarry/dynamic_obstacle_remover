
/* 
*/ 

#include <ros/ros.h>
#include <iostream>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "tool.hpp"


using namespace std;

typedef pcl::PointXYZ PointX;
typedef pcl::PointXYZI PointI;
//typedef pcl::PointXYZRGB PointI;
typedef pcl::PointCloud<PointX> CloudX;
typedef pcl::PointCloud<PointI> CloudI;
typedef pcl::PointCloud<PointX>::Ptr CloudXPtr;
typedef pcl::PointCloud<PointI>::Ptr CloudIPtr;

CloudIPtr input_cloud (new pcl::PointCloud<PointI>);
CloudIPtr save_cloud (new pcl::PointCloud<PointI>);

class BufferTF
{
	private:
		ros::Rate r;
		ros::Subscriber laser_sub;
		ros::Publisher save_pub;
	
		tf::TransformBroadcaster br;
		tf::TransformListener listener;
		tf::Transform broad_transform;
		tf::StampedTransform buffer_transform;
	
		sensor_msgs::PointCloud buffer_point;
		vector<sensor_msgs::PointCloud> save_point;
		vector<sensor_msgs::PointCloud2> save_point2;
		sensor_msgs::PointCloud pub_point;
		ros::Time time_now;
		
		int count;
		int count_s;
		//param
		int skip;
		int STEP;
		string Parent_id;
		string Child_id;
	
	public:
		BufferTF(ros::NodeHandle n, ros::NodeHandle priv_nh);
		void laserCallback(const sensor_msgs::PointCloud2 input);
		void listen_tf();
		void save_points2pcl();
		void buffer2velo();


};

BufferTF::BufferTF(ros::NodeHandle n, ros::NodeHandle priv_nh):
	r(20)	
{
	laser_sub = n.subscribe("buffer5", 10, &BufferTF::laserCallback, this);
	
	save_pub = n.advertise<sensor_msgs::PointCloud2>("save_point", 10);
	
	priv_nh.getParam("Parent_id", Parent_id);
	priv_nh.getParam("Child_id", Child_id);
	priv_nh.getParam("skip_num", skip);
	priv_nh.getParam("step_num", STEP);
	
	save_point = vector<sensor_msgs::PointCloud> (STEP);
	save_point2 = vector<sensor_msgs::PointCloud2> (STEP);

	count = count_s = 0;
}

void
BufferTF::laserCallback(const sensor_msgs::PointCloud2 input){

	if(count%skip==0){
		count_s++;
		sensor_msgs::convertPointCloud2ToPointCloud(input, buffer_point);
		listen_tf();
		save_points2pcl();
		if(count_s%3==0){
			pcl::toROSMsg(*save_cloud,save_point2[0]);
			save_point2[0].header.frame_id = "/odom"; //odomとかに変更
			save_point2[0].header.stamp = time_now;
			save_pub.publish(save_point2[0]);
            
			save_cloud->points.clear();
			count=count_s=0;





			// pcl::toROSMsg(*save_cloud,velo_point2);
			// sensor_msgs::convertPointCloud2ToPointCloud(velo_point2, buffer_point);
			// buffer2velo();
            //
			// save_cloud->points.clear();
			// count=count_s=0;
		}

	}

	count++;

}

void
BufferTF::listen_tf(){

	try{
		ros::Time time_now = buffer_point.header.stamp;
		ros::Time past = time_now - ros::Duration(5.0);

		listener.waitForTransform(
				Child_id,time_now, 
				Parent_id,past,
				"/map",ros::Duration(3.0));

		listener.transformPointCloud(Child_id, time_now, buffer_point, Parent_id, save_point[0]);
		sensor_msgs::convertPointCloudToPointCloud2(save_point[0], save_point2[0]);
		pcl::fromROSMsg(save_point2[0], *input_cloud);   
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}
}


void
BufferTF::save_points2pcl(){
	
	size_t point_size = input_cloud->points.size();

	for(size_t i = 0; i < point_size; i++){
		
			pcl::PointXYZI temp_point;
			temp_point.x = input_cloud->points[i].x; 
			temp_point.y = input_cloud->points[i].y;
			temp_point.z = input_cloud->points[i].z;
			temp_point.intensity = input_cloud->points[i].intensity;

			save_cloud->points.push_back(temp_point);
	}

}


void
BufferTF::buffer2velo(){

	try{
		// ros::Time time_now = buffer_point.header.stamp;
		// ros::Time past = time_now - ros::Duration(5.0);
		// listener.waitForTransform(
		// 		Parent_id,time_now, 
		// 		Child_id,past,
		// 		"/map",ros::Duration(3.0));

		// listener.waitForTransform(
		// 		Child_id,time_now, 
		// 		Parent_id,past,
		// 		"/map",ros::Duration(3.0));

		//listener.transformPointCloud(Child_id, time_now, buffer_point, Parent_id, velo_point);
		// listener.transformPointCloud(Parent_id, time_now, buffer_point, Child_id, velo_point);
		// sensor_msgs::convertPointCloudToPointCloud2(velo_point, velo_point2);
		// velo_point2.header.frame_id = "/velodyne";
		// velo_point2.header.stamp = time_now;
		// save_pub.publish(velo_point2);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}
}


int main(int argc, char **argv){

	ros::init(argc, argv, "buffer_tf");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");

	cout<<"-------buffer tf ok--------"<<endl;

	BufferTF buffer_tf(n,priv_nh);

	ros::spin();

	return 0;
}



