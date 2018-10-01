#include "save_points.hpp"
#include "tool.hpp"

Save_points::Save_points()
{

}

void
Save_points::prepare(int step_num,int threshold){
	
	save_point2 = vector<sensor_msgs::PointCloud2> (step_num);
	swap_save_point2 = vector<sensor_msgs::PointCloud2> (step_num);

	distance_threshold = threshold;
}

bool
Save_points::first_process(int step_num){

	for(int i=1;i<step_num;i++){
		save_point2[i]=save_point2[0];
	}
	return false;
}

void
Save_points::listen_tf(sensor_msgs::PointCloud buffer_point, string Child_id, string Parent_id){

	try{
		ros::Time time_now = buffer_point.header.stamp;
		ros::Time past = time_now - ros::Duration(5.0);

		listener.waitForTransform(
				Child_id,time_now, 
				Parent_id,past,
				"/map",ros::Duration(3.0));
		listener.lookupTransform(Child_id, Parent_id,  
				time_now, buffer_transform);
		listener.transformPointCloud(Child_id, time_now, buffer_point, Parent_id, save_point);
		sensor_msgs::convertPointCloudToPointCloud2(save_point, save_point2[0]);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}
}


void
Save_points::save_points2pcl(int step_num,pcl::PointCloud<pcl::PointXYZI>::Ptr save_cloud){

	for(int i=0;i<step_num;i++){

		pcl::fromROSMsg(save_point2[i], *input_cloud);   
		size_t point_size = input_cloud->points.size();

		for(size_t i = 0; i < point_size; i++){

			float d = distance(input_cloud->points[i].x-buffer_transform.getOrigin().x(),
					input_cloud->points[i].y-buffer_transform.getOrigin().y());

			if(d<distance_threshold){

				pcl::PointXYZI temp_point;
				temp_point.x = input_cloud->points[i].x; 
				temp_point.y = input_cloud->points[i].y;
				temp_point.z = input_cloud->points[i].z;
				temp_point.intensity = input_cloud->points[i].intensity;

				save_cloud->points.push_back(temp_point);
			}
		}

		if(!(i==step_num-1)) swap_save_point2[i+1] = save_point2[i];
	}

	save_point2 = swap_save_point2;
}


void
Save_points::points_clear(pcl::PointCloud<pcl::PointXYZI>::Ptr save_cloud){

	save_cloud->points.clear();

}
