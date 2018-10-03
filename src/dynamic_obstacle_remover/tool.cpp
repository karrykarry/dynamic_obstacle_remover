#include "tool.hpp"


float distance(float x,float y){
    float ans; 

    ans =  sqrt(x*x+y*y);

    return ans;

}


// void return_globalxy(double x, double y, double yaw, double& return_x, double& return_y){
// 	
// 	Eigen::Rotation2Dd rot(yaw*(-1));
// 	Eigen::Vector2d input_xy;
// 	Eigen::Vector2d ans;
//
// 	input_xy << x, y;
//
// 	ans = rot*input_xy;
//
// 	return_x = ans.x();
// 	return_y = ans.y();
// }



void voxel_grid(float size,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,pcl::PointCloud<pcl::PointXYZI>::Ptr &filtered_cloud){

    pcl::VoxelGrid<pcl::PointXYZI> vg;
    
	vg.setLeafSize (size, size, size);
    vg.setInputCloud (cloud);
    vg.filter (*filtered_cloud);

}



void point_pub(ros::Publisher pub,const pcl::PointCloud<pcl::PointXYZI>& cloud, std::string frame,ros::Time time)
{
	sensor_msgs::PointCloud2 pub_points;

    pcl::toROSMsg(cloud,pub_points);

    pub_points.header.stamp = time;
    pub_points.header.frame_id = frame;

    pub.publish(pub_points);
}

