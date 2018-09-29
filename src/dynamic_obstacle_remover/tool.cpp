#include "tool.hpp"


float distance(float x,float y){
    float ans; 

    ans =  sqrt(x*x+y*y);

    return ans;

}



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

