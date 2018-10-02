#include "save_points.hpp"
#include "tool.hpp"

#define MIN(x,y) ((x) < (y) ? (x) : (y)) 
#define MAX(x,y) ((x) > (y) ? (x) : (y)) 

Save_points::Save_points()
{

}

//paramの読み込み
void
Save_points::prepare(int step_n,int grid_dim,float per_cell,float s_threshold){

	step_num = step_n;
	grid_dim_ = grid_dim;
	m_per_cell_ = per_cell;
	static_threshold = s_threshold;

	save_point2 = vector<sensor_msgs::PointCloud2> (step_n);
	swap_save_point2 = vector<sensor_msgs::PointCloud2> (step_n);
	prob = vector< vector<float> >(grid_dim_, vector<float>(grid_dim_,0));
}

//はじめの点群を入れ込む
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
Save_points::minmax_method(
		sensor_msgs::PointCloud2 s_points, 
		pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud, 
		pcl::PointCloud<pcl::PointXYZI>::Ptr clear_cloud)
{

	float min[grid_dim_][grid_dim_];
	float max[grid_dim_][grid_dim_];
	bool init[grid_dim_][grid_dim_];
	bool prob_flag[grid_dim_][grid_dim_]; //1回のminmax-methodで1回だけカウントするため
	//初期化
	fill(min[0],min[grid_dim_],0);
	fill(max[0],max[grid_dim_],0);
	fill(init[0],init[grid_dim_],0);
	fill(prob_flag[0],prob_flag[grid_dim_],0);

	pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZI>);

	pcl::fromROSMsg(s_points, *input_cloud);   
	size_t point_size = input_cloud->points.size();
	// build height map
	for (size_t i = 0; i < point_size; ++i) {
		int x = ((grid_dim_/2)+
				(input_cloud->points[i].x-buffer_transform.getOrigin().x())/m_per_cell_);
		
		int y = ((grid_dim_/2)+
				(input_cloud->points[i].y-buffer_transform.getOrigin().y())/m_per_cell_);

		if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_) {
			if (!init[x][y]) {
				min[x][y] = input_cloud->points[i].z;
				max[x][y] = input_cloud->points[i].z;

				init[x][y] = true;
			} else {
				min[x][y] = MIN(min[x][y], input_cloud->points[i].z);
				max[x][y] = MAX(max[x][y], input_cloud->points[i].z);
			}   
		}   
	}

	// display points where map has height-difference > threshold
	for (size_t i = 0; i < point_size; ++i) {
		int x = ((grid_dim_/2)+
				(input_cloud->points[i].x-buffer_transform.getOrigin().x())/m_per_cell_);
		
		int y = ((grid_dim_/2)+
				(input_cloud->points[i].y-buffer_transform.getOrigin().y())/m_per_cell_);

		if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_ && init[x][y]) {//gridの中に

			if ((max[x][y] - min[x][y] > height_diff_threshold_) && (max[x][y] - min[x][y] < 10.0) ) {  
				if(!prob_flag[x][y]) prob[x][y]++;

				prob_flag[x][y] = true;
				
				pcl::PointXYZI temp_point;
				temp_point.x = input_cloud->points[i].x; 
				temp_point.y = input_cloud->points[i].y;
				temp_point.z = input_cloud->points[i].z;
				temp_point.intensity = input_cloud->points[i].intensity;

				obstacle_cloud->points.push_back(temp_point);
			}

			else {

				if(min[x][y]+0.1>input_cloud->points[i].z) {//////////12/09変更
					
					pcl::PointXYZI temp_point;
					temp_point.x = input_cloud->points[i].x-buffer_transform.getOrigin().x(); 
					temp_point.y = input_cloud->points[i].y-buffer_transform.getOrigin().y();
					temp_point.z = input_cloud->points[i].z-buffer_transform.getOrigin().z();
					temp_point.intensity = input_cloud->points[i].intensity;

					clear_cloud->points.push_back(temp_point);

				}
			}
		}
	}

	input_cloud->points.clear();
}

void
Save_points::dynamic_or_static(int step_num,
		pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud,
		pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_cloud,
		pcl::PointCloud<pcl::PointXYZI>::Ptr static_cloud)
{

	size_t point_size = obstacle_cloud->points.size();
	
	for (size_t i = 0; i < point_size; ++i) {
		int x = ((grid_dim_/2)+
				(obstacle_cloud->points[i].x-buffer_transform.getOrigin().x())/m_per_cell_);
		
		int y = ((grid_dim_/2)+
				(obstacle_cloud->points[i].y-buffer_transform.getOrigin().y())/m_per_cell_);

		if (prob[x][y]>(step_num*static_threshold)) {//gridの中に
				
				
				pcl::PointXYZI temp_point;
				temp_point.x = obstacle_cloud->points[i].x-buffer_transform.getOrigin().x(); 
				temp_point.y = obstacle_cloud->points[i].y-buffer_transform.getOrigin().y();
				temp_point.z = obstacle_cloud->points[i].z-buffer_transform.getOrigin().z();
				temp_point.intensity = obstacle_cloud->points[i].intensity;

				static_cloud->points.push_back(temp_point);
			}
	
		else{
				pcl::PointXYZI temp_point;
				temp_point.x = obstacle_cloud->points[i].x-buffer_transform.getOrigin().x(); 
				temp_point.y = obstacle_cloud->points[i].y-buffer_transform.getOrigin().y();
				temp_point.z = obstacle_cloud->points[i].z-buffer_transform.getOrigin().z();
				temp_point.intensity = obstacle_cloud->points[i].intensity;

				dynamic_cloud->points.push_back(temp_point);
		}	
	}
	
}


//貯めてる点群をpclに変換
void
Save_points::save_points2pcl(int step_num,
		pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_cloud,
		pcl::PointCloud<pcl::PointXYZI>::Ptr static_cloud,
		pcl::PointCloud<pcl::PointXYZI>::Ptr clear_cloud)
{

	pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud (new pcl::PointCloud<pcl::PointXYZI>);

	for(int i=0;i<step_num;i++){

		minmax_method(save_point2[i], obstacle_cloud, clear_cloud);

		if(!(i==step_num-1)) swap_save_point2[i+1] = save_point2[i];
	}
	
	dynamic_or_static(step_num,obstacle_cloud,dynamic_cloud,static_cloud);
	obstacle_cloud->points.clear();

	save_point2 = swap_save_point2;

	for(int i=0;i<grid_dim_;i++){	//確率の初期化
		for(int j=0;j<grid_dim_;j++){
			prob[i][j] = 0;
		}
	}

}


//clear
void
Save_points::points_clear(pcl::PointCloud<pcl::PointXYZI>::Ptr save_cloud){

	save_cloud->points.clear();

}
