#include "save_points_minmax.hpp"
#include "tool.hpp"

#define MIN(x,y) ((x) < (y) ? (x) : (y)) 
#define MAX(x,y) ((x) > (y) ? (x) : (y)) 

Save_points_minmax::Save_points_minmax()
{

}

//paramの読み込み
void
Save_points_minmax::prepare(int step_n,int grid_dim,float per_cell,float s_threshold){

	step_num = step_n;
	grid_dim_ = grid_dim;
	m_per_cell_ = per_cell;
	static_threshold = s_threshold;

	save_point2 = vector<sensor_msgs::PointCloud2> (step_n);
	swap_save_point2 = vector<sensor_msgs::PointCloud2> (step_n);
	prob = vector< vector<float> >(grid_dim_+1, vector<float>(grid_dim_+1,0));
}

//はじめの点群を入れ込む
bool
Save_points_minmax::first_process(int step_num){

	for(int i=1;i<step_num;i++){
		save_point2[i]=save_point2[0];
	}
	return false;
}

void
Save_points_minmax::listen_tf(sensor_msgs::PointCloud buffer_point, string Child_id, string Parent_id){

	try{
		ros::Time time_now = buffer_point.header.stamp;	
		listener.waitForTransform(Child_id, Parent_id, time_now, ros::Duration(0.05));
		
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
Save_points_minmax::return_globalxy(double x, double y, double yaw, double& return_x, double& return_y){
	
	Eigen::Rotation2Dd rot(yaw*(-1));
	Eigen::Vector2d input_xy;
	Eigen::Vector2d ans;

	input_xy << x, y;

	ans = rot*input_xy;

	return_x = ans.x();
	return_y = ans.y();
}



void
Save_points_minmax::minmax_method(
		sensor_msgs::PointCloud2 s_points, 
		pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud, 
		pcl::PointCloud<pcl::PointXYZI>::Ptr clear_cloud)
{

	float min[grid_dim_+1][grid_dim_+1];
	float max[grid_dim_+1][grid_dim_+1];
	bool init[grid_dim_+1][grid_dim_+1];
	bool prob_flag[grid_dim_+1][grid_dim_+1]; //1回のminmax-methodで1回だけカウントするため
	//初期化
	fill(min[0],min[grid_dim_+1],0);
	fill(max[0],max[grid_dim_+1],0);
	fill(init[0],init[grid_dim_+1],0);
	fill(prob_flag[0],prob_flag[grid_dim_+1],0);
	
	double t_x=0,t_y=0,t_yaw=0,r_x=0,r_y=0;

	pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZI>);

	pcl::fromROSMsg(s_points, *input_cloud);   
	size_t point_size = input_cloud->points.size();
	// build height map
	for (size_t i = 0; i < point_size; ++i) {
		t_x = input_cloud->points[i].x-buffer_transform.getOrigin().x();
		t_y = input_cloud->points[i].y-buffer_transform.getOrigin().y();
		t_yaw = tf::getYaw(buffer_transform.getRotation());
		return_globalxy(t_x,t_y,t_yaw,r_x,r_y);

		int x = ((grid_dim_/2)+(r_x)/m_per_cell_);
		int y = ((grid_dim_/2)+(r_y)/m_per_cell_);	

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
		t_x = input_cloud->points[i].x-buffer_transform.getOrigin().x();
		t_y = input_cloud->points[i].y-buffer_transform.getOrigin().y();
		t_yaw = tf::getYaw(buffer_transform.getRotation());
		
		return_globalxy(t_x,t_y,t_yaw,r_x,r_y);
		int x = ((grid_dim_/2)+(r_x)/m_per_cell_);
		
		int y = ((grid_dim_/2)+(r_y)/m_per_cell_);

		if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_ && init[x][y]) {//gridの中に

			if ((max[x][y] - min[x][y] > height_diff_threshold_) && (max[x][y] - min[x][y] < 10.0) ) {  
				if(!prob_flag[x][y]) prob[x][y]++;

				prob_flag[x][y] = true;
				
				pcl::PointXYZI temp_point;
				temp_point.x = r_x; 
				temp_point.y = r_y;	
				temp_point.z = input_cloud->points[i].z;
				temp_point.intensity = input_cloud->points[i].intensity;

				obstacle_cloud->points.push_back(temp_point);
			}

			else {

				if(min[x][y]+0.1>input_cloud->points[i].z) {//////////12/09変更
					
					pcl::PointXYZI temp_point;
					temp_point.x = r_x; 
					temp_point.y = r_y;
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
Save_points_minmax::dynamic_or_static(int step_num,
		pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud,
		pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_cloud,
		pcl::PointCloud<pcl::PointXYZI>::Ptr static_cloud)
{

	size_t point_size = obstacle_cloud->points.size();
	
	for (size_t i = 0; i < point_size; ++i) {
		
		int x = ((grid_dim_/2)+(obstacle_cloud->points[i].x)/m_per_cell_);
		
		int y = ((grid_dim_/2)+(obstacle_cloud->points[i].y)/m_per_cell_);
		
		if (prob[x][y]>(step_num*static_threshold)) {//gridの中に
							
				pcl::PointXYZI temp_point;
				temp_point.x = obstacle_cloud->points[i].x; 
				temp_point.y = obstacle_cloud->points[i].y;
				temp_point.z = obstacle_cloud->points[i].z-buffer_transform.getOrigin().z();
				temp_point.intensity = obstacle_cloud->points[i].intensity;

				static_cloud->points.push_back(temp_point);
			}
	
		else{
				pcl::PointXYZI temp_point;
				temp_point.x = obstacle_cloud->points[i].x; 
				temp_point.y = obstacle_cloud->points[i].y;
				temp_point.z = obstacle_cloud->points[i].z-buffer_transform.getOrigin().z();
				temp_point.intensity = obstacle_cloud->points[i].intensity;

				dynamic_cloud->points.push_back(temp_point);
		}	
	}
	
}


//貯めてる点群をpclに変換
void
Save_points_minmax::save_points2pcl(int step_num,
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


void
Save_points_minmax::say(){
	
	// cout<<"x"<<buffer_transform.getOrigin().x()<<endl;
	// cout<<"y"<<buffer_transform.getOrigin().y()<<endl;
	// cout<<tf::getYaw(buffer_transform.getRotation())<<endl;

}
