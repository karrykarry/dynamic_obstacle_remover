#include "save_points.hpp"
#include "tool.hpp"


Save_points::Save_points()
{

}

//paramの読み込み
void
Save_points::prepare(int step_n,int r_length,float per_cell,float s_threshold){

	step_num = step_n;
	grid_dim_ = r_length*2/per_cell;
	grid_dim_ex_ = grid_dim_+1;
	m_per_cell_ = per_cell;
	static_threshold = s_threshold;

	save_point2 = vector<sensor_msgs::PointCloud2> (step_n);
	swap_save_point2 = vector<sensor_msgs::PointCloud2> (step_n);
	prob = vector< vector<float> >(grid_dim_ex_, vector<float>(grid_dim_ex_,0));

	cout<<"gridの個数："<<grid_dim_<<endl;
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
		listener.waitForTransform(Child_id, Parent_id, time_now, ros::Duration(0.5));
		
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
Save_points::return_globalxy(double x, double y, double yaw, double& return_x, double& return_y){
	
	Eigen::Rotation2Dd rot(yaw*(-1));
	Eigen::Vector2d input_xy;
	Eigen::Vector2d ans;

	input_xy << x, y;

	ans = rot*input_xy;
	if(fabs(ans.x())<0.1) ans.x() = 0;
	if(fabs(ans.y())<0.1) ans.y() = 0;

	return_x = ans.x();
	return_y = ans.y();
}



void
Save_points::withprob_method(
		sensor_msgs::PointCloud2 s_points, 
		pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud) 
{

	bool prob_flag[grid_dim_ex_][grid_dim_ex_]; //1回のminmax-methodで1回だけカウントするため
	//初期化
	fill(prob_flag[0],prob_flag[grid_dim_ex_],0);

	double t_x=0,t_y=0,t_yaw=0,r_x=0,r_y=0;

	pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZI>);

	pcl::fromROSMsg(s_points, *input_cloud);   
	size_t point_size = input_cloud->points.size();
	// build height map
	// cout<<point_size<<endl;

	for (size_t i = 0; i < point_size; ++i) {
		t_x = input_cloud->points[i].x-buffer_transform.getOrigin().x();
		t_y = input_cloud->points[i].y-buffer_transform.getOrigin().y();
		t_yaw = tf::getYaw(buffer_transform.getRotation());
		return_globalxy(t_x,t_y,t_yaw,r_x,r_y);

		int x = ((grid_dim_/2)+(r_x)/m_per_cell_);
		int y = ((grid_dim_/2)+(r_y)/m_per_cell_);	

		if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_) {

			if(!prob_flag[x][y]) prob[x][y]++;

			prob_flag[x][y] = true;

		 if(!count_rs){
			pcl::PointXYZI temp_point;
			temp_point.x = r_x; 
			temp_point.y = r_y;	
			temp_point.z = input_cloud->points[i].z;
			// temp_point.intensity = input_cloud->points[i].intensity;
			temp_point.intensity = input_cloud->points[i].intensity;

			obstacle_cloud->points.push_back(temp_point);
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
		
		int x = ((grid_dim_/2)+(obstacle_cloud->points[i].x)/m_per_cell_);
		
		int y = ((grid_dim_/2)+(obstacle_cloud->points[i].y)/m_per_cell_);
		
		if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_) {
		
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
				// temp_point.intensity = prob[x][y];

				dynamic_cloud->points.push_back(temp_point);
		}	
		}
	}

		// cout<<buffer_transform.getOrigin().z()<<endl;


}


//貯めてる点群をpclに変換
void
Save_points::save_points2pcl(int step_num,
		pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_cloud,
		pcl::PointCloud<pcl::PointXYZI>::Ptr static_cloud)
{

	pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud (new pcl::PointCloud<pcl::PointXYZI>);

	for(int i=0;i<step_num;i++){
		count_rs = i;
		withprob_method(save_point2[i], obstacle_cloud);

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
Save_points::say(){
	
	// cout<<"x"<<buffer_transform.getOrigin().x()<<endl;
	// cout<<"y"<<buffer_transform.getOrigin().y()<<endl;
	// cout<<tf::getYaw(buffer_transform.getRotation())<<endl;

}
