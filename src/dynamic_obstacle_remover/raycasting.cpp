#include "raycasting.hpp"

Raycasting::Raycasting()
{
	n.getParam("/theta_cell",theta_dim_);
	n.getParam("/r_cell",r_dim_);
  	n.getParam("radius_length", radius_length);
	
	occupy = vector< vector<float> >(r_dim_ , vector<float>(theta_dim_, -1.0));
}


//
void
Raycasting::Cast(pcl::PointCloud<pcl::PointXYZI>::Ptr scan)
{
	size_t cloud_size = scan->points.size();

	for (unsigned i = 0; i < cloud_size; ++i) {
        

		xy2rtheta(scan->points[i].x, scan->points[i].y, r_, theta_);
		int r = (int)(r_ / p_cell_size);
		int theta = (int)theta_/(M_degree/theta_dim_);
		if(theta == theta_dim_) theta--;
		if(r == p_grid_length) r--;

		if(r >= 0 && r < p_grid_length && theta >= 0 && theta < theta_dim_ && p_init[theta][r] ){
			occupy[r][theta] = 1.0;
		}

	}

}
