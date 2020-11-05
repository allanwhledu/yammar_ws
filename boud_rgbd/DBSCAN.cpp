/*
  各种聚类算法：
  1-DBSCAN
  date:2019-07-15
*/
#include "DBSCAN.h"

using namespace std;
namespace exact_plane
{
	// float dbscan_solver::stringToFloat(string i){
	// 	stringstream sf;
	// 	float score=0;
	// 	sf<<i;
	// 	sf>>score;
	// 	return score;
	// }

	void dbscan_solver::loadData(const pcl::PointCloud<PointT> &cloud){
		// fstream file;
		// file.open(dataset,ios::in);
		// if(!file)
	  //   {
	  //       cout <<"Open File Failed!" <<endl;
	  //       vector<point> a;
	  //       return a;
	  //   }
		// vector<dbscan_point> data;
		int k=1;
		for (int i=0;i<cloud.points.size();i++)
		{
			// string temp;
			// file>>temp;
			// int split = temp.find(',',0);
			dbscan_point p(cloud.points[i].x, cloud.points[i].z, k++, cloud.points[i].intensity,cloud.points[i].y);
			dataset.push_back(p);
		}
		// file.close();
		cout<<"successful load data!"<<endl;
		// cloud_data = data;
	}

	float dbscan_solver::squareDistance(dbscan_point a,dbscan_point b){
		return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
	}

	void dbscan_solver::dbscan(){
		int len = dataset.size();
		//calculate pts
		cout<<"calculate pts"<<endl;
		for(int i=0;i<len;i++){
			for(int j=i+1;j<len;j++){
				if(squareDistance(dataset[i],dataset[j])<Eps)
					dataset[i].pts++;
					dataset[j].pts++;
			}
		}
		//core point
		cout<<"core point "<<endl;
		// vector<dbscan_point> corePoint;
		for(int i=0;i<len;i++){
			if(dataset[i].pts>=MinPts) {
				dataset[i].pointType = 3;
				corePoint.push_back(dataset[i]);
			}
		}
		// cout<<"corepoint_size: "<<corePoint.size()<<endl;
		cout<<"joint core point"<<endl;
		//joint core point
		for(int i=0;i<corePoint.size();i++){
			for(int j=i+1;j<corePoint.size();j++){
				if(squareDistance(corePoint[i],corePoint[j])<Eps){
					corePoint[i].corepts.push_back(j);
					corePoint[j].corepts.push_back(i);
				}
			}
		}
		for(int i=0;i<corePoint.size();i++){
			stack<dbscan_point*> ps;
			if(corePoint[i].visited == 1) continue;
			ps.push(&corePoint[i]);
			dbscan_point *v;
			while(!ps.empty()){
				v = ps.top();
				v->visited = 1;
				ps.pop();
				for(int j=0;j<v->corepts.size();j++){
					if(corePoint[v->corepts[j]].visited==1) continue;
					corePoint[v->corepts[j]].cluster = corePoint[i].cluster;
					// cout<<"once"<<endl;
					corePoint[v->corepts[j]].visited = 1;
					ps.push(&corePoint[v->corepts[j]]);
				}
			}
		}
		cout<<"border point,joint border point to core point"<<endl;
		//border point,joint border point to core point
		for(int i=0;i<len;i++){
			if(dataset[i].pointType==3) continue;
			for(int j=0;j<corePoint.size();j++){
				if(squareDistance(dataset[i],corePoint[j])<Eps) {
					dataset[i].pointType = 2;
					dataset[i].cluster = corePoint[j].cluster;
					break;
				}
			}
		}
		//all points in dbscan-clusters.
		TotalPts = corePoint.size();
		//down-threshold for being a meanful cluster.
		PtsThreshold = TotalPts * ratio_diff_cluster_thresh;
		//save or not.
		if (save_mode){
			cout<<"output"<<endl;
			//output
			fstream clustering;
			clustering.open("clustering.txt",ios::out|ios::app);
			// for(int i=0;i<len;i++){
			// 	if(dataset[i].pointType == 2)
			// 		clustering<<dataset[i].x<<","<<dataset[i].y<<","<<dataset[i].cluster<<","<<dataset[i].index<<"\n";
			// }
			for(int i=0;i<corePoint.size();i++){
					clustering<<corePoint[i].x<<","<<corePoint[i].y<<","<<corePoint[i].cluster<<","<<corePoint[i].index<<"\n";
			}
			clustering<<"corePoint_size: "<<corePoint.size()<<"\n";
			clustering.close();
		}
	}

	//DBSCAN聚类结果处理
	void dbscan_solver::cluster(){
		// vector<pcl::PointCloud<PointT>> cloud_list;
		vector<int> cluster_list;
		vector<int>::iterator location_index;
		for (int i=0;i<corePoint.size();i++)
		{
			cluster_list.push_back(corePoint[i].cluster);
		}
		unique_element_in_vector(cluster_list);
		if (!print_mode){  //TODO
			for (int i =0;i<cluster_list.size();i++)
			{
				cout<<"cluster_size: " <<cluster_list[i]<<endl;
			}
		}

		int cluster_num = cluster_list.size();
		vector<pcl::PointCloud<PointT>> cloud_list(cluster_num, pcl::PointCloud<PointT>());
		cloud_list_db = cloud_list;
		// pcl::PointCloud<PointT> cloud;
		PointT point;
		for (int i=0;i<corePoint.size();i++)
		{
			int clu = corePoint[i].cluster;
			location_index = find(cluster_list.begin(),cluster_list.end(),clu);
			int num = location_index - cluster_list.begin();
			if( num > cluster_num-1 || num < 0) continue;
			point.x = corePoint[i].x;
			point.y = corePoint[i].z;
			point.z = corePoint[i].y;
			point.intensity = corePoint[i].index;
			cloud_list_db[num].push_back(point);
		}
		cout<<"cluster finished."<<endl;

	}

	//抛弃数量不足的聚类结果，只取前keep_cluster_in_plane个聚类
	void dbscan_solver::discard(){
		int list_size = cloud_list_db.size();
		// vector<int>cloud_sizes;
		map<int,int> cloud_map;
		for(int i=0;i<list_size;i++)
		{
			// cloud_sizes.push_back(cloud_list_db[i].points.size());
			pair<int,int> temp;
			temp = make_pair(cloud_list_db[i].points.size(),i);
			cloud_map.insert(temp);
		}
		map<int, int>::reverse_iterator iter;
		// vector<pcl::PointCloud<PointT>>::iterator iter_discard;
		vector<pcl::PointCloud<PointT>> cloud_remain;
		iter = cloud_map.rbegin();
		int threshold = PtsThreshold > basic_diff_cluster_thresh_global
										? PtsThreshold:basic_diff_cluster_thresh_global;
		while(keep_cluster_in_plane--)
		{
			if(iter != cloud_map.rend())
			{
				int tt = cloud_list_db[iter->second].points.size();
				if(tt > threshold){
					cloud_remain.push_back(cloud_list_db[iter->second]);
				}
				iter++;
			}
			else{
				break;
			}
		}
		cloud_list_db.clear();
		cloud_list_db = cloud_remain;
		if (print_mode){
			for(int i=0;i<cloud_list_db.size();i++){
				cout<<"size"<<cloud_list_db[i].points.size()<<endl;
			}
		}
		cout<<"discard finished."<<endl;
	}

	//可视化显示聚类结果
	void dbscan_solver::visualize(const string name){
		//visualization.
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(name));

		for (int i=0; i<cloud_list_db.size();i++)
		{
			stringstream ss;
			ss <<"cluster_" << i << ".pcd";//路径加文件名和后缀
			pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>);
			cloud_ptr = cloud_list_db[i].makeShared();
			int *rgb = rand_rgb();//随机生成0-255的颜色值
			pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb1(cloud_ptr,rgb[0],rgb[1],rgb[2]);//提取的平面不同彩色展示
			delete[]rgb;
			viewer->addPointCloud(cloud_ptr, rgb1, ss.str());
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, ss.str());
			// ss1 <<"ext_plane_clouds" << 2 << ".pcd";//路径加文件名和后缀
			// pcl::PointCloud<PointT>::Ptr cornerPointsSharp_ptr(new pcl::PointCloud<PointT>);
			// cornerPointsSharp_ptr = cornerPointsSharp.makeShared();
			// rgb = rand_rgb();//随机生成0-255的颜色值
			// visualization::PointCloudColorHandlerCustom<PointT>rgb2(cornerPointsSharp_ptr,rgb[0],rgb[1],rgb[2]);//提取的平面不同彩色展示
			// delete[]rgb;
			// viewer->addPointCloud(cornerPointsSharp_ptr, rgb2 ,ss1.str());
			// viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, ss1.str());
		}
		viewer->setBackgroundColor(255,255,255);
		while(!(viewer->wasStopped())){ // Display the visualiser until 'q' key is pressed
				viewer->spinOnce ();
		}
		viewer->close();
	}

	//query for private data.
	vector<pcl::PointCloud<PointT>> dbscan_solver::get_cloud_list_db() const
	{
		return cloud_list_db;
	}

}//end DBSCAN.cpp
