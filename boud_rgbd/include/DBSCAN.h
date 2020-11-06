/*
  各种聚类算法：
  1-DBSCAN
  date:2019-07-15
*/
#ifndef DBSCAN_H
#define DBSCAN_H
//sys
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <ctime>
#include <cstdlib>
#include <limits>
#include <cmath>
#include <stack>
#include <algorithm>
#include <map>
//pcl TODO
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

using namespace std;

typedef pcl::PointXYZI PointT;
extern bool print_mode;
extern bool visualization_mode;
extern bool save_mode;
extern int keep_cluster_in_plane;
extern float ratio_diff_cluster_thresh_global;
extern int basic_diff_cluster_thresh_global;


namespace exact_plane
{
  int *rand_rgb()
  {
  	int *rgb = new int[3];
  	rgb[0] = rand() % 255;
  	rgb[1] = rand() % 255;
  	rgb[2] = rand() % 255;
  	return rgb;
  }

  void unique_element_in_vector(vector<int> &v)
  {
    vector<int>::iterator vector_iterator;
    sort(v.begin(),v.end());
    vector_iterator = unique(v.begin(),v.end());
    if(vector_iterator != v.end()){
      v.erase(vector_iterator,v.end());
    }
    // return v;
  }

  //point class used in dbscan.
  class dbscan_point{
  public:
  	float x;   //refer to cloud.point.x
  	float y;   //refer to cloud.point.z
    float z;   //refer to cloud.point.y, due to coordinate change.
    int cluster=0;
    int index; //index in origianl point cloud
  	int pointType=1;//1 noise 2 border 3 core
  	int pts=0;//points in MinPts
  	vector<int> corepts;
  	int visited = 0;
  	dbscan_point (){}
  	dbscan_point (float a,float b,int c,int d, float e){
  		x = a;
  		y = b;
  		cluster = c;
      index = d;
      z = e;
  	}
  };

  class dbscan_solver
  {
  public:
    dbscan_solver(){};
    dbscan_solver(float eps,int minpts)
    {
      Eps = eps;
      MinPts = minpts;
      ratio_diff_cluster_thresh = ratio_diff_cluster_thresh_global;
    };
    ~dbscan_solver(){};

    // float stringToFloat(string i);
    float squareDistance(dbscan_point a, dbscan_point b);
    void loadData(const pcl::PointCloud<PointT> &cloud);
    void dbscan();
    void cluster();
    void visualize(const string name);
    void discard();
    vector<pcl::PointCloud<PointT>> get_cloud_list_db() const;

  private:

    float Eps;
    float ratio_diff_cluster_thresh;
    int MinPts;
    int TotalPts;
    int PtsThreshold;
    vector<dbscan_point> dataset;
    vector<dbscan_point> corePoint;
    vector<pcl::PointCloud<PointT>> cloud_list_db;
  };

}


#endif  // DBSCAN_H
