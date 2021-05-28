#include <iostream>
#include <string.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointField.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Core>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/mls.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/filters/extract_indices.h>

#include "height_msgs/height.h"
#include "border_msgs/border.h"
#include "curve_point_z_msgs/curve_point_z.h"
#include "fit.h"

vector<float> coeff_uncut_height_mean(4,0);
int Estimated_height=0;  //估计的高度平均值

ros::Publisher pointcloud_pub;   //发布点云
ros::Publisher border_param;     //发布分界线
ros::Publisher curve_point_pub;  //发布转晚点
ros::Publisher height_pub;       //发布作物高度
image_transport::Publisher image_pub;
using namespace std;
using namespace cv;

//按y值升序
struct cmp{
    bool operator()(Point2i a,Point2i b) {
        return a.y < b.y;
    }
};

void border_RGBD(const sensor_msgs::ImageConstPtr& rgbimg,const sensor_msgs::ImageConstPtr& depthimg); //主函数入口,回调函数


//基于颜色空间模型,检测导航路径线点
void border_rgb(Mat& rgb,vector<Point2i>& inliners_rgb);
int calHist(Mat& gray_2r_g_b); //灰度值直方图统计
int drawHist(vector<int> nums_gray_value); //灰度值直方图绘制
void removeHole(Mat& close);  //去除mask左下角黑洞
int chooseCenterROI(Mat& close); //自适应获取ROI中心线
Mat get_Vertical(Mat src); //获取水平算子
Mat get_Horizontal(Mat src);//获取垂直算子
Mat erode_Img(Mat src,Mat kernel); //腐蚀
Mat dilate_Img(Mat src, Mat kernel); //膨胀
bool ifRegionValid(Mat& ROIimg,int lrows,int hrows,int lcols,int hcols); //判断未收割区域分割是否正确
void remove_contour(Mat& ROIimg,int lrows,int hrows,int lcols,int hcols); //去除边缘检测,边界线右侧的杂线
void removeOutlier(vector<Point2i> inData, int radius, int k, vector<Point2i> &outData);//去除离散点


//基于深度信息,检测导航路径线点
void border_depth(Mat& rgb,Mat& depth,vector<Point2i>& inliners_depth_2D,vector<Point3f>& inliners_depth_3D);
void transform_to_pointcloud(Mat& rgb,Mat& depth,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudin);  //恢复三维结构
void uncutRegion_search(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudin,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& plane_uncut);   //查找未收割区域
bool ifPlane_uncut_valid(Mat& rgb,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& plane_uncut); //判断未收割区域有效性
Eigen::VectorXf borderpoints_clusterd(Mat& rgb,Mat& depth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& plane_uncut,
                                      vector<Point2i>& pointimg,vector<Point3f>& pointimg_3d); //二维三维分界线点初步聚类
void removeOutlier_depth_2D_3D(vector<Point2i>& inData_depth_2D, vector<Point2i> &outData_depth_2D, int radius, int k,
                               vector<Point3f>& inData_depth_3D, vector<Point3f>& outData_depth_3D); //二维三维外点去除
void border_offset(Mat& rgb,vector<Point2i>& pointimg,vector<Point3f>& pointimg_3d,
                   border_msgs::border& borderMsg,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudin);        //三维真实偏移量计算
void drawArrow(cv::Mat& img, cv::Point pStart, cv::Point pEnd, int len, int alpha,
               cv::Scalar& color, int thickness = 5, int lineType = 8);
float max_num(vector<float> height);
Point3f random_rotateObject(double x, double y, double z, double a, double b, double c,double a0,double b0,double c0,double theta);
void dis_cal(vector<float>& dis,vector<Point3f>& pointdepthimg_3d,Point3f begin_standard, Point3f end_standard);


//rgb+depth导航路径点融合
void removeOutlier_depth_rgb(vector<Point2i>& inData_rgb, vector<Point2i>& inData_depth_2D, vector<Point2i> &outData_fusion_2D, int radius, int k,
                             vector<Point3f>& inData_depth_3D, vector<Point3f>& outData_fusion_3D);  //融合后,再去除一次外点
void kdtree_fusion( vector<Point2i>& outData_fusion_2D,vector<Point2i>& outData_fusion_kdtree);      //kdtree融合
void final_discrete(Mat& rgb, vector<Point2i>& outData_fusion_kdtree);  //离散化路径点,避免坐标点重叠
void curve_detect(Mat& rgb, Mat& depth, vector<Point2i>& outData_fusion_kdtree, curve_point_z_msgs::curve_point_z& curve_pointMsg);   //转弯点识别
void Affine_trans(Mat& rgb,vector<Point2i>& outData_fusion_kdtree);  //仿射变换


//height_detection
void height_detection(Mat& rgb,Mat& depth, height_msgs::height& heightMsg);  //作物高度估计主函数入口


int main(int argc,char** argv)
{
    ros::init(argc,argv,"border_height_detect");
    ros::NodeHandle nh;
    image_transport::ImageTransport transport(nh);
    image_pub= transport.advertise("/border", 1);

    //同步接收rgb,depth
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/realsense_sr300/ylx/rgb", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/realsense_sr300/ylx/depth", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);

    sync.registerCallback(boost::bind(&border_RGBD,_1,_2));//主函数入口

    pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/farm_cloud", 1000);
    border_param=nh.advertise<border_msgs::border>("/border_param", 1000);
    curve_point_pub=nh.advertise<curve_point_z_msgs::curve_point_z>("/curve_point_z", 1000);
    height_pub = nh.advertise<height_msgs::height>("/height", 1000);
    ros::spin();
    return 0;
}

void border_RGBD(const sensor_msgs::ImageConstPtr& rgbimg,const sensor_msgs::ImageConstPtr& depthimg) {
    //图像预处理,rgb,depth信息
    Mat depth_raw;
    cv_bridge::CvImageConstPtr cv_ptrdepth;
    cv_ptrdepth = cv_bridge::toCvShare(depthimg);
    depth_raw = cv_ptrdepth->image;

    Mat rgb;
    cv_bridge::CvImageConstPtr cv_ptrrgb;
    cv_ptrrgb = cv_bridge::toCvShare(rgbimg);
    rgb = cv_ptrrgb->image;

    Mat depth(depth_raw.size(),CV_16UC1);
    for(int row=0;row<rgb.rows;row++)
        for(int col=0;col<rgb.cols;col++ ) {
            float z = float(depth_raw.at<float>(row, col)) * 1000; //深度图之前是32FC1编码
            depth.at<ushort>(row, col) = z;  //转成16位编码

            Vec3b temp = rgb.at<Vec3b>(row, col);
            rgb.at<Vec3b>(row, col)[0] = temp[2];
            rgb.at<Vec3b>(row, col)[2] = temp[0];
        }

    vector<Point2i> inliners_rgb;       //基于颜色获取的导航路径点,二维
    border_rgb(rgb,inliners_rgb);       //基于颜色获取导航路径点方法,主函数入口

    vector<Point2i> inliners_depth_2D;  //基于深度信息获取的导航路径点,二维
    vector<Point3f> inliners_depth_3D;  //基于深度信息获取的导航路径点,三维
    border_depth(rgb, depth,inliners_depth_2D,inliners_depth_3D); //基于深度信息获取导航路径点方法,主函数入口

    vector<Point2i> outData_fusion_2D;
    vector<Point3f> outData_fusion_3D;
    removeOutlier_depth_rgb(inliners_rgb, inliners_depth_2D, outData_fusion_2D, 30, 5,
                            inliners_depth_3D, outData_fusion_3D);   //两种方法二维点叠加之后去除外点

    vector<Point2i> outData_fusion_kdtree;
    kdtree_fusion(outData_fusion_2D,outData_fusion_kdtree); //基于kdtree最近邻搜索,设置权重值调整路径点位置.

//    for(int i=0;i<outData_fusion_kdtree.size();i++)
//        circle(rgb, Point(outData_fusion_kdtree[i].x,outData_fusion_kdtree[i].y), 3, Scalar(0, 0, 255));

    if(outData_fusion_kdtree.size() < 20) outData_fusion_kdtree=outData_fusion_2D; //去除离散点后剩余点太少则不去除（离散点中也有正确的点,只是较稀疏）
    final_discrete(rgb, outData_fusion_kdtree);    //均匀离散路径点,避免点重合,不利于控制

    std_msgs::Header curve_point_header;
    curve_point_z_msgs::curve_point_z curve_pointMsg;
    curve_pointMsg.header = curve_point_header;
    curve_pointMsg.dis_z=0;
    curve_detect(rgb,depth,outData_fusion_kdtree,curve_pointMsg);       //转弯点识别

    Affine_trans(rgb,outData_fusion_kdtree);

    if(outData_fusion_kdtree.size() > 20 && curve_pointMsg.dis_z==0)
    {
        std_msgs::Header height_header;
        height_msgs::height heightMsg;
        heightMsg.header = height_header;
        heightMsg.height=0;
        height_detection(rgb,depth,heightMsg);
    }
    imshow("reslut",rgb);
    waitKey(1);
}

void border_rgb(Mat& rgb, vector<Point2i>& inliners_rgb) {
    Mat rgb_clone = rgb.clone();
    Mat Gaussian, Binarization, close, ROIimg;

    GaussianBlur(rgb_clone, Gaussian, Size(5, 5), 0.7);
    vector<Mat> channels;
    split(Gaussian, channels);
    Mat blue = channels.at(0);
    Mat green = channels.at(1);
    Mat red = channels.at(2);

    Binarization = 2 * red - green - blue;//2R-G-B超红模型特征
    int treshod = calHist(Binarization); //计算&绘制 灰度值直方图
   // imshow("2R_G_B", Binarization);

    Binarization = Binarization > treshod;//二值化图片,根据灰度值直方图,最后一个波峰附近为合适的阈值(自适应选择阈值)
   // imshow("Binarization", Binarization);

    Mat kernel_close = getStructuringElement(MORPH_ELLIPSE, Size(9, 9));
    morphologyEx(Binarization, close, MORPH_CLOSE, kernel_close);//闭运算,去除小型黑洞，先膨胀，再腐蚀
   // imshow("Binarization_close", close);

    //消除左下角黑洞
    removeHole(close);

    //按列索引,寻找ROI窗口中心线，自适应选择中心
    int colsROI = chooseCenterROI(close);

    if (colsROI > 150 && colsROI < 440) {  //避免ROI超出矩阵大小
        Rect ROI(colsROI - 100, 200, 150, 200);//设置ROI

        Mat mask = Mat::zeros(rgb.size(), CV_8UC1);
        mask(ROI).setTo(255);
        close.copyTo(ROIimg, mask);

        Canny(ROIimg, ROIimg, 100, 200, 3);
        //imshow("canny", ROIimg);

        Mat verticalLine = get_Vertical(ROIimg);
        Mat horizontalLine = get_Horizontal(ROIimg);

        Mat vertical_Line_erode = erode_Img(ROIimg, verticalLine);
        Mat vertical_Line_dilate = dilate_Img(vertical_Line_erode, verticalLine);
        Mat horizontal_Line_erode = erode_Img(ROIimg, horizontalLine);
        Mat horizontal_Line_dilate = dilate_Img(horizontal_Line_erode, horizontalLine);

        subtract(ROIimg, vertical_Line_dilate, ROIimg, Mat(), -1);
        subtract(ROIimg, horizontal_Line_dilate, ROIimg, Mat(), -1);

        bool regionValid = ifRegionValid(ROIimg,200,400,colsROI-100,colsROI+50);

        remove_contour(ROIimg,200,400,colsROI-100,colsROI+50);
     //   imshow("removeVerandHori", ROIimg);


        vector<Point2i> linepoints;
        for (int i = 200; i < 400; i+=4) {  //i,j遍历范围根据ROI设置
            for (int j = colsROI - 100; j < colsROI + 50; j++) {
                if (int(ROIimg.at<uchar>(i, j))==255)
                {
                    linepoints.push_back(Point(j, i));
                    break;
                }
            }
        }
        removeOutlier(linepoints,20,5,inliners_rgb); //去除离群点

        if(inliners_rgb.size() > 20)
        {
            vector<Point> convex_all;
            convexHull(Mat(inliners_rgb), convex_all, true);

            if(contourArea(convex_all, false) / (ROI.width * ROI.height) < 0.3 && regionValid) //去除点全错误的情况
            {
                for(int i=0;i<inliners_rgb.size();i++)
                {
                    circle(rgb_clone, Point(inliners_rgb[i].x,inliners_rgb[i].y), 3, Scalar(255, 0, 0));
                }
            }
            else inliners_rgb.clear();
        }
        else inliners_rgb.clear();
    }
  //  imshow("result", rgb_clone);
  //  waitKey(1);
}


//计算直方图，统计各灰度级像素个数
int calHist(Mat& gray_2r_g_b)
{
    //计算各灰度级像素个数
    vector<int> nums_gray_value(256);
    for (int i = 0; i < gray_2r_g_b.rows; i++)
    {
        uchar* p = gray_2r_g_b.ptr<uchar>(i);
        for (int j = 0; j < gray_2r_g_b.cols; j++)
        {
            nums_gray_value[p[j]]++;
        }
    }
    for(int i=0;i<40;i++)
        nums_gray_value[i]=0;

    return drawHist(nums_gray_value);
}

//绘制灰度值直方图
int drawHist(vector<int> nums_gray_value)
{
    Mat hist( 600, 800, CV_8UC3, Scalar(255,255,255));
    auto Max = max_element(nums_gray_value.begin(), nums_gray_value.end());//max迭代器类型,最大数目
    putText(hist, "Hist", Point(370, 100), FONT_HERSHEY_DUPLEX, 2, Scalar(0, 0, 0),2);
    //*********绘制坐标系************//
    Point o = Point(100, 550);
    Point x = Point(700, 550);
    Point y = Point(100, 150);

    //x轴,y轴绘制
    arrowedLine(hist, o, x, Scalar(0, 0, 0), 3, 8, 0, 0.03);
    arrowedLine(hist, o, y, Scalar(0, 0, 0), 3, 8, 0, 0.03);

    //********绘制灰度曲线***********//
    vector<Point> pts(256);
    //生成坐标值,x轴
    for (int i = 0; i < 256; i++)
    {
        pts[i].x = i * 2 + 100;
        pts[i].y = 550 - int(nums_gray_value[i]*(300.0/(*Max)));//归一化到[0, 300]
        //显示横坐标
        if ((i + 1) % 16 == 0)
        {
            string num = format("%d", i + 1);
            putText(hist, num, Point(pts[i].x, 570), FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 0, 0));
        }
    }
    //生成坐标值,y轴
    for (int i = 550; i >= 200; i-=50)
    {
        if (i % 50 == 0)
        {
            string num = format("%d", (11-i/50) * 50 );
            putText(hist, num, Point(65, i), FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 0, 0));
        }
    }

    //绘制线
    for (int i = 1; i < 256; i++)
    {
        line(hist, pts[i - 1], pts[i], Scalar(100, 200, 0), 2);
    }

    //高阶曲线拟合,fit类
    vector<double> pts_x; //x,y坐标输入
    vector<double> pts_y;
    for(int i=0;i<pts.size();i++)
    {
        pts_x.push_back(pts[i].x);
        pts_y.push_back(pts[i].y);
    }

    Fit fit;
    fit.polyfit(pts_x,pts_y,19,true);
    vector<double> y_fitted; //拟合后的y值
    fit.getFitedYs(y_fitted);

    //绘制拟合后的曲线
    for (int i = 1; i < 256; i++)
    {
        pts[i].y=y_fitted[i];
        Point pre;
        Point cur;
        pre.x=pts_x[i-1];
        pre.y=y_fitted[i-1];

        cur.x=pts_x[i];
        cur.y=y_fitted[i];

        line(hist,pre, cur, Scalar(0, 200, 200), 2);
    }

    vector<int> t_shod;
    for(int i=1;i<y_fitted.size()-1;i++)
    {
        //选取峰值,峰值较大,峰值小的是曲线拟合偏差
        //峰值可以作为二值化阈值依据
        if(y_fitted[i] <= y_fitted[i-1] && y_fitted[i] <= y_fitted[i+1] && y_fitted[i] < 500)
            t_shod.push_back(pts[i].x);
    }
    //imshow("Histogram" ,hist);

    //自适应选择阈值
    if(t_shod.size()==0) return 57;
    else if(t_shod.size()==1)  return (t_shod.back()-100)/2-5;
    else return (t_shod.back()-100)/2-20;
}

void removeHole(Mat& close)
{
    //消除左下角黑洞
    //1.设置mask: close_contours
    //2.为了轮廓检测,将close_contours中的值为255的地方设成0,0的地方设置成255
    //3.轮廓填充
    //4.将填充后的黑洞复制到原二值化图close
    Mat close_contours=close(Rect(0,150,300,330)).clone();
    for(int i=0;i<close_contours.rows;i++)
        for(int j=0;j<close_contours.cols;j++)
        {
            if((int) close_contours.at<uchar>(i,j)==255) close_contours.at<uchar>(i,j)=0;
            else close_contours.at<uchar>(i,j)=255;
        }
    std::vector<vector<Point> > contours;
    std::vector<Vec4i> hierarchy;
    findContours(close_contours, contours, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

    vector<Vec4i> empty(0);
    for (size_t k = 0; k < contours.size(); k++) {
        for(int i=0;i<contours[k].size();i++)
        {
            //将最上侧,右侧轮廓区域去除,不是要填充的黑洞
            if(contours[k][i].x > close_contours.cols-50)
            {
                contours[k][i].x=close_contours.cols-50;
                contours[k][i].y=close_contours.rows;
            }
            if(contours[k][i].y < 50)
            {
                contours[k][i].x=0;
                contours[k][i].y=50;
            }
        }
        drawContours(close_contours, contours, k, 100, -1, LINE_8, empty, 0, Point(0, 0));
    }

    for(int i=0;i<close_contours.rows;i++)
        for(int j=0;j<close_contours.cols;j++)
        {
            if((int)close_contours.at<uchar>(i,j)==100)
                close.at<uchar>(i+150,j)=255;
        }
    //imshow("close_contours2",close);
}
//自适应计算ROI中心
int chooseCenterROI(Mat& close)
{
    int colsROI=0;
    vector<int> count255;
    for (int i = 0; i < close.cols; i++)
    {
        int count = 0;
        for (int j = 0; j < close.rows; j++)
        {
            if (int(close.at<uchar>(j, i))==0)
            count++;
        }
        if (count>= 450) //根据安装位置可进行调整
        {
            colsROI = i;
            break;
        }
    }
    return colsROI;
}

Mat get_Vertical(Mat src) //结构元素（获取垂直算子）
{
    Mat dst = Mat::zeros(src.size(), src.type());
    return getStructuringElement(MORPH_RECT,Size(src.cols/16,1),Point(-1,-1));
}

Mat get_Horizontal(Mat src) //结构元素（获取水平算子）
{
    Mat dst = Mat::zeros(src.size(), src.type());
    return getStructuringElement(MORPH_RECT, Size(1, src.rows / 16), Point(-1, -1));
}

Mat erode_Img(Mat src,Mat kernel) //腐蚀
{
    Mat dst = Mat::zeros(src.size(), src.type());
    erode(src, dst, kernel);
    return dst;
}

Mat dilate_Img(Mat src, Mat kernel) //膨胀
{
    Mat dst = Mat::zeros(src.size(), src.type());
    dilate(src, dst, kernel);
    return dst;
}

bool ifRegionValid(Mat& ROIimg,int lrows,int hrows,int lcols,int hcols)
{
    int count_255=0;
    for(int i=lrows;i<=hrows;i++)
    {
        for(int j=lcols;j<=hcols;j++)
        {
            if((int)ROIimg.at<uchar>(i,j)==255)
            {
                count_255++;
            }
        }
    }
    return (count_255 < 400) ? true:false;
}
//去除边缘检测,边界线右侧的杂线
void remove_contour(Mat& ROIimg,int lrows,int hrows,int lcols,int hcols)
{
    for(int i=lrows;i<=hrows;i++)
    {
        for(int j=lcols;j<=hcols;j++)
        {
            if((int)ROIimg.at<uchar>(i,j)==255)
            {
                for(int k=j+1;k<=hcols;k++)
                    ROIimg.at<uchar>(i,k) = 0;
                j=hcols+1;
            }
        }
    }
}

//半径radius中点的个数k,去除离群点
void removeOutlier(vector<Point2i> inData, int radius, int k, vector<Point2i> &outData)
{
    outData.clear();

    int cnt = 0;
    int n = 0;
    for (int m = 0; m < inData.size(); m++)
    {
        cnt = 0;
        for (n = 0; n < inData.size(); n++)
        {
            if (n == m)
                continue;

            if (sqrt(pow(inData[m].x - inData[n].x,2) + pow(inData[m].y - inData[n].y,2)) <= radius)
            {
                cnt++;
                if (cnt >= k) //超过k,保存
                {
                    outData.push_back(inData[m]);
                    n = 0;
                    break;
                }
            }
        }
    }
}

void border_depth(Mat& rgb, Mat& depth,vector<Point2i>& inliners_depth_2D,vector<Point3f>& inliners_depth_3D)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudin (new pcl::PointCloud<pcl::PointXYZRGB>);
    transform_to_pointcloud(rgb,depth,cloudin);   //恢复三维结构

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_uncut(new pcl::PointCloud<pcl::PointXYZRGB>);
    uncutRegion_search(cloudin,plane_uncut);  //查找未收割区域

    if(ifPlane_uncut_valid(rgb,plane_uncut))
    {
        vector<Point3f> pointimg_3d;
        vector<Point2i> pointimg;
        Eigen::VectorXf coeff_uncut;
        coeff_uncut=borderpoints_clusterd(rgb,depth,cloudin,plane_uncut,pointimg,pointimg_3d);

        removeOutlier_depth_2D_3D(pointimg, inliners_depth_2D, 30, 3, pointimg_3d, inliners_depth_3D);

        std_msgs::Header borderHeader;
        border_msgs::border borderMsg;
        borderMsg.header = borderHeader;
        border_offset(rgb,pointimg,pointimg_3d,borderMsg,cloudin);

        cv_bridge::CvImage cvi;
        ros::Time time = ros::Time::now();
        cvi.header.stamp = time;
        cvi.header.frame_id = "image";
        cvi.encoding = "bgr8";
        cvi.image = rgb;
        sensor_msgs::Image im;
        cvi.toImageMsg(im);
        image_pub.publish(im);
        sensor_msgs::PointCloud2 cloud_boud;
        for(int i=0;i<plane_uncut->size();i++)
        {
            plane_uncut->points[i].g=255;
            plane_uncut->points[i].r=255;
        }
        cloudin->width = cloudin->points.size();
        pcl::toROSMsg(*cloudin + *plane_uncut, cloud_boud);
        pointcloud_pub.publish(cloud_boud);
    }

//    imshow("rgb", rgb);
//    waitKey(1);
}

void transform_to_pointcloud(Mat& rgb,Mat& depth,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudin)
{
    cloudin->header.frame_id="/frame";
    pcl::PointXYZRGB Point;
    for(int row=0;row<rgb.rows;row+=3)  //row,col这里做了稀疏化,就不用滤波稀疏了
        for(int col=0;col<rgb.cols/2+50;col+=3)  //这里只取了深度图像的左边一半的点云,减少计算量,
        {
            float z = float(depth.at<ushort>(row,col))/1000;
            float y = (row - 232.171) * z / 615.312;
            float x = (col - 323.844) * z / 615.372;

            if(y>0 && z<10)  //根据相机坐标系,只选择向下的点云,z方向十米以内的点云;
            {
                Point.x=x;
                Point.y=y;
                Point.z=z;
                Point.b=rgb.ptr<uchar>(row)[col*3];
                Point.g=rgb.ptr<uchar>(row)[col*3+1];
                Point.r=rgb.ptr<uchar>(row)[col*3+2];
                cloudin->points.push_back(Point);
            }
        }
    //pointcloud_pub.publish(cloudin);
}

//考虑检测未收割平面,点云分割
void uncutRegion_search(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudin,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& plane_uncut)
{
//    未收割直接根据物理安装分割
//    for (int index=0; index<cloudin->size();index++)
//    {
//        if(cloudin->points[index].x<-0.3)
//        {
//            cloudin->points[index].r=100;
//            cloudin->points[index].g=100;
//            cloudin->points[index].b=0;
//            plane_uncut->push_back(cloudin->points[index]);//未收割作物平面
//        }
//    }

//局域平面分割模型分割未收割平面
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    // 距离阈值 单位m
    seg.setDistanceThreshold (0.1);
    seg.setInputCloud (cloudin);
    seg.segment (*inliers, *coefficients);

    // 提取地面
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloudin);
    extract.setIndices (inliers);
    extract.filter (*cloud_filtered);

//    提取除地面外的物体
//    extract.setNegative (true);
//    extract.filter (*cloud_filtered);

    cloud_filtered->header.frame_id="/frame";
    pointcloud_pub.publish(cloud_filtered);

    plane_uncut=cloud_filtered;
}

//判断未收割区域是否正确,如果正确一定靠左边,可视化project_plane调整相应参数
bool ifPlane_uncut_valid(Mat& rgb,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& plane_uncut)
{
    Mat project_plane( rgb.size(), CV_8UC1, Scalar(0));
    int count_right_roi_1=0;
    int count_right_roi_2=0;
    int count_left_roi=0;
    int row=0;int col=0;
    for(int i=0;i<plane_uncut->points.size();i++)
    {
        row = 615.312 * plane_uncut->points[i].y  / plane_uncut->points[i].z  + 232.171;
        col = 615.372 * plane_uncut->points[i].x  / plane_uncut->points[i].z  + 323.844;
        if(rgb.cols/2+50 - col < 5) count_right_roi_1++;
        if(rgb.cols/2+50 - col < 30) count_right_roi_2++;
        if(col < 50) count_left_roi++;
        project_plane.at<uchar>(row,col) = 255;
    }
//    imshow("project",project_plane);
//    cout<<plane_uncut->points.size() <<" "<<count_left_roi<<" "<<count_right_roi_1<<" "<<count_right_roi_2<<endl;
//    阈值参数细调,是否可以自适应调整                                                    &&  count_right_roi_2 > 150 识别边界
    if(count_right_roi_1 > 10 || count_right_roi_2 > 150 || plane_uncut->points.size()  < 4000) return false;
    else if(count_left_roi  > 500) return true;
    else return false;
}

//二维三维分界线点初步聚类
Eigen::VectorXf borderpoints_clusterd(Mat& rgb, Mat& depth,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudin,
                                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr& plane_uncut,vector<Point2i>& pointimg,vector<Point3f>& pointimg_3d)
{
    //基准面平面拟合,(基准面可以选择未收割平面实时估计,也可以安装好相机后以地面为基准面,在程序中写定参数)
    vector<int> inliers_uncut;
    pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model_uncut(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(plane_uncut));
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac_uncut (model_uncut);
    ransac_uncut.setDistanceThreshold(0.01);
    ransac_uncut.computeModel();
    ransac_uncut.getInliers(inliers_uncut);
    Eigen::VectorXf coeff_uncut;
    ransac_uncut.getModelCoefficients(coeff_uncut);
    pcl::copyPointCloud(*plane_uncut,inliers_uncut,*plane_uncut);

    if(coeff_uncut[3]<0)  //确保平面参数,z轴始终指向同一个方向
    {
        coeff_uncut[0]=-coeff_uncut[0];
        coeff_uncut[1]=-coeff_uncut[1];
        coeff_uncut[2]=-coeff_uncut[2];
        coeff_uncut[3]=-coeff_uncut[3];
    }
    //用作高度估计,平面参数估计平均值
    coeff_uncut_height_mean[0]= coeff_uncut_height_mean[0]==0 ? coeff_uncut[0]:(coeff_uncut[0]*0.125+coeff_uncut_height_mean[0]*0.875);
    coeff_uncut_height_mean[1]= coeff_uncut_height_mean[1]==0 ? coeff_uncut[1]:(coeff_uncut[1]*0.125+coeff_uncut_height_mean[1]*0.875);
    coeff_uncut_height_mean[2]= coeff_uncut_height_mean[2]==0 ? coeff_uncut[2]:(coeff_uncut[2]*0.125+coeff_uncut_height_mean[2]*0.875);
    coeff_uncut_height_mean[3]= coeff_uncut_height_mean[3]==0 ? coeff_uncut[3]:(coeff_uncut[3]*0.125+coeff_uncut_height_mean[3]*0.875);
    //cout<<coeff_uncut[0]<<" "<<coeff_uncut[1]<<" "<<coeff_uncut[2]<<" "<<coeff_uncut[3]<<endl;

    Point3f  pointdepth_3d; //分界线点的三维坐标
    Point2i  pointdepth;    //分界线点的二维坐标

    float A=coeff_uncut[0],B=coeff_uncut[1],C=coeff_uncut[2],D=coeff_uncut[3];  //基准面系数

    pcl::PointXYZRGB Point;

    for(int row=250;row<=rgb.rows;row+=4)  //ROI遍历范围
        for(int col=50;col<350;col++ )
        {
            float z = float(depth.at<ushort>(row,col))/1000;
            float y = (row - 232.171) * z / 615.312;
            float x = (col - 323.844) * z / 615.372;

            float distance=abs(A*x+B*y+C*z+D);
            if(distance<0.42 && distance>0.39) //距离阈值判断
            {
                pointdepth.x=col;
                pointdepth.y=row;
                int n=pointimg.size();

                //三维分界线点投影基准平面上,方便观察
                pointdepth_3d.x=((B*B+C*C)*x-A*(B*y+C*z+D));
                pointdepth_3d.y=((A*A+C*C)*y-B*(A*x+C*z+D));
                pointdepth_3d.z=((A*A+B*B)*z-C*(A*x+B*y+D));

                pointimg_3d.push_back(pointdepth_3d);
                pointimg.push_back(pointdepth);
                //circle(rgb, pointdepth, 3, Scalar(100, 255, 100));
                Point.x=pointdepth_3d.x;
                Point.y=pointdepth_3d.y;
                Point.z=pointdepth_3d.z;
                Point.r=255;
                Point.g=0;
                Point.b=0;
                cloudin->points.push_back(Point); //三维点云显示
                break;
            }
        }
    return coeff_uncut;
}

void removeOutlier_depth_2D_3D(vector<Point2i>& inData_depth_2D, vector<Point2i> &outData_depth_2D, int radius, int k,
                            vector<Point3f>& inData_depth_3D, vector<Point3f>& outData_depth_3D)
{
    vector<bool> flag(inData_depth_2D.size(),false);  //用于判断哪些点是内点,然后处理三维路径点
    outData_depth_2D.clear();

    int cnt = 0;
    int n = 0;
    for (int m = 0; m < inData_depth_2D.size(); m++)
    {
        cnt = 0;
        for (n = 0; n < inData_depth_2D.size(); n++)
        {
            if (n == m)
                continue;

            if (sqrt(pow(inData_depth_2D[m].x - inData_depth_2D[n].x,2) + pow(inData_depth_2D[m].y - inData_depth_2D[n].y,2)) <= radius)
            {
                cnt++;
                if (cnt >= k) //超过k,保存
                {
                    outData_depth_2D.push_back(inData_depth_2D[m]);
                    flag[m]=true;
                    n = 0;
                    break;
                }
            }
        }
    }
    //处理3D内点
    for(int i=0;i<inData_depth_3D.size();i++)
    {
        if(flag[i])
            outData_depth_3D.push_back(inData_depth_3D[i]);
    }
}

void removeOutlier_depth_rgb(vector<Point2i>& inData_rgb, vector<Point2i>& inData_depth_2D, vector<Point2i>& outData_fusion_2D, int radius, int k,
                             vector<Point3f>& inData_depth_3D, vector<Point3f>& outData_fusion_3D)
{
    vector<Point2i> fusion_2D;
    for(int i=0;i<inData_rgb.size();i++) fusion_2D.push_back(inData_rgb[i]);
    for(int i=0;i<inData_depth_2D.size();i++) fusion_2D.push_back(inData_depth_2D[i]);

    vector<bool> flag(inData_depth_2D.size(),false);  //用于判断哪些点是内点,然后处理三维路径点
    outData_fusion_2D.clear();

    int cnt = 0;
    int n = 0;
    for (int m = 0; m < inData_rgb.size(); m++)
    {
        cnt = 0;
        for (n = 0; n < fusion_2D.size(); n++)
        {
            if (sqrt(pow(inData_rgb[m].x - fusion_2D[n].x,2) + pow(inData_rgb[m].y - fusion_2D[n].y,2)) <= radius)
            {
                cnt++;
                if (cnt >= k) //超过k,保存
                {
                    outData_fusion_2D.push_back(inData_rgb[m]);
                    n = 0;
                    break;
                }
            }
        }
    }

    for (int m = 0; m < inData_depth_2D.size(); m++)
    {
        cnt = 0;
        for (n = 0; n < fusion_2D.size(); n++)
        {
            if (sqrt(pow(inData_depth_2D[m].x - fusion_2D[n].x,2) + pow(inData_depth_2D[m].y - fusion_2D[n].y,2)) <= radius)
            {
                cnt++;
                if (cnt >= k) //超过k,保存
                {
                    outData_fusion_2D.push_back(inData_depth_2D[m]);
                    n = 0;
                    break;
                }
            }
        }
    }
    //处理3D内点
    for(int i=0;i<inData_depth_3D.size();i++)
    {
        if(flag[i])
            outData_fusion_3D.push_back(inData_depth_3D[i]);
    }
}


void kdtree_fusion( vector<Point2i>& outData_fusion_2D,vector<Point2i>& outData_fusion_kdtree)  //kdtree融合
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_kd(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ Point_kd;
    Point2i kd_fusion;
    if(outData_fusion_2D.size() >0 )
    {
        for(int i=0;i<outData_fusion_2D.size();i++)
        {
            Point_kd.x=outData_fusion_2D[i].x;
            Point_kd.y=outData_fusion_2D[i].y;
            Point_kd.z=1;
            cloud_kd->points.push_back(Point_kd);
        }
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud_kd);
        int k = 5;
        vector<int> v_id(k);
        vector<float> v_dist(k);

        for(int i=0;i<outData_fusion_2D.size();i++)
        {
            Point_kd.x=outData_fusion_2D[i].x;
            Point_kd.y=outData_fusion_2D[i].y;
            Point_kd.z=1;
            if (kdtree.nearestKSearch(Point_kd, k, v_id, v_dist) > 0)
            {
                if(v_dist[4] < 400 && v_dist[3] < 225 && v_dist[1] < 100)
                {
                    kd_fusion.x=(5*outData_fusion_2D[i].x + 2*outData_fusion_2D[v_id[1]].x + outData_fusion_2D[v_id[2]].x
                                 + outData_fusion_2D[v_id[3]].x + outData_fusion_2D[v_id[4]].x)/10;
                    kd_fusion.y=(5*outData_fusion_2D[i].y + 2*outData_fusion_2D[v_id[1]].y + outData_fusion_2D[v_id[2]].y
                                 + outData_fusion_2D[v_id[3]].y + outData_fusion_2D[v_id[4]].y)/10;
                    outData_fusion_kdtree.push_back(kd_fusion);
                }
            }
        }
    }
}

//离散化坐标点
void final_discrete(Mat& rgb, vector<Point2i>& outData_fusion_kdtree)
{
    if(outData_fusion_kdtree.size() > 0)
    {
        sort(outData_fusion_kdtree.begin(),outData_fusion_kdtree.end(),cmp()); //按y值升序
        Point2i temp=outData_fusion_kdtree[0];
        Point2i final;
        int index=1;
        while(index<outData_fusion_kdtree.size())
        {
            if(outData_fusion_kdtree[index].y==outData_fusion_kdtree[index-1].y)
            {
                index++;
                continue;
            }
            if(temp.y+15 <= outData_fusion_kdtree[index].y)
            {
                final.x=(temp.x+outData_fusion_kdtree[index].x)/2;
                final.y=(temp.y+outData_fusion_kdtree[index].y)/2;
                temp=final;
                circle(rgb, final, 3, Scalar(0, 255, 0));
            }
            else
            {
                index++;
            }
        }
    }
}

//转弯点识别
void curve_detect(Mat& rgb,Mat& depth, vector<Point2i>& outData_fusion_kdtree,curve_point_z_msgs::curve_point_z& curve_pointMsg) {
    if (outData_fusion_kdtree.size() > 20) {
        int path_points_size = outData_fusion_kdtree.size();
        vector<Point2i> outData_fusion_kdtree_before(outData_fusion_kdtree.begin() + path_points_size / 2,
                                           outData_fusion_kdtree.end());                           //路径点后半段
        vector<Point2i> outData_fusion_kdtree_last(outData_fusion_kdtree.begin(),
                                             outData_fusion_kdtree.begin() + path_points_size / 2);//路径点前半段

        Vec4f line_para_last;
        fitLine(outData_fusion_kdtree_last, line_para_last, cv::DIST_L2, 0, 1e-2, 1e-2);
        Vec4f line_para_before;
        fitLine(outData_fusion_kdtree_before, line_para_before, cv::DIST_L2, 0, 1e-2, 1e-2);

        double k_before = line_para_before[1] / line_para_before[0];  //后半段线段拟合而成的直线参数
        Point2i point0_before;
        point0_before.x = (int)line_para_before[2];
        point0_before.y = (int)line_para_before[3];

        double k_last = line_para_last[1] / line_para_last[0];       //前半段线段拟合而成的直线参数
        Point2i point0_last;
        point0_last.x = (int)line_para_last[2];
        point0_last.y = (int)line_para_last[3];

        double PI = 3.1415926;
        int angle_before =  (int) (atan(k_before) * 180 / PI);
        int angle_last   =  (int) (atan(k_last) * 180 / PI);

        if (angle_before < 0)
            angle_before = 180 + angle_before;
        if (angle_last < 0)
            angle_last = 180 + angle_last;

        int last_x=outData_fusion_kdtree_last[0].x+outData_fusion_kdtree_last[1].x+
                   outData_fusion_kdtree_last[2].x+outData_fusion_kdtree_last[3].x+
                   outData_fusion_kdtree_last[4].x;
        int size_before = outData_fusion_kdtree_before.size();
        int before_x=outData_fusion_kdtree_before[size_before-1].x+outData_fusion_kdtree_before[size_before-2].x+
                     outData_fusion_kdtree_before[size_before-3].x+outData_fusion_kdtree_before[size_before-4].x+
                     outData_fusion_kdtree_before[size_before-5].x;

        //两层约束,前后段斜率+两端路径点x方向偏差
        if (abs(angle_before - angle_last) > 40 && before_x -last_x > 300 ) {
            Point2i point_curve;    //转弯点二维坐标
            point_curve.y =(int)
                    ((-k_before * point0_last.y + k_before * k_last * point0_last.x + k_last * point0_before.y -
                     k_before * k_last * point0_before.x) / (k_last - k_before));
            point_curve.x = (int) ((point_curve.y - point0_before.y + k_before * point0_before.x) / k_before);
            circle(rgb, point_curve, 10, Scalar(255, 0, 0), -1);

            float curve_disz = float(depth.at<ushort>(point_curve.y,point_curve.x))/1000;  //转弯点三维坐标
            float curve_disy = (point_curve.y - 240) * curve_disz / 381.62634;
            float curve_disx = (point_curve.x - 320) * curve_disz / 381.62634;

            float z_direction=sqrt(curve_disz*curve_disz-9-9); //距离前进方向的距离
            curve_pointMsg.dis_z = z_direction;
            curve_point_pub.publish(curve_pointMsg);
        }
    }
}

void Affine_trans(Mat& rgb,vector<Point2i>& outData_fusion_kdtree) {
    //仿射变换
    Point2f raw_perceptive[4];
    raw_perceptive[0]=(Point2i(220,209));
    raw_perceptive[1]=(Point2i(420,209));
    raw_perceptive[2]=(Point2i(596,477));
    raw_perceptive[3]=(Point2i(-43,477));

    Point2f target_perceptive[4];
    target_perceptive[0]=(Point2i(220,209));
    target_perceptive[1]=(Point2i(420,209));
    target_perceptive[2]=(Point2i(420,477));
    target_perceptive[3]=(Point2i(220,477));

    Mat transmtx = getPerspectiveTransform(raw_perceptive,target_perceptive);
    cv::warpPerspective(rgb, rgb, transmtx, rgb.size());

    //可做二维平面横向偏差辅助
//    vector<Point2i> affine_border;

//    if(outData_fusion_kdtree.size() > 20)
//    {
//        Mat temp=Mat::ones(3,1,CV_64FC1);
//        Point2i temp_point;
//        for(int i=0;i<outData_fusion_kdtree.size();i++)
//        {
//            temp.at<double>(0,0)=outData_fusion_kdtree[i].x;
//            temp.at<double>(1,0)=outData_fusion_kdtree[i].y;
//            temp.at<double>(2,0)=1;
//            temp = transmtx*temp;
//            temp_point.x =(int) (temp.at<double>(0,0) / temp.at<double>(2,0));
//            temp_point.y =(int) (temp.at<double>(1,0) / temp.at<double>(2,0));
//            affine_border.push_back(temp_point);
//        }
//
//        cv::Vec4f line_para;
//        cv::fitLine(affine_border, line_para, cv::DIST_L2, 0, 1e-2, 1e-2);  //当前二维路径点拟合直线
//
//        double k = line_para[1] / line_para[0];
//        double PI = 3.1415926;
//        int angle =  (int) (atan(k) * 180 / PI);
//        if (angle < 0) angle=angle+180;
//
//        Point2i point0;
//        point0.x= line_para[2];
//        point0.y= line_para[3];
//
//        Point2i point1;
//        point1.x= line_para[2]+line_para[0];
//        point1.y= line_para[3]+line_para[1];
//
//        double angle_pi=angle/180*PI;
//        Point2i rotate_point0;
//        rotate_point0.x = (point0.x - 320)*cos(angle_pi) - (point0.y - 240)*sin(angle_pi) + 320;
//        rotate_point0.y = (point0.x - 320)*sin(angle_pi) + (point0.y - 240)*cos(angle_pi) + 240;
//
//        Point2i rotate_point1;
//        rotate_point1.x = (point1.x - 320)*cos(angle_pi) - (point1.y - 240)*sin(angle_pi) + 320;
//        rotate_point1.y = (point1.x - 320)*sin(angle_pi) + (point1.y - 240)*cos(angle_pi) + 240;
//
//        double a = (rotate_point0.x - point0.x) * (point1.y - point0.y);
//        double b = (rotate_point0.y - point0.y) * (point0.x - point1.x);
//        double c = a + b;
//
//        c*=c;//平方(pow(c,2)貌似在这里更加麻烦)
//        a=pow(point1.y-point0.y,2);//分母左半部分
//        b=pow(point0.x-point1.x,2);//分母右半部分
//        c/=(a+b);//分子分母相除
//        double res=sqrt(c);//开方,可做二维平面横向偏差
//    }
}


void border_offset(Mat& rgb,vector<Point2i>& pointimg,vector<Point3f>& pointimg_3d, border_msgs::border& borderMsg,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudin) {
    if (pointimg.size() > 20) {

        //标准分界线
        float Standard_2Dline_fun_0=0.176318, Standard_2Dline_fun_1=0.984333,Standard_2Dline_fun_2= 379.282,Standard_2Dline_fun_3= 420.846; //正常行走时,标准分界线位置
        Point2i point0_Standard;
        point0_Standard.x = (int) Standard_2Dline_fun_2;
        point0_Standard.y = (int) Standard_2Dline_fun_3;
        double k_Standard = Standard_2Dline_fun_1 / Standard_2Dline_fun_0;
        Point2i pStart_Standard;
        Point2i pEnd_Standard;
        pStart_Standard.x = (int) ((480 - point0_Standard.y + k_Standard * point0_Standard.x) / k_Standard);
        pStart_Standard.y = 480;
        pEnd_Standard.x = (int) ((380 - point0_Standard.y + k_Standard * point0_Standard.x) / k_Standard);
        pEnd_Standard.y = 380;
        Point2i vec_bound_standard;//标准分界线方向向量
        vec_bound_standard.x=pEnd_Standard.x-pStart_Standard.x;
        vec_bound_standard.y=pEnd_Standard.y-pStart_Standard.y;

        //Scalar lineColor_Standard(0, 0, 255);
        //drawArrow(rgb, pStart_Standard, pEnd_Standard, 10, 45, lineColor_Standard);

        Vec4f line_para;
        fitLine(pointimg, line_para, cv::DIST_WELSCH, 0, 1e-2, 1e-2);
        double k_cur=line_para[1]/line_para[0];
        Point2i pStart_cur;
        Point2i pEnd_cur;
        pStart_cur.x = (int) ((480 - line_para[3] + k_cur * line_para[2]) / k_cur);
        pStart_cur.y = 480;
        pEnd_cur.x = (int) ((380 - line_para[3] + k_cur * line_para[2]) / k_cur);
        pEnd_cur.y = 380;
        Point2i vec_bound_cur;//当前分界线方向向量
        vec_bound_cur.x=pEnd_cur.x-pStart_cur.x;
        vec_bound_cur.y=pEnd_cur.y-pStart_cur.y;

        //判断往左偏or右偏
        bool left=(vec_bound_standard.x*vec_bound_cur.y-vec_bound_cur.x*vec_bound_standard.y)>0; //left==true,则往左偏,否则往右(右手螺旋定则，叉乘)

        Vec6f line_para_3d; //三维空间点直线拟合
        fitLine(pointimg_3d, line_para_3d, cv::DIST_WELSCH, 0, 1e-2, 1e-2);
        //cout<<line_para_3d[0]<<" "<<line_para_3d[1]<<" "<<line_para_3d[2]<<" "<<line_para_3d[3]<<" "<<line_para_3d[4]<<" "<<line_para_3d[5]<<endl;

        double Standard_line_fun_0 = 0.00251082, Standard_line_fun_1 =-0.127729, Standard_line_fun_2= 0.991806,  //分界线标准线
               Standard_line_fun_3= 0.451316,Standard_line_fun_4= 1.08251, Standard_line_fun_5=5.69316;

        if(line_para_3d[2]<0)  //调整矢量方向一致,避免余弦角计算1变179问题
        {
            line_para_3d[0]=-line_para_3d[0];
            line_para_3d[1]=-line_para_3d[1];
            line_para_3d[2]=-line_para_3d[2];
        }

        //计算分界线夹角
        double cosvalue = line_para_3d[0] * Standard_line_fun_0 +  line_para_3d[1] * Standard_line_fun_1 + line_para_3d[2] * Standard_line_fun_2;
        double arc_cosvalue_inangle = acos(cosvalue);
        if(!left) arc_cosvalue_inangle=-arc_cosvalue_inangle;

        double A_standard_plane=0.04488149,B_standard_plane=-0.986913152,C_standard_plane=-0.153237743,D_standard_plane=1.906296245;  //该平面为偏差基准面

        Point3f begin_standard = Point3f(Standard_line_fun_3, Standard_line_fun_4, Standard_line_fun_5);
        Point3f end_standard = Point3f(Standard_line_fun_3+Standard_line_fun_0, Standard_line_fun_4+Standard_line_fun_1, Standard_line_fun_5+Standard_line_fun_2);

        int distance=0;
        if(abs(arc_cosvalue_inangle*180/3.1415926)<2.5)
        {
            vector<float> dis;
            dis_cal(dis,pointimg_3d,begin_standard,end_standard);
            sort(dis.begin(), dis.end());
            float dis_reslut = max_num(dis);
            distance=dis_reslut*100;
            if(line_para_3d[3]<end_standard.x) distance=-distance;
        }
        else
        {
            vector<Point3f> rotate_point;
            for(int i=0;i<pointimg_3d.size();i++)
            {
                Point3f tmp_rotate_point = random_rotateObject( pointimg_3d[i].x,pointimg_3d[i].y,pointimg_3d[i].z, 0, 0, 0,
                                                                A_standard_plane,B_standard_plane,C_standard_plane+0.03,
                                                                arc_cosvalue_inangle);
                rotate_point.push_back(tmp_rotate_point);
            }

            pcl::PointXYZRGB tmp;
            for(int i=0;i<rotate_point.size();i++)
            {
                tmp.x=rotate_point[i].x;
                tmp.y=rotate_point[i].y;
                tmp.z=rotate_point[i].z;
                tmp.g=255;
                cloudin->points.push_back(tmp);
            }

            vector<float> dis;
            dis_cal(dis,rotate_point,begin_standard,end_standard);
            sort(dis.begin(), dis.end());
            float dis_reslut = max_num(dis);
            distance=dis_reslut*100;
            if(line_para_3d[3]<end_standard.x) distance=-distance;
        }

        Point2i point0;
        point0.x = line_para[2];
        point0.y = line_para[3];
        double k = line_para[1] / line_para[0];
        Point2i pStart;
        Point2i pEnd;
        pStart.x = (480 - point0.y + k * point0.x) / k;
        pStart.y = 480;
        pEnd.x = (380 - point0.y + k * point0.x) / k;
        pEnd.y = 380;


        Scalar lineColor(0, 255, 0);
        //drawArrow(rgb, pStart, pEnd, 10, 45, lineColor);
        int zs = arc_cosvalue_inangle*180/3.1415926;
        int xs = abs(int((arc_cosvalue_inangle*180/3.1415926 - zs) * 10));//保留一位小数
        string angle = to_string(zs) + '.' + to_string(xs);
        borderMsg.angle = angle;
        borderMsg.dis = to_string(distance);

        if(arc_cosvalue_inangle<40 && arc_cosvalue_inangle>-40 && distance<70 && distance>-70)
        {
            float value_inangle = arc_cosvalue_inangle * 180 / 3.1415926;
            int xs_value_inangle = (value_inangle - int(value_inangle)) * 10;//保留一位小数
            string angle =
                    "Ang: " + std::to_string(int(value_inangle)) + '.' + std::to_string(abs(xs_value_inangle))+"deg";
            cv::putText(rgb, angle, Point2i(400, 50), cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255, 0, 0), 4);

            int zs_distance = distance;
            string dis = "Dis: " + std::to_string(zs_distance)+"cm";
            cv::putText(rgb, dis, Point2i(400, 100), cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255, 0, 0), 5);
        }
        border_param.publish(borderMsg);
    }
}

//绘制方向箭头
void drawArrow(cv::Mat& img, cv::Point pStart, cv::Point pEnd, int len, int alpha,
     cv::Scalar& color, int thickness, int lineType)
 {
     const double PI = 3.1415926;
     Point arrow;
    //计算 θ 角（最简单的一种情况在下面图示中已经展示，关键在于 atan2 函数，详情见下面）
     double angle = atan2((double)(pStart.y - pEnd.y), (double)(pStart.x - pEnd.x));
     line(img, pStart, pEnd, color, thickness, lineType);
     //计算箭角边的另一端的端点位置（上面的还是下面的要看箭头的指向，也就是pStart和pEnd的位置）
     arrow.x = pEnd.x + len * cos(angle + PI * alpha / 180);
     arrow.y = pEnd.y + len * sin(angle + PI * alpha / 180);
     line(img, pEnd, arrow, color, thickness, lineType);
     arrow.x = pEnd.x + len * cos(angle - PI * alpha / 180);
     arrow.y = pEnd.y + len * sin(angle - PI * alpha / 180);
     line(img, pEnd, arrow, color, thickness, lineType);
 }

 //计算众数,出现次数最多的数
float max_num(vector<float> height)
{
    int count =1;
    int sum_position=0;
    vector<int> element_num;
    for(int index=0;index<height.size()-1;index++)
    {
        if(height[index]==height[index+1])
        {
            count++;
            if(index+1==height.size()-1)
                element_num.push_back(count);
        }
        if(height[index]!=height[index+1])
        {
            element_num.push_back(count);
            count=1;
        }
    }
    int Position = max_element(element_num.begin(),element_num.end()) - element_num.begin();

    for(int i=0;i<=Position;i++)
    {

        sum_position+=element_num[i];
    }
    return height[sum_position-1];
}

//旋转当前分界线上一点, 使当前分界线平行于正确行驶时的分界线,进而求得横向误差
Point3f random_rotateObject(double x, double y, double z, double a, double b, double c,double a0,double b0,double c0,double theta)
{
    //(a0,b0,c0)轴线的始点
    //(a,b,c)轴线L末点
    //(x,y,z)初始点位置
    //点Point3f 绕L旋转后的位置
    double u = a0-a;
    double v = b0-b;
    double w = c0-c;  //方向向量(u,v,w)需为单位向量！！！
    Point3f p;
    double SinA = sin(theta);
    double CosA = cos(theta);

    double uu=u*u;
    double vv=v*v;
    double ww=w*w;
    double uv=u*v;
    double uw=u*w;
    double vw=v*w;

    float t00 = uu + (vv + ww) * CosA;
    float t10 = uv * (1 - CosA) + w * SinA;
    float t20 = uw * (1 - CosA) - v * SinA;
    float t30 = 0;

    float t01 = uv * (1 - CosA) - w * SinA;
    float t11 = vv + (uu + ww) * CosA;
    float t21 = vw * (1 - CosA) + u * SinA;
    float t31 = 0;

    float t02 = uw * (1 - CosA) + v * SinA;
    float t12 = vw * (1 - CosA) - u * SinA;
    float t22 = ww + (uu + vv) * CosA;
    float t32 = 0;

    float t03 = (a * (vv + ww) - u * (b * v + c * w)) * (1 - CosA) + (b * w - c * v) * SinA;
    float t13 = (b * (uu + ww) - v * (a * u + c * w)) * (1 - CosA) + (c * u - a * w) * SinA;
    float t23 = (c * (uu + vv) - w * (a * u + b * v)) * (1 - CosA) + (a * v - b * u) * SinA;
    float t33 = 1;

    p.x = t00 * x + t01 * y + t02 * z + t03;
    p.y = t10 * x + t11 * y + t12 * z + t13;
    p.z = t20 * x + t21 * y + t22 * z + t23;
    return p;
}


void dis_cal(vector<float>& dis,vector<Point3f>& pointdepthimg_3d,Point3f begin_standard, Point3f end_standard)
{
    Point3f s;Point3f a=begin_standard; Point3f b=end_standard;
    for(int i=0;i<pointdepthimg_3d.size();i++)
    {
        s=pointdepthimg_3d[i];
        double ab = sqrt(pow((a.x - b.x), 2.0) + pow((a.y - b.y), 2.0) + pow((a.z - b.z), 2.0));
        double as = sqrt(pow((a.x - s.x), 2.0) + pow((a.y - s.y), 2.0) + pow((a.z - s.z), 2.0));
        double bs = sqrt(pow((s.x - b.x), 2.0) + pow((s.y - b.y), 2.0) + pow((s.z - b.z), 2.0));
        double cos_A = (pow(as, 2.0) + pow(ab, 2.0) - pow(bs, 2.0)) / (2 * ab*as);
        double sin_A = sqrt(1 - pow(cos_A, 2.0));
        dis.push_back(as*sin_A);
    }
}

void height_detection(Mat& rgb,Mat& depth,height_msgs::height& heightMsg)
{
    vector<vector<Point2i> > mask_area;
    vector<Point2i> mask_points;
    mask_points.push_back(Point2i(450,477));
    mask_points.push_back(Point2i(407,309));
    mask_points.push_back(Point2i(439,306));
    mask_points.push_back(Point2i(516,477));
    mask_area.push_back(mask_points);

    cv::Mat mask, dst_mask;
    rgb.copyTo(mask);
    mask.setTo(cv::Scalar::all(0));
    fillPoly(mask, mask_area, Scalar(255, 255, 255));
    rgb.copyTo(dst_mask, mask);
    vector<float> height;
    for (int row = 0; row < rgb.rows; row++)
        for (int col = 0; col < rgb.cols; col++) {
            if (dst_mask.at<Vec3b>(row, col)[0] != 0) {
                float z = float(depth.at<ushort>(row,col))/1000;
                float y = (row - 232.171) * z / 615.312;
                float x = (col - 323.844) * z / 615.372;

                float distanceheight =
                        abs(coeff_uncut_height_mean[0] * x + coeff_uncut_height_mean[1] * y + coeff_uncut_height_mean[2] * z + coeff_uncut_height_mean[3]) * 100;
                distanceheight = floor(distanceheight);
                height.push_back(distanceheight);
            }
        }

    sort(height.begin(), height.end());
    int temp_reslut = max_num(height);
    Estimated_height = Estimated_height==0 ? temp_reslut:0.875*Estimated_height+0.125*temp_reslut;

    heightMsg.height = Estimated_height;
    height_pub.publish(heightMsg);
}
