//ROS
#include <ros/ros.h>
#include <ros/package.h>
//SDK
#include "CameraApi.h"
#include "CameraDefine.h"
//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//normal
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>
#include <thread>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#define TRUE 1
#define FALSE 0
#define BEIJINGTIME 8
#define DAY        (60*60*24)
#define YEARFIRST  2001
#define YEARSTART  (365*(YEARFIRST-1970) + 8)
#define YEAR400    (365*4*100 + (4*(100/4 - 1) + 1))
#define YEAR100    (365*100 + (100/4 - 1))
#define YEAR004    (365*4 + 1)
#define YEAR001    365


using namespace std;
using namespace cv;
unsigned char           * g_pRgbBuffer[4];
bool                    printmode = false;

void readInstric(string& calib,
                 vector<Mat>& intrinsic_matrix,
                 vector<Mat>& distortion_coeffs);
class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher cam_pub;
    //string topic_name_;
public:
  ImageConverter(string topic):
  it_(nh_)
  {
    //topic_name_ = topic;
    cam_pub = it_.advertise(topic,1);
  }

  ~ImageConverter()
  {
  }
  void pub(sensor_msgs::ImagePtr msg)
  {
    cam_pub.publish(msg);
    ros::spinOnce();
  }

};

string sec_to_date(struct timeval &tv)
{
  //struct timeval tv;
    string filename;
    char temp[50];
    long sec = 0, usec = 0;
    int yy = 0, mm = 0, dd = 0, hh = 0, mi = 0, ss = 0, ms = 0;
    int ad = 0;
    int y400 = 0, y100 = 0, y004 = 0, y001 = 0;
    int m[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    int i;
    //memset(&tv, 0, sizeof(timeval));
    //gettimeofday(&tv, NULL);
    sec = tv.tv_sec;
    usec = tv.tv_usec;
    sec = sec + (60*60)*BEIJINGTIME;
    ad = sec/DAY;
    ad = ad - YEARSTART;
    y400 = ad/YEAR400;
    y100 = (ad - y400*YEAR400)/YEAR100;
    y004 = (ad - y400*YEAR400 - y100*YEAR100)/YEAR004;
    y001 = (ad - y400*YEAR400 - y100*YEAR100 - y004*YEAR004)/YEAR001;
    yy = y400*4*100 + y100*100 + y004*4 + y001*1 + YEARFIRST;
    dd = (ad - y400*YEAR400 - y100*YEAR100 - y004*YEAR004)%YEAR001;
    //月 日
    if(0 == yy%1000)
    {
        if(0 == (yy/1000)%4)
        {
            m[1] = 29;
        }
    }
    else
    {
        if(0 == yy%4)
        {
            m[1] = 29;
        }
    }
    for(i = 1; i <= 12; i++)
    {
        if(dd - m[i] < 0)
        {
            break;
        }
        else
        {
            dd = dd -m[i];
        }
    }
    mm = i;
    //小时
    hh = sec/(60*60)%24;
    //分
    mi = sec/60 - sec/(60*60)*60;
    //秒
    ss = sec - sec/60*60;
    ms = usec/1000;
    sprintf(temp, "%d%02d%02d%02d%02d%02d%03d",yy, mm, dd, hh, mi, ss, ms);
    //cout<<temp<<endl;
    filename = temp;
    return filename;
}

void multithreads(string filename, int hCamera, tSdkFrameHead sFrameInfo,
                  BYTE*	pbyBuffer,unsigned char *outBuffer, ImageConverter ic, Mat& intrinsic_matrix, Mat& distortion_coeffs)
{
  struct timeval start;
  struct timeval end;
  gettimeofday(&start,NULL);
  if (printmode == 1)
    cout<<sec_to_date(start)<<endl;

  CameraClearBuffer(hCamera);
  CameraSoftTrigger(hCamera);
  if(CameraGetImageBuffer(hCamera,&sFrameInfo,&pbyBuffer,2000) == CAMERA_STATUS_SUCCESS)
  {
      /*
      //if save raw data, please uncommit the following para.
      //保存图片 filename: xxx.raw
      for(int i=;i<4;i++)
      {
      filename.pop_back();
      }
      filename += ".raw";
       CameraSaveImage(hCamera, (char *)filename.data(), pbyBuffer, &sFrameInfo, FILE_RAW, 100);
      //释放，和CameraGetImageBuffer 配套使用
       CameraReleaseImageBuffer(hCamera,pbyBuffer);
    */

    // if not save raw data, please uncommit the following para.
     CameraImageProcess(hCamera, pbyBuffer, outBuffer,&sFrameInfo);
    //释放，和CameraGetImageBuffer 配套使用
     CameraReleaseImageBuffer(hCamera,pbyBuffer);
/*
    //保存图片
     CameraSaveImage(hCamera, (char *)filename.data(), outBuffer, &sFrameInfo, FILE_BMP, 100);
*/
    //publish to ros topic
     Mat curr (960,1280,CV_8UC3,outBuffer);

     Mat curr_distort = curr.clone();
     cv::undistort(curr, curr_distort, intrinsic_matrix, distortion_coeffs);

     sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),"rgb8", curr_distort).toImageMsg();
     ic.pub(msg);

     if (printmode == 0)
          cout<<"pub one frame from camera >>>>" <<endl;
  }
  else
  {
     printf("WARNING: timeout, some thing happened to at least one camera .\n");
  }
  gettimeofday(&end,NULL);
  if (printmode == 1) cout<<sec_to_date(end)<<endl;
}

int main(int argc, char**argv)
{
    // Only 3 cameras
    // Mark the topic as label on real camera(02, 03, 04)
    int camera_count = 3;
    ros::init(argc,argv,"multicamera_realtime");
    ImageConverter ic0("camera/image_2");
    ImageConverter ic1("camera/image_3");
    ImageConverter ic2("camera/image_4");
//    ImageConverter ic3("camera/image_3");     // Only 3 cameras

    string filenum = "0_0";

    int                     iCameraCounts = 12;
    int                     iStatus[camera_count]={-1,-1,-1};
    tSdkCameraDevInfo       tCameraEnumList[camera_count];
    int                     hCamera[camera_count];

    tSdkCameraCapbility     tCapability[camera_count];
    tSdkFrameHead           sFrameInfo[camera_count];
    BYTE*			              pbyBuffer[camera_count];
    tSdkImageResolution     sImageSize[camera_count];

    string datadirs[camera_count];
    // todo need change with IPC
    string datadir = "/home/yangzt/catkin_ws/data/camera/";
    for (int i=0; i<camera_count;i++)
    {
      datadirs[i] = datadir + filenum + "/" + to_string(i) + "/";
    }

    //sdk初始化  0 English 1中文
    CameraSdkInit(1);
    //change the data savedir
    //CameraSetDataDirectory("/home/dosu/dosuss_ws_test/src/multicamera/data");
    //枚举设备，并建立设备列表
    CameraEnumerateDevice(tCameraEnumList,&iCameraCounts);

    //TODO 固定相机昵称，并按照昵称顺序排列
    // Camera ID set as 2 3 4


    // read the in matrix of cameras to distort
    // todo: change as IPC
    string camera_cali = "/home/yangzt/catkin_ws/src/multicamera/calib.txt";
    vector<Mat> intrinsic_matrix(4);
    vector<Mat> distortion_coeffs(4);
    readInstric(camera_cali, intrinsic_matrix, distortion_coeffs);

    printf("iCameraCounts =%d  \n",iCameraCounts);

    if(iCameraCounts==0){
        return -1;
    }

    for(int i=0;i<iCameraCounts;i++)
    {
        printf("num =%d %s  %s \n",i,tCameraEnumList[i].acProductName,tCameraEnumList[i].acFriendlyName);
    }

    for (int i=0;i<iCameraCounts;i++){
      //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
//      iStatus[i] = CameraInit(&tCameraEnumList[i],-1,-1,&hCamera[i]);

      // init camera as Name 02 03 04
        string camNameStr = "Camera" + to_string(i + 2);
        char ptrCamName[20];
        strcpy(ptrCamName, camNameStr.c_str());

        iStatus[i] = CameraInitEx2(ptrCamName, &hCamera[i]);
      
      printf("CameraInit iStatus =%d \n",iStatus[i]);
      //初始化失败
      if(iStatus[i]!=CAMERA_STATUS_SUCCESS){
        return -1;
      }

      //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
      CameraGetCapability(hCamera[i],&tCapability[i]);
      printf("CameraGetCapability \n");
      //data byte size for one frame
      g_pRgbBuffer[i] = (unsigned char*)malloc(tCapability[i].sResolutionRange.iHeightMax*tCapability[i].sResolutionRange.iWidthMax*3);
      //g_readBuf = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);

      //set soft trigger mode
      //each trigger refer to one frame
      CameraSetTriggerMode(hCamera[i],1);
      CameraSetTriggerCount(hCamera[i],1);
      //manual exposure, set time to 30ms.
      CameraSetAeState(hCamera[i], TRUE);

      /*让SDK进入工作模式，开始接收来自相机发送的图像
      数据。如果当前相机是触发模式，则需要接收到
      触发帧以后才会更新图像。    */
      //CameraPlay(hCamera[i]);

    //  printf("CameraPlay \n");


#if  0
   memset(&sImageSize[i],0,sizeof(tSdkImageResolution));
   sImageSize[i].iIndex=0xff;
   sImageSize[i].iHOffsetFOV=0;
   sImageSize[i].iVOffsetFOV=0;
   sImageSize[i].iWidthFOV=800;
   sImageSize[i].iHeightFOV=600;
   sImageSize[i].iWidth=800;
   sImageSize[i].iHeight=600;


   CameraSetImageResolution(hCamera[i],&sImageSize[i]);
#else
   CameraSetImageResolution(hCamera[i],&tCapability[i].pImageSizeDesc[0]);
#endif

/*
   设置图像处理的输出格式，彩色黑白都支持RGB24位
*/
     if(tCapability[i].sIspCapacity.bMonoSensor){
        CameraSetIspOutFormat(hCamera[i],CAMERA_MEDIA_TYPE_MONO8);
     }else{
        CameraSetIspOutFormat(hCamera[i],CAMERA_MEDIA_TYPE_RGB8);
     }
    // printf("CameraSetIspOutFormat \n");
     sleep(1);
    }
    for (int i=0;i<iCameraCounts;i++)
    {
      CameraPlay(hCamera[i]);
    }
    int count=0;
    string filename[4];
    struct timeval time_now;
    struct timeval time_new;
    double diff;
    //loop start.
    while(ros::ok())
    {
      gettimeofday(&time_now,NULL);
      //multi thread request
      for(int i=0;i<iCameraCounts;i++){
        filename[i] = datadirs[i] + sec_to_date(time_now) + ".bmp";
      }
      {
        thread grasp0(multithreads, filename[0],hCamera[0],sFrameInfo[0],pbyBuffer[0], g_pRgbBuffer[0],ic0, ref(intrinsic_matrix[1]),ref(distortion_coeffs[1]));
        thread grasp1(multithreads, filename[1],hCamera[1],sFrameInfo[1],pbyBuffer[1], g_pRgbBuffer[1],ic1, ref(intrinsic_matrix[2]),ref(distortion_coeffs[2]));
        thread grasp2(multithreads, filename[2],hCamera[2],sFrameInfo[2],pbyBuffer[2], g_pRgbBuffer[2],ic2, ref(intrinsic_matrix[3]),ref(distortion_coeffs[3]));
//        thread grasp3(multithreads, filename[3],hCamera[3],sFrameInfo[3],pbyBuffer[3], g_pRgbBuffer[3],ic3);

        grasp0.join();
        grasp1.join();
        grasp2.join();
//        grasp3.join();
      }

      count++;
      gettimeofday(&time_new,NULL);
      diff = time_new.tv_sec-time_now.tv_sec+ (time_new.tv_usec-time_now.tv_usec)/1000000.0;
      diff = diff *1000; //ms

      if (diff < 100) //10HZ
      {
        usleep((100-(int)diff) * 1000);
        if (printmode == 1)
        printf("RUNNING: in 10 HZ. \n");
      }
      else if (diff <200) //5hz
      {
        usleep((200-(int)diff) * 1000);
        if (printmode == 1)
        printf("RUNNING: in 5 HZ. \n");
      }
      else
      {
        if (printmode ==1)
        printf("WARNING: the frequency is too low, lower than 5HZ for each cam. \n");
        //  exit(1);
      }
      if (printmode ==1)
      cout<<"time used in one shot(four cameras):"<<diff<<" ms"<<endl; //ms

      ros::spinOnce();
    }

    for(int i=0;i<iCameraCounts;i++)
    {
      CameraUnInit(hCamera[i]);
      free(g_pRgbBuffer[i]);
    }

    printf("end  \n");
    ros::spin();

    return 0;
}
void readInstric(string& calib_dir, vector<Mat>& intrinsic_matrix, vector<Mat>& distortion_coeffs){
    assert(intrinsic_matrix.size()==4 && distortion_coeffs.size()==4);
    ifstream in(calib_dir);
//    in.open(calib_dir);
    if (!in.good())
    {
        printf(" ...calib.txt not found. Cannot operate without calib.txt, shutting down.\n");
        exit(1);
    }
    double fx,fy,cx,cy,k1,k2,k3,r1,r2;
    for (int i = 0; i < 4; ++i) //data文件只有4行
    {
        in >> fx >> fy >> cx >> cy >> k1>> k2 >> k3 >> r1 >> r2;
        intrinsic_matrix[i] = Mat::zeros(Size(3,3),CV_64FC1);
        distortion_coeffs[i] = Mat::zeros(Size(1,5),CV_64FC1);
        intrinsic_matrix[i].at<double>(0, 0) = fx;
        intrinsic_matrix[i].at<double>(0, 2) = cx;
        intrinsic_matrix[i].at<double>(1, 1) = fy;
        intrinsic_matrix[i].at<double>(1, 2) = cy;
        intrinsic_matrix[i].at<double>(2, 2) = 1.0;
        distortion_coeffs[i].at<double>(0, 0) = k1;
        distortion_coeffs[i].at<double>(0, 1) = k2;
        distortion_coeffs[i].at<double>(0, 2) = r1;
        distortion_coeffs[i].at<double>(0, 3) = r2;
        distortion_coeffs[i].at<double>(0, 4) = k3;
        // cout << a << " " << b << endl;
    }

}
