//ROS
#include <ros/ros.h>
#include <ros/package.h>
//sys
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>
#include <thread>
//boost
#include <boost/program_options.hpp>
//camera SDK
#include "CameraApi.h"
#include "CameraDefine.h"

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
//***if use commad line input please uncommit it***
namespace bpo = boost::program_options;

unsigned char           * g_pRgbBuffer[4];
bool                    printmode = true;
//#define DATADIR   "/home/dosu/dosuss_ws_test/src/multicamera/data/"

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


//multi threads to grasp frames from cams in same trigger.
void multithreads(string filename, int hCamera, tSdkFrameHead sFrameInfo,
                  BYTE*	pbyBuffer,unsigned char *outBuffer)
{
  //filename = datadirs[i] + sec_to_date(time_now) + ".bmp";
//  filename = datadir + to_string(count)+".bmp";
  struct timeval start;
  struct timeval end;
  gettimeofday(&start,NULL);
  if (printmode == 1)
    cout<<sec_to_date(start)<<endl;

  CameraClearBuffer(hCamera);
  CameraSoftTrigger(hCamera);
  if(CameraGetImageBuffer(hCamera,&sFrameInfo,&pbyBuffer,2000) == CAMERA_STATUS_SUCCESS)
  {
       CameraImageProcess(hCamera, pbyBuffer, outBuffer,&sFrameInfo);
      // //释放，和CameraGetImageBuffer 配套使用
       CameraReleaseImageBuffer(hCamera,pbyBuffer);
      // //保存图片
       CameraSaveImage(hCamera, (char *)filename.data(), outBuffer, &sFrameInfo, FILE_BMP, 100);
    //   CameraSaveImage(hCamera, (char *)filename.data(), g_pRgbBuffer, &sFrameInfo, FILE_BMP, 100);
       if (printmode == 1)
          cout<<"save one frame from camera >>>>" <<endl;
  }else{
     printf("WARNING: timeout, some thing happened to at least one camera .\n");
  }
  gettimeofday(&end,NULL);
  if (printmode == 1)
    cout<<sec_to_date(end)<<endl;
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"multicamera");
    string output_dir = "/home/dosu/dosuss_ws_test/src/multicamera/data/2/";   //TODO  change to ros parameter input method.
    bool printmode =false;
    //use boost program_options to input some params. if useful, please uncommit it.
    //command input
    bpo::options_description desc("Program options");
    desc.add_options()
    //Options
    // ("laser_type", bpo::value<int>(), "the laser_type, Usage: --laser_type sick/velodyne16/velodyne64")
    ("print_mode", bpo::value<bool>(&printmode) -> default_value(false),
                  "whether print processing data or not, Usage: --print_mode true/false")
    ("output_dir",bpo::value<string>(&output_dir),
                  "output directory, Usage: --output_dir xxx/xxx/");
    // Parse the command line
    bpo::variables_map vm;
    bpo::store(bpo::parse_command_line(argc, argv, desc), vm);

    // option help
    if (vm.count("help"))
    {
      cout << desc << "\n";
      return false;
    }
    // Process options.
    bpo::notify(vm);


    int                     iCameraCounts = 16;
    int                     iStatus[4]={-1,-1,-1,-1};
    tSdkCameraDevInfo       tCameraEnumList[4];
    int                     hCamera[4];
  //  int                     hCamera1;
    tSdkCameraCapbility     tCapability[4];
    tSdkFrameHead           sFrameInfo[4];
    BYTE*			              pbyBuffer[4];
    tSdkImageResolution     sImageSize[4];
//    printmode = false;
    //datadir
    string datadirs[4];
    for (int i=0; i<4;i++)
    {
      datadirs[i] = output_dir + to_string(i) + "/";
    }

    //sdk初始化  0 English 1中文
    CameraSdkInit(1);
    //change the data savedir
    //CameraSetDataDirectory("/home/dosu/dosuss_ws_test/src/multicamera/data");
    //枚举设备，并建立设备列表
    CameraEnumerateDevice(tCameraEnumList,&iCameraCounts);


	  printf("iCameraCounts =%d  \n",iCameraCounts);

    if(iCameraCounts==0){
        return -1;
    }

    for(int i=0;i<iCameraCounts;i++)
    {
        printf("num =%d %s  %s \n",i,tCameraEnumList[i].acProductName,tCameraEnumList[i].acFriendlyName);
    }
    // 按照相机昵称顺序排列重新　tCameraRaEnumList　TODO

    for (int i=0;i<iCameraCounts;i++)
    {
      //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
      iStatus[i] = CameraInit(&tCameraEnumList[i],-1,-1,&hCamera[i]);

      printf("CameraInit iStatus =%d \n",iStatus[i]);
      //初始化失败
      if(iStatus[i]!=CAMERA_STATUS_SUCCESS)
      {
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

      //manual/auto exposure
      //when manual, uncommit line-228 and set time to 20ms.
      CameraSetAeState(hCamera[i], TRUE);
      //CameraSetExposureTime(hCamera[i],20*1000);

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
      for(int i=0;i<iCameraCounts;i++)
      {
        filename[i] = datadirs[i] + sec_to_date(time_now)+".bmp";
      }
      {
        thread grasp0(multithreads, filename[0],hCamera[0],sFrameInfo[0],pbyBuffer[0], g_pRgbBuffer[0]);
        thread grasp1(multithreads, filename[1],hCamera[1],sFrameInfo[1],pbyBuffer[1], g_pRgbBuffer[1]);
        thread grasp2(multithreads, filename[2],hCamera[2],sFrameInfo[2],pbyBuffer[2], g_pRgbBuffer[2]);
        thread grasp3(multithreads, filename[3],hCamera[3],sFrameInfo[3],pbyBuffer[3], g_pRgbBuffer[3]);

        grasp0.join();
        grasp1.join();
        grasp2.join();
        grasp3.join();
      }

      count++;
      gettimeofday(&time_new,NULL);
      diff = time_new.tv_sec-time_now.tv_sec+ (time_new.tv_usec-time_now.tv_usec)/1000000.0;
      diff = diff *1000; //ms

      if( diff< 50)  //check whether is in 20HZ ?
      {
        usleep((50-(int)diff) * 1000); //in us
        printf("RUNNING: in 20HZ. \n");
      }
      else if (diff < 100) //10HZ
      {
        usleep((100-(int)diff) * 1000);
        printf("RUNNING: in 10 HZ. \n");
      }
      else if (diff <200) //5hz
      {
        usleep((200-(int)diff) * 1000);
        printf("RUNNING: in 5 HZ. \n");
      }
      else
      {
        printf("WARNING: the frequency is too low, lower than 5HZ for each cam. \n");
        //  exit(1);
      }
      cout<<"time used in one shot(four cameras) is:"<<diff<<" ms"<<endl; //ms

      ros::spinOnce();
    }

    for(int i=0;i<iCameraCounts;i++)
    {
      CameraUnInit(hCamera[i]);
      free(g_pRgbBuffer[i]);
    }




    printf("end  \n");

    return 0;
}
