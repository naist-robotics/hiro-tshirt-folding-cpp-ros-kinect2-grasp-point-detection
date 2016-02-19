#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <kinect2_bridge/kinect2_definitions.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "TcpLib.hpp"

using namespace std;

int
main(int argc, char *argv[])
{
  cv::Mat src_img = cv::imread("addDepth.png", 1);
  if(src_img.empty()) return -1;


  cv::namedWindow("image1", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
  // ウィンドウ名でウィンドウを指定して，そこに画像を描画
  cv::imshow("image1", src_img);
   
  // デフォルトのプロパティで表示
  //cv::imshow("image2", src_img);

  
  TcpServer server;
  int PORT = 50001;
  server.Init(PORT);

    // キー入力を（無限に）待つ
    //cv::waitKey(0);
  
  while(true){
    vector<char> ReadData;
    char nSend=server.Read(ReadData);  //受信
    cout << "out:" << nSend << endl;
    cv::waitKey(0);
    /*
    vector<char> SendData;
    SendData.push_back(12);//0x01というデータを送る 
    int nWrite=server.Write(SendData);//送信
    */
    
  }
  
}
