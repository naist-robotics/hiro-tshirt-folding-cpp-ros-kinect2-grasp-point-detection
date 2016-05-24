/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author: Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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

#include "TcpLib.hpp"

#define IPADDRESS "163.221.44.226"
#define PORTNUM 3794

//######Kinect Parameters##################
#define au 365.97	
#define av 357.0580	
#define u0 247	// Optical centre u coordinate
#define v0 207	// Optical center v coordinate  

using namespace std;
using namespace cv;

TcpClient client;//ソケット通信のクライアント「client」を作成

// Limits field of view in depth. Check.
static const float DEPTHMAX = 740;//深度画像を見やすくする　間隔は60
static const float DEPTHMIN = 680;//750, 690

class Receiver
{
public:
  enum Mode
  {
    IMAGE = 0,
    CLOUD,
    BOTH
  };

private:
  std::mutex lock;

  const std::string topicColor, topicDepth;
  const bool useExact, useCompressed;

  bool updateImage, updateCloud;
  bool save;
  bool running;
  size_t frame;
  const size_t queueSize;

  cv::Mat color, depth;
  cv::Mat cameraMatrixColor, cameraMatrixDepth;
  cv::Mat lookupX, lookupY;

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateSyncPolicy;

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner;
  image_transport::ImageTransport it;
  image_transport::SubscriberFilter *subImageColor, *subImageDepth;
  message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor, *subCameraInfoDepth;

  message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
  message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate;

  std::thread imageViewerThread;
  Mode mode;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  pcl::PCDWriter writer;
  std::ostringstream oss;
  std::vector<int> params;

public:
  Receiver(const std::string &topicColor, const std::string &topicDepth, const bool useExact, const bool useCompressed)
    : topicColor(topicColor), topicDepth(topicDepth), useExact(useExact), useCompressed(useCompressed),
      updateImage(false), updateCloud(false), save(false), running(false), frame(0), queueSize(5),
      nh("~"), spinner(0), it(nh), mode(CLOUD)
  {
    cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
    cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(100);
    params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    params.push_back(1);
    params.push_back(cv::IMWRITE_PNG_STRATEGY);
    params.push_back(cv::IMWRITE_PNG_STRATEGY_RLE);
    params.push_back(0);
  }

  ~Receiver()
  {
  }

  void run(const Mode mode)
  {
    start(mode);
    stop();
  }

private:
  void start(const Mode mode)
  {
    this->mode = mode;
    running = true;

    std::string topicCameraInfoColor = topicColor.substr(0, topicColor.rfind('/')) + "/camera_info";
    std::string topicCameraInfoDepth = topicDepth.substr(0, topicDepth.rfind('/')) + "/camera_info";

    image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");
    subImageColor = new image_transport::SubscriberFilter(it, topicColor, queueSize, hints);
    subImageDepth = new image_transport::SubscriberFilter(it, topicDepth, queueSize, hints);
    subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoColor, queueSize);
    subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoDepth, queueSize);

    if(useExact)
    {
      syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
      syncExact->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
    }
    else
    {
      syncApproximate = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
      syncApproximate->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
    }

    spinner.start();

    std::chrono::milliseconds duration(1);
    while(!updateImage || !updateCloud)
    {
      if(!ros::ok())
      {
        return;
      }
      std::this_thread::sleep_for(duration);
    }
    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
    cloud->height = color.rows;
    cloud->width = color.cols;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);
    createLookup(this->color.cols, this->color.rows);

    switch(mode)
    {
    case CLOUD:
      cloudViewer();
      break;
    case IMAGE:
      imageViewer();
      break;
    case BOTH:
      imageViewerThread = std::thread(&Receiver::imageViewer, this);
      cloudViewer();
      break;
    }
  }

  void stop()
  {
    spinner.stop();

    if(useExact)
    {
      delete syncExact;
    }
    else
    {
      delete syncApproximate;
    }

    delete subImageColor;
    delete subImageDepth;
    delete subCameraInfoColor;
    delete subCameraInfoDepth;

    running = false;
    if(mode == BOTH)
    {
      imageViewerThread.join();
    }
  }

  void callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
                const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth)
  {
    cv::Mat color, depth;

    readCameraInfo(cameraInfoColor, cameraMatrixColor);
    readCameraInfo(cameraInfoDepth, cameraMatrixDepth);
    readImage(imageColor, color);
    readImage(imageDepth, depth);

    // IR image input
    if(color.type() == CV_16U)
    {
      cv::Mat tmp;
      color.convertTo(tmp, CV_8U, 0.02);
      cv::cvtColor(tmp, color, CV_GRAY2BGR);
    }

    lock.lock();
    this->color = color;
    this->depth = depth;
    updateImage = true;
    updateCloud = true;
    lock.unlock();
  }

  void imageViewer()
  {
    cv::Mat color, depth, depthDisp, combined;
    cv::Mat depth0, depth1, depth2, depth3, depth4, depth5, depth6, depth7, depth8, depth9, depthM;
    std::chrono::time_point<std::chrono::high_resolution_clock> start, now;
    double fps = 0;
    size_t frameCount = 0;
    std::ostringstream oss;
    const cv::Point pos(5, 15);
    const cv::Scalar colorText = CV_RGB(255, 255, 255);
    const double sizeText = 0.5;
    const int lineText = 1;
    const int font = cv::FONT_HERSHEY_SIMPLEX;
    //float depthMax = 780.0f, depthMin = 620.0f;//20160121
    float depthMax = 743.0f, depthMin = 615.0f;
    bool firstMat = false;

    //ソケット通信
    cv::Mat rightArmGp(4,1,CV_64F);// vector[x,y,z,1] , the one is needed for calculations bellow
    cv::Mat leftArmGp(4,1,CV_64F);


    int rightX, rightY, leftX, leftY;


    cv::namedWindow("Image Viewer");
    oss << "starting...";

    start = std::chrono::high_resolution_clock::now();
    for(; running && ros::ok();)
    {
      if(updateImage)
      {
        lock.lock();
        color = this->color;
        depth = this->depth;
        updateImage = false;
        lock.unlock();

	//メディアンを用いた平滑化
	cv::medianBlur(depth, depthM, 3);
	if(firstMat == false){
	  cout << "in first ######################" << endl;
	  depth9 = depthM.clone();
	  depth8 = depthM.clone();
	  depth7 = depthM.clone();
	  depth6 = depthM.clone();
	  depth5 = depthM.clone();
	  depth4 = depthM.clone();
	  depth3 = depthM.clone();
	  depth2 = depthM.clone();
	  depth1 = depthM.clone();
	  depth0 = depthM.clone();
	  firstMat = true;
	}
	depth9 = depth8.clone();
	depth8 = depth7.clone();
	depth7 = depth6.clone();
	depth6 = depth5.clone();
	depth5 = depth4.clone();
	depth4 = depth3.clone();
	depth3 = depth2.clone();
	depth2 = depth1.clone();
	depth1 = depth0.clone();
	depth0 = depthM.clone();
	average10Pictures(depth0, depth1, depth2, depth3, depth4, depth5, depth6, depth7, depth8, depth9, depthM);

	medianBlur(depthM, depth, 3);

        ++frameCount;
        now = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;
        if(elapsed >= 1.0)
        {
          fps = frameCount / elapsed;
          oss.str("");
          oss << "fps: " << fps << " ( " << elapsed / frameCount * 1000.0 << " ms)";
          start = now;
          frameCount = 0;
        }

	cv::Mat depth_8, depth_canny, can1, can2, can3, can4;
	depth_8 = cv::Mat::zeros(Size(depth.cols,depth.rows), CV_8U);
	convert16Uto8U(depth, depth_8);
	//depth.convertTo(depth_8, CV_8U, 0.3);
	cv::Canny(depth_8, depth_canny, 100, 50);
	/*cv::Canny(depth_8, can1, 100, 20);
	cv::Canny(depth_8, can2, 100, 30);
	cv::Canny(depth_8, can3, 100, 40);
	cv::Canny(depth_8, can4, 100, 50);*/

	cout<<"BEGIN#############################################################"<<endl;

		
	getSidePoints(depth,depth_canny, rightX, rightY, leftX, leftY,rightArmGp,leftArmGp); // (find rightmost and leftmost points (within hard-coded limits) in input matrix)

	//cout << "before adjust data:" << rightX << ", " << rightY << ", " << leftX << ", " << leftY << endl;



	adjustHIROcoordinate(rightX, rightY, leftX, leftY,rightArmGp,leftArmGp); // Transformation from Kinect CSYS to HIRO's coordinate system	

	/*cout << "send data:" << rightX << ", " << rightY << ", " << leftX << ", " << leftY << endl;
	cout << "test after:" <<  endl;
	cout << "rightArmGp" << rightArmGp.at<double>(0,1) <<","<<rightArmGp.at<double>(1,1) <<","<< rightArmGp.at<double>(2,1)<<","<<  rightArmGp.at<double>(3,1)<<endl;
	cout << "leftArmGp" << leftArmGp.at<double>(0,1) <<","<<leftArmGp.at<double>(1,1) <<","<< leftArmGp.at<double>(2,1)<<","<<  leftArmGp.at<double>(3,1)<<endl;
*/

	dispDepth(depth, depthDisp, depthMax, depthMin);
	//depthMaxCut(depth, depthDisp, depthMax, depthMin);
        combine(color, depthDisp, combined);
        //combined = color;

        cv::putText(combined, oss.str(), pos, font, sizeText, colorText, lineText, CV_AA);
        //cv::imshow("Image Viewer", combined);

	//cout << "depth.rows:" << depth.rows << "   depth.cols" << depth.cols << endl;
	line(depthDisp, Point(depthDisp.cols/2, 0), Point(depthDisp.cols/2, depthDisp.rows), Scalar(255,255,255), 1, 4);
	line(depthDisp, Point(depthDisp.cols/2-155, 0), Point(depthDisp.cols/2-155, depthDisp.rows), Scalar(255,255,255), 1, 4);
	line(depthDisp, Point(depthDisp.cols/2+155, 0), Point(depthDisp.cols/2+155, depthDisp.rows), Scalar(255,255,255), 1, 4);
	line(depthDisp, Point(depthDisp.cols/2-200, 0), Point(depthDisp.cols/2-200, depthDisp.rows), Scalar(255,255,255), 1, 4);
	line(depthDisp, Point(depthDisp.cols/2+200, 0), Point(depthDisp.cols/2+200, depthDisp.rows), Scalar(255,255,255), 1, 4);
	//横は200で30cm相当
	line(depthDisp, Point(0, depthDisp.rows/2), Point(depthDisp.cols, depthDisp.rows/2), Scalar(255,255,255), 1, 4);
	line(depthDisp, Point(0, depthDisp.rows/2-100), Point(depthDisp.cols, depthDisp.rows/2-100), Scalar(255,255,255), 1, 4);
	line(depthDisp, Point(0, depthDisp.rows/2-50), Point(depthDisp.cols, depthDisp.rows/2-50), Scalar(255,255,255), 1, 4);
	//縦は100で15.3cm相当
	//281:281
	//エッジの端点を探す範囲
	line(depthDisp, Point(150, 0), Point(150, depthDisp.rows), Scalar(155,155,155), 1, 4);
	line(depthDisp, Point(810, 0), Point(810, depthDisp.rows), Scalar(155,155,155), 1, 4);
	line(depthDisp, Point(0, 520), Point(depthDisp.cols, 520), Scalar(155,155,155), 1, 4);
	line(depthDisp, Point(0, 100), Point(depthDisp.cols, 100), Scalar(155,155,155), 1, 4);

	//cv::imshow("detph_8", depth_8);
	cv::imshow("depth viewer", depthDisp);
	cv::imshow("canny edge", depth_canny);
	//cv::imshow("can1", can1);
	//cv::imshow("can2", can2);
	//cv::imshow("can3", can3);
	//cv::imshow("can4", can4);

	//ソケット通信
	string message = "tech cam3d ";//送信メッセージ
	message = message + to_string(rightX) + " " + to_string(rightY) + " " + to_string(leftX) + " " + to_string(leftY);
	vector<char> SendData;
	for(int i = 0; i < int(message.size()); i++){
	  SendData.push_back(message[i]);
	}	
	client.Write(SendData);//送信
	for(int i = 0; i < int(SendData.size()); i++){
	  cout << SendData[i];
	}
	SendData.clear();

	//readしないと通信がうまく行かない
	vector<char> ReadData;
	char nSend = client.Read(ReadData);
	cout << "read:" << nSend << endl;
	//ここまでソケット通信
	
      }

      int key = cv::waitKey(1);
      switch(key & 0xFF)
      {
      case 27:
      case 'q':
        running = false;
        break;
      case ' ':
      case 's':
        if(mode == IMAGE)
        {
          createCloud(depth, color, cloud);
          saveCloudAndImages(cloud, color, depth, depthDisp);
        }
        else
        {
          save = true;
        }
        break;
      }
    }
    cv::destroyAllWindows();
    cv::waitKey(100);
  }


  void cloudViewer()
  {
    cv::Mat color, depth;
    pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
    const std::string cloudName = "rendered";

    lock.lock();
    color = this->color;
    depth = this->depth;
    updateCloud = false;
    lock.unlock();

    createCloud(depth, color, cloud);

    visualizer->addPointCloud(cloud, cloudName);
    visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
    visualizer->initCameraParameters();
    visualizer->setBackgroundColor(0, 0, 0);
    visualizer->setPosition(mode == BOTH ? color.cols : 0, 0);
    visualizer->setSize(color.cols, color.rows);
    visualizer->setShowFPS(true);
    visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
    visualizer->registerKeyboardCallback(&Receiver::keyboardEvent, *this);

    for(; running && ros::ok();)
    {
      if(updateCloud)
      {
        lock.lock();
        color = this->color;
        depth = this->depth;
        updateCloud = false;
        lock.unlock();

        createCloud(depth, color, cloud);

        visualizer->updatePointCloud(cloud, cloudName);
      }
      if(save)
      {
        save = false;
        cv::Mat depthDisp;
        dispDepth(depth, depthDisp, 12000.0f);
        saveCloudAndImages(cloud, color, depth, depthDisp);
      }
      visualizer->spinOnce(10);
    }
    visualizer->close();
  }

  void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *)
  {
    if(event.keyUp())
    {
      switch(event.getKeyCode())
      {
      case 27:
      case 'q':
        running = false;
        break;
      case ' ':
      case 's':
        save = true;
        break;
      }
    }
  }

  void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const
  {
    cv_bridge::CvImageConstPtr pCvImage;
    pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
    pCvImage->image.copyTo(image);
  }

  void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const
  {
    double *itC = cameraMatrix.ptr<double>(0, 0);
    for(size_t i = 0; i < 9; ++i, ++itC)
    {
      *itC = cameraInfo->K[i];
    }
  }

  void dispDepth(const cv::Mat &in, cv::Mat &out, const float maxValue, const float minValue = 500.0f)
  {
    cv::Mat tmp = cv::Mat(in.rows, in.cols, CV_8U);
    const uint32_t maxInt = 255;

    #pragma omp parallel for
    for(int r = 0; r < in.rows; ++r)
    {
      const uint16_t *itI = in.ptr<uint16_t>(r);
      uint8_t *itO = tmp.ptr<uint8_t>(r);

      for(int c = 0; c < in.cols; ++c, ++itI, ++itO)
      {
	//最小値をいじるんじゃなくてスケールをいじる
	if(*itI > minValue){
        //*itO = (uint8_t)std::min((*itI * maxInt / maxValue), 255.0f);
	*itO = (uint8_t)std::min(( (maxValue - (maxValue - *itI)*(maxValue/(maxValue-minValue))) * maxInt / maxValue), 255.0f);
	}
	else{
	*itO = (uint8_t)0.0f;
	}
      }
    }

    cv::applyColorMap(tmp, out, cv::COLORMAP_JET);
  }

 void getSidePoints(cv::Mat depth,cv::Mat &in, int &rightX, int &rightY, int &leftX, int &leftY, cv::Mat &rightArmGp, cv::Mat &leftArmGp)//左右の端点を探索
  {
    	cv::Mat tmp = cv::Mat(in.rows, in.cols, CV_8U);
    	rightX = 500;
    	leftX = 0;


	rightArmGp =Mat::ones(rightArmGp.rows, rightArmGp.cols, CV_64F);//Clear/initialize the vectors
	leftArmGp  =Mat::ones(leftArmGp.rows, leftArmGp.cols, CV_64F);



    
	//Get the coordinate of the two side points in the image coordinate system (unit:pixels)
    	for(int r = 0; r < in.rows; ++r){
     		uint8_t *itI = in.ptr<uint8_t>(r);
		for(int c = 0; c < in.cols; ++c, ++itI){
			if(*itI == 255 && c > 200 && c < 650 && r < 450 && r > 120){// research window in the depth image
				if(rightX > c){
	    				rightX = c; rightY = r;

					rightArmGp.at<double>(-1,1)=r;
					rightArmGp.at<double>(0,1)=c;
	  				}
	  		if(leftX < c){
	   	 			leftX = c; leftY = r;

					leftArmGp.at<double>(-1,1)=r;
					leftArmGp.at<double>(0,1)=c;
	  				}
			}
      		}
    	}
	
	//Circle the points in the image
    	cv::circle(in, cv::Point(rightX, rightY), 10, cv::Scalar(200,200,200), 2, 4);
    	cv::circle(in, cv::Point(rightX, rightY), 15, cv::Scalar(200,200,200), 2, 4);
    	cv::circle(in, cv::Point(leftX, leftY), 13, cv::Scalar(160,160,160), 2, 4);

	//Get the corresponding depth for each side points
	rightArmGp.at<double>(1,1)=depth.at<short>(rightArmGp.at<double>(-1,1),rightArmGp.at<double>(0,1));
	leftArmGp.at<double>(1,1)=depth.at<short>(leftArmGp.at<double>(-1,1),leftArmGp.at<double>(0,1));

  } //左右の端点を探索

// Check for watabe's changes: function not used
  void depthMaxCut(const cv::Mat &in, cv::Mat &out, const float maxValue, const float minValue)
  {
    cv::Mat tmp = cv::Mat(in.rows, in.cols, CV_8U);
    const uint32_t maxInt = 255;

    #pragma omp parallel for
    for(int r = 0; r < in.rows; ++r)
    {
      const uint16_t *itI = in.ptr<uint16_t>(r);
      uint8_t *itO = tmp.ptr<uint8_t>(r);

      for(int c = 0; c < in.cols; ++c, ++itI, ++itO)
      {
	//最小値をいじるんじゃなくてスケールをいじる
	if(*itI > minValue){
        //*itO = (uint8_t)std::min((*itI * maxInt / maxValue), 255.0f);
	*itO = (uint8_t)std::min(( (maxValue - (maxValue - *itI)*(maxValue/(maxValue-minValue))) * maxInt / maxValue), 255.0f);
	}
	else{
	*itO = (uint8_t)0.0f;
	}
	if(*itI > maxValue){
	  *itO = (uint8_t)0.0f;
	}

	if(r == in.rows/2 && c == in.cols/2){
	  int distance = *itI;
	  cout << "distance:" << distance << endl;
	}
      }
    }

    cv::applyColorMap(tmp, out, cv::COLORMAP_JET);
  }


void average10Pictures(const cv::Mat &in0, const cv::Mat &in1, const cv::Mat &in2, const cv::Mat &in3, const cv::Mat &in4, const cv::Mat &in5,
			 const cv::Mat &in6, const cv::Mat &in7, const cv::Mat &in8, const cv::Mat &in9, cv::Mat &out){//画像10個の平均を出力
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //オリジナルのdepthはCV_16U
    cv::Mat tmp = cv::Mat(in0.rows, in0.cols, CV_16U);//一旦tmpに入れて最後にoutに入れる

    #pragma omp parallel for
    for(int r = 0; r < in1.rows; ++r){
      const uint16_t *it0In = in0.ptr<uint16_t>(r);
      const uint16_t *it1In = in1.ptr<uint16_t>(r);
      const uint16_t *it2In = in2.ptr<uint16_t>(r);
      const uint16_t *it3In = in3.ptr<uint16_t>(r);
      const uint16_t *it4In = in4.ptr<uint16_t>(r);
      const uint16_t *it5In = in5.ptr<uint16_t>(r);
      const uint16_t *it6In = in6.ptr<uint16_t>(r);
      const uint16_t *it7In = in7.ptr<uint16_t>(r);
      const uint16_t *it8In = in8.ptr<uint16_t>(r);
      const uint16_t *it9In = in9.ptr<uint16_t>(r);
      uint16_t *itOut = tmp.ptr<uint16_t>(r);

      for(int c = 0; c < in0.cols; ++c, ++it0In, ++it1In, ++it2In, ++it3In, ++it4In, ++it5In, ++it6In, ++it7In, ++it8In, ++it9In, ++itOut){
	*itOut = (*it0In + *it1In + *it2In + *it3In + *it4In + *it5In + *it6In + *it7In + *it8In + *it9In)/10;
      }
    }
    out = tmp;
  }

  void convert16Uto8U(const cv::Mat &in1, cv::Mat &out){//16Uのin1を8UのoutにMAXVALUEとMINVALUEに基づいて変換

    for(int r = 0; r < in1.rows; r++){
      const uint16_t *it1In = in1.ptr<uint16_t>(r);
      uint8_t *itOut = out.ptr<uint8_t>(r);
      for(int c = 0; c < in1.cols; c++, it1In++, itOut++){
	if(*it1In > DEPTHMAX){
	  *itOut = (uint8_t)255;
	}else if(*it1In < DEPTHMIN){
	  *itOut = (uint8_t)0;
	}else{
	  *itOut = (uint8_t)std::min((*it1In - DEPTHMIN) * (255/(DEPTHMAX - DEPTHMIN)), 255.0f);//深度の分解能は (DEPTHMAX-DEPTHMIN)/255
	}
      }
    }
  }

  void adjustHIROcoordinate(int &rightX, int &rightY, int &leftX, int &leftY, cv::Mat &rightArmGp,cv::Mat &leftArmGp){
    int rX = rightX, rY = rightY, lX = leftX, lY = leftY;

	rightX = int( ((rY-277)*0.0015 + 0.340)*10000 );
	rightY = int( ((rX-285)*0.0015 - 0.358)*10000 );
	leftX  = int( ((lY-277)*0.0015 + 0.340)*10000 );
	leftY  = int( ((lX-675)*0.0015 + 0.358)*10000 );
    	//cout << "adjust data:" << rightX << ", " << rightY << ", " << leftX << ", " << leftY << endl;

	cout<<"IMAGE COORDINATES=========================="<<endl;
	cout<<"rightArm"<<endl<<rightArmGp<<endl;
	cout<<"leftArm"<<endl<<leftArmGp<<endl;

    	cv::Mat rightArmGpHIRO=Mat::ones(4,1,CV_64F);
    	cv::Mat leftArmGpHIRO=Mat::ones(4,1,CV_64F);
    	
	//Tranformation from picture to Kinect Coordinate system
	int z=rightArmGp.at<double>(1,1);
	rightArmGp.at<double>(-1,1) = z*rightArmGp.at<double>(-1,1)/au - u0*z/au;
	rightArmGp.at<double>(0,1) = z*rightArmGp.at<double>(0,1)/av - v0*z/av;

	z=leftArmGp.at<double>(1,1);
	leftArmGp.at<double>(-1,1) = z*leftArmGp.at<double>(-1,1)/au - u0*z/au;
	leftArmGp.at<double>(0,1) = z*leftArmGp.at<double>(0,1)/av - v0*z/au;

	cout<<"KINECT COORDINATES========================="<<endl;
	cout<<"rightArm"<<endl<<rightArmGp<<endl;
	cout<<"leftArm"<<endl<<leftArmGp<<endl;

	
	


    //Tranformation from Kinect Coordinate System to HIRo Coordinate system

	cv::Mat kinectToHIRO =(Mat_<double>(4,4) <<	0.0384,    0.9992,   -0.0055,   -0.0741, 
							0.9992,   -0.0385,   -0.0108,   -0.5143, 
							-0.0110,   -0.0050,   -0.9999,   0.7813,
					     		0 ,          0 ,         0 ,     1.0000	);


	rightArmGpHIRO = kinectToHIRO * rightArmGp;
	leftArmGpHIRO = kinectToHIRO * leftArmGp;
	
	cout<<"HIRO COORDINATES========================="<<endl;	
	cout<<"kinectToHIRO:"<<endl<<kinectToHIRO<<endl;
	cout<<"rightArmGpHIRO"<<endl<<rightArmGpHIRO<<endl;
	cout<<"leftArmGpHIRO"<<endl<<leftArmGpHIRO<<endl;


    
  }

  void combine(const cv::Mat &inC, const cv::Mat &inD, cv::Mat &out)
  {
    out = cv::Mat(inC.rows, inC.cols, CV_8UC3);

    #pragma omp parallel for
    for(int r = 0; r < inC.rows; ++r)
    {
      const cv::Vec3b
      *itC = inC.ptr<cv::Vec3b>(r),
       *itD = inD.ptr<cv::Vec3b>(r);
      cv::Vec3b *itO = out.ptr<cv::Vec3b>(r);

      for(int c = 0; c < inC.cols; ++c, ++itC, ++itD, ++itO)
      {
        itO->val[0] = (itC->val[0] + itD->val[0]) >> 1;
        itO->val[1] = (itC->val[1] + itD->val[1]) >> 1;
        itO->val[2] = (itC->val[2] + itD->val[2]) >> 1;
      }
    }
  }

  void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const
  {
    const float badPoint = std::numeric_limits<float>::quiet_NaN();

    #pragma omp parallel for
    for(int r = 0; r < depth.rows; ++r)
    {
      pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
      const uint16_t *itD = depth.ptr<uint16_t>(r);
      const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
      const float y = lookupY.at<float>(0, r);
      const float *itX = lookupX.ptr<float>();

      for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itX)
      {
        register const float depthValue = *itD / 1000.0f;
        // Check for invalid measurements
        if(isnan(depthValue) || depthValue <= 0.001)
        {
          // not valid
          itP->x = itP->y = itP->z = badPoint;
          itP->rgba = 0;
          continue;
        }
        itP->z = depthValue;
        itP->x = *itX * depthValue;
        itP->y = y * depthValue;
        itP->b = itC->val[0];
        itP->g = itC->val[1];
        itP->r = itC->val[2];
        itP->a = 255;
      }
    }
  }

  void saveCloudAndImages(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, const cv::Mat &color, const cv::Mat &depth, const cv::Mat &depthColored)
  {
    oss.str("");
    oss << "./" << std::setfill('0') << std::setw(4) << frame;
    const std::string baseName = oss.str();
    const std::string cloudName = baseName + "_cloud.pcd";
    const std::string colorName = baseName + "_color.jpg";
    const std::string depthName = baseName + "_depth.png";
    const std::string depthColoredName = baseName + "_depth_colored.png";

    OUT_INFO("saving cloud: " << cloudName);
    writer.writeBinary(cloudName, *cloud);
    OUT_INFO("saving color: " << colorName);
    cv::imwrite(colorName, color, params);
    OUT_INFO("saving depth: " << depthName);
    cv::imwrite(depthName, depth, params);
    OUT_INFO("saving depth: " << depthColoredName);
    cv::imwrite(depthColoredName, depthColored, params);
    OUT_INFO("saving complete!");
    ++frame;
  }

  void createLookup(size_t width, size_t height)
  {
    const float fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
    const float fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
    const float cx = cameraMatrixColor.at<double>(0, 2);
    const float cy = cameraMatrixColor.at<double>(1, 2);
    float *it;

    lookupY = cv::Mat(1, height, CV_32F);
    it = lookupY.ptr<float>();
    for(size_t r = 0; r < height; ++r, ++it)
    {
      *it = (r - cy) * fy;
    }

    lookupX = cv::Mat(1, width, CV_32F);
    it = lookupX.ptr<float>();
    for(size_t c = 0; c < width; ++c, ++it)
    {
      *it = (c - cx) * fx;
    }
  }
};

// Defines additional functions in Receiver class
//#include "viewerExtraFunctions.cpp"

void help(const std::string &path)
{
  std::cout << path << FG_BLUE " [options]" << std::endl
            << FG_GREEN "  name" NO_COLOR ": " FG_YELLOW "'any string'" NO_COLOR " equals to the kinect2_bridge topic base name" << std::endl
            << FG_GREEN "  mode" NO_COLOR ": " FG_YELLOW "'qhd'" NO_COLOR ", " FG_YELLOW "'hd'" NO_COLOR ", " FG_YELLOW "'sd'" NO_COLOR " or " FG_YELLOW "'ir'" << std::endl
            << FG_GREEN "  visualization" NO_COLOR ": " FG_YELLOW "'image'" NO_COLOR ", " FG_YELLOW "'cloud'" NO_COLOR " or " FG_YELLOW "'both'" << std::endl
            << FG_GREEN "  options" NO_COLOR ":" << std::endl
            << FG_YELLOW "    'compressed'" NO_COLOR " use compressed instead of raw topics" << std::endl
            << FG_YELLOW "    'approx'" NO_COLOR " use approximate time synchronization" << std::endl;
}

int main(int argc, char **argv)
{
#if EXTENDED_OUTPUT
  ROSCONSOLE_AUTOINIT;
  if(!getenv("ROSCONSOLE_FORMAT"))
  {
    ros::console::g_formatter.tokens_.clear();
    ros::console::g_formatter.init("[${severity}] ${message}");
  }
#endif

  ros::init(argc, argv, "kinect2_viewer", ros::init_options::AnonymousName);

  if(!ros::ok())
  {
    return 0;
  }
  //ソケット通信Socket connection
  string LOCALHOST = IPADDRESS;
  int PORT = PORTNUM;
  client.Init(LOCALHOST, PORT);



  std::string ns = K2_DEFAULT_NS;
  std::string topicColor = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
  std::string topicDepth = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
  bool useExact = true;
  bool useCompressed = false;
  Receiver::Mode mode = Receiver::CLOUD;

  for(size_t i = 1; i < (size_t)argc; ++i)
  {
    std::string param(argv[i]);

    if(param == "-h" || param == "--help" || param == "-?" || param == "--?")
    {
      help(argv[0]);
      ros::shutdown();
      return 0;
    }
    else if(param == "qhd")
    {
      topicColor = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
    else if(param == "hd")
    {
      topicColor = K2_TOPIC_HD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_HD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
    else if(param == "ir")
    {
      topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_IR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
    else if(param == "sd")
    {
      topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
    else if(param == "approx")
    {
      useExact = false;
    }

    else if(param == "compressed")
    {
      useCompressed = true;
    }
    else if(param == "image")
    {
      mode = Receiver::IMAGE;
    }
    else if(param == "cloud")
    {
      mode = Receiver::CLOUD;
    }
    else if(param == "both")
    {
      mode = Receiver::BOTH;
    }
    else
    {
      ns = param;
    }
  }

  topicColor = "/" + ns + topicColor;
  topicDepth = "/" + ns + topicDepth;
  OUT_INFO("topic color: " FG_CYAN << topicColor << NO_COLOR);
  OUT_INFO("topic depth: " FG_CYAN << topicDepth << NO_COLOR);

  Receiver receiver(topicColor, topicDepth, useExact, useCompressed);

  OUT_INFO("starting receiver...");
  receiver.run(mode);

  ros::shutdown();
  return 0;
}

/*
struct camera_params_t
{
    struct ir_t{
        float cx = 254.878f;
        float cy = 205.395f;
        float fx = 365.456f;
        float fy = 365.456f;
        float k1 = 0.0905474;
        float k2 = -0.26819;
        float k3 = 0.0950862;
        float p1 = 0.0;
        float p2 = 0.0;
    }ir;
}camera_params;


bool ObjectTracker::depth_image_to_point_cloud(cv::Mat& depth, cv::Mat& xycords, pcl::PointCloud<PointT>::Ptr dst)
{
    //Process Depth To PCL Cloud
    uint pixel_count = depth.rows * depth.cols;
    dst->resize(pixel_count);
    dst->height = depth.rows;
    dst->width = depth.cols;

    float x = 0.0f, y = 0.0f;

    float* ptr = (float*) (depth.data);
    for (uint i = 0; i < pixel_count; ++i)
    {
        cv::Vec2f xy = xycords.at<cv::Vec2f>(0, i);
        x = xy[1]; y = xy[0];

        PointT point;
        point.z = (static_cast<float>(*ptr)) / (1000.0f); // Convert from mm to meters
        point.x = (x - config.camera_params.ir.cx) * point.z / config.camera_params.ir.fx;
        point.y = (y - config.camera_params.ir.cy) * point.z / config.camera_params.ir.fy;
        point.r = 128; point.g = 128; point.b = 128;
        dst->at(i) = point;

        ++ptr;
    }
}

nt width = 512;
int height = 424;

cv::Mat cv_img_cords = cv::Mat(1, width*height, CV_32FC2);
for (int r = 0; r < height; ++r) {
    for (int c = 0; c < width; ++c) {
        cv_img_cords.at<cv::Vec2f>(0, r*width + c) = cv::Vec2f((float)r, (float)c);
    }
}

cv::Mat k = cv::Mat::eye(3, 3, CV_32F);
k.at<float>(0,0) = config.camera_params.ir.fx;
k.at<float>(1,1) = config.camera_params.ir.fy;
k.at<float>(0,2) = config.camera_params.ir.cx;
k.at<float>(1,2) = config.camera_params.ir.cy;

cv::Mat dist_coeffs = cv::Mat::zeros(1, 8, CV_32F);
dist_coeffs.at<float>(0,0) = config.camera_params.ir.k1;
dist_coeffs.at<float>(0,1) = config.camera_params.ir.k2;
dist_coeffs.at<float>(0,2) = config.camera_params.ir.p1;
dist_coeffs.at<float>(0,3) = config.camera_params.ir.p2;
dist_coeffs.at<float>(0,4) = config.camera_params.ir.k3;

cv::Mat new_camera_matrix = cv::getOptimalNewCameraMatrix(k, dist_coeffs, cv::Size2i(height,width), 0.0);

cv::undistortPoints(cv_img_cords, cv_img_corrected_cords, k, dist_coeffs, cv::noArray(), new_camera_matrix);



*/












