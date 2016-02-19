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
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>


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

static const int PICTURES = 10;

cv::Rect box;
bool drawing_box = false;

void draw_box(cv::Mat* img, cv::Rect rect){
    cv::rectangle(*img, cv::Point2d(box.x, box.y), cv::Point2d(box.x + box.width, box.y + box.height),
        cv::Scalar(0xff, 0x00, 0x00));
}

void PrintDepth(unsigned int depth)//Mat型の画像のビット深度を出す
{
  std::string strDepth = 
    (
      depth == CV_8U ? "CV_8U" :
      depth == CV_8S ? "CV_8S" :
      depth == CV_16U ? "CV_16U" :
      depth == CV_16S ? "CV_16S" :
      depth == CV_32S ? "CV_32S" :
      depth == CV_32F ? "CV_32F" :
      depth == CV_64F ? "CV_64F" :
      "Other"
    );
  std::cout << "depth: " << strDepth << std::endl;
}

// コールバック関数
void my_mouse_callback(int event, int x, int y, int flags, void* param){
    cv::Mat* image = static_cast<cv::Mat*>(param);

    switch (event){
    case cv::EVENT_MOUSEMOVE:
        if (drawing_box){
            box.width = x - box.x;
            box.height = y - box.y;
        }
        break;

    case cv::EVENT_LBUTTONDOWN:
        drawing_box = true;
        box = cv::Rect(x, y, 0, 0);
        break;

    case cv::EVENT_LBUTTONUP:
        drawing_box = false;
        if (box.width < 0){
            box.x += box.width;
            box.width *= -1;
        }
        if (box.height < 0){
            box.y += box.height;
            box.height *= -1;
        }
        draw_box(image, box);
        break;
    }
}
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
    cv::Mat color, depth, depthDisp, combined, previousDepth, aveDepth, aveDepthDisp;
    cv::Mat depth0, depth1, depth2, depth3, depth4, depth5, depth6, depth7, depth8, depth9;
    cv::Mat aveDepthMedian, aveDepthMedianLaplacian, aveDepthMedianGaussian, aveDepthMedianGaussianLaplacian, aveDepthMedianGaussianCanny;
    cv::Mat aveDepthMedianGaussian_convert01;
    std::chrono::time_point<std::chrono::high_resolution_clock> start, now;
    double fps = 0;
    size_t frameCount = 0, trueFrameCount = 0;
    std::ostringstream oss;
    const cv::Point pos(5, 15);
    const cv::Scalar colorText = CV_RGB(255, 255, 255);
    const double sizeText = 0.5;
    const int lineText = 1;
    const int font = cv::FONT_HERSHEY_SIMPLEX;
    //float depthMax = 780.0f, depthMin = 620.0f;
    float depthMax = 742.0f, depthMin = 620.0f;

    std::string name = "Image Viewer";
    box = cv::Rect(-1, -1, 0, 0);
    cv::Mat temp = depthDisp.clone();

    cv::namedWindow(name, CV_WINDOW_AUTOSIZE);
    oss << "starting...";

    cv::setMouseCallback(name, my_mouse_callback, (void *)&depthDisp);

    start = std::chrono::high_resolution_clock::now();
    for(; running && ros::ok();)
    {

      if(updateImage)
      {
        lock.lock();
        color = this->color;
        depth = this->depth;

	//ファイルから読み込み
	//color = cv::imread("~/catkin_ws/color01.png", 0);
	//depth = cv::imread("~/catkin_ws/depth01.png", 0);

        updateImage = false;
        lock.unlock();

	//exit(0);


        ++frameCount;
	++trueFrameCount;
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

	cv::Mat out_gaussian_img1, out_bilateral_img1, out_bilateral_img2, out_laplacian_img1,
	  out_laplacian_img2, out_laplacian_img3, out_median_img1;
	cv::Mat depth_8, depth_32, depthDisp_gaussian, depthDisp_bilateral, depthDisp_bilateral2, depthDisp_median_bilateral, depthDisp_median, depthDisp_median_bilateral_32;
	cv::Mat depthM;

	cout << trueFrameCount << " frame" << endl;
	if(trueFrameCount <= 10){
	  //メディアンを用いた平滑化
	  cv::medianBlur(depth, depthM, 3);

	  if(trueFrameCount == 1)
	    depth0 = depthM;
	  else if(trueFrameCount == 2)
	    depth1 = depthM;
	  else if(trueFrameCount == 3)
	    depth2 = depthM;
	  else if(trueFrameCount == 4)
	    depth3 = depthM;
	  else if(trueFrameCount == 5)
	    depth4 = depthM;
	  else if(trueFrameCount == 6)
	    depth5 = depthM;
	  else if(trueFrameCount == 7)
	    depth6 = depthM;
	  else if(trueFrameCount == 8)
	    depth7 = depthM;
	  else if(trueFrameCount == 9)
	    depth8 = depthM;
	  else if(trueFrameCount == 10){
	    depth9 = depthM;
	    average10Pictures(depth0, depth1, depth2, depth3, depth4, depth5, depth6, depth7, depth8, depth9, aveDepth);

	    cv::medianBlur(aveDepth, aveDepthMedian, 3);
	  }
	}
	previousDepth = depth;


	depth.convertTo(depth_8, CV_8U, 1.0/255);//16bitから8bitに落とすので中の値を8bit分スケール落とす
	depth.convertTo(depth_32, CV_32F, 1.0/65535);//16bitの値をそのまま小数におさめただけ
	

	unsigned int colorint = color.depth();
	PrintDepth(colorint);//colorはCV_8U


	cv::Mat out_median_32, out_median_bilateral_32, out_2times, out_3times, out_4times, out_2, out_3, out_4;

	//cv::imshow("flesh depth", depth);
	//cv::imshow("flesh depth8", depth_8);


	//ガウシアンを用いた平滑化
	//入力画像，出力画像，カーネルサイズ，標準偏差x, y
	cv::GaussianBlur(depth, out_gaussian_img1, cv::Size(3, 3), 10, 10);
	//cv::GaussianBlur(depth, out_gaussian_img1, cv::Size(15, 15), 50, 50);

	//メディアンを用いた平滑化
	cv::medianBlur(depth_8, out_median_img1, 3);
	cv::medianBlur(depth_32, out_median_32, 3);


	//バイラテラルを用いた平滑化
	cv::bilateralFilter(depth_8, out_bilateral_img1, 7, 5, 5);
	cv::bilateralFilter(depth_32, out_bilateral_img2, 7, 5, 5);
	cv::bilateralFilter(out_median_32, out_median_bilateral_32, 5, 50, 20);

	/*
	//何回かかける
	cv::Mat bilateralLoop;
	cv::bilateralFilter(out_median_bilateral_32, bilateralLoop, 5, 50, 20);
	cv::bilateralFilter(bilateralLoop, out_median_bilateral_32, 5, 50, 20);
	*/

	cv::Mat out_median_bilateral;
	cv::Mat depth_8_median;
	out_median_img1.convertTo(depth_8_median, CV_8U);
	cv::bilateralFilter(depth_8_median, out_median_bilateral, 3, 3, 5);//3, 3, 5でとりあえず出る

	//ガウシアン
	cv::Mat out_median_bilateral_gaussian_32, gaussianLoop;
	cv::GaussianBlur(out_median_bilateral_32, out_median_bilateral_gaussian_32, cv::Size(3, 3), 2, 2);
	//cv::GaussianBlur(out_median_bilateral_gaussian_32, gaussianLoop, cv::Size(3, 3), 2, 2);
	//cv::GaussianBlur(gaussianLoop, out_median_bilateral_gaussian_32, cv::Size(3, 3), 2, 2);
	//何回かガウシアンかける
	//cv::GaussianBlur(out_median_bilateral_gaussian_32, out_median_bilateral_gaussian_32, cv::Size(3, 3), 0, 0);


	//ラプラシアンを用いたエッジ抽出
	cv::Mat out_median_bilateral_laplacian, out_median_bilateral_laplacian_32;
	cv::Laplacian(out_median_bilateral, out_median_bilateral_laplacian, CV_32F, 3);

	out_median_bilateral_32.convertTo(out_median_bilateral_32, CV_16U, 65535);//
	out_median_bilateral_gaussian_32.convertTo(out_median_bilateral_gaussian_32, CV_16S, 65535);

	cv::Mat out_median_bilateral_gaussian_laplacian_32;
	cv::Laplacian(out_median_bilateral_32, out_median_bilateral_laplacian_32, CV_32F, 3);
	cv::Laplacian(out_median_bilateral_gaussian_32, out_median_bilateral_gaussian_laplacian_32, CV_32F, 1);
	
	//Cannyエッジ
	/*
	cv::Mat out_median_bilateral_gaussian_canny_32, out_median_bilateral_gaussian_32_8, out_median_bilateral_32_8, out_median_32_8;
	out_median_bilateral_gaussian_32.convertTo(out_median_bilateral_gaussian_32_8, CV_8U, 65535/255);
	out_median_bilateral_32.convertTo(out_median_bilateral_32_8, CV_8U, 65535/255);
	out_median_32.convertTo(out_median_32_8, CV_8U, 65535/255);
	cv::Canny(out_median_bilateral_32_8, out_median_bilateral_gaussian_canny_32, 100, 200);

	unsigned int out_median_bilateral_32_8int = out_median_bilateral_32_8.depth();
	unsigned int out_median_32_8int = out_median_32_8.depth();
	PrintDepth(out_median_bilateral_32_8int);
	PrintDepth(out_median_32_8int);
	*/
    
	//素の深度画像は見づらいので見やすく変換
	dispDepth(depth, depthDisp, depthMax, depthMin);
	if(trueFrameCount >= 10){
	  dispDepth(aveDepthMedian, aveDepthDisp, depthMax, depthMin);
	  //aveDepthMedian.convertTo(aveDepthMedian, CV_32F, 1.0/65535);//16bitの値をそのまま小数におさめただけ
	  cv::GaussianBlur(aveDepthMedian, aveDepthMedianGaussian, cv::Size(7, 7), 0);
	  cv::Laplacian(aveDepthMedian, aveDepthMedianLaplacian, CV_32F, 3);
	  cv::Laplacian(aveDepthMedianGaussian, aveDepthMedianGaussianLaplacian, CV_32F, 3);

	  //aveDepthMedianGaussian.convertTo(aveDepthMedianGaussian_convert01, CV_8U, 65535/255);
	  //cv::Canny(aveDepthMedianGaussian_convert01, aveDepthMedianGaussianCanny, 3, 2);
	  //cv::Canny(aveDepthMedianGaussian, aveDepthMedianGaussianCanny, 3, 2);
	}
	dispDepth(out_bilateral_img1, depthDisp_bilateral, depthMax, depthMin);
	//out_median_bilateral.convertTo(out_median_bilateral, CV_16U, 255);//8から16なので8bit分掛ける
	dispDepth(out_median_bilateral, depthDisp_median_bilateral, depthMax, depthMin);
	dispDepth(out_gaussian_img1, depthDisp_gaussian, depthMax, depthMin);

	out_bilateral_img2.convertTo(out_bilateral_img2, CV_16U, depthMin);
	dispDepth(out_bilateral_img2, depthDisp_bilateral2, depthMax, depthMin);

	out_median_img1.convertTo(out_median_img1, CV_16U, 8);
	//dispDepth(out_median_img1, depthDisp_median, 800.0f, 500.0f);

	out_median_32.convertTo(out_median_32, CV_16U, 65535);
	dispDepth(out_median_32, depthDisp_median, depthMax, depthMin);

	//out_median_bilateral_32.convertTo(out_median_bilateral_32, CV_16U, 65535);//上の方に書いてる
	dispDepth(out_median_bilateral_32, depthDisp_median_bilateral_32, depthMax, depthMin);


	cv::Mat color_laplacian;
	cv::Laplacian(color, color_laplacian, CV_8U, 3);

        combine(color, depthDisp, combined);

	//ビット深度表示
	unsigned int depthint = depth.depth();
	unsigned int depthDispint = depthDisp.depth();
	PrintDepth(depthint);
	PrintDepth(depthDispint);

	cout << depth.type() << " " << depth.depth() <<  endl;
	depthDisp.copyTo(temp);
	if(drawing_box){
	  draw_box(&temp, box);
	  cout << "square drawing" << endl;
	}
	cout << "drawing" << endl;
        cv::putText(combined, oss.str(), pos, font, sizeText, colorText, lineText, CV_AA);

	cv::imshow("color laplacian", color_laplacian);
	cv::imshow("Origin depth", depthDisp);
	if(trueFrameCount >= 10){
	  cv::imshow("average depth", aveDepthDisp);
	  cv::imshow("average depth edge", aveDepthMedianLaplacian);
	  cv::imshow("average depth edge gaussian", aveDepthMedianGaussianLaplacian);
	  //cv::imshow("average depth edge gaussian canny", aveDepthMedianGaussianCanny);
	}
	//cv::imshow("flesh depth", depth);
	//cv::imshow("flesh depth8", depth_8);
        cv::imshow("Image Viewer", color);
	//cv::imshow("depth Viewer", depth);
	//cv::imshow("gaussian depth", out_gaussian_img1);
	//cv::imshow("gaussian depth", depthDisp_gaussian);
	cv::imshow("median depth", depthDisp_median);
	//cv::imshow("bilateral depth", depthDisp_bilateral2);
	//cv::imshow("median bilateral 32", depthDisp_median_bilateral_32);
	//cv::imshow("median bilateral", depthDisp_median_bilateral);
	//cv::imshow("median bilateral origin", out_median_bilateral);
	//cv::imshow("bilateral depth", depthDisp_bilateral);
	//cv::imshow("LOG viewer", out_laplacian_img1);
	//cv::imshow("bilateral viewer", out_laplacian_img2);
	//cv::imshow("median viewer", out_laplacian_img3);
	//cv::imshow("median bilateral laplacian", out_median_bilateral_laplacian);//まあまあ

	cv::imshow("edge", out_median_bilateral_laplacian_32);
	//cv::imshow("median bilateral gaussian laplacian", out_median_bilateral_gaussian_laplacian_32);
	//cv::imshow("median bilateral gaussian canny", out_median_bilateral_gaussian_canny_32);
	//cv::imshow("edge2", out_2);
	//cv::imshow("edge3", out_3);
	//cv::imshow("edge4", out_4);

	cout << "showed" << endl;

	/*
	if(trueFrameCount >= 10)
	  getchar();
	*/

	cv::imwrite("color02.png", color);
	cv::imwrite("depth02.png", depth);
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
    cv::Mat depth_32, out_median_img1, out_median_bilateral_img1, out_median_bilateral_gaussian_img1;
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



	depth.convertTo(depth_32, CV_32F, 1.0/65535);//16bitの値をそのまま小数におさめただけ

	//メディアンを用いた平滑化
	cv::medianBlur(depth_32, out_median_img1, 3);

	//バイラテラルを用いた平滑化
	cv::bilateralFilter(out_median_img1, out_median_bilateral_img1, 5, 50, 20);
	
	cv::Mat bilateral_loop;
	cv::bilateralFilter(out_median_bilateral_img1, bilateral_loop, 5, 50, 20);
	cv::bilateralFilter(bilateral_loop, out_median_bilateral_img1, 5, 50, 20);
	

	//ガウシアン
	cv::GaussianBlur(out_median_bilateral_img1, out_median_bilateral_gaussian_img1, cv::Size(3, 3), 2, 2);

	//depth_32.convertTo(depth, CV_16U, 65535);
	out_median_img1.convertTo(depth, CV_16U, 65535);
	//out_median_bilateral_img1.convertTo(depth, CV_16U, 65535);
	//out_median_bilateral_gaussian_img1.convertTo(depth, CV_16U, 65535);


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
	if(*itI > maxValue){
	  *itO = (uint8_t)0.0f;
	}
      }
    }

    cv::applyColorMap(tmp, out, cv::COLORMAP_JET);
  }

  
  void average2Pictures(const cv::Mat &in1, const cv::Mat &in2, cv::Mat &out){//画像in1とin2の平均を出力
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //オリジナルのdepthはCV_16U
    cv::Mat tmp = cv::Mat(in1.rows, in1.cols, CV_16U);//一旦tmpに入れて最後にoutに入れる

    #pragma omp parallel for
    for(int r = 0; r < in1.rows; ++r){
      const uint16_t *it1In = in1.ptr<uint16_t>(r);
      const uint16_t *it2In = in2.ptr<uint16_t>(r);
      uint16_t *itOut = tmp.ptr<uint16_t>(r);

      for(int c = 0; c < in1.cols; ++c, ++it1In, ++it2In, ++itOut){
	*itOut = (*it1In + *it2In)/2;
      }
    }
    out = tmp;
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
    cout << "average10Pictures finish" << endl;
  }

  /*
  void averagePluralPictures(const cv::Mat &in[PICTURES], cv::Mat &out){
    cv::Mat tmp = cv::Mat(in[0].rows, in[0].cols, CV_16U);
  }
  */  

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

    std::cout << "saving cloud: " << cloudName << std::endl;
    writer.writeBinary(cloudName, *cloud);
    std::cout << "saving color: " << colorName << std::endl;
    cv::imwrite(colorName, color, params);
    std::cout << "saving depth: " << depthName << std::endl;
    cv::imwrite(depthName, depth, params);
    std::cout << "saving depth: " << depthColoredName << std::endl;
    cv::imwrite(depthColoredName, depthColored, params);
    std::cout << "saving complete!" << std::endl;
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

void help(const std::string &path)
{
  std::cout << path << " [options]" << std::endl
            << "  name: 'any string' equals to the kinect2_bridge topic base name" << std::endl
            << "  mode: 'qhd', 'hd', 'sd' or 'ir'" << std::endl
            << "  visualization: 'image', 'cloud' or 'both'" << std::endl
            << "  options:" << std::endl
            << "    'compressed' use compressed instead of raw topics" << std::endl
            << "    'approx' use approximate time synchronization" << std::endl;
}





int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinect2_viewer", ros::init_options::AnonymousName);


  if(!ros::ok())
  {
    return 0;
  }

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
  std::cout << "topic color: " << topicColor << std::endl;
  std::cout << "topic depth: " << topicDepth << std::endl;

  Receiver receiver(topicColor, topicDepth, useExact, useCompressed);

  std::cout << "starting receiver..." << std::endl;
  receiver.run(mode);

  ros::shutdown();
  return 0;
}
