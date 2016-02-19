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
/*
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>
#include <sys/stat.h>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>

#include <tf/transform_broadcaster.h>

#include <compressed_depth_image_transport/compression_common.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/config.h>
#include <libfreenect2/registration.h>

#include <kinect2_bridge/kinect2_definitions.h>
#include <kinect2_registration/kinect2_registration.h>
*/
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>
#include <sys/stat.h>

#include <opencv2/opencv.hpp>


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>


#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>

#include <tf/transform_broadcaster.h>

#include <compressed_depth_image_transport/compression_common.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/config.h>
#include <libfreenect2/registration.h>

#include <kinect2_bridge/kinect2_definitions.h>
#include <kinect2_registration/kinect2_registration.h>


//
static const int NUMOFDEPTH = 80;
static const float DEPTHMAX = 7400.0f;//深度画像を見やすくする
static const float DEPTHMIN = 6800.0f;

static const int DEPTHROWS = 424;
static const int DEPTHCOLS = 512;

cv::Mat addDepth(DEPTHROWS, DEPTHCOLS, CV_32S, cv::Scalar(0));
cv::Mat addDepth_16U(DEPTHROWS, DEPTHCOLS, CV_16U, cv::Scalar(0));
cv::Mat addDepth_16U_8Uscaled(DEPTHROWS, DEPTHCOLS, CV_16U, cv::Scalar(0));
cv::Mat addDepth_8U(DEPTHROWS, DEPTHCOLS, CV_8U, cv::Scalar(0));
cv::Mat addDepth_Canny01, addDepth_Canny02, addDepth_Canny03, addDepth_Canny04, addDepth_Canny05;
size_t addCount = 0;
//

class Kinect2Bridge
{
private:
  std::vector<int> compressionParams;
  std::string compression16BitExt, compression16BitString, baseNameTF;

  cv::Size sizeColor, sizeIr, sizeLowRes;
  cv::Mat color, ir, depth;
  cv::Mat cameraMatrixColor, distortionColor, cameraMatrixLowRes, cameraMatrixIr, distortionIr, cameraMatrixDepth, distortionDepth;
  cv::Mat rotation, translation;
  cv::Mat map1Color, map2Color, map1Ir, map2Ir, map1LowRes, map2LowRes;

  std::vector<std::thread> threads;
  std::mutex lockIrDepth, lockColor, lockColorFrame;
  std::mutex lockSync, lockPub, lockTime, lockStatus;
  std::mutex lockRegLowRes, lockRegHighRes;

  bool publishTF;
  std::thread tfPublisher, mainThread;

  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *device;
  libfreenect2::SyncMultiFrameListener *listenerColor, *listenerIrDepth;
  libfreenect2::PacketPipeline *packetPipeline;
  libfreenect2::Registration *registration;
  libfreenect2::Freenect2Device::ColorCameraParams colorParams;
  libfreenect2::Freenect2Device::IrCameraParams irParams;
  libfreenect2::Frame colorFrame;

  ros::NodeHandle nh, priv_nh;

  DepthRegistration *depthRegLowRes, *depthRegHighRes;

  size_t frameColor, frameIrDepth, pubFrameColor, pubFrameIrDepth;
  ros::Time lastColor, lastDepth;

  bool nextColor, nextIrDepth;
  double deltaT, depthShift, elapsedTimeColor, elapsedTimeIrDepth;
  bool running, deviceActive, clientConnected;

  enum Image
  {
    IR_SD = 0,
    IR_SD_RECT,//1

    DEPTH_SD,//2
    DEPTH_SD_RECT,//3
    DEPTH_HD,//4
    DEPTH_QHD,//5

    COLOR_SD_RECT,//6
    COLOR_HD,//7
    COLOR_HD_RECT,//8
    COLOR_QHD,//9
    COLOR_QHD_RECT,//10

    MONO_HD,//11
    MONO_HD_RECT,//12
    MONO_QHD,//13
    MONO_QHD_RECT,//14

    COUNT//15
  };

  enum Status
  {
    UNSUBCRIBED = 0,
    RAW,
    COMPRESSED,
    BOTH
  };

  std::vector<ros::Publisher> imagePubs, compressedPubs;
  ros::Publisher infoHDPub, infoQHDPub, infoIRPub;
  sensor_msgs::CameraInfo infoHD, infoQHD, infoIR;
  std::vector<Status> status;

public:
  Kinect2Bridge(const ros::NodeHandle &nh = ros::NodeHandle(), const ros::NodeHandle &priv_nh = ros::NodeHandle("~"))
    : sizeColor(1920, 1080), sizeIr(512, 424), sizeLowRes(sizeColor.width / 2, sizeColor.height / 2), colorFrame(1920, 1080, 4), nh(nh), priv_nh(priv_nh),
      frameColor(0), frameIrDepth(0), pubFrameColor(0), pubFrameIrDepth(0), lastColor(0, 0), lastDepth(0, 0), nextColor(false),
      nextIrDepth(false), depthShift(0), running(false), deviceActive(false), clientConnected(false)
  {
    color = cv::Mat::zeros(sizeColor, CV_8UC3);
    ir = cv::Mat::zeros(sizeIr, CV_32F);
    depth = cv::Mat::zeros(sizeIr, CV_32F);
    memset(colorFrame.data, 0, colorFrame.width * colorFrame.height * colorFrame.bytes_per_pixel);

    status.resize(COUNT, UNSUBCRIBED);
  }

  bool start()
  {
    if(!initialize())
    {
      std::cerr << "Initialization failed!" << std::endl;
      return false;
    }
    running = true;

    if(publishTF)
    {
      tfPublisher = std::thread(&Kinect2Bridge::publishStaticTF, this);
    }

    for(size_t i = 0; i < threads.size(); ++i)
    {
      threads[i] = std::thread(&Kinect2Bridge::threadDispatcher, this, i);
    }

    mainThread = std::thread(&Kinect2Bridge::main, this);
    return true;
  }

  void stop()
  {
    running = false;

    mainThread.join();

    for(size_t i = 0; i < threads.size(); ++i)
    {
      threads[i].join();
    }

    if(publishTF)
    {
      tfPublisher.join();
    }

    device->stop();
    device->close();
    delete listenerIrDepth;
    delete listenerColor;
    delete registration;

    delete depthRegLowRes;
    delete depthRegHighRes;

    for(size_t i = 0; i < COUNT; ++i)
    {
      imagePubs[i].shutdown();
      compressedPubs[i].shutdown();
      infoHDPub.shutdown();
      infoQHDPub.shutdown();
      infoIRPub.shutdown();
    }

    nh.shutdown();
  }

private:
  bool initialize()
  {
    double fps_limit, maxDepth, minDepth;
    bool use_png, bilateral_filter, edge_aware_filter;
    int32_t jpeg_quality, png_level, queueSize, reg_dev, depth_dev, worker_threads;
    std::string depth_method, reg_method, calib_path, sensor, base_name;

    std::string depthDefault = "cpu";
    std::string regDefault = "default";

#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
    depthDefault = "opengl";
#endif
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
    depthDefault = "opencl";
#endif
#ifdef DEPTH_REG_OPENCL
    regDefault = "opencl";
#endif

    priv_nh.param("base_name", base_name, std::string(K2_DEFAULT_NS));
    priv_nh.param("sensor", sensor, std::string(""));
    priv_nh.param("fps_limit", fps_limit, -1.0);
    priv_nh.param("calib_path", calib_path, std::string(K2_CALIB_PATH));
    priv_nh.param("use_png", use_png, false);
    priv_nh.param("jpeg_quality", jpeg_quality, 90);
    priv_nh.param("png_level", png_level, 1);
    priv_nh.param("depth_method", depth_method, depthDefault);
    priv_nh.param("depth_device", depth_dev, -1);
    priv_nh.param("reg_method", reg_method, regDefault);
    priv_nh.param("reg_devive", reg_dev, -1);
    priv_nh.param("max_depth", maxDepth, 12.0);
    priv_nh.param("min_depth", minDepth, 0.1);
    priv_nh.param("queue_size", queueSize, 2);
    priv_nh.param("bilateral_filter", bilateral_filter, true);
    priv_nh.param("edge_aware_filter", edge_aware_filter, true);
    priv_nh.param("publish_tf", publishTF, false);
    priv_nh.param("base_name_tf", baseNameTF, base_name);
    priv_nh.param("worker_threads", worker_threads, 4);

    worker_threads = std::max(1, worker_threads);
    threads.resize(worker_threads);

    std::cout << "parameter:" << std::endl
              << "        base_name: " << base_name << std::endl
              << "           sensor: " << sensor << std::endl
              << "        fps_limit: " << fps_limit << std::endl
              << "       calib_path: " << calib_path << std::endl
              << "          use_png: " << (use_png ? "true" : "false") << std::endl
              << "     jpeg_quality: " << jpeg_quality << std::endl
              << "        png_level: " << png_level << std::endl
              << "     depth_method: " << depth_method << std::endl
              << "     depth_device: " << depth_dev << std::endl
              << "       reg_method: " << reg_method << std::endl
              << "       reg_devive: " << reg_dev << std::endl
              << "        max_depth: " << maxDepth << std::endl
              << "        min_depth: " << minDepth << std::endl
              << "       queue_size: " << queueSize << std::endl
              << " bilateral_filter: " << (bilateral_filter ? "true" : "false") << std::endl
              << "edge_aware_filter: " << (edge_aware_filter ? "true" : "false") << std::endl
              << "       publish_tf: " << (publishTF ? "true" : "false") << std::endl
              << "     base_name_tf: " << baseNameTF << std::endl
              << "   worker_threads: " << worker_threads << std::endl << std::endl;

    deltaT = fps_limit > 0 ? 1.0 / fps_limit : 0.0;

    if(calib_path.empty() || calib_path.back() != '/')
    {
      calib_path += '/';
    }

    initCompression(jpeg_quality, png_level, use_png);

    if(!initPipeline(depth_method, depth_dev, bilateral_filter, edge_aware_filter, minDepth, maxDepth))
    {
      return false;
    }

    if(!initDevice(sensor))
    {
      return false;
    }

    initCalibration(calib_path, sensor);

    if(!initRegistration(reg_method, reg_dev, maxDepth))
    {
      device->close();
      delete listenerIrDepth;
      delete listenerColor;
      return false;
    }

    createCameraInfo();
    initTopics(queueSize, base_name);

    return true;
  }

  bool initRegistration(const std::string &method, const int32_t device, const double maxDepth)
  {
    DepthRegistration::Method reg;

    if(method == "default")
    {
      reg = DepthRegistration::DEFAULT;
    }
    else if(method == "cpu")
    {
#ifdef DEPTH_REG_CPU
      reg = DepthRegistration::CPU;
#else
      std::cerr << "CPU registration is not available!" << std::endl;
      return false;
#endif
    }
    else if(method == "opencl")
    {
#ifdef DEPTH_REG_OPENCL
      reg = DepthRegistration::OPENCL;
#else
      std::cerr << "OpenCL registration is not available!" << std::endl;
      return false;
#endif
    }
    else
    {
      std::cerr << "Unknown registration method: " << method << std::endl;
      return false;
    }

    depthRegLowRes = DepthRegistration::New(reg);
    depthRegHighRes = DepthRegistration::New(reg);

    if(!depthRegLowRes->init(cameraMatrixLowRes, sizeLowRes, cameraMatrixDepth, sizeIr, distortionDepth, rotation, translation, 0.5f, maxDepth, device) ||
       !depthRegHighRes->init(cameraMatrixColor, sizeColor, cameraMatrixDepth, sizeIr, distortionDepth, rotation, translation, 0.5f, maxDepth, device))
    {
      delete depthRegLowRes;
      delete depthRegHighRes;
      return false;
    }

    registration = new libfreenect2::Registration(irParams, colorParams);

    return true;
  }

  bool initPipeline(const std::string &method, const int32_t device, const bool bilateral_filter, const bool edge_aware_filter, const double minDepth, const double maxDepth)
  {
    if(method == "default")
    {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
      packetPipeline = new libfreenect2::OpenCLPacketPipeline(device);
#elif defined(LIBFREENECT2_WITH_OPENGL_SUPPORT)
      packetPipeline = new libfreenect2::OpenGLPacketPipeline();
#else
      packetPipeline = new libfreenect2::CpuPacketPipeline();
#endif
    }
    else if(method == "cpu")
    {
      packetPipeline = new libfreenect2::CpuPacketPipeline();
    }
    else if(method == "opencl")
    {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
      packetPipeline = new libfreenect2::OpenCLPacketPipeline(device);
#else
      std::cerr << "OpenCL depth processing is not available!" << std::endl;
      return false;
#endif
    }
    else if(method == "opengl")
    {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
      packetPipeline = new libfreenect2::OpenGLPacketPipeline();
#else
      std::cerr << "OpenGL depth processing is not available!" << std::endl;
      return false;
#endif
    }
    else
    {
      std::cerr << "Unknown depth processing method: " << method << std::endl;
      return false;
    }

    libfreenect2::DepthPacketProcessor::Config config;
    config.EnableBilateralFilter = bilateral_filter;
    config.EnableEdgeAwareFilter = edge_aware_filter;
    config.MinDepth = minDepth;
    config.MaxDepth = maxDepth;
    packetPipeline->getDepthPacketProcessor()->setConfiguration(config);
    return true;
  }

  void initCompression(const int32_t jpegQuality, const int32_t pngLevel, const bool use_png)
  {
    compressionParams.resize(7, 0);
    compressionParams[0] = CV_IMWRITE_JPEG_QUALITY;
    compressionParams[1] = jpegQuality;
    compressionParams[2] = CV_IMWRITE_PNG_COMPRESSION;
    compressionParams[3] = pngLevel;
    compressionParams[4] = CV_IMWRITE_PNG_STRATEGY;
    compressionParams[5] = CV_IMWRITE_PNG_STRATEGY_RLE;
    compressionParams[6] = 0;

    if(use_png)
    {
      compression16BitExt = ".png";
      compression16BitString = sensor_msgs::image_encodings::TYPE_16UC1 + "; png compressed";
    }
    else
    {
      compression16BitExt = ".tif";
      compression16BitString = sensor_msgs::image_encodings::TYPE_16UC1 + "; tiff compressed";
    }
  }

  void initTopics(const int32_t queueSize, const std::string &base_name)
  {
    std::vector<std::string> topics(COUNT);
    topics[IR_SD] = K2_TOPIC_SD K2_TOPIC_IMAGE_IR;
    topics[IR_SD_RECT] = K2_TOPIC_SD K2_TOPIC_IMAGE_IR K2_TOPIC_IMAGE_RECT;

    topics[DEPTH_SD] = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH;
    topics[DEPTH_SD_RECT] = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    topics[DEPTH_HD] = K2_TOPIC_HD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    topics[DEPTH_QHD] = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;

    topics[COLOR_SD_RECT] = K2_TOPIC_SD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;

    topics[COLOR_HD] = K2_TOPIC_HD K2_TOPIC_IMAGE_COLOR;
    topics[COLOR_HD_RECT] = K2_TOPIC_HD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
    topics[COLOR_QHD] = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR;
    topics[COLOR_QHD_RECT] = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;

    topics[MONO_HD] = K2_TOPIC_HD K2_TOPIC_IMAGE_MONO;
    topics[MONO_HD_RECT] = K2_TOPIC_HD K2_TOPIC_IMAGE_MONO K2_TOPIC_IMAGE_RECT;
    topics[MONO_QHD] = K2_TOPIC_QHD K2_TOPIC_IMAGE_MONO;
    topics[MONO_QHD_RECT] = K2_TOPIC_QHD K2_TOPIC_IMAGE_MONO K2_TOPIC_IMAGE_RECT;

    imagePubs.resize(COUNT);
    compressedPubs.resize(COUNT);
    ros::SubscriberStatusCallback cb = boost::bind(&Kinect2Bridge::callbackStatus, this);

    for(size_t i = 0; i < COUNT; ++i)
    {
      imagePubs[i] = nh.advertise<sensor_msgs::Image>(base_name + topics[i], queueSize, cb, cb);
      compressedPubs[i] = nh.advertise<sensor_msgs::CompressedImage>(base_name + topics[i] + K2_TOPIC_COMPRESSED, queueSize, cb, cb);
    }
    infoHDPub = nh.advertise<sensor_msgs::CameraInfo>(base_name + K2_TOPIC_HD + K2_TOPIC_INFO, queueSize, cb, cb);
    infoQHDPub = nh.advertise<sensor_msgs::CameraInfo>(base_name + K2_TOPIC_QHD + K2_TOPIC_INFO, queueSize, cb, cb);
    infoIRPub = nh.advertise<sensor_msgs::CameraInfo>(base_name + K2_TOPIC_SD + K2_TOPIC_INFO, queueSize, cb, cb);
  }

  bool initDevice(std::string &sensor)
  {
    bool deviceFound = false;
    const int numOfDevs = freenect2.enumerateDevices();

    if(numOfDevs <= 0)
    {
      std::cerr << "Error: no Kinect2 devices found!" << std::endl;
      delete packetPipeline;
      return false;
    }

    if(sensor.empty())
    {
      sensor = freenect2.getDefaultDeviceSerialNumber();
    }

    std::cout << "Kinect2 devices found: " << std::endl;
    for(int i = 0; i < numOfDevs; ++i)
    {
      const std::string &s = freenect2.getDeviceSerialNumber(i);
      deviceFound = deviceFound || s == sensor;
      std::cout << "  " << i << ": " << s << (s == sensor ? " (selected)" : "") << std::endl;
    }

    if(!deviceFound)
    {
      std::cerr << "Error: Device with serial '" << sensor << "' not found!" << std::endl;
      delete packetPipeline;
      return false;
    }

    device = freenect2.openDevice(sensor, packetPipeline);

    if(device == 0)
    {
      std::cout << "no device connected or failure opening the default one!" << std::endl;
      return false;
    }

    listenerColor = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color);
    listenerIrDepth = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);

    device->setColorFrameListener(listenerColor);
    device->setIrAndDepthFrameListener(listenerIrDepth);

    std::cout << std::endl << "starting kinect2" << std::endl << std::endl;
    device->start();

    std::cout << std::endl << "device serial: " << sensor << std::endl;
    std::cout << "device firmware: " << device->getFirmwareVersion() << std::endl;

    colorParams = device->getColorCameraParams();
    irParams = device->getIrCameraParams();

    device->stop();

    std::cout << std::endl << "default ir camera parameters: " << std::endl;
    std::cout << "fx " << irParams.fx << ", fy " << irParams.fy << ", cx " << irParams.cx << ", cy " << irParams.cy << std::endl;
    std::cout << "k1 " << irParams.k1 << ", k2 " << irParams.k2 << ", p1 " << irParams.p1 << ", p2 " << irParams.p2 << ", k3 " << irParams.k3 << std::endl;

    std::cout << std::endl << "default color camera parameters: " << std::endl;
    std::cout << "fx " << colorParams.fx << ", fy " << colorParams.fy << ", cx " << colorParams.cx << ", cy " << colorParams.cy << std::endl;

    cameraMatrixColor = cv::Mat::eye(3, 3, CV_64F);
    distortionColor = cv::Mat::zeros(1, 5, CV_64F);

    cameraMatrixColor.at<double>(0, 0) = colorParams.fx;
    cameraMatrixColor.at<double>(1, 1) = colorParams.fy;
    cameraMatrixColor.at<double>(0, 2) = colorParams.cx;
    cameraMatrixColor.at<double>(1, 2) = colorParams.cy;
    cameraMatrixColor.at<double>(2, 2) = 1;

    cameraMatrixIr = cv::Mat::eye(3, 3, CV_64F);
    distortionIr = cv::Mat::zeros(1, 5, CV_64F);

    cameraMatrixIr.at<double>(0, 0) = irParams.fx;
    cameraMatrixIr.at<double>(1, 1) = irParams.fy;
    cameraMatrixIr.at<double>(0, 2) = irParams.cx;
    cameraMatrixIr.at<double>(1, 2) = irParams.cy;
    cameraMatrixIr.at<double>(2, 2) = 1;

    distortionIr.at<double>(0, 0) = irParams.k1;
    distortionIr.at<double>(0, 1) = irParams.k2;
    distortionIr.at<double>(0, 2) = irParams.p1;
    distortionIr.at<double>(0, 3) = irParams.p2;
    distortionIr.at<double>(0, 4) = irParams.k3;

    cameraMatrixDepth = cameraMatrixIr.clone();
    distortionDepth = distortionIr.clone();

    rotation = cv::Mat::eye(3, 3, CV_64F);
    translation = cv::Mat::zeros(3, 1, CV_64F);
    return true;
  }

  void initCalibration(const std::string &calib_path, const std::string &sensor)
  {
    std::string calibPath = calib_path + sensor + '/';

    struct stat fileStat;
    bool calibDirNotFound = stat(calibPath.c_str(), &fileStat) != 0 || !S_ISDIR(fileStat.st_mode);
    if(calibDirNotFound || !loadCalibrationFile(calibPath + K2_CALIB_COLOR, cameraMatrixColor, distortionColor))
    {
      std::cerr << "using sensor defaults for color intrinsic parameters." << std::endl;
    }

    if(calibDirNotFound || !loadCalibrationFile(calibPath + K2_CALIB_IR, cameraMatrixDepth, distortionDepth))
    {
      std::cerr << "using sensor defaults for ir intrinsic parameters." << std::endl;
    }

    if(calibDirNotFound || !loadCalibrationPoseFile(calibPath + K2_CALIB_POSE, rotation, translation))
    {
      std::cerr << "using defaults for rotation and translation." << std::endl;
    }

    if(calibDirNotFound || !loadCalibrationDepthFile(calibPath + K2_CALIB_DEPTH, depthShift))
    {
      std::cerr << "using defaults for depth shift." << std::endl;
      depthShift = 0.0;
    }

    cameraMatrixLowRes = cameraMatrixColor.clone();
    cameraMatrixLowRes.at<double>(0, 0) /= 2;
    cameraMatrixLowRes.at<double>(1, 1) /= 2;
    cameraMatrixLowRes.at<double>(0, 2) /= 2;
    cameraMatrixLowRes.at<double>(1, 2) /= 2;

    const int mapType = CV_16SC2;
    cv::initUndistortRectifyMap(cameraMatrixColor, distortionColor, cv::Mat(), cameraMatrixColor, sizeColor, mapType, map1Color, map2Color);
    cv::initUndistortRectifyMap(cameraMatrixIr, distortionIr, cv::Mat(), cameraMatrixIr, sizeIr, mapType, map1Ir, map2Ir);
    cv::initUndistortRectifyMap(cameraMatrixColor, distortionColor, cv::Mat(), cameraMatrixLowRes, sizeLowRes, mapType, map1LowRes, map2LowRes);

    std::cout << std::endl << "camera parameters used:" << std::endl
              << "camera matrix color:" << std::endl << cameraMatrixColor << std::endl
              << "distortion coefficients color:" << std::endl << distortionColor << std::endl
              << "camera matrix ir:" << std::endl << cameraMatrixIr << std::endl
              << "distortion coefficients ir:" << std::endl << distortionIr << std::endl
              << "camera matrix depth:" << std::endl << cameraMatrixDepth << std::endl
              << "distortion coefficients depth:" << std::endl << distortionDepth << std::endl
              << "rotation:" << std::endl << rotation << std::endl
              << "translation:" << std::endl << translation << std::endl
              << "depth shift:" << std::endl << depthShift << std::endl << std::endl;
  }

  bool loadCalibrationFile(const std::string &filename, cv::Mat &cameraMatrix, cv::Mat &distortion) const
  {
    cv::FileStorage fs;
    if(fs.open(filename, cv::FileStorage::READ))
    {
      fs[K2_CALIB_CAMERA_MATRIX] >> cameraMatrix;
      fs[K2_CALIB_DISTORTION] >> distortion;
      fs.release();
    }
    else
    {
      std::cerr << "can't open calibration file: " << filename << std::endl;
      return false;
    }
    return true;
  }

  bool loadCalibrationPoseFile(const std::string &filename, cv::Mat &rotation, cv::Mat &translation) const
  {
    cv::FileStorage fs;
    if(fs.open(filename, cv::FileStorage::READ))
    {
      fs[K2_CALIB_ROTATION] >> rotation;
      fs[K2_CALIB_TRANSLATION] >> translation;
      fs.release();
    }
    else
    {
      std::cerr << "can't open calibration pose file: " << filename << std::endl;
      return false;
    }
    return true;
  }

  bool loadCalibrationDepthFile(const std::string &filename, double &depthShift) const
  {
    cv::FileStorage fs;
    if(fs.open(filename, cv::FileStorage::READ))
    {
      fs[K2_CALIB_DEPTH_SHIFT] >> depthShift;
      fs.release();
    }
    else
    {
      std::cerr << "can't open calibration depth file: " << filename << std::endl;
      return false;
    }
    return true;
  }

  void createCameraInfo()
  {
    cv::Mat projColor = cv::Mat::zeros(3, 4, CV_64F);
    cv::Mat projIr = cv::Mat::zeros(3, 4, CV_64F);
    cv::Mat projLowRes = cv::Mat::zeros(3, 4, CV_64F);

    cameraMatrixColor.copyTo(projColor(cv::Rect(0, 0, 3, 3)));
    cameraMatrixIr.copyTo(projIr(cv::Rect(0, 0, 3, 3)));
    cameraMatrixLowRes.copyTo(projLowRes(cv::Rect(0, 0, 3, 3)));

    createCameraInfo(sizeColor, cameraMatrixColor, distortionColor, cv::Mat::eye(3, 3, CV_64F), projColor, infoHD);
    createCameraInfo(sizeIr, cameraMatrixIr, distortionIr, cv::Mat::eye(3, 3, CV_64F), projIr, infoIR);
    createCameraInfo(sizeLowRes, cameraMatrixLowRes, distortionColor, cv::Mat::eye(3, 3, CV_64F), projLowRes, infoQHD);
  }

  void createCameraInfo(const cv::Size &size, const cv::Mat &cameraMatrix, const cv::Mat &distortion, const cv::Mat &rotation, const cv::Mat &projection, sensor_msgs::CameraInfo &cameraInfo) const
  {
    cameraInfo.height = size.height;
    cameraInfo.width = size.width;

    const double *itC = cameraMatrix.ptr<double>(0, 0);
    for(size_t i = 0; i < 9; ++i, ++itC)
    {
      cameraInfo.K[i] = *itC;
    }

    const double *itR = rotation.ptr<double>(0, 0);
    for(size_t i = 0; i < 9; ++i, ++itR)
    {
      cameraInfo.R[i] = *itR;
    }

    const double *itP = projection.ptr<double>(0, 0);
    for(size_t i = 0; i < 12; ++i, ++itP)
    {
      cameraInfo.P[i] = *itP;
    }

    cameraInfo.distortion_model = "plumb_bob";
    cameraInfo.D.resize(distortion.cols);
    const double *itD = distortion.ptr<double>(0, 0);
    for(size_t i = 0; i < (size_t)distortion.cols; ++i, ++itD)
    {
      cameraInfo.D[i] = *itD;
    }
  }

  void callbackStatus()
  {
    lockStatus.lock();
    clientConnected = updateStatus();

    if(clientConnected && !deviceActive)
    {
      std::cout << "[kinect2_bridge] client connected. starting device..." << std::endl << std::flush;
      deviceActive = true;
      device->start();
    }
    else if(!clientConnected && deviceActive)
    {
      std::cout << "[kinect2_bridge] no clients connected. stopping device..." << std::endl << std::flush;
      deviceActive = false;
      device->stop();
    }
    lockStatus.unlock();
  }

  bool updateStatus()
  {
    bool any = false;
    for(size_t i = 0; i < COUNT; ++i)
    {
      Status s = UNSUBCRIBED;
      if(imagePubs[i].getNumSubscribers() > 0)
      {
        s = RAW;
      }
      if(compressedPubs[i].getNumSubscribers() > 0)
      {
        s = s == RAW ? BOTH : COMPRESSED;
      }

      status[i] = s;
      any = any || s != UNSUBCRIBED;
    }
    return any || infoHDPub.getNumSubscribers() > 0 || infoQHDPub.getNumSubscribers() > 0 || infoIRPub.getNumSubscribers() > 0;
  }

  void main()
  {
    std::cout << "[kinect2_bridge] waiting for clients to connect" << std::endl << std::endl;
    double nextFrame = ros::Time::now().toSec() + deltaT;
    double fpsTime = ros::Time::now().toSec();
    size_t oldFrameIrDepth = 0, oldFrameColor = 0;
    nextColor = true;
    nextIrDepth = true;

    for(; running && ros::ok();)
    {
      if(!deviceActive)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        fpsTime =  ros::Time::now().toSec();
        nextFrame = fpsTime + deltaT;
        continue;
      }

      double now = ros::Time::now().toSec();

      if(now - fpsTime >= 3.0)
      {
        fpsTime = now - fpsTime;
        size_t framesIrDepth = frameIrDepth - oldFrameIrDepth;
        size_t framesColor = frameColor - oldFrameColor;
        oldFrameIrDepth = frameIrDepth;
        oldFrameColor = frameColor;

        lockTime.lock();
        double tColor = elapsedTimeColor;
        double tDepth = elapsedTimeIrDepth;
        elapsedTimeColor = 0;
        elapsedTimeIrDepth = 0;
        lockTime.unlock();

        std::cout << "[kinect2_bridge] depth processing: ~" << framesIrDepth / tDepth << "Hz (" << (tDepth / framesIrDepth) * 1000 << "ms) publishing rate: ~" << framesIrDepth / fpsTime << "Hz" << std::endl
                  << "[kinect2_bridge] color processing: ~" << framesColor / tColor << "Hz (" << (tColor / framesColor) * 1000 << "ms) publishing rate: ~" << framesColor / fpsTime << "Hz" << std::endl << std::flush;
        fpsTime = now;
      }

      if(now >= nextFrame)
      {
        nextColor = true;
        nextIrDepth = true;
        nextFrame += deltaT;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(10));

      if(!deviceActive)
      {
        oldFrameIrDepth = frameIrDepth;
        oldFrameColor = frameColor;
        lockTime.lock();
        elapsedTimeColor = 0;
        elapsedTimeIrDepth = 0;
        lockTime.unlock();
        continue;
      }
    }
  }

  void threadDispatcher(const size_t id)
  {
    const size_t checkFirst = id % 2;
    bool processedFrame = false;
    int oldNice = nice(0);
    oldNice = nice(19 - oldNice);

    for(; running && ros::ok();)
    {
      processedFrame = false;

      for(size_t i = 0; i < 2; ++i)
      {
        if(i == checkFirst)
        {
          if(nextIrDepth && lockIrDepth.try_lock())
          {
            nextIrDepth = false;
            receiveIrDepth();
            processedFrame = true;
          }
        }
        else
        {
          if(nextColor && lockColor.try_lock())
          {
            nextColor = false;
            receiveColor();
            processedFrame = true;
          }
        }
      }

      if(!processedFrame)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
  }

  void receiveIrDepth()
  {
    libfreenect2::FrameMap frames;
    cv::Mat depth, ir;
    std_msgs::Header header;
    std::vector<cv::Mat> images(COUNT);
    std::vector<Status> status = this->status;
    size_t frame;

//
    cv::Mat depth_median;
//

    if(!receiveFrames(listenerIrDepth, frames))
    {
      lockIrDepth.unlock();
      return;
    }
    double now = ros::Time::now().toSec();

    header = createHeader(lastDepth, lastColor);

    libfreenect2::Frame *irFrame = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depthFrame = frames[libfreenect2::Frame::Depth];

    ir = cv::Mat(irFrame->height, irFrame->width, CV_32FC1, irFrame->data);
    depth = cv::Mat(depthFrame->height, depthFrame->width, CV_32FC1, depthFrame->data);///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
    //cv::Mat addDepth(depth_median.rows, depth_median.cols, CV_32S, cv::Scalar(0));//グローバル変数にした

    cv::medianBlur(depth, depth_median, 3);//足す前にメディアンかける

    addCount++;
    if(addCount <= NUMOFDEPTH){
      std::cout << addCount << std::endl;
      add2Pictures(addDepth, depth_median);
    }
    if(addCount == NUMOFDEPTH){
      dividePicture(addDepth);
      convert32Sto16U(addDepth, addDepth_16U);
    }

	if(addCount == NUMOFDEPTH){
	  cv::Mat addDepthDisp;
	  
	  cv::medianBlur(addDepth_16U, addDepth_16U, 3);
	  //cv::medianBlur(addDepthInt, addDepthInt, 3);//メディアンかける
	  dispDepth(addDepth_16U, addDepthDisp, DEPTHMAX, DEPTHMIN);


	  convert16Uto8U(addDepth_16U, addDepth_16U_8Uscaled);
	  addDepth_16U_8Uscaled.convertTo(addDepth_8U, CV_8U);
	  cv::Canny(addDepth_8U, addDepth_Canny01, 180, 5);//180, 5
	  cv::Canny(addDepth_8U, addDepth_Canny02, 180, 11);
	  cv::Canny(addDepth_8U, addDepth_Canny03, 180, 12);
	  cv::Canny(addDepth_8U, addDepth_Canny04, 180, 13);
	  cv::Canny(addDepth_8U, addDepth_Canny05, 180, 14);
	  cv::imshow("Canny edge01", addDepth_Canny01);
	  cv::imshow("Canny edge02", addDepth_Canny02);
	  cv::imshow("Canny edge03", addDepth_Canny03);
	  cv::imshow("Canny edge04", addDepth_Canny04);
	  cv::imshow("Canny edge05", addDepth_Canny05);
	  cv::imshow("add Depth", addDepthDisp);
          cv::imwrite("addDepth.png", addDepthDisp);	

	}


    dispCenterPixel(depth);
    //cv::imshow("bridge depth", depth);//imshowでは32Fの場合、画素値を255倍にしてしまうので影になる部分以外は真っ白になる
    

    cv::Mat depth_16;
    depth.convertTo(depth_16, CV_16U);
    cv::imwrite("depth_16_01.png", depth_16);


      int key = cv::waitKey(1);
      switch(key & 0xFF)
      {
      case 27:
      case 'q':
        running = false;
        break;
      }
    //ここまで追加
*/





    frame = frameIrDepth++;
    lockIrDepth.unlock();

    processIrDepth(ir, depth, images, status, depthFrame);

    publishImages(images, header, status, frame, pubFrameIrDepth, IR_SD, COLOR_HD);

    listenerIrDepth->release(frames);

    double elapsed = ros::Time::now().toSec() - now;
    lockTime.lock();
    elapsedTimeIrDepth += elapsed;
    lockTime.unlock();
  }








//後から加えた関数
  void dispCenterPixel(cv::Mat &in1){
    //in1　深度32F
    cv::Mat tmp = cv::Mat(in1.rows, in1.cols, CV_32F);//一旦tmpに入れて最後にoutに入れる

    for(int r = 0; r < in1.rows; ++r){
      const float *it1In = in1.ptr<float>(r);
      for(int c = 0; c < in1.cols; ++c, ++it1In){
	if(r == in1.rows/2 && c == in1.rows/2){
	  std::cout << "depth's center pixel :" << *it1In << std::endl;
	}
      }
    }
  }

//加算
  void add2Pictures(cv::Mat &in1, const cv::Mat &in2){
    //in1がaddDepthで深度32S、in2がdepth_medianで深度32F
    //in2を1000倍してin1に加算していく
    for(int r = 0; r < in1.rows; ++r){
      int32_t *it1In = in1.ptr<int32_t>(r);
      const float *it2In = in2.ptr<float>(r);

      for(int c = 0; c < in1.cols; ++c, ++it1In, ++it2In){
        *it1In = *it1In + (*it2In * 1000);
	if(r == in1.rows/2 && c == in1.rows/2){
	  std::cout << "add2Pictures center pixel   add:" << *it1In << "  origin:" << *it2In << std::endl;
	}
      }
    }
  }

//割る
  void dividePicture(cv::Mat &in1){
    for(int r = 0; r < in1.rows; r++){
      int32_t *it1In = in1.ptr<int32_t>(r);
      for(int c = 0; c < in1.cols; c++, it1In++){
	*it1In = *it1In / NUMOFDEPTH;
	if(r == in1.rows/2 && c == in1.rows/2){
          std::cout << "divided add depth:" << *it1In << std::endl;
	}
      }
    }
  }

//スケール調整
  void dispDepth(const cv::Mat &in, cv::Mat &out, const float maxValue, const float minValue = 500.0f)
  {
//inは16U
    cv::Mat tmp = cv::Mat(in.rows, in.cols, CV_8U);
    const uint32_t maxInt = 255;

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

  void convert32Sto16U(const cv::Mat &in1, cv::Mat &out){//100で割る
    for(int r = 0; r < in1.rows; r++){
      const int32_t *it1In = in1.ptr<int32_t>(r);
      uint16_t *itOut = out.ptr<uint16_t>(r);
      for(int c = 0; c < in1.cols; c++, it1In++, itOut++){
	*itOut = *it1In / 100;
        if(r == in1.rows/2 && c == in1.rows/2){
          std::cout << "convert32Sto16U center pixel   in:" << *it1In << "  out:" << *itOut << std::endl;
	}
      }
    }
  }

  void convert16Uto8U(const cv::Mat &in1, cv::Mat &out){//16Uのin1を8UのoutにMAXVALUEとMINVALUEに基づいて変換

    for(int r = 0; r < in1.rows; r++){
      const uint16_t *it1In = in1.ptr<uint16_t>(r);
      uint16_t *itOut = out.ptr<uint16_t>(r);
      for(int c = 0; c < in1.cols; c++, it1In++, itOut++){
	if(*it1In > DEPTHMAX){
	  *itOut = (uint16_t)255;
	}else if(*it1In < DEPTHMIN){
	  *itOut = (uint16_t)0;
	}else{
	  *itOut = (uint16_t)std::min((*it1In - DEPTHMIN) * (255/(DEPTHMAX - DEPTHMIN)), 255.0f);//深度の分解能は (DEPTHMAX-DEPTHMIN)/255
	  if(r == in1.rows/2 && c == in1.rows/2){
	    std::cout << "convert16Uto8U center pixel   in:" << *it1In << "  out:" << *itOut << std::endl;
	  }
	}
      }
    }
  }








  void receiveColor()
  {
    libfreenect2::FrameMap frames;
    cv::Mat color;
    std_msgs::Header header;
    std::vector<cv::Mat> images(COUNT);
    std::vector<Status> status = this->status;
    size_t frame;

    if(!receiveFrames(listenerColor, frames))
    {
      lockColor.unlock();
      return;
    }
    double now = ros::Time::now().toSec();

    header = createHeader(lastColor, lastDepth);

    libfreenect2::Frame *colorFrame = frames[libfreenect2::Frame::Color];

    color = cv::Mat(colorFrame->height, colorFrame->width, CV_8UC4, colorFrame->data);

    frame = frameColor++;
    lockColor.unlock();

    processColor(color, images, status, colorFrame);

    publishImages(images, header, status, frame, pubFrameColor, COLOR_HD, COUNT);

    listenerColor->release(frames);

    double elapsed = ros::Time::now().toSec() - now;
    lockTime.lock();
    elapsedTimeColor += elapsed;
    lockTime.unlock();
  }

  bool receiveFrames(libfreenect2::SyncMultiFrameListener *listener, libfreenect2::FrameMap &frames)
  {
    bool newFrames = false;
    for(; !newFrames;)
    {
#ifdef LIBFREENECT2_THREADING_STDLIB
      newFrames = listener->waitForNewFrame(frames, 1000);
#else
      newFrames = true;
      listener->waitForNewFrame(frames);
#endif
      if(!deviceActive || !running || !ros::ok())
      {
        if(newFrames)
        {
          listener->release(frames);
        }
        return false;
      }
    }
    return true;
  }

  std_msgs::Header createHeader(ros::Time &last, ros::Time &other)
  {
    ros::Time timestamp = ros::Time::now();
    lockSync.lock();
    if(other.isZero())
    {
      last = timestamp;
    }
    else
    {
      timestamp = other;
      other = ros::Time(0, 0);
    }
    lockSync.unlock();

    std_msgs::Header header;
    header.seq = 0;
    header.stamp = timestamp;
    return header;
  }

  void processIrDepth(const cv::Mat &ir, const cv::Mat &depth, std::vector<cv::Mat> &images, const std::vector<Status> &status, libfreenect2::Frame *depthFrame)
  {
    // COLOR registered to depth
    if(status[COLOR_SD_RECT])
    {
      cv::Mat tmp;
      libfreenect2::Frame undistorted(sizeIr.width, sizeIr.height, 4), registered(sizeIr.width, sizeIr.height, 4);
      lockColorFrame.lock();
      registration->apply(&colorFrame, depthFrame, &undistorted, &registered);
      lockColorFrame.unlock();
      cv::flip(cv::Mat(sizeIr, CV_8UC4, registered.data), tmp, 1);
      cv::cvtColor(tmp, images[COLOR_SD_RECT], CV_BGRA2BGR);
    }

    // IR
    if(status[IR_SD] || status[IR_SD_RECT])
    {
      ir.convertTo(images[IR_SD], CV_16U);
      cv::flip(images[IR_SD], images[IR_SD], 1);
    }
    if(status[IR_SD_RECT])
    {
      cv::remap(images[IR_SD], images[IR_SD_RECT], map1Ir, map2Ir, cv::INTER_AREA);
    }

    // DEPTH
    cv::Mat depthShifted;
    if(status[DEPTH_SD])
    {
      depth.convertTo(images[DEPTH_SD], CV_16U, 1);////////////////////////////////////////////////////////////////////
      cv::flip(images[DEPTH_SD], images[DEPTH_SD], 1);
    }
    if(status[DEPTH_SD_RECT] || status[DEPTH_QHD] || status[DEPTH_HD])
    {
      //depth.convertTo(depthShifted, CV_16U, 1, depthShift);
      depth.convertTo(depthShifted, CV_16U, 10, depthShift);//分解能0.1mmにしたいので１０倍
      std::cout << "depthShift: " << depthShift << std::endl;
      //depth.convertTo(depthShifted, CV_32S, 10, depthShift);//分解能0.1mmにしたいので１０倍
      cv::flip(depthShifted, depthShifted, 1);
    }
    if(status[DEPTH_SD_RECT])
    {
      cv::remap(depthShifted, images[DEPTH_SD_RECT], map1Ir, map2Ir, cv::INTER_NEAREST);
    }
    if(status[DEPTH_QHD])
    {
      lockRegLowRes.lock();
      depthRegLowRes->registerDepth(depthShifted, images[DEPTH_QHD]);
      lockRegLowRes.unlock();
    }
    if(status[DEPTH_HD])
    {
      lockRegHighRes.lock();
      depthRegHighRes->registerDepth(depthShifted, images[DEPTH_HD]);
      lockRegHighRes.unlock();
    }
  }

  void processColor(const cv::Mat &color, std::vector<cv::Mat> &images, const std::vector<Status> &status, libfreenect2::Frame *colorFrame)
  {
    if(status[COLOR_SD_RECT])
    {
      this->colorFrame.timestamp = colorFrame->timestamp;
      this->colorFrame.sequence = colorFrame->sequence;
      size_t size = colorFrame->height * colorFrame->width * colorFrame->bytes_per_pixel;
      lockColorFrame.lock();
      memcpy(this->colorFrame.data, colorFrame->data, size);
      lockColorFrame.unlock();
    }

    // COLOR
    if(status[COLOR_HD] || status[COLOR_HD_RECT] || status[COLOR_QHD] || status[COLOR_QHD_RECT] ||
       status[MONO_HD] || status[MONO_HD_RECT] || status[MONO_QHD] || status[MONO_QHD_RECT])
    {
      cv::Mat tmp;
      cv::flip(color, tmp, 1);
      cv::cvtColor(tmp, images[COLOR_HD], CV_BGRA2BGR);
    }
    if(status[COLOR_HD_RECT] || status[MONO_HD_RECT])
    {
      cv::remap(images[COLOR_HD], images[COLOR_HD_RECT], map1Color, map2Color, cv::INTER_AREA);
    }
    if(status[COLOR_QHD] || status[MONO_QHD])
    {
      cv::resize(images[COLOR_HD], images[COLOR_QHD], sizeLowRes, 0, 0, cv::INTER_AREA);
    }
    if(status[COLOR_QHD_RECT] || status[MONO_QHD_RECT])
    {
      cv::remap(images[COLOR_HD], images[COLOR_QHD_RECT], map1LowRes, map2LowRes, cv::INTER_AREA);
    }

    // MONO
    if(status[MONO_HD])
    {
      cv::cvtColor(images[COLOR_HD], images[MONO_HD], CV_BGR2GRAY);
    }
    if(status[MONO_HD_RECT])
    {
      cv::cvtColor(images[COLOR_HD_RECT], images[MONO_HD_RECT], CV_BGR2GRAY);
    }
    if(status[MONO_QHD])
    {
      cv::cvtColor(images[COLOR_QHD], images[MONO_QHD], CV_BGR2GRAY);
    }
    if(status[MONO_QHD_RECT])
    {
      cv::cvtColor(images[COLOR_QHD_RECT], images[MONO_QHD_RECT], CV_BGR2GRAY);
    }
  }

  void publishImages(const std::vector<cv::Mat> &images, const std_msgs::Header &header, const std::vector<Status> &status, const size_t frame, size_t &pubFrame, const size_t begin, const size_t end)
  {
    std::vector<sensor_msgs::ImagePtr> imageMsgs(COUNT);
    std::vector<sensor_msgs::CompressedImagePtr> compressedMsgs(COUNT);
    sensor_msgs::CameraInfoPtr infoHDMsg,  infoQHDMsg,  infoIRMsg;
    std_msgs::Header _header = header;

    if(begin < COLOR_HD)
    {
      _header.frame_id = baseNameTF + K2_TF_IR_OPT_FRAME;

      infoIRMsg = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo);
      *infoIRMsg = infoIR;
      infoIRMsg->header = _header;
    }
    else
    {
      _header.frame_id = baseNameTF + K2_TF_RGB_OPT_FRAME;

      infoHDMsg = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo);
      *infoHDMsg = infoHD;
      infoHDMsg->header = _header;

      infoQHDMsg = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo);
      *infoQHDMsg = infoQHD;
      infoQHDMsg->header = _header;

    }

    for(size_t i = begin; i < end; ++i)
    {
      if(i < DEPTH_HD || i == COLOR_SD_RECT)
      {
        _header.frame_id = baseNameTF + K2_TF_IR_OPT_FRAME;
      }
      else
      {
        _header.frame_id = baseNameTF + K2_TF_RGB_OPT_FRAME;
      }
      std::cout << "status[" << i << "]:" << status[i] << "  Image(" << i << "):" << Image(i) << std::endl;//
      switch(status[i])
      {
      case UNSUBCRIBED:
        break;
      case RAW:
        imageMsgs[i] = sensor_msgs::ImagePtr(new sensor_msgs::Image);
        createImage(images[i], _header, Image(i), *imageMsgs[i]);
        break;
      case COMPRESSED:
        compressedMsgs[i] = sensor_msgs::CompressedImagePtr(new sensor_msgs::CompressedImage);
        createCompressed(images[i], _header, Image(i), *compressedMsgs[i]);
        break;
      case BOTH:
        imageMsgs[i] = sensor_msgs::ImagePtr(new sensor_msgs::Image);
        compressedMsgs[i] = sensor_msgs::CompressedImagePtr(new sensor_msgs::CompressedImage);
        createImage(images[i], _header, Image(i), *imageMsgs[i]);
        createCompressed(images[i], _header, Image(i), *compressedMsgs[i]);
        break;
      }
    }

    while(frame != pubFrame)
    {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    lockPub.lock();
    for(size_t i = begin; i < end; ++i)
    {
      switch(status[i])
      {
      case UNSUBCRIBED:
        break;
      case RAW:
        imagePubs[i].publish(imageMsgs[i]);
        break;
      case COMPRESSED:
        compressedPubs[i].publish(compressedMsgs[i]);
        break;
      case BOTH:
        imagePubs[i].publish(imageMsgs[i]);
        compressedPubs[i].publish(compressedMsgs[i]);
        break;
      }
    }

    if(begin < COLOR_HD)
    {
      if(infoIRPub.getNumSubscribers() > 0)
      {
        infoIRPub.publish(infoIRMsg);
      }
    }
    else
    {
      if(infoHDPub.getNumSubscribers() > 0)
      {
        infoHDPub.publish(infoHDMsg);
      }
      if(infoQHDPub.getNumSubscribers() > 0)
      {
        infoQHDPub.publish(infoQHDMsg);
      }
    }

    ++pubFrame;
    lockPub.unlock();
  }

  void createImage(const cv::Mat &image, const std_msgs::Header &header, const Image type, sensor_msgs::Image &msgImage) const
  {
    size_t step, size;
    step = image.cols * image.elemSize();
    size = image.rows * step;

    switch(type)
    {
    case IR_SD:
    case IR_SD_RECT:
    case DEPTH_SD:
    case DEPTH_SD_RECT:
    case DEPTH_HD:
    case DEPTH_QHD:
      msgImage.encoding = sensor_msgs::image_encodings::TYPE_16UC1;///////////////////////////////////////////////
      break;
      //ここから修正点
      /*
    case DEPTH_SD_RECT:
    case DEPTH_HD:
    case DEPTH_QHD:
      msgImage.encoding = sensor_msgs::image_encodings::TYPE_32SC1;
      break;
      */
      //ここまで修正点
    case COLOR_SD_RECT:
    case COLOR_HD:
    case COLOR_HD_RECT:
    case COLOR_QHD:
    case COLOR_QHD_RECT:
      msgImage.encoding = sensor_msgs::image_encodings::BGR8;
      break;
    case MONO_HD:
    case MONO_HD_RECT:
    case MONO_QHD:
    case MONO_QHD_RECT:
      msgImage.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
      break;
    case COUNT:
      return;
    }

    msgImage.header = header;
    msgImage.height = image.rows;
    msgImage.width = image.cols;
    msgImage.is_bigendian = false;
    msgImage.step = step;
    msgImage.data.resize(size);
    memcpy(msgImage.data.data(), image.data, size);
  }

  void createCompressed(const cv::Mat &image, const std_msgs::Header &header, const Image type, sensor_msgs::CompressedImage &msgImage) const
  {
    msgImage.header = header;

    switch(type)
    {
    case IR_SD:
    case IR_SD_RECT:
    case DEPTH_SD:
    case DEPTH_SD_RECT:
    case DEPTH_HD:
    case DEPTH_QHD:
      msgImage.format = compression16BitString;
      cv::imencode(compression16BitExt, image, msgImage.data, compressionParams);
      break;
    case COLOR_SD_RECT:
    case COLOR_HD:
    case COLOR_HD_RECT:
    case COLOR_QHD:
    case COLOR_QHD_RECT:
      msgImage.format = sensor_msgs::image_encodings::BGR8 + "; jpeg compressed bgr8";
      cv::imencode(".jpg", image, msgImage.data, compressionParams);
      break;
    case MONO_HD:
    case MONO_HD_RECT:
    case MONO_QHD:
    case MONO_QHD_RECT:
      msgImage.format = sensor_msgs::image_encodings::TYPE_8UC1 + "; jpeg compressed ";
      cv::imencode(".jpg", image, msgImage.data, compressionParams);
      break;
    case COUNT:
      return;
    }
  }

  void publishStaticTF()
  {
    tf::TransformBroadcaster broadcaster;
    tf::StampedTransform stColorOpt, stIrOpt;
    ros::Time now = ros::Time::now();

    tf::Matrix3x3 rot(rotation.at<double>(0, 0), rotation.at<double>(0, 1), rotation.at<double>(0, 2),
                      rotation.at<double>(1, 0), rotation.at<double>(1, 1), rotation.at<double>(1, 2),
                      rotation.at<double>(2, 0), rotation.at<double>(2, 1), rotation.at<double>(2, 2));

    tf::Quaternion qZero;
    qZero.setRPY(0, 0, 0);
    tf::Vector3 trans(translation.at<double>(0), translation.at<double>(1), translation.at<double>(2));
    tf::Vector3 vZero(0, 0, 0);
    tf::Transform tIr(rot, trans), tZero(qZero, vZero);

    stColorOpt = tf::StampedTransform(tZero, now, baseNameTF + K2_TF_LINK, baseNameTF + K2_TF_RGB_OPT_FRAME);
    stIrOpt = tf::StampedTransform(tIr, now, baseNameTF + K2_TF_RGB_OPT_FRAME, baseNameTF + K2_TF_IR_OPT_FRAME);

    for(; running && ros::ok();)
    {
      now = ros::Time::now();
      stColorOpt.stamp_ = now;
      stIrOpt.stamp_ = now;

      broadcaster.sendTransform(stColorOpt);
      broadcaster.sendTransform(stIrOpt);

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
};

class Kinect2BridgeNodelet : public nodelet::Nodelet
{
private:
  std::thread kinect2BridgeThread;
  Kinect2Bridge *pKinect2Bridge;

public:
  Kinect2BridgeNodelet() : Nodelet(), pKinect2Bridge(NULL)
  {
  }

  ~Kinect2BridgeNodelet()
  {
    if(pKinect2Bridge)
    {
      pKinect2Bridge->stop();
      delete pKinect2Bridge;
    }
  }

  virtual void onInit()
  {
    pKinect2Bridge = new Kinect2Bridge(getNodeHandle(), getPrivateNodeHandle());
    pKinect2Bridge->start();
  }
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(Kinect2BridgeNodelet, nodelet::Nodelet)

void helpOption(const std::string &name, const std::string &stype, const std::string &value, const std::string &desc)
{
  std::cout  << '_' << name << ":=<" << stype << '>' << std::endl
             << "    default: " << value << std::endl
             << "    info:    " << desc << std::endl;
}

void help(const std::string &path)
{
  std::string depthMethods = "cpu";
  std::string depthDefault = "cpu";
  std::string regMethods = "default";
  std::string regDefault = "default";

#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
  depthMethods += ", opengl";
  depthDefault = "opengl";
#endif
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
  depthMethods += ", opencl";
  depthDefault = "opencl";
#endif
#ifdef DEPTH_REG_CPU
  regMethods += ", cpu";
#endif
#ifdef DEPTH_REG_OPENCL
  regMethods += ", opencl";
  regDefault = "opencl";
#endif

  std::cout << path << " [_options:=value]" << std::endl;
  helpOption("base_name",         "string", K2_DEFAULT_NS,  "set base name for all topics");
  helpOption("sensor",            "double", "-1.0",         "serial of the sensor to use");
  helpOption("fps_limit",         "double", "-1.0",         "limit the frames per second");
  helpOption("calib_path",        "string", K2_CALIB_PATH,  "path to the calibration files");
  helpOption("use_png",           "bool",   "false",        "Use PNG compression instead of TIFF");
  helpOption("jpeg_quality",      "int",    "90",           "JPEG quality level from 0 to 100");
  helpOption("png_level",         "int",    "1",            "PNG compression level from 0 to 9");
  helpOption("depth_method",      "string", depthDefault,   "Use specific depth processing: " + depthMethods);
  helpOption("depth_device",      "int",    "-1",           "openCL device to use for depth processing");
  helpOption("reg_method",        "string", regDefault,     "Use specific depth registration: " + regMethods);
  helpOption("reg_devive",        "int",    "-1",           "openCL device to use for depth registration");
  helpOption("max_depth",         "double", "12.0",         "max depth value");
  helpOption("min_depth",         "double", "0.1",          "min depth value");
  helpOption("queue_size",        "int",    "2",            "queue size of publisher");
  helpOption("bilateral_filter",  "bool",   "true",         "enable bilateral filtering of depth images");
  helpOption("edge_aware_filter", "bool",   "true",         "enable edge aware filtering of depth images");
  helpOption("publish_tf",        "bool",   "false",        "publish static tf transforms for camera");
  helpOption("base_name_tf",      "string", "as base_name", "base name for the tf frames");
  helpOption("worker_threads",    "int",    "4",            "number of threads used for processing the images");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinect2_bridge", ros::init_options::AnonymousName);


  for(int argI = 1; argI < argc; ++argI)
  {
    std::string arg(argv[argI]);

    if(arg == "--help" || arg == "--h" || arg == "-h" || arg == "-?" || arg == "--?")
    {
      help(argv[0]);
      ros::shutdown();
      return 0;
    }
    else
    {
      std::cerr << "Unknown argument: " << arg << std::endl;
      return -1;
    }
  }

  if(!ros::ok())
  {
    std::cerr << "ros::ok failed!" << std::endl;
    return -1;
  }

  Kinect2Bridge kinect2;
  if(kinect2.start())
  {
    ros::spin();

    kinect2.stop();
  }

  ros::shutdown();
  return 0;
}
