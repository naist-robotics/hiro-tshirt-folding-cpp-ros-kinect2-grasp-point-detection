
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include "ros/ros.h"

using namespace std;
using namespace cv;
class recognizer
{
  // attributs
  public:
  private:
    vector<Mat> reference;
    Mat target;
    vector<Mat> GraspingP;
    std::vector<string> category={"A","B","C","none"};
    string path="/home/robotics/catkin_ws/devel/lib/kinect2_viewer/";
    string refFolder="Reference/";

  //methods
  public:  
    recognizer()
    {
    }  

    ~recognizer()
    {
    }
    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
      try
      {
        cv::imshow("view", cv_bridge::toCvShare(msg, "mono8")->image);
        cv::waitKey(30);
        target= cv_bridge::toCvShare(msg, "mono8")->image;
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
      }
    }
    void loadReference(){

      Mat tmp;
      int index;
      char filename[100];

      for(uint i=0; i < category.size()-1;i++){
          cout<<"Category:"<<category[i]<<endl;
          index=1;
          while(1){  
            sprintf(filename,"%s%s%s%d.png",path.c_str(),refFolder.c_str(),category[i].c_str(),index);
            tmp=imread(filename,CV_LOAD_IMAGE_GRAYSCALE);
            if(!tmp.data ){ 

              break;}
            cout<<"filename:"<<filename<<endl;           
            reference.push_back(tmp);
            imshow(filename,tmp);

            index++;
            
          }
      }

    }
  void loadGraspingPoint(string filename){

  }  

  string detectCategory(){
    return "none";

  }
  void getGraspingPoints(string srcRef){

  }
  void setpath(string newPath)
  {
    path=newPath;
  }
  private: 

};



int main(int argc, char **argv)
{
  recognizer TshirtAnalyzer ;

  ros::init(argc, argv, "recognitionNode");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("kinect2_viewer/extratedContour", 1, &recognizer::imageCallback,&TshirtAnalyzer);
  ros::spin();
  cv::destroyWindow("view");

  

  //TshirtAnalyzer.loadReference();

waitKey();

  return 0;
}