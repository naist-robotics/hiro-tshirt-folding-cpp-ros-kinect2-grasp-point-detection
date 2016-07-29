
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
#include "std_msgs/String.h"

using namespace std;
using namespace cv;
class recognizer
{
  // attributs
  public:
    string categoryFound,outputGP,request;
    ros::Publisher category_pub;
    ros::Publisher graspingPoints_pub;
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

  void detectCategory(){
    //categoryFound=...
    categoryFound="categoryFound";
  }
  void getGraspingPoints(){
    // deformation(target)
    outputGP="Test grasping points";
  }
  void setpath(string newPath)
  {
    path=newPath;
  }
  void publishOnNodes(){
    std_msgs::String msg;
    msg.data = categoryFound;
    category_pub.publish(msg);
    msg.data = outputGP;
    graspingPoints_pub.publish(msg);
    ros::spinOnce();
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
  void requestCallback(const std_msgs::String::ConstPtr& msg)
  {
    //ROS_INFO("imgInfoTopicCallback: [%s]", msg->data.c_str());
    request=msg->data.c_str();
    std::size_t found = request.find("getCategory");
    if (found!=std::string::npos){
      cout<<"Category request"<<endl;
      detectCategory();
    }
    found = request.find("getGraspingPoints"); 
    if (found!=std::string::npos){
      cout<<"Grasping points request"<<endl;
      if(categoryFound=="")
        detectCategory();
      getGraspingPoints();      
    }
    publishOnNodes(); 

  }

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
  ros::Subscriber request_sub = nh.subscribe("request", 1000, &recognizer::requestCallback,&TshirtAnalyzer);
  TshirtAnalyzer.category_pub = nh.advertise<std_msgs::String>("category", 100);
  TshirtAnalyzer.graspingPoints_pub = nh.advertise<std_msgs::String>("graspingPoints", 100);

  ros::spin();
  cv::destroyWindow("view");

  

  //TshirtAnalyzer.loadReference();

waitKey();

  return 0;
}