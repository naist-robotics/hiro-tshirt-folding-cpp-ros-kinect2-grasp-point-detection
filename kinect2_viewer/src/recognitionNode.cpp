
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <stdlib.h>
#include <fstream>
#include <stdio.h>
#include <vector>
#include <fstream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"

#define au 523.5  // pixels 
#define av 523.8
#define u0 472.75 // Optical centre u coordinate
#define v0 276      // Optical center v coordinate

using namespace std;
using namespace cv;

string path="/home/robotics/catkin_ws/src/iai_kinect2/kinect2_viewer/src/";
string refFolder="Reference/";

cv::Mat conversion=(Mat_<double>(4,4) <<  0.001, 0, 0, 0, 
                    0, 0.001, 0, 0, 
                    0, 0, 0.001, 0,
                    0, 0, 0, 1); // conversion from mm to meters
class recognizer
{
// attributs
public:
string categoryFound,outputGP;
ros::Publisher category_pub;
ros::Publisher graspingPoints_pub;
ros::Publisher sidePoints_pub;
private:
// informations about the references
Mat contourRef,contourData,dataGravityCenter, groundTruth,depth;


// Target
Mat target;
Point anchorTarget;
//Output
vector<Mat> GraspingP;


//methods

private:
void kinectToHIRO( cv::Mat &inputPoint){


  //Tranformation of pixel coordinatesa to 3D x-, y-coordinates in 3D Kinect coordinate system
  double z=inputPoint.at<double>(1,1);  // Taken from depth sensor
  inputPoint.at<double>(-1,1) = ( (-z/au) *(inputPoint.at<double>(-1,1)- u0) );
  inputPoint.at<double>(0,1) = ( (-z/av)*(inputPoint.at<double>(0,1) - v0) );



  inputPoint=conversion*inputPoint; // conversion from mm to meters;


  //Tranformation from Kinect Coordinate System to HIRo Coordinate system

  // cv::Mat kinectToHIRO =(Mat_<double>(4,4) <<             -0.0830,   -0.9965,   -0.0109,    0.4416,
  //                             -0.9965,    0.0829,    0.0070,    0.1177,
  //                             -0.0061,    0.0114,   -0.9999,    0.9961,
  //                                   0,         0,         0,    1.0000);

  cv::Mat kinectToHIRO =(Mat_<double>(4,4) << -0.0830,   -0.9965,   -0.0109,    0.5016,
                  -0.9965,    0.0829,    0.0070,    0.0677,
                  -0.0061,    0.0114,   -0.9999,    0.9461,
                       0,         0,         0,    1.0000);

  inputPoint = kinectToHIRO * inputPoint;


}
int polarContourClassification(Mat polarContourSet, Mat groundTruth, Mat inputPolarC)
{ 
  // int sizeLearningSet=polarContourSet.cols;
  // Mat error=Mat::zeros(sizeLearningSet,1,CV_32F);

  // for (int i=0;i<polarContourSet.cols;i++)
  // {
  //   Mat sub=polarContourSet.col(i)-inputPolarC;
  //   cout << polarContourSet.col(i) << endl;
  //   cout << inputPolarC << endl;
  //   cout << sub << endl;
  //   Scalar sum=sum(sub);
  //   error.at<float>(i,1)=sum[0];

  // }
  return 4;
  // Mat sortedError;

  // sortIdx(error, sortedError, CV_SORT_ASCENDING);
  // return groundTruth.at<float>(sortedError.at<float>(0,0));
}
int gravityCenterClassification( Mat gravityCenterSet, Mat groundTruth, Mat inputGc, int neighbourNum )
{
    return 4;
}
void getImgInfos( Mat inputImg, Mat &gravityCenter, Mat &polarContour )
{

}
int classification(Mat image, Rect boundingBox){
  Scalar color=cv::Scalar(255,0,0);
  //enlarge your bounding box
  Point offset= Point(30,30);
  Point tl=boundingBox.tl()-offset;
  Point br=boundingBox.br()+offset;
  Rect bound = Rect(Point(0,0),Point(image.cols,image.rows));
  cv::Mat cropped;

  if (tl.inside(bound) && br.inside(bound)){

      Rect bigBoundingBox= Rect(tl,br);
      cv::rectangle( image, bigBoundingBox.tl(), bigBoundingBox.br(), color, 2, 8, 0 );
      anchorTarget=bigBoundingBox.tl();
      //imshow( "BigBoudingbox", image );
      //crop the image
      cv::Mat croppedRef(image, bigBoundingBox);

      // Copy the data into new matrix
      croppedRef.copyTo(cropped);
      resize(cropped.clone(), cropped, Size(300,300), 0, 0);
  }
  else
      cropped=Mat::zeros(300,300,CV_8U);

  imshow( "Croped image", cropped );

  Mat gravityCenter,polarContour;
  getImgInfos(image,gravityCenter,polarContour);
  if(gravityCenterClassification( dataGravityCenter, groundTruth, gravityCenter, 5 )<=4)
  {
    switch(polarContourClassification( contourData, groundTruth, polarContour ))
    {
      case 1: categoryFound="A";
              return 1;
              break;
      case 2: categoryFound="B";
              return 2;
              break;
      case 3: categoryFound="C";  
              return 3;
              break;
      default:categoryFound="none"; 
              return 4;
    }
  }
  else
    cout<<"classification out of bounds"<<endl;

  return 0;
}

string getGraspingPoints(Mat image){
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  Mat leftArmGp,rightArmGp;
  //find the contour and find the bounding box
  findContours( image.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
  vector<Rect> boundRect( contours.size() );
  for( uint i = 0; i < contours.size(); i++ )
     boundRect[i] = boundingRect( Mat(contours[i]) );

  //rectangle( image, boundRect[0].tl(), boundRect[0].br(), color, 2, 8, 0 );
  //imshow( "Boudingbox", image );
  Point tl=boundRect[0].tl();
  Point br=boundRect[0].br();
  Point Gp1,Gp2;
  switch(classification(image,boundRect[0]))
  {
      case 1: // A: grasp the bottom and leftmost point
                cout<<"Classification: A"<<endl;
                Gp1=tl;
                Gp2=Point(br.x,0);
                while (image.at<uint8_t>(Gp2)<=0)
                      Gp2.y++;
              break;
      case 2: //B: grasp shoulders approx :GP1 on x=tl.x+(br.x-tl.x)/4   GP2 on x=br.x-(br.x-tl.x)/4
                cout<<"Classification: B"<<endl;
                Gp1=Point(tl.x+(br.x-tl.x)/4,0);
                Gp2=Point(br.x-(br.x-tl.x)/4,0);
                while (image.at<uint8_t>(Gp1)<=0)
                      Gp1.y++;
                while (image.at<uint8_t>(Gp2)<=0)
                      Gp2.y++;
              break;
      case 3:// C: grasp bottom and rightmost point 
                cout<<"Classification: C"<<endl;
                Gp1=Point(br.x,0); 
                while (image.at<uint8_t>(Gp2)<=0)
                      Gp1.y++;
                Gp2=Point(tl.x,br.y);
              break;
      default: // two points at the center of the bounding box
                cout<<"Classification: none"<<endl;
                Gp1=Point((tl.x+br.x)/2,(tl.y+br.y)/2);
                Gp2=Point((tl.x+br.x)/2,(tl.y+br.y)/2);            
  }

  //circle grasping points
   cv::circle(image, Gp1, 10, cv::Scalar(255,0,0), 2, 4);
   cv::circle(image, Gp2, 10, cv::Scalar(255,0,0), 2, 4);

 //  rightArmGp.at<double>(-1,1)=Gp1.x;
 //  rightArmGp.at<double>(0,1)=Gp1.y;
 //  rightArmGp.at<double>(1,1)=depth.at<short>(rightArmGp.at<double>(0,1),rightArmGp.at<double>(-1,1));

 //  leftArmGp.at<double>(-1,1)=Gp2.x;
 //  leftArmGp.at<double>(0,1)=Gp2.y;
 //  leftArmGp.at<double>(1,1)=depth.at<short>(leftArmGp.at<double>(0,1),leftArmGp.at<double>(-1,1));

 //  kinectToHIRO(rightArmGp);
 //  kinectToHIRO(leftArmGp);

 // return "graspingPoints " +to_string(rightArmGp.at<double>(-1,1))+" "
 //                          +to_string(rightArmGp.at<double>(0,1))+" "
 //                          +to_string(rightArmGp.at<double>(1,1))+" "
 //                          +to_string(leftArmGp.at<double>(-1,1))+" "
 //                          +to_string(leftArmGp.at<double>(0,1))+" "
 //                          +to_string(leftArmGp.at<double>(1,1));
  return "graspingPoints ";
}  

Mat textFile2Mat(String filename, int rowNum, int colNum)
{
  char globalPath[100];
  string delimiter = ",";
  int rows=0;
  int cols=0;
  Mat output=Mat::zeros(rowNum,colNum,CV_32F);

 ifstream myReadFile;
 sprintf(globalPath,"%s%s%s",path.c_str(),refFolder.c_str(),filename.c_str());
 myReadFile.open(globalPath);
 std::string token,line;
 string::size_type sz; 
 if (myReadFile.is_open()) {
   while(!myReadFile.eof()) // To get you all the lines.
    {
      getline(myReadFile,line,'\n'); // Saves the line in STRING.
      //cout<<"line:"<<line<<endl;

      stringstream ss(line);
      string item;

      while (getline(ss, item, ',')) {
          //cout<<item<<endl;
          output.at<float>(rows,cols) =stof(item,&sz) ;
          cols++;
      }
      cols=0;
      rows++;




    }
  }
  else
        cout<<"File not found: "<<globalPath<<endl;

  myReadFile.close();
  return output;

}


public:  
recognizer()
{
}  

~recognizer()
 {
 }



void setpath(string newPath)
{
  path=newPath;
}
void loadData(){
  cout<<"Loading: contourRef.txt"<<endl;
  contourRef=textFile2Mat("contourRef.txt",120,5);

  cout<<"Loading: contourData.txt"<<endl;
  contourData=textFile2Mat("contourData.txt",120,91);

  cout<<"Loading: dataGravityCenter.txt"<<endl;
  dataGravityCenter=textFile2Mat("dataGravityCenter.txt",91,2);

  cout<<"Loading: groundTruth.txt"<<endl;
  groundTruth=textFile2Mat("groundTruth.txt",91,1);

}
void publishOnNodes(){
  std_msgs::String msg;
  //Category found with classification
  msg.data = categoryFound;
  category_pub.publish(msg);
  //Grasping points adapted to classification
  msg.data = outputGP;
  graspingPoints_pub.publish(msg);
  ros::spinOnce();
    }

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  Mat image;

  //recceive image from topic
  try
   {
     cv::imshow("view", cv_bridge::toCvShare(msg, "mono8")->image);
     cv::waitKey(30);
     image= cv_bridge::toCvShare(msg, "mono8")->image;
   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
   }

  outputGP=getGraspingPoints(image);// result of the classification stored in categoryFound


}


Mat getTarget(){
  return target;
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
TshirtAnalyzer.category_pub = nh.advertise<std_msgs::String>("category", 100);
TshirtAnalyzer.graspingPoints_pub = nh.advertise<std_msgs::String>("graspingPoints", 100);


//int fileindex=76;
string filename;

TshirtAnalyzer.loadData();


while(1)
{
//Image acquisition pressing a key
// if(waitKey(100)!=-1){
//    filename="edge"+to_string(fileindex)+".png";
//    imwrite(filename, TshirtAnalyzer.getTarget());
//    fileindex++;
//    cout<<"Saved as:  "<<filename<<endl;
//   }
ros::spinOnce();
}


cv::destroyWindow("view");



//TshirtAnalyzer.loadReference();

waitKey();

return 0;
}