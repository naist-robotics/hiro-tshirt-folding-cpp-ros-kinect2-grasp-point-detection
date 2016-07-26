#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h>
#include <stdio.h>
#include <thread>
#include <chrono>
#include "TcpLib.hpp"
#define IPADDRESS "163.221.44.226"
#define PORTNUM 3794

using namespace std;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
TcpClient* client;

bool reconnectSocket(){

  delete client;
  std::chrono::milliseconds duration(1500);
  std::this_thread::sleep_for(duration);
  client = new TcpClient;
  return client->Init(IPADDRESS, PORTNUM);

}
int sendTCP (string message){
    vector<char> SendData;
    for(int i = 0; i < int(message.size()); i++){
        SendData.push_back(message[i]);
    }
    return client->Write(SendData);//送信
  }

vector<char> receiveTCP(){
      vector<char> ReadData;
      client->Read(ReadData);
      return ReadData;
}

void imgInfoTopicCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("imgInfoTopicCallback: [%s]", msg->data.c_str());
  
  if ((sendTCP(msg->data.c_str()))<=0){
    std::cout << "Trying to reconnect " << std::endl;
    reconnectSocket();

  }  

}

// void recognitionInfoTopicCallback(const std_msgs::String::ConstPtr& msg)
// {
//   ROS_INFO("recognitionInfoTopicCallback: [%s]", msg->data.c_str());

//   if ((sendTCP(msg->data.c_str()))<=0){
//     std::cout << "Trying to reconnect. flag: " << sendSuccess << std::endl;
//     reconnectSocket();

//   }  
// }


int main(int argc, char **argv)
{
  client= new TcpClient;
  client->Init(IPADDRESS, PORTNUM);

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("kinect2_viewer/imgInfoTopic", 1000, imgInfoTopicCallback);
  //ros::Subscriber sub = n.subscribe("recognitionInfoTopic", 1000, recognitionInfoTopicCallback);


    


  ros::spin();

  return 0;
}