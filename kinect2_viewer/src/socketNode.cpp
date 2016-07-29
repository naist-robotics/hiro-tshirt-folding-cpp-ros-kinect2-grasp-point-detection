#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include "TcpLib.hpp"
#define IPADDRESS "163.221.44.226"
#define PORTNUM 3794

using namespace std;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

class rosTopicManager{

  public:
      ros::Subscriber imgInfoTopic_sub ;
      ros::Publisher request_pub;
      string imgInfoTopic,request,category,graspingPoints;



  
  public:
      rosTopicManager(){}
      ~rosTopicManager(){}

      void imgInfoTopicCallback(const std_msgs::String::ConstPtr& msg)
      {
        //ROS_INFO("imgInfoTopicCallback: [%s]", msg->data.c_str());
        imgInfoTopic=msg->data.c_str();   

      }
      void categoryCallback(const std_msgs::String::ConstPtr& msg)
      {
        //ROS_INFO("Category: [%s]", msg->data.c_str());
        category=msg->data.c_str();   

      }
      void graspingPointsCallback(const std_msgs::String::ConstPtr& msg)
      {
        //ROS_INFO("graspingPoints: [%s]", msg->data.c_str());
        igraspingPoints=msg->data.c_str();   

      }
      void publishOnNodes(){
        std_msgs::String msg;
        msg.data = request;

        request_pub.publish(msg);
        ros::spinOnce();
      }

    // void recognitionInfoTopicCallback(const std_msgs::String::ConstPtr& msg)
    // {
    //   ROS_INFO("recognitionInfoTopicCallback: [%s]", msg->data.c_str());

    //   if ((sendTCP(msg->data.c_str()))<=0){
    //     std::cout << "Trying to reconnect. flag: " << sendSuccess << std::endl;
    //     reconnectSocket();

    //   }  
    // }

};

class tcpManager
{
    private:
      TcpClient* client;
      #define IPADDRESS "163.221.44.226"
      #define PORTNUM 3794

    public:
      tcpManager(){
        client = new TcpClient;
        client->Init(IPADDRESS, PORTNUM);
      }
      ~tcpManager(){
        delete client;
      }
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
        return client->Write(SendData);
       }

      string receiveTCP(){
        vector<char> ReadData;
        client->Read(ReadData); 
        std::string str(ReadData.begin(),ReadData.end()); 
        return "Received";
      }
 

};






int main(int argc, char **argv)
{


  ros::init(argc, argv, "socketNode");

  rosTopicManager rosInterface;
  tcpManager tcpInterface;

  ros::NodeHandle n;
  rosInterface.imgInfoTopic_sub = n.subscribe("kinect2_viewer/imgInfoTopic", 100,&rosTopicManager::imgInfoTopicCallback, &rosInterface);
  rosInterface.imgInfoTopic_sub = n.subscribe("category", 100,&rosTopicManager::categoryCallback, &rosInterface);
  rosInterface.imgInfoTopic_sub = n.subscribe("graspingPoints", 100,&rosTopicManager::graspingPointsCallback, &rosInterface);
  rosInterface.request_pub = n.advertise<std_msgs::String>("request", 100);
  ros::spinOnce();

     while(1){
      //transfert from TCP to ros topic
      cout<<"transfert from TCP to ros topic"<<endl;
      //rosInterface.request=tcpInterface.receiveTCP();
      rosInterface.publishOnNodes();
      //transfert from ros topic to TCP
      cout<<"transfert from ros topic to TCP"<<endl;
      if(rosInterface.category!="")
      { tcpInterface.sendTCP(rosInterface.category);
        category="";}
      if(rosInterface.graspingPoints!="")
      { tcpInterface.sendTCP(rosInterface.graspingPoints);
        graspingPoints="";}
      if(tcpInterface.sendTCP(rosInterface.imgInfoTopic)<=0){
        cout<<"Trying to reconnect"<<endl;
        tcpInterface.reconnectSocket(); 
      }

      //Display
      cout<<"Request:"<< rosInterface.request<<endl;
      cout<<"imgInfoTopic:"<<rosInterface.imgInfoTopic<<endl;
      cout<<"Category:"<<rosInterface.category<<endl;
      cout<<"graspingPoints:"<<rosInterface.graspingPoints<<endl;
      std::chrono::milliseconds duration(100);
      std::this_thread::sleep_for(duration);
     }

 


  return 0;
}