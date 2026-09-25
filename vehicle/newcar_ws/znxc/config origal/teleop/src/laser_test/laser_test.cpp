#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <laser_test/aimPose.h>
#include <laser_test/laser_control.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h> 
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Bool.h>
using namespace std;
using namespace cv;

float nearest_x,nearest_y,nearest_dist;
ackermann_msgs::AckermannDriveStamped  ackermann_cmd;
float k = 0;
float nearest_angle;
                                                                                                                                                                                                                                
float midx,midy,sx=0,sy=0;
laser_test::laser_control laser_control;
laser_geometry::LaserProjection projector_;
sensor_msgs::PointCloud2 cloud;
int num = 1440;//num为雷达扫描一周的点数
//int low_range = num/16,up_range = num*7/16;
int low_range = 600,up_range = 840;
//int low_range = 1,up_range2 = 120;
int flag = 1;
std_msgs::Bool laser_contr;
void LaserOP(const sensor_msgs::LaserScan& laser)
{
  ackermann_cmd.drive.speed = -20;
  tf::TransformListener tfListener_; 
  projector_.transformLaserScanToPointCloud("laser_link", laser, cloud, tfListener_);
  int i,j,tt=0; 
  int l=100,r=100;
  float xba=0,yba=0,px1,py1,px2,py2,similarity=-1,d1=0,d2=0;
  nearest_x=0; nearest_y=100; nearest_dist=100;  
  for (j=low_range;j<up_range;j=j+4,i=(j+720)%1440)
  {
    float ang=laser.angle_min+i*laser.angle_increment;
    float px=-sin(ang)*laser.ranges[i];
    float py= cos(ang)*laser.ranges[i];
    if (laser.ranges[i]<nearest_dist ) //&& fabs(px)<=0.25 && py>=0.0 && py<=1.2
    {             	                                                                               
      tt++; 
      if (tt==3) 
      {
        nearest_dist=laser.ranges[i]; nearest_x=px; nearest_y=py; tt=0; l=i; r=i; 
        nearest_angle = laser.angle_min+i*laser.angle_increment;
      }
    }
    else
	tt=0;
    
  }
  tt=0;
  ROS_INFO_STREAM("The nearest distance is: "<<nearest_dist<<" The nearest angle is: "<<nearest_angle/CV_PI*180<<endl);
  //ackermann_cmd.drive.steering_angle = 0;
  ROS_INFO_STREAM(ackermann_cmd.drive.steering_angle);
  if (nearest_dist<0.6) //阈值需要调参
  {
    
    ROS_INFO_STREAM("The obstacle is detected!"<<endl);
    low_range = 0;
    up_range = num;                                                       
    laser_control.laser_control = 1; 
    ackermann_cmd.drive.speed = -20;
    if(flag == 1 )
    {
      ackermann_cmd.drive.steering_angle = 22;//左转弯  角度需要调参
      ROS_INFO_STREAM(ackermann_cmd.drive.steering_angle);
      flag = 0;
    }
    if(nearest_dist>0.8 && flag == 0 )  //阈值需要调参
    {
      ackermann_cmd.drive.steering_angle = 42;//左转弯  角度需要调参
      ROS_INFO_STREAM(ackermann_cmd.drive.steering_angle);
    } 
    if(nearest_angle/CV_PI*180>65&&nearest_angle/CV_PI*180<295)// 阈值需要调参
    {
      ROS_INFO_STREAM(nearest_angle/CV_PI*180);
      ackermann_cmd.drive.steering_angle = -122;//右转弯   角度需要调参
      ROS_INFO_STREAM(ackermann_cmd.drive.steering_angle);
    }
    laser_contr.data=true;
  }
  else
  {
       flag = 1;
      //  ackermann_cmd.drive.steering_angle =0;
      //  laser_control.laser_control = 0; 
       ROS_INFO_STREAM(ackermann_cmd.drive.steering_angle);
       low_range = 400;
       up_range = 1040;
  }
}

// void LaserOP(const sensor_msgs::LaserScan& laser) // This is used for the rear camera 
// {
//   //ackermann_cmd.drive.speed = 100;
//   tf::TransformListener tfListener_; 
//   projector_.transformLaserScanToPointCloud("laser_link", laser, cloud, tfListener_);
//   int i,tt=0; 
//   int l=100,r=100;
//   float xba=0,yba=0,px1,py1,px2,py2,similarity=-1,d1=0,d2=0;
//   nearest_x=0; nearest_y=100; nearest_dist=100;  
//   for (i = 300;i < 1140;i=i+4)
//   {
//     float ang=laser.angle_min+i*laser.angle_increment;
//     float px=-sin(ang)*laser.ranges[i];
//     float py= cos(ang)*laser.ranges[i];
//     if (laser.ranges[i]<nearest_dist ) //&& fabs(px)<=0.25 && py>=0.0 && py<=1.2
//     {             	                                                                               
//       tt++; 
//       if (tt==3) 
//       {
//         nearest_dist=laser.ranges[i]; nearest_x=px; nearest_y=py; tt=0; l=i; r=i; 
//         nearest_angle = laser.angle_min+i*laser.angle_increment;
//       }
//     }
//     else
// 	tt=0;
    
//   }
  
//   ROS_INFO_STREAM("The nearest distance is: "<<nearest_dist<<" The nearest angle is: "<<nearest_angle/CV_PI*180<<endl);
//   if (nearest_dist<0.4) //阈值需要调参
//   {                                                       
//     // laser_control.laser_control = 1; 
//     //ackermann_cmd.drive.speed = 0;
//    // ackermann_cmd.drive.steering_angle = 0;
//    laser_contr.data=true;
//     ROS_INFO_STREAM("STOP!!!"<<endl);
//   }
//   else
//   {
//     laser_contr.data=false;
//   }
// }

int main(int argc,char** argv){
  ros::init(argc,argv,"laser_test");
  ros::NodeHandle nh;

  ros::Subscriber sublaser=nh.subscribe("/scan",1,&LaserOP);
  //ros::Subscriber sublaser=nh.subscribe("/scan",1,&LaserOP1);
  ros::Publisher pub_vel=nh.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd",1);
  ros::Publisher pub_laser_control=nh.advertise<std_msgs::Bool>("/laser_control",1);
  ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/PointCloud", 1);
  laser_contr.data=false;
  ros::Rate rate(40);
  while (ros::ok())
  {
    ros::spinOnce();
    if (laser_contr.data==true)
    {
      cout<<ackermann_cmd.drive.steering_angle<<endl;
      pub_vel.publish(ackermann_cmd); 
    }
    
    pub_cloud.publish(cloud);
    pub_laser_control.publish(laser_contr);                                      
    rate.sleep();
  }

}
