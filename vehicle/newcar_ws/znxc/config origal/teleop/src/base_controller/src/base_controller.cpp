/****************************************************
a5 15 a4 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 5a 
****************************************************/

#include "ros/ros.h" 
#include <ackermann_msgs/AckermannDriveStamped.h>


#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include "serial/serial.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
serial::Serial ser;
float speed=0,steering_angle=0;

unsigned char data_header=0xa5;  //   "/r"
unsigned char data_end=0x5a;  //   "/n"
//unsigned char speed_data[14]={0} ;    //serial data
//unsigned char length=0x07;
//unsigned char buf[11]={0xa5,0x07,0x00,0x03,0x01,0xff,0x00,0x00,0xff,0x00,0x5a};
unsigned char buf[11]={0};

/***
union floatData
{
  float d;
  unsigned char data[4];
}linear_speed_x,linear_speed_y,angular_speed_z;
***/
union SendData
{
	char d;
	unsigned char data[1];
}car_dir,car_speed,car_corner,corner_value,continu_time;


void callback(const ackermann_msgs::AckermannDriveStamped & cmd_input)
{
  speed  = cmd_input.drive.speed;
  steering_angle = cmd_input.drive.steering_angle;
}


void writeSpeed(float speed, float steering_angle) //Subscribe Twist
{
 

 
 buf[0] = data_header;
 static unsigned char data_control =0x00;
 if(speed>0)
   car_dir.d=0x01;
 else if(speed<0)
   car_dir.d=0x02;
 else car_dir.d=0x00;
  if(steering_angle>0)
   car_corner.d=0x01;
 else if(steering_angle<0)
   car_corner.d=0x02;
 else car_corner.d=0x00;
 if(speed>0)
  car_speed.d = speed;
 else if(speed<0)
  car_speed.d = -speed;
 else car_speed.d=0;
 if(steering_angle>0)
  corner_value.d = steering_angle;
 else if(speed<0)
  corner_value.d = -steering_angle;
 else corner_value.d=0;

 
 //continu_time.d = 0x50;
 buf[0]=data_header;
 buf[1]=0x07;
 buf[2]=0x03;
 buf[3]=data_control==128? data_control=0:data_control++;
 buf[4]=car_dir.d;
 buf[5]=car_speed.d;
 buf[6]=car_corner.d;
 buf[7]=corner_value.d;//corner_value.data[4];
 buf[8]=0x50;
 buf[9]=0x00;
 buf[10]=data_end;
 
/***
 car_dir.d =  0x01;
 car_speed.d= 0x01;
 car_corner.d=0x01;
 corner_value.d = 0x50;
 continu_time.d = 0x28;
 buf[0]=data_header;
 buf[1]=0x07;
 buf[2]=0x03;
 // buf[3]=data_control==128? data_control=0:data_control++;
 buf[3]=0x01;
 memcpy(&buf[4], &car_dir.d, 1);
 memcpy(&buf[5], &car_speed.d, 1);
 memcpy(&buf[6], &car_corner.d, 1);
 memcpy(&buf[7], &corner_value.d, 1);
 memcpy(&buf[8], &continu_time.d, 1);
 buf[9]=data_end;
***/
 //write to serial
 int n = ser.write(buf,11);
 ROS_INFO("data send success  %d ",n);
}



int main(int argc, char **argv)
{
 

 ros::init(argc, argv, "base_controller"); //init Node
 ros::NodeHandle n; 
 ros::Subscriber sub = n.subscribe("ackermann_cmd", 20, callback);
 
 try
 {
 	ser.setPort("/dev/ttyACM0");
	ser.setBaudrate(115200);
	serial::Timeout to = serial::Timeout::simpleTimeout(1000);
 	ser.setTimeout(to);
	ser.open();
 }
 catch (serial::IOException& e)
 {
	ROS_ERROR_STREAM("UNABLE TO open port");
 	return -1;
 }
 if(ser.isOpen())
{
	ROS_INFO_STREAM("Serial port opened");
        

}
 else
{
	return -1;
	}

 
 ros::Rate loop_rate(20);
 while (ros::ok())
 {  
    ros::spinOnce();
    writeSpeed(speed,steering_angle);
    loop_rate.sleep();
  }
 ser.close();
 return 0;
}


