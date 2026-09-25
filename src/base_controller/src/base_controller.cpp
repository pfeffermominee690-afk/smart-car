

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
ros::WallTime last_cmd_time;
bool command_received=false;
const double command_timeout=0.5;

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
  last_cmd_time = ros::WallTime::now();
  command_received = true;
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
 else if(steering_angle<0)
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
 static float last_logged_speed = 9999;
 static float last_logged_steering = 9999;
 if(speed != last_logged_speed || steering_angle != last_logged_steering)
 {
   ROS_INFO("TX command speed=%.1f steering=%.1f dir=%u speed_byte=%u "
            "steer_dir=%u steer_byte=%u bytes_written=%d",
            speed, steering_angle, static_cast<unsigned int>(buf[4]),
            static_cast<unsigned int>(buf[5]), static_cast<unsigned int>(buf[6]),
            static_cast<unsigned int>(buf[7]), n);
   last_logged_speed = speed;
   last_logged_steering = steering_angle;
 }
}



int main(int argc, char **argv)
{
 

 ros::init(argc, argv, "base_controller"); //init Node
 ros::NodeHandle n; 
 ros::NodeHandle private_n("~");
 ros::Subscriber sub = n.subscribe("ackermann_cmd", 20, callback);

 string serial_port;
 int baudrate;
 int serial_open_attempts;
 private_n.param<string>("serial_port", serial_port, "/dev/ttyACM0");
 private_n.param<int>("baudrate", baudrate, 115200);
 private_n.param<int>("serial_open_attempts", serial_open_attempts, 20);
 private_n.setParam("serial_ready", false);

 ser.setPort(serial_port);
 ser.setBaudrate(baudrate);
 serial::Timeout to = serial::Timeout::simpleTimeout(1000);
 ser.setTimeout(to);

 for(int attempt = 1; ros::ok() && !ser.isOpen() && attempt <= serial_open_attempts; ++attempt)
 {
   try
   {
     ser.open();
   }
   catch (serial::IOException& e)
   {
     if(attempt == 1 || attempt == serial_open_attempts || attempt % 4 == 0)
       ROS_WARN_STREAM("Unable to open serial port " << serial_port
                       << " (attempt " << attempt << "/" << serial_open_attempts
                       << "): " << e.what());
   }
   if(!ser.isOpen())
     ros::WallDuration(0.5).sleep();
 }
if(ser.isOpen())
{
	private_n.setParam("serial_ready", true);
	ROS_INFO_STREAM("Serial port opened: " << serial_port);
        

}
 else
{
	ROS_ERROR_STREAM("Unable to open serial port after " << serial_open_attempts
	                 << " attempts: " << serial_port);
	return -1;
	}

 
 ros::Rate loop_rate(20);
 while (ros::ok())
 {  
    ros::spinOnce();
    if(command_received && (ros::WallTime::now()-last_cmd_time).toSec() <= command_timeout)
      writeSpeed(speed,steering_angle);
    else
      writeSpeed(0,0);
    loop_rate.sleep();
  }
 ser.close();
 return 0;
}
