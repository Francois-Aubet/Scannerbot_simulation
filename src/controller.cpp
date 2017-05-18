#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include <signal.h>
#include "sensor_msgs/LaserScan.h"
#include <termios.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include <iostream>

#include "controller.h"

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

using namespace std;

double static ranges[25];


/*int kfd = 0;
struct termios cooked, raw;
void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}*/

Controller::Controller():
  ph_("~"),
  linear_(0),
  angular_(0),
  l_scale_(1.0),
  a_scale_(1.0)
{
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);

  
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("my_robot/cmd_vel", 1);
  ros::Subscriber subHokuyo = nSub.subscribe("/my_robot/laser/scan", 1000, getHokuyoVal);
}



void Controller::watchdog()
{
  boost::mutex::scoped_lock lock(publish_mutex_);
  if ((ros::Time::now() > last_publish_ + ros::Duration(0.15)) && 
      (ros::Time::now() > first_publish_ + ros::Duration(0.50)))
    publish(0, 0);
}


void Controller::publish(double angular, double linear)  
{
    geometry_msgs::Twist vel;
    vel.angular.z = a_scale_*angular;
    vel.linear.x = l_scale_*linear;

    vel_pub_.publish(vel);    


  return;
}



void Controller::getHokuyoVal(const sensor_msgs::LaserScan laser){
    //ROS_INFO("size[%d]: ", laser.intensities.size());
    std::cout << "size[%d]: " << laser.ranges.size();
    for (unsigned int i=0; i<laser.ranges.size();i++)
    {
        ranges[i] = laser.ranges[i];
        //ROS_INFO("intens[%f]: ", laser.intensities[i]);
        std::cout << "intens[" << i << "]: " << ranges[i] << "\n";
    }
}


void Controller::printHokuyoRanges(void){
    for (unsigned int i=0; i<20;i++)
    {
        std::cout << "intens[" << i << "]: " << ranges[i] << "\n";
    }
}




void Controller::neunanteDegRot(void){
    ros::Rate r(10);
    int i = 0;
    while(i < 30){
        publish(-0.475, 0);

        r.sleep();
        i++;
    }
    std::cout << "done rot!\n";

    while(i < 40){
        publish(0, 0);

        r.sleep();
        i++;
    }
    //std::cout << "stoped!\n";
}



void Controller::forwardOne(void){
    ros::Rate r(10);
    int i = 0;
    while(i < 30){
        publish(0, 0.475);

        r.sleep();
        i++;
    }
    std::cout << "done for!\n";

    while(i < 40){
        publish(0, 0);

        r.sleep();
        i++;
    }
    //std::cout << "stoped!\n";
}



/*

// command with the keyboard
void Controller::keyLoop()
{
  char c;


  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtlebot.");


  while (ros::ok())
  {

    //publish(0.0, 0.0);

    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }


    linear_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);

    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        angular_ = 0.8;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        angular_ = -0.8;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        linear_ = 0.8;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        linear_ = -0.8;
        break;
    }
    boost::mutex::scoped_lock lock(publish_mutex_);
    if (ros::Time::now() > last_publish_ + ros::Duration(1.0)) {
      first_publish_ = ros::Time::now();
    }
    last_publish_ = ros::Time::now();
    publish(angular_, linear_);


  }

  return;
}

*/
