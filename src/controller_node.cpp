/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

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

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

class Controller
{
public:
  Controller();
  void keyLoop();
  void watchdog();

  double linear_, angular_;
  double l_scale_, a_scale_;
  //double static intensities[30];


  ros::NodeHandle nh_,ph_, nSub;
  ros::Time first_publish_;
  ros::Time last_publish_;
  ros::Publisher vel_pub_;
  boost::mutex publish_mutex_;

  void publish(double, double);
  void static getHokuyoVal(const sensor_msgs::LaserScan laser);


 private:
};



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

}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}





int main(int argc, char** argv)
{
  ros::init(argc, argv, "theControlleNode");
  Controller theMaster;
  ros::NodeHandle n;

  signal(SIGINT,quit);


  std::cout << "starting the controller! ";

  boost::thread my_thread(boost::bind(&Controller::keyLoop, &theMaster));
  
  
  ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&Controller::watchdog, &theMaster));


  ros::Subscriber subHokuyo = theMaster.nSub.subscribe("/my_robot/laser/scan", 1000, theMaster.getHokuyoVal);


  ros::spin();

  //my_thread.interrupt() ;
  //my_thread.join() ;
      
  return(0);
}





void Controller::watchdog()
{
  boost::mutex::scoped_lock lock(publish_mutex_);
  if ((ros::Time::now() > last_publish_ + ros::Duration(0.15)) && 
      (ros::Time::now() > first_publish_ + ros::Duration(0.50)))
    publish(0, 0);
}

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
        //intensities[i] = laser.intensities[i];
        //ROS_INFO("intens[%f]: ", laser.intensities[i]);
        std::cout << "intens[" << i << "]: " << laser.ranges[i] << "\n";
    }


}




