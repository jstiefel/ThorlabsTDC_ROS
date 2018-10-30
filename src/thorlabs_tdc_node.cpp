//============================================================================
// Name        : thorlabs_tdc_node.cpp
// Author      : Julian Stiefel
// Version     : 1.0.0
// Created on  : 28.03.2018
// Copyright   : BSD 3-Clause
// Description : Main ROS node for thorlabs_tdc
//============================================================================

#include <ros/ros.h>
#include "thorlabs_tdc/Controller.hpp"

int main(int argc, char** argv)
{

	/*
	 * We have a publisher, so we can not use ros::spin(), but spinOnce() and sleep in a while loop instead.
	 * spinOnce() is needed for subscriber callback.
	 */
  ros::init(argc, argv, "thorlabs_tdc");
  ros::NodeHandle nodeHandle("~");
  ros::Rate loop_rate(100); //frequency in Hz, TDC is 10 Hz

  //initializes class, initializeKeyHandle, initializeController, initialize publisher, initialize subscriber
  thorlabs_tdc::Controller Controller(nodeHandle);

  //publish and subscribe until node gets interrupted
  while (ros::ok()){
	  Controller.motor_loop();
	  ros::spinOnce();
	  loop_rate.sleep();
  }

  //if node is interrupted, close device
  Controller.close_device();
  return 0;
}
