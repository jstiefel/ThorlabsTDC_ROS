//============================================================================
// Name        : Controller.cpp
// Author      : Julian Stiefel
// Version     : 1.0.0
// Created on  : 28.03.2018
// Copyright   : BSD 3-Clause
// Description : Main node class providing ROS interface (subscribers,
//				 parameters, timers, etc.
//============================================================================

#include "thorlabs_tdc/Controller.hpp"

namespace thorlabs_tdc {

Controller::Controller(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }

  //Set stage type from launch file
  setStageType();
  //Initialize Key handle
  initializeKeyHandle();
  //Initialize Motor Controller
  initializeController();

  publisher_ = nodeHandle_.advertise<thorlabs_tdc::tdc_motor_info>(publisherTopic_, 10);
  homing_service_ = nodeHandle_.advertiseService("thorlabs_homing_service", &Controller::homingCallback, this);
  position_service_ = nodeHandle_.advertiseService("thorlabs_set_pos", &Controller::positionCallback, this);

  ROS_INFO("Successfully launched Thorlabs Controller node.");
}

Controller::~Controller()
{
}

bool Controller::readParameters()
{
  if (!nodeHandle_.getParam("publisher_topic", publisherTopic_)) return false;
  return true;
}

void Controller::initializeKeyHandle(){
	// Get serialnumber from launch file
	std::string serialnumber;
	ros::param::get("~SerialNumber", serialnumber);
	if(!tdc_.initializeKeyHandle(serialnumber)) ROS_ERROR("Keyhandle initialization failed");
}

void Controller::initializeController(){
	if(!tdc_.initializeController()) ROS_ERROR("Motor Controller initialization failed");
}

void Controller::motor_loop(){
	tdc_.read_tdc(&motor.position, &motor.velocity);
	publisher_.publish(motor);
}

void Controller::setStageType(){
	int stage_type;
	ros::param::get("~StageType",stage_type);
	tdc_.set_stage_type(stage_type);
}

//service callback functions need to be of type bool
bool Controller::homingCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response){
	ROS_INFO("Requested homing service");
	ROS_INFO("Homing...");
	int stage_type;
	ros::param::get("~StageType",stage_type);
	if((tdc_.setHoming(stage_type))==0){
		ROS_ERROR("Device homing failed");
		response.success = 0;
	}
	else {
		response.success = 1;
	}
	return true;
}

bool Controller::positionCallback(thorlabs_tdc::tdc_motor_service::Request& request, thorlabs_tdc::tdc_motor_service::Response& response){
	//only absolute position
	//in mm
	ROS_INFO_STREAM("Requested position" << request.position_setpoint);
	if(!tdc_.setPosAbs(request.position_setpoint)) ROS_ERROR("setPosAbs failed");
	response.success = true;
	/*
	 * response does not work properly because frequency of TDC001 update is 10Hz. If asked
	 * between, there is nothing in queue. Therefore, response is not used:
	 */
//	if((tdc_.read_tdc(&response.position, &response.velocity)) == 0) ROS_ERROR("read_tdc failed for service");
	return true;
}

void Controller::close_device(){
	if(!tdc_.closeDevice()) ROS_ERROR("Motor Controller closing failed");
}

} /* namespace */
