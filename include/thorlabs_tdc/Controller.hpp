//============================================================================
// Name        : Controller.hpp
// Author      : Julian Stiefel
// Version     : 1.0.0
// Created on  : 28.03.2018
// Copyright   : BSD 3-Clause
// Description : Main node class providing ROS interface (subscribers,
//				 parameters, timers, etc.
//============================================================================

#pragma once

#include "thorlabs_tdc/CommunicationFunctions.hpp"

// ROS
#include <ros/ros.h>
// STD
#include <string>

#include "thorlabs_tdc/tdc_motor_service.h"
#include "thorlabs_tdc/tdc_motor_info.h"
#include <std_srvs/Trigger.h>

namespace thorlabs_tdc {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class Controller
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  Controller(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~Controller();

  void motor_loop();
  void close_device();

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  void initializeKeyHandle();

  void initializeController();
  void setStageType();
  bool homingCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
  bool positionCallback(thorlabs_tdc::tdc_motor_service::Request& request, thorlabs_tdc::tdc_motor_service::Response& response);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! ROS topic publisher.
  ros::Publisher publisher_;

  //! ROS service server
  ros::ServiceServer homing_service_;
  ros::ServiceServer position_service_;

  std::string publisherTopic_;

  //! Algorithm computation object.
  CommunicationFunctions tdc_;

  //! Create variable for publishing motor info
  thorlabs_tdc::tdc_motor_info motor;
};

} /* namespace */
