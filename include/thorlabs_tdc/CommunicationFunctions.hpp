//============================================================================
// Name        : CommunicationFunctions.hpp
// Author      : Julian Stiefel
// Version     : 1.0.0
// Created on  : 28.03.2018
// Copyright   : BSD 3-Clause
// Description : Device communication class for Thorlabs stages with
//               TDC001 controller. Based on Daniel Lehmann's tdc_manager
//				 and completed/edited.
//============================================================================

#pragma once

#ifndef COMMUNICATIONFUNCTIONS_H_
#define COMMUNICATIONFUNCTIONS_H_
// Include USB communication needed headers, given from FTDI driver
#include "thorlabs_tdc/ftd2xx.h"
#include "thorlabs_tdc/WinTypes.h"

#include <string>
#include "stdint.h"
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <ros/ros.h>
#include <cmath>
#include <unistd.h>

namespace thorlabs_tdc {

/*!
 * Class containing the algorithmic part of the package.
 */
class CommunicationFunctions
{
 public:
  /*!
   * Constructor.
   */
  CommunicationFunctions();

  /*!
   * Destructor.
   */
  virtual ~CommunicationFunctions();

  bool initializeKeyHandle(std::string serialnumber);
  bool int2hexLE(int32_t value_int, uint8_t* value_hexLE);
  bool initializeController();
  bool getPosVel(int32_t* position, uint16_t* velocity, uint8_t* RxBuffer);
  bool read_tdc(float* position, float* velocity);
  bool setPosAbs(float setpoint);
  bool setPosRel(float setpoint);
  bool setHoming(int stage_type);
  bool getStatus();
  bool getLatency();
  bool stopMot();
  bool manualPurge();
  bool closeDevice();
  void set_stage_type(int stageType);

 private:
  /** Handle used to locate the motor controller */
  FT_HANDLE keyHandle;
  /** Status word of the motor controller (True/False) */
  FT_STATUS ftStatus;
  /** Written command sent to the motor controller */
  DWORD written;

  float countsTomm(int counts);
  int mmToCounts(float mm);
  bool homingCheck;
  int stage_type;

};

#endif /* COMMUNICATIONFUNCTIONS_H_ */

} /* namespace */
