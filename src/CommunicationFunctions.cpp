//============================================================================
// Name        : CommunicationFunctions.cpp
// Author      : Julian Stiefel
// Version     : 1.0.0
// Created on  : 28.03.2018
// Copyright   : BSD 3-Clause
// Description : Device communication class for Thorlabs stages with
//               TDC001 controller. Based on Daniel Lehmann's tdc_manager
//				 and completed/edited.
//============================================================================

#include "thorlabs_tdc/CommunicationFunctions.hpp"

namespace thorlabs_tdc {

CommunicationFunctions::CommunicationFunctions()
{
  ftStatus = 0;
  keyHandle = NULL;
  written = 0;
  homingCheck = 0;
  stage_type = 0;
}

CommunicationFunctions::~CommunicationFunctions()
{
}

void CommunicationFunctions::set_stage_type(int stageType){
	//global stage type variable from launch file is set
	//1: linear, 2: rotational
	stage_type = stageType;
}

float CommunicationFunctions::countsTomm(int counts){
	//see calculations, called mm, but also means deg in case 2
	float mm;
	switch(stage_type){
	case 1:
		mm = counts/34304.;
		break;

	case 2:
		//actually degree
		mm = (360/589824.) * counts;
		break;
	}
	return mm;
}

int CommunicationFunctions::mmToCounts(float mm){
	//see calculations
	int counts;
	switch(stage_type){
	case 1:
		counts = 34304 * mm;
		break;

	case 2:
		//actually degree
		counts = (589824/360.) * mm;
		break;
	}
	return counts;
}

bool CommunicationFunctions::initializeKeyHandle(std::string serialnumber){
  //INITIALIZATION//
  /*
   * This function initializes the TDC motor controller and finds its corresponding keyhandle.
   *
   * @param serialnumber Serial number of TDC controller
   * @return A boolean indicating the success of the method.
   */
  keyHandle = NULL;

  // To open the device the Vendor and Product ID must set correct
  ftStatus = FT_SetVIDPID(0x403,0xfaf0);
  if(ftStatus != FT_OK) {
    ROS_ERROR("FT_SetVIDPID failed \n");
    return false;
  }

  sleep(2); //2s sleep is necessary here, otherwise device will not be opened

  DWORD iNumDevs;
  ftStatus = FT_CreateDeviceInfoList(&iNumDevs);
      if (FT_OK != ftStatus)
      {
          ROS_INFO_STREAM("Error: FT_CreateDeviceInfoList:" << (int)ftStatus);
      }
  ROS_INFO_STREAM("Devices: " << iNumDevs);

  /*
   * Now it is time to open the device. If it does not initialize within 20
   * tries, then there is likely a connection error
   */
  const char* tmp = serialnumber.c_str();
  int numAttempts=0;
  while (keyHandle ==0){
    ftStatus = FT_OpenEx(const_cast<char*>(tmp),FT_OPEN_BY_SERIAL_NUMBER, &keyHandle);
    if (numAttempts++>20){
      ROS_ERROR("Device Could Not Be Opened \n");
      return false;
    }
  }
  printf("Device Opened \n");


  /*
   * Once the device is connected, we must set the baudrate and timeout according to the
   * specifications provided by the manufacturer in the APT Communication Protocol
   */

  // Set baud rate to 115200
  ftStatus = FT_SetBaudRate(keyHandle,115200);
    if(ftStatus != FT_OK) {
      ROS_ERROR("FT_SetBaudRate failed \n");
      return false;
      }

//  // Set USB request transfer size
//  DWORD InTransferSize = 64;
//  ftStatus = FT_SetUSBParameters(keyHandle, InTransferSize, 0);
//    if(ftStatus != FT_OK){
//      ROS_ERROR("FT_SetUSBParameters failed \n");
//      return false;
//    }
//
//  // Set latency timer
//  UCHAR LatencyTimer = 10;
//  ftStatus = FT_SetLatencyTimer(keyHandle, LatencyTimer);
//    if(ftStatus != FT_OK){
//      ROS_ERROR("FT_SetUSBParameters failed \n");
//      return false;
//    }

  // 8 data bits, 1 stop bit, no parity
  ftStatus = FT_SetDataCharacteristics(keyHandle, FT_BITS_8, FT_STOP_BITS_1, FT_PARITY_NONE);
    if(ftStatus != FT_OK) {
      ROS_ERROR("FT_SetDataCharacteristics failed \n");
      return false;
      }

  // Pre purge dwell 50ms.
  usleep(50);

  // Purge the device.
  ftStatus = FT_Purge(keyHandle, FT_PURGE_RX | FT_PURGE_TX);
    if(ftStatus != FT_OK) {
      ROS_ERROR("FT_Purge failed \n");
      return false;
      }

  // Post purge dwell 50ms.
  usleep(50);

  // Reset device.
  ftStatus = FT_ResetDevice(keyHandle);
    if(ftStatus != FT_OK) {
      ROS_ERROR("FT_ResetDevice failed \n");
      return false;
      }

  // Set flow control to RTS/CTS.
  ftStatus = FT_SetFlowControl(keyHandle, FT_FLOW_RTS_CTS, 0, 0);
    if(ftStatus != FT_OK) {
      ROS_ERROR("FT_SetFlowControl failed \n");
      return false;
      }

  // Set RTS.
  ftStatus = FT_SetRts(keyHandle);
    if(ftStatus != FT_OK) {
      ROS_ERROR("FT_SetRts failed \n");
      return false;
      }
  return true;
}

bool CommunicationFunctions::initializeController(){
	/*
    * This function initializes the motor controller. Additionally, the minimal/maximal velocity
    * and acceleration of the movements are set.
    * Optional: MGMSG_HW_START_UPDATEMSGS can be set here to fill the queue of TDC with update
    * messages. We do not rely on complete data and just need actual position and velocity.
    * Therefore, we will send a separate request each time we need the data and there is no
    * need to search the queue for the most recent data.
    * @return A boolean indicating whether or not the controller is initialized correctly.
    */

	// Start Update Message
	uint8_t startUpdateMsgs[6] = {0x11,0x0,0xa,0x0,0x50,0x1};

	// Set command to device
	ftStatus = FT_Write(keyHandle, startUpdateMsgs, (DWORD)6, &written);
	if(ftStatus != FT_OK){
		ROS_ERROR("Command could not be transmitted. StartUpdateMsg");
		return false;
	}

  // Set min./max. velocity and acceleration
  // max velocity of hardware is 2.4mm/s
  float min_velocity_mm = 0; // [mm/s]
  float max_velocity_mm = 90; // [mm/s] //default: 45
  float acceleration_mm = 0.4; // [mm/s^2] //default: 0.2

  int32_t min_velocity_count = (int32_t)mmToCounts(min_velocity_mm);
  int32_t max_velocity_count = (int32_t)mmToCounts(max_velocity_mm);
  int32_t acc_count = (int32_t)mmToCounts(acceleration_mm);

  uint8_t max_vel_buf[4] = {0};
  uint8_t min_vel_buf[4] = {0};
  uint8_t acc_buf[4] = {0};

  // Change from integer to 8-bit sequences
  int2hexLE(max_velocity_count,max_vel_buf);
  int2hexLE(min_velocity_count,min_vel_buf);
  int2hexLE(acc_count,acc_buf);

  uint8_t setVelbuf[20] = {0x13,0x04,0x0e,0x00,0x81,0x01,0x01,0x00,min_vel_buf[0],min_vel_buf[1],min_vel_buf[2],min_vel_buf[3],acc_buf[0],acc_buf[1],acc_buf[2],acc_buf[3],max_vel_buf[0],max_vel_buf[1],max_vel_buf[2],max_vel_buf[3]};

  // Set command to device
  ftStatus = FT_Write(keyHandle, setVelbuf, (DWORD)20, &written);
  if(ftStatus != FT_OK){
    ROS_ERROR("Command could not be transmitted. Set min./max. velocity and acceleration \n");
    return false;
  }

  // Set homing parameters
  float home_velocity_mm = 15; // [mm/s]
  int32_t home_velocity_count = (int32_t)mmToCounts(home_velocity_mm);
  uint8_t home_vel_buf[4] = {0};
  int2hexLE(home_velocity_count,home_vel_buf);
  uint8_t setHomebuf[20] = {0x40,0x04,0x0e,0x00,0x81,0x01,0x01,0x00,0x00,0x00,0x00,0x00,home_vel_buf[0],home_vel_buf[1],home_vel_buf[2],home_vel_buf[3],0x00,0x00,0x00,0x00};

  // Set command to device
  ftStatus = FT_Write(keyHandle, setHomebuf, (DWORD)20, &written);
  if(ftStatus != FT_OK){
    ROS_ERROR("Command could not be transmitted. Set homing parameters \n");
    return false;
  }
  ROS_INFO("Initializing Controller completed \n");
  return true;
}

bool CommunicationFunctions::int2hexLE(int32_t value_int, uint8_t* value_hexLE){
	/*
    * This function takes in a 32-bit integer value_int, split it in four 8-bit sequences
    * and saves the result in the Little-Endian format in the array value_hexLE.
    * @param value_int: 32-bit integer values
    * @param value_hexLE: A pointer to an 8-bit array, where the integer value will be saved in the Little-Endian format
    */
  value_hexLE[0] = value_int;
  value_hexLE[1] = value_int>>8;
  value_hexLE[2] = value_int>>16;
  value_hexLE[3] = value_int>>24;
  return true;
}

bool CommunicationFunctions::manualPurge(){
  // Pre purge dwell 50ms.
  usleep(50);

  // Purge the device.
  ftStatus = FT_Purge(keyHandle, FT_PURGE_RX | FT_PURGE_TX);
    if(ftStatus != FT_OK) {
      ROS_ERROR("FT_Purge failed \n");
      return false;
      }

  // Post purge dwell 50ms.
  usleep(50);

  return true;
}

bool CommunicationFunctions::getStatus(){
  DWORD receive_queueRx;
  DWORD transmit_queueTx;
  DWORD event_status;
  ftStatus = FT_GetStatus(keyHandle, &receive_queueRx, &transmit_queueTx, &event_status);
  if(ftStatus != FT_OK){
    ROS_ERROR("Command could not be transmitted: Request status \n");
    return false;
  }
  ROS_INFO_STREAM("Receive queue: " << receive_queueRx << " Transmit queue: " << transmit_queueTx << " Event status: " << event_status);
  return true;
}

bool CommunicationFunctions::getLatency(){
  unsigned char latency;

  ftStatus = FT_GetLatencyTimer(keyHandle, &latency);
  if(ftStatus != FT_OK){
    ROS_ERROR("Command could not be transmitted: Request latency \n");
    return false;
  }
  ROS_INFO_STREAM("Latency: " << (unsigned)latency);
  return true;
}

bool CommunicationFunctions::read_tdc(float* position, float* velocity){
	/*
	* This function reads out the TDC motor controller and saves the actual position
	* and velocity in encoder counts.
	* Optional: If we would use MGMSG_HW_START_UPDATEMSGS, we would need to send a
	* "server alive" message in every call (MGMSG_MOT_ACK_DCSTATUSUPDATE) to confirm
	* that the host computer is still listening, otherwise, the motor controller
	* would stop sending update messages after 50 messages.
	*
	* Velocity is faulty. Probably error from Thorlabs.


	* @param position A pointer to a variable, where the position will be stored.
	* @param velocity A pointer to a variable, where the velocity will be stored.
	* @return A boolean indicating whether or not the read out was successful.
	*/

	/** Server alive message buffer */
	uint8_t serverAlive[6] = {0x92,0x04,0x0,0x0,0x50,0x01};
	uint8_t *RxBuffer = new uint8_t[256];
	DWORD RxBytes = 0;
	DWORD BytesReceived = 0;

	/* Note about regular messages:
	* If we turn regular messages on and we read all data which is available in
	* queue of TDC001 in the RxBuffer, this could exceed the 256 bytes
	* length of RxBuffer, because not only the recent data is in queue. It is
	* then not too easy to get the right data (most recent and right message).
	*/

	// Set command to device that the host computer is still listening
	ftStatus = FT_Write(keyHandle, serverAlive, (DWORD)6, &written);
	if(ftStatus != FT_OK){
		//*****only for DEBUGGING*****
		//ROS_ERROR("Command could not be transmitted: Server alive");
		return false;
	}

	// Get number of bytes in queue of TDC001
	FT_GetQueueStatus(keyHandle,&RxBytes);

	// Check if there are bytes in queue before reading them, otherwise do
	// not read anything in, frequency of new data in TDC queue is low
	if(RxBytes>0){
	ftStatus=FT_Read(keyHandle,RxBuffer,RxBytes,&BytesReceived);
	if(ftStatus != FT_OK){
	  ROS_ERROR("Read device failed! \n");
	  return false;
		}
	}

	/*
	 * Check if enough bytes are received, i.e. if signal is right.
	 * If not 20 bytes are received, there was either no new data available.
	 * (BytesReceived initialized as 0) or data received was wrong.
	 */

	if(!(BytesReceived == 20)){
	//    ROS_INFO("No new status update in queue \n");
	return false;
	}

	//  // Output number of bytes received
	//  ROS_INFO_STREAM("Bytes received: " << BytesReceived);

	int32_t position_counts;
	uint16_t velocity_counts;
	getPosVel(&position_counts,&velocity_counts,RxBuffer);

	//convert to mm/degree:
	*position = countsTomm((int)position_counts);
	*velocity = countsTomm((int)velocity_counts);

	//  // Output whole RxBuffer:
	//  ROS_INFO_STREAM("RxBuffer:");
	//  for (int i=0; i<int(RxBytes); i++){
	//    ROS_INFO_STREAM(unsigned(RxBuffer[i]) << "\n");
	//  }

	// Delete receive buffer
	delete[] RxBuffer;
	RxBuffer = NULL;

	return true;
}

bool CommunicationFunctions::getPosVel(int32_t* position, uint16_t* velocity, uint8_t* RxBuffer){
	/*
    * This function saves the information of position and velocity from the receive buffer (Rx)
    * in the specified position and velocity variables. The function only reads in update messages.
    *
    * Used message is 0x0491

    * @param position A pointer to a variable, where the position in a 32-bit integer is stored.
    * @param velocity A pointer to a variable, where the velocity in a unsigned 16-bit integer is
    * stored.
    * @param RxBuffer A pointer to RxBuffer, where the information of position and velocity is stored
    * in the Little-Endian format.
    */

  // Check if received message is MGMSG_MOT_GET_DCSTATUSUPDATE and check if we have a data
  // packet length of 14 bytes
  if((RxBuffer[0] == 0x91) && (RxBuffer[2] == 0xE)){
    *position = (RxBuffer[8])|(RxBuffer[9]<<8)|(RxBuffer[10]<<16)|(RxBuffer[11]<<24);
    *velocity = (RxBuffer[12])|(RxBuffer[13]<<8);
    }
  return true;
}

bool CommunicationFunctions::stopMot(){
  uint8_t stop_msgs[6] = {0x65,0x04,0x01,0x01,0x50,0x01};
  ftStatus = FT_Write(keyHandle, stop_msgs, (DWORD)6, &written);
  if(ftStatus != FT_OK){
    ROS_ERROR("Command could not be transmitted: Motor stop \n");
    return false;
  }
  return true;
}

bool CommunicationFunctions::setPosAbs(float setpoint){
   /*
    * This function moves the stage to the specified absolute position in encoder counts.

    * @param setpoint float which has to be converted to 32-bit integer value to move in encoder counts
    * @return A boolean indicating whether or not the command is sent successfully.
    */

	//target position in mm or degree is converted to int (counts)
  if(setpoint >= -33 && setpoint <= 49 && homingCheck==1){
  uint8_t absPos[4]={0};
  int2hexLE(mmToCounts(setpoint),absPos);
  uint8_t AbsPosBuf[12]={0x53,0x04,0x06,0x00,0x81,0x01,0x01,0x00,absPos[0],absPos[1],absPos[2],absPos[3]};
  ftStatus = FT_Write(keyHandle, AbsPosBuf, (DWORD)12, &written);
  if(ftStatus != FT_OK){
    ROS_ERROR("Command could not be transmitted: Move absolute position \n");
    return false;
    }
  }
  else{
	  ROS_ERROR("Setpoint out of range or not homed");
  }
  return true;
}

bool CommunicationFunctions::setPosRel(float setpoint){
   /*
    * This function moves the stage to the specified relative position in encoder counts.

    * @param setpoint 32-bit integer value to move in encoder counts
    * @return A boolean indicating whether or not the command is sent successfully.
    */

	//target position in mm/degree converted to int (counts)
  uint8_t relPos[4]={0};
  int2hexLE(mmToCounts(setpoint),relPos);
  uint8_t RelPosBuf[12]={0x48,0x04,0x06,0x00,0x81,0x01,0x01,0x00,relPos[0],relPos[1],relPos[2],relPos[3]};

  ftStatus = FT_Write(keyHandle, RelPosBuf, (DWORD)12, &written);
  if(ftStatus != FT_OK){
    ROS_ERROR("Command could not be transmitted: Move relative position \n");
    return false;
    }
  return true;
}

bool CommunicationFunctions::setHoming(int stage_type){
   /*
    * This function homes the stage, which depends on the stage type. MTS50 linear
    * stages are homed by using limit switches. CR1 rotational stages are homed by
    * moving the stage manually to the desired position and set the position counter
    * of the motor controller to zero. The stage type is specified in the launch file.
    * 1 is linear, 2 is rotational.
    * This function checks if homing was completed.
    * @return A boolean indicating whether or not the command is sent successfully.
    */

	uint8_t *RxBuffer = new uint8_t[256];
	DWORD RxBytes = 0;
	DWORD BytesReceived = 0;

  // Stage type 1
  uint8_t bufHome[6]={0x43,0x04,0x1,0x0,0x50,0x1};
  // Stage type 2
  uint8_t bufHomePos[12]={0x09,0x04,0x06,0x81,0x01,0x01,0x00,0x0,0x0,0x0,0x0};

  switch(stage_type){
    case 1:
      //move a short relative distance, because homing does not work if already at homing point
      setPosRel(0.2);
      sleep(1);
      ftStatus = FT_Write(keyHandle, bufHome, (DWORD)6, &written);
      if(ftStatus != FT_OK){
        ROS_ERROR("Command could not be transmitted: Thorlabs Homing \n");
        return false; //stops execution of function immediately and returns value
        }
      else {
    	  bool success = 0;

    	  while(success==0){

    	  //wait for homing completed, this works only if homing was executed and not if limit is already reached:
    	  FT_GetQueueStatus(keyHandle,&RxBytes);

    	  // Check if there are bytes in queue before reading them, otherwise do
    	  // not read anything in
    	  if(RxBytes>0){
    	    ftStatus=FT_Read(keyHandle,RxBuffer,RxBytes,&BytesReceived);
    	    if(ftStatus != FT_OK){
    	      ROS_ERROR("Read device failed! \n");
    	      return false;
    	    }
    	  }
    	  if((RxBuffer[0] == 0x44) && (RxBuffer[1] == 0x04)){
    		  ROS_INFO("Thorlabs Homing reached");
    		  success=1;
    	  	  }
    	  }
      }
        ROS_INFO("Thorlabs Homing completed \n");
        break;

    case 2: //zero is set at the current position
      ftStatus = FT_Write(keyHandle, bufHomePos, (DWORD)12, &written);
      if(ftStatus != FT_OK){
        ROS_ERROR("Command could not be transmitted: Homing \n");
        return false;
        }
      else {
        ROS_INFO("Thorlabs revolute homing completed \n");
      }
      break;
  }
  homingCheck = 1; //homing was completed check var.
  return true;
}


bool CommunicationFunctions::closeDevice(){
	FT_Close(keyHandle);
	if (ftStatus == FT_OK){
		ROS_INFO("Device successfully closed. \n");
		return true;
	}
	else {
		return false;
	}
}

} /* namespace */
