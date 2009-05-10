/*
* Copyright (c) 2008, AMT – The Association For Manufacturing Technology (“AMT”)
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of the AMT nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* DISCLAIMER OF WARRANTY. ALL MTCONNECT MATERIALS AND SPECIFICATIONS PROVIDED
* BY AMT, MTCONNECT OR ANY PARTICIPANT TO YOU OR ANY PARTY ARE PROVIDED "AS IS"
* AND WITHOUT ANY WARRANTY OF ANY KIND. AMT, MTCONNECT, AND EACH OF THEIR
* RESPECTIVE MEMBERS, OFFICERS, DIRECTORS, AFFILIATES, SPONSORS, AND AGENTS
* (COLLECTIVELY, THE "AMT PARTIES") AND PARTICIPANTS MAKE NO REPRESENTATION OR
* WARRANTY OF ANY KIND WHATSOEVER RELATING TO THESE MATERIALS, INCLUDING, WITHOUT
* LIMITATION, ANY EXPRESS OR IMPLIED WARRANTY OF NONINFRINGEMENT,
* MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. 

* LIMITATION OF LIABILITY. IN NO EVENT SHALL AMT, MTCONNECT, ANY OTHER AMT
* PARTY, OR ANY PARTICIPANT BE LIABLE FOR THE COST OF PROCURING SUBSTITUTE GOODS
* OR SERVICES, LOST PROFITS, LOSS OF USE, LOSS OF DATA OR ANY INCIDENTAL,
* CONSEQUENTIAL, INDIRECT, SPECIAL OR PUNITIVE DAMAGES OR OTHER DIRECT DAMAGES,
* WHETHER UNDER CONTRACT, TORT, WARRANTY OR OTHERWISE, ARISING IN ANY WAY OUT OF
* THIS AGREEMENT, USE OR INABILITY TO USE MTCONNECT MATERIALS, WHETHER OR NOT
* SUCH PARTY HAD ADVANCE NOTICE OF THE POSSIBILITY OF SUCH DAMAGES.
*/

#ifndef FANUC_ADAPTER_HPP
#define FANUC_ADAPTER_HPP

#include "adapter.hpp"
#include "device_datum.hpp"

/* 
 * Provides a connection to the data available from the FANUC Focus library.
 */
class FanucAdapter : public Adapter 
{
protected:
  /* Define all the data values here */
  
  /* Events */
  StatefullAlarm mOvertravel;
  StatefullAlarm mOverheat;
  StatefullAlarm mServo;
  StatefullAlarm mDataIo;
  StatefullAlarm mMacroAlarm;
  StatefullAlarm mSpindleAlarm;
  StatefullAlarm mEstop;
  Alarm mMessage;
  Alarm mAlarm;
  
  Power mPower;
  Execution mExecution;
  IntEvent mLine; 
  ControllerMode mMode;
  
  /* Samples */
  /* Linear Axis */
  Sample mXact;
  Sample mYact;
  Sample mZact;
  
  Sample mXcom;
  Sample mYcom;
  Sample mZcom;

  Sample mXload;
  Sample mYload;
  Sample mZload;
  
  /* Spindle */
  Sample mSpindleSpeed;  
  Sample mSpindleLoad;
  
  /* Path Feedrate */
  Sample mPathFeedrate;
  
  unsigned short mFlibhndl;
  bool mConnected;
  int mDevicePort;
  const char *mDeviceIP;

protected:
  void connect();
  void disconnect();
  void getPositions();
  void getSpeeds();
  void getLine();
  void getStatus();
  void getMessages();
  void getAlarms();
  void setAlarm(StatefullAlarm &aAlarm, int data, int bit_mask);
  void getAxisLoad();
  void getSpindleLoad();
  
public:
  FanucAdapter(int aServerPort, const char *aDeviceIP, int aDevicePort);
  ~FanucAdapter();
  
  virtual void gatherDeviceData();
};

#endif
