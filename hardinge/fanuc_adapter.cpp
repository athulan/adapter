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

#include "internal.hpp"
#include "Fwlib32.h"
#include "fanuc_adapter.hpp"

FanucAdapter::FanucAdapter(int aPort, const char *aDeviceIP, int aDevicePort) : 
  Adapter(aPort), 
  mPower("power"), mExecution("execution"), mLine("line"),
  mXact("Xact"), mYact("Yact"), mZact("Zact"), mZ2act("Z2act"),
  mXcom("Xcom"), mYcom("Ycom"), mZcom("Zcom"),
  mXload("Xload"), mYload("Yload"), mZload("Zload"), mZ2load("Z2load"),
  mSpindle1Load("S1load"), mSpindle2Load("S2load"), mSpindle3Load("S3load"),
  mC1act("C1act"), mC2act("C2act"), mC1load("C1load"), mC2load("C2load"),
  mSpindle1Speed("S1speed"), mSpindle2Speed("S2speed"), mSpindle3Speed("S3speed"), 
  mPathFeedrate("path_feedrate"), 
  mMode("mode"), mMessage("alarm"),
  mEstop("alarm", Alarm::eESTOP, "EMerGency", Alarm::eCRITICAL, "EMerGency Status Set"),
  mOvertravel("alarm", Alarm::eOVERLOAD, "Overtravel", Alarm::eCRITICAL, "Overtravel alarm"),
  mOverheat("alarm", Alarm::eOVERLOAD, "Overheat", Alarm::eCRITICAL, "Overheat alarm"),
  mServo("alarm", Alarm::eFAULT, "Servo", Alarm::eCRITICAL, "Servo alarm"),
  mSpindleAlarm("alarm", Alarm::eFAULT, "Spindle", Alarm::eCRITICAL, "Spindle alarm"),
  mMacroAlarm("alarm", Alarm::eOTHER, "Macro", Alarm::eINFO, "Macro alarm"),
  mDataIo("alarm", Alarm::eOTHER, "DataIO", Alarm::eERROR, "DataIO error"), mAlarm("alarm")
{
  addDatum(mMessage);
  addDatum(mEstop);
  addDatum(mOvertravel);
  addDatum(mOverheat);
  addDatum(mServo);
  addDatum(mSpindleAlarm);
  addDatum(mMacroAlarm);
  addDatum(mDataIo);
  addDatum(mAlarm);
  
  addDatum(mPower);
  addDatum(mExecution);
  addDatum(mLine);
  addDatum(mXact);
  addDatum(mYact);
  addDatum(mZact);
  addDatum(mZ2act);
  addDatum(mXcom);
  addDatum(mYcom);
  addDatum(mZcom);
  addDatum(mXload);
  addDatum(mYload);
  addDatum(mZload);
  addDatum(mZ2load);
  addDatum(mC1act);
  addDatum(mC2act);
  addDatum(mC1load);
  addDatum(mC2load);
  addDatum(mSpindle1Speed);
  addDatum(mSpindle1Load);
  addDatum(mSpindle1Speed);
  addDatum(mSpindle2Load);
  addDatum(mSpindle2Speed);
  addDatum(mSpindle3Load);
  addDatum(mSpindle3Speed);
  addDatum(mPathFeedrate);
  addDatum(mMode);

  mDevicePort = aDevicePort;
  mDeviceIP = aDeviceIP;
  mConnected = false;
}

FanucAdapter::~FanucAdapter()
{
  disconnect();
}

void FanucAdapter::gatherDeviceData()
{
  if (!mConnected)
    connect();
  else
  {
    getPositions();
	getAxisLoad();
	getSpindleLoad();
    getSpeeds();
    getLine();
    getStatus();
    getMessages();
	getAlarms();
  }
}

void FanucAdapter::disconnect()
{
  if (!mConnected)
  {
    mPower.setValue(Power::eOFF);
    cnc_freelibhndl(mFlibhndl);  
    mConnected = false;
  }
}

void FanucAdapter::connect()
{
  short ret = ::cnc_allclibhndl3(mDeviceIP, mDevicePort, 60, &mFlibhndl);
  printf("Result: %d\n", ret);
  if (ret == EW_OK) 
  {
      mConnected = true;
      mPower.setValue(Power::eON);
  }
  else
  {
    mPower.setValue(Power::eOFF);
    mConnected = false;
    Sleep(5000);
  }
}

void FanucAdapter::getPositions()
{
  if (!mConnected)
    return;

  ODBPOS PosData[MAX_AXIS]; //Position Data
  short data_num = 7;
  short ret = cnc_rdposition(mFlibhndl, -1, &data_num, &PosData[0]);
  if (ret == EW_OK)
  {
    mXact.setValue(PosData[0].abs.data / pow( (long double)10.0,( long double) PosData[0].abs.dec));
    mYact.setValue(PosData[1].abs.data / pow( (long double)10.0,(long double) PosData[1].abs.dec  ));
    mZact.setValue(PosData[2].abs.data / pow( (long double)10.0,(long double) PosData[2].abs.dec  ));
    mZ2act.setValue(PosData[3].abs.data / pow( (long double)10.0,(long double) PosData[3].abs.dec  ));
    mC1act.setValue(PosData[4].abs.data / pow( (long double)10.0,(long double) PosData[4].abs.dec  ));
    mC2act.setValue(PosData[5].abs.data / pow( (long double)10.0,(long double) PosData[5].abs.dec  ));
/*
    ODBAXIS delay;
    ret = cnc_srvdelay(mFlibhndl, 1, 8 * 1, &delay);
	if (ret == EW_OK)
      printf("Axis 0: %d\n", delay.data[0]);
    ret = cnc_srvdelay(mFlibhndl, 2, 8 * 1, &delay);
	if (ret == EW_OK)
      printf("Axis 0: %d\n", delay.data[0]);
    ret = cnc_srvdelay(mFlibhndl, 3, 8 * 1, &delay);
	if (ret == EW_OK)
      printf("Axis 0: %d\n", delay.data[0]);
*/
  }
  else
  {
    disconnect();
  }
}

void FanucAdapter::getSpeeds()
{
  if (!mConnected)
    return;
    
  ODBSPEED speed;
  short ret = cnc_rdspeed(mFlibhndl, 0, &speed);
  if (ret == EW_OK)
  {
    mPathFeedrate.setValue(speed.actf.data);
  }
  
  ODBACT2 speeds;
  ret = cnc_acts2(mFlibhndl, -1, &speeds);
  if (ret == EW_OK)
  {
    mSpindle1Speed.setValue(speeds.data[0]);
    mSpindle2Speed.setValue(speeds.data[1]);
    mSpindle3Speed.setValue(speeds.data[2]);
  }
  
}

void FanucAdapter::getLine()
{
  if (!mConnected)
    return;
  
  ODBSEQ block;
  int ret = cnc_rdseqnum(mFlibhndl, &block );
  if (ret == EW_OK)
  {
      mLine.setValue(block.data);
  }
  else
  {
    disconnect();
  }
}

void FanucAdapter::getStatus()
{
  if (!mConnected)
    return;

  ODBST status;
  int ret = cnc_statinfo(mFlibhndl, &status);
  if (ret == EW_OK)
  {
    if (status.aut == 5 || status.aut == 6) // other than no selection
      mMode.setValue(ControllerMode::eMANUAL);
    else if (status.aut == 0) // MDI for aut
      mMode.setValue(ControllerMode::eMANUAL_DATA_INPUT);
    else // Other than MDI and Manual
      mMode.setValue(ControllerMode::eAUTOMATIC);
      
    if (status.run == 0) // STOP
      mExecution.setValue(Execution::eSTOPPED);
    else if (status.run == 1 || status.motion == 3) // HOLD or motion is Wait
      mExecution.setValue(Execution::eINTERRUPTED);
    else if (status.run == 2) // STaRT
      mExecution.setValue(Execution::eACTIVE);
    else
      mExecution.setValue(Execution::eREADY);
      
    if (status.emergency == 1)
      mEstop.active();
    else
      mEstop.cleared();
  }
  else
  {
    disconnect();
  }
}

void FanucAdapter::getMessages()
{
  OPMSG messages[6];
  int ret = cnc_rdopmsg(mFlibhndl, 0, 6 + 256, messages);
  if (ret == EW_OK && messages->datano != -1)
  {
    char buf[32];
    sprintf(buf, "%04", messages->datano);
    mMessage.setValue(Alarm::eMESSAGE, buf, Alarm::eINFO, Alarm::eINSTANT, messages->data);
  }
}

void FanucAdapter::setAlarm(StatefullAlarm &aAlarm, int data, int bit_mask)
{
  if (data & bit_mask)
    aAlarm.active();
  else
    aAlarm.cleared();
}

void FanucAdapter::getAlarms()
{
  ODBALM buf ;
  cnc_alarm(mFlibhndl, &buf ) ;

  int data = buf.data;
  setAlarm(mDataIo, data, (0x1 << 2) | (0x1 << 7));
  setAlarm(mServo, data, 0x1 << 6);
  setAlarm(mMacroAlarm, data, 0x1 << 8);
  setAlarm(mOverheat, data, 0x1 << 5);
  setAlarm(mOvertravel, data, 0x1 << 4);
  setAlarm(mSpindleAlarm, data, 0x1 << 9); 
}

void FanucAdapter::getAxisLoad()
{
  ODBSVLOAD load[MAX_AXIS];
  short num = MAX_AXIS;
  short ret = cnc_rdsvmeter(mFlibhndl, &num, load);
  if (ret == EW_OK) {
    mXload.setValue(load[0].svload.data / pow( (long double) 10.0,( long double) load[0].svload.dec));
    mYload.setValue(load[1].svload.data / pow( (long double) 10.0,( long double) load[1].svload.dec));
    mZload.setValue(load[2].svload.data / pow( (long double) 10.0,( long double) load[2].svload.dec));
    mZ2load.setValue(load[3].svload.data / pow( (long double) 10.0,( long double) load[3].svload.dec));
    mC1load.setValue(load[4].svload.data / pow( (long double) 10.0,( long double) load[4].svload.dec));
    mC2load.setValue(load[5].svload.data / pow( (long double) 10.0,( long double) load[5].svload.dec));
  }
}

void FanucAdapter::getSpindleLoad()
{
  ODBSPLOAD load[4];
  short num = MAX_AXIS;
  short ret = cnc_rdspmeter(mFlibhndl, 0, &num, load);
  if (ret == EW_OK) {
    mSpindle1Load.setValue(load[0].spload.data / pow( (long double) 10.0,( long double) load[0].spload.dec));
    mSpindle2Load.setValue(load[1].spload.data / pow( (long double) 10.0,( long double) load[0].spload.dec));
    mSpindle3Load.setValue(load[2].spload.data / pow( (long double) 10.0,( long double) load[0].spload.dec));
  }
}

