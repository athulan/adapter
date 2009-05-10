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
#include "emc_adapter.hpp"

EmcAdapter::EmcAdapter(int aPort, const char *aNmlFile)
  : Adapter(aPort, 10), 
    mAlarm("alarm"), mPower("power"), mExecution("execution"), mLine("line"),
    mXact("Xact"), mYact("Yact"), mZact("Zact"), 
    mXcom("Xcom"), mYcom("Ycom"), mZcom("Zcom"),
    mSpindleSpeed("spindle_speed"), mPathFeedrate("path_feedrate"),
    mProgram("program"), mMode("mode"), mBlock("block")
{
  addDatum(mAlarm);
  addDatum(mPower);
  addDatum(mExecution);
  addDatum(mLine);
  addDatum(mXact);
  addDatum(mYact);
  addDatum(mZact);
  addDatum(mXcom);
  addDatum(mYcom);
  addDatum(mZcom);
  addDatum(mSpindleSpeed);
  addDatum(mPathFeedrate);
  addDatum(mProgram);
  addDatum(mMode);
  addDatum(mBlock);
  
  mErrorString[0] = 0;
  mEmcErrorBuffer = mEmcStatusBuffer = 0;
  strcpy(mNmlFile, aNmlFile);
  mConnected = false;
}

bool EmcAdapter::connect()
{
  int retval = 0;
  if (mEmcStatusBuffer == 0)
  {
    mEmcStatusBuffer = new RCS_STAT_CHANNEL(emcFormat, "emcStatus", "adapter", mNmlFile);
    if (! mEmcStatusBuffer->valid() || EMC_STAT_TYPE != mEmcStatusBuffer->peek())
    {
      rcs_print_error("emcStatus buffer not available\n");
      delete mEmcStatusBuffer;
      
      mEmcStatusBuffer = 0;
      mPower.setValue(Power::eOFF);
      return false;
    }
  }
    
  if (mEmcErrorBuffer == 0)
  {
    mEmcErrorBuffer = new NML(nmlErrorFormat, "emcError", "adapter", mNmlFile);
    if (!mEmcErrorBuffer->valid())
    {
      rcs_print_error("emcError buffer not available\n");
      delete mEmcErrorBuffer;
      mEmcErrorBuffer = 0;
      mPower.setValue(Power::eOFF);
      return false;
    }
  }
  mConnected = true;
  return true;
}

void EmcAdapter::disconnect()
{
  if (mConnected)
  {
    if (mEmcErrorBuffer)
      delete mEmcErrorBuffer;
    mEmcErrorBuffer = 0;

    if (mEmcStatusBuffer)
      delete mEmcStatusBuffer;
    mEmcStatusBuffer = 0;
    mConnected = false;
  }
}

void EmcAdapter::actual()
{
  EmcPose &pose = mEmcStatus.motion.traj.actualPosition;
  mXact.setValue(pose.tran.x);
  mYact.setValue(pose.tran.y);
  mZact.setValue(pose.tran.z);
}

void EmcAdapter::commanded()
{
  EmcPose &pose = mEmcStatus.motion.traj.position;
  mXcom.setValue(pose.tran.x);
  mYcom.setValue(pose.tran.y);
  mZcom.setValue(pose.tran.z);
}

void EmcAdapter::spindle()
{
  mSpindleSpeed.setValue(mEmcStatus.motion.spindle.speed);
}

void EmcAdapter::feedrate()
{
  mPathFeedrate.setValue(mEmcStatus.motion.traj.current_vel);
}

void EmcAdapter::program()
{
  if (mProgram.setValue(mEmcStatus.task.file))
  {
    // Read the file and split it into lines.
    mBlocks.readFile(mEmcStatus.task.file);
  }

  int line = mEmcStatus.motion.traj.id;
  if (mLine.setValue(line))
  {
    if (line <= 0)
      mBlock.setValue("");
    else
      mBlock.setValue(mBlocks[line - 1]);
  }
}

void EmcAdapter::machine()
{
  if (mEmcStatus.task.state == EMC_TASK_STATE_ON)
  {
    if (mPower.setValue(Power::eON))
    {
      Alarm warn("alarm");
      warn.setValue(Alarm::eMESSAGE, "ON", Alarm::eINFO, Alarm::eINSTANT, "Power ON");
      sendDatum(&warn);
    }
  }
  else
  {
    if (mPower.setValue(Power::eOFF))
    {
      Alarm warn("alarm");
      warn.setValue(Alarm::eMESSAGE, "OFF", Alarm::eINFO, Alarm::eINSTANT, "Power OFF");
      sendDatum(&warn);
    }
  }
}

void EmcAdapter::execution()
{
  if (mEmcStatus.task.mode == EMC_TASK_MODE_MANUAL)
    mMode.setValue(ControllerMode::eMANUAL);
  else if (mEmcStatus.task.mode == EMC_TASK_MODE_AUTO)
    mMode.setValue(ControllerMode::eAUTOMATIC);
  else if (mEmcStatus.task.mode == EMC_TASK_MODE_MDI)
    mMode.setValue(ControllerMode::eMANUAL_DATA_INPUT);
  
  if (mEmcStatus.task.interpState == EMC_TASK_INTERP_PAUSED)
    mExecution.setValue(Execution::eINTERRUPTED);
  else if (mEmcStatus.task.interpState == EMC_TASK_INTERP_IDLE)
    mExecution.setValue(Execution::eREADY);
  else if (mEmcStatus.task.interpState == EMC_TASK_INTERP_WAITING)
    mExecution.setValue(Execution::eACTIVE);  
}

void EmcAdapter::alarms()
{
  if (mEmcStatus.task.state == EMC_TASK_STATE_ESTOP)
    mAlarm.setValue(Alarm::eESTOP, "ESTOP", Alarm::eCRITICAL, Alarm::eACTIVE, "ESTOP Pressed");
  else
    mAlarm.setValue(Alarm::eESTOP, "ESTOP", Alarm::eCRITICAL, Alarm::eCLEARED, "ESTOP Reset");
}

EmcAdapter::~EmcAdapter()
{
  disconnect();
}

void EmcAdapter::gatherDeviceData()
{
  if (!mConnected)
  {
    if (!connect())
      sleep(5);
  }
  else
  {
    if (!mEmcStatusBuffer->valid())
    {
      disconnect();
      return;
    }
    
    if(mEmcStatusBuffer->peek() == EMC_STAT_TYPE) {
      memcpy(&mEmcStatus, mEmcStatusBuffer->get_address(), sizeof(EMC_STAT));
      actual();
      commanded();
      spindle();
      program();
      machine();
      execution();
      alarms();
    }
  } 
}

