/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*
 * motion_module.h
 *
 *  Created on: 2016. 1. 15.
 *      Author: zerom
 */

#ifndef ROBOTIS_FRAMEWORK_COMMON_MOTION_MODULE_H_
#define ROBOTIS_FRAMEWORK_COMMON_MOTION_MODULE_H_


#include <map>
#include <string>

#include "singleton.h"
#include "robotis_device/robot.h"
#include "robotis_device/dynamixel.h"

namespace robotis_framework
{

enum ControlMode
{
  PositionControl,
  VelocityControl,
  TorqueControl
};

class MotionModule
{
protected:
  bool        enable_;
  std::string module_name_;
  ControlMode control_mode_;

public:
  std::map<std::string, DynamixelState *> result_;

  virtual ~MotionModule() { }

  std::string getModuleName()   { return module_name_; }
  ControlMode getControlMode()  { return control_mode_; }

  void setModuleEnable(bool enable)
  {
    if(this->enable_ == enable)
      return;

    this->enable_ = enable;
    if(enable)
      onModuleEnable();
    else
      onModuleDisable();
  }
  bool getModuleEnable() { return enable_; }

  virtual void  onModuleEnable() { }
  virtual void  onModuleDisable() { }

  virtual void  initialize(const int control_cycle_msec, Robot *robot) = 0;
  virtual void  process(std::map<std::string, Dynamixel *> dxls, std::map<std::string, double> sensors) = 0;

  virtual void	stop() = 0;
  virtual bool	isRunning() = 0;
};


}


#endif /* ROBOTIS_FRAMEWORK_COMMON_MOTION_MODULE_H_ */
