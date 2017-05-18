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
 * robot.h
 *
 *  Created on: 2015. 12. 11.
 *      Author: zerom
 */

#ifndef ROBOTIS_DEVICE_ROBOT_H_
#define ROBOTIS_DEVICE_ROBOT_H_


#include <vector>

#include "sensor.h"
#include "dynamixel.h"
#include "dynamixel_sdk/port_handler.h"

#define DYNAMIXEL             "dynamixel"
#define SENSOR                "sensor"

#define SESSION_PORT_INFO     "port info"
#define SESSION_DEVICE_INFO   "device info"

#define SESSION_TYPE_INFO     "type info"
#define SESSION_CONTROL_TABLE "control table"

namespace robotis_framework
{

class Robot
{
public:
  std::map<std::string, dynamixel::PortHandler *> ports_;   // string: port name
  std::map<std::string, std::string>  port_default_device_; // port name, default device name

  std::map<std::string, Dynamixel *>  dxls_;       // string: joint name
  std::map<std::string, Sensor *>     sensors_;    // string: sensor name

  Robot(std::string robot_file_path, std::string dev_desc_dir_path);

  Sensor     *getSensor(std::string path, int id, std::string port, float protocol_version);
  Dynamixel  *getDynamixel(std::string path, int id, std::string port, float protocol_version);
};

}


#endif /* ROBOTIS_DEVICE_ROBOT_H_ */
