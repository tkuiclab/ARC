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

/* Author: Ryu Woon Jung (Leon) */

//
// *********     Factory Reset Example      *********
//
//
// Available Dynamixel model on this example : All models using Protocol 2.0
// This example is designed for using a Dynamixel PRO 54-200, and an USB2DYNAMIXEL.
// To use another Dynamixel model, such as X series, see their details in E-Manual(support.robotis.com) and edit below variables yourself.
// Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 3 (Baudrate : 1000000 [1M])
//

// Be aware that:
// This example resets all properties of Dynamixel to default values, such as %% ID : 1 / Baudnum : 1 (Baudrate : 57600)
//

using System;
using System.Threading;
using dynamixel_sdk;

namespace factory_reset
{
  class FactoryReset
  {
    // Control table address
    public const int ADDR_PRO_BAUDRATE  = 8;                // Control table address is different in Dynamixel model

    // Protocol version
    public const int PROTOCOL_VERSION   = 2;                // See which protocol version is used in the Dynamixel

    // Default setting
    public const int DXL_ID             = 1;                // Dynamixel ID: 1
    public const int BAUDRATE           = 1000000;
    public const string DEVICENAME      = "COM1";   // Check which port is being used on your controller
                                                            // ex) "COM1"   Linux: "/dev/ttyUSB0"

    public const int FACTORYRST_DEFAULTBAUDRATE = 57600;    // Dynamixel baudrate set by factoryreset
    public const int NEW_BAUDNUM        = 3;                // New baudnum to recover Dynamixel baudrate as it was
    public const byte OPERATION_MODE    = 0x01;             // 0xFF : reset all values
                                                            // 0x01 : reset all values except ID
                                                            // 0x02 : reset all values except ID and baudrate

    public const int COMM_SUCCESS       = 0;                // Communication Success result value
    public const int COMM_TX_FAIL       = -1001;            // Communication Tx Failed

    static void Main(string[] args)
    {
      // Initialize PortHandler Structs
      // Set the port path
      // Get methods and members of PortHandlerLinux or PortHandlerWindows
      int port_num = dynamixel.portHandler(DEVICENAME);

      // Initialize PacketHandler Structs
      dynamixel.packetHandler();

      int dxl_comm_result = COMM_TX_FAIL;                  // Communication result

      byte dxl_error = 0;                                  // Dynamixel error
      byte dxl_baudnum_read;                               // Read baudnum

      // Open port
      if (dynamixel.openPort(port_num))
      {
        Console.WriteLine("Succeeded to open the port!");
      }
      else
      {
        Console.WriteLine("Failed to open the port!");
        Console.WriteLine("Press any key to terminate...");
        Console.ReadKey();
        return;
      }

      // Set port baudrate
      if (dynamixel.setBaudRate(port_num, BAUDRATE))
      {
        Console.WriteLine("Succeeded to change the baudrate!");
      }
      else
      {
        Console.WriteLine("Failed to change the baudrate!");
        Console.WriteLine("Press any key to terminate...");
        Console.ReadKey();
        return;
      }


      // Read present baudrate of the controller
      Console.WriteLine("Now the controller baudrate is : {0}", dynamixel.getBaudRate(port_num));

      // Try factoryreset
      Console.WriteLine("[ID: {0}] Try factoryreset : ", DXL_ID);
      dynamixel.factoryReset(port_num, PROTOCOL_VERSION, DXL_ID, OPERATION_MODE);
      if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
      {
        Console.WriteLine("Aborted");
        dynamixel.printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
        return;
      }
      else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
      {
        dynamixel.printRxPacketError(PROTOCOL_VERSION, dxl_error);
      }

      // Wait for reset
      Console.WriteLine("Wait for reset...");
      Thread.Sleep(2000);

      Console.WriteLine("[ID: {0}] factoryReset Success!", DXL_ID);

      // Set controller baudrate to dxl default baudrate
      if (dynamixel.setBaudRate(port_num, FACTORYRST_DEFAULTBAUDRATE))
      {
        Console.WriteLine("Succeed to change the controller baudrate to : {0}", FACTORYRST_DEFAULTBAUDRATE);
      }
      else
      {
        Console.WriteLine("Failed to change the controller baudrate");
        Console.WriteLine("Press any key to terminate...");
        Console.ReadKey();
        return;
      }

      // Read Dynamixel baudnum
      dxl_baudnum_read = dynamixel.read1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_BAUDRATE);
      if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
      {
        dynamixel.printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
      }
      else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
      {
        dynamixel.printRxPacketError(PROTOCOL_VERSION, dxl_error);
      }
      else
      {
        Console.WriteLine("[ID: {0}] Dynamixel baudnum is now : {1}", DXL_ID, dxl_baudnum_read);
      }

      // Write new baudnum
      dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_BAUDRATE, NEW_BAUDNUM);
      if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
      {
        dynamixel.printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
      }
      else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
      {
        dynamixel.printRxPacketError(PROTOCOL_VERSION, dxl_error);
      }
      else
      {
        Console.WriteLine("[ID: {0}] Set Dynamixel baudnum to : {1}", DXL_ID, NEW_BAUDNUM);
      }

      // Set port baudrate to BAUDRATE
      if (dynamixel.setBaudRate(port_num, BAUDRATE))
      {
        Console.WriteLine("Succeed to change the controller baudrate to : {0}", BAUDRATE);
      }
      else
      {
        Console.WriteLine("Failed to change the controller baudrate");
        Console.WriteLine("Press any key to terminate...");
        Console.ReadKey();
        return;
      }

      Thread.Sleep(200);

      // Read Dynamixel baudnum
      dxl_baudnum_read = dynamixel.read1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_BAUDRATE);
      if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
      {
        dynamixel.printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
      }
      else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
      {
        dynamixel.printRxPacketError(PROTOCOL_VERSION, dxl_error);
      }
      else
      {
        Console.WriteLine("[ID: {0}] Dynamixel Baudnum is now : {1}", DXL_ID, dxl_baudnum_read);
      }

      // Close port
      dynamixel.closePort(port_num);

      return;
    }
  }
}
