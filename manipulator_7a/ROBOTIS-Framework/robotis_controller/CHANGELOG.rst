^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robotis_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2016-08-31)
-----------
* bug fixed (position pid gain & velocity pid gain sync write).
* added velocity_to_value_ratio to DXL Pro-H series.
* changed some debug messages.
* added velocity p/i/d gain and position i/d gain sync_write code.
* SyncWriteItem bug fixed.
* add function / modified the code simple (using auto / range based for loop)
* added XM-430-W210 / XM-430-W350 device file.
* rename ControlMode(CurrentControl -> TorqueControl)
* rename (port_to_sync_write_torque\_ -> port_to_sync_write_current\_)
* rename (present_current\_ -> present_torque\_)
* modified torque control code
* fixed typos / changed ROS_INFO -> fprintf (for processing speed)
* startTimer() : after bulkread txpacket(), need some sleep()
* changed the order of processing in the Process() function.
* added missing mutex for gazebo
* fixed crash when running in gazebo simulation
* sync write bug fix.
* added position_p_gain sync write
* MotionModule/SensorModule member variable access changed (public -> protected).
* Contributors: Jay Song, Zerom, Pyo, SCH

0.1.1 (2016-08-18)
-----------
* updated the package information

0.1.0 (2016-08-12)
-----------
* first public release for Kinetic
* modified the package information for release
* develop branch -> master branch
* function name changed : DeviceInit() -> InitDevice()
* Fixed high CPU consumption due to busy waits
* add SensorState
  add Singleton template
* XM-430 / CM-740 device file added.
  Sensor device added.
* added code to support the gazebo simulator
* added first bulk read failure protection code
* renewal
* Contributors: Alexander Stumpf, Jay Song, Zerom, Pyo
