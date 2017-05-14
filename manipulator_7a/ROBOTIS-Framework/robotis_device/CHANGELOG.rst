^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robotis_device
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2016-08-31)
-----------
* bug fixed (position pid gain & velocity pid gain sync write).
* added velocity_to_value_ratio to DXL Pro-H series.
* added velocity p/i/d gain and position i/d gain sync_write code.
* fixed robotis_device build_depend.
* added XM-430-W210 / XM-430-W350 device file.
* rename (present_current\_ -> present_torque\_)
* modified torque control code
* added device file for MX-64 / MX-106
* adjusted position min/max value. (MX-28, XM-430)
* Contributors: Zerom, Pyo

0.1.1 (2016-08-18)
-----------
* updated the package information

0.1.0 (2016-08-12)
-----------
* first public release for Kinetic
* modified the package information for release
* develop branch -> master branch
* Setting the license to BSD.
* add SensorState
  add Singleton template
* XM-430 / CM-740 device file added.
  Sensor device added.
* modified.
* variable name changed.
  ConvertRadian2Value / ConvertValue2Radian function bug fixed.
* added code to support the gazebo simulator
* renewal
* Contributors: ROBOTIS, ROBOTIS-zerom, pyo
