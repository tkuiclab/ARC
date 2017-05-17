* Install libmodbus
```bash
$ git clone https://github.com/stephane/libmodbus.git
$ cd libmodbus
$ ./autogen.sh
$ ./configure
$ make
$ sudo make install
```

* Run linear motor

```bash
$ roscore
[New terminal]

$ sudo chmod 777 /dev/ttyUSB0
$ sudo chmod 777 /dev/ttyUSB1
$ rosrun linear_motion linear_z
[if the LED in driver is red, stop terminal and reset the power of driver]
[New terminal]

$ sudo chmod 777 /dev/ttyUSB0
$ sudo chmod 777 /dev/ttyUSB1
$ rosrun linear_motion LM_Control.py <driver id> <$1>
[$1 is a char from a~l, each char represents a pulse]
[The status of linear motor will display in terminal]
```
