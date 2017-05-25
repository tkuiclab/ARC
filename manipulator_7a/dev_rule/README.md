# How To Create My Rules :sunglasses:

* Find the Port of your device
```bash
$ ls /dev/tty*

/dev/ttyUSB0
```

* Find Vendor and Product for your device
```bash
$ lsusb

Bus 003 Device 071: ID 10c4:ea60 Cygnal Integrated Products, Inc. CP210x UART Bridge / myAVR mySmartUSB light
```

* Find Serial for your device
```bash
$ udevadm info -a -n /dev/ttyUSB0 | grep '{serial}' | head -n1

    ATTRS{serial}=="IFD65000W650005"
```

* Create the file "99-example.rules" in "/etc/udev/rules.d"
```bash
$ sudo vim /etc/udev/rules.d/99-example.rules

edit or add the following line:
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="IFD65000W650005", SYMLINK+="arc/motion"
```
