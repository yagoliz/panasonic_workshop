# How to set udev rules for USB components
USB devices used for these workshop are:
* TerMITes (Device developed at the [MIT Media Lab](https://www.media.mit.edu/projects/termites/overview/) by Carson Smuts)
* USB camera
* Arduino board

[Hardware/Udev](https://wiki.debian.org/udev) rules are a great tool to organize your peripherals, since they allow you to give them unique symbolic names.

In the case of the these USB devices, while in this folder, open a terminal and run:
```bash
user@machine:this_directory$ chmod +x setRule.sh
user@machine:this_directory$ ./setRule.sh
```

Once this is done, you can chech that the udev rules were properly loaded by typing in the terminal:
```bash
user@machine:~$ ls /dev/termite
/dev/termite (in blue color)
user@machine:~$ ls /dev/panasonic_camera
/dev/panasonic_camera (in blue color)
user@machine:~$ ls /dev/arduino_led
/dev/arduino_led (in blue color)
```
