# audio_research
This repository is from research done on audio localizationa and environmental perception in autonomous systems. This is accomplished through ROS and the [Re-Speaker Far Field Microphone array](https://www.seeedstudio.com/ReSpeaker-Mic-Array-Far-field-w%2F-7-PDM-Microphones-p-2719.html).

The purpose of this package was to create an easy plug and play platform to do audio research in robotics. All experiments were done on a Turtlebot with the mic array attachment.

# Setup

In order to interface with the Mic array, you will need [HID](https://github.com/signal11/hidapi) and the associated [python wrapper](https://pypi.python.org/pypi/hidapi).

You will also need to create a new .rules file inside of '/etc/udev/rules.d/' continaing the following line

  SUBSYSTEMS=="usb", ENV{DEVTYPE}=="usb_device", ATTRS{idVendor}=="2886", ATTRS{idProduct}=="0007", MODE="0666"

Which changes the permissions for the device ID allowing you to read it.

In addition you will need to install ROS however the basic functionality of the gaussian mixture class can be run independently. This is not version specific.

For an example on interfacing the the Mic Array, you can find that [here](https://github.com/bwhitman/respeaker-xmos-hid). And if you would like to know the list of registers available on the Mic Array standard firmware, that can be found [here](https://gist.github.com/bwhitman/db16df744ee1065e5a7132e611dfdcb4)

# Use

gm_fusion.py takes bearing inputs and updates a gaussian mixture based on a generated observation model.
bearing_pub.py takes an audio localization message from the far field array and publishes it to the rostopic local_sound.
