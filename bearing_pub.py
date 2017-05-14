#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
import numpy as np
import hid # https://pypi.python.org/pypi/hidapi aka https://github.com/trezor/cython-hidapi

class Respeaker(object):
    """docstring for Respeaker."""
    def __init__(self):
        RESPEAKER_VENDOR_ID = 0x2886
        RESPEAKER_PRODUCT_ID = 0x07

        # Set up the HID driver
        self.dev = hid.device()
        # self.dev.open(RESPEAKER_VENDOR_ID, RESPEAKER_PRODUCT_ID)

    # Write data to a register, return how many bytes were written
    def write_register(self, register, data):
        send_data = [0, register, 0, len(data), 0 ] + data
        return self.dev.write(send_data)

    # Read length data from a register, return the data
    def read_register(self, register, length):
        # To read a register you send reg & 0x80, and then read it back
        # If you have blocking off the read will return none if it's too soon after
        send_data = [0, register, 0x80, length, 0, 0, 0]
        what = self.dev.write(send_data)
        ret = self.dev.read(len(send_data) + length)
        return ret[4:4+length] # Data comes in at the 4th byte

    def read_auto_report(self):
        # Temporarily turn off blocking, the auto report only comes in on VAD changes
        self.dev.set_nonblocking(1)
        ret = self.dev.read(9)
        # print ret
        self.dev.set_nonblocking(0)
        if(len(ret)):
            # Make sure there is angle information
            if(len(ret)>4):
                # Angle is two bytes
                angle = ret[6]*256 + ret[5]
                self.write_register(0, [7, 0, 0, 7]) # mode, b, g, r  -- mode 0 is "all off"
                # VAD is 2 for speaking, 0 for not, 1 for ???
                vad = ret[4]
                if (vad==0):
                    return None
                else:
                    return angle
        return None

def angle(array=False):
    pub = rospy.Publisher('local_sound', Float64, queue_size=10)
    rospy.init_node('bearing', anonymous=True)
    rate = rospy.Rate(.5) # 10hz
    if (array==False):
        while not rospy.is_shutdown():
            sigma = 25
            theta = 0
            theta_prime = sigma * np.random.randn() + theta
            bearing_out = float(theta_prime)
            rospy.loginfo(bearing_out)
            pub.publish(bearing_out)
            rate.sleep()
    elif (array==True):
        while not rospy.is_shutdown():
            theta=mic_array.read_auto_report()
            while (theta==None):
                theta=mic_array.read_auto_report()
            bearing_out = float(theta)
            rospy.loginfo(bearing_out)
            pub.publish(bearing_out)
            rate.sleep()

if __name__ == '__main__':
    mic_array=Respeaker()
    try:
        angle(array=False)
    except rospy.ROSInterruptException:
        pass
