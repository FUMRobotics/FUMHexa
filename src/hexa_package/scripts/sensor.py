#!/usr/bin/env python3
import rospy
import can
from time import sleep
import binascii
from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import UInt16
from hexa_package.msg import sensor
import time
import os

if __name__ == '__main__':
    param = os.sched_param(os.sched_get_priority_max(os.SCHED_FIFO))
    os.sched_setscheduler(0, os.SCHED_FIFO, param)
    rospy.init_node('sensor', anonymous=True)
    pub1 = rospy.Publisher('sensor', sensor, queue_size=2)
    #lastMassageSentToRos = 0
    bus = can.Bus(interface='socketcan',channel='can0',receive_own_messages=False)
    can_filters = [ {"can_id" : 292, "can_mask" : 0xF, "extended" :False},
                    {"can_id" : 293, "can_mask" : 0xF, "extended" :False}]
    bus.set_filters(can_filters)
    while True:
        msg = bus.recv(None)
        #print(msg)
        #print(msg.arbitration_id)
        #pub1.publish(10, 20)
        if msg.arbitration_id == 292 and len(msg.data) == 8:
            # if msg.timestamp >= 0.00005 + lastMassageSentToRos:
            loadCell0 = msg.data[0] + msg.data[1] * 256
            loadCell1 = msg.data[2] + msg.data[3] * 256
            pub1.publish(loadCell0, loadCell1)


        if msg.arbitration_id == 293 and len(msg.data) == 4:
            # if msg.timestamp >= 0.00005 + lastMassageSentToRos:
            loadCell0 = msg.data[0] + msg.data[1] * 256
            loadCell1 = msg.data[2] + msg.data[3] * 256
            # lastMassageSentToRos = msg.timestamp
            #print(time.time(),loadCell0,loadCell1)
            pub1.publish(loadCell0, loadCell1)

        time.sleep(0.0001)
