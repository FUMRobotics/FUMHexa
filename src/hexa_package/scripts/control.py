#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Int16
from std_msgs.msg import UInt16
from hexa_package.msg import setting
from rospy_message_converter import message_converter
from hexa_package.msg import drivesFeedBack
from hexa_package.msg import drivesAction


from math import sin as sin
import time
#import math
# def sin(a):
#     return math.sin(a)


a_filter = 0.1
pi = 3.14
D_time = D_Time = 0.005
Current_Limit = 3000

timer = 0
Assist_R = 0
Assist_L = 0

# Feedback
LoadcellRightHip = 0
LoadcellLeftHip = 0
PositionActualValueright = 0
PositionActualValueleft = 0
VelocityActualValueright = 0
VelocityActualValueleft = 0

CurrentActualValueright = 0
CurrentActualValueleft = 0


Zero_Impedance_Gain = 1

I_R = 0.3
C_R = 1.5
FK_R = 1.5
MgL_R =0.5
PID_Gain_R = 0.1
N_RH = 100

I_L = 0.3
C_L = 1.5
FK_L = 1.5
MgL_L = 0.5
PID_Gain_L = 0.1
N_LH = 100

# initial value


Flt_Vel_RH_old = 0
Flt_Vel_LH_old = 0
L_Assist = 755
R_Assist = 750
R_o = 0
L_o = 0
Flt_Acc_RH_old = 0
Flt_Acc_LH_old = 0
ef_RH_old = 0
ef_LH_old = 0
PD_RH_old = 0
PD_LH_old = 0

P_RH = 30
P_LH = 30
D_RH = 1
D_LH = 1
timer_r = 0
timer_l = 0




rospy.init_node('control', anonymous=True)
rate = rospy.Rate(200)  # 200hz

# Publish
# desiredCurrentRight = rospy.Publisher('desiredCurrent0', Int16, queue_size=1000)
# desiredCurrentLeft = rospy.Publisher('desiredCurrent1', Int16, queue_size=1000)

drives_action = rospy.Publisher('drivesAction', drivesAction, queue_size=1)


# Subscribe


def get_load_cell1(message):
    global LoadcellRightHip
    LoadcellRightHip = message.data
#    print("load cell right: " + str(LoadcellRightHip))

def get_load_cell2(message):
    global LoadcellLeftHip
    LoadcellLeftHip = message.data
#    print("load cell left: " + str(LoadcellLeftHip))



def get_setting(message):
    data = message_converter.convert_ros_message_to_dictionary(message)
    print(data)
    global P_RH
    P_RH = data['right_assistive_force']
    global P_LH
    P_LH = data['left_assistive_force']


def get_drive_feedback(message):
    global PositionActualValueleft
    global PositionActualValueright
    global CurrentActualValueleft
    global CurrentActualValueright

    PositionActualValueleft = message.actualPosition0
    PositionActualValueright = message.actualPosition1

    CurrentActualValueleft = message.actualCurrent0
    CurrentActualValueright = message.actualCurrent1
    print("time:" + str(time.time()) )


rospy.Subscriber('loadcell1', UInt16, get_load_cell1)
rospy.Subscriber('loadcell2', UInt16, get_load_cell2)
# rospy.Subscriber('actualPosition0', Int32, get_position_actual_right)
# rospy.Subscriber('actualPosition1', Int32, get_position_actual_left)

# rospy.Subscriber('actualCurrent0', Int16, get_current_actual_right)
# rospy.Subscriber('actualCurrent1', Int16, get_current_actual_left)

rospy.Subscriber('drivesFeedBack', drivesFeedBack, get_drive_feedback)
#rospy.Subscriber('setting', setting, send_setting)
loopCounter = 0
while not rospy.is_shutdown():
    Force_RH = 0.07*(- 0.023306 * (LoadcellRightHip) + 458.15 - 6.0)
    Force_LH = 0.07*(- 0.023359 * (LoadcellLeftHip) + 756.79 - 6.0)

    # Theta
    Theta_RH = -(2 * pi / (4800)) * PositionActualValueright
    Theta_LH = +(2 * pi / (4800)) * PositionActualValueleft

    # Position Filter
    Flt_Vel_RH = (1 - D_Time / a_filter) * Flt_Vel_RH_old + Theta_RH * D_Time / a_filter
    Flt_Vel_LH = (1 - D_Time / a_filter) * Flt_Vel_LH_old + Theta_LH * D_Time / a_filter

    # Velocity
    Theta_dt_RH = -2 * pi * VelocityActualValueright / (60 * 100)
    Theta_dt_LH = +2 * pi * VelocityActualValueleft / (60 * 100)

    Velocity_RH = (Flt_Vel_RH - Flt_Vel_RH_old) / D_Time
    Velocity_LH = (Flt_Vel_LH - Flt_Vel_LH_old) / D_Time

    Flt_Vel_RH_old = Flt_Vel_RH
    Flt_Vel_LH_old = Flt_Vel_LH

    # M*g*sin(Theta)
    mgsin_RH = 0.5 * 5.8 * 9.81 * 0.341 * sin(Theta_RH)
    mgsin_LH = 0.5 * 5.8 * 9.81 * 0.341 * sin(Theta_LH)

    # Velocity Filter
    Flt_Acc_RH = (1 - D_Time / a_filter) * Flt_Acc_RH_old + Theta_dt_RH * D_Time / a_filter
    Flt_Acc_LH = (1 - D_Time / a_filter) * Flt_Acc_LH_old + Theta_dt_LH * D_Time / a_filter

    # Acceleration
    ACC_RH = (Flt_Acc_RH - Flt_Acc_RH_old) / D_Time
    ACC_LH = (Flt_Acc_LH - Flt_Acc_LH_old) / D_Time

    Flt_Acc_RH_old = Flt_Acc_RH
    Flt_Acc_LH_old = Flt_Acc_LH

    # I*Acceleration
    I_Theta2dot_RH = 1.61 * ACC_RH
    I_Theta2dot_LH = 1.61 * ACC_LH


    # Identification
    if Force_RH > 0:
        SIGN_Theta_dt_RH = -1
    elif Force_RH < 0:
        SIGN_Theta_dt_RH = +1

    if Force_LH > 0:
        SIGN_Theta_dt_LH = -1
    elif Force_LH < 0:
        SIGN_Theta_dt_LH = +1

    # PD Code
    ef_RH = 0 - Force_RH
    ef_LH = 0 - Force_LH

    ef_RH_dt = (ef_RH - ef_RH_old) / D_time
    ef_LH_dt = (ef_LH - ef_LH_old) / D_time

    PD_RH = (1 - N_RH * D_time) * PD_RH_old + N_RH * P_RH * D_time * (ef_RH) + (P_RH + N_RH * D_RH) * D_time * ef_RH_dt
    PD_LH = (1 - N_LH * D_time) * PD_LH_old + N_LH * P_LH * D_time * (ef_LH) + (P_LH + N_LH * D_LH) * D_time * ef_LH_dt

    ef_RH_old = ef_RH
    ef_LH_old = ef_LH

    PD_RH_old = PD_RH
    PD_LH_old = PD_LH

    # Zero Impedance
    Trq_RH = Zero_Impedance_Gain * (
            I_R * ACC_RH + C_R * Theta_dt_RH + FK_R * SIGN_Theta_dt_RH + MgL_R * sin(Theta_RH) + PID_Gain_R * PD_RH)
    Trq_LH = Zero_Impedance_Gain * (
            I_L * ACC_LH + C_L * Theta_dt_LH + FK_L * SIGN_Theta_dt_LH + MgL_L * sin(Theta_LH) + PID_Gain_L * PD_LH)

    # ASSIST
    if timer_r > timer:
        R_o = 0
    elif Velocity_RH > 0 and Theta_RH <= 1.2 and Velocity_LH <= 0:
        R_o = 1

    if timer_l > timer:
        L_o = 0
    elif Velocity_LH > 0 and Theta_LH <= 1.2 and Velocity_RH <= 0:
        L_o = 1

    if R_o == 1:
        timer_r = timer_r + 1
        R_Assist = Assist_R
    elif R_o == 0:
        timer_r = 0
        R_Assist = 0

    if L_o == 1:
        timer_l = timer_l + 1
        L_Assist = Assist_L
    elif L_o == 0:
        timer_l = 0
        L_Assist = 0

    Trq_RH = -(3000*Trq_RH/100/12.8 + 6 * R_Assist)*10
    Trq_LH = +(3000*Trq_LH/100/12.8 + 6 * L_Assist)*10

    # Torque Limit
    if Trq_RH > Current_Limit:
        Trq_RH = Current_Limit
    elif Trq_RH < -Current_Limit:
        Trq_RH = -Current_Limit

    if Trq_LH > Current_Limit:
        Trq_LH = Current_Limit
    elif Trq_LH < -Current_Limit:
        Trq_LH = -Current_Limit

    # Send TO Derive
    Currentright = (1 * Trq_RH)
    Currentleft = (1 * Trq_LH)
    #print("Currentright " + str(Currentright))
    #print("Currentleft " + str(Currentleft))

    Currentright = 200 * sin(loopCounter / 200.00)
    Currentleft = 200 * sin(loopCounter / 200.00)

    # desiredCurrentRight.publish(int(Currentright))
    # desiredCurrentLeft.publish(int(Currentleft))

    # drivesFeedBack df
    # df.actualCurrent0 = Currentleft
    # df.actualCurrent1 = Currentright
    drives_action.publish(drivesAction(int(Currentleft), int(Currentright)))

    #desiredCurrentRight.publish(int(500))
    #desiredCurrentLeft.publish(int(500))
    # if(loopCounter % 1 == 0):
    #     print("time:" + str(time.time()) + "\tForce_LH:" + str(Force_LH) + "\tForce_RH:" + str(Force_RH) +"\tCurrentleft:" + str(Currentleft) + "\tCurrentright:" + str(Currentright) + "\tCurrentActualValueright:" + str(CurrentActualValueright) + "\tCurrentActualValueleft" + str(CurrentActualValueleft) + "\tLoadcellLeftHip:" + str(LoadcellLeftHip) + "\tLoadcellRightHip:" + str(LoadcellRightHip) + "\tPositionActualValueleft:" + str(PositionActualValueleft)  + "\tPositionActualValueright:" + str(PositionActualValueright)) 
    loopCounter = loopCounter + 1
    #print("time:" + str(time.time()) )
    rate.sleep()

