#!/usr/bin/env python3

import asyncio
import websockets
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import UInt16
from hexa_package.msg import setting
from hexa_package.msg import sensor
from hexa_package.msg import drivesFeedBack
from hexa_package.msg import controlParameters
from std_msgs.msg import String

from rospy_message_converter import message_converter

import json
import traceback
import subprocess
import shlex
import uuid
import csv
import time

connected = set()

load_cell_right = 0
load_cell_left = 0

actual_position_left = 0
actual_position_right = 0

ws = None
report_buffer = None
report = None


def create_report_file(name):
    global report_buffer
    global report
    report = open('/home/pi/robot_data/' + name, 'w')
    report_buffer = csv.writer(report, quoting=csv.QUOTE_ALL)
    report_buffer.writerow(["Actual Position Right", "Actual Position Left", "LoadCell Right", "LoadCell Left"])


def close_report_file():
    global report
    global report_buffer
    if report is not None:
        report.close()
        report = None
        report_buffer = None
    else:
        print("report file is none")


def publish(data, message_type):
    try:
        if message_type == 'setting':
            message = message_converter.convert_dictionary_to_ros_message('hexa_package/setting', data)
            print("Setting Published")
            print(data)
            setting_publisher.publish(message)
        if message_type == 'control_parameters':
            message = message_converter.convert_dictionary_to_ros_message('hexa_package/controlParameters', data)
            control_parameters_publisher.publish(message)
    except Exception:
        traceback.print_exc()


def get_motor_data(message):
    data = message_converter.convert_ros_message_to_dictionary(message)
    global actual_position_left
    global actual_position_right
    global load_cell_left
    global load_cell_right

    actual_position_left = data['actualPosition0']
    actual_position_right = data['actualPosition1']
    load_cell_left = data['loadcell0']
    load_cell_right = data['loadcell1']
    if report_buffer is not None:
        report_buffer.writerow([actual_position_right, actual_position_left, load_cell_right, load_cell_left])


def parse_data(message):
    data = json.loads(message)
    if data['type'] == 'setting':
        data.pop('type', None)
        publish(data, 'setting')
    elif data['type'] == 'control_parameters':
        data.pop('type', None)
        publish(data, 'control_parameters')
    elif data['type'] == 'date':
        linux_set_time(data['date'])
    elif data['type'] == 'new_report':
        create_report_file(data['name'])
    elif data['type'] == 'end_report':
        close_report_file()


def linux_set_time(date):
    subprocess.call(shlex.split("date +%s -s @" + date))


rospy.init_node('gui', anonymous=True)


async def pinger(websocket, path):
    zero_impedance_command = {
        'assist_algorithm': 'assist_as_needed',
        'left_leg_algorithm': 'zero_impedance',
        'right_leg_algorithm': 'zero_impedance',
        'left_assistive_force': 0,
        'right_assistive_force': 0,
        'left_assistive_time': 0,
        'right_assistive_time': 0,
        'assist_delay_left': 0,
        'assist_delay_right': 0,
    }
    while True:
        try:
            try:
                pong_waiter = await websocket.ping()
                await asyncio.wait_for(pong_waiter, timeout=5)
            except asyncio.TimeoutError:
                connected.remove(websocket)
                # Set drive mode to zero impedance on reset
                close_report_file()
                publish(zero_impedance_command, 'setting')
                print("disconnected")
            await asyncio.sleep(1)
        except websockets.exceptions.ConnectionClosed:
            connected.remove(websocket)
            # Set drive mode to zero impedance on reset
            close_report_file()
            publish(zero_impedance_command, 'setting')
            print("disconnected")
            break


async def data_sender_interval(websocket):
    while True:
        if websocket in connected:
            data = {
                'type': 'plot',
                'load_cell_right': load_cell_right,
                'load_cell_left': load_cell_left,
                'actual_position_left': actual_position_left,
                'actual_position_right': actual_position_right,
                'time': int(time.time() * 1000)
            }

            await websocket.send(json.dumps(data))
        else:
            break
        await asyncio.sleep(0.05)


async def socket(websocket, path):
    connected.add(websocket)
    asyncio.ensure_future(pinger(websocket, path))
    asyncio.ensure_future(data_sender_interval(websocket))
    async for message in websocket:
        if message != "pong":
            try:
                parse_data(message)
            except Exception:
                traceback.print_exc()


control_parameters_publisher = rospy.Publisher('controlParameters', controlParameters, queue_size=10)
setting_publisher = rospy.Publisher('setting', setting, queue_size=10)
rospy.Subscriber("drivesFeedBack", drivesFeedBack, get_motor_data)

loop = asyncio.get_event_loop()
try:
    loop.run_until_complete(websockets.serve(socket, port=3000))
    loop.run_forever()
except KeyboardInterrupt:
    print("Ctrl+C")
    rospy.signal_shutdown("Ctrl+C Manual Shutdown")
    print("Received exit, exiting")
    pending_tasks = [
        task for task in asyncio.Task.all_tasks() if not task.done()
    ]
    loop.run_until_complete(asyncio.gather(*pending_tasks))
    loop.close()
    report.close()
    exit()
