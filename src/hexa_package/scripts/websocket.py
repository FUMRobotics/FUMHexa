#!/usr/bin/env python3

import asyncio
import websockets
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import UInt16
from hexa_package.msg import setting
from hexa_package.msg import sensor
from hexa_package.msg import controlParameters
from std_msgs.msg import String

from rospy_message_converter import message_converter

import json
import traceback

connected = set()
sensor_data = {
    'load_cell_right': [],
    'load_cell_left': []
}
motor_data = {
    'actual_position_left': [],
    'actual_position_right': []
}

connected = set()
ws = None


def publish(data, message_type):
    try:
        if message_type == 'setting':
            message = message_converter.convert_dictionary_to_ros_message('hexa_package/setting', data)
            setting_publisher.publish(message)
        if message_type == 'control_parameters':
            print(data)
            message = message_converter.convert_dictionary_to_ros_message('hexa_package/controlParameters', data)
            print(message)
            control_parameters_publisher.publish(message)
    except Exception:
        traceback.print_exc()


def get_sensor_data(message):
    data = message_converter.convert_ros_message_to_dictionary(message)
    sensor_data['load_cell_left'].append(data['loadcell0'])
    sensor_data['load_cell_right'].append(data['loadcell1'])
    if len(sensor_data['load_cell_left']) > 100:
        sensor_data['load_cell_left'].remove(0)
        sensor_data['load_cell_right'].remove(0)

def get_motor_data(message):
    data = message_converter.convert_ros_message_to_dictionary(message)
    motor_data['actual_position_left'].append(data['loadcell0'])
    motor_data['actual_position_right'].append(data['loadcell1'])
    if len(motor_data['actual_position_left']) > 100:
        motor_data['actual_position_left'].remove(0)
        motor_data['actual_position_right'].remove(0)


def parse_data(message):
    data = json.loads(message)
    if data['type'] == 'setting':
        data.pop('type', None)
        publish(data, 'setting')
    elif data['type'] == 'control_parameters':
        data.pop('type', None)
        print(data)
        publish(data, 'control_parameters')


rospy.init_node('gui', anonymous=True)


async def pinger(websocket, path):
    while True:
        try:
            try:
                pong_waiter = await websocket.ping()
                await asyncio.wait_for(pong_waiter, timeout=5)
            except asyncio.TimeoutError:
                connected.remove(websocket)
                print("disconnected")
            await asyncio.sleep(1)
        except websockets.exceptions.ConnectionClosed:
            connected.remove(websocket)
            print("disconnected")
            break


async def data_sender_interval(websocket):
    while True:
        if websocket in connected:
            data = {
                'type': 'plot',
                'load_cell_right': 0,
                'load_cell_left': 0,
                'actual_position_left': 0,
                'actual_position_right': 0,
            }
            if len(sensor_data['load_cell_right']) > 0:
                data['load_cell_right'] = sensor_data['load_cell_right'][-1]
            if len(sensor_data['load_cell_left']) > 0:
                data['load_cell_left'] = sensor_data['load_cell_left'][-1]
            if len(motor_data['actual_position_left']) > 0:
                data['actual_position_left'] = motor_data['actual_position_left'][-1]
            if len(motor_data['actual_position_right']) > 0:
                data['actual_position_right'] = motor_data['actual_position_right'][-1]
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


rospy.Subscriber('sensor', sensor, get_sensor_data)
# rospy.Subscriber('motor', sensor, get_sensor_data)

control_parameters_publisher = rospy.Publisher('controlParameters', controlParameters, queue_size=10)
setting_publisher = rospy.Publisher('setting', setting, queue_size=10)

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
    exit()
