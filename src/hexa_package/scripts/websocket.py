#!/usr/bin/env python3

import asyncio
import websockets
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import UInt16
from hexa_package.msg import setting
from hexa_package.msg import sensor
from std_msgs.msg import String

from rospy_message_converter import message_converter
from rospy_message_converter import json_message_converter
import json
connected = set()
sensor_data = {
    'load_cell_right' : [],
    'load_cell_left': []
}
connected = set()
ws = None
pong = True
def publish(data):
    message = message_converter.convert_dictionary_to_ros_message('hexa_package/setting', data)
    print(message)
    pub.publish(message)


async def send(data):
    await ws.send(data)




def get_sensor_data(message):
    data = message_converter.convert_ros_message_to_dictionary(message)
    sensor_data['load_cell_left'].append(data['loadcell0'])
    sensor_data['load_cell_right'].append(data['loadcell1'])
    if len(sensor_data['load_cell_left']) > 8 and ws is not None:
        asyncio.run(send(sensor_data))
        sensor_data['load_cell_left'] = []
        sensor_data['load_cell_right'] = []




def parse_data(message):
    data = json.loads(message)
    publish(data)



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
            else:
                print("ping")
            await asyncio.sleep(1)
        except websockets.exceptions.ConnectionClosed:
            connected.remove(websocket)
            print("disconnected")
            break



async def socket(websocket, path):
    global pong
    connected.add(websocket)
    pong = True
    pinger_task = asyncio.ensure_future(
        pinger(websocket, path))
    async for message in websocket:
        print(message)
        if message != "pong":
            print(message)
            parse_data(message)
        else:
            pong = True






rospy.Subscriber('sensor', sensor, get_sensor_data)
pub = rospy.Publisher('setting', setting, queue_size=10)

loop = asyncio.get_event_loop()
try:
    loop.run_until_complete(
        websockets.serve(socket, port=3000))
    loop.run_forever()
except KeyboardInterrupt:
    print("Received exit, exiting")
    pending_tasks = [
        task for task in asyncio.Task.all_tasks() if not task.done()
    ]
    loop.run_until_complete(asyncio.gather(*pending_tasks))
    loop.close()
    exit()#stops

