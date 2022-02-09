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
import sqlite3


import json
import traceback
import subprocess
import shlex
import uuid
import csv
import time
from datetime import datetime as dt
import os
import shutil
from glob import glob

connected = set()
database = sqlite3.connect('/home/pi/hexa.db')
db = database.cursor()

load_cell_right = 0
load_cell_left = 0

actual_position_left = 0
actual_position_right = 0

statusword = 0

ws = None
report_buffer = None
report = None
start_report_time = None
current_report_id = None
async def create_report_file(name,patient_id):
    global report_buffer
    global report
    global start_report_time
    global current_report_id
    report = open('/home/pi/robot_data/' + name, 'w')
    report_buffer = csv.writer(report, quoting=csv.QUOTE_ALL)
    report_buffer.writerow(["Time", "Actual Position Right", "Actual Position Left", "LoadCell Right", "LoadCell Left"])
    start_report_time = int(time.time() * 1000)
    data = {
        'type': 'report_timer',
        "report_timer":start_report_time,
    }
    db.execute("insert into report (patient_id,file_name,created_at,duration) values (?,?,?,?)", (int(patient_id),name,time.strftime('%Y-%m-%d %H:%M:%S'), 0))
    database.commit()
    current_report_id = db.lastrowid
    print("New Report session started")
    await ws.send(json.dumps(data))


def close_report_file():
    global report
    global report_buffer
    global start_report_time
    global current_report_id
    if report is not None:
        report_buffer = None
        report.close()
        db.execute("update report set duration = ? where id = ?", ( int(time.time() * 1000)- start_report_time, current_report_id))
        database.commit()
        report = None
        start_report_time = None
        print("report ended")
    else:
        print("report file is none")


def publish(data, message_type):
    try:
        if message_type == 'setting':
            message = message_converter.convert_dictionary_to_ros_message('hexa_package/setting', data)
            print("Setting Published")
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
    global statusword
    actual_position_left = data['actualPosition0']
    actual_position_right = data['actualPosition1']
    load_cell_left = data['loadcell0']
    load_cell_right = data['loadcell1']
    statusword = data['statusword']
    if report_buffer is not None:
        report_buffer.writerow([int(time.time() * 1000), actual_position_right, actual_position_left, load_cell_right, load_cell_left])


def parse_data(message):
    data = json.loads(message)
    if data['type'] == 'setting':
        data.pop('type', None)
        publish(data, 'setting')
    elif data['type'] == 'control_parameters':
        data.pop('type', None)
        print(data)
        publish(data, 'control_parameters')
    elif data['type'] == 'date':
        linux_set_time(data['date'])
    elif data['type'] == 'new_report':
        print(data)
        asyncio.ensure_future(create_report_file(data['name'],data['patient_id']))
    elif data['type'] == 'end_reporting':
        close_report_file()
    elif data['type'] == 'get_report_list':
        asyncio.ensure_future(send_report_list(data['patient_id']))
    elif data['type'] == 'clear_logs':
        asyncio.ensure_future(clear_logs(data['patient_id']))
    elif data['type'] == 'disconnect':
        close_report_file()
        zero_imedance()
        asyncio.ensure_future(ready_to_disconnect())
        


async def ready_to_disconnect():
    data = {
        'type': 'disconnect',
        "status":'ready',
    }
    await ws.send(json.dumps(data))    

def get_size(start_path = '/home/pi/robot_data'):
    total_size = 0
    for dirpath, dirnames, filenames in os.walk(start_path):
        for f in filenames:
            fp = os.path.join(dirpath, f)
            # skip if it is symbolic link
            if not os.path.islink(fp):
                total_size += os.path.getsize(fp)

    return total_size #bytes



async def clear_logs(id):
    files = glob('/home/pi/robot_data/*')
    for f in files:
        file_name = f.split('/')
        file_name = file_name[-1]
        if file_name.startswith(str(id) + '_'):
            os.remove(f)
    data = {
        'type': 'clear_logs',
        "status":'completed',
    }
    db.execute("DELETE FROM report")
    database.commit()
    await ws.send(json.dumps(data))

async def send_report_list(id):
    db.execute("Select * from report where patient_id = :patient_id", {"patient_id":1})
    reports = db.fetchall()
    patient_log_list = []
    print(reports)
    for report in reports:
        each = {
                "patient_id":report[1],
                "file_name":report[2],
                "created_at":report[3],
                "duration":report[4],
        }
        patient_log_list.append(each)
    
    total, used, free = shutil.disk_usage("/")
    
    log_size = get_size()
    data = {
        'type': 'patient_report_list',
        "log_size":str(int(log_size / 1024 / 1024)),
        "total_size":str(int(free / 1024 / 1024)),
        'patient_log_list': patient_log_list
    }
    await ws.send(json.dumps(data))



def linux_set_time(date):
    subprocess.call(shlex.split("date +%s -s @" + date))


rospy.init_node('gui', anonymous=True)

def zero_imedance():
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
        'delta_theta_start':0,
        'delta_theta_end':0,
        'epsilon':0
    }
    publish(zero_impedance_command, 'setting')

async def pinger(websocket, path):
    while True:
        try:
            try:
                pong_waiter = await websocket.ping()
                await asyncio.wait_for(pong_waiter, timeout=5)
            except asyncio.TimeoutError:
                connected.remove(websocket)
                # Set drive mode to zero impedance on reset
                #close_report_file()
                #publish(zero_impedance_command, 'setting') 
                print("Timeout Error")
            await asyncio.sleep(1)
        except websockets.exceptions.ConnectionClosed:
            connected.remove(websocket)
            # Set drive mode to zero impedance on reset
            #close_report_file()
            #publish(zero_impedance_command, 'setting')
            print("ConnectionClosed")
            break


async def data_sender_interval(websocket):
    global statusword
    global start_report_time
    while True:
        if websocket in connected:
            data = {
                'type': 'plot',
                'load_cell_right': load_cell_right,
                'load_cell_left': load_cell_left,
                'actual_position_left': actual_position_left,
                'actual_position_right': actual_position_right,
                'statusword': statusword,
                'time': int(time.time() * 1000),
            }
            
            statusword = 0
            await websocket.send(json.dumps(data))
        else:
            break
        await asyncio.sleep(0.05)


async def socket(websocket, path):
    print("New Connection")
    global ws
    connected.clear()
    connected.add(websocket)
    ws = websocket
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
