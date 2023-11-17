
from typing import *
import sys
import os
import signal
import subprocess


FIRST_SENSOR_PORT = 7777
FIRST_CAMERA_PORT = 8888


class RobotInfo:
    def __init__(self, robot_idx: int, addr: str, pid: int):
        self.robot_idx = robot_idx
        self.addr = addr
        self.pid = pid
        self.running = True
        self.sensor_port = FIRST_SENSOR_PORT + robot_idx
        self.camera_pot = FIRST_CAMERA_PORT + robot_idx


def process_command(command: str, robot_info: List[RobotInfo]):
    # command is of the form kill [number]
    if command == 'exit':
        for num_to_kill, info in enumerate(robot_info):
            if not info.running:
                continue
            print(f'Killing robot {num_to_kill} with PID {robot_info[num_to_kill].pid}')
            os.kill(info.pid, signal.SIGTERM)
            print('Done')
            info.running = False
        return True
    num_to_kill = int(command.split(' ')[-1])
    if robot_info[num_to_kill].running:
        print(f'Killing robot {num_to_kill} with PID {robot_info[num_to_kill].pid}')
        os.kill(robot_info[num_to_kill].pid, signal.SIGTERM)
        print('Done')
        robot_info[num_to_kill].running = False
    return not any(r.running for r in robot_info)


def get_actual_addr(base_addr):
    if '192.168' in base_addr:
        return base_addr
    if '.' in base_addr:
        return f'192.168.{base_addr}'
    return f'192.168.16.{base_addr}'


def main():
    ip_addrs = sys.argv[1:]
    robot_info = []
    for idx, base_addr in enumerate(ip_addrs):
        addr = get_actual_addr(base_addr)
        child_id = os.fork()
        if child_id == 0:
            subprocess.run(f'ros2 launch neato_node2 bringup_multi.py host:={addr} udp_sensor_port:={FIRST_SENSOR_PORT + idx} udp_video_port:={FIRST_CAMERA_PORT + idx} robot_name:=robot{idx}'.split(' '),
                           stdout=subprocess.DEVNULL)
        else:
            print(f'Spawning PID {child_id} for robot {idx} ({addr})')
            robot_info.append(RobotInfo(idx, addr, child_id))
    
    while True:
        command = input('Enter the command you want to run ').strip().lower()
        if process_command(command, robot_info):
            print('All robots disconnected')
            break


if __name__ == '__main__':
    main()
