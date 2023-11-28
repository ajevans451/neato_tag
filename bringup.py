
from typing import *
import sys
from multiprocessing import Process
import subprocess


FIRST_SENSOR_PORT = 7777
FIRST_CAMERA_PORT = 8888


class RobotInfo:
    def __init__(self, robot_idx: int, addr: str, proc: Process):
        self.robot_idx = robot_idx
        self.addr = addr
        self.proc = proc
        self.sensor_port = FIRST_SENSOR_PORT + robot_idx
        self.camera_pot = FIRST_CAMERA_PORT + robot_idx


def process_command(command: str, robot_info: List[RobotInfo]):
    # command is of the form kill [number]
    if command == 'exit':
        for num_to_kill, info in enumerate(robot_info):
            if not info.proc.is_alive:
                continue
            print(f'Killing robot {num_to_kill}')
            robot_info[num_to_kill].proc.terminate()
        return True
    
    num_to_kill = int(command.split(' ')[-1])
    if robot_info[num_to_kill].proc.is_alive:
        print(f'Killing robot {num_to_kill}')
        robot_info[num_to_kill].proc.terminate()
    return not any(r.proc.is_alive for r in robot_info)


def get_actual_addr(base_addr):
    if '192.168' in base_addr:
        return base_addr
    if '.' in base_addr:
        return f'192.168.{base_addr}'
    return f'192.168.16.{base_addr}'


def bringup_neato(addr, idx):
    subprocess.run(f'ros2 launch neato_node2 bringup_multi.py host:={addr} udp_sensor_port:={FIRST_SENSOR_PORT + idx} udp_video_port:={FIRST_CAMERA_PORT + idx} robot_name:=robot{idx}'.split(' '),
                    stdout=subprocess.DEVNULL)


def main():
    ip_addrs = sys.argv[1:]
    robot_info = []
    for idx, base_addr in enumerate(ip_addrs):
        addr = get_actual_addr(base_addr)
        proc = Process(target=bringup_neato, args=(addr, idx))
        proc.start()
        robot_info.append(RobotInfo(idx, addr, proc))
    
    while True:
        command = input('Enter the command you want to run ').strip().lower()
        if process_command(command, robot_info):
            print('All robots disconnected')
            break


if __name__ == '__main__':
    main()
