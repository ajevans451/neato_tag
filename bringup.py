
from typing import *
import sys
from neato_tag.game import NEATO_TAG


FIRST_SENSOR_PORT = 7777
FIRST_CAMERA_PORT = 8888


def get_actual_addr(base_addr):
    if '192.168' in base_addr:
        return base_addr
    if '.' in base_addr:
        return f'192.168.{base_addr}'
    return f'192.168.16.{base_addr}'



def main():
    for idx, addr in enumerate(NEATO_TAG.hosts):
        print(f'ros2 launch neato_node2 bringup_multi.py host:={addr} udp_sensor_port:={FIRST_SENSOR_PORT + idx} udp_video_port:={FIRST_CAMERA_PORT + idx} robot_name:=robot{idx}')
        print(f'ros2 run neato_tag camera_detector --ros-args -r camera/image_raw:=robot{idx}/camera/image_raw neatos_in_camera:=robot{idx}/neatos_in_camera')
        print(f'ros2 run neato_tag tagging --ros-args -r camera/image_raw:=robot{idx}/camera/image_raw bump:=robot{idx}/bump')
    print()
    for idx, addr in enumerate(NEATO_TAG.hosts):
        print(f'ros2 run neato_tag navigator --ros-args -r scan:=robot{idx}/scan neatos_in_camera:=robot{idx}/neatos_in_camera')


if __name__ == '__main__':
    main()
