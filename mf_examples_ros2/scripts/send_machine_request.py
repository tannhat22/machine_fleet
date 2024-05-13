#!/usr/bin/env python3

# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import argparse

import rclpy
from rclpy.node import Node

from machine_fleet_msgs.msg import MachineMode
from machine_fleet_msgs.msg import MachineRequest


def main(argv = sys.argv):
    '''
    Example charge request:
    - fleet_name: magni
    - machine_name: magni123
    - request_id: 6tyghb4edujrefyd
    - mode.mode: RELEASE
    '''

    default_fleet_name = 'amr_vdm'
    default_machine_name = 'nqvlm104'
    default_request_id = '576y13ewgyffeijuais'
    default_mode = 'release'
    default_topic_name = '/machine_request'

    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--fleet-name', default=default_fleet_name)
    parser.add_argument('-c', '--machine-name', default=default_machine_name)
    parser.add_argument('-m', '--mode', default=default_mode)
    parser.add_argument('-i', '--request-id', default=default_request_id)
    parser.add_argument('-t', '--topic-name', default=default_topic_name)
    args = parser.parse_args(argv[1:])

    print('fleet_name: {}'.format(args.fleet_name))
    print('machine_name: {}'.format(args.machine_name))
    print('mode: {}'.format(args.mode))
    print('request_id: {}'.format(args.request_id))
    print('topic_name: {}'.format(args.topic_name))

    rclpy.init()
    node = rclpy.create_node('send_machine_request_node')
    pub = node.create_publisher(MachineRequest, args.topic_name, 10)

    msg = MachineRequest()
    msg.fleet_name = args.fleet_name
    msg.machine_name = args.machine_name
    msg.request_id = args.request_id
    
    if args.mode == 'mode':
        print('Please insert desired mode: release or clamp')
        return
    elif args.mode == 'release':
        msg.mode.mode = MachineMode.MODE_RELEASE 
    elif args.mode == 'clamp':
        msg.mode.mode = MachineMode.MODE_CLAMP
    else:
        print('unrecognized mode requested, only use release or clamp please')
        return
  
    rclpy.spin_once(node, timeout_sec=1.0)
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.5)
    print('all done!')
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
