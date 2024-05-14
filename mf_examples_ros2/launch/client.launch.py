import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    
    return LaunchDescription([
        Node(
            package='machine_server_ros2',
            namespace='',
            executable='machine_server_ros2',
            name='machine_service_rasp',
            output="screen",
            emulate_tty=True,
            respawn=True,
            parameters=[
                {
                    'PLC_IP_address': '192.168.1.1',
                    'PLC_Port_address': 8501,
                    'timeout': 10.0,
                    'frequency': 2.0,
                    'dropoff_station_name': 'tramtha',
                    'pickup_station_name': 'tramcap'
                }
            ]
        ),

        Node(
            package='machine_fleet_client_ros2',
            namespace='',
            executable='machine_fleet_client_ros2',
            name='fleet_machine_client_node',
            output="screen",
            emulate_tty=True,
            respawn=True,
            parameters=[
                {
                    'fleet_name': 'amr_vdm',
                    'machine_name': 'nqvlm104',
                    'machine_state_topic': '/machine_state_rasp',
                    'delivery_request_topic': '/delivery_request_rasp',
                    'station_request_topic': '/station_request_rasp',
                    'machine_trigger_server_name': '/machine_server_rasp',
                    'dds_domain': 62,
                    'dds_state_topic': 'machine_state',
                    'dds_delivery_request_topic': 'delivery_request',
                    'dds_machine_request_topic': 'machine_request',
                    'dds_station_request_topic': 'station_request',
                    'update_frequency': 5.0,
                    'publish_frequency': 1.0,
                }
            ]
        ),
    ])
