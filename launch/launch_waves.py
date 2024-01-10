from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                   OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown

import os
import codecs
import subprocess
import time

MODEL_NAME = 'vereniki'


def monitor_sim():
    # wait a few secs before starting to pgrep for process
    time.sleep(10)
    quit = False
    # monitor gazebo process until it exits
    while not quit:
        time.sleep(1)
        process = subprocess.Popen(['pgrep', '-f', 'gz sim -v 4'],
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE)
        stdout = process.communicate()[0]
        str_output = codecs.getdecoder('unicode_escape')(stdout)[0]
        if len(str_output) == 0:
            quit = True


def gz_simulation(world_name, headless=False, paused=False, extra_gz_args=''):
    gz_args = ['-v 4']
    if not paused:
        gz_args.append('-r')

    if headless:
        gz_args.append('-s')

    gz_args.append(extra_gz_args)
    gz_args.append(f'{world_name}.sdf')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch'),
            '/gz_sim.launch.py']),
        launch_arguments={'gz_args': ' '.join(gz_args)}.items())

    return gz_sim


def gz_shutdown_handle():
    sim_exit_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=monitor_sim(),
            on_exit=[
                EmitEvent(event=Shutdown(reason='Simulation ended'))
            ]
        )
    )
    return sim_exit_event_handler


def bridges():
    waves_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/wave/force@geometry_msgs/msg/Vector3[gz.msgs.Vector3d',
                   '/wave/torque@geometry_msgs/msg/Vector3[gz.msgs.Vector3d',
                   f'/world/waves/model/vereniki/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
                   f'/model/vereniki/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                   f'/model/vereniki/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                   f'/model/vereniki/joint/engine_jointA/cmd_steer@std_msgs/msg/Float64]gz.msgs.Double',
                   f'/model/vereniki/joint/engine_jointB/cmd_steer@std_msgs/msg/Float64]gz.msgs.Double',
                   f'/model/vereniki/joint/engine_jointC/cmd_steer@std_msgs/msg/Float64]gz.msgs.Double',
                   f'/model/vereniki/joint/propeller_jointA/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
                   f'/model/vereniki/joint/propeller_jointB/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
                   f'/model/vereniki/joint/propeller_jointC/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
                   f'/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   f'/added_mass/force@std_msgs/msg/Float64[gz.msgs.Double',
                   f'/added_mass/torque@std_msgs/msg/Float64[gz.msgs.Double',
                   f'/waterCurrent/direction@std_msgs/msg/Float64[gz.msgs.Double',
                   f'/waterCurrent/speed@std_msgs/msg/Float64[gz.msgs.Double',
                   f'/wind/direction@std_msgs/msg/Float64[gz.msgs.Double',
                   f'/wind/speed@std_msgs/msg/Float64[gz.msgs.Double'],
        output='screen')
    return waves_bridge


def generate_launch_description():
    return LaunchDescription([
        gz_simulation("../worlds/waves", paused=True),
        bridges(),
        gz_shutdown_handle()
    ])