#!/usr/bin/env python
# Copyright (c) 2025 TOYOTA MOTOR CORPORATION
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted (subject to the limitations in the disclaimer
# below) provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of the copyright holder nor the names of its contributors may be used
#   to endorse or promote products derived from this software without specific
#   prior written permission.
# NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
# LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
# OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.
# -*- coding: utf-8 -*-
import os
import sys

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

try:
    import robot_description
    from utils import (
        EXAMPLES,
        load_file,
        load_yaml,
    )
except ImportError:
    sys.path.append(os.path.dirname(__file__))
    import robot_description
    from utils import (
        EXAMPLES,
        load_file,
        load_yaml,
    )


def launch_setup(context, example_name, description_package, description_file):
    description_package_str = context.perform_substitution(description_package)
    description_file_str = context.perform_substitution(description_file)

    robot_description_moveit = robot_description.parse(description_package_str, description_file_str)
    robot_description_semantic = {'robot_description_semantic': load_file('config/hsrb.srdf')}
    kinematics_yaml = load_yaml('config/kinematics.yaml')
    robot_name = {'robot_name': LaunchConfiguration('robot_name')}

    example_name_str = context.perform_substitution(example_name)
    commander_node = Node(package='hsrb_moveit_config',
                          executable=example_name_str,
                          output='screen',
                          parameters=[{'robot_description': robot_description_moveit,
                                       'use_sim_time': LaunchConfiguration('use_sim_time')},
                                      robot_description_semantic,
                                      robot_name,
                                      kinematics_yaml])

    return [commander_node]


def declare_arguments():
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument('example_name', choices=EXAMPLES))

    declared_arguments.append(
        DeclareLaunchArgument('description_package',
                              default_value='hsrb_description',
                              description='Description package with robot URDF/xacro files.'))
    declared_arguments.append(
        DeclareLaunchArgument('description_file',
                              default_value='hsrb4s.urdf.xacro',
                              description='URDF/XACRO description file with the robot.'))
    declared_arguments.append(
        DeclareLaunchArgument('robot_name', default_value='hsrb', choices=['hsrb', 'hsrc'],
                              description='Robot name for kinematics plugin.'))
    declared_arguments.append(
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false'],
                              description='Launch with simulator.'))
    return declared_arguments


def generate_launch_description():
    return LaunchDescription(declare_arguments() + [
        OpaqueFunction(function=launch_setup,
                       args=[LaunchConfiguration('example_name'),
                             LaunchConfiguration('description_package'),
                             LaunchConfiguration('description_file')])])
