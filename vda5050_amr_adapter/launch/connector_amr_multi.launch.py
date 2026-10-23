# BSD 3-Clause License
#
# Copyright (c) 2022 InOrbit, Inc.
# Copyright (c) 2022 Clearpath Robotics, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the InOrbit, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import shutil
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _launch_robots(context, package_dir):
    config_path = context.perform_substitution(
        LaunchConfiguration("robots_config_file")
    )
    launch_use_sim_time = context.perform_substitution(
        LaunchConfiguration("use_sim_time")
    )
    if launch_use_sim_time.lower() not in {"true", "false"}:
        raise ValueError("'use_sim_time' launch argument must be true or false")
    default_use_sim_time = launch_use_sim_time.lower() == "true"
    with open(config_path, "r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)

    if not isinstance(config, dict):
        raise ValueError("Multi-robot configuration must be a YAML mapping")

    mqtt_defaults = config.get("mqtt", {})
    robots = config.get("robots", [])
    if not isinstance(mqtt_defaults, dict):
        raise ValueError("The 'mqtt' configuration must be a mapping")
    if not isinstance(robots, list) or not robots:
        raise ValueError("The 'robots' configuration must be a non-empty list")

    used_namespaces = set()
    used_identities = set()
    temporary_directory = tempfile.mkdtemp(prefix="connector_amr_multi_")

    def cleanup_temporary_configs(_context):
        shutil.rmtree(temporary_directory, ignore_errors=True)
        return []

    launch_actions = [
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[OpaqueFunction(function=cleanup_temporary_configs)]
            )
        )
    ]

    connector_launch = os.path.join(package_dir, "launch", "connector_amr.launch.py")
    for index, robot in enumerate(robots):
        if not isinstance(robot, dict):
            raise ValueError(f"Robot entry {index} must be a mapping")

        robot_name = robot.get("robot_name")
        if not isinstance(robot_name, str) or not robot_name.strip():
            raise ValueError(f"Robot entry {index} needs a non-empty 'robot_name'")

        namespace = robot.get("namespace", robot_name)
        serial_number = robot.get("serial_number", robot_name)
        state_pub_period = robot.get("state_pub_period", 0.2)
        visualization_pub_period = robot.get("visualization_pub_period", 0.2)
        manufacturer_name = robot.get(
            "manufacturer_name", mqtt_defaults.get("manufacturer_name", "IFC")
        )
        if not all(isinstance(value, str) and value.strip() for value in (
                namespace, serial_number, manufacturer_name)):
            raise ValueError(
                f"Robot '{robot_name}' needs non-empty namespace, serial_number, "
                "and manufacturer_name values"
            )
        if namespace in used_namespaces:
            raise ValueError(f"Duplicate robot namespace: {namespace}")
        identity = (manufacturer_name, serial_number)
        if identity in used_identities:
            raise ValueError(f"Duplicate VDA5050 robot identity: {identity}")
        used_namespaces.add(namespace)
        used_identities.add(identity)

        mqtt_parameters = {
            "mqtt_address": mqtt_defaults.get("mqtt_address", "localhost"),
            "mqtt_port": mqtt_defaults.get("mqtt_port", 1883),
            "mqtt_username": mqtt_defaults.get("mqtt_username", ""),
            "mqtt_password": mqtt_defaults.get("mqtt_password", ""),
            "manufacturer_name": manufacturer_name,
            "serial_number": serial_number,
            "use_sim_time": default_use_sim_time,
        }
        robot_mqtt_parameters = robot.get("mqtt", {})
        if not isinstance(robot_mqtt_parameters, dict):
            raise ValueError(f"Robot '{robot_name}' mqtt settings must be a mapping")
        allowed_robot_mqtt_parameters = {
            "mqtt_address", "mqtt_port", "mqtt_username", "mqtt_password"
        }
        unsupported_parameters = set(robot_mqtt_parameters) - allowed_robot_mqtt_parameters
        if unsupported_parameters:
            raise ValueError(
                f"Robot '{robot_name}' mqtt settings cannot override: "
                f"{sorted(unsupported_parameters)}"
            )
        mqtt_parameters.update(robot_mqtt_parameters)

        node_parameters = {
            "robot_name": robot_name,
            "manufacturer_name": manufacturer_name,
            "serial_number": serial_number,
            "use_sim_time": default_use_sim_time,
            "visualization_pub_period": visualization_pub_period,
            "state_pub_period": state_pub_period,
        }
        controller_parameters = dict(node_parameters)
        robot_base_frame = robot.get(
            "robot_base_frame", f"{robot_name}_base_link"
        )
        map_frame = robot.get("map_frame", "map")
        if not all(isinstance(value, str) and value.strip() for value in (
                robot_base_frame, map_frame)):
            raise ValueError(
                f"Robot '{robot_name}' needs non-empty robot_base_frame and map_frame values"
            )
        factsheet = robot.get("factsheet", {})
        if not isinstance(factsheet, dict):
            raise ValueError(f"Robot '{robot_name}' factsheet must be a mapping")
        controller_parameters["factsheet"] = factsheet
        robot_use_sim_time = robot.get("use_sim_time", default_use_sim_time)
        if not isinstance(robot_use_sim_time, bool):
            raise ValueError(f"Robot '{robot_name}' use_sim_time must be true or false")
        mqtt_parameters["use_sim_time"] = robot_use_sim_time
        node_parameters["use_sim_time"] = robot_use_sim_time
        adapter_parameters = dict(node_parameters)
        adapter_parameters.update({
            "robot_base_frame": robot_base_frame,
            "map_frame": map_frame,
            "vda_map_id": robot.get("vda_map_id", map_frame),
            "use_sim_time": robot_use_sim_time,
        })
        robot_parameters = {
            "mqtt_bridge": {"ros__parameters": mqtt_parameters},
            "controller": {"ros__parameters": controller_parameters},
            "amr_adapter": {"ros__parameters": adapter_parameters},
        }
        robot_config_path = os.path.join(
            temporary_directory, f"robot_{index}.yaml"
        )
        with open(robot_config_path, "w", encoding="utf-8") as robot_config_file:
            yaml.safe_dump(robot_parameters, robot_config_file, sort_keys=False)

        launch_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(connector_launch),
                launch_arguments={
                    "namespace": namespace,
                    "parameters_config_file": robot_config_path,
                }.items(),
            )
        )

    return launch_actions


def generate_launch_description():
    package_dir = get_package_share_directory("vda5050_amr_adapter")
    default_config = os.path.join(package_dir, "config", "connector_amr_multi.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robots_config_file",
            default_value=default_config,
            description="YAML file containing shared MQTT settings and robot entries",
        ), DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time if true",
        ),
        OpaqueFunction(function=_launch_robots, args=[package_dir]),
    ])