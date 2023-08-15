#!/usr/bin/env python3

import os

from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
    GroupAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import PythonExpression, LaunchConfiguration


def get_names(pkg: str, tag: str) -> tuple:
    return f"{pkg}_{tag}", f"{pkg}:{tag}"


def shutdown_callback(name, event, context):
    os.system(f"docker kill {name}")
    return LaunchDescriptionEntity()


def generate_launch_description():
    package_name = "rqt_multiplot_bridge"
    cmd_base = ["rocker", "--network", "host"]

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            "multiplot_config",
            default_value="",
            description="multiplot config file to preload",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "bridge_config",
            default_value="",
            description=(
                "config file that specifies which topics to bridge. If left un"
                "specified, it bridges all topics (parameter_bridge vs dynamic_bridge)"
            ),
        )
    )

    multiplot_cfg = LaunchConfiguration("multiplot_config")
    multiplot_cfg_cdn = PythonExpression(["len('", multiplot_cfg, "') > 0"])
    multiplot_cfg_mnt = PythonExpression(["'", multiplot_cfg, ":/root/multiplot.xml'"])

    # Start multiplot
    cont_rmp_name, img_rmp_name = get_names(package_name, "multiplot")
    ld.add_action(
        ExecuteProcess(
            condition=UnlessCondition(multiplot_cfg_cdn),
            cmd=cmd_base + ["--x11", "--name", cont_rmp_name, img_rmp_name],
            name="multiplot",
            output="screen",
        )
    )
    ld.add_action(
        ExecuteProcess(
            condition=IfCondition(multiplot_cfg_cdn),
            cmd=cmd_base
            + [
                "--x11",
                "--name",
                cont_rmp_name,
                "--volume",
                multiplot_cfg_mnt,
                "--",
                img_rmp_name,
            ],
            name="multiplot",
            output="screen",
        )
    )

    bridge_cfg = LaunchConfiguration("bridge_config")
    bridge_cfg_cdn = PythonExpression(["len('", bridge_cfg, "') > 0"])
    bridge_cfg_mnt = PythonExpression(["'", bridge_cfg, ":/bridge.yaml'"])

    # Start dynamic bridge
    cont_dyn_name, img_dyn_name = get_names(package_name, "dynamic_bridge")
    cb_dyn = lambda event, context: shutdown_callback(cont_dyn_name, event, context)
    ld.add_action(
        GroupAction(
            condition=UnlessCondition(bridge_cfg_cdn),
            actions=[
                TimerAction(
                    period=2.0,
                    actions=[
                        ExecuteProcess(
                            cmd=cmd_base + ["--name", cont_dyn_name, img_dyn_name],
                            name="bridge",
                            output="screen",
                        )
                    ],
                ),
                RegisterEventHandler(event_handler=OnShutdown(on_shutdown=cb_dyn)),
            ],
        )
    )

    # Start parameter bridge
    cont_param_name, img_param_name = get_names(package_name, "parameter_bridge")
    cb_param = lambda event, context: shutdown_callback(cont_param_name, event, context)
    ld.add_action(
        GroupAction(
            condition=IfCondition(bridge_cfg_cdn),
            actions=[
                TimerAction(
                    period=2.0,
                    actions=[
                        ExecuteProcess(
                            cmd=cmd_base
                            + [
                                "--name",
                                cont_param_name,
                                "--volume",
                                bridge_cfg_mnt,
                                "--",
                                img_param_name,
                            ],
                            name="bridge",
                            output="screen",
                        )
                    ],
                ),
                RegisterEventHandler(event_handler=OnShutdown(on_shutdown=cb_param)),
            ],
        )
    )

    return ld
