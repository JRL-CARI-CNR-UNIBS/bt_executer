# This Python file uses the following encoding: utf-8
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  config_folder = PathJoinSubstitution([FindPackageShare("bt_executer"),"config"])

  return LaunchDescription([
    ExecuteProcess(
        cmd = [
          FindExecutable(name="cnr_param_server"),
          "--path-to-file",
          PathJoinSubstitution([
            config_folder,
            "bt_config.yaml"        # load the behavior tree plugins and the tree
          ]),
          "--path-to-file",
          PathJoinSubstitution([
            config_folder,
            "skills_config.yaml"    # load the parameters of each skill plugin
          ]),
          PathJoinSubstitution([
            config_folder,
            "poses.yaml"            # load the poses (TFs) used by the moveTo skill
          ])
        ],
        shell=False
      ),

    Node(
      package="bt_executer",
      executable="bt_executer_node",
      output="screen",
      namespace="bt_executer",
      ros_arguments=["--log-level", "info"]
      )
])
