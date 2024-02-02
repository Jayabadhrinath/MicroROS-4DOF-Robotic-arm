#To create a LaunchDescription Object that contains a list of all Applications/Nodes that needs to be launched via the launch file
from launch import LaunchDescription 

#To define the objects of the Node class that should be launched
from launch_ros.actions import Node

#To create a vaiable that contains the configuration parameter for the node
from launch_ros.parameter_descriptions import ParameterValue

#To declare an argument for a launch file, 
#To set an environment variable to properly load & visualize the Robot's URDF Model and 
#To include another launch file into the current launch file
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription

#To execute another python launch file from the current launch file
from launch.launch_description_sources import PythonLaunchDescriptionSource

# To To run a command from the launch file and To read the argument values given as inputs during the launch file invocation
from launch.substitutions import Command, LaunchConfiguration

#To indicate the directory path where the required file is located
import os
from ament_index_python.packages import get_package_share_directory

#Definition of the function that is executed each time the launch file is invoked
def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(get_package_share_directory("asrs_description"), "urdf", "asrs_robot.urdf.xacro"),
        description="Absolute path to the ASRS Arm URDF File"
    )

    env_variable = SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.path.join(get_package_share_directory("asrs_description"), "share"))

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))

    robot_state_publisher = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, 'use_sim_time': True}] 
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py'],)
        )

    # start_gazebo_server = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py"))
    # )

    # start_gazebo_client = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py"))
    # )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=["-entity", "asrs_robot", "-topic", "robot_description"]
    )

    return LaunchDescription([
        env_variable,
        model_arg,
        robot_state_publisher,
        gazebo,
        # start_gazebo_server,
        # start_gazebo_client,
        spawn_robot
    ])