#To create a LaunchDescription Object that contains a list of all Applications/Nodes that needs to be launched via the launch file
from launch import LaunchDescription 

#To define the objects of the Node class that should be launched
from launch_ros.actions import Node

#To create a vaiable that contains the configuration parameter for the node
from launch_ros.parameter_descriptions import ParameterValue

#To declare an argument for a launch file
from launch.actions import DeclareLaunchArgument

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
        description="Absolute path to the complete ASRS Robot URDF"
    )

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))

    robot_state_publisher = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        parameters=[{"robot_description": robot_description}] 
    )

    joint_state_publisher_gui = Node(
        package = "joint_state_publisher_gui",
        executable = "joint_state_publisher_gui"
    )

    rviz_node = Node(
        package = "rviz2",
        executable = "rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(get_package_share_directory("asrs_description"), "config", "display.rviz")] 
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])
