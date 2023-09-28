from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    # turtlesim node
    turtlesim_node = Node(package="turtlesim",
                          executable="turtlesim_node")
    ld.add_action(turtlesim_node)
    
    # node to publish cmd_vel for tutles in turtlesim
    turtlesim_fig_eight_node = Node(package="turtlesim_fig_eight",
                                    executable="turtlesim_fig_eight")
    ld.add_action(turtlesim_fig_eight_node)

    return ld