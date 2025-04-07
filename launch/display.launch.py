import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import ExecuteProcess
import yaml

def generate_launch_description():

    yaml_file = os.path.join(get_package_share_directory('my_robot_rviz'), 'config', 'robot_params.yaml')
    with open(yaml_file, 'r') as file:
        params = yaml.safe_load(file)
    urdf_file = os.path.join(get_package_share_directory('my_robot_rviz'), 'urdf', 'my_robot.urdf.xacro')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    robot_description = Command(['xacro', urdf_file])

    return LaunchDescription([

    ExecuteProcess(
                cmd=['ros2', 'run', 'my_robot_rviz', 'forward_kinematics'],
                output='screen'
            ),
    
            # Uruchomienie komendy `ros2 topic echo` (można to zrobić w tle)
     ExecuteProcess(
                cmd=['ros2', 'topic', 'echo', '/dobot/end_effector_pose'],
                output='screen',
                shell=True
    ),
    Node(
                package='my_robot_rviz',
                executable='forward_kinematics',
                name='forward_kinematics',
                output='screen'
    ),
    Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'robot_description': ParameterValue(
                        Command(['xacro ', str(urdf_file)]), value_type=str
                    )
            }]
    ),
 	Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.025', '0', '0', '0', 'map', 'base_link'],
            output='screen'
	),
	Node(
    	    package="tf2_ros",
    	    executable="static_transform_publisher",
    	    arguments=["0", "0", "0", "0", "0", "0", "base_link", "link1"],
    	    output="screen"
	),
    Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
	        name='robot_state_publisher',
            parameters=[{'robot_description': ParameterValue(
                        Command(['xacro ', str(urdf_file)]), value_type=str
                    )
                }],
            output='screen'
    ),
    Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
    ),
    ])

