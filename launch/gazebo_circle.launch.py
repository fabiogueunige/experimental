"""
Spawn Robot Description
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    test_robot_description_share = FindPackageShare(package='robot_urdf').find('robot_urdf')
    default_model_path = os.path.join(test_robot_description_share, 'urdf/robot4.xacro')
    default_world_path = os.path.join(test_robot_description_share, 'worlds/marker_test.world')
    rviz_config_path = os.path.join(test_robot_description_share, 'config/rviz.rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    aruco_node = Node(
        package='ros2_aruco',
        executable='aruco_node',
        name='aruco_node'
        # parameters=[{'marker_size': 0.0625},
        #             {'aruco_dictionary_id': 'DICT_5X5_250'},
        #             {'image_topic': '/camera/image_raw'},  !! cambia to compressed!!
        #             {'camera_info_topic': '/camera/camera_info'}],
        # output='screen'
    )

    robot_rotation_node = Node(
        package='robot_urdf',
        executable='robot_rotation',
        name='robot_rotation',
        output='screen'
    )

    marker_sub_node = Node(
        package='robot_urdf',
        executable='marker_sub',
        name='marker_sub'
    )


    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'my_test_robot', '-topic', '/robot_description', '-Y', '3.14'],
                        output='screen')
    

    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                                    description='Absolute path to robot urdf file'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        aruco_node,
        robot_rotation_node,
        marker_sub_node,
        spawn_entity,
        
        ExecuteProcess(
            cmd=['gazebo', '--verbose', default_world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'),
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            output='screen'),
        
    ])
