import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    robotXacroName = 'differential_drive_robot'
    namePackage = 'agrirover'  # <--- CHANGE IF NEEDED
    modelFileRelativePath = "model/robot.xacro"

    # Process the xacro file
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)
    robotDescription = xacro.process_file(pathModelFile).toxml()

    # Launch Gazebo with empty world
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]
    )

    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch,
        launch_arguments={
            'gz_args': '-r -v 4 empty.sdf',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Spawn the robot into Gazebo
    spawnModelNodeGazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robotXacroName,
            '-topic', 'robot_description',
        ],
        output='screen',
    )

    # Publish robot state
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robotDescription,
             'use_sim_time': True}
        ],
        output='screen',
    )

    # Bridge topics between ROS 2 and Gazebo
    bridge_params = os.path.join(
        get_package_share_directory(namePackage),
        'parameters',
        'bridge_parameters.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    # Launch teleop keyboard control (press keys in terminal)
    teleopNode = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e',  # Opens in a separate terminal
        remappings=[
            ('/cmd_vel', '/cmd_vel')  # Should match what your robot listens to
        ]
    )

    # Final launch description
    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(gazeboLaunch)
    launchDescriptionObject.add_action(spawnModelNodeGazebo)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    launchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)
    launchDescriptionObject.add_action(teleopNode)

    return launchDescriptionObject
