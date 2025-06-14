import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch.event_handlers import OnProcessExit, OnExecutionComplete, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
from launch_ros.descriptions import ParameterValue


def generate_launch_description():

    description_package_name = "rb1_ros2_description"
    install_dir = get_package_prefix(description_package_name)

    # GAZEBO model and plugin paths
    gazebo_models_path = os.path.join(description_package_name, 'models')
    os.environ['GAZEBO_MODEL_PATH'] = os.environ.get('GAZEBO_MODEL_PATH', '') + ':' + install_dir + '/share:' + gazebo_models_path
    os.environ['GAZEBO_PLUGIN_PATH'] = os.environ.get('GAZEBO_PLUGIN_PATH', '') + ':' + install_dir + '/lib'

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch'),
            '/gazebo.launch.py'
        ]),
        launch_arguments={
            'verbose': 'false',
            'pause': 'false',
        }.items(),
    )

    # Robot description from xacro
    robot_name_1 = "rb1_robot"
    robot_desc_file = "rb1_ros2_base.urdf.xacro"
    robot_desc_path = os.path.join(get_package_share_directory(
        "rb1_ros2_description"), "xacro", robot_desc_file)

    controller_params_file = os.path.join(
        get_package_share_directory("rb1_ros2_description"),
        "config",
        "rb1_controller.yaml"
    )

    robot_description = ParameterValue(
        Command(['xacro ', robot_desc_path, ' robot_name:=', robot_name_1]),
        value_type=str
    )

    # Robot State Publisher
    rsp_robot1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_name_1,
        parameters=[{
            'frame_prefix': robot_name_1 + '/',
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }],
        output="screen"
    )

    # Spawn robot in Gazebo
    spawn_robot1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name_1,
            '-x', '0.0', '-y', '0.0', '-z', '0.0',
            '-topic', robot_name_1 + '/robot_description'
        ],
        output="screen"
    )

    # Controller spawners
    joint_state_broadcaster_spawner = TimerAction(
        period=120.0,  # Wait 8 seconds for gazebo_ros2_control plugin to load
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster",
                       "-c", "/controller_manager"],
            output="screen"
        )]
    )

    diff_drive_spawner = TimerAction(
        period=120.0,  # Wait 12 seconds
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["rb1_base_controller",
                       "-c", "/controller_manager"],
            output="screen"
        )]
    )

    return LaunchDescription([
        gazebo,
        rsp_robot1,
        spawn_robot1,
        joint_state_broadcaster_spawner,
        diff_drive_spawner
    ])
