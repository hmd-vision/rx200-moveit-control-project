import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():


    # Declare launch arguments
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='fake',  # 'actual' or 'fake'
        description='Type of robot to launch: actual or fake'
    )
    default_gripper_state = DeclareLaunchArgument(
        'default_gripper_state',
        default_value='true',
        description='Initial state of the gripper (true=open, false=closed)'
    )

    # Launch file with **Options**
    robot_type = LaunchConfiguration('robot_type')

    hardware_type = PythonExpression(
        ["'", robot_type, "' if '", robot_type, "' == 'actual' else 'fake'"]
    )

    # Start Gazebo server
    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            )
        ),
        condition=IfCondition(PythonExpression(["'", robot_type, "' == 'fake'"]))
    )

    # Start Gazebo GUI
    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            )
        ),
        condition=IfCondition(PythonExpression(["'", robot_type, "' == 'fake'"]))
    )

    # Interbotix MoveIt launch
    interbotix_launch_dir = os.path.join(
        get_package_share_directory('interbotix_xsarm_moveit'),
        'launch'
    )  

    interbotix_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(interbotix_launch_dir, 'xsarm_moveit.launch.py')
        ),
        launch_arguments={
            'robot_model': 'rx200',
            'hardware_type': hardware_type
        }.items()
    )

    # Launch MoveIt client node
    moveit_client_node = Node(
        package='rx200_moveit_control',
        executable='rx200_moveit_client',
        name='rx200_moveit_client',
        output='screen',
        parameters=[{
            'start_state_gripper': LaunchConfiguration('default_gripper_state'),
            'use_sim_time': True
        }]
    )

    # Launch keyboard GUI node
    keyboard_gui_node = Node(
        package='rx200_moveit_control',
        executable='keyboard_gui',
        name='keyboard_gui',
        output='screen'
    )

    # Combine and return the LaunchDescription
    return LaunchDescription([
        robot_type_arg,
        default_gripper_state,
        interbotix_moveit_launch,
        gzserver_launch,
        gzclient_launch,
        moveit_client_node,
        keyboard_gui_node
    ])