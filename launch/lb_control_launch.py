# lb_control_launch.py

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

import xacro
import tempfile

def to_urdf(xacro_path, parameters=None):
    """Convert the given xacro file to URDF file.
    * xacro_path -- the path to the xacro file
    * parameters -- to be used when xacro file is parsed.
    """
    with tempfile.NamedTemporaryFile(prefix="%s_" % os.path.basename(xacro_path), delete=False) as xacro_file:
        urdf_path = xacro_file.name

    # open and process file
    doc = xacro.process_file(xacro_path, mappings=parameters)
    # open the output file
    with open(urdf_path, 'w') as urdf_file:
        urdf_file.write(doc.toprettyxml(indent='  '))

    return urdf_path


def generate_launch_description():

    package_dir = get_package_share_directory('lb_description')

    # Declare a launch argument for the URDF file
    urdf_file_path = os.path.join(package_dir, 'urdf', 'lb_description.urdf.xacro')
    #process file with xacro
    urdf = to_urdf(urdf_file_path)

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("lb_control"),
            "config",
            "lb_controllers.yaml",
        ]
    )

    controller_params_file = os.path.join(get_package_share_directory('lb_control'),'config','lb_controllers.yaml')

    return LaunchDescription([


        # Launch the robot state publisher to publish joint states
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf],
            #parameters=[{'robot_description': open(urdf_file_path).read()}]
        ),

        # Launch the controller manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            # name='ros2_control_node',
            output={'stdout': 'screen','stderr': 'screen'},
            parameters=[{'robot_description': open(urdf_file_path).read()},controller_params_file],
            arguments=['start', '--controller-manager', 'ros2_control/ControllerManager']
            # prefix=['xterm -e gdb -ex run --args']
        ),


        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['--controller-manager-timeout', '60','joint_state_broadcaster'],
            output = 'screen'
        ),

        # Load the differential drive controller
        Node(
            package='controller_manager',
            executable='spawner',
            name='controller_spawner',
            output='screen',
            arguments=['--controller-manager-timeout', '60','lb_base_controller'],
            parameters=[{'use_sim_time': False}]
        ),

    ])