import os
from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'twbot_demo' 

    world_path = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'demoworld.sdf'
    )

    
    # Include your robot_state_publisher launch
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'rsp.launch.py'
            )
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Launch Gazebo Sim 
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            # Use "gz_args" exactly like the example in ros_gz_sim
            'gz_args': f'-r -v 4 "{world_path}"'
        }.items()
    )
    # Spawn robot 
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_bot',
            '-topic', 'robot_description',
        ],
        output='screen'
    )
    bridge_params = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'gz_bridge.yaml'
    )

    # ros_gz_bridge
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_params}],
        output='screen'
        )
    
    torque_driver_node = Node(
        package='twbot_demo',
        executable='torque_driver.py',  
        name='torque_driver',
        output='screen'
    )

    #node pour control par PID
    balance_node = Node(
        package='twbot_demo',
        executable='balancing.py',  
        name='balance',
        output='screen'
    )
    #node pour control par ADRC
    ADRC_node = Node(
        package='twbot_demo',
        executable='ADRC_control.py',  
        name='ADRC',
        output='screen'
    )


    #zid hna t3 LQR ida dertha

    return LaunchDescription([
        rsp,
        gazebo_launch,
        spawn_entity,
        gz_bridge,
        torque_driver_node,
        ADRC_node,
    ])