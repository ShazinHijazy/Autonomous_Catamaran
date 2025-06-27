from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='catamaran_ros',
            executable='imu_node',
            name='imu_node',
            output='screen'
        ),
        Node(
            package='catamaran_ros',
            executable='gnss_node',
            name='gnss_node',
            output='screen'
        ),
        Node(
            package='catamaran_ros',
            executable='rplidar_launch',
            name='rplidar_node',
            output='screen'
        ),
        Node(
            package='catamaran_ros',
            executable='ultrasonic_waterproof_node',
            name='ultrasonic_waterproof_node',
            output='screen'
        ),
        Node(
            package='catamaran_ros',
            executable='ultrasonic_node',
            name='ultrasonic_node',
            output='screen'
        ),
       # Node(
            #package='sensor_nodes',
            #executable='power_distribution_board_node',
            #name='power_distribution_board_node',
            #output='screen'
        #),
        #Node(
         #   package='catamaran_ros',
          #  executable='voltage_and_current_node',
           # name='voltage_and_current_node',
            #output='screen'
        #),
    ])
