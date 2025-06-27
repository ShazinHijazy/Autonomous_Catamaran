from setuptools import setup
import os
from glob import glob

package_name = 'autonomous_catamaran'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shazin Hijazy',
    maintainer_email='shazhijazy@gmail.com',
    description='Complete ROS 2 Humble-based system for controlling and navigating an autonomous catamaran using integrated sensors, thruster control, obstacle avoidance, and telemetry.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Sensor Nodes
            'imu_node = autonomous_catamaran.imu_node:main',
            'gnss_node = autonomous_catamaran.gnss_node:main',
            'ultrasonic_waterproof_node = autonomous_catamaran.ultrasonic_waterproof_node:main',
            'ultrasonic_servo_node = autonomous_catamaran.ultrasonic_servo_node:main',
            'voltage_and_current_node = autonomous_catamaran.voltage_and_current_node:main',
            'power_distribution_board_node = autonomous_catamaran.power_distribution_board_node:main',

            # Thruster Control
            'thruster_control = autonomous_catamaran.thruster_control:main',
            'teleop_thruster = autonomous_catamaran.teleop_thruster:main',
            'pwm_output = autonomous_catamaran.pwm_output:main',

            # Telemetry & Failsafe
            'telemetry_node = autonomous_catamaran.telemetry_node:main',
            'return_to_home_node = autonomous_catamaran.return_to_home_node:main',

            # Obstacle Avoidance
            'obstacle_avoidance_node = autonomous_catamaran.obstacle_avoidance_node:main',
        ],
    },
)
