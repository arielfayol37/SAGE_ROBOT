from setuptools import find_packages, setup

package_name = 'web_teleop_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='agi',
    maintainer_email='arielfayol1@gmail.com',
    description='Bridge between WebRTC, MCU, and ros2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = web_teleop_bridge.my_node:main',
            'control_bridge = web_teleop_bridge.control_bridge:main',
            'serial_bridge_without_imu = web_teleop_bridge.serial_bridge_without_imu:main',
            'serial_bridge = web_teleop_bridge.serial_bridge:main',
        ],
    },
)
