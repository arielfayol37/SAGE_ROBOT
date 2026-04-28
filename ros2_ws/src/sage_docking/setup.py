from setuptools import find_packages, setup

package_name = 'sage_docking'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'docking_node = sage_docking.docking_node:main',
            'dock_pose_publisher = sage_docking.dock_pose_publisher:main',
	],
    },
)
