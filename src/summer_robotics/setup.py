import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'summer_robotics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
	    ('share/ament_index/resource_index/packages',
		['resource/' + package_name]),
	    ('share/' + package_name, ['package.xml']),
	    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
	    (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
	    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
	    (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sri',
    maintainer_email='sri@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        	'robot_status_publisher = summer_robotics.robot_status_publisher:main',
        	'robot_status_subscriber = summer_robotics.robot_status_subscriber:main',
        	'arm_position_commander = summer_robotics.arm_position_commander:main',
        	'object_selector = summer_robotics.object_selector:main',
        	'task_planner = summer_robotics.task_planner:main',
        	'object_registry_node = summer_robotics.object_registry_node:main',
        	'fake_perception_node = summer_robotics.fake_perception_node:main',
        	'robot_adapter_node = summer_robotics.robot_adapter_node:main',
        	'object_visualizer_node = summer_robotics.object_visualizer_node:main',
        	'color_perception_node = summer_robotics.color_perception_node:main',
        	'visualization_marker_node = summer_robotics.visualization_marker_node:main',
        	'fake_gaze_node = summer_robotics.fake_gaze_node:main',
        	'webcam_gaze_node = summer_robotics.webcam_gaze_node:main',
        	'gaze_object_selector_node = summer_robotics.gaze_object_selector_node:main',
        ],
    },
)
