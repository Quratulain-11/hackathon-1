from setuptools import setup
from glob import glob
import os

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        # Include all config files
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maintainer Name',
    maintainer_email='maintainer@example.com',
    description='Basic movement and control nodes for humanoid robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'basic_controller = robot_control.basic_controller:main',
            'communication_example = robot_control.communication_example:main',
            'sensor_publisher = robot_control.sensor_publisher:main',
            'actuator_subscriber = robot_control.actuator_subscriber:main',
        ],
    },
)