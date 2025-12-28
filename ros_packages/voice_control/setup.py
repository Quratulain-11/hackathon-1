from setuptools import setup
from glob import glob
import os

package_name = 'voice_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maintainer Name',
    maintainer_email='maintainer@example.com',
    description='Speech recognition and command parsing for humanoid robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_node = voice_control.voice_node:main',
            'command_parser = voice_control.command_parser:main',
        ],
    },
)