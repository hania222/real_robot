from setuptools import setup
import os
from glob import glob

package_name = 'real_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
        (os.path.join('share', package_name, 'web'),
            glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Launch files and config for the real robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'joystick_ws_node = real_bringup.joystick_ws_node:main',
            'twist_to_twiststamped = real_bringup.twist_to_twiststamped:main',
        ],
    },
)
