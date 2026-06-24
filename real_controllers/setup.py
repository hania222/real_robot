from setuptools import find_packages, setup

package_name = 'real_controllers'

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
    maintainer='hania',
    maintainer_email='haniahazem60@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hardware_bridge_node = real_controllers.hardware_bridge_node:main',
            'carriage_bridge_node= real_controllers.carriage_bridge_node:main',
            'lift_bridge_node = real_controllers.lift_bridge_node:main',
        ],
    },
)
