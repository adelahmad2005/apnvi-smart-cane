from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'apnvi_stage3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adel Abdelmoaty',
    maintainer_email='adelahmad2005@gmail.com',
    description='APNVI Stage 3: the device draws a map with the LiDAR and plans a route',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'map_view_node = apnvi_stage3.map_view_node:main',
            'map_manager_node = apnvi_stage3.map_manager_node:main',
            'scan_filter = apnvi_stage3.scan_filter:main',
        ],
    },
)
