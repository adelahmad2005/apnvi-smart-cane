from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'apnvi_stage4'

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
    description='APNVI Stage 4: the device guides the user with its two driven side wheels',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'drive_link_node = apnvi_stage4.drive_link_node:main',
            'hazard_gate_node = apnvi_stage4.hazard_gate_node:main',
            'guide_node = apnvi_stage4.guide_node:main',
            'vibration_node = apnvi_stage4.vibration_node:main',
            'keyboard_drive = apnvi_stage4.keyboard_drive:main',
            'free_mode_node = apnvi_stage4.free_mode_node:main',
            'health_node = apnvi_stage4.health_node:main',
        ],
    },
)
