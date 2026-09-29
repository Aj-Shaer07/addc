import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'addc_autonomy'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        ('share/' + package_name + '/models/landing_pad', glob('models/landing_pad/*.*')),
        ('share/' + package_name + '/models/intelligence_cache', glob('models/intelligence_cache/*.*')),
        ('share/' + package_name + '/models/tree_obstacle', glob('models/tree_obstacle/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team NIDAR',
    maintainer_email='team@nidar.aero',
    description='SAEISS ADDC Autonomous Drone Reconnaissance and HMI Fallback System',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qr_ros = addc_autonomy.qr_ros:main',
            'search_node = addc_autonomy.search_node:main',
            'mission_control = addc_autonomy.mission_control_node:main',
            'precision_landing = addc_autonomy.precision_landing_node:main',
            'hmi_bridge = addc_autonomy.hmi_bridge_node:main',
        ],
    },
)
