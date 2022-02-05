from glob import glob
import os

from setuptools import setup, find_packages

package_name = "car"
packages = find_packages()

setup(
    name=package_name,
    version='1.0.0',
    packages=packages,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nagib Robotics',
    maintainer_email='vadimrm01@gmail.com',
    description='The car control package (ROS2 version)',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'main = car.main:main',
        ],
    },
)