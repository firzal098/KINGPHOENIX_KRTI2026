import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gripper'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Firzal',
    maintainer_email='firzalyt@gmail.com',
    description='Gripper control package with WebotsGripper UDP bridge node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'webots_gripper = gripper.webots_gripper:main',
            'WebotsGripper = gripper.webots_gripper:main',
        ],
    },
)
