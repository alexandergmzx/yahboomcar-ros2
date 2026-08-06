import os
from glob import glob

from setuptools import setup

package_name = 'yahboomcar_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'param'),
         glob(os.path.join('param', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexander Gomez',
    maintainer_email='alexander.gomez.contact@gmail.com',
    description='Lidar odometry with degeneracy detection for the Yahboom microROS car.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'laser_odometry = yahboomcar_localization.laser_odometry_node:main',
        ],
    },
)
