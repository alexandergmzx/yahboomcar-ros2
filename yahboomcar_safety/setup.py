import os
from glob import glob

from setuptools import setup

package_name = 'yahboomcar_safety'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexander Gomez',
    maintainer_email='alexander.gomez.contact@gmail.com',
    description='Lidar speed governor for the Yahboom microROS car.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_governor = yahboomcar_safety.cmd_vel_governor:main',
        ],
    },
)
