import os
from glob import glob

from setuptools import setup

package_name = 'yahboomcar_config'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'param'),
         glob('param/*.yaml') + glob('param/*.json')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexander Gomez',
    maintainer_email='alexander.gomez.contact@gmail.com',
    description='First-party robot1 config: corrected EKF, SLAM params/launch, RViz views.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
