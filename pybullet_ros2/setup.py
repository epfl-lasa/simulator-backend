import os
from glob import glob

from setuptools import setup

package_name = 'pybullet_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'plugins'), glob('src/pybullet_ros2/plguins/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dominic Reber',
    maintainer_email='dominic@aica.tech',
    description='ROS2 wrapper for PyBullet simulation',
    license='TODO',
    package_dir={'': 'src'},
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['preloader = pybullet_ros2.preloader:main',
                            'pybullet_ros2 = pybullet_ros2.pybullet_ros2:main']
    }
)
