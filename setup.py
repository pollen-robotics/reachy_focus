import os
from glob import glob
from setuptools import setup

package_name = 'reachy_focus'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'zoom_kurokesu>=1',
    ],
    zip_safe=True,
    maintainer='nuc2',
    maintainer_email='contact@pollen-robotics.com',
    description='ROS2 autofocus algorithm for Reachy',
    license='Apache-2.0 License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_focus = reachy_focus.camera_focus:main',
        ],
    },
)
