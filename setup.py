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
    ],
    install_requires=[
        'setuptools',
        'zoom_kurokesu>=1',
        'pynput',
    ],
    zip_safe=True,
    maintainer='nuc2',
    maintainer_email='glannuzel@ensc.fr',
    description='ROS2 Auto-focus algorithm for Reachy',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_focus = reachy_focus.camera_focus:main',
            'focus = reachy_focus.focus:main',
            'camera_focus_1traitement = reachy_focus.camera_focus_1traitement:main',
            'view_cam = reachy_focus.view_cam:main',
            'camera_zoom_client = reachy_focus.camera_zoom_client:main',
        ],
    },
)
