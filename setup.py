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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nuc2',
    maintainer_email='glannuzel@ensc.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_focus = reachy_focus.camera_focus:main',
            'camera_focus_1traitement = reachy_focus.camera_focus_1traitement:main',
            'view_cam = reachy_focus.view_cam:main',
        ],
    },
)
