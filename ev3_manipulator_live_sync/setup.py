from setuptools import setup
import os
from glob import glob

package_name = 'ev3_manipulator_live_sync'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Filter explicitly for .xacro and .urdf files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro') + glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ajaykrishna Venkatesan',
    maintainer_email='aj.grizzy@gmail.com',
    description='The ' + package_name + ' package',
    license='MIT: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sorting_node = ev3_manipulator_live_sync.sorting_node:main',
            'hardware_interface = ev3_manipulator_live_sync.hardware_interface:main',
        ],
    },
)
