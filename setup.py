from setuptools import setup
import os
from glob import glob

package_name = 'odrive_can'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Veera Ragav Chikkanachettiyar Veeramani',
    maintainer_email='veeraragav0@gmail.com',
    description='ROS2 package for Odrive Controller with CAN interface',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'odrive_can_node = odrive_can.odrive_can:main',
             'cmdvel_to_wheelvel_node = odrive_can.cmdvel_to_wheelvel:main',
             'odometry_node = odrive_can.odometry:main',
             'fake_encoder_node = odrive_can.fake_encoder_pub:main'
        ],
    },
)
