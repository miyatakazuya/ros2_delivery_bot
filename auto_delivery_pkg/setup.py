import os
from glob import glob
from setuptools import setup

package_name = 'auto_delivery_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('models/*.blob')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_controller = auto_delivery_pkg.mission_controller:main',
            'parking_controller = auto_delivery_pkg.parking_controller:main',
            'servo_controller = auto_delivery_pkg.servo_controller:main',
            'box_detection = auto_delivery_pkg.box_detection:main',
            'apriltag_node = auto_delivery_pkg.apriltag_node:main',      
            'apriltag_rear_node = auto_delivery_pkg.webcam_apriltag:main',
        ],
    },
)