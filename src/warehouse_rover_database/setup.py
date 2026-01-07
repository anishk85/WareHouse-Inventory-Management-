from setuptools import setup
import os
from glob import glob

package_name = 'warehouse_rover_database'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Warehouse Rover Team',
    maintainer_email='team@example.com',
    description='MongoDB-based warehouse inventory management for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inventory_node_mongo.py = warehouse_rover_database.inventory_node_mongo:main',
        ],
    },
)
