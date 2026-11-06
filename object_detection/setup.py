import os
from glob import glob
from setuptools import setup

package_name = 'object_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_transform_publisher = object_detection.static_transform_publisher:main',
            'object_detection = object_detection.object_detection:main',
            'test_pointcloud = object_detection.test_pointcloud:main',
            'static_transform_publisher_real = object_detection.static_transform_publisher_real:main',
            'object_detection_real = object_detection.object_detection_real:main',
        ],
    },
)
