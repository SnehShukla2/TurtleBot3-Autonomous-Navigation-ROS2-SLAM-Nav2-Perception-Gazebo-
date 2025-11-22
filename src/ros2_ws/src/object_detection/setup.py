import os
from setuptools import setup
from glob import glob

package_name = 'object_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sneh',
    maintainer_email='sneh@example.com',
    description='ROS2 package for object detection',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detection = object_detection.object_detection:main'
        ],
    },
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py'))
    ],
)

