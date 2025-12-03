import os
from glob import glob

from setuptools import setup

package_name = 'haar_op3_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # === FOLDER YANG DIINSTALL ===
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config', glob('config/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotis',
    maintainer_email='robotis@example.com',
    description='Haar OP3 Detector',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'detector_node = haar_op3_detector.detector_node:main'
        ],
    },
)
