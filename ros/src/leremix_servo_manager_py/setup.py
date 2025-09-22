from setuptools import setup
import os
from glob import glob

package_name = 'leremix_servo_manager_py'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LeRemix Team',
    maintainer_email='dev@leremix.com',
    description='Python-based servo manager for LeRemix robot using ST3215 library',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_manager_node = leremix_servo_manager_py.servo_manager_node:main',
        ],
    },
)