from setuptools import setup
import os
from glob import glob

package_name = 'joint_state_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Custom joint state publisher for ROS 2.',
    license='License Declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_publisher = joint_state_publisher.joint_state_publisher:main',
        ],
    },
)

