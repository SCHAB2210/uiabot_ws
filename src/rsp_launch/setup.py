from setuptools import setup
import os
from glob import glob

package_name = 'rsp_launch'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Install launch files, including rsp.launch.py
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include URDF, Xacro, and STL files if needed for visualization
        (os.path.join('share', package_name, 'description'), glob('description/*')),
        # Configuration files like RViz and YAML if necessary
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Package for launching robot visualization with URDF and sensor nodes.',
    license='License Declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This package is dedicated to launch files, so no executables are needed
        ],
    },
)

