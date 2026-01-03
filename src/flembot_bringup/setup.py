from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'flembot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.worlds')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='flemming',
    maintainer_email='flhof23@student.sdu.dk',
    description='Bringup and simulation launch files',
    license='MIT',
)
