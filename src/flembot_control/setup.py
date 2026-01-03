from setuptools import setup

package_name = 'flembot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='flemming',
    maintainer_email='flhof23@student.sdu.dk',
    description='Robot control nodes',
    license='MIT',
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'teleop = flembot_control.teleop:main',
        ],
    },
)
