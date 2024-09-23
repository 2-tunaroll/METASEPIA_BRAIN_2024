from setuptools import find_packages, setup

package_name = 'arduino'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/arduino_launch.py']),
        ('share/' + package_name + '/config', ['config/joystick.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='metasepia',
    maintainer_email='peppegrasso02@gmail.com',
    description='handles communication between arduino and pi',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_node = arduino.arduino_node:main',
            'controller_node = arduino.controller_node:main'
        ],
    },
)
