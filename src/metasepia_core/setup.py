import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'metasepia_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.json')) + glob(os.path.join('config', '*.yaml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='metasepia',
    maintainer_email='peppegrasso02@gmail.com',
    description='launch package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
    },
)
