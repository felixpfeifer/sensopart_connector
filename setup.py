import os
from glob import glob
from setuptools import setup, find_packages


package_name = 'sensopart_connector'

setup(
    name=package_name,
    version='0.1.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rlh',
    maintainer_email='rlh@teknologisk.dk',
    description='ROS2 connector to SensoPart VISOR Cameras',
    license='No License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensopart_node = sensopart_connector.sensopart_node:main'
        ],
    },
)
