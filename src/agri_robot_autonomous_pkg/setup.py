from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'agri_robot_autonomous_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.py')),
        ('share/' + package_name, glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ebrahim',
    maintainer_email='ibrahim.abdelghafar@dakahlia.net',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'magnify_cmd_vel = agri_robot_autonomous_pkg.magnify_cmd_vel:main',
        ],
    },
)
