from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'zd_px4_comm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install the launch files into the package's share directory
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        # Install the package.xml into the share directory
        ('share/' + package_name, ['package.xml']),
        # Install other necessary resource files
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='junyan',
    maintainer_email='chua.junyan0612@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'zd_px4_command = zd_px4_comm.zd_px4_comm:main',
            'zd_processes = zd_px4_comm.zd_processes:main',
            'zd_processes_actual = zd_px4_comm.zd_processes_actual:main',
            'zd_calibration = zd_px4_comm.zd_px4_calibration:main',
            'zd_logging = zd_px4_comm.zd_logging:main',
        ],
    },
)
