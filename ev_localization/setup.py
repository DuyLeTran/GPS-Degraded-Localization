from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ev_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='GPS-Degraded Localization Package',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'gps_monitor = ev_localization.gps_monitor:main',
            'monocular_vio = ev_localization.monocular_vio:main',
            'landmark_ghost = ev_localization.landmark_ghost:main',
            'ekf_fusion = ev_localization.ekf_fusion:main',
        ],
    },
)
