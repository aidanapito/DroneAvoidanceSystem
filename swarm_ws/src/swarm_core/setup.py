from setuptools import setup
import os
from glob import glob

package_name = 'swarm_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('../worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your-email@example.com',
    description='Multi-UAV swarm coordination with ORCA collision avoidance and grid coverage',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coordinator = swarm_core.coordinator:main',
            'avoidance_orca = swarm_core.avoidance_orca:main',
            'coverage = swarm_core.coverage:main',
        ],
    },
)
