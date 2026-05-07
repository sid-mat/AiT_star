from setuptools import setup
import os, glob

package_name = 'ait_star_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob.glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'),
            glob.glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'ait_star_node = ait_star_sim.ait_star_node:main',
            'path_follower  = ait_star_sim.path_follower:main',
        ],
    },
)
