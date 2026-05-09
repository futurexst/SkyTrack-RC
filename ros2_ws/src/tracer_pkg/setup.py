from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'tracer_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')),

        (os.path.join('share', package_name, 'models', 'lane_bot'),
            glob('models/lane_bot/*.sdf') + glob('models/lane_bot/*.config')),

        (os.path.join('share', package_name, 'models', 'lane_bot', 'materials', 'textures'),
            glob('models/lane_bot/materials/textures/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='u24',
    maintainer_email='u24@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node=tracer_pkg.camera_node:main',
            'perception_node=tracer_pkg.perception_node:main',
            'video_logger_node = tracer_pkg.video_logger_node:main',
            'multi_logger_node = tracer_pkg.multi_logger_node:main',
            'sim_controller_node = tracer_pkg.sim_controller_node:main',
        ],
    },
)
