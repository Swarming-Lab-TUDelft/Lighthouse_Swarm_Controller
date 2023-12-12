from setuptools import setup
import os
from glob import glob

package_name = 'swarm_operation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Swarming Lab TUDelft',
    maintainer_email='operations.swarminglab@tudelft.nl',
    description='Swarming operations package for the Crazyflie 2, for 24/7 demonstrations or swarm research.',
    license='GNU General Public License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'RadioHandler = swarm_operation.RadioHandler:main',
            'CrazyflieNode = swarm_operation.CrazyflieNode:main',
            'MainController = swarm_operation.MainController:main',
            'PositionCommander = swarm_operation.pos_command:main',
            'CollisionAvoidance = swarm_operation.collision_avoidance.ca_node:main',
            'PadManager = swarm_operation.PadManager:main',
            'MasterCommander = swarm_operation.examples.master_commander:main'
            ],
    },
)
