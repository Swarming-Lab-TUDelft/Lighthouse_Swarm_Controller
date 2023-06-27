from setuptools import setup
import os
from glob import glob

package_name = '247Swarm'

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
            'RadioHandler = 247Swarm.RadioHandler:main',
            'CrazyflieNode = 247Swarm.CrazyflieNode:main',
            'MainController = 247Swarm.MainController:main',
            'GUI = 247Swarm.GUI:main',
            'GUI_test_pub = 247Swarm.GUI_test_pub:main',
            'PositionCommander = 247Swarm.pos_command:main',
            'CollisionAvoidance = 247Swarm.collision_avoidance.ca_node:main',
            'PadManager = 247Swarm.PadManager:main',
        ],
    },
)
