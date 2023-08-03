from setuptools import setup
import os
from glob import glob

package_name = 'a1_simulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xing',
    maintainer_email='yingguang.xing@alumni.fh-aachen.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'sim_start = a1_simulator.sim_start:main',
        'sim_balance = a1_simulator.sim_balance:main',
        'sim_trotting = a1_simulator.sim_trotting:main',
        'sim_swingtest = a1_simulator.sim_swingtest:main'
        ],
    },
)
