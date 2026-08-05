import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'dino_proposer_debug'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gerald Ebmer',
    maintainer_email='gerald.ebmer@gmail.com',
    description=(
        'Standalone diagnostic node: overlays blockpose DINO region-proposer '
        'boxes on a live RGB topic for a human go/no-go check. Not part of '
        'the production detection/planning stack.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dino_proposer_overlay_node = '
            'dino_proposer_debug.dino_proposer_overlay_node:main',
        ],
    },
)
