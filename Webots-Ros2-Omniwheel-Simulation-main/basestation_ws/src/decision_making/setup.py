from setuptools import setup
import os
from glob import glob

package_name = 'decision_making'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arbaaz',
    maintainer_email='your.email@example.com',
    description='Decision making package for RoboCup MSL',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'decision_node = decision_making.decision_node:main'
        ],
    },
)
