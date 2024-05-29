import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*launch.py")),
        (os.path.join(package_name, "src"), glob("src/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mannaja',
    maintainer_email='manman7852078520@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'runRobot = robot_core.main_run:main',
        ],
    },
)
