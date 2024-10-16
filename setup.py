from pathlib import Path

import pytest
from ament_mypy.main import main
from setuptools import setup

from glob import glob

import os

package_name = 'ripspy'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*_launch.py')),
        (os.path.join('share', package_name, 'bin'), ['bin/rips'])
    ],
    py_modules=['ripspy.ripscontext.ripscontext'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='esoriano',
    maintainer_email='esoriano@gsyc.urjc.es',
    description='RIPS prototype',
    license='GPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'ripspy = ripspy.ripspy:main',
        ],
},
)
