from setuptools import find_packages
from setuptools import setup

setup(
    name='argj801_sensors_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('argj801_sensors_msgs', 'argj801_sensors_msgs.*')),
)
