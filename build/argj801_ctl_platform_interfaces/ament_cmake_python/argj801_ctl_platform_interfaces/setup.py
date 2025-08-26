from setuptools import find_packages
from setuptools import setup

setup(
    name='argj801_ctl_platform_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('argj801_ctl_platform_interfaces', 'argj801_ctl_platform_interfaces.*')),
)
