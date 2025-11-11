from setuptools import find_packages
from setuptools import setup

setup(
    name='husarion_ugv_msgs',
    version='2.3.1',
    packages=find_packages(
        include=('husarion_ugv_msgs', 'husarion_ugv_msgs.*')),
)
