from setuptools import find_packages
from setuptools import setup

setup(
    name='turtlesim_goal',
    version='0.0.1',
    packages=find_packages(
        include=('turtlesim_goal', 'turtlesim_goal.*')),
)
