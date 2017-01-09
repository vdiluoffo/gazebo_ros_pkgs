from setuptools import find_packages
from setuptools import setup

setup(
    name='gazebo_plugins',
    version='0.0.0',
    packages=find_packages('src'),
    package_dir={'': 'src'},
)
