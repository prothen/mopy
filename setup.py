## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
import os
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=["mopy"],
    package_dir={'mopy': 'mopy'})

setup(**setup_args)
