## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import find_packages, setup

d = generate_distutils_setup(
    packages=find_packages('src'),
    package_dir={'': 'src'}
)

setup(**d)
