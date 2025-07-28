#  ## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
# from distutils.core import setup
# from catkin_pkg.python_setup import generate_distutils_setup

# d = generate_distutils_setup(
#     packages=['transformations'],
#     scripts=[''],
#     package_dir={'': 'scripts'}
# )

# setup(**d)

import os
from setuptools import setup

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['tf_function'],
    scripts=[],
    package_dir={'': '.'}
)
setup(**d)
