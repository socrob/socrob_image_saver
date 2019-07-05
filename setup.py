#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['image_saver_ros'],
 package_dir={'image_saver_ros': 'ros/src/image_saver_ros'}
)

setup(**d)
