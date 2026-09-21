"""The native extension and Python package are built/installed together by CMake."""
from setuptools import setup
setup(name='pin_ik_python', version='2.2.0', packages=['pin_ik_python'],
      package_dir={'': 'src'}, package_data={'pin_ik_python': ['*.so']})
