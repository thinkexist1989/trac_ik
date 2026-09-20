"""The native extension and Python package are built/installed together by CMake."""
from setuptools import setup
setup(name='trac_ik_python', version='2.2.0', packages=['trac_ik_python'],
      package_dir={'': 'src'}, package_data={'trac_ik_python': ['*.so']})
