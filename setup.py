try:
    from setuptools import setup
except ImportError:
    from distutils.core import setup

config = {
    'description': 'Flexible Trajectory Optimization Framework',
    'author': ['Michael Gleicher','Christopher Bodden'],
    'url': '',
    'download_url': '',
    'author_email': ['gleicher@cs.wisc.edu','cbodden@cs.wisc.edu'],
    'version': '0.1',
    'install_requires': ['ad', 'scipy', 'numpy'],
    'packages': ['trajopt','robot','solver','spacetime','utilities'],
    'scripts': [],
    'name': 'TrajectoryOptimization'
}

setup(**config)