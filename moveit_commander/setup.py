from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['moveit_commander']
d['scripts'] = ['bin/moveit_commander_cmdline.py', 'bin/moveit_planning_scene_from_ork.py']
d['package_dir'] = {'': 'src'}

setup(**d)
