import sys, os

sys.path += [ os.path.abspath( 'doc' )]

extensions = [ 'sphinx.ext.extlinks']

# The master toctree document.
master_doc = 'index'

# The suffix of source filenames.
source_suffix = '.rst'

project = u'moveit_ikfast'

copyright = u'2013,  SRI International'

# If true, sectionauthor and moduleauthor directives will be shown in the
# output. They are ignored by default.
show_authors = True

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = 'sphinx'

extlinks = {'codedir': ('https://github.com/ros-planning/moveit_pr2/blob/hydro-devel/pr2_moveit_tutorials/%s', ''),
            'moveit_core': ('http://docs.ros.org/api/moveit_core/html/classmoveit_1_1core_1_1%s.html', ''),
            'planning_scene': ('http://docs.ros.org/api/moveit_core/html/classplanning__scene_1_1%s.html', ''),
            'planning_scene_monitor': ('http://docs.ros.org/api/moveit_ros_planning/html/classplanning__scene__monitor_1_1%s.html', ''),
            'collision_detection_struct': ('http://docs.ros.org/api/moveit_core/html/structcollision__detection_1_1%s.html', ''),
            'collision_detection_class': ('http://docs.ros.org/api/moveit_core/html/classcollision__detection_1_1%s.html', ''),
            'kinematic_constraints': ('http://docs.ros.org/api/moveit_core/html/classkinematic__constraints_1_1%s.html', ''),
            'moveit_core_files': ('http://docs.ros.org/api/moveit_core/html/%s.html', ''),
            'move_group_interface': ('http://docs.ros.org/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1%s.html', ''),
            'moveit_website': ('http://moveit.ros.org/%s/', '')}
