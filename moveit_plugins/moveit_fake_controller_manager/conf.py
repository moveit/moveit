import sys, os

extensions = [ 'sphinx.ext.extlinks' ]

# The master toctree document.
master_doc = 'doc/tutorial'

# The suffix of source filenames.
source_suffix = '.rst'

project = u'moveit_rviz_plugin_tutorial'

copyright = u'2016, Bielefeld University'

# If true, sectionauthor and moduleauthor directives will be shown in the
# output. They are ignored by default.
show_authors = True

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = 'sphinx'

html_theme = "moveit-theme"
html_theme_path = ["doc"]

extlinks = {'moveit_website': ('http://moveit.ros.org/%s/', '')}
