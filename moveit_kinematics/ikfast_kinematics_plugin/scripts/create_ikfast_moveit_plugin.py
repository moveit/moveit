#! /usr/bin/env python
'''
IKFast Plugin Generator for MoveIt!

Creates a kinematics plugin using the output of IKFast from OpenRAVE.
This plugin and the move_group node can be used as a general
kinematics service, from within the moveit planning environment, or in
your own ROS node.

Author: Dave Coleman, CU Boulder
        Based heavily on the arm_kinematic_tools package by Jeremy Zoss, SwRI
        and the arm_navigation plugin generator by David Butterworth, KAIST
Date: March 2013

'''
'''
Copyright (c) 2013, Jeremy Zoss, SwRI
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of the Willow Garage, Inc. nor the names of its
contributors may be used to endorse or promote products derived from
this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
IABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
'''

import glob
import sys
import roslib
import re
import os
import yaml
from lxml import etree
import shutil

plugin_gen_pkg = 'moveit_kinematics'  # package containing this file
plugin_sub_dir = 'ikfast_kinematics_plugin' # sub directory which contains the template directory
# Allowed search modes, see SEARCH_MODE enum in template file
search_modes = ['OPTIMIZE_MAX_JOINT', 'OPTIMIZE_FREE_JOINT' ]

if __name__ == '__main__':
   # Check input arguments
   try:
      robot_name = sys.argv[1]
      planning_group_name = sys.argv[2]
      moveit_plugin_pkg = sys.argv[3]
      if len(sys.argv) == 6:
         ikfast_output_file = sys.argv[5]
         search_mode = sys.argv[4]
         if search_mode not in search_modes:
            print 'Invalid search mode. Allowed values: ', search_modes
            raise Exception()
      elif len(sys.argv) == 5:
         search_mode = search_modes[0];
         print "Warning: The default search has changed from OPTIMIZE_FREE_JOINT to now %s!" % (search_mode)
         ikfast_output_file = sys.argv[4]
      else:
         raise Exception()
   except:
      print("\nUsage: create_ikfast_plugin.py <yourrobot_name> <planning_group_name> <moveit_plugin_pkg> [<search_mode>] <ikfast_output_path>\n")
      sys.exit(-1)
   print '\nIKFast Plugin Generator'

   # Setup key package directories
   try:
      plan_pkg = robot_name + '_moveit_config'
      plan_pkg_dir = roslib.packages.get_pkg_dir(plan_pkg)
      print 'Loading robot from \''+plan_pkg+'\' package ... '
   except:
      print '\nERROR: can\'t find package '+plan_pkg+'\n'
      sys.exit(-1)
   try:
      plugin_pkg = moveit_plugin_pkg
      plugin_pkg_dir = roslib.packages.get_pkg_dir(plugin_pkg)
      print 'Creating plugin in \''+plugin_pkg+'\' package ... '
   except:
      print '\nERROR: can\'t find package '+plugin_pkg+'\n'
      sys.exit(-1)

   # Check for at least 1 planning group
   try:
      srdf_files = glob.glob(plan_pkg_dir+'/config/*.srdf')
      if (len(srdf_files) == 1):
         srdf_file_name = srdf_files[0]
      else:
         srdf_file_name = plan_pkg_dir + '/config/' + robot_name + '.srdf'
      srdf = etree.parse(srdf_file_name).getroot()
   except:
      print("\nERROR: unable to parse robot configuration file\n" + srdf_file_name + "\n")
      sys.exit(-1)
   try:
      if (robot_name != srdf.get('name')):
         print '\nERROR: non-matching robot name found in ' + srdf_file_name + '.' \
             + '  Expected \'' + robot_name + '\',' + ' found \''+srdf.get('name')+'\''
         raise

      groups = srdf.findall('group')
      if(len(groups) < 1) : # No groups
         raise
      if groups[0].get('name') == None: # Group name is blank
         raise
   except:
      print("\nERROR: need at least 1 planning group in robot planning description ")
      print srdf_file_name + '\n'
      sys.exit(-1)
   print '  found ' + str(len(groups)) + ' planning groups: ' \
       + ", ".join([g.get('name') for g in groups])

   # Select manipulator arm group
   planning_group = None
   for g in groups:
      foundName  = (g.get('name').lower() == planning_group_name.lower())

      if (foundName):
         planning_group = g
         break
   if planning_group is None:
      print '\nERROR: could not find planning group ' + planning_group_name + ' in SRDF.\n'
      sys.exit(-1)
   print '  found group \'' + planning_group_name + '\''

   # Create src and include folders in target package
   plugin_pkg_src_dir = plugin_pkg_dir+'/src/'
   plugin_pkg_include_dir = plugin_pkg_dir+'/include/'

   if not os.path.exists(plugin_pkg_src_dir):
      os.makedirs(plugin_pkg_src_dir)
   if not os.path.exists(plugin_pkg_include_dir):
      os.makedirs(plugin_pkg_include_dir)

   # Check for source code generated by IKFast
   if not os.path.exists(ikfast_output_file):
      print '\nERROR: can\'t find IKFast source code at \'' + \
          ikfast_output_file + '\'\n'
      print 'Make sure this input argument is correct \n'
      sys.exit(-1)

   # Copy the source code generated by IKFast into our src folder
   solver_file_name = plugin_pkg_dir+'/src/'+robot_name+'_'+planning_group_name+'_ikfast_solver.cpp'
   # Check if they are the same file - if so, skip
   skip = False
   if os.path.exists(ikfast_output_file) & os.path.exists(solver_file_name):
      if os.path.samefile(ikfast_output_file, solver_file_name):
         print 'Skipping copying ' + solver_file_name + ' since it is already in the correct location'
         skip = True
   if not skip:
      shutil.copy2(ikfast_output_file,solver_file_name)
   # Error check
   if not os.path.exists(solver_file_name):
      print '\nERROR: Unable to copy IKFast source code from \'' + ikfast_output_file + '\'' + ' to \'' + solver_file_name + '\''
      print 'Manually copy the source file generated by IKFast to this location \n'
      sys.exit(-1)

   # Detect version of IKFast used to generate solver code
   solver_version = 0
   with open(solver_file_name,'r') as src:
      for line in src:
         if line.startswith('/// ikfast version'):
            line_search = re.search('ikfast version (.*) generated', line)
            if line_search:
               solver_version = int(line_search.group(1), 0)
            break
   print '  found source code generated by IKFast version ' + str(solver_version)

   # Get template folder location
   try:
      plugin_gen_dir = os.path.join(roslib.packages.get_pkg_dir(plugin_gen_pkg), plugin_sub_dir)
   except:
      print '\nERROR: can\'t find package '+plugin_gen_pkg+' \n'
      sys.exit(-1)

   # Chose template depending on IKFast version
   if solver_version >= 56:
      template_version = 61
   else:
      print '\nERROR this converter is not made for IKFast 54 or anything but 61'
      sys.exit(-1)

   # Check if IKFast header file exists
   template_header_file = plugin_gen_dir + '/templates/ikfast.h'
   if not os.path.exists(template_header_file):
      print '\nERROR: can\'t find ikfast header file at \'' + template_header_file + '\'\n'
      sys.exit(-1)

   # Copy the IKFast header file into the include directory
   header_file_name = plugin_pkg_dir+'/include/ikfast.h'
   shutil.copy2(template_header_file,header_file_name)
   if not os.path.exists(header_file_name):
      print '\nERROR: Unable to copy IKFast header file from \'' + \
          template_header_file + '\'' + ' to \'' + header_file_name + '\' \n'
      print 'Manually copy ikfast.h to this location \n'
      sys.exit(-1)

   # Check if template exists
   template_file_name = plugin_gen_dir + '/templates/ikfast' + str(template_version) + '_moveit_plugin_template.cpp'

   if not os.path.exists(template_file_name):
      print '\nERROR: can\'t find template file at \'' + template_file_name + '\'\n'
      sys.exit(-1)

   # Create plugin source from template
   template_file_data = open(template_file_name, 'r')
   template_text = template_file_data.read()
   template_text = re.sub('_ROBOT_NAME_', robot_name, template_text)
   template_text = re.sub('_GROUP_NAME_', planning_group_name, template_text)
   template_text = re.sub('_SEARCH_MODE_', search_mode, template_text)
   plugin_file_base = robot_name + '_' + planning_group_name + '_ikfast_moveit_plugin.cpp'

   plugin_file_name = plugin_pkg_dir + '/src/' + plugin_file_base
   with open(plugin_file_name,'w') as f:
      f.write(template_text)
   print '\nCreated plugin file at \'' + plugin_file_name + '\''

   # Create plugin definition .xml file
   ik_library_name = robot_name + "_" + planning_group_name + "_moveit_ikfast_plugin"
   plugin_name = robot_name + '_' + planning_group_name + \
       '_kinematics/IKFastKinematicsPlugin'
   plugin_def = etree.Element("library", path="lib/lib"+ik_library_name)
   cl = etree.SubElement(plugin_def, "class")
   cl.set("name", plugin_name)
   cl.set("type", 'ikfast_kinematics_plugin::IKFastKinematicsPlugin')
   cl.set("base_class_type", "kinematics::KinematicsBase")
   desc = etree.SubElement(cl, "description")
   desc.text = 'IKFast'+str(template_version)+' plugin for closed-form kinematics'

   # Write plugin definition to file
   def_file_base = ik_library_name + "_description.xml"
   def_file_name = plugin_pkg_dir + "/" + def_file_base
   with open(def_file_name,'w') as f:
      etree.ElementTree(plugin_def).write(f, xml_declaration=True, pretty_print=True, encoding="UTF-8")
   print '\nCreated plugin definition at: \''+def_file_name+'\''




   # Check if CMakeLists file exists
   cmake_template_file = plugin_gen_dir+"/templates/CMakeLists.txt"
   if not os.path.exists(cmake_template_file):
      print '\nERROR: can\'t find CMakeLists template file at \'' + cmake_template_file + '\'\n'
      sys.exit(-1)

   # Create new CMakeLists file
   cmake_file = plugin_pkg_dir+'/CMakeLists.txt'

   # Create plugin source from template
   template_file_data = open(cmake_template_file, 'r')
   template_text = template_file_data.read()
   template_text = re.sub('_ROBOT_NAME_', robot_name, template_text)
   template_text = re.sub('_GROUP_NAME_', planning_group_name, template_text)
   template_text = re.sub('_PACKAGE_NAME_', moveit_plugin_pkg, template_text)
   template_text = re.sub('_LIBRARY_NAME_', ik_library_name, template_text)

   with open(cmake_file,'w') as f:
      f.write(template_text)
   print '\nOverwrote CMakeLists file at \'' + cmake_file + '\''

   # Add plugin export to package manifest
   parser = etree.XMLParser(remove_blank_text=True)
   package_file_name = plugin_pkg_dir+"/package.xml"
   package_xml = etree.parse(package_file_name, parser)

   # Make sure at least all required dependencies are in the depends lists
   build_deps = ["liblapack-dev", "moveit_core", "pluginlib", "roscpp", "tf_conversions"]
   run_deps   = ["liblapack-dev", "moveit_core", "pluginlib", "roscpp", "tf_conversions"]

   def update_deps(reqd_deps, req_type, e_parent):
      curr_deps = [e.text for e in e_parent.findall(req_type)]
      missing_deps = set(reqd_deps) - set(curr_deps)
      for d in missing_deps:
         etree.SubElement(e_parent, req_type).text = d
      return missing_deps

   update_deps(build_deps, "build_depend", package_xml.getroot())
   update_deps(run_deps, "exec_depend", package_xml.getroot())

   # Check that plugin definition file is in the export list
   new_export = etree.Element("moveit_core", \
                                 plugin="${prefix}/"+def_file_base)
   export_element = package_xml.getroot().find("export")
   if export_element == None:
      export_element = etree.SubElement(package_xml.getroot(), "export")
   found = False
   for el in export_element.findall("moveit_core"):
      found = (etree.tostring(new_export) == etree.tostring(el))
      if found: break

   if not found:
      export_element.append(new_export)

   # Always write the package xml file, even if there are no changes, to ensure
   # proper encodings are used in the future (UTF-8)
   with open(package_file_name,"w") as f:
      package_xml.write(f, xml_declaration=True, pretty_print=True, encoding="UTF-8")
   print '\nModified package.xml at \''+package_file_name+'\''

   # Modify kinematics.yaml file
   kin_yaml_file_name = plan_pkg_dir+"/config/kinematics.yaml"
   with open(kin_yaml_file_name, 'r') as f:
      kin_yaml_data = yaml.safe_load(f)
   kin_yaml_data[planning_group_name]["kinematics_solver"] = plugin_name
   with open(kin_yaml_file_name, 'w') as f:
      yaml.dump(kin_yaml_data, f,default_flow_style=False)
   print '\nModified kinematics.yaml at ' + kin_yaml_file_name

   # Create a script for easily updating the plugin in the future in case the plugin needs to be updated
   easy_script_file_name = "update_ikfast_plugin.sh"
   easy_script_file_path = plugin_pkg_dir + "/" + easy_script_file_name
   with open(easy_script_file_path,'w') as f:
      f.write("rosrun moveit_kinematics create_ikfast_moveit_plugin.py"
              + " " + robot_name
              + " " + planning_group_name
              + " " + plugin_pkg
              + " " + solver_file_name )

   print '\nCreated update plugin script at '+easy_script_file_path
