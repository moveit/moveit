#! /usr/bin/env python
'''
IKFast Plugin Generator for MoveIt!

Creates a kinematics plugin using the output of IKFast from OpenRAVE.
This plugin and the move_group node can be used as a general
kinematics service, from within the moveit planning environment, or in
your own ROS node.

Author: Michael Lautman, PickNik Robotics
        Dave Coleman PickNik Robotics

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
import argparse

plugin_gen_pkg = 'moveit_kinematics'  # package containing this file
plugin_sub_dir = 'ikfast_kinematics_plugin' # sub directory which contains the template directory
# Allowed search modes, see SEARCH_MODE enum in template file
search_modes = ['OPTIMIZE_MAX_JOINT', 'OPTIMIZE_FREE_JOINT' ]


def create_parser():
  parser = argparse.ArgumentParser(description="Generate an IKFast MoveIt! kinematic plugin")
  parser.add_argument("robot_name", help="The name of your robot")
  parser.add_argument("planning_group_name", help="The name of the planning group for which your IKFast solution was generated")
  parser.add_argument("kinematic_plugin_pkg", help="The name of the MoveIt! IKFast Kinematics Plugin to be updated")
  parser.add_argument("base_link_name", help="The name of the base link that was used when generating your IKFast solution")
  parser.add_argument("eef_link_name", help="The name of the end effector link that was used when generating your IKFast solution")
  parser.add_argument("ikfast_output_path", help="The full path to the analytic IK solution output by IKFast")
  parser.add_argument("--search_mode", default=search_modes[0], help="The search mode used to solve IK for robots with more than 6DOF")
  parser.add_argument("--srdf_filename", help="The name of your robot. If your robot's srdf is not <robot_name>.srdf, then set this value with the correct filename. Defaults to <robot_name>.srdf")
  parser.add_argument("--robot_name_in_srdf", help="The name of your robot as defined in the srdf. Defaults to <robot_name>")
  parser.add_argument("--moveit_config_pkg", help="The robot moveit_config pagkacge. Defaults to <robot_name>_moveit_config")
  return parser

def populate_optional(args):
  if args.srdf_filename is None:
    args.srdf_filename = args.robot_name + ".srdf"
  if args.robot_name_in_srdf is None:
    args.robot_name_in_srdf = args.robot_name
  if args.moveit_config_pkg is None:
    args.moveit_config_pkg = args.robot_name + "_moveit_config"

def print_args(args):
  print "robot_name:", args.robot_name
  print "base_link_name:", args.base_link_name
  print "eef_link_name:", args.eef_link_name
  print "planning_group_name:", args.planning_group_name
  print "kinematic_plugin_pkg:", args.kinematic_plugin_pkg
  print "ikfast_output_path:", args.ikfast_output_path
  print "search_mode:", args.search_mode
  print "srdf_filename:", args.srdf_filename
  print "robot_name_in_srdf:", args.robot_name_in_srdf
  print "moveit_config_pkg:", args.moveit_config_pkg

def update_deps(reqd_deps, req_type, e_parent):
  curr_deps = [e.text for e in e_parent.findall(req_type)]
  missing_deps = set(reqd_deps) - set(curr_deps)
  for dep in missing_deps:
     etree.SubElement(e_parent, req_type).text = dep

def update_package(args):
  try:
    moveit_config_pkg_path = roslib.packages.get_pkg_dir(args.moveit_config_pkg)
  except:
    print "Failed to find package: %s" % args.moveit_config_pkg
    sys.exit(-1)

  try:
    kinematic_plugin_pkg_path = roslib.packages.get_pkg_dir(args.kinematic_plugin_pkg)
  except:
    print "Failed to find package: %s" % args.kinematic_plugin_pkg
    sys.exit(-1)

  try:
    srdf_file_name = moveit_config_pkg_path + '/config/' + args.srdf_filename
    srdf = etree.parse(srdf_file_name).getroot()
  except:
    print "Failed to find srdf file: %s" % srdf_file_name
    sys.exit(-1)

  if args.robot_name_in_srdf != srdf.get('name'):
    print "\nERROR: non-matching robot name found in '%s'. Expected '%s' but found '%s'" % (srdf_file_name,
                                                                                            args.robot_name_in_srdf,
                                                                                            srdf.get('name'))
    sys.exit(-1)

  groups = srdf.findall('group')
  if len(groups) < 1:
    print "Failed to load planning group since no planning groups are defined in the SRDF"
    sys.exit(-1)

  planning_group = None
  for group in groups:
    if group.get('name').lower() == args.planning_group_name.lower():
      planning_group = group

  if planning_group is None:
    print "Failed to load planning group '%s' since it is not defined in the SRDF. "\
          "Available groups: \n%s" % (args.planning_group_name,
                                      "".join([group_name.get('name') for group_name in groups]))

  kinematic_plugin_pkg_src_path = kinematic_plugin_pkg_path + '/src/'
  if not os.path.exists(kinematic_plugin_pkg_src_path):
     os.makedirs(kinematic_plugin_pkg_src_path)

  kinematic_plugin_pkg_include_path = kinematic_plugin_pkg_path + '/include/'
  if not os.path.exists(kinematic_plugin_pkg_include_path):
     os.makedirs(kinematic_plugin_pkg_include_path)


  if not os.path.exists(args.ikfast_output_path):
    print "\nERROR: can't find IKFast source code at '%s'\n" % args.ikfast_output_path
    sys.exit(-1)

  # Copy the source code generated by IKFast into our src folder
  solver_file_path = kinematic_plugin_pkg_src_path + args.robot_name + '_' + args.planning_group_name + '_ikfast_solver.cpp'

  # Check if they are the same file - if so, skip
  skip = False
  if os.path.exists(args.ikfast_output_path) and os.path.exists(solver_file_path):
    if os.path.samefile(args.ikfast_output_path, solver_file_path):
       print 'Skipping copying ' + solver_file_path + ' since it is already in the correct location'
       skip = True

  if not skip:
    shutil.copy2(args.ikfast_output_path, solver_file_path)

  # Error check
  if not os.path.exists(solver_file_path):
    print "\nERROR: Unable to copy IKFast source code from '%s' to '%s'" % (args.ikfast_output_path, solver_file_path)
    print 'Manually copy the source file generated by IKFast to this location and re-run'
    sys.exit(-1)

  # Detect version of IKFast used to generate solver code
  solver_version = 0
  with open(solver_file_path,'r') as src:
    for line in src:
       if line.startswith('/// ikfast version'):
          line_search = re.search('ikfast version (.*) generated', line)
          if line_search:
             solver_version = int(line_search.group(1), 0)
          break
  print 'Found source code generated by IKFast version %s' % str(solver_version)

  # Chose template depending on IKFast version
  if solver_version >= 56:
    template_version = 61
  else:
    print 'ERROR this converter is not made for IKFast 54 or anything but 61'
    sys.exit(-1)

  # Get template folder location
  try:
    plugin_gen_dir = os.path.join(roslib.packages.get_pkg_dir(plugin_gen_pkg), plugin_sub_dir)
  except:
    print "ERROR: can't find package " + plugin_gen_pkg
    sys.exit(-1)

  # Check if IKFast header file exists
  template_header_path = plugin_gen_dir + '/templates/ikfast.h'
  if not os.path.exists(template_header_path):
    print "ERROR: can't find ikfast header file at '%s'" % template_header_path
    sys.exit(-1)

  # Copy the IKFast header file into the include directory
  output_header_path = kinematic_plugin_pkg_include_path + "ikfast.h"
  shutil.copy2(template_header_path, output_header_path)
  if not os.path.exists(output_header_path):
    print "ERROR: Unable to copy IKFast header file from '%s' to '%s'" % (template_header_path, output_header_path)
    sys.exit(-1)

  # Check if template exists
  template_src_path = plugin_gen_dir + '/templates/ikfast' + str(template_version) + '_moveit_plugin_template.cpp'
  if not os.path.exists(template_src_path):
    print "ERROR: can't find template file at '%s' " % template_src_path
    sys.exit(-1)

  # Create plugin source from template
  template_file_data = open(template_src_path, 'r')
  template_text = template_file_data.read()
  template_text = re.sub('_ROBOT_NAME_', args.robot_name, template_text)
  template_text = re.sub('_GROUP_NAME_', args.planning_group_name, template_text)
  template_text = re.sub('_SEARCH_MODE_', args.search_mode, template_text)
  template_text = re.sub('_EEF_LINK_', args.eef_link_name, template_text)
  template_text = re.sub('_BASE_LINK_', args.base_link_name, template_text)

  output_src_path = kinematic_plugin_pkg_src_path + args.robot_name + '_' + args.planning_group_name + "_ikfast_moveit_plugin.cpp"
  with open(output_src_path, 'w') as f:
    f.write(template_text)
  print "Created plugin file at '%s'" % output_src_path

  # Create plugin definition .xml file
  ik_library_name = args.robot_name + "_" + args.planning_group_name + "_moveit_ikfast_plugin"
  plugin_name = args.robot_name + '_' + args.planning_group_name + '_kinematics/IKFastKinematicsPlugin'
  plugin_def = etree.Element("library", path="lib/lib"+ik_library_name)
  cl = etree.SubElement(plugin_def, "class")
  cl.set("name", plugin_name)
  cl.set("type", 'ikfast_kinematics_plugin::IKFastKinematicsPlugin')
  cl.set("base_class_type", "kinematics::KinematicsBase")
  desc = etree.SubElement(cl, "description")
  desc.text = 'IKFast'+str(template_version)+' plugin for closed-form kinematics'

  # Write plugin definition to file
  plugin_file_name = ik_library_name + "_description.xml"
  plugin_file_path = kinematic_plugin_pkg_path + "/" + plugin_file_name
  with open(plugin_file_path,'w') as f:
    etree.ElementTree(plugin_def).write(f, xml_declaration=True, pretty_print=True, encoding="UTF-8")
  print "Created plugin definition at: '%s'" % plugin_file_path

  # Check if CMakeLists file exists
  cmake_template_file = plugin_gen_dir + "/templates/CMakeLists.txt"
  if not os.path.exists(cmake_template_file):
    print "ERROR: can't find CMakeLists template file at '%s'" % cmake_template_file
    sys.exit(-1)

  # Create new CMakeLists file
  cmake_file = kinematic_plugin_pkg_path+'/CMakeLists.txt'

  # Create plugin source from template
  template_file_data = open(cmake_template_file, 'r')
  template_text = template_file_data.read()
  template_text = re.sub('_ROBOT_NAME_', args.robot_name, template_text)
  template_text = re.sub('_GROUP_NAME_', args.planning_group_name, template_text)
  template_text = re.sub('_PACKAGE_NAME_', args.kinematic_plugin_pkg, template_text)
  template_text = re.sub('_LIBRARY_NAME_', ik_library_name, template_text)

  with open(cmake_file, 'w') as f:
    f.write(template_text)
  print "Overwrote CMakeLists file at '%s'" % cmake_file

  # Add plugin export to package manifest
  parser = etree.XMLParser(remove_blank_text=True)
  package_file_name = kinematic_plugin_pkg_path+"/package.xml"
  package_xml = etree.parse(package_file_name, parser)

  # Make sure at least all required dependencies are in the depends lists
  build_deps = ["liblapack-dev", "moveit_core", "pluginlib", "roscpp", "tf_conversions", "moveit_ros_planning"]
  run_deps   = ["liblapack-dev", "moveit_core", "pluginlib", "roscpp", "tf_conversions", "moveit_ros_planning"]

  update_deps(build_deps, "build_depend", package_xml.getroot())
  update_deps(run_deps, "exec_depend", package_xml.getroot())

  # Check that plugin definition file is in the export list
  new_export = etree.Element("moveit_core",
                             plugin="${prefix}/" + plugin_file_name)

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
  with open(package_file_name, "w") as f:
    package_xml.write(f, xml_declaration=True, pretty_print=True, encoding="UTF-8")
  print "Modified package.xml at '%s'" % package_file_name

  # Modify kinematics.yaml file
  kin_yaml_file_name = moveit_config_pkg_path+"/config/kinematics.yaml"
  with open(kin_yaml_file_name, 'r') as f:
    kin_yaml_data = yaml.safe_load(f)
  kin_yaml_data[args.planning_group_name]["kinematics_solver"] = plugin_name
  with open(kin_yaml_file_name, 'w') as f:
    yaml.dump(kin_yaml_data, f, default_flow_style=False)
  print "Modified kinematics.yaml at '%s'" % kin_yaml_file_name

  # Create a script for easily updating the plugin in the future in case the plugin needs to be updated
  easy_script_file_name = "update_ikfast_plugin.sh"
  easy_script_file_path = kinematic_plugin_pkg_path + "/" + easy_script_file_name
  with open(easy_script_file_path,'w') as f:
    f.write("rosrun moveit_kinematics create_ikfast_moveit_plugin.py"
            + " --search_mode=" + args.search_mode
            + " --srdf_filename=" + args.srdf_filename
            + " --robot_name_in_srdf=" + args.robot_name_in_srdf
            + " --moveit_config_pkg=" + args.moveit_config_pkg
            + " " + args.robot_name
            + " " + args.planning_group_name
            + " " + args.kinematic_plugin_pkg
            + " " + args.base_link_name
            + " " + args.eef_link_name
            + " " + solver_file_path )

  print "Created update plugin script at '%s'" % easy_script_file_path

def main(args):
  populate_optional(args)
  print_args(args)
  update_package(args)

if __name__ == '__main__':
  parser = create_parser()
  args = parser.parse_args()
  main(args)