#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import os
import sys
import catkin_pkg
from catkin_pkg.packages import find_packages

def create_header(ros_ubuntu_dict):
  ros_distros =  sorted(ros_ubuntu_dict.keys())
  section_header = "## ROS Buildfarm\n"
  header="MoveIt! Package"
  header_lines = '-'*len(header)
  for ros in ros_distros:
      source = ' '.join([ros.capitalize(), "Source"])
      debian = ' '.join([ros.capitalize(), "Debian"])
      header = ' | '.join([header, source, debian])
      header_lines = ' | '.join([header_lines, '-'*len(source), '-'*len(debian)])
  return '\n'.join([section_header, header, header_lines])

def create_table_line_template(ros_ubuntu_dict):
    ros_distros =  sorted(ros_ubuntu_dict.keys())
    line = "{package_name}"
    for ros in ros_distros:
        ubuntu = ros_ubuntu_dict[ros]
        source_badge_url = ''.join(["http://build.ros.org/buildStatus/icon?job=",
                                    ros[0].upper(), "src_u", ubuntu[0].upper(),
                                    "__", "{package_name}", "__ubuntu_",
                                    ubuntu.lower(), "__source"])
        source_build_url = ''.join(["http://build.ros.org/view/", ros[0].upper(),
                                    "src_u", ubuntu[0].upper(), "/job/",
                                    ros[0].upper(), "src_u", ubuntu[0].upper(),
                                    "__", "{package_name}", "__ubuntu_",
                                    ubuntu.lower(), "__source/"])
        source_str = ''.join(["[![Build Status](", source_badge_url,
                             ")](", source_build_url,")"])
        debian_badge_url = ''.join(["http://build.ros.org/buildStatus/icon?job=",
                                    ros[0].upper(), "bin_u", ubuntu[0].upper(),
                                    "64__", "{package_name}", "__ubuntu_",
                                    ubuntu.lower(), "_amd64__binary"])
        debian_build_url = ''.join(["http://build.ros.org/view/", ros[0].upper(),
                                    "bin_u", ubuntu[0].upper(), "64/job/",
                                    ros[0].upper(), "bin_u", ubuntu[0].upper(),
                                    "64__", "{package_name}", "__ubuntu_",
                                    ubuntu.lower(), "_amd64__binary/"])
        debian_str = ''.join(["[![Build Status](", debian_badge_url,
                              ")](", debian_build_url,")"])
        line = ' | '.join([line, source_str, debian_str])
    return line

def create_moveit_buildfarm_table():
    """
    Creates MoveIt! buildfarm badge table
    """
    # Update the following dictionary with the appropriate ROS-Ubuntu
    # combinations for supported distribitions. For instance, in Noetic,
    # remove {"indigo":"trusty"} and add {"noetic":"fbuntu"} with "fbuntu"
    # being whatever the 20.04 distro is named
    supported_distro_ubuntu_dict = {"indigo":"trusty", "kinetic":"xenial", "melodic":"bionic"}

    all_packages = sorted([package.name for _, package in find_packages(os.getcwd()).items()])
    moveit_packages = list()
    other_packages = list()
    for package in all_packages:
        if package.startswith('moveit'):
            moveit_packages.append(package)
        else:
            other_packages.append(package)
    moveit_packages.extend(other_packages)

    line_template = create_table_line_template(supported_distro_ubuntu_dict)
    buildfarm_table = create_header(supported_distro_ubuntu_dict)
    for package in moveit_packages:
      buildfarm_table = '\n'.join([buildfarm_table, line_template.format(package_name=package)])
    print(buildfarm_table)

if __name__ == "__main__":
    sys.exit(create_moveit_buildfarm_table())
