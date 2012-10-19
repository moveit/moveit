#! /usr/bin/env python

import sys
import yaml
import roslib
import re
import os
from lxml import etree

if __name__ == '__main__':
    # Expect robot name as argument
    if(len(sys.argv) != 2):
        print("Usage: create_ik_fast_plugin.py <your_robot_name>")
        sys.exit(-1)

    robot_name = sys.argv[1]
    directory_name = roslib.packages.get_pkg_dir(robot_name + "_arm_navigation")
    
    print directory_name

    #if(not roslib.packages.is_pkg_dir(directory_name)):
    #    print(robot_name + " package does not exist ")
    #    sys.exit(-2)

    yaml_file_name = directory_name + '/config/' + robot_name + '_planning_description.yaml'
            
    # Get dictionary of yaml defs
    data = yaml.safe_load(open(yaml_file_name, 'r'))

    # List of groups
    groups = data['groups']

    if(len(groups) < 1) :
        print("Need at least 1 planning group in the robot to continue!")
        sys.exit(-1)

    #first we write the template
    arm_kinematics_directory = roslib.packages.get_pkg_dir("moveit_setup_assistant")

    template_file_name = arm_kinematics_directory+"/templates/ik_fast_plugin_template.cxx"

    for i in groups:
        if 'tip_link' in i:
            template_file_data = open(template_file_name, 'r')
            group_name = i['name']
            template_text = template_file_data.read()
            template_text = re.sub('_ROBOT_NAME_', robot_name, template_text)
            template_text = re.sub('_GROUP_NAME_', group_name, template_text)
            
            if not os.path.exists(directory_name+'/src'):
                os.makedirs(directory_name+'/src')
                
            print 'Writing to ', directory_name+'/src/'+robot_name+"_"+group_name+"_ikfast_plugin.cpp"

            output_template = open(directory_name+'/src/'+robot_name+"_"+group_name+"_ikfast_plugin.cpp",'w')
            output_template.write(template_text)
                       
    #now the plugin definition

    plugin_file = open(directory_name+"/kinematics_plugins.xml",'w')

    root_one = etree.Element("library", path="lib/lib"+robot_name+"_kinematics_lib")

    for i in groups:
        if 'tip_link' in i:
            group_name = i['name']
            cl = etree.SubElement(root_one, "class")
            cl.set("name", robot_name+'_'+group_name+'_kinematics/IKFastKinematicsPlugin')
            cl.set("type", robot_name+'_'+group_name+'_kinematics::IKFastKinematicsPlugin')
            cl.set("base_class_type", "kinematics::KinematicsBase")
            desc = etree.SubElement(cl, "description")
            desc.text = "A plugin created by using OpenRAVE's IK Fast component"

    etree.ElementTree(root_one).write(plugin_file, xml_declaration=True, pretty_print=True)

    #now we try to mess with the CMakelists

    cmake_file = open(directory_name+"/CMakeLists.txt", 'a+')
    cmake_file_text = cmake_file.read()
    if re.search(robot_name+'_kinematics_lib', cmake_file_text) == None:
        extra_text = '\n#Library containing ikfast plugins\nrosbuild_add_library('+robot_name+'_kinematics_lib\n'
        for i in groups:
            if 'tip_link' in i:
                group_name = i['name']
                extra_text += '  src/'+robot_name+"_"+group_name+'_ikfast_plugin.cpp\n'
    
        extra_text += "  )"     
        
        cmake_file.write(extra_text)
    
    cmake_file.close()

    parser = etree.XMLParser(remove_blank_text=True)

    #now we add the plugin export to the manifest
    manifest_xml = etree.parse(directory_name+"/manifest.xml", parser)

    #first making sure that kinematics_base is in the depends list
    found = False
    for depend_entry in manifest_xml.getroot().findall("depend"):
        if depend_entry.get("package") == "kinematics_base":
            found = True
            break

    if not found:
        for idx, entry in enumerate(manifest_xml.getroot().getchildren()):
            if entry.tag == "depend":
                manifest_xml.getroot().insert(idx, etree.Element("depend", package="kinematics_base"))
                break
    
    export_element = manifest_xml.getroot().find("export")
    if export_element == None:
        export_element = etree.SubElement(manifest_xml.getroot(), "export")

    kinematics_element = export_element.find("kinematics_base")
    if kinematics_element == None:
        kinematics_element = etree.SubElement(export_element, "kinematics_base", plugin="${prefix}/kinematics_plugins.xml")

    manifest_xml_file = open(directory_name+"/manifest.xml", "w")
    manifest_xml.write(manifest_xml_file, xml_declaration=True, pretty_print=True)

    #Finally, we change the constraint_aware kinematics launch file

    constraint_aware_xml = etree.parse(directory_name+"/launch/constraint_aware_kinematics.launch", parser)
    
    for node_entry in constraint_aware_xml.getroot().findall("node"):
        for param_entry in node_entry.getchildren():
            if param_entry.get("name") == "group":
                group_name = param_entry.get("value")
        for param_entry in node_entry.getchildren():
            if param_entry.get("name") == "kinematics_solver":
                param_entry.set("value", robot_name+"_"+group_name+"_kinematics/IKFastKinematicsPlugin")

    constraint_aware_xml_file = open(directory_name+"/launch/constraint_aware_kinematics.launch", "w")
    constraint_aware_xml.write(constraint_aware_xml_file, xml_declaration=True, pretty_print=True)
