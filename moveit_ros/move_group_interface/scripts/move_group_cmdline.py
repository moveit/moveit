#!/usr/bin/env python

import roslib
roslib.load_manifest('move_group_interface')
import rospy
import sys
import re

from move_group import *

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'

def print_help():
    print "Known commands:"
    print "  help\t\t show this screen"
    print "  record <name>\t record the current joint values under the name <name>"
    print

def run(group_name):
    g = MoveGroup(group_name)
    print
    print bcolors.HEADER + "Waiting for commands. Type help to get a list of known commands." + bcolors.ENDC
    print

    while not rospy.is_shutdown():
        sys.stdout.write(bcolors.OKBLUE + '> ' + bcolors.ENDC)
        cmd = ""
        try:
            cmd = sys.stdin.readline()
        except:
            print
            break
        if (cmd == "\n" or cmd == "\r" or cmd == "\r\n" or cmd == "\n\r"):
            continue
        cmd = cmd.strip()
        if cmd == "":
            break
        cmd = cmd.lower()
        if cmd == "q" or cmd == "quit" or cmd == "exit":
            break
        if cmd == "help":
            print_help()
            continue
        if cmd == "show":
            known = g.get_remembered_joint_values()
            for k in known.keys():
                print k + " = [" + " ".join([str(x) for x in known[k]]) + "]"
            continue
        if cmd == "id" or cmd == "which":
            print g.get_name()
            continue

        # provide command alias 
        if cmd == "rand" or cmd == "random":
            cmd = "go rand"

        # see if we have assignment between variables
        assign_match = re.match(r"^(\w+)\s*=\s*(\w+)$", cmd)
        if assign_match:
            known = g.get_remembered_joint_values()
            if known.has_key(assign_match.group(2)):
                g.remember_joint_values(assign_match.group(1), known[assign_match.group(2)])
            else:
                print bcolors.WARNING + "Unknown command: '" + cmd + "'" + bcolors.ENDC
            continue

        # see if we have assignment of matlab-like vector syntax
        set_match = re.match(r"^(\w+)\s*=\s*\[([\d\.e\-\+\s]+)\]$", cmd)
        if set_match:
            try:
                g.remember_joint_values(set_match.group(1), [float(x) for x in set_match.group(2).split()])
                print bcolors.OKGREEN + "Remembered joint values [" + set_match.group(2) + "] under the name " + set_match.group(1) + bcolors.ENDC
            except:
                print bcolors.WARNING + "Unable to parse joint value [" + set_match.group(2) + "]" + bcolors.ENDC
            continue

        # see if we have assignment of matlab-like element update
        component_match = re.match(r"^(\w+)\s*\[\s*(\d+)\s*\]\s*=\s*([\d\.e\-\+]+)$", cmd)
        if component_match:
            known = g.get_remembered_joint_values()
            if known.has_key(component_match.group(1)):
                try:
                    val = known[component_match.group(1)]
                    val[int(component_match.group(2))] = float(component_match.group(3))
                    g.remember_joint_values(component_match.group(1), val)
                except:
                    print bcolors.WARNING + "Unable to parse index or value in '" + cmd +"'" + bcolors.ENDC
            else:
                print bcolors.WARNING + "Unknown command: '" + cmd + "'" + bcolors.ENDC
            continue

        clist = cmd.split()

        # if this is an unknown one-word command, it is probably a variable
        if len(clist) == 1:
            known = g.get_remembered_joint_values()
            if known.has_key(cmd):
                print known[cmd]
            else:
                print bcolors.WARNING + "Unknown command: '" + cmd + "'" + bcolors.ENDC
            continue

        # command with one argument
        if len(clist) == 2:
            if clist[0] == "go":
                if clist[1] == "rand" or clist[1] == "random":
                    g.set_random_target()
                    if g.move():
                        print bcolors.OKGREEN + "Moved to random target" + bcolors.ENDC
                    else:
                        print bcolors.WARNING + "Failed while moving to random target" + bcolors.ENDC
                else:
                    if g.set_named_target(clist[1]):
                        if g.move():
                            print bcolors.OKGREEN + "Moved to " + clist[1] + bcolors.ENDC
                        else:
                            print bcolors.FAIL + "Failed while moving to " + clist[1] + bcolors.ENDC
                    else:
                        print bcolors.WARNING + clist[1] + " is unknown" + bcolors.ENDC
            elif clist[0] == "record" or clist[0] == "rec":
                g.remember_joint_values(clist[1])
                print bcolors.OKGREEN + "Remembered current joint values under the name " + clist[1] + bcolors.ENDC
            elif clist[0] == "del" or clist[0] == "delete":
                g.forget_joint_values(clist[1])
            else:
                print bcolors.WARNING + "Unknown command: '" + cmd + "'" + bcolors.ENDC

        if len(clist) > 2:
            if clist[0] == "set":
                try:
                    g.remember_joint_values(clist[1], [float(x) for x in clist[2:]])
                    print bcolors.OKGREEN + "Remembered joint values [" + " ".join(clist[2:]) + "] under the name " + clist[1] + bcolors.ENDC
                except:
                    print bcolors.WARNING + "Unable to parse joint value [" + " ".join(clist[2:]) + "]" + bcolors.ENDC
            else:
                print bcolors.WARNING + "Unknown command: '" + cmd + "'" + bcolors.ENDC

if __name__=='__main__':
    rospy.init_node('move_group_interface_cmdline', anonymous=True)
    if len(sys.argv) != 2:
        print "Usage: %s <group_name>" % sys.argv[0]
    else:
        run(sys.argv[1])
        print "Bye bye!"
