#!/usr/bin/env python

import roslib
roslib.load_manifest('moveit_commander')
import rospy
import readline
import sys
import os


from optparse import OptionParser, OptionGroup
from moveit_commander import MoveGroupCommandInterpreter, MoveGroupInfoLevel

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'

class SimpleCompleter(object):
    
    def __init__(self, options):
        self.options = sorted(options)
        return

    def complete(self, text, state):
        response = None
        if state == 0:
            # This is the first time for this text, so build a match list.
            if text:
                self.matches = [s 
                                for s in self.options
                                if s and s.startswith(text)]
            else:
                self.matches = self.options[:]
        
        # Return the state'th item from the match list,
        # if we have that many.
        try:
            response = self.matches[state]
        except IndexError:
            response = None
        return response

def print_message(level, msg):
    if level == MoveGroupInfoLevel.FAIL:
        print bcolors.FAIL + msg + bcolors.ENDC
    elif level == MoveGroupInfoLevel.WARN:
        print bcolors.WARNING + msg + bcolors.ENDC
    elif level == MoveGroupInfoLevel.SUCCESS:
        print bcolors.OKGREEN + msg + bcolors.ENDC
    elif level == MoveGroupInfoLevel.DEBUG:
        print bcolors.OKBLUE + msg + bcolors.ENDC
    else:
        print msg

def run_interactive(group_names):
    c = MoveGroupCommandInterpreter()   
    readline.set_completer(SimpleCompleter(c.get_keywords()).complete)
    for g in group_names:
        c.execute( "use " + g)
    print
    print bcolors.HEADER + "Waiting for commands. Type 'help' to get a list of known commands." + bcolors.ENDC
    print
    readline.parse_and_bind('tab: complete')

    while not rospy.is_shutdown():
        cmd = ""
        try:  
            cmd = raw_input(bcolors.OKBLUE + c.get_active_group() + '> ' + bcolors.ENDC)
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
        if cmd == "host":
            print_message(MoveGroupInfoLevel.INFO, "Master is '" + os.environ['ROS_MASTER_URI'] + "'")
            continue

        (level, msg) = c.execute(cmd)
        print_message(level, msg)

def run_service(group_names): 
    c = MoveGroupCommmander()
    for g in group_names:
        c.execute("use " + g)
    # add service stuff
    print "Running ROS service"
    rospy.spin()

if __name__=='__main__':
    rospy.init_node('move_group_interface_cmdline', anonymous=True)

    usage = """%prog [options] <group_name_1> [<group_name_2> ...]"""
    parser = OptionParser(usage)
    parser.add_option("-i", "--interactive", action="store_true", dest="interactive", default=True,
                      help="Run the command processing script in interactive mode")
    parser.add_option("-s", "--service", action="store_true", dest="service", default=False,
                      help="Run the command processing script as a ROS service")
    (options, args) = parser.parse_args()

    if options.service:
        run_service(args)
    else:
        run_interactive(args)
    print "Bye bye!"
