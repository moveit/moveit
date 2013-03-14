#!/usr/bin/env python

import roslib
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
        self.options = options

    def set_options(self, options):
        self.options = options

    def complete(self, text, state):
        response = None
        cmds = readline.get_line_buffer().split()
        prefix = ""
        if len(cmds) > 0:
            prefix = cmds[0]
            if not self.options.has_key(prefix):
                prefix = ""

        if state == 0:
            # This is the first time for this text, so build a match list.
            if text:
                if len(prefix) == 0:
                    self.matches = sorted([s 
                                           for s in self.options.keys()
                                           if s and s.startswith(text)])
                else:
                    self.matches = sorted([s 
                                           for s in self.options[prefix]
                                           if s and s.startswith(text)])
            else:
                if len(prefix) == 0:
                    self.matches = sorted(self.options.keys())
                else:
                    self.matches = self.options[prefix]
        
        # Return the state'th item from the match list,
        # if we have that many.
        try:
            response = self.matches[state] + " "
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

def get_context_keywords(interpreter):
    kw = interpreter.get_keywords()            
    kw["quit"] = []
    return kw

def run_interactive(group_names):
    c = MoveGroupCommandInterpreter()
    for g in group_names:
        c.execute( "use " + g)
    completer = SimpleCompleter(get_context_keywords(c))
    readline.set_completer(completer.complete)

    print
    print bcolors.HEADER + "Waiting for commands. Type 'help' to get a list of known commands." + bcolors.ENDC
    print
    readline.parse_and_bind('tab: complete')

    while not rospy.is_shutdown():
        cmd = ""
        try:  
            name = ""
            ag = c.get_active_group()
            if ag != None:
                name = ag.get_name()
            cmd = raw_input(bcolors.OKBLUE + name + '> ' + bcolors.ENDC)
        except:
            break
        cmd = cmd.strip()
        if cmd == "":
            continue
        cmd = cmd.lower()

        if cmd == "q" or cmd == "quit" or cmd == "exit":
            break
        if cmd == "host":
            print_message(MoveGroupInfoLevel.INFO, "Master is '" + os.environ['ROS_MASTER_URI'] + "'")
            continue

        (level, msg) = c.execute(cmd)
        print_message(level, msg)
        # update the set of keywords
        completer.set_options(get_context_keywords(c))
            
def run_service(group_names): 
    c = MoveGroupCommandInterpreter()
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
