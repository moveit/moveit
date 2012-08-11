# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Ioan Sucan

import roslib
roslib.load_manifest('move_group_interface')
from move_group import MoveGroup
import re
import time

class MoveGroupInfoLevel:
    FAIL = 1
    WARN = 2
    SUCCESS = 3
    INFO = 4
    DEBUG = 5

class MoveGroupCommander:
    """
    Execution of simple commands
    """

    DEFAULT_FILENAME = "move_group.cfg"
    GO_DIRS = {"up" : (2,1), "down" : (2, -1), "z" : (2, 1),
               "left" : (1, 1), "right" : (1, -1), "y" : (1, 1),
               "forward" : (0, 1), "backward" : (0, -1), "back" : (0, -1), "x" : (0, 1) }

    def __init__(self):
        self._gdict = {}
        self._group_name = ""

    def get_active_group(self):
        return self._group_name

    def execute(self, cmd):
        cmd = self.resolve_command_alias(cmd)

        if cmd.startswith("use"):
            if cmd == "use":
                return (MoveGroupInfoLevel.INFO, "\n".join(self._gdict.keys()))
            clist = cmd.split()
            if len(clist) == 2:
                if clist[0] == "use":
                    if self._gdict.has_key(clist[1]):
                        self._group_name = clist[1]
                        return (MoveGroupInfoLevel.DEBUG, "OK")
                    else:
                        try:
                            mg = MoveGroup(clist[1])
                            self._gdict[clist[1]] = mg
                            self._group_name = clist[1]
                            return (MoveGroupInfoLevel.DEBUG, "OK")
                        except:
                            return (MoveGroupInfoLevel.FAIL, "Unable to initialize " + clist[1])
        elif cmd.startswith("load"):
            filename = self.DEFAULT_FILENAME
            clist = cmd.split()
            if len(clist) > 2:
                return (MoveGroupInfoLevel.WARN, "Unable to parse load command")
            if len(clist) == 2:
                filename = clist[1]
            try:
                f = open(filename, 'r')
                for line in f:
                    self.execute(line)
                return (MoveGroupInfoLevel.DEBUG, "OK")
            except:
                return (MoveGroupInfoLevel.WARN, "Unable to load " + filename)
        elif cmd.startswith("save"):
            filename = self.DEFAULT_FILENAME
            clist = cmd.split()
            if len(clist) > 2:
                return (MoveGroupInfoLevel.WARN, "Unable to parse save command")
            if len(clist) == 2:
                filename = clist[1]
            try:
                f = open(filename, 'w')
                for gr in self._gdict.keys():
                    f.write("use " + gr + "\n")
                    known = self._gdict[gr].get_remembered_joint_values()
                    for v in known.keys():
                        f.write(v + " = [" + " ".join([str(x) for x in known[v]]) + "]\n")
                return (MoveGroupInfoLevel.DEBUG, "OK")
            except:
                return (MoveGroupInfoLevel.WARN, "Unable to save " + filename)
        else:
            if len(self._group_name) > 0:
                return self.execute_command(self._gdict[self._group_name], cmd)
            else:
                return (MoveGroupInfoLevel.INFO, self.get_help() + "\n\nNo groups initialized yet. You must call the 'use' or the 'load' command first.\n")

    def execute_command(self, g, cmd):
        
        if cmd == "stop":
            g.stop()
            return (MoveGroupInfoLevel.DEBUG, "OK")

        if cmd == "id":
            return (MoveGroupInfoLevel.INFO, g.get_name())

        if cmd == "help":
            return (MoveGroupInfoLevel.INFO, self.get_help())

        if cmd == "vars":
            known = g.get_remembered_joint_values()
            return (MoveGroupInfoLevel.INFO, "[" + " ".join(known.keys()) + "]")

        if cmd == "show":
            return self.command_show(g)

        if cmd == "current":
            return self.command_current(g)

        if cmd == "eef":
            if len(g.get_end_effector_link()) > 0:
                return (MoveGroupInfoLevel.INFO, g.get_end_effector_link())
            else:
                return (MoveGroupInfoLevel.INFO, "There is no end effector defined")

        # see if we have assignment between variables
        assign_match = re.match(r"^(\w+)\s*=\s*(\w+)$", cmd)
        if assign_match:
            known = g.get_remembered_joint_values()
            if known.has_key(assign_match.group(2)):
                g.remember_joint_values(assign_match.group(1), known[assign_match.group(2)])
                return (MoveGroupInfoLevel.SUCCESS, assign_match.group(1) + " is now the same as " + assign_match.group(2))
            else:
                return (MoveGroupInfoLevel.WARN, "Unknown command: '" + cmd + "'")
        
        # see if we have assignment of matlab-like vector syntax
        set_match = re.match(r"^(\w+)\s*=\s*\[([\d\.e\-\+\s]+)\]$", cmd)
        if set_match:
            try:
                g.remember_joint_values(set_match.group(1), [float(x) for x in set_match.group(2).split()])
                return (MoveGroupInfoLevel.SUCCESS, "Remembered joint values [" + set_match.group(2) + "] under the name " + set_match.group(1))
            except:
                return (MoveGroupInfoLevel.WARN, "Unable to parse joint value [" + set_match.group(2) + "]")

        # see if we have assignment of matlab-like element update
        component_match = re.match(r"^(\w+)\s*\[\s*(\d+)\s*\]\s*=\s*([\d\.e\-\+]+)$", cmd)
        if component_match:
            known = g.get_remembered_joint_values()
            if known.has_key(component_match.group(1)):
                try:
                    val = known[component_match.group(1)]
                    val[int(component_match.group(2))] = float(component_match.group(3))
                    g.remember_joint_values(component_match.group(1), val)
                    return (MoveGroupInfoLevel.SUCCESS, "Updated " + component_match.group(1) + "[" + component_match.group(2) + "]")
                except:
                    return (MoveGroupInfoLevel.WARN, "Unable to parse index or value in '" + cmd +"'")
            else:
                return (MoveGroupInfoLevel.WARN, "Unknown command: '" + cmd + "'")

        clist = cmd.split()

        # if this is an unknown one-word command, it is probably a variable
        if len(clist) == 1:
            known = g.get_remembered_joint_values()
            if known.has_key(cmd):
                return (MoveGroupInfoLevel.INFO, "[" + " ".join([str(x) for x in known[cmd]]) + "]")
            else:
                return (MoveGroupInfoLevel.WARN, "Unknown command: '" + cmd + "'")

        # command with one argument
        if len(clist) == 2:
            if clist[0] == "go":
                if clist[1] == "rand" or clist[1] == "random":
                    g.set_random_target()
                    if g.move():
                        return (MoveGroupInfoLevel.SUCCESS, "Moved to random target")
                    else:
                        return (MoveGroupInfoLevel.FAIL, "Failed while moving to random target")
                else:
                    if g.set_named_target(clist[1]):
                        if g.move():
                            return (MoveGroupInfoLevel.SUCCESS, "Moved to " + clist[1])
                        else:
                            return (MoveGroupInfoLevel.FAIL, "Failed while moving to " + clist[1])
                    else:
                        return (MoveGroupInfoLevel.WARN, clist[1] + " is unknown")
            elif clist[0] == "record" or clist[0] == "rec":
                g.remember_joint_values(clist[1])
                return (MoveGroupInfoLevel.SUCCESS, "Remembered current joint values under the name " + clist[1])
            elif clist[0] == "rand" or clist[0] == "random":
                g.remember_joint_values(clist[1], g.get_random_joint_values())
                return (MoveGroupInfoLevel.SUCCESS, "Remembered random joint values under the name " + clist[1])
            elif clist[0] == "del" or clist[0] == "delete":
                g.forget_joint_values(clist[1])    
                return (MoveGroupInfoLevel.SUCCESS, "Forgot joint values under the name " + clist[1])
            elif clist[0] == "show":
                known = g.get_remembered_joint_values()
                if known.has_key(clist[1]):
                    return (MoveGroupInfoLevel.INFO, "[" + " ".join([str(x) for x in known[clist[1]]]) + "]")
                else:
                    return (MoveGroupInfoLevel.WARN, "Joint values for " + clist[1] + " are not known")
            elif clist[0] == "wait":
                try:
                    time.sleep(float(clist[1]))
                    return (MoveGroupInfoLevel.SUCCESS, clist[1] + " seconds passed")
                except:
                    return (MoveGroupInfoLevel.WARN, "Unable to wait '" + clist[1] + "' seconds")
            else:
                return (MoveGroupInfoLevel.WARN, "Unknown command: '" + cmd + "'")

        if len(clist) == 3:
            if clist[0] == "go" and self.GO_DIRS.has_key(clist[1]):     
                try:
                    offset = float(clist[2])
                    (axis, factor) = self.GO_DIRS[clist[1]]
                    return self.command_go_offset(g, offset, factor, axis, clist[1])
                except:
                    return (MoveGroupInfoLevel.WARN, "Unable to parse distance '" + clist[2] + "'")
 
        if len(clist) == 4:
            if clist[0] == "rotate":
                try:
                    xyz = [float(x) for x in clist[1:]]
                    g.set_orientation_target(xyz[0], xyz[1], xyz[2])
                    if g.move():
                        return (MoveGroupInfoLevel.SUCCESS, "Rotation complete")
                    else:
                        return (MoveGroupInfoLevel.FAIL, "Failed while rotating to " + " ".join(clist[1:]))
                except:
                    return (MoveGroupInfoLevel.WARN, "Unable to parse X-Y-Z rotation  values '" + " ".join(clist[1:]) + "'")

        return (MoveGroupInfoLevel.WARN, "Unknown command: '" + cmd + "'")

    def command_show(self, g):
        known = g.get_remembered_joint_values()
        res = []
        for k in known.keys():
            res.append(k + " = [" + " ".join([str(x) for x in known[k]]) + "]")
        return (MoveGroupInfoLevel.INFO, "\n".join(res))
        
    def command_current(self, g):
        res = "joints = [" + " ".join([str(x) for x in g.get_current_joint_values()]) + "]"
        if len(g.get_end_effector_link()) > 0:
            res = res + "\n" + g.get_end_effector_link() + " pose = [" + " ".join([str(x) for x in g.get_current_pose()]) + "]"
        return (MoveGroupInfoLevel.INFO, res)

    def command_go_offset(self, g, offset, factor, dimension_index, direction_name):
        if len(g.get_end_effector_link()) > 0:
            pose = g.get_current_pose()
            pose[dimension_index] = pose[dimension_index] + offset * factor
            g.set_pose_target(pose)
            if g.move():
                return (MoveGroupInfoLevel.SUCCESS, "Moved " + direction_name + " by " + str(offset) + " m")
            else:
                return (MoveGroupInfoLevel.FAIL, "Failed while moving " + direction_name)
        else:
            return (MoveGroupInfoLevel.WARN, "No known end effector. Cannot move " + direction_name)

    def resolve_command_alias(self, cmd):
        if cmd == "which":
            cmd = "id"
        if cmd == "groups":
            cmd = "use"
        return cmd

    def get_help(self):
        res = []
        res.append("Known commands:")
        res.append("  help\t\t show this screen")
        res.append("  id|which\t display the name of the group that is operated on")
        res.append("  use <name>\t switch to using the group named <name> (and load it if necessary)")
        res.append("  use|groups\t show the group names that are already loaded")
        res.append("  vars\t\t display the names of the known states")
        res.append("  show\t\t display the names and values of the known states")
        res.append("  show <name>\t display the value of a state")
        res.append("  record <name>\t record the current joint values under the name <name>")
        res.append("  delete <name>\t forget the joint values under the name <name>")
        res.append("  current\t show the current state of the active group")
        res.append("  go <name>\t plan and execute a motion to the state <name>")
        res.append("  go rand\t plan and execute a motion to a random state")
        res.append("  wait <dt>\t sleep for <dt> seconds")
        res.append("  x = y\t\t assign the value of y to x")
        res.append("  x[idx] = val\t assign a value to dimension idx of x")
        res.append("  x = [v1 v2...] assign a vector of values to x")
        return "\n".join(res)

