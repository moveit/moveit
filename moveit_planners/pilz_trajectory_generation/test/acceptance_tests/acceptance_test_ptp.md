<!--
Copyright (c) 2018 Pilz GmbH & Co. KG

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
-->


# Acceptance Test PTP Motion using the MoveIt Motion Planning Plugin on the real robot
This test checks that the real robot system is able to perform a PTP Motion to a goal state given by the user. The test is performed using the moveit motion planning plugin.

## Prerequisites
  - Properly connect and startup the robot. Make sure a emergency stop is within reach.

## Test Sequence:
  1. Bringup can: `sudo ip link set can0 up type can bitrate 1000000`
  2. Run `roslaunch prbt_moveit_config moveit_planning_execution.launch sim:=False pipeline:=pilz_command_planner`
  3. The motion planning widget (lower left part of moveit) choose PTP in the dropdown below "Simple Command Planner" (see image)
![moveit_1](img/acceptance_test_ptp_img1.png)
  4. Switch to the tab "Planning" in the moveit planning plugin. Move the ball handle the select goal pose. Click on "plan and execute".
![moveit_2](img/acceptance_test_ptp_img2.png)

## Expected Results:
  1. Can should be visible with `ifconfig` displayed as can0
  2. A -click- indicates the enabling of the drives.
  3. PTP was available for selection
  4. The robot should move to the desired position. All axis should start and stop at the same time.
---
