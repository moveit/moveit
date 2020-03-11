/*
 * Copyright (c) 2019 Pilz GmbH & Co. KG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef XML_CONSTANTS_H
#define XML_CONSTANTS_H

#include <string>

namespace pilz_industrial_motion_planner_testutils
{
const std::string EMPTY_STR{};

const std::string XML_ATTR_STR{ "<xmlattr>" };
const std::string JOINT_STR{ "joints" };
const std::string POSE_STR{ "pos" };
const std::string XYZ_QUAT_STR{ "xyzQuat" };
const std::string XYZ_EULER_STR{ "xyzEuler" };
const std::string SEED_STR{ "seed" };

const std::string PTP_STR{ "ptp" };
const std::string LIN_STR{ "lin" };
const std::string CIRC_STR{ "circ" };
const std::string BLEND_STR{ "blend" };
const std::string GRIPPER_STR{ "gripper" };

const std::string PLANNING_GROUP_STR{ "planningGroup" };
const std::string TARGET_LINK_STR{ "targetLink" };
const std::string START_POS_STR{ "startPos" };
const std::string END_POS_STR{ "endPos" };
const std::string INTERMEDIATE_POS_STR{ "intermediatePos" };
const std::string CENTER_POS_STR{ "centerPos" };
const std::string VEL_STR{ "vel" };
const std::string ACC_STR{ "acc" };

const std::string POSES_PATH_STR{ "testdata.poses" };
const std::string PTPS_PATH_STR{ "testdata." + PTP_STR + "s" };
const std::string LINS_PATH_STR{ "testdata." + LIN_STR + "s" };
const std::string CIRCS_PATH_STR{ "testdata." + CIRC_STR + "s" };
const std::string SEQUENCE_PATH_STR{ "testdata.sequences" };
const std::string GRIPPERS_PATH_STR{ "testdata." + GRIPPER_STR + "s" };

const std::string NAME_PATH_STR{ XML_ATTR_STR + ".name" };
const std::string CMD_TYPE_PATH_STR{ XML_ATTR_STR + ".type" };
const std::string BLEND_RADIUS_PATH_STR{ XML_ATTR_STR + ".blend_radius" };
const std::string LINK_NAME_PATH_STR{ XML_ATTR_STR + ".link_name" };
const std::string GROUP_NAME_PATH_STR{ XML_ATTR_STR + ".group_name" };
}

#endif  // XML_CONSTANTS_H
