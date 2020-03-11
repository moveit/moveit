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

#ifndef COMMAND_TYPES_TYPEDEF_H
#define COMMAND_TYPES_TYPEDEF_H

#include <boost/variant.hpp>

#include "ptp.h"
#include "lin.h"
#include "circ.h"
#include "gripper.h"
#include "jointconfiguration.h"
#include "cartesianconfiguration.h"
#include "circ_auxiliary_types.h"

namespace pilz_industrial_motion_planner_testutils
{
typedef Ptp<JointConfiguration, JointConfiguration> PtpJoint;
typedef Ptp<JointConfiguration, CartesianConfiguration> PtpJointCart;
typedef Ptp<CartesianConfiguration, CartesianConfiguration> PtpCart;

typedef Lin<JointConfiguration, JointConfiguration> LinJoint;
typedef Lin<JointConfiguration, CartesianConfiguration> LinJointCart;
typedef Lin<CartesianConfiguration, CartesianConfiguration> LinCart;

typedef Circ<CartesianConfiguration, CartesianCenter, CartesianConfiguration> CircCenterCart;
typedef Circ<CartesianConfiguration, CartesianInterim, CartesianConfiguration> CircInterimCart;

typedef Circ<JointConfiguration, CartesianCenter, JointConfiguration> CircJointCenterCart;
typedef Circ<JointConfiguration, CartesianInterim, JointConfiguration> CircJointInterimCart;

typedef boost::variant<PtpJoint, PtpJointCart, PtpCart, LinJoint, LinCart, CircCenterCart, CircInterimCart,
                       CircJointCenterCart, CircJointInterimCart, Gripper>
    CmdVariant;
}

#endif  // COMMAND_TYPES_TYPEDEF_H
