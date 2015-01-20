/*
 * Copyright (C) 2013 CoDyCo
 * Author: Andrea Del Prete
 * email:  andrea.delprete@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef WHOLE_BODY_REACH_WBI_ID_LISTS
#define WHOLE_BODY_REACH_WBI_ID_LISTS

#include <wbi/wbiID.h>

namespace wholeBodyReach
{

    const static wbi::IDList ICUB_LEFT_ARM_JOINTS = wbi::IDList("l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw", "l_elbow", "l_wrist_pitch");
    const static wbi::IDList ICUB_RIGHT_ARM_JOINTS = wbi::IDList("r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw", "r_elbow", "r_wrist_pitch");
    const static wbi::IDList ICUB_TORSO_JOINTS = wbi::IDList("torso_pitch","torso_roll","torso_yaw");
    const static wbi::IDList ICUB_LEFT_LEG_JOINTS = wbi::IDList("l_hip_pitch","l_hip_roll","l_hip_yaw","l_knee","l_ankle_pitch","l_ankle_roll");
    const static wbi::IDList ICUB_RIGHT_LEG_JOINTS = wbi::IDList("r_hip_pitch","r_hip_roll","r_hip_yaw","r_knee","r_ankle_pitch","r_ankle_roll");
    const static wbi::IDList ICUB_MAIN_JOINTS = wbi::IDList(ICUB_TORSO_JOINTS,ICUB_LEFT_ARM_JOINTS,ICUB_RIGHT_ARM_JOINTS,ICUB_LEFT_LEG_JOINTS,ICUB_RIGHT_LEG_JOINTS);
}

#endif
