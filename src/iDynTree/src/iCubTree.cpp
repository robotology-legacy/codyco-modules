/*
 * Copyright (C) 2013 IIT - Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 *
 */

#include <iCub/iDynTree/iCubTree.h>
#include <iCub/iDynTree/idyn2kdl_icub.h>

#include <iCub/iDyn/iDynBody.h>

#include <iCub/skinDynLib/common.h>

using namespace iCub::skinDynLib;

namespace iCub {
namespace iDynTreeLib {

iCubTree::iCubTree(iCubTree_version_tag version, unsigned int verbose)
{
	//Allocate an old iCubWholeBody object, with right version
	iCub::iDyn::version_tag ver;
	ver.head_version = version.head_version;
	ver.legs_version = version.legs_version;
	
    iCub::iDyn::iCubWholeBody icub_idyn(ver);
    
    //Convert it to a KDL::Tree (this preserve all the frame of reference, is the conversion to URDF that changes them)
    KDL::Tree icub_kdl;
    bool ret = toKDL_iDynDebug(icub_idyn,icub_kdl);
    assert(ret);
    
    //Imu link name
    std::string imu_link_name = "imu_link";

	//Construct F/T sensor name list
	std::vector< std::string > ft_names(0);
	std::vector<KDL::Frame> child_sensor_transforms(0);
	KDL::Frame kdlFrame; 

	ft_names.push_back("l_arm_ft_sensor_joint");
	ft_names.push_back("r_arm_ft_sensor_joint");
	ft_names.push_back("l_leg_ft_sensor_joint");
	ft_names.push_back("r_leg_ft_sensor_joint");
	
	
	
	//Define an explicit serialization of the links and the DOFs of the iCub
	//Serialization obtained from current iDyn: left leg (6), right leg (6), torso (3), left arm (7), right arm (7), head (3)
	//The DOF serialization done in icub_kdl construction is ok
	KDL::CoDyCo::TreeSerialization serial = KDL::CoDyCo::TreeSerialization(icub_kdl);

	KDL::CoDyCo::TreePartition icub_partition = get_iCub_partition(serial);
	
	
	this->constructor(icub_kdl,ft_names,imu_link_name,serial,icub_partition);
	
}

KDL::CoDyCo::TreePartition iCubTree::get_iCub_partition(const KDL::CoDyCo::TreeSerialization & icub_serialization)
{
	//Define an explicit partition of the links and the DOFs of the iCub 
	//The parts are defined in http://wiki.icub.org/wiki/ICub_joints
	//The parts ID are instead definde in skinDynLib http://wiki.icub.org/iCub_documentation/common_8h_source.html
	KDL::CoDyCo::TreePart head(HEAD,BodyPart_s[HEAD]);
	head.addDOF(icub_serialization.getDOFId("neck_pitch_joint"));
	head.addDOF(icub_serialization.getDOFId("neck_roll_joint"));
	head.addDOF(icub_serialization.getDOFId("neck_yaw_joint"));
		
	head.addLink(icub_serialization.getLinkId("neck_pitch_link"));
	head.addLink(icub_serialization.getLinkId("neck_roll_link"));	
	head.addLink(icub_serialization.getLinkId("neck_yaw_link"));	
	head.addLink(icub_serialization.getLinkId("imu_link"));	
		
	KDL::CoDyCo::TreePart torso(TORSO,BodyPart_s[TORSO]);
	torso.addDOF(icub_serialization.getDOFId("torso_yaw_joint"));
	torso.addDOF(icub_serialization.getDOFId("torso_roll_joint"));
	torso.addDOF(icub_serialization.getDOFId("torso_pitch_joint"));

	torso.addLink(icub_serialization.getLinkId("root_link"));
	torso.addLink(icub_serialization.getLinkId("torso_yaw_link"));	
	torso.addLink(icub_serialization.getLinkId("torso_roll_link"));	
	torso.addLink(icub_serialization.getLinkId("torso_pitch_link"));
	torso.addLink(icub_serialization.getLinkId("torso"));	
	
	
	KDL::CoDyCo::TreePart left_arm(LEFT_ARM,BodyPart_s[LEFT_ARM]);
	left_arm.addDOF(icub_serialization.getDOFId("l_shoulder_pitch_joint"));
	left_arm.addDOF(icub_serialization.getDOFId("l_shoulder_roll_joint"));
	left_arm.addDOF(icub_serialization.getDOFId("l_shoulder_yaw_joint"));
	left_arm.addDOF(icub_serialization.getDOFId("l_elbow_joint"));
	left_arm.addDOF(icub_serialization.getDOFId("l_wrist_prosup_joint"));
	left_arm.addDOF(icub_serialization.getDOFId("l_wrist_pitch_joint"));
	left_arm.addDOF(icub_serialization.getDOFId("l_wrist_yaw_joint"));
    
    //The link serialization is done in a way to be compatible with skinDynLib 
    //(so the the upper part of the forerarm is shifted at the end)
	left_arm.addLink(icub_serialization.getLinkId("l_shoulder_pitch_link"));
	left_arm.addLink(icub_serialization.getLinkId("l_shoulder_roll_link"));
	left_arm.addLink(icub_serialization.getLinkId("l_arm_ft_sensor_link"));
	left_arm.addLink(icub_serialization.getLinkId("l_elbow_link"));
	left_arm.addLink(icub_serialization.getLinkId("l_wrist_prosup_link"));
	left_arm.addLink(icub_serialization.getLinkId("l_wrist_pitch_link"));
	left_arm.addLink(icub_serialization.getLinkId("l_wrist_yaw_link"));
	//new links	
	left_arm.addLink(icub_serialization.getLinkId("l_shoulder_yaw_link"));
	left_arm.addLink(icub_serialization.getLinkId("l_gripper"));

	
	KDL::CoDyCo::TreePart right_arm(RIGHT_ARM,BodyPart_s[RIGHT_ARM]);
	right_arm.addDOF(icub_serialization.getDOFId("r_shoulder_pitch_joint"));
	right_arm.addDOF(icub_serialization.getDOFId("r_shoulder_roll_joint"));
	right_arm.addDOF(icub_serialization.getDOFId("r_shoulder_yaw_joint"));
	right_arm.addDOF(icub_serialization.getDOFId("r_elbow_joint"));
	right_arm.addDOF(icub_serialization.getDOFId("r_wrist_prosup_joint"));
	right_arm.addDOF(icub_serialization.getDOFId("r_wrist_pitch_joint"));
	right_arm.addDOF(icub_serialization.getDOFId("r_wrist_yaw_joint"));
    
    //The link serialization is done in a way to be compatible with skinDynLib 
    //(so the the upper part of the forerarm is shifted at the end)
	right_arm.addLink(icub_serialization.getLinkId("r_shoulder_pitch_link"));
	right_arm.addLink(icub_serialization.getLinkId("r_shoulder_roll_link"));
	right_arm.addLink(icub_serialization.getLinkId("r_arm_ft_sensor_link"));
	right_arm.addLink(icub_serialization.getLinkId("r_elbow_link"));
	right_arm.addLink(icub_serialization.getLinkId("r_wrist_prosup_link"));
	right_arm.addLink(icub_serialization.getLinkId("r_wrist_pitch_link"));
	right_arm.addLink(icub_serialization.getLinkId("r_wrist_yaw_link"));
	//new links	
	right_arm.addLink(icub_serialization.getLinkId("r_shoulder_yaw_link"));
	right_arm.addLink(icub_serialization.getLinkId("r_gripper"));
	
	KDL::CoDyCo::TreePart left_leg(LEFT_LEG,BodyPart_s[LEFT_LEG]);
	left_leg.addDOF(icub_serialization.getDOFId("l_hip_pitch_joint"));
	left_leg.addDOF(icub_serialization.getDOFId("l_hip_roll_joint"));
	left_leg.addDOF(icub_serialization.getDOFId("l_hip_yaw_joint"));
	left_leg.addDOF(icub_serialization.getDOFId("l_knee_joint"));
	left_leg.addDOF(icub_serialization.getDOFId("l_ankle_pitch_joint"));
	left_leg.addDOF(icub_serialization.getDOFId("l_ankle_roll_joint"));
    
    //The link serialization is done in a way to be compatible with skinDynLib 
    //(so the the upper part of the forerarm is shifted at the end)
	left_leg.addLink(icub_serialization.getLinkId("l_hip_pitch_link"));
	left_leg.addLink(icub_serialization.getLinkId("l_hip_roll_link"));
	left_leg.addLink(icub_serialization.getLinkId("l_leg_ft_sensor_link"));
	left_leg.addLink(icub_serialization.getLinkId("l_knee_link"));
	left_leg.addLink(icub_serialization.getLinkId("l_ankle_pitch_link"));
	left_leg.addLink(icub_serialization.getLinkId("l_ankle_roll_link"));
	//new links	
	left_leg.addLink(icub_serialization.getLinkId("l_hip_yaw_link"));
	left_leg.addLink(icub_serialization.getLinkId("l_sole"));

	
	KDL::CoDyCo::TreePart right_leg(RIGHT_LEG,BodyPart_s[RIGHT_LEG]);
	right_leg.addDOF(icub_serialization.getDOFId("r_hip_pitch_joint"));
	right_leg.addDOF(icub_serialization.getDOFId("r_hip_roll_joint"));
	right_leg.addDOF(icub_serialization.getDOFId("r_hip_yaw_joint"));
	right_leg.addDOF(icub_serialization.getDOFId("r_knee_joint"));
	right_leg.addDOF(icub_serialization.getDOFId("r_ankle_pitch_joint"));
	right_leg.addDOF(icub_serialization.getDOFId("r_ankle_roll_joint"));
    
    //The link serialization is done in a way to be compatible with skinDynLib 
    //(so the the upper part of the forerarm is shifted at the end)
	right_leg.addLink(icub_serialization.getLinkId("r_hip_pitch_link"));
	right_leg.addLink(icub_serialization.getLinkId("r_hip_roll_link"));
	right_leg.addLink(icub_serialization.getLinkId("r_leg_ft_sensor_link"));
	right_leg.addLink(icub_serialization.getLinkId("r_knee_link"));
	right_leg.addLink(icub_serialization.getLinkId("r_ankle_pitch_link"));
	right_leg.addLink(icub_serialization.getLinkId("r_ankle_roll_link"));
	//new links	                                     
	right_leg.addLink(icub_serialization.getLinkId("r_hip_yaw_link"));
	right_leg.addLink(icub_serialization.getLinkId("r_sole"));
	
	KDL::CoDyCo::TreePartition partition;
	partition.addPart(head);
	partition.addPart(torso);
	partition.addPart(left_arm);
	partition.addPart(right_arm);
	partition.addPart(left_leg);
	partition.addPart(right_leg);
	
	return partition;
}

iCubTree::~iCubTree() {}

}
}
