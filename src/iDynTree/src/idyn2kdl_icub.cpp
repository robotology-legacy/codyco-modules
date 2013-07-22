/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
 
#include <iCub/iDynTree/idyn2kdl_icub.h>

template<typename T, size_t N>
T * end(T (&ra)[N]) {
    return ra + N;
}

bool names2links_joints(const std::vector<std::string> names,std::vector<std::string> & names_links,std::vector<std::string> & names_joints)
{
    names_links = names;
    names_joints = names;
    for(int i=0;i<(int)names.size();i++) {
        names_links[i] = names_links[i]+"_link";
        names_joints[i] = names_joints[i]+"_joint";
    }
    return true;
}

bool toKDL(const iCub::iDyn::iCubWholeBody & icub_idyn, KDL::Tree & icub_kdl,  iCub::iDynTree::iCubTree_serialization_tag serial, bool debug)
{
    bool status_ok = true;
    //Joint names extracted from http://eris.liralab.it/wiki/ICub_joints
    
    //Default "fake" base link for humanoids URDF
    std::string fake_root_name = "base_link";
    icub_kdl = KDL::Tree(fake_root_name);
    
    std::vector<std::string> joints,links;
    
    //Creating left leg
    KDL::Chain ll, old_ll;
    const char *ll_joints_cstr[] = {"l_hip_pitch", "l_hip_roll", "l_leg_ft_sensor", "l_hip_yaw", "l_knee", "l_ankle_pitch", "l_ankle_roll"};    
    std::vector<std::string> ll_joints(ll_joints_cstr,end(ll_joints_cstr));
    names2links_joints(ll_joints,links,joints);
    status_ok = idynSensorChain2kdlChain(*(icub_idyn.lowerTorso->left),*(icub_idyn.lowerTorso->leftSensor),old_ll,links,joints,"l_sole");
    if(!status_ok) return false;
    
    
    //Creating right leg
    KDL::Chain rl, old_rl;
    const char *rl_joints_cstr[] = {"r_hip_pitch", "r_hip_roll", "r_leg_ft_sensor", "r_hip_yaw", "r_knee", "r_ankle_pitch", "r_ankle_roll"};    
    std::vector<std::string> rl_joints(rl_joints_cstr,end(rl_joints_cstr));
    names2links_joints(rl_joints,links,joints);
    status_ok = idynSensorChain2kdlChain(*(icub_idyn.lowerTorso->right),*(icub_idyn.lowerTorso->rightSensor),old_rl,links,joints,"r_sole");
    if(!status_ok) return false;

    
    //Creating torso
    KDL::Chain torso, old_torso;
    const char *torso_joints_cstr[] = {"torso_yaw","torso_roll","torso_pitch"};
    std::vector<std::string> torso_joints(torso_joints_cstr,end(torso_joints_cstr));
    names2links_joints(torso_joints,links,joints);
    status_ok = idynChain2kdlChain(*(icub_idyn.lowerTorso->up),old_torso,links,joints);
    if(!status_ok) return false;


    
    //Creating left arm
    KDL::Chain la, old_la;
    const char *la_joints_cstr[] = {"l_shoulder_pitch", "l_shoulder_roll","l_shoulder_yaw" ,"l_arm_ft_sensor" , "l_elbow", "l_wrist_prosup", "l_wrist_pitch","l_wrist_yaw",};    
    std::vector<std::string> la_joints(la_joints_cstr,end(la_joints_cstr));
    names2links_joints(la_joints,links,joints);
    status_ok = idynSensorChain2kdlChain(*(icub_idyn.upperTorso->left),*(icub_idyn.upperTorso->leftSensor),old_la,links,joints,"l_gripper");
    if(!status_ok) return false;


    //Creating right arm
    KDL::Chain ra, old_ra;
    const char *ra_joints_cstr[] = {"r_shoulder_pitch", "r_shoulder_roll","r_shoulder_yaw", "r_arm_ft_sensor", "r_elbow", "r_wrist_prosup", "r_wrist_pitch","r_wrist_yaw",};     
    std::vector<std::string> ra_joints(ra_joints_cstr,end(ra_joints_cstr));
    names2links_joints(ra_joints,links,joints);
    status_ok = idynSensorChain2kdlChain(*(icub_idyn.upperTorso->right),*(icub_idyn.upperTorso->rightSensor),old_ra,links,joints,"r_gripper");
    if(!status_ok) return false;


    //Creating head
    KDL::Chain head, old_head;
    const char *head_joints_cstr[] = {"neck_pitch","neck_roll","neck_yaw","imu"};
    std::vector<std::string> head_joints(head_joints_cstr,end(head_joints_cstr));
    names2links_joints(head_joints,links,joints);
    status_ok = idynChain2kdlChain(*(icub_idyn.upperTorso->up),old_head,links,joints);
    if(!status_ok) return false;

    //Now that all the chain are created, it is possible to compose them
    //to create the iCub KDL::Tree

    //First we have to had the root_link, ( for now without RigidBodyInertia!)
    status_ok = icub_kdl.addSegment(KDL::Segment("root_link",KDL::Joint("base_joint",KDL::Joint::None),KDL::Frame::Identity()),fake_root_name);
    if(!status_ok) return false;

    
    
    //Adding the chains, selecting the default serialization
    KDL::Frame kdlFrame;
    idynMatrix2kdlFrame(icub_idyn.lowerTorso->HLeft,kdlFrame);
    addBaseTransformation(old_ll,ll,kdlFrame);


    
    idynMatrix2kdlFrame(icub_idyn.lowerTorso->HRight,kdlFrame);
    addBaseTransformation(old_rl,rl,kdlFrame);
    
    idynMatrix2kdlFrame(icub_idyn.lowerTorso->HUp,kdlFrame);
    addBaseTransformation(old_torso,torso,kdlFrame);
    
    //not using RBT because it is an identity, and it is not clear is 
    //semantical meaning (if is H_upper_lower or H_lower_upper ) 
    idynMatrix2kdlFrame(icub_idyn.upperTorso->HLeft,kdlFrame);
    addBaseTransformation(old_la,la,kdlFrame);
    
    idynMatrix2kdlFrame(icub_idyn.upperTorso->HRight,kdlFrame);
    addBaseTransformation(old_ra,ra,kdlFrame);
    
    idynMatrix2kdlFrame(icub_idyn.upperTorso->HUp,kdlFrame);
    addBaseTransformation(old_head,head,kdlFrame);

    if( serial == iCub::iDynTree::IDYN_SERIALIZATION  ) {
        //Using serialization in iCubWholeBody
        icub_kdl.addChain(ll,"root_link");
        icub_kdl.addChain(rl,"root_link");
        icub_kdl.addChain(torso,"root_link");
        icub_kdl.addChain(la,"torso_pitch_link");
        icub_kdl.addChain(ra,"torso_pitch_link");    
        icub_kdl.addChain(head,"torso_pitch_link");
    } else {
        assert(serial == iCub::iDynTree::SKINDYNLIB_SERIALIZATION);
        //Using serialization used in wholeBodyInterfaceYarp
        icub_kdl.addChain(torso,"root_link");
        icub_kdl.addChain(head,"torso_pitch_link");
        icub_kdl.addChain(la,"torso_pitch_link");
        icub_kdl.addChain(ra,"torso_pitch_link"); 
        icub_kdl.addChain(ll,"root_link");
        icub_kdl.addChain(rl,"root_link");   
    }


    //REP 120
    KDL::Segment kdlSegment = KDL::Segment("torso",KDL::Joint("torso_joint",KDL::Joint::None));
    icub_kdl.addSegment(kdlSegment,"torso_pitch_link");    
    
    
        
    if( debug ) {
        kdlSegment = KDL::Segment("torso_yaw",KDL::Joint("torso_yaw_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,"torso_yaw_link");    
    
        kdlSegment = KDL::Segment("torso_roll",KDL::Joint("torso_roll_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,"torso_roll_link");    
        
        kdlSegment = KDL::Segment("torso_pitch",KDL::Joint("torso_pitch_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,"torso_pitch_link");    
        
        
        kdlSegment = KDL::Segment("l_shoulder_pitch",KDL::Joint("l_shoulder_pitch_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,"l_shoulder_pitch_link");   
        
        kdlSegment = KDL::Segment("l_shoulder_roll",KDL::Joint("l_shoulder_roll_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,"l_shoulder_roll_link");   
        
        kdlSegment = KDL::Segment("l_arm_ft_sensor",KDL::Joint("l_arm_ft_sensor_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,"l_arm_ft_sensor_link");   
        
    
        kdlSegment = KDL::Segment("r_shoulder_pitch",KDL::Joint("r_shoulder_pitch_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,"r_shoulder_pitch_link");   
        
        kdlSegment = KDL::Segment("r_shoulder_roll",KDL::Joint("r_shoulder_roll_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,"r_shoulder_roll_link");   
        
        kdlSegment = KDL::Segment("r_arm_ft_sensor",KDL::Joint("r_arm_ft_sensor_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,"r_arm_ft_sensor_link");   
        
        kdlSegment = KDL::Segment("r_shoulder_yaw",KDL::Joint("r_shoulder_yaw_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,"r_shoulder_yaw_link");   
        
        kdlSegment = KDL::Segment("r_elbow",KDL::Joint("r_elbow_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,"r_elbow_link");   
       
    }
    
    return true;
    
}

bool toKDL_iDynDebug(const iCub::iDyn::iCubWholeBody & icub_idyn, KDL::Tree & icub_kdl, bool debug)
{
    bool status_ok = true;
    //Joint names extracted from http://eris.liralab.it/wiki/ICub_joints
    //Serialization: left leg (6), right leg (6), torso (3), left arm (7), right arm (7), head (3). 
    
    //Default "fake" base link for humanoids URDF
    std::string fake_root_name = "base_link";
    icub_kdl = KDL::Tree(fake_root_name);
    
    std::vector<std::string> joints,links;
    
    //Creating left leg
    KDL::Chain ll, old_ll;
    const char *ll_joints_cstr[] = {"l_hip_pitch", "l_hip_roll", "l_leg_ft_sensor", "l_hip_yaw", "l_knee", "l_ankle_pitch", "l_ankle_roll"};    
    std::vector<std::string> ll_joints(ll_joints_cstr,end(ll_joints_cstr));
    names2links_joints(ll_joints,links,joints);
    status_ok = idynSensorChain2kdlChain(*(icub_idyn.lowerTorso->left),*(icub_idyn.lowerTorso->leftSensor),old_ll,links,joints,"l_sole");
    if(!status_ok) return false;
    
    
    //Creating right leg
    KDL::Chain rl, old_rl;
    const char *rl_joints_cstr[] = {"r_hip_pitch", "r_hip_roll", "r_leg_ft_sensor", "r_hip_yaw", "r_knee", "r_ankle_pitch", "r_ankle_roll"};    
    std::vector<std::string> rl_joints(rl_joints_cstr,end(rl_joints_cstr));
    names2links_joints(rl_joints,links,joints);
    status_ok = idynSensorChain2kdlChain(*(icub_idyn.lowerTorso->right),*(icub_idyn.lowerTorso->rightSensor),old_rl,links,joints,"r_sole");
    if(!status_ok) return false;

    
    //Creating torso
    KDL::Chain torso, old_torso;
    const char *torso_joints_cstr[] = {"torso_yaw","torso_roll","torso_pitch"};
    std::vector<std::string> torso_joints(torso_joints_cstr,end(torso_joints_cstr));
    names2links_joints(torso_joints,links,joints);
    status_ok = idynChain2kdlChain(*(icub_idyn.lowerTorso->up),old_torso,links,joints);
    if(!status_ok) return false;


    
    //Creating left arm
    KDL::Chain la, old_la;
    const char *la_joints_cstr[] = {"l_shoulder_pitch", "l_shoulder_roll","l_shoulder_yaw" ,"l_arm_ft_sensor" , "l_elbow", "l_wrist_prosup", "l_wrist_pitch","l_wrist_yaw",};    
    std::vector<std::string> la_joints(la_joints_cstr,end(la_joints_cstr));
    names2links_joints(la_joints,links,joints);
    status_ok = idynSensorChain2kdlChain(*(icub_idyn.upperTorso->left),*(icub_idyn.upperTorso->leftSensor),old_la,links,joints,"l_gripper");
    if(!status_ok) return false;


    //Creating right arm
    KDL::Chain ra, old_ra;
    const char *ra_joints_cstr[] = {"r_shoulder_pitch", "r_shoulder_roll","r_shoulder_yaw", "r_arm_ft_sensor", "r_elbow", "r_wrist_prosup", "r_wrist_pitch","r_wrist_yaw",};     
    std::vector<std::string> ra_joints(ra_joints_cstr,end(ra_joints_cstr));
    names2links_joints(ra_joints,links,joints);
    status_ok = idynSensorChain2kdlChain(*(icub_idyn.upperTorso->right),*(icub_idyn.upperTorso->rightSensor),old_ra,links,joints,"r_gripper");
    if(!status_ok) return false;


    //Creating head
    KDL::Chain head, old_head;
    const char *head_joints_cstr[] = {"neck_pitch","neck_roll","neck_yaw","imu"};
    std::vector<std::string> head_joints(head_joints_cstr,end(head_joints_cstr));
    names2links_joints(head_joints,links,joints);
    status_ok = idynChain2kdlChain(*(icub_idyn.upperTorso->up),old_head,links,joints);
    if(!status_ok) return false;

    //Now that all the chain are created, it is possible to compose them
    //to create the iCub KDL::Tree

    //First we have to had the root_link, ( for now without RigidBodyInertia!)
    status_ok = icub_kdl.addSegment(KDL::Segment("root_link",KDL::Joint("base_joint",KDL::Joint::None),KDL::Frame::Identity()),fake_root_name);
    if(!status_ok) return false;

    
    
    //Adding the chains following the serialization
    KDL::Frame kdlFrame;
    idynMatrix2kdlFrame(icub_idyn.lowerTorso->HLeft,kdlFrame);
    addBaseTransformation(old_ll,ll,kdlFrame);

    status_ok = icub_kdl.addChain(ll,"root_link");
    if(!status_ok) return false;

    
    idynMatrix2kdlFrame(icub_idyn.lowerTorso->HRight,kdlFrame);
    addBaseTransformation(old_rl,rl,kdlFrame);
    icub_kdl.addChain(rl,"root_link");
    
    idynMatrix2kdlFrame(icub_idyn.lowerTorso->HUp,kdlFrame);
    addBaseTransformation(old_torso,torso,kdlFrame);
    icub_kdl.addChain(torso,"root_link");
    
    //not using RBT because it is an identity, and it is not clear is 
    //semantical meaning (if is H_upper_lower or H_lower_upper ) 
    idynMatrix2kdlFrame(icub_idyn.upperTorso->HLeft,kdlFrame);
    addBaseTransformation(old_la,la,kdlFrame);
    icub_kdl.addChain(la,"torso_pitch_link");
    
    idynMatrix2kdlFrame(icub_idyn.upperTorso->HRight,kdlFrame);
    //addBaseTransformation(old_ra,ra,kdlFrame);
    status_ok = icub_kdl.addSegment(KDL::Segment("r_torso_debug_link",KDL::Joint("r_torso_debug_joint",KDL::Joint::None),kdlFrame),"torso_pitch_link");
    if(!status_ok) return false;

    //icub_kdl.addChain(ra,"torso_pitch_link");    
    icub_kdl.addChain(old_ra,"r_torso_debug_link");    

    idynMatrix2kdlFrame(icub_idyn.upperTorso->HUp,kdlFrame);
    addBaseTransformation(old_head,head,kdlFrame);
    icub_kdl.addChain(head,"torso_pitch_link");


    
    //REP 120
    KDL::Segment kdlSegment = KDL::Segment("torso",KDL::Joint("torso_joint",KDL::Joint::None));
    icub_kdl.addSegment(kdlSegment,"torso_pitch_link");    
    
    
        
    if( debug ) {
        kdlSegment = KDL::Segment("torso_yaw",KDL::Joint("torso_yaw_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,"torso_yaw_link");    
    
        kdlSegment = KDL::Segment("torso_roll",KDL::Joint("torso_roll_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,"torso_roll_link");    
        
        kdlSegment = KDL::Segment("torso_pitch",KDL::Joint("torso_pitch_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,"torso_pitch_link");    
        
        
        kdlSegment = KDL::Segment("l_shoulder_pitch",KDL::Joint("l_shoulder_pitch_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,"l_shoulder_pitch_link");   
        
        kdlSegment = KDL::Segment("l_shoulder_roll",KDL::Joint("l_shoulder_roll_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,"l_shoulder_roll_link");   
        
        kdlSegment = KDL::Segment("l_arm_ft_sensor",KDL::Joint("l_arm_ft_sensor_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,"l_arm_ft_sensor_link");   
        
    
        kdlSegment = KDL::Segment("r_shoulder_pitch",KDL::Joint("r_shoulder_pitch_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,"r_shoulder_pitch_link");   
        
        kdlSegment = KDL::Segment("r_shoulder_roll",KDL::Joint("r_shoulder_roll_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,"r_shoulder_roll_link");   
        
        kdlSegment = KDL::Segment("r_arm_ft_sensor",KDL::Joint("r_arm_ft_sensor_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,"r_arm_ft_sensor_link");   
        
        kdlSegment = KDL::Segment("r_shoulder_yaw",KDL::Joint("r_shoulder_yaw_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,"r_shoulder_yaw_link");   
        
        kdlSegment = KDL::Segment("r_elbow",KDL::Joint("r_elbow_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,"r_elbow_link");   
       
    }
    
    return true;
    
}




