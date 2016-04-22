#include "iCubWalkingIKThread.h"

iCubWalkingIKThread::iCubWalkingIKThread ( int period,
                                           yarpWbi::yarpWholeBodyModel* wbm,
                                           yarpWbi::yarpWholeBodyStates* wbs,
                                           walkingParams params,
                                           odometryParams &odometry_params,
                                           inverseKinematicsParams &inverseKin_params,
                                           yarp::os::ResourceFinder& rf,
                                           std::string walkingPatternFile,
                                           std::string outputDir ) :
RateThread(period),
m_period(period),
m_walkingPatternFile(walkingPatternFile),
m_walkingParams(params),
m_odometryParams(odometry_params),
m_inverseKinematicsParams(inverseKin_params),
m_rf(rf),
m_wbm(wbm),
m_wbs(wbs),
m_outputDir(outputDir)
{ }
#pragma mark -
#pragma mark Initialization
bool iCubWalkingIKThread::threadInit() {
    // Generate feet trajectories
    generateFeetTrajectories(m_walkingPatternFile, m_walkingParams);
    
    m_odometry = new floatingBaseOdometry(m_wbm);
    // Initialize floating base odometry
    
#pragma mark NOTE: Next two lines for future implementation
    // Initial (current) robot configuration
    KDL::Vector computed_initial_world_offset;
    computed_initial_world_offset.Zero();
    if ( m_odometryParams.world_between_feet ) {
//    if ( true ) {
        if ( !computeCenterBetweenFeet(computed_initial_world_offset, m_odometryParams.initial_world_reference_frame) ) {
            yError("iCubWalkingIKThread::threadInit(): There were problems computing the center between the twoo feet at the current configuration");
            return false;
        }
    }

#pragma mark NOTE: This is now specified in the configuration file of this module
    KDL::Vector initial_world_offset;
    if ( m_odometryParams.world_between_feet ) {
        // Use the computed offset to the point between both feet
        initial_world_offset = computed_initial_world_offset;
    } else {
        // Use the offset defined in the configuration file
        initial_world_offset = m_odometryParams.offset_from_world_reference_frame;
    }
    std::string initial_world_frame_position = m_odometryParams.initial_world_reference_frame;
    std::string initial_fixed_link = m_odometryParams.initial_fixed_frame;
    std::string floating_base_frame_index = m_odometryParams.floating_base;
    
    if ( !m_odometry->init(initial_world_frame_position,
                           initial_fixed_link,
                           floating_base_frame_index,
                           initial_world_offset) ) {
        yError("iCubWalkingIKThread could not initialize the odometry object");
        return false;
    }
    
    //NOTE: Added an additional odometry object to stream the floating base rototranslation when it is actually walking, not for the planning.
    //FIXME: Remember to destroy this object during closure
    m_walkingOdometry = new floatingBaseOdometry(m_wbm);
    if ( !m_walkingOdometry->init(initial_world_frame_position,
                                 initial_fixed_link,
                                 floating_base_frame_index,
                                 initial_world_offset) ) {
        yError("iCubWalkingIKThread could not initialize the odometry object to rack iCub's floating base");
        return false;
    }
    
    m_actual_q.resize(m_wbm->getDoFs());
    std::string module_name = m_rf.find("name").asString();
    m_ft_foot_port.open(std::string("/" + module_name + "/foot_ft:i").c_str());
    std::string robot_name = m_rf.find("robot").asString();
    yarp::os::Network::connect( "/" + robot_name + "/right_foot/analog:o", m_ft_foot_port.getName());
    m_output_floating_base.open(std::string("/" + module_name + "/floating_base:o").c_str());
    
    m_stream_floating_base_flag = m_rf.find("stream_floating_base_pose").asBool();
    if ( m_stream_floating_base_flag ) {
        std::string feet_constraints_port_name = std::string("/" + module_name + "/feet_constraints:i");
        m_feet_constraints.open(feet_constraints_port_name.c_str());
        if ( !yarp::os::Network::connect("/walkPlayer/constraints:o", feet_constraints_port_name) ) {
            yError("walkPlayer is not streaming the constraints... In this version it is necessary");
            return false;
        }
    }
    //Initialize previous constraints to 1 1, as we assume the robot always starts on two feet.
    m_previous_constraints.resize(2, 1);
    m_switch_foot = false;
    //END - odometry while walking
    
    yInfo("Finished initialization... \nWaiting for RPC command [run]...");
    return true;
}

iCubWalkingIKThread::~iCubWalkingIKThread() {}

#pragma mark -
void iCubWalkingIKThread::run() {
    this->thread_mutex.wait();
    // if ( this->planner_flag )
    if ( this->planner_flag ) {
        double init_time = yarp::os::Time::now();
        inverseKinematics(m_walkingParams);
        this->planner_flag = false;
        double final_time = yarp::os::Time::now();
        yInfo("Total elapsed time: %f seconds", final_time - init_time);
    }
    
    if ( m_stream_floating_base_flag ) {
        m_wbs->getEstimates(wbi::ESTIMATE_JOINT_POS, m_actual_q.data());
        // If feet FT sensor is under threshold and walkPlayer says it can switch, then do it. For now, set to false;
        // Read feet FT sensor
        yarp::os::Bottle foot_ft;
        yarp::os::Bottle constraints;
        m_ft_foot_port.read(foot_ft,true);
        m_feet_constraints.read(constraints,true);
        yarp::sig::Vector new_constraints(2);
        new_constraints.zero();
        new_constraints(0) = constraints.get(0).asInt();
        new_constraints(1) = constraints.get(1).asInt();
        // Read walkPlayer
        // Constraints are interpreted as: 1 1 => left (on) right (on)
        // If weight on right leg is above 27.5Kg and the left foot is supposed to be off (as by the planner) or ...
        // If weight on right leg is near zero and the right foot is supposed to be off (as by the planner).
        if (false) {
            std::cerr << "FT_z right foot: " << foot_ft.get(2).asDouble() << std::endl;
            std::cerr << "Left contact: " << constraints.get(0).asInt() << std::endl;
            std::cerr << "Right contact: " << constraints.get(1).asInt() << std::endl;
        }
        // When planner says to lift off the foot
        if ( ( m_previous_constraints(0) == 1 && m_previous_constraints(1) == 1 ) &&
             ( new_constraints(0) != m_previous_constraints(0) || new_constraints(1) != m_previous_constraints(1) ) ) {
            m_switch_foot = true;
            yInfo("FOOT WAS SWITCHED");
        }
            
        //std::cout << "" << std::endl;
        m_previous_constraints(0) = new_constraints(0);
        m_previous_constraints(1) = new_constraints(1);
        m_walkingOdometry->update(m_actual_q.data(), m_switch_foot);
        m_switch_foot = false;
        m_walkingOdometry->get_world_H_floatingbase( m_world_H_floatingbase );
        yarp::sig::Vector &out_floatingbase = m_output_floating_base.prepare();
        yarp::sig::Vector tmp(12); // [translation - rotation matrix(column-wise)]
        tmp[0] = m_world_H_floatingbase.p[0];
        tmp[1] = m_world_H_floatingbase.p[1];
        tmp[2] = m_world_H_floatingbase.p[2];
        unsigned int kk = 3;
        for (unsigned int j=0; j<3; j++) {
            for (unsigned int i=0; i<3; i++) {
                tmp[kk] = m_world_H_floatingbase.R(i,j);
                kk++;
            }
        }
        out_floatingbase = tmp;
        m_output_floating_base.write();
    }
    this->thread_mutex.post();
}

#pragma mark -
#pragma mark Main inverse kinematics and walking methods
void iCubWalkingIKThread::generateFeetTrajectories(std::string walkingPatternFile,
                                                   walkingParams paramsList) {
    std::string inputfile = walkingPatternFile;
    int N = paramsList.n_samples;
    double ts = (double) m_period/1000;
    int N_steps = paramsList.n_strides;
    // interpolators for the feet and com trajectories from the patterns
    // It uses a spline interpolation made by Martin Felis
    SplineInterpolator<Eigen::VectorXd> r_foot_interp;
    SplineInterpolator<Eigen::VectorXd> l_foot_interp;
    SplineInterpolator<Eigen::VectorXd> com_interp;
    
    // reads the input MUSCOD file
    Eigen::VectorXd temp_vec(5);
    std::vector<Eigen::VectorXd> inputs(N,temp_vec);
    readFromCSV(inputfile,inputs);
    
    // pattern vectors
    Eigen::VectorXd temp_vec_2(4);
    std::vector<Eigen::VectorXd> com_pattern(N,temp_vec_2);
    
    //Fixed by Yue
    // separate com pattern
    for(unsigned int i = 0; i < N; i++)
    {
        Eigen::VectorXd temp_com(4);
        temp_com[0] = inputs[i][0];
        temp_com[1] = inputs[i][1];
        temp_com[2] = inputs[i][2];
        temp_com[3] = paramsList.z_c;
        com_pattern[i] = temp_com;
    }
    //FIXME: This file should be written in the YARP_ROBOT_NAME installation dir along with all the others that have been already migrated.
//    std::string com_pattern_file = m_rf.findFile("com_pattern.csv");
    writeOnCSV(com_pattern, "com_pattern.csv");
    
    // compute useful quantities
    double finalTime = inputs[N-1][0];
    
    // generate feet and com trajectories
    // [§§§] Modified by Jorhabib on December 8 sine this was not taking
    // into account that time starts from 0
    int N_traj = ceil(finalTime/ts)+1;
    
    //FIXME: Make the following variables private to this class so that they can  be accesed from the run method
    // trajectories
    std::vector<Eigen::VectorXd> r_foot_traj(N_traj);
    std::vector<Eigen::VectorXd> l_foot_traj(N_traj);
    std::vector<Eigen::VectorXd> com_traj(N_traj);
    
    std::string r_foot_pattern_aug_file = m_rf.findFile("r_foot_pattern_aug.csv");
    std::string l_foot_pattern_aug_file = m_rf.findFile("l_foot_pattern_aug.csv");
    r_foot_interp.generateFromCSV(r_foot_pattern_aug_file.c_str());
    l_foot_interp.generateFromCSV(l_foot_pattern_aug_file.c_str());
    com_interp.generateFromCSV("com_pattern.csv");
    
    double t = 0.0;
    int h = 1; //index used to separate left and right feet
    // time in which the single support occurs is the time of a half step - the time of com switching (double support)
    double t_ss = paramsList.T_stride/2 - 2*paramsList.T_switch;
    for(unsigned int i = 0; i < N_traj; i++)
    {
        r_foot_traj[i] = r_foot_interp.getValues(t);
        l_foot_traj[i] = l_foot_interp.getValues(t);
        com_traj[i]    = com_interp.getValues(t);
        // For the first
        if (h==1 && i>0)
        {
            r_foot_traj[i] = r_foot_traj[i-1];
            r_foot_traj[i][2] = 0;
            l_foot_traj[i] = l_foot_traj[i-1];
            l_foot_traj[i][2] = 0;
        }
        if(h%2==0) // workaround for spline interpolation bug.
        {
            r_foot_traj[i] = r_foot_traj[i-1];
            r_foot_traj[i][2] = 0;
            // com switching at the beginning and at the end of each step, robot in double support phase
            if(t<((h-1)*paramsList.T_stride/2 + paramsList.T_switch) || t>((h-1)*paramsList.T_stride/2 + t_ss + paramsList.T_switch))
            {
                l_foot_traj[i] = l_foot_traj[i-1];
                l_foot_traj[i][2] = 0;
            }
        }
        else if(i>0)
        {
            l_foot_traj[i] = l_foot_traj[i-1];
            l_foot_traj[i][2] = 0;
            // com switching at the beginning and at the end of each step, robot in double support phase
            if(t<((h-1)*paramsList.T_stride/2 + paramsList.T_switch) || t>((h-1)*paramsList.T_stride/2 + t_ss + paramsList.T_switch))
            {
                r_foot_traj[i] = r_foot_traj[i-1];
                r_foot_traj[i][2] = 0;
            }
        }
        // Time at which a step occurs
        if(fabs(t - h*paramsList.T_stride/2) < 10e-08)
            h++;
        //     t += ts;
        t = (i+1)*ts;
    }
    
    writeOnCSV(r_foot_traj,"r_foot_traj.csv");
    writeOnCSV(l_foot_traj,"l_foot_traj.csv");
    writeOnCSV(com_traj,"com_traj.csv");

}

void iCubWalkingIKThread::inverseKinematics(walkingParams params) {
    double step_duration = params.T_stride;
    double ts = (double) m_period/1000;
    int N_steps = params.n_strides;
    
    int step_N = step_duration/ts;
    int N = N_steps*step_N + step_N + 1;
    
    // quantities to store trajectories read from file
    Eigen::VectorXd temp = Eigen::VectorXd::Zero(3);
    std::vector<Eigen::VectorXd> l_foot(N,temp);
    std::vector<Eigen::VectorXd> r_foot(N,temp);
    std::vector<Eigen::VectorXd> com(N,temp);
    Eigen::VectorXd time_vec(N);
    
    //FIXME: This should be passed to the init method of this thread.
    // read the feet and com trajectories
    double init_time = yarp::os::Time::now();
    readFromCSV("l_foot_traj.csv",l_foot);
    readFromCSV("r_foot_traj.csv",r_foot);
    readFromCSV("com_traj.csv",com);
    double end_time = yarp::os::Time::now();
    yInfo("Time reading trajectories in inverseKinematics: %lf", end_time - init_time);
    // initial guess of the joint configuration
    Eigen::VectorXd qinit = Eigen::VectorXd::Zero(m_wbm->getDoFs());
    // result of the inverse kinematics
    Eigen::VectorXd qres = Eigen::VectorXd::Zero(m_wbm->getDoFs());
    // store all the resulting configurations
    std::vector<Eigen::VectorXd> res(N,qres);
    
    // body ids of the bodies we want to use with IK
    std::vector<unsigned int> body_ids(3);
    // the points attached to the respective bodies
    std::vector<Eigen::Vector3d> body_points(3);
    // taget positions in world ref frame of the specified points
    std::vector<Eigen::Vector3d> target_pos(3);
    // target orientation
    std::vector<Eigen::Matrix3d> target_orientation(3);

    // we want to match left and right feet and a point attached to the root as "CoM"
    // since IK of RBDL does not have the concept of orientation because it uses points, we need at least 3 points on each foot to ensure the foot comes flat on the ground
    //    body_ids[0] = model.GetBodyId("l_sole");
    //    body_ids[1] = model.GetBodyId("r_sole");
    //    body_ids[2] = model.GetBodyId("chest");
    //    body_points[0] = Vector3dZero;Vector3d(0.0,0.00,0.0);
    //    body_points[1] = Vector3dZero;Vector3d(0.0,0.00,0.0);
    //    body_points[2] = RigidBodyDynamics::Math::Vector3d(0,-0.15,0); // "CoM", fixed wrt chest
    //
    //    string ort_order = "123";

    int tmp;
    m_wbm->getFrameList().idToIndex("l_sole", tmp);
    body_ids[0] = tmp;
    m_wbm->getFrameList().idToIndex("r_sole", tmp);
    body_ids[1] = tmp;
    m_wbm->getFrameList().idToIndex("chest", tmp);
    body_ids[2] = tmp;
    body_points[0] = Eigen::Vector3d::Zero();
    body_points[1] = Eigen::Vector3d::Zero();
    //FIXME: This should be read at configuration time (as well as the previous body_points, although at the moment it's being overwritten later.
    body_points[2] = Eigen::Vector3d(0,-0.11,0);
    
    std::string ort_order = "123";
    
    Eigen::Vector3d l_foot_ort = Eigen::Vector3d(DEG2RAD(0),DEG2RAD(0),DEG2RAD(0));
    Eigen::Vector3d r_foot_ort = Eigen::Vector3d(DEG2RAD(0),DEG2RAD(0),DEG2RAD(0));
    //TODO: need to check orientation of chest wrt l_sole (world)
    Eigen::Vector3d com_ort = Eigen::Vector3d(DEG2RAD(0),0,DEG2RAD(0)); // chest
    
    target_orientation[0] = CalcOrientationEulerXYZ(l_foot_ort,ort_order); //l_foot
    target_orientation[1] = CalcOrientationEulerXYZ(r_foot_ort,ort_order); //r_foot
    //TODO: Change this into rotation matrix when orientation of chest wrt l_sole is checked
    target_orientation[2] = Eigen::Matrix3d::Zero();//CalcOrientationEulerXYZ(com_ort,ort_order); //chest
    
    //TODO: Not setting to any qinit for now, since it will be read inside IKinematics from the current configuration of the robot.
    // set initial position close to the target to avoid weird solutions
//    qinit[2] = 0.6;
//    qinit[6] = 0.54;
//    qinit[9] = -0.57;
//    qinit[10] = -0.23;
//    qinit[12] = 0.54;
//    qinit[15] = -0.57;
//    qinit[16] = -0.23;
    
    // Initializing variable to store actual com trajectory after solving inverse kinematics.
    std::vector<Eigen::VectorXd> com_real(N,Eigen::Vector3d::Zero());
    // Initializing variable to store actual feet trajectory after solving inverse kinematics.
    std::vector<Eigen::VectorXd> l_foot_real(N,Eigen::Vector3d::Zero());
    std::vector<Eigen::VectorXd> r_foot_real(N,Eigen::Vector3d::Zero());

    // IK parameters
#pragma mark NOTE: step_tol is now 1e-04, it does not converge for lower tollerances, you should not change lambda
    double step_tol = m_inverseKinematicsParams.step_tolerance;
    double lambda   = m_inverseKinematicsParams.lambda;
    double max_iter = m_inverseKinematicsParams.max_iter;
    
    // perform some initial IK to get a closer qinit and real com
    int trials = m_inverseKinematicsParams.trials_initial_IK;
    target_pos[0] = l_foot[0];
    target_pos[1] = r_foot[0];
    target_pos[2] = com[0];
    
    m_wbs->getEstimates(wbi::ESTIMATE_JOINT_POS, qinit.data());

    //FIXME: parameter for switching fixed foot
    bool switch_fixed = false;
    for(int k = 0; k < trials; k++)
    {
    //FIXME: introduced parameter for switching fixed foot
        //NOTE: It would be nice if body_ids, target_pos, target_orientation, body_points, lambda and max_iter were put in some structure for inverse kinematics (like ik_params).
        if (!IKinematics(m_wbm, m_wbs, m_odometry, qinit, body_ids, target_pos, target_orientation, body_points, qres, switch_fixed, step_tol, lambda, max_iter))
        {
            yWarning("iCubWalkingIKThread::inverseKinematics \n COM Inv. Kinematics \n - Could not converge to a solution with the desired tolerance of %lf", step_tol);
        } /*else {
           std::cout << "[INFO] RigidBodyDynamics::InverseKinematics - IK converged! " << std::endl;
           }*/
        qinit = qres;
        
        // Transforming COM coordinates in chest
        int chestId;
        m_wbm->getFrameList().idToIndex("chest", chestId);
        wbi::Frame H_from_chest_to_root;
        m_wbm->computeH(qinit.data(), wbi::Frame(), chestId, H_from_chest_to_root);
        Eigen::VectorXd com_from_chest(7);
        m_wbm->forwardKinematics(qres.data(), H_from_chest_to_root, wbi::iWholeBodyModel::COM_LINK_ID, com_from_chest.data());
        body_points[2] = com_from_chest.head(3);
        //!!!!: The following line is very specific to iCub and has been figured out comparing the height of the resulting COM with the resulting joint angles from inverse kinematics against the planned COM trajectory. The exact difference has been then used to find this offset from the chest (in the y direction which is vertical to the floor)!
        body_points[2] <<  0.0, -0.0317, 0.0;
//        RigidBodyDynamics::Utils::CalcCenterOfMass(model,qinit,qdot,mass,com_temp);
//        com_real[0] = com_temp;
//        body_points[2] = CalcBaseToBodyCoordinates(model,qinit,body_ids[2],com_real[0]);
    }
    
    m_wbs->getEstimates(wbi::ESTIMATE_JOINT_POS, qinit.data());
    
    // perform inverse kinematics using all the defined points and the target positions as from the planned feet and com trajectories
    double t = 0;
    for(int i = 0; i < N; i++)
    {
        // Retrieve real COM in chest coordinates
        int chestId;
        m_wbm->getFrameList().idToIndex("chest", chestId);
        wbi::Frame H_from_chest_to_root;
        m_wbm->computeH(qinit.data(), wbi::Frame(), chestId, H_from_chest_to_root);
        Eigen::VectorXd com_from_chest(7);
        Eigen::VectorXd com_from_root(7);
        //!!!!: Maybe in the following line instead of qres I should have qinit!!!
        m_wbm->forwardKinematics(qinit.data(), H_from_chest_to_root, wbi::iWholeBodyModel::COM_LINK_ID, com_from_chest.data());
        m_wbm->forwardKinematics(qinit.data(), wbi::Frame(), wbi::iWholeBodyModel::COM_LINK_ID, com_from_root.data());
        body_points[2] = com_from_chest.head(3);
//        std::cout << "com_from_chest: " << com_from_chest.head(3) << std::endl;
//        std::cout << "com_from_root: " << com_from_root.head(3) << std::endl;
        //!!!!: The following line is very specific to iCub and has been figured out comparing the height of the resulting COM with the resulting joint angles from inverse kinematics against the planned COM trajectory. The exact difference has been then used to find this offset from the chest (in the y direction which is vertical to the floor)!
        body_points[2] << 0.0, -0.0317, 0.0;

//        // use the real com as body point
//        RigidBodyDynamics::Utils::CalcCenterOfMass(model,qinit,qdot,mass,com_temp);
//        com_real[i] = com_temp;
//        body_points[2] = CalcBaseToBodyCoordinates(model,qinit,body_ids[2],com_real[i]);
        
        //FIXME: parameter to switch fixed foot, always to false except when need to switch
        switch_fixed = false;
        if(i>0)//step_N) //use step_N if starting with r_sole
        {
          // switch when one of the feet is lifting from the ground
          if(fabs(r_foot[i-1][2]-l_foot[i-1][2]) == 0 && fabs(r_foot[i][2]-l_foot[i][2]) > 0)
          {
            switch_fixed = true;
            std::cout << "Switching fixed link at time " << t << std::endl; 
          }
        }
        
        target_pos[0] = l_foot[i];
        target_pos[1] = r_foot[i];
        target_pos[2] = com[i];
        time_vec[i] = t;
        if (!IKinematics(m_wbm, m_wbs, m_odometry, qinit, body_ids, target_pos, target_orientation, body_points, qres, switch_fixed,  step_tol, lambda, max_iter))
        {
            yWarning("iCubWalkingIKThread::inverseKinematics \n Inv. Kinematics for all targets \n Could not converge to a solution with the desired tolerance of %lf", step_tol);
        }
        
        // Upating real com trajectory after performing inverse kinematics
        Eigen::VectorXd tmp_com_real(3);
        tmp_com_real << 0,0,0;
        Eigen::VectorXd offset(3);
        offset.setZero();
        updateCOMreal( m_odometry, tmp_com_real, offset );
        com_real[i] = tmp_com_real;
        
        // Updating real feet trajectory after performing inverse kinematics
        Eigen::VectorXd tmp_l_foot_real(3); tmp_l_foot_real.setZero();
        Eigen::VectorXd tmp_r_foot_real(3); tmp_r_foot_real.setZero();
        updateFootTrajReal(m_odometry, qres.data(), tmp_l_foot_real, "l_sole");
        updateFootTrajReal(m_odometry, qres.data(), tmp_r_foot_real, "r_sole");
        l_foot_real[i] = tmp_l_foot_real;
        r_foot_real[i] = tmp_r_foot_real;
        
        
        res[i] = qres;
        qinit = qres;
        t += ts;
        
        double percentage = (double)i/N*100.0;
        std::cout << "Percentage: " << int(percentage) << "% \r";
        std::cout.flush();
    }
    
    // convert into degrees
    // store all the resulting configurations
    std::vector<Eigen::VectorXd> res_deg(N,qres);
//     Eigen::VectorXd qres_no_fb(qres.size()-6);
//     std::vector<Eigen::VectorXd> res_deg_cut(N,qres_no_fb);
    Eigen::VectorXd time_vec_new(N);
    t = 0;
    for(int i = 0; i < N; i++)
    {
        time_vec_new[i] = t;
        for(int j = 0; j < qres.size(); j++)
            res_deg[i][j] = res[i][j];

        t += ts;
    }

    // Change the reference of the com from root to l_sole as per real iCub convention for torque control
    std::vector<Eigen::VectorXd> com_l_sole(N,Eigen::Vector3d::Zero());
    int l_sole_idx;
    m_wbm->getFrameList().idToIndex("l_sole", l_sole_idx);
    wbi::Frame H_from_l_sole_to_root;
    m_wbm->computeH(qinit.data(), wbi::Frame(), l_sole_idx, H_from_l_sole_to_root);
    Eigen::VectorXd l_sole_pos = Eigen::Map<Eigen::VectorXd>(H_from_l_sole_to_root.p,3,1);
    std::cout << "l_sole pos wrt root: " << H_from_l_sole_to_root.p[0] << " " << H_from_l_sole_to_root.p[1] << " " << H_from_l_sole_to_root.p[2] << std::endl;
    //TODO: This copying needs to be double-checked.
    Eigen::MatrixXd l_sole_ort = Eigen::Map<Eigen::MatrixXd>(H_from_l_sole_to_root.R.data,3,3);
    Eigen::Vector3d l_foot_ang = Eigen::Vector3d::Zero();
    getEulerAngles(l_sole_ort, l_foot_ang, ort_order);
    std::cout << "l_sole rot wrt root: " << l_foot_ang.transpose() << std::endl;
    for(int i = 0; i < N; i++)
    {
        com_l_sole[i] = l_sole_ort*com[i] + l_sole_pos;
    }
    
    // Changed from res_deg_cut to res_deg as res_deg_cut does not make sense now
    writeOnCSV(res_deg,m_outputDir + "/test_ik_pg.csv");
    yInfo("Wrote test_ik_pg file: %s ", std::string(m_outputDir + "/test_ik_pg.csv").c_str());
    writeOnCSV(com_real,m_outputDir + "/real_com_traj.csv");
    yInfo("Wrote real_com_traj file: %s ", std::string(m_outputDir + "/real_com_traj.csv").c_str());
    writeOnCSV(com_l_sole,m_outputDir + "/com_l_sole.csv");
    yInfo("Wrote com_l_sole file: %s ", std::string(m_outputDir + "/com_l_sole.csv").c_str());
    writeOnCSV(l_foot_real, std::string(m_outputDir + "/real_left_foot.csv").c_str());
    yInfo("Wrote real_left_foot file: %s ", std::string(m_outputDir + "/real_left_foot.csv").c_str());
    writeOnCSV(r_foot_real, std::string(m_outputDir + "/real_right_foot.csv").c_str());
    yInfo("Wrote real_left_foot file: %s ", std::string(m_outputDir + "/real_right_foot.csv").c_str());

}

#pragma mark -
#pragma mark Helper and diagnosis methods
bool iCubWalkingIKThread::computeCenterBetweenFeet(KDL::Vector &v, std::string ref_frame) {
    // Read current kinematic configuration of th    yarp::sig::Vector initial_configuration_wbm(m_wbm->getDoFs());
    yarp::sig::Vector initial_configuration_wbm(m_wbm->getDoFs());
    m_wbs->getEstimates(wbi::ESTIMATE_JOINT_POS, initial_configuration_wbm.data());
    
    // Compute distance between two feet from ref_frame
    yarp::sig::Vector initial_configuration_dyntree(m_wbm->getRobotModel()->getNrOfDOFs());
    m_wbm->convertQ(initial_configuration_wbm.data(), initial_configuration_dyntree);
    
    // Update internal buffers of DynTree robot model
    iDynTree::RobotJointStatus m_joint_status;
    m_joint_status.setNrOfDOFs(m_wbm->getRobotModel()->getNrOfDOFs());
    m_joint_status.zero();
    m_joint_status.setJointPosYARP(initial_configuration_dyntree);
    m_joint_status.updateKDLBuffers();
    
    m_wbm->getRobotModel()->setAng(m_joint_status.getJointPosYARP());
    
    // Return distance to the center of the twoo feet expressed in ref_frame
    int index_ref_frame = m_wbm->getRobotModel()->getLinkIndex(ref_frame);
    int index_right_foot = m_wbm->getRobotModel()->getLinkIndex("r_sole");
    KDL::Frame tmpFrame = m_wbm->getRobotModel()->getPositionKDL(index_ref_frame, index_right_foot);
    v = 0.5*tmpFrame.p;
    return true;
}

bool iCubWalkingIKThread::updateCOMreal(floatingBaseOdometry * odometry, Eigen::VectorXd &com_real, Eigen::VectorXd offset) {
    wbi::Frame world_H_floatingbase;
    odometry->get_world_H_floatingbase(world_H_floatingbase);
    com_real << world_H_floatingbase.p[0], world_H_floatingbase.p[1], world_H_floatingbase.p[2];
    return true;
}

bool iCubWalkingIKThread::updateFootTrajReal(floatingBaseOdometry * odometry, double * q, Eigen::VectorXd &foot_real, std::string which_foot) {
    wbi::Frame world_H_floatingbase;
    odometry->get_world_H_floatingbase(world_H_floatingbase);
    // Retrieve rototranslation from <which_foot> to <root>
    int foot_index;
    wbi::Frame floatingbase_H_foot;
    m_wbm->getFrameList().idToIndex(which_foot.c_str(), foot_index);
    m_wbm->computeH(q, wbi::Frame(), foot_index, floatingbase_H_foot);
    // Compose rototranslations to express foot trajectory in world reference frame
    wbi::Frame world_H_foot;
    world_H_foot = world_H_floatingbase*floatingbase_H_foot;
    foot_real << world_H_foot.p[0], world_H_foot.p[1], world_H_foot.p[2];
    return true;
}

#pragma mark -
#pragma mark Cleanup and closure
void iCubWalkingIKThread::threadRelease() {
    if ( m_odometry ) {
        delete m_odometry;
        m_odometry = 0;
        yInfo("m_odometry was deleted ...");
    }
}
