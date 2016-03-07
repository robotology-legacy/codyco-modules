#include "iCubWalkingIKThread.h"

iCubWalkingIKThread::iCubWalkingIKThread ( int period,
                                           wbi::iWholeBodyModel* wbm,
                                           wbi::iWholeBodyStates* wbs,
                                           walkingParams params,
                                           yarp::os::ResourceFinder& rf,
                                           std::string walkingPatternFile,
                                           std::string outputDir) :
RateThread(period),
m_period(period),
m_walkingPatternFile(walkingPatternFile),
m_walkingParams(params),
m_rf(rf),
m_wbm(wbm),
m_wbs(wbs),
m_outputDir(outputDir)
{ }

bool iCubWalkingIKThread::threadInit() {
    // Generate feet trajectories
    generateFeetTrajectories(m_walkingPatternFile, m_walkingParams);

    return true;
}

void iCubWalkingIKThread::run() {
    inverseKinematics(m_walkingParams);
    this->stop();
}

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
    
    // separate com pattern
    for(unsigned int i = 0; i < N; i++)
    {
        Eigen::VectorXd temp_com(4);
        temp_com[0] = inputs[i][0];
        temp_com[1] = -inputs[i][1];
        temp_com[2] = -inputs[i][2];
        temp_com[3] = paramsList.z_c;
        com_pattern[i] = temp_com;
    }
//    std::string com_pattern_file = m_rf.findFile("com_pattern.csv");
    writeOnCSV(com_pattern, "com_pattern.csv");
    
    // compute useful quantities
    double finalTime = inputs[N-1][0];
    
    // generate feet and com trajectories
    // [§§§] Modified by Jorhabib on December 8 sine this was not taking
    // into account that time starts from 0
    int N_traj = ceil(finalTime/ts)+1;
    
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
    
    // read the feet trajectories
    readFromCSV("l_foot_traj.csv",l_foot);
    readFromCSV("r_foot_traj.csv",r_foot);
    readFromCSV("com_traj.csv",com);
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
    body_points[2] = Eigen::Vector3d(0,-0.15,0);
    
    std::string ort_order = "123";
    
    Eigen::Vector3d l_foot_ort = Eigen::Vector3d(DEG2RAD(0),DEG2RAD(0),DEG2RAD(-180));//use 179 for Heidelberg01, -179.9 for iCubGenova02
    Eigen::Vector3d r_foot_ort = Eigen::Vector3d(DEG2RAD(0),DEG2RAD(0),DEG2RAD(-180));//use -179 for Heidelberg01, -179.9 for iCubGenova02
    Eigen::Vector3d com_ort = Eigen::Vector3d(DEG2RAD(0),0,DEG2RAD(0)); // root_link
    
    target_orientation[0] = CalcOrientationEulerXYZ(l_foot_ort,ort_order); //l_foot
    target_orientation[1] = CalcOrientationEulerXYZ(r_foot_ort,ort_order); //r_foot
    target_orientation[2] = Eigen::Matrix3d::Zero();//CalcOrientationEulerXYZ(com_ort,ort_order); //root_link
    
    //TODO: Not setting to anything qinit for now, since it will be read inside IKinematics from the current configuration of the robot.
    // set initial position close to the target to avoid weird solutions
//    qinit[2] = 0.6;
//    qinit[6] = 0.54;
//    qinit[9] = -0.57;
//    qinit[10] = -0.23;
//    qinit[12] = 0.54;
//    qinit[15] = -0.57;
//    qinit[16] = -0.23;
    
    std::vector<Eigen::VectorXd> com_real(N,Eigen::Vector3d::Zero());

    // IK parameters
    double step_tol = 1e-8;
    double lambda = 0.001;
    double max_iter = 100;
    
    // for the real com
    double mass = 0;
    Eigen::Vector3d com_temp = Eigen::Vector3d::Zero();
    
    // perform some initial IK to get a closer qinit and real com
    int trials = 10;
    target_pos[0] = l_foot[0];
    target_pos[1] = r_foot[0];
    target_pos[2] = com[0];
    
    for(int k = 0; k < trials; k++)
    {
        if (!IKinematics(m_wbm, m_wbs, qinit, body_ids, target_pos, target_orientation, body_points, qres, step_tol, lambda, max_iter))
        {
            yWarning("iCubWalkingIKThread::inverseKinematics - Could not converge to a solution with the desired tolerance of ");
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
        body_points[2] = com_from_chest;
//        RigidBodyDynamics::Utils::CalcCenterOfMass(model,qinit,qdot,mass,com_temp);
//        com_real[0] = com_temp;
//        body_points[2] = CalcBaseToBodyCoordinates(model,qinit,body_ids[2],com_real[0]);
    }
    
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
        m_wbm->forwardKinematics(qres.data(), H_from_chest_to_root, wbi::iWholeBodyModel::COM_LINK_ID, com_from_chest.data());
        body_points[2] = com_from_chest;
        
//        // use the real com as body point
//        RigidBodyDynamics::Utils::CalcCenterOfMass(model,qinit,qdot,mass,com_temp);
//        com_real[i] = com_temp;
//        body_points[2] = CalcBaseToBodyCoordinates(model,qinit,body_ids[2],com_real[i]);
        
        
        target_pos[0] = l_foot[i];
        target_pos[1] = r_foot[i];
        target_pos[2] = com[i];
        time_vec[i] = t;
        if (!IKinematics(m_wbm, m_wbs, qinit, body_ids, target_pos, target_orientation, body_points, qres, step_tol, lambda, max_iter))
        {
            yWarning("iCubWalkingIKThread::inverseKinematics - Could not converge to a solution with the desired tolerance of %lf", step_tol);
        }
        
        res[i] = qres;
        qinit = qres;
        t += ts;
    }
    
    // convert into degrees
    // store all the resulting configurations
    std::vector<Eigen::VectorXd> res_deg(N,qres);
    Eigen::VectorXd qres_no_fb(qres.size()-6);
    std::vector<Eigen::VectorXd> res_deg_cut(N,qres_no_fb);
    Eigen::VectorXd time_vec_new(N);
    t = 0;
    for(int i = 0; i < N; i++)
    {
        time_vec_new[i] = t;
        for(int j = 0; j < qres.size(); j++)
            res_deg[i][j] = res[i][j];

        t += ts;
    }

    // store the q only without floating base
    for(int i = 0; i < N; i++)
    {
        for(int j = 6; j < qres.size(); j++)
            res_deg_cut[i][j-6] = res_deg[i][j];
    }

    // Change the reference of the com from root to l_sole as per real iCub convention for torque control
    std::vector<Eigen::VectorXd> com_l_sole(N,Eigen::Vector3d::Zero());
    int l_sole_idx;
    m_wbm->getFrameList().idToIndex("l_sole", l_sole_idx);
    wbi::Frame H_from_l_sole_to_root;
    m_wbm->computeH(qinit.data(), wbi::Frame(), l_sole_idx, H_from_l_sole_to_root);
    Eigen::VectorXd l_sole_pos = Eigen::Map<Eigen::VectorXd>(H_from_l_sole_to_root.p,3,1);
    std::cout << "l_sole pos wrt root: " << H_from_l_sole_to_root.p << std::endl;
    //TODO: This copying needs to be double-checked.
    Eigen::MatrixXd l_sole_ort = Eigen::Map<Eigen::MatrixXd>(H_from_l_sole_to_root.R.data,3,3);
    Eigen::Vector3d l_foot_ang = Eigen::Vector3d::Zero();
    getEulerAngles(l_sole_ort, l_foot_ang, ort_order);
    std::cout << "l_sole rot wrt root: " << l_foot_ang.transpose() << std::endl;
    for(int i = 0; i < N; i++)
    {
        com_l_sole[i] = l_sole_ort*com[i] + l_sole_pos;
    }
    
    // write out the resulting joint trajectories for meshup visualization and for the robot
    writeOnCSV(time_vec_new,res,m_outputDir + "/test_ik_pg_meshup.csv","");//meshup_header);
    yInfo("Wrote MESHUP file: %s ", std::string(m_outputDir + "/test_ik_pg_meshup.csv").c_str());
    writeOnCSV(res_deg_cut,m_outputDir + "/test_ik_pg.csv");
    yInfo("Wrote MESHUP file: %s ", std::string(m_outputDir + "/test_ik_pg.csv").c_str());
    writeOnCSV(com_real,m_outputDir + "/real_com_traj.csv");
    yInfo("Wrote MESHUP file: %s ", std::string(m_outputDir + "/real_com_traj.csv").c_str());
    writeOnCSV(com_l_sole,m_outputDir + "/com_l_sole.csv");
    yInfo("Wrote MESHUP file: %s ", std::string(m_outputDir + "/com_l_sole.csv").c_str());
    yInfo("Waiting 10 seconds before restarting ... " );
    yarp::os::Time::delay(10.0);

}

void iCubWalkingIKThread::threadRelease() {

}
