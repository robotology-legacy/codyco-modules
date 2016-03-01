#include "iCubWalkingIKThread.h"

iCubWalkingIKThread::iCubWalkingIKThread ( int period,
                                           wbi::iWholeBodyModel* wbm,
                                           wbi::iWholeBodyStates* wbs,
                                           walkingParams params,
                                           yarp::os::ResourceFinder& rf,
                                           std::string walkingPatternFile) :
RateThread(period),
m_period(period),
m_walkingPatternFile(walkingPatternFile),
m_walkingParams(params),
m_rf(rf)
{

}

bool iCubWalkingIKThread::threadInit() {
    // Generate feet trajectories
    generateFeetTrajectories(m_walkingPatternFile, m_walkingParams);

    return true;
}

void iCubWalkingIKThread::run() {

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
    std::vector<RigidBodyDynamics::Math::VectorNd> r_foot_traj(N_traj);
    std::vector<RigidBodyDynamics::Math::VectorNd> l_foot_traj(N_traj);
    std::vector<RigidBodyDynamics::Math::VectorNd> com_traj(N_traj);
    
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

void iCubWalkingIKThread::threadRelease() {

}
