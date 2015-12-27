#include "actionClass.h"

using namespace std;

// ******************** ACTION CLASS
actionStruct::actionStruct() {
        for (int i = 0; i < 6; i++) {
            actionStruct::q_left_leg[i] = actionStruct::q_right_leg[i] = 0.0;
        }
        for (int i = 0; i < 3; i++) {
            actionStruct::q_torso[i] = actionStruct::q_torso[i] = 0.0;
        }
        actionStruct::tag = "UNKNOWN";
}

actionStructForTorqueBalancing::actionStructForTorqueBalancing()
{
//    for (int i = 0; i < COM_TRAJ_NUM_COLS; i++)
//        actionStructForTorqueBalancing::com_traj[i] = 0.0;
//    for (int i = 0; i < POSTURAL_TRAJ_NUM_COLS; i++)
//        actionStructForTorqueBalancing::postural_traj[i] = 0.0;
//    for (int i = 0; i < CONSTRAINTS_NUM_COLS; i++)
//        actionStructForTorqueBalancing::constraints[i] = 0.0;
}

actionClass::actionClass()
{
    current_action = 0;
    current_status = ACTION_IDLE;
}

bool actionClass::openFile(std::string filename, yarp::os::ResourceFinder &rf)
{
    bool ret = true;
    FILE* data_file1 = 0;
    FILE* data_file2 = 0;
    FILE* data_file3 = 0;
    string filename_left  = filename + "_left" + ".txt";
    string filename_right = filename + "_right" + ".txt";
    string filename_torso = filename + "_torso" + ".txt";
    filename_left  = rf.findFile(filename_left);
    filename_right = rf.findFile(filename_right);
    filename_torso = rf.findFile(filename_torso);
    fprintf(stderr, "||| File found for left leg: %s\n", filename_left.c_str());
    fprintf(stderr, "||| File found for right leg: %s\n", filename_right.c_str());
    fprintf(stderr, "||| File found for torso: %s\n", filename_torso.c_str());
    data_file1 = fopen(filename_left.c_str(),"r");
    data_file2 = fopen(filename_right.c_str(),"r");
    data_file3 = fopen(filename_torso.c_str(),"r");

    if (data_file1 != NULL && data_file2 != NULL && data_file3 != NULL)
    {
        char* bb1 = 0;
        char* bb2 = 0;
        char* bb3 = 0;
        int   line =0;
        do
        {
            char trajectory_line1[1024];
            char trajectory_line2[1024];
            char trajectory_line3[1024];
            bb1 = fgets (trajectory_line1, 1024, data_file1);
            bb2 = fgets (trajectory_line2, 1024, data_file2);
            bb3 = fgets (trajectory_line3, 1024, data_file3);
            if (bb1 == 0 || bb2 == 0 || bb3 == 0) break;
            if(!parseCommandLine(trajectory_line1, trajectory_line2, trajectory_line3, line++))
                {
                    printf ("error parsing file, line %d\n", line);
                    ret = false;
                    break;
                };
        }
        while (1);

        fclose (data_file1);
        fclose (data_file2);
        fclose (data_file3);
    }
    else
    {
        //file not opened
        ret = false;
    }
    return ret;
}

bool actionClass::openTorqueBalancingSequence(std::string filenamePrefix,
                                              yarp::os::ResourceFinder &rf,
                                              std::string comTrajSuffix,
                                              std::string formatComTraj,
                                              std::string posturalTrajSuffix,
                                              std::string formatPosturalTraj,
                                              std::string constraintsTrajSuffix,
                                              std::string formatConstraintsTraj)
{
    bool ret = false;
    // Retrieve com data
    ret = parseTorqueBalancingSequences(filenamePrefix, comTrajSuffix, COM_ID, formatComTraj, rf);
    // Retrieve postural data
    ret = ret && parseTorqueBalancingSequences(filenamePrefix, posturalTrajSuffix, POSTURAL_ID, formatPosturalTraj, rf);
    // Retrieve constraints data
    ret = ret && parseTorqueBalancingSequences(filenamePrefix, constraintsTrajSuffix, CONSTRAINTS_ID, formatConstraintsTraj, rf);
    cout << "All trajectories retrieved correctly" << endl;
    cout << "Size of com_traj = " << action_vector_torqueBalancing.com_traj.size() << endl;
    cout << "Size of postural_traj = " << action_vector_torqueBalancing.postural_traj.size() << endl;
    cout << "Size of constraints = " << action_vector_torqueBalancing.constraints.size() << endl;
    assert(this->action_vector_torqueBalancing.com_traj.size() == this->action_vector_torqueBalancing.postural_traj.size());
    assert(this->action_vector_torqueBalancing.com_traj.size() == this->action_vector_torqueBalancing.constraints.size());
    return ret;
}

bool actionClass::parseTorqueBalancingSequences(std::string              filenamePrefix,
                                                std::string              filenameSuffix,
                                                int                      partID,
                                                std::string              format,
                                                yarp::os::ResourceFinder &rf)
{
    bool ret = false;
    actionStructForTorqueBalancing tmp_action;
    string filename  = filenamePrefix + "_" + filenameSuffix + ".txt";
    filename  = rf.findFile(filename);
    fprintf(stderr, "[!!!] File found for %s: %s\n", filenameSuffix.c_str(), filename.c_str());
    // Open file
    ifstream data_file( filename.c_str() );
    string line;
    std::deque<double> tmp_com;
    std::deque<double> tmp_postural;
    std::deque<int>    tmp_constraints;

    while( std::getline(data_file, line) )
    {
        int col = 0;
        std::istringstream iss( line );
        std::string result;
        tmp_com.clear();
        tmp_postural.clear();
        tmp_constraints.clear();
        while( std::getline( iss, result , ' ') )
        {
            if ( strcmp(result.c_str(),"") != 0 )
            {
                std::stringstream convertor;
                convertor.clear();
                convertor.str(result);
                if (partID == COM_ID)
                {
                    tmp_com.push_back(atof(result.c_str()));
                }
                if (partID == POSTURAL_ID)
                    tmp_postural.push_back(atof(result.c_str()));
                if (partID == CONSTRAINTS_ID)
                    tmp_constraints.push_back(atoi(result.c_str()));
                col++;
            }
        }
        if (partID == COM_ID)
            action_vector_torqueBalancing.com_traj.push_back(tmp_com);
        if (partID == POSTURAL_ID)
            action_vector_torqueBalancing.postural_traj.push_back(tmp_postural);
        if (partID == CONSTRAINTS_ID)
            action_vector_torqueBalancing.constraints.push_back(tmp_constraints);
    }
    
    data_file.close();
    ret = true;
    return ret;
}

bool actionClass::parseCommandLine(char* command_line1, char* command_line2, char* command_line3, int line)
{
        actionStruct tmp_action;
        double tmp_double = 0;
        int ret1 = sscanf(command_line1, "%lf %lf    %lf %lf %lf %lf %lf %lf  ",
        &tmp_double,
        &tmp_action.time,

        &tmp_action.q_left_leg[0],
        &tmp_action.q_left_leg[1],
        &tmp_action.q_left_leg[2],
        &tmp_action.q_left_leg[3],
        &tmp_action.q_left_leg[4],
        &tmp_action.q_left_leg[5]
        );

//         cout << "actionClass::parseCommandLine2 says:  Counter: " << tmp_action.counter << " Time: " << tmp_action.time << endl;
//         cout << "q_left_leg: \n";
//         for (int i=0 ; i<6; i++)
//             cout << tmp_action.q_left_leg[i] << "  ";
//         cout << endl;

        int ret2 = sscanf(command_line2, "%lf %lf    %lf %lf %lf %lf %lf %lf  ",
        &tmp_double,
        &tmp_action.time,

        &tmp_action.q_right_leg[0],
        &tmp_action.q_right_leg[1],
        &tmp_action.q_right_leg[2],
        &tmp_action.q_right_leg[3],
        &tmp_action.q_right_leg[4],
        &tmp_action.q_right_leg[5]
        );

        // TODO Third column repeats the second one!! This must be removed
        int ret3 = sscanf(command_line3, "%lf %lf %lf %lf %lf",
        &tmp_double,
        &tmp_action.time,
        &tmp_action.q_torso[0],
        &tmp_action.q_torso[1],
        &tmp_action.q_torso[2]
        );

        tmp_action.counter = static_cast<int> (tmp_double);

        // TODO ret3 should be 5 instead of 6
        if (ret1 == 8 && ret2 == 8 && ret3 == 5)
        {
            action_vector.push_back(tmp_action);
            return true;
        }

        return false;
}
