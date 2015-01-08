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

actionClass::actionClass()
{
    current_action = 0;
    current_status = ACTION_IDLE;
}

bool actionClass::openFile2(std::string filename, yarp::os::ResourceFinder &rf)
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
            if(!parseCommandLine2(trajectory_line1, trajectory_line2, trajectory_line3, line++))
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

bool actionClass::openFile(std::string filename)
{
    bool ret = true;
    FILE* data_file = 0;
    data_file = fopen(filename.c_str(),"r");
    if (data_file!=NULL)
    {
        char* bb = 0;
        int   line =0;
        do
        {
            char command_line[1024];
            bb = fgets (command_line, 1024, data_file);
            if (bb == 0) break;
            if(!parseCommandLine(command_line, line++))
                {
                    printf ("error parsing file, line %d\n", line);
                    ret = false;
                    break;
                };
        }
        while (1);

        fclose (data_file);
    }
    return ret;
}

bool actionClass::parseCommandLine2(char* command_line1, char* command_line2, char* command_line3, int line)
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
        double trash = 0;
        int ret3 = sscanf(command_line3, "%lf %lf %lf    %lf %lf %lf  ",
        &tmp_double,
        &tmp_action.time,

        &trash,

        &tmp_action.q_torso[0],
        &tmp_action.q_torso[1],
        &tmp_action.q_torso[2]
        );

        tmp_action.counter = static_cast<int> (tmp_double);

        // TODO ret3 should be 5 instead of 6
        if (ret1 == 8 && ret2 == 8 && ret3 == 6)
        {
            action_vector.push_back(tmp_action);
            return true;
        }

        return false;
}

bool actionClass::parseCommandLine(char* command_line, int line)
{
        actionStruct tmp_action;
        int ret = sscanf(command_line, "%d %lf    %lf %lf %lf %lf %lf %lf    %lf %lf %lf %lf %lf %lf",
        &tmp_action.counter,
        &tmp_action.time,

        &tmp_action.q_left_leg[0],
        &tmp_action.q_left_leg[1],
        &tmp_action.q_left_leg[2],
        &tmp_action.q_left_leg[3],
        &tmp_action.q_left_leg[4],
        &tmp_action.q_left_leg[5],

        &tmp_action.q_right_leg[0],
        &tmp_action.q_right_leg[1],
        &tmp_action.q_right_leg[2],
        &tmp_action.q_right_leg[3],
        &tmp_action.q_right_leg[4],
        &tmp_action.q_right_leg[5]);

        if (ret == 14)
        {
            action_vector.push_back(tmp_action);
            return true;
        }

        return false;
}
