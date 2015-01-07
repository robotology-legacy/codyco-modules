#ifndef ACTIONCLASS_H
#define ACTIONCLASS_H


#include <stdio.h>
#include <vector>
#include <string>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <yarp/os/RFModule.h>

#include "constants.h"


// ******************** ACTION CLASS
struct actionStruct
{
    int         counter;
    double      time;
    double      q_left_leg  [6];
    double      q_right_leg [6];
    double      q_torso     [3];
    std::string tag;

    public:
    actionStruct();
};

class actionClass
{
    public:
    size_t                     current_action;
    int                        current_status;
    std::vector <actionStruct> action_vector;
   // bool parseCommandLine(char* command_line, int line);
   // bool parseCommandLine2(char* command_line1, char* command_line2, int line);

    actionClass();
    bool openFile2(std::string filename, yarp::os::ResourceFinder &rf);
    bool openFile(std::string filename);
    bool parseCommandLine2(char* command_line1, char* command_line2, int line);
    bool parseCommandLine(char* command_line, int line);
};

#endif
