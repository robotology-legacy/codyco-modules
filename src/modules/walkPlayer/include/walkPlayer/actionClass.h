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

    /** \brief Opens and parses a trajectory file.
      * \param filename Name of the txt file without the _left or _right suffix. E.g. seq02_leg

      * This method opens a file with a sequence of lines defining a walking trajectory.
      * For each limb of the robot involved in the trajectory a file must be created, e.g. seq02_leg_left.txt would be the sequence file for the left leg. The same must be done for every limb involved in the trajectory and \param filename corresponds to the suffix seq02_leg. This method is used when param filename2 is passed to the walkPlayer module. */
    bool openFile2(std::string filename, yarp::os::ResourceFinder &rf);

    bool openFile(std::string filename);

    /** \brief Trajectory file parser.
     *  \param trajectory_line1 A single line from the left leg trajectory file.
     *  \param trajectory_line2 A single line from the right leg trajectory file.
     *  \param trajectory_line3 A single line from the torso trajectory line.
     *  \param line UNUSED.
     *
     * When using openFile2, the method parseCommandLine2 is used to parse each line of the file opened by openFile2
     */
    bool parseCommandLine2(char* command_line1, char* command_line2, char* command_line3, int line);

    bool parseCommandLine(char* command_line, int line);
};

#endif
