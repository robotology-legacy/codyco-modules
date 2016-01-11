#ifndef ACTIONCLASS_H
#define ACTIONCLASS_H


#include <stdio.h>
#include <vector>
#include <string.h>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <stdlib.h>
#include <yarp/os/RFModule.h>
#include <assert.h>
#include <deque>

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

struct actionStructForTorqueBalancing
{
    std::deque< std::deque<double> >  com_traj;
    std::deque< std::deque<double> >  postural_traj;
    std::deque< std::deque<double> >     constraints;
    
public:
    actionStructForTorqueBalancing();
};

class actionClass
{
    public:
    size_t                          current_action;
    int                             current_status;
    std::vector <actionStruct>      action_vector;
    actionStructForTorqueBalancing  action_vector_torqueBalancing;

    /**
     *  Class containing data structures for the different types of data parsed by this module. When playing back walking trajectories in position mode, the parsed data is stored in the variable action_vector, which will contain the trajectories for both legs and the torso. Instead, when playing back trajectories suitable for the torqueBalancing module, the parsed files are stored in the variable action_vector_torqueBalancing.
     */
    actionClass();

    /** \brief Opens and parses a trajectory file.
      * \param filename Name of the txt file without the _left or _right suffix. E.g. seq02_leg

      * This method opens a file with a sequence of lines defining a walking trajectory.
      * For each limb of the robot involved in the trajectory a file must be created, e.g. seq02_leg_left.txt would be the sequence file for the left leg. The same must be done for every limb involved in the trajectory and filename corresponds to the suffix seq02_leg. This method is used when param filename2 is passed to the walkPlayer module. */
    bool openFile(std::string filename, yarp::os::ResourceFinder &rf);

    /**
     *  Opens and parses a trajectory file specific for the torqueBalancing. When this file is
     *  specified, this module will simply stream these trajectories through ports. Therefore,
     *  options "execute" and "torqueBalancingSequence" are mutually exclusive.
     *
     *  @param filenamePrefix         Stem of the files generated for the torqueBalancing module.
     *  @param rf                     Reference to the initialized resource finder object.
     *  @param comTrajSuffix          <#"comTraj" description#>
     *  @param formatComTraj          <#"" description#>
     *  @param posturalTrajSuffix     <#"posturalTraj" description#>
     *  @param formatPosturalTraj     <#"" description#>
     *  @param constraintsTrajSuffix  <#"constraints" description#>
     *  @param formatConstraintsTraj  <#"" description#>
     *
     *  @return true when successful, false otherwise.
     */
    bool openTorqueBalancingSequence(std::string filenamePrefix,
                                     yarp::os::ResourceFinder &rf,
                                     std::string comTrajSuffix = "comTraj",
                                     std::string formatComTraj = "",
                                     std::string posturalTrajSuffix = "posturalTraj",
                                     std::string formatPosturalTraj = "",
                                     std::string constraintsTrajSuffix = "constraints",
                                     std::string formatConstraintsTraj = "");
    /**
     *  Reads already existing trajectory files to be used by torqueBalancing and stores the retrieved com, postural and constraints trajectories in variable action_vector_torqueBalancing.
     *
     *  @param filenamePrefix Prefix of the existing files e.g. the part torqueBalancing in torqueBalancing_comTraj.txt.
     *  @param filenameSuffix Suffix of the existing files e.g. the part comTraj in torqueBalancing_comTraj.txt.
     *  @param partID         ID of the part to be parsed as defined in constants.h. e.g. COM_ID, POSTURAL_ID, CONSTRAINTS_ID.
     *  @param format         UNUSED. This could be used by sscanf, but it appeared unusable when the file has too many columns.
     *  @param rf             Reference to the resourceFinder object of the containing yarp module.
     *
     *  @return true if successful, false otherwise.
     */
    bool parseTorqueBalancingSequences(std::string               filenamePrefix,
                                       std::string               filenameSuffix,
                                       int                       partID,
                                       std::string               format,
                                       yarp::os::ResourceFinder  &rf);
    
    /** \brief Trajectory file parser.
     *  \param command_line1 A single line from the left leg trajectory file.
     *  \param command_line2 A single line from the right leg trajectory file.
     *  \param command_line3 A single line from the torso trajectory line.
     *  \param line UNUSED.
     */
    bool parseCommandLine(char* command_line1, char* command_line2, char* command_line3, int line);
    
};

#endif
