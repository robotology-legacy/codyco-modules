/* 
 * Copyright (C) 2013 CoDyCo
 * Author: Andrea Del Prete
 * email:  andrea.delprete@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <locomotionPlanner/locomotionPlannerThread.h>
#include <locomotion/locomotionConstants.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>


using namespace locomotion;
using namespace locomotionPlanner;


LocomotionPlannerThread::LocomotionPlannerThread(string _name, string _robotName, ParamHelperServer *_ph, ParamHelperClient *_lc, wholeBodyInterface *_wbi, string _filename)
    :  name(_name), robotName(_robotName), paramHelper(_ph), locoCtrl(_lc), robot(_wbi), filename(_filename)
{
    mustStop = false;
    codyco_root = "CODYCO_ROOT";

}

//*************************************************************************************************************************
bool LocomotionPlannerThread::threadInit()
{
    // For parsing input parameters file
    if( filename.length() == 0 ) {
        filename = get_env_var(codyco_root).c_str();
        string temp = "/locomotionPlanner/conf/data/timestamp10/randomStandingPoses_iCubGenova01_100poses_A.txt";        // PROBABLY IT WOULD BE BEST TO PUT THIS FILE IN ANOTHER FOLDER
        temp = get_env_var(codyco_root)+temp;
        filename = temp;
    }
    cout << "Found params config file: " << filename << endl;

    // resize vectors that are not fixed-size

    // link module rpc parameters to member variables

    // link controller input streaming parameters to member variables
    YARP_ASSERT(locoCtrl->linkParam(PARAM_ID_SUPPORT_PHASE,       &supportPhase));
    YARP_ASSERT(locoCtrl->linkParam(PARAM_ID_XDES_COM,            xd_com.data()));
    YARP_ASSERT(locoCtrl->linkParam(PARAM_ID_XDES_FOOT,           xd_foot.data()));
    YARP_ASSERT(locoCtrl->linkParam(PARAM_ID_QDES,                qd.data()));
    // link module output streaming parameters to member variables

    // Register callbacks for some module parameters

    // Register callbacks for some module commands
    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_START,           this));
    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_STOP,            this));
    
    return true;
}

//*************************************************************************************************************************
void LocomotionPlannerThread::run()
{

    ifstream file(filename.c_str(), ifstream::in);

    //get number of lines in file
    int mycount = count(istreambuf_iterator<char>(file), istreambuf_iterator<char>(),'\n');
    int lineNumber = mycount;
    //reset file status
    file.clear();
    file.seekg(0,ios::beg);                         // returns to the beginning of fstream
    double  timePrev = 0.0;

    if(!file.fail()){
        while(lineNumber)
        {
            Matrix<double,1,36> paramLine;
            int j=0;

            //  read one line at a time from text file
            string line = readParamsFile(file);
            istringstream iss(line);

            while(iss && j<=35)
            {
                double  sub;
                iss  >> sub;
                paramLine(0,j) = sub;
                j++;
            };

            /* At this point paramLines has the current line of data in the following order
            <time> <support_phase> <pos_com_desired> <pos_foot_desired> <joint_desired> */
//             cout << "paramLine: "  << paramLine << endl;

            // updating parameters
            Matrix<double,1,1> tmp = paramLine.segment(1,1);    //extracting support phase which is integer
            unsigned int tmp2 = (unsigned int)tmp(0,0);
            supportPhase = tmp2;
            xd_com       = paramLine.segment(2,2); // segment(position,size) It doesn't modify the original Eigen vector
            xd_foot      = paramLine.segment(4,7);
            qd           = paramLine.segment(11,ICUB_DOFS);

            lineNumber--;

            double timeStep = paramLine(0,0) - timePrev;
            timePrev = paramLine(0,0);

            locoCtrl->sendStreamParams();
            Time::delay(timeStep);
        }
    }
    else
    {
        fprintf(stderr,"INPUT PARAMETERS FILE NOT FOUND /n");
    }

    file.close();
}

//*************************************************************************************************************************
void LocomotionPlannerThread::threadRelease()
{

}

//*************************************************************************************************************************
void LocomotionPlannerThread::parameterUpdated(const ParamDescription &pd)
{
    //switch(pd.id)
    //{
    //default:
    sendMsg("A callback is registered but not managed for the parameter "+pd.name, MSG_WARNING);
    //}
}

//*************************************************************************************************************************
void LocomotionPlannerThread::commandReceived(const CommandDescription &cd, const Bottle &params, Bottle &reply)
{
    switch(cd.id)
    {
    case COMMAND_ID_START:
        sendMsg("Starting the planner.", MSG_INFO); break;
    case COMMAND_ID_STOP:
        sendMsg("Stopping the planner.", MSG_INFO); break;
    default:
        sendMsg("A callback is registered but not managed for the command "+cd.name, MSG_WARNING);
    }
}

//*************************************************************************************************************************
string LocomotionPlannerThread::readParamsFile(ifstream& fp)
{
    string lineStr;
    getline(fp,lineStr);
    return(lineStr);
}
//*************************************************************************************************************************
string LocomotionPlannerThread::get_env_var( string const & key ) {
    char * val;
    val = getenv( key.c_str() );
    std::string retval = "";
    if (val != NULL) {
        retval = val;
    }
    return retval;
}
//*************************************************************************************************************************
void LocomotionPlannerThread::sendMsg(const string &s, MsgType type)
{
    if(type>=MSG_DEBUG)
        printf("[LocomotionPlannerThread] %s\n", s.c_str());
}
