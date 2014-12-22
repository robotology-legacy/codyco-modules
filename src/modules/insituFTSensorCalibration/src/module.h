/*
* Copyright (C) 2014 ...
* Author: ...
* email: ...
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

#ifndef INSITU_FT_CALIBRATION_MODULE_H
#define INSITU_FT_CALIBRATION__MODULE_H

#include <iostream>
#include <string>
#include <vector>
#include <map>

#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/RateThread.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlLimits2.h>

#include <yarp/sig/Vector.h>

#include <wbi/iWholeBodySensors.h>
#include <iCub/ctrl/filters.h>

#include <InSituFTCalibration/force_calibration_matrix_estimator.h>
#include <InSituFTCalibration/offset_estimator.h>

#include "insituFTSensorCalibration_IDLServer.h"

#include <yarpWholeBodyInterface/yarpWholeBodySensors.h>


using namespace std;
using namespace yarp::os;

class FTCalibrationDataset
{
public:
    FTCalibrationDataset();
    ~FTCalibrationDataset();

    bool fromBottle(const yarp::os::Bottle & bot);
    std::string dataset_name;
    double added_mass;
};

struct controlledJoint
{
public:
    std::string part_name;
    int axis_number;
    double lower_limit;
    double upper_limit;
    double delta;
};

class desiredPositions
{
public:
    yarp::sig::Vector pos;
    double waiting_time;
    bool is_return_point;
    desiredPositions(yarp::sig::Vector _pos, double _waiting_time, bool _is_return_point=false):
    pos(_pos), waiting_time(_waiting_time), is_return_point(_is_return_point)
    {}
};

class insituFTSensorCalibrationModule;

enum FTSensorCalibrationStatus
{
  COLLECTING_DATASET,
  WAITING_NEW_DATASET_START
};

class insituFTRpcHandler : public insituFTSensorCalibration_IDLServer
{
private:
    insituFTSensorCalibrationModule & parent;

public:
    insituFTRpcHandler(insituFTSensorCalibrationModule & _parent);

    ~insituFTRpcHandler();

    bool startNewDatasetAcquisition();

    bool quit();
};

class insituFTSensorCalibrationThread: public RateThread
{
private:
    FTSensorCalibrationStatus status;

    InSituFTCalibration::ForceCalibrationMatrixEstimator estimator;



    wbi::iWholeBodySensors * sensors;
    yarp::os::ResourceFinder & rf;
    yarp::os::Mutex threadMutex;

    int currentDataset;
    std::vector<FTCalibrationDataset> training_datasets;
    std::vector<InSituFTCalibration::ForceTorqueOffsetEstimator *> estimator_datasets;

    double cutOffFrequency;

    std::vector<iCub::ctrl::FirstOrderLowPassFilter *> ft_filters;
    std::vector<yarp::sig::Vector> raw_ft;
    std::vector<yarp::sig::Vector> smooth_ft;

    std::vector<iCub::ctrl::FirstOrderLowPassFilter *> acc_filters;
    std::vector<yarp::sig::Vector> raw_acc;
    std::vector<yarp::sig::Vector> smooth_acc;

public:
    /**
     *
     * @param cutOffFrequency frequency of the first order low pass filter used for fitlering the input data [Hz]
     * @param period          update period of the RateThread, passed to the setRate method of the thread [ms]
     */
    insituFTSensorCalibrationThread(wbi::iWholeBodySensors * _sensors,
                                    yarp::os::ResourceFinder & _rf
    );

    bool threadInit();

    void run();

    void threadRelease();

    bool startDatasetAcquisition();

    bool getCurrentDataset(std::string & dataset_name, int & dataset_int);

    int getNrOfTrainingDatasets();

    bool stopDatasetAcquisition();

    bool getCalibration(yarp::sig::Matrix & mat);
    bool writeCalibrationToFile(std::string filename);

};

class insituFTSensorCalibrationModule: public RFModule
{
    /* module parameters */
    string moduleName;
    string robotName;
    double period;
    double avgTime, stdDev, avgTimeUsed, stdDevUsed;

    enum { GRID_VISIT, GRID_MAPPING_WITH_RETURN } mode;
    FTSensorCalibrationStatus status;
    
    /** estimation thread */
    insituFTSensorCalibrationThread * estimation_thread;
    
    /** sensor interface */
    yarpWbi::yarpWholeBodySensors      * sensors;

    /* RPC handling */
    insituFTRpcHandler rpc_handler;
    yarp::os::Port  rpcPort;        // a port to handle rpc messages

    yarp::os::Mutex moduleMutex;

    // Motor excitation stuff
    bool boringModeInitialized;

    double static_pose_period;
    double return_point_waiting_period;
    double elapsed_time; // time passed from when the desired pose was reached
    double ref_speed;

    std::vector< controlledJoint > controlledJoints;
    yarp::sig::Vector commandedPositions;
    double desired_waiting_time;
    std::vector<desiredPositions> listOfDesiredPositions;
    yarp::os::BufferedPort<yarp::os::Bottle> isTheRobotInReturnPoint;
    bool is_desired_point_return_point;

    int next_desired_position;
    yarp::sig::Vector originalPositions;
    yarp::sig::Vector originalRefSpeeds;
    bool jointInitialized;

    std::map<std::string,yarp::dev::PolyDriver *> drivers;
    std::map<std::string,yarp::dev::IPositionControl *> pos;
    std::map<std::string,yarp::dev::IEncoders *>encs;
    std::map<std::string,yarp::dev::IControlLimits *>lims;

    void close_drivers();

    // Dataset stuff

public:
    insituFTSensorCalibrationModule();

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule(); // interrupt, e.g., the ports
    bool close(); // close and shut down the module
    double getPeriod();
    bool getNewDesiredPosition(yarp::sig::Vector & desired_pos, double & desired_parked_time, bool & is_return_point);
    bool updateModule();

    // Rpc methods
    bool startNewDatasetAcquisition();

};


#endif
//empty line to make gcc happy
