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

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Random.h>
#include <yarp/os/LockGuard.h>
#include <yarp/os/Log.h>

#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <yarp/os/idl/WireTypes.h>

#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>

#include <cstdio>
#include <string.h>

#include <cmath>

#include <InSituFTCalibration/yarp_wrappers.h>

#include "module.h"

YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::dev;

insituFTRpcHandler::insituFTRpcHandler(insituFTSensorCalibrationModule& _parent): parent(_parent)
{

}

insituFTRpcHandler::~insituFTRpcHandler()
{

}


bool insituFTRpcHandler::startNewDatasetAcquisition()
{
    return parent.startNewDatasetAcquisition();
}

bool insituFTRpcHandler::quit()
{
    parent.stopModule();
    return true;
}

FTCalibrationDataset::FTCalibrationDataset()
{

}


FTCalibrationDataset::~FTCalibrationDataset()
{

}

bool FTCalibrationDataset::fromBottle(const yarp::os::Bottle &bot)
{
    if( bot.size() != 2
        || !(bot.get(0).isString())
        || !(bot.get(1).isList())
        || !(bot.get(1).asList()->size() == 2)
        || !(bot.get(1).asList()->get(0).isString())
        || !(bot.get(1).asList()->get(0).asString() == "mass")
        || !(bot.get(1).asList()->get(1).isDouble())
    )
    {
        return false;
    }

    dataset_name = bot.get(0).asString();
    added_mass   = bot.get(1).asList()->get(1).asDouble();
    return true;
}

insituFTSensorCalibrationThread::insituFTSensorCalibrationThread(wbi::iWholeBodySensors* _sensors,
                                                                 ResourceFinder& _rf): RateThread(1000),
                                                                                       sensors(_sensors),
                                                                                       rf(_rf),
                                                                                       status(WAITING_NEW_DATASET_START)
{
}

bool insituFTSensorCalibrationThread::threadInit()
{
    int nrOfFTSensors=1;
    int nrOfAccelerometers=1;

    //Load datasets
    yarp::os::Bottle & bot = rf.findGroup("training_datasets");
    int nrOfDatasets = bot.size()-1;

    for(int dataset = 0; dataset < nrOfDatasets; dataset++)
    {
        FTCalibrationDataset ft_dataset;
        if( !(bot.get(dataset+1).isList()) )
        {
            std::cerr << "[ERR] insituFTSensorCalibrationThread: error in loading training_datasets group " << std::endl;
            return false;
        }
        if( !(ft_dataset.fromBottle(*(bot.get(dataset+1).asList()))) )
        {
            std::cerr << "[ERR] insituFTSensorCalibrationThread: error in loading training_datasets group " << std::endl;
            return false;
        }
        training_datasets.push_back(ft_dataset);
        estimator_datasets.push_back(0);
    }

    double estimation_period = rf.check("estimation_period",yarp::os::Value(20.0),"Period (in ms) of sensor readings").asDouble();
    setRate(estimation_period);
    cutOffFrequency = rf.check("estimation_cutoff_frequency",yarp::os::Value(0.3),"Cutoff frequency (in Hz) of the first order filter used to filter measurements").asDouble();

    acc_filters.resize(nrOfAccelerometers,0);
    raw_acc.resize(nrOfAccelerometers,yarp::sig::Vector(3,0.0));
    smooth_acc.resize(nrOfAccelerometers,yarp::sig::Vector(3,0.0));
    for(int acc=0; acc < nrOfAccelerometers; acc++)
    {
        acc_filters[acc] = new iCub::ctrl::FirstOrderLowPassFilter(cutOffFrequency,getRate()*1000.0,raw_acc[acc]);
    }

    ft_filters.resize(nrOfFTSensors,0);
    raw_ft.resize(nrOfFTSensors,yarp::sig::Vector(6,0.0));
    smooth_ft.resize(nrOfFTSensors,yarp::sig::Vector(6,0.0));
    for(int ft=0; ft < nrOfFTSensors; ft++)
    {
        ft_filters[ft] = new iCub::ctrl::FirstOrderLowPassFilter(cutOffFrequency,getRate()*1000.0,raw_ft[ft]);
    }


    return true;
}

bool insituFTSensorCalibrationThread::startDatasetAcquisition()
{
    yarp::os::LockGuard guard(threadMutex);

    if( status != WAITING_NEW_DATASET_START )
    {
        return false;
    }

    status = COLLECTING_DATASET;
    currentDataset++;
    if( currentDataset < 0 && currentDataset >= getNrOfTrainingDatasets() )
    {
        currentDataset--;
        return false;
    }

    estimator.addCalibrationDataset(training_datasets[currentDataset].dataset_name,
                                    training_datasets[currentDataset].added_mass);

    estimator.getCalibrationDataset(training_datasets[currentDataset].dataset_name,
                                    estimator_datasets[currentDataset]);

    return true;
}

bool insituFTSensorCalibrationThread::stopDatasetAcquisition()
{
    yarp::os::LockGuard guard(threadMutex);
    status = WAITING_NEW_DATASET_START;
    return true;
}

void insituFTSensorCalibrationThread::run()
{
    yarp::os::LockGuard guard(threadMutex);

    if( status == COLLECTING_DATASET )
    {
        double * p_timestamp = 0;
        bool blocking = false;
        sensors->readSensor(wbi::SENSOR_ACCELEROMETER,0,raw_acc[0].data(),p_timestamp,blocking);
        sensors->readSensor(wbi::SENSOR_FORCE_TORQUE,0,raw_ft[0].data(),p_timestamp,blocking);

        smooth_acc[0] = acc_filters[0]->filt(raw_acc[0]);
        smooth_ft[0] = ft_filters[0]->filt(raw_ft[0]);

        estimator_datasets[currentDataset]->addMeasurements(InSituFTCalibration::wrapVec(smooth_ft[0]),InSituFTCalibration::wrapVec(smooth_acc[0]));
    }
}



void insituFTSensorCalibrationThread::threadRelease()
{
    yarp::os::LockGuard guard(threadMutex);

    for(int acc=0; acc < acc_filters.size(); acc++)
    {
        if( acc_filters[acc] )
        {
            delete acc_filters[acc];
            acc_filters[acc] = 0;
        }
    }

    for(int ft=0; ft < ft_filters.size(); ft++)
    {
        if( ft_filters[ft] )
        {
            delete ft_filters[ft];
            ft_filters[ft] = 0;
        }
    }

}

bool insituFTSensorCalibrationThread::getCurrentDataset(string& dataset_name, int& dataset_int)
{
    yarp::os::LockGuard guard(threadMutex);
    bool ret = false;
    if( status == COLLECTING_DATASET )
    {
        dataset_name  = training_datasets[currentDataset].dataset_name;
        dataset_int   = currentDataset;
        ret = true;
    }
    return ret;
}

int insituFTSensorCalibrationThread::getNrOfTrainingDatasets()
{
    yarp::os::LockGuard guard(threadMutex);
    return training_datasets.size();
}

bool insituFTSensorCalibrationThread::getCalibration(yarp::sig::Matrix & mat)
{
    mat.resize(3,6);

    bool ret = true;

    std::string err;
    ret = estimator.computeForceCalibrationMatrixEstimation(err);
    if( !ret )
    {
        yError("insituFTSensorCalibrationThread CalibrationMatrixEstimation failed %s",err.c_str());
        return false;
    }

    return estimator.getEstimatedForceCalibrationMatrix(InSituFTCalibration::wrapMat(mat));
}

/**
 * Function to convert a decimal number into 1.15 hex format
 * Ported from octave function convert_onedotfifteen available
 * in FTsens calibration software.
 *
 */
bool convert_onedotfifteen(const double indecimal, std::string & hex_out)
{
    if(indecimal < -1.0 ||
       indecimal >= 1.0 )
    {
        return false;
    }

    char buf[10];

    double quant = round((pow(2,15))*indecimal); // Quantize to 15 bits and round to the nearest integer;                                            // cast to 2-complement integer
    int16_t quant_int = (int16_t) quant;
    sprintf(buf,"%x",quant_int);   // Convert the decimal number to hex string
    std::string hex_buf = buf;
    //return only the last four hex digits, useful if int16_t is not actually 16bit
    hex_out = hex_buf.substr(hex_buf.size()-4,4);
}

/**
 * Write the calibration matrix to a file suitable to be used by the IIT FTsens sensor.
 * calibration_matrix should map the raw straing gauge values to SI (Newton,NewtonMeters)
 * units.
 *
 * Full scale vector should contain the desired full scales in SI units.
 */
bool writeFTsensCalibrationToFile(std::string         filename,
                                  yarp::sig::Matrix & calibration_matrix,
                                  yarp::sig::Vector & full_scale)
{
    if( calibration_matrix.rows() != 6 ||
        calibration_matrix.cols() != 6 ||
        full_scale.size()         != 6 )
    {
        return false;
    }

    std::ofstream myfile;
    myfile.open (filename.c_str());
    if( !myfile.is_open() )
    {
        std::cerr << "[ERROR] Error in writing calibration matrix to " << filename << std::endl;
        return false;
    }

    //It seems that full_scale is required to be an integer, TODO check
    std::vector<int> full_scale_int;
    full_scale_int.resize(6);

    for(int i=0; i < 6; i++ )
    {
        full_scale_int[i] = round(full_scale[i]);
    }

    for(int i=0; i < 6; i++ )
    {
        for(int j=0; j < 6; j++ )
        {
            //The matrix that is passed to the actual sensor use
            //coefficients gains that are encoded with respect to
            //the full scale
            double firmware_coeff = calibration_matrix(i,j)/((double)full_scale_int[i]);
            //Then this firmware coefficient is expressed in 1.15 two complement fixed point way
            //that we encode in the file in hex format TODO CHECK endianess problems
            std::string hex;
            if( !convert_onedotfifteen(firmware_coeff,hex) )
            {
                std::cerr << "[ERROR] Error in writing calibration matrix to file, for the given choice"
                          << " of fullscale the " << i << " " << j << " coefficient is not in [-1.0,1.0]" << std::endl;
                myfile.close();
                return false;
            }

            myfile << hex << "\r\n";
        }
    }

    myfile << 1 << "\r\n";

    for(int i=0; i < 6; i++ )
    {
        myfile << full_scale_int[i] << "\r\n";
    }

    myfile.close();
}


bool insituFTSensorCalibrationThread::writeCalibrationToFile(string filename)
{
    //For now we support only force calibration
    yarp::sig::Matrix force_calibration_matrix(3,6), full_calibration_matrix(6,6);
    if( !getCalibration(force_calibration_matrix) )
    {
        return false;
    }

    full_calibration_matrix.zero();
    for(int i  = 0; i < 3; i++ )
    {
        for(int j=0; j < 6; j++ )
        {
            full_calibration_matrix(i,j) = force_calibration_matrix(i,j);
        }
    }

    //Set sensor full scale (TODO: set it from calibration matrix and ADC fullscale)
    yarp::sig::Vector full_scale(6);
    for(int i =0; i < 3; i++ )
    {
        full_scale[i] = 1000.0;
    }
    for(int i =3; i < 6; i++ )
    {
        full_scale[i] = 20.0;
    }

    return writeFTsensCalibrationToFile(filename,full_calibration_matrix,full_scale);
}


insituFTSensorCalibrationModule::insituFTSensorCalibrationModule(): rpc_handler(*this)
{
    period = 1;
}

void insituFTSensorCalibrationModule::close_drivers()
{
    std::map<string,PolyDriver*>::iterator it;
    if(jointInitialized)
    {
        for(int jnt=0; jnt < originalPositions.size(); jnt++ )
        {
            std::string part = controlledJoints[jnt].part_name;
            int axis = controlledJoints[jnt].axis_number;
            pos[part]->setRefSpeed(axis,originalRefSpeeds[jnt]);
            pos[part]->positionMove(axis,originalPositions[jnt]);
        }
    }
    for(it=drivers.begin(); it!=drivers.end(); it++ )
    {
        if(it->second)
        {
            it->second->close();
            it->second = 0;
        }
    }
}


bool insituFTSensorCalibrationModule::configure(ResourceFinder &rf)
{
    jointInitialized = false;

    std::cout << rf.toString() << std::endl;

    if( rf.check("robot") )
    {
        robotName = rf.find("robot").asString().c_str();
    }
    else
    {
        robotName = "icub";
    }

    std::string mode_cfg = rf.find("mode").asString().c_str();
    if( mode_cfg == "gridVisit" )
    {
        mode = GRID_VISIT;
    }
    else if( mode_cfg == "gridMappingWithReturn" )
    {
        mode = GRID_MAPPING_WITH_RETURN;
    }
    else
    {
        std::cerr << "[ERR] insituFTSensorCalibrationModule: mode " << mode_cfg << "is not available, exiting." << std::endl;
        std::cerr << "[ERR] existing modes: random, gridVisit, gridMappingWithReturn" << std::endl;
    }


    if( rf.check("local") )
    {
        moduleName = rf.find("local").asString().c_str();
    }

    static_pose_period = rf.check("static_pose_period",1.0).asDouble();
    return_point_waiting_period = rf.check("return_point_waiting_period",5.0).asDouble();
    elapsed_time = 0.0;
    ref_speed = rf.check("ref_speed",3.0).asDouble();
    period = rf.check("period",1.0).asDouble();


    if ( !rf.check("joints") )
    {
        fprintf(stderr, "Please specify the name and joints of the robot\n");
        fprintf(stderr, "--robot name (e.g. icub)\n");
        fprintf(stderr, "--joints ((part_name axis_number lower_limit upper_limit) ... )\n");
        return false;
    }

    yarp::os::Bottle & jnts = rf.findGroup("joints");

    if( jnts.isNull() )
    {
        fprintf(stderr, "Please specify the name and joints of the robot\n");
        fprintf(stderr, "--robot name (e.g. icub)\n");
        fprintf(stderr, "--joints ((part_name axis_number lower_limit upper_limit) ... )\n");
        return false;
    }

    int nrOfControlledJoints = jnts.size()-1;

    std::cout << "Found " << nrOfControlledJoints << " joint to control" << std::endl;

    controlledJoints.resize(nrOfControlledJoints);
    commandedPositions.resize(nrOfControlledJoints,0.0);
    originalPositions.resize(nrOfControlledJoints);
    originalRefSpeeds.resize(nrOfControlledJoints);

    for(int jnt=0; jnt < nrOfControlledJoints; jnt++ )
    {
        yarp::os::Bottle * jnt_ptr = jnts.get(jnt+1).asList();
        if( jnt_ptr == 0 || jnt_ptr->isNull() || !(jnt_ptr->size() == 4 || jnt_ptr->size() == 5)  )
        {
            fprintf(stderr, "Malformed configuration file (joint %d)\n",jnt);
            return false;
        }

        std::cout << jnt_ptr->toString() << std::endl;

        controlledJoint new_joint;
        new_joint.part_name = jnt_ptr->get(0).asString().c_str();
        new_joint.axis_number = jnt_ptr->get(1).asInt();
        new_joint.lower_limit = jnt_ptr->get(2).asDouble();
        new_joint.upper_limit = jnt_ptr->get(3).asDouble();
        new_joint.delta = jnt_ptr->get(4).asDouble();

        controlledJoints[jnt] = new_joint;
    }

    for(int jnt=0; jnt < nrOfControlledJoints; jnt++ )
    {
        if( drivers.count(controlledJoints[jnt].part_name) == 1 )
        {
            //parts controlboard already opened, continue
            continue;
        }

        std::string part_name = controlledJoints[jnt].part_name;

        //Open the polydrivers
        std::string remotePort="/";
          remotePort+=robotName;
          remotePort+="/";
          remotePort+= part_name;

        std::string localPort="/"+moduleName+remotePort;
        Property options;
        options.put("device", "remote_controlboard");
        options.put("local", localPort.c_str());   //local port names
        options.put("remote", remotePort.c_str());         //where we connect to

       // create a device
       PolyDriver * new_driver = new PolyDriver(options);
       if( !new_driver->isValid() )
       {
           fprintf(stderr, "[ERR] Error in opening %s part\n",remotePort.c_str());
           close_drivers();
           return false;
       }

       bool ok = true;

       IEncoders * new_encs;
       IPositionControl * new_poss;
       IControlLimits * new_lims;
       ok = ok && new_driver->view(new_encs);
       ok = ok && new_driver->view(new_poss);
       ok = ok && new_driver->view(new_lims);

       if(!ok)
       {
           fprintf(stderr, "Error in opening %s part\n",remotePort.c_str());
           close_drivers();
           return false;
       }

       drivers[part_name] = new_driver;
       pos[part_name] = new_poss;
       encs[part_name] = new_encs;
       lims[part_name] = new_lims;
    }

    for(int jnt=0; jnt < nrOfControlledJoints; jnt++ )
    {
        std::string part = controlledJoints[jnt].part_name;
        int axis = controlledJoints[jnt].axis_number;
        encs[part]->getEncoder(axis,originalPositions.data()+jnt);
        pos[part]->getRefSpeed(axis,originalRefSpeeds.data()+jnt);
        pos[part]->setRefSpeed(axis,ref_speed);
    }

    jointInitialized = true;

    //Configure
    if( mode == GRID_MAPPING_WITH_RETURN || mode == GRID_VISIT )
    {
        is_desired_point_return_point = false;
        listOfDesiredPositions.resize(0,desiredPositions(yarp::sig::Vector(),0.0));
        next_desired_position = 0;
        //Generate vector of desired positions
        if( controlledJoints.size() != 2)
        {
            std::cerr << "GRID_MAPPING_WITH_RETURN mode available only for two controlled joints" << std::endl;
            close_drivers();
            return false;
        }

        yarp::sig::Vector center(2), lower_left(2),lower_right(2),upper_left(2),upper_right(2);
        std::vector<int> semi_nr_of_lines(2,0);
        lower_left[0] = lower_right[0] = controlledJoints[0].lower_limit;
        upper_left[0] = upper_right[0] = controlledJoints[0].upper_limit;
        lower_left[1] = upper_left[1] = controlledJoints[1].lower_limit;
        lower_right[1] = upper_right[0] = controlledJoints[1].upper_limit;
        center[0] =  (controlledJoints[0].lower_limit+controlledJoints[0].upper_limit)/2;
        center[1] =  (controlledJoints[1].lower_limit+controlledJoints[1].upper_limit)/2;
        semi_nr_of_lines[0] = ceil((controlledJoints[0].upper_limit-center[0])/controlledJoints[0].delta);
        semi_nr_of_lines[1] = ceil((controlledJoints[1].upper_limit-center[1])/controlledJoints[1].delta);
        //Start at the center of the workspace
        listOfDesiredPositions.push_back(desiredPositions(center,return_point_waiting_period,true));

        for(int i=0; i < (int)semi_nr_of_lines[0]; i++ )
        {
            //Draw upper row
            yarp::sig::Vector row_center(2), row_lower(2), row_upper(2);
            row_upper[0] = row_lower[0] = row_center[0] = center[0]+i*controlledJoints[0].delta;
            row_center[1] = center[1];
            row_lower[1] = controlledJoints[1].lower_limit;
            row_upper[1] = controlledJoints[1].upper_limit;
            listOfDesiredPositions.push_back(desiredPositions(row_center,static_pose_period));
            listOfDesiredPositions.push_back(desiredPositions(row_lower,static_pose_period));
            listOfDesiredPositions.push_back(desiredPositions(row_upper,static_pose_period));
            listOfDesiredPositions.push_back(desiredPositions(row_center,static_pose_period));
            if( mode == GRID_MAPPING_WITH_RETURN )
            {
                listOfDesiredPositions.push_back(desiredPositions(center,return_point_waiting_period,true));
            }

            //Draw lower row
            row_upper[0] = row_lower[0] = row_center[0] = center[0]-i*controlledJoints[0].delta;
            row_center[1] = center[1];
            row_lower[1] = controlledJoints[1].lower_limit;
            row_upper[1] = controlledJoints[1].upper_limit;
            listOfDesiredPositions.push_back(desiredPositions(row_center,static_pose_period));
            listOfDesiredPositions.push_back(desiredPositions(row_lower,static_pose_period));
            listOfDesiredPositions.push_back(desiredPositions(row_upper,static_pose_period));
            listOfDesiredPositions.push_back(desiredPositions(row_center,static_pose_period));
            if( mode == GRID_MAPPING_WITH_RETURN )
            {
                listOfDesiredPositions.push_back(desiredPositions(center,return_point_waiting_period));
            }
        }
        for(int j=0; j < (int)semi_nr_of_lines[1]; j++ )
        {
            //Draw upper row
            yarp::sig::Vector row_center(2), row_lower(2), row_upper(2);
            row_upper[1] = row_lower[1] = row_center[1] = center[1]+j*controlledJoints[1].delta;
            row_center[0] = center[0];
            row_lower[0] = controlledJoints[0].lower_limit;
            row_upper[0] = controlledJoints[0].upper_limit;
            listOfDesiredPositions.push_back(desiredPositions(row_center,static_pose_period));
            listOfDesiredPositions.push_back(desiredPositions(row_lower,static_pose_period));
            listOfDesiredPositions.push_back(desiredPositions(row_upper,static_pose_period));
            listOfDesiredPositions.push_back(desiredPositions(row_center,static_pose_period));
            if( mode == GRID_MAPPING_WITH_RETURN )
            {
                listOfDesiredPositions.push_back(desiredPositions(center,return_point_waiting_period,true));
            }

            //Draw lower row
            row_upper[1] = row_lower[1] = row_center[1] = center[1]-j*controlledJoints[1].delta;
            row_center[0] = center[0];
            row_lower[0] = controlledJoints[0].lower_limit;
            row_upper[0] = controlledJoints[0].upper_limit;
            listOfDesiredPositions.push_back(desiredPositions(row_center,static_pose_period));
            listOfDesiredPositions.push_back(desiredPositions(row_lower,static_pose_period));
            listOfDesiredPositions.push_back(desiredPositions(row_upper,static_pose_period));
            listOfDesiredPositions.push_back(desiredPositions(row_center,static_pose_period));
            if( mode == GRID_MAPPING_WITH_RETURN )
            {
                listOfDesiredPositions.push_back(desiredPositions(center,return_point_waiting_period,true));
            }
        }
    }

    ///////////////////////////////////////////////
    //// RPC PORT
    ///////////////////////////////////////////////
    rpc_handler.yarp().attachAsServer(rpcPort);
    std::string rpcPortName= "/";
    rpcPortName+= getName();
    rpcPortName += "/rpc:i";
    if (!rpcPort.open(rpcPortName.c_str())) {
        std::cerr << getName() << ": Unable to open port " << rpcPortName << std::endl;
        return false;
    }


    isTheRobotInReturnPoint.open("/"+moduleName+"/isTheRobotInReturnPoint:o");

    return true;
}


bool insituFTSensorCalibrationModule::interruptModule()
{
    return true;
}

bool insituFTSensorCalibrationModule::close()
{
    close_drivers();
    isTheRobotInReturnPoint.close();
    return true;
}

/**
 *
 */
bool insituFTSensorCalibrationModule::getNewDesiredPosition(yarp::sig::Vector & desired_pos,
                                                            double & desired_parked_time,
                                                            bool & is_return_point)
{
    switch(mode)
    {
        case GRID_VISIT:
        case GRID_MAPPING_WITH_RETURN:
            if( next_desired_position >= 0 && next_desired_position < listOfDesiredPositions.size() )
            {
                desired_pos = listOfDesiredPositions[next_desired_position].pos;
                desired_parked_time = listOfDesiredPositions[next_desired_position].waiting_time;
                is_return_point = listOfDesiredPositions[next_desired_position].is_return_point;
                next_desired_position++;
                return true;
            }
            else
            {
                return false;
            }
        break;
        default:
            std::cerr << "[ERR] insituFTSensorCalibrationModule: unknown mode " << mode << ", exiting" << std::endl;
            return false;
        break;
    }

}

bool insituFTSensorCalibrationModule::updateModule()
{
    yarp::os::LockGuard guard(moduleMutex);

    bool dataset_finish = false;

    if( status == COLLECTING_DATASET )
    {
        //Check that all desired position have been reached
        bool dones=true;
        for(int jnt=0; jnt < controlledJoints.size(); jnt++ )
        {
            bool done=true;
            std::string part = controlledJoints[jnt].part_name;
            int axis = controlledJoints[jnt].axis_number;
            pos[part]->checkMotionDone(axis,&done);
            dones = dones && done;
        }

        if(dones)
        {
            //std::cout << "elapsed_time: " << elapsed_time << std::endl;
            elapsed_time += getPeriod();
            //std::cout << "elapsed_time: " << elapsed_time << std::endl;
            if(mode == GRID_MAPPING_WITH_RETURN)
            {
                isTheRobotInReturnPoint.prepare().clear();
                if( is_desired_point_return_point )
                {
                    isTheRobotInReturnPoint.prepare().addInt(1);
                }
                else
                {
                    isTheRobotInReturnPoint.prepare().addInt(0);
                }
                isTheRobotInReturnPoint.write();
            }
        }

        if( elapsed_time > desired_waiting_time )
        {
            elapsed_time = 0.0;
            //set a new position for the controlled joints
            bool new_position_available = getNewDesiredPosition(commandedPositions,desired_waiting_time,is_desired_point_return_point);
            if( !new_position_available )
            {
                //no new position available, exiting
                dataset_finish = true;
            }

        }

        if( dataset_finish )
        {
            status = WAITING_NEW_DATASET_START;
        }
    }

    return true;
}

double insituFTSensorCalibrationModule::getPeriod()
{
    return period;
}

bool insituFTSensorCalibrationModule::startNewDatasetAcquisition()
{
    yarp::os::LockGuard guard(moduleMutex);
    if( status == WAITING_NEW_DATASET_START )
    {
        status = COLLECTING_DATASET;
    }
    return true;
}

