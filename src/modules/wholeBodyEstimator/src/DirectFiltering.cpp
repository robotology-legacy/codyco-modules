#include "DirectFiltering.h"

REGISTERIMPL(DirectFiltering);

using namespace yarp::math;

DirectFiltering::DirectFiltering ( ) : m_className("DirectFiltering"), m_lsole_R_acclsensor(3,3)
{
    // Store rotation from sensor to left sole frame
    MatrixWrapper::Quaternion lsole_Rq_acclsensor(1.0,0.0,0.0,0.0);
    lsole_Rq_acclsensor.getRotation(m_lsole_R_acclsensor);
}

DirectFiltering::~DirectFiltering()
{}

bool DirectFiltering::init ( yarp::os::ResourceFinder &rf, wbi::iWholeBodySensors *wbs )
{
    // Read directFiltering params
    DirectFiltering::readEstimatorParams(rf, m_params);
    
    // Open and configure port for reading measurements
    sensorMeasPort = new yarp::os::Port;
    std::string srcPort = std::string("/" + this->m_params.robotPrefix + "/right_leg/inertialMTB");
    if ( !sensorDataPort.configurePort(this->m_className, std::string("rightFootMTBreader"), srcPort, sensorMeasPort) )
    {
        yError( "[DirectFiltering::init] Could not connect to %s", srcPort.c_str() );
        return false;
    }
    
    // Open and configure port for direct orientation estimate
    if ( !m_estimatePort.configurePort(this->m_className, std::string("orientationEuler")) )
    {
        yError("[DirectFiltering::init] Estimate output port could not be configured");
        return false;
    } else {
        yInfo("[DirectFiltering::init] Estimate output port configured correctly");
    }
    
    // Open and configure port for tilt estimate
    if ( !m_tiltPort.configurePort(this->m_className, std::string("tiltEuler")) )
    {
        yError("[DirectFiltering::init] Tilt output port could not be configured");
        return false;
    } else{
        yInfo("[DirectFiltering::init] Tilt output port configured correctly");
    }
    
    m_meas.linAcc.resize(3,0.0);
    m_meas.angVel.resize(3,0.0);
    std::cout << "Running DirectFiltering... \n" << std::endl;
    return true;
}

void DirectFiltering::run ( )
{
    // Read sensor measurements
    if ( !sensorDataPort.extractMTBDatafromPort(MTB_RIGHT_FOOT_ACC_PLUS_GYRO_2_ID, sensorMeasPort, m_meas) )
    {
        yError( "[DirectFiltering::run] Could not read measurement" );
    }
    
    yarp::sig::Vector orientation;
    yarp::sig::Vector tilt;
    orientation.resize(3,0.0);
    tilt.resize(1,0.0);
    computeOrientation(&m_meas.linAcc, orientation);
    //computeTilt(&m_meas.linAcc, tilt);
    
    // Stream estimate
    this->m_estimatePort.publishEstimateToPort(orientation);
    
    // Stream tilt
    //this->m_tiltPort.publishEstimateToPort(tilt);
    
}

bool DirectFiltering::readEstimatorParams ( yarp::os::ResourceFinder &rf, directFilteringParams &params )
{
    yarp::os::Bottle botParams;
    botParams = rf.findGroup("DirectFiltering");
    yInfo("DirectFiltering Params are: %s ", botParams.toString().c_str());
    if ( botParams.isNull() )
    {
        yError("[DirectFiltering::readEstimatorParams] No parameters were read from DirectFiltering group");
        return false;
    } else {
        yarp::os::Bottle moduleParamsGroup = rf.findGroup("module_parameters");
        if ( moduleParamsGroup.isNull() )
        {
            yError("[DirectFiltering::readEstimatorParams] No module_params group was found.");
            return false;
        }
        std::string robotName = moduleParamsGroup.find("robot").asString();
        params.robotPrefix = rf.findGroup("module_parameters").find("robot").asString();
        params.streamMeasurements = botParams.find("stream_measurements").asBool();
    }
    
}

void DirectFiltering::release ( )
{
    
}

void DirectFiltering::computeOrientation ( yarp::sig::Vector* sensorReading, yarp::sig::Vector& output )
{
    //REF: https://www.nxp.com/files/sensors/doc/app_note/AN3461.pdf
    MatrixWrapper::ColumnVector input(sensorReading->data(), sensorReading->length());
    // Rotate sensor reading (expressed in the sensor frame) to the foot reference frame
    input = m_lsole_R_acclsensor*input;
    // Roll assuming xyz rotation
    double phi_xyz = atan2( input(2),input(3) );
    // Pitch assuming xyz rotation
    double theta_xyz = (-input(1))/(sqrt(pow(input(2),2) + pow(input(3),2)));
    output(0) = (180.0/(double)PI)*phi_xyz;
    output(1) = (180.0/(double)PI)*theta_xyz;
    output(2) = 0.0;
}

void DirectFiltering::computeTilt(yarp::sig::Vector *sensorReading, yarp::sig::Vector &output)
{
    assert(output.size() == 1);
    //REF: https://www.nxp.com/files/sensors/doc/app_note/AN3461.pdf
    MatrixWrapper::ColumnVector input(sensorReading->data(), sensorReading->length());
    // Rotate sensor reading (expressed in the sensor frame) to the foot reference frame
    input = m_lsole_R_acclsensor*input;
    // Roll assuming xyz rotation
    double tilt = acos(input(3)/input.norm());
    output(0) = (180.0/(double)PI)*tilt;
}

void DirectFiltering::setWorldOrientation ( MatrixWrapper::Quaternion& worldOrientation )
{

}
