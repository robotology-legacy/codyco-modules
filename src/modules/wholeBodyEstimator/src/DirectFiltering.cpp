#include "DirectFiltering.h"

REGISTERIMPL(filter::DirectFiltering);

using namespace filter;
using namespace yarp::math;

DirectFiltering::DirectFiltering(): m_className("DirectFiltering")
{}

DirectFiltering::DirectFiltering ( MatrixWrapper::Quaternion lsole_Rq_acclsensor ) : m_lsole_R_acclsensor(3,3)
{
    // Store rotation from sensor to left sole frame
    lsole_Rq_acclsensor.getRotation(m_lsole_R_acclsensor);

}

DirectFiltering::~DirectFiltering()
{}

bool DirectFiltering::init ( yarp::os::ResourceFinder &rf, wbi::iWholeBodySensors *wbs )
{
    // Read directFiltering params
    filter::DirectFiltering::readEstimatorParams(rf, m_params);
    
    // Open and configure port for reading measurements
    sensorMeasPort = new yarp::os::Port;
    std::string srcPort = std::string("/" + this->m_params.robotPrefix + "/right_leg/inertialMTB");
    if ( !sensorDataPort.configurePort(this->m_className, std::string("rightFootMTBreader"), srcPort, sensorMeasPort) )
    {
        yError( "[DirectFiltering::init] Could not connect to %s", srcPort.c_str() );
        return false;
    }
}

void DirectFiltering::run ( )
{
    std::cout << "Running DirectFiltering" << std::endl;
    measurementsStruct meas;
    // Read sensor measurements
    if ( !sensorDataPort.extractMTBDatafromPort(MTB_RIGHT_FOOT_ACC_PLUS_GYRO_2_ID, sensorMeasPort, meas) )
    {
        yError( "[DirectFiltering::run] Could not read measurement" );
    }
    
    yarp::sig::Vector orientation;
    computeOrientation(&meas.linAcc, orientation);
    
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
        params.robotPrefix = rf.findGroup("module_params").find("robot").asString();
        params.streamMeasurements = botParams.find("stream_measurements").asBool();
    }
    
}

void DirectFiltering::release ( )
{
    
}

void DirectFiltering::computeOrientation ( yarp::sig::Vector* sensorReading, yarp::sig::Vector& output )
{
    // Applying scaling factor
    (*sensorReading) = CONVERSION_FACTOR_ACC*(*sensorReading);
    MatrixWrapper::ColumnVector input(sensorReading->data(), sensorReading->length());
    // Rotate sensor reading (expressed in the sensor frame) to the foot reference frame
    input = m_lsole_R_acclsensor*input;
    // Roll assuming xyz rotation
    double phi_xyz = atan2( input(2),input(3) );
    // Pitch assuming xyz rotation
    double theta_xyz = (input(1))/(sqrt(pow(input(2),2) + pow(input(3),2)));
    output(0) = (180.0/(double)PI)*phi_xyz;
    output(1) = (180.0/(double)PI)*theta_xyz;
    output(2) = 0.0;
}

void DirectFiltering::setWorldOrientation ( MatrixWrapper::Quaternion& worldOrientation )
{

}
