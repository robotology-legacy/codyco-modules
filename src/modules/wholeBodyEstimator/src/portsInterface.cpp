#include "portsInterface.h"


using namespace yarp::math;

// ############## PUBLISHER_PORT CLASS ##################################################################################

publisherPort::publisherPort()
{}

publisherPort::~publisherPort()
{
    yInfo("[publisherPort] Closing port");
    this->outputPort->close();
    delete this->outputPort;
    this->outputPort = NULL;
}

bool publisherPort::configurePort(std::string className, std::string pName)
{
    this->estimatorName = className;
    this->portName = pName;
    this->outputPort = new yarp::os::BufferedPort<yarp::sig::Vector>;
    std::string fullPortName = std::string("/" + estimatorName + "/" + portName + ":o").c_str();
    if ( !this->outputPort->open( fullPortName ) )
    {
        yError ("Could not open output port %s ", fullPortName.c_str());
        return false;
    }
    return true;
}

void publisherPort::publishEstimateToPort(yarp::sig::Vector& data)
{
    yarp::sig::Vector& tmp = this->outputPort->prepare();
    tmp = data;
    this->outputPort->write();
}

bool publisherPort::closePort()
{
    yInfo("[publisherPort] Closing port");
    this->outputPort->close();
    delete this->outputPort;
    this->outputPort = NULL;
    
    return true;
}
// ############## READER_PORT CLASS #####################################################################################

readerPort::readerPort()
{}

readerPort::~readerPort()
{}

bool readerPort::configurePort(std::string className, std::string pName, std::string srcPort, yarp::os::Port * inputPort)
{
    inPort = inputPort;
    this->estimatorName = className;
    this->portName = pName;
    this->fullPortName = std::string("/" + estimatorName + "/" + portName + ":i").c_str();
    if ( !inputPort->open(this->fullPortName) ) {
        yError ("Could not open input port %s ", fullPortName.c_str());
        return false;
    } else {
        if ( !yarp::os::Network::connect(srcPort, fullPortName) )
        {
            yError ("Could not connect to port %s", srcPort.c_str() );
            return false;
        }
    }
    return true;
}

bool readerPort::extractMTBDatafromPort(int boardNum, yarp::os::Port * sensorMeasPort, measurementsStruct &measurements)
{
    yarp::sig::Vector fullMeasurement;
    yarp::os::Bottle measurementBottle;
    int indexSubVector = 0;
    if ( !sensorMeasPort->read(fullMeasurement) ) {
        yError("[extractMTBDatafromPort] There was an error trying to read from the MTB port");
        return false;
    } else {
            //yInfo("[QuaternionEKF::extractMTBDatafromPort] Raw meas: %s", fullMeasurement.toString().c_str());
        /******************* searching for multiple instances of the sensor  **************************/
        double* tmp;
        tmp = fullMeasurement.data();
        double *it = tmp + 2; // First two elements of the vector can be skipped
        while (it < tmp + fullMeasurement.size()) {
            it = std::find(it, it + (fullMeasurement.size() - indexSubVector), boardNum);
            if (it < tmp + fullMeasurement.size()) {
                indexSubVector = static_cast<int>(it - tmp) + 1;

                // Parse sensor data
                int trueindexSubVector = indexSubVector - 1;
                if ( static_cast<int>(tmp[trueindexSubVector])  == boardNum ) {
                    //  If sensor from board "boardNum" is an accelerometer
                    if ( tmp[trueindexSubVector + 1] == 1.0) {
                        measurements.linAcc(0) = tmp[trueindexSubVector + 3];
                        measurements.linAcc(1) = tmp[trueindexSubVector + 4];
                        measurements.linAcc(2) = tmp[trueindexSubVector + 5];
                    } else {
                        // If sensor from board "boardNum" is a gyroscope
                        if ( tmp[trueindexSubVector + 1] == 2.0 ) {
                            measurements.angVel(0) = tmp[trueindexSubVector + 3];
                            measurements.angVel(1) = tmp[trueindexSubVector + 4];
                            measurements.angVel(2) = tmp[trueindexSubVector + 5];
                        }
                    }
                }
                it = it + MTB_PORT_DATA_PACKAGE_OFFSET; //Move to the next position in the vector where a package data is expected
            }
        }
        /**********************************************************************************/
        measurements.linAcc = measurements.linAcc*CONVERSION_FACTOR_ACC;
        if (yarp::math::norm(measurements.linAcc) > 11.0) {
            yWarning("[QuaternionEKF::extractMTBDatafromPort]  WARNING!!! Gravity's norm is too big!");
        }
        measurements.angVel = PI/180*CONVERSION_FACTOR_GYRO*measurements.angVel;
        if (yarp::math::norm(measurements.angVel) > 100.0) {
            yWarning("[QuaternionEKF::extractMTBDatafromPort]  WARNING!!! [QuaternionEKF::extractMTBDatafromPort] Ang vel's norm is too big!");
        }
    }
    return true;
}

bool readerPort::closePort()
{
        yInfo("[QuaternionEKF::readerPort] Closing port");
        inPort->close();
        delete inPort;
        inPort = NULL;
        
        return true;
}
