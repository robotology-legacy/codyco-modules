#ifndef PORTS_INTERFACE_H_
#define PORTS_INTERFACE_H_

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/os/Log.h>
#include <string>
#include <iostream>
#include <constants.h>

/**
 *  Structure holding IMU-like information, namely linear acceleration, angular velocity and real orientation when available in body frame.
 */
struct measurementsStruct
{
    yarp::sig::Vector linAcc;
    yarp::sig::Vector angVel;
    // Assuming real orientation is given in Eulers
    yarp::sig::Vector realOrientation;
};


class publisherPort
{
private:
    std::string                                 estimatorName;
    std::string                                 portName;
    yarp::sig::Vector                           outputData;
    yarp::os::BufferedPort<yarp::sig::Vector> * outputPort;
public:
    publisherPort();
    virtual ~publisherPort();
    /**
     *  Opens a publisher port and provides methods to configure and publish data.
     *
     *  @param className Current estimator class name.
     *  @param pName     Publisher port name. e.g. "rawAccelerometerData". Notice that no ":o" must be added at the end.
     *
     *  @return True if port was successfully opened, false otherwise.
     */
    bool configurePort(std::string className, std::string pName);

    /**
     *  Publishes the data passed to this method on the port opened by this object.
     *
     *  @param data Data vector to be published.
     */
    void publishEstimateToPort(yarp::sig::Vector& data);
    
    /**
     *  Closes the publisher ports opened by this object.
     *
     *  @return True if successful, false otherwise.
     */
    bool closePort();
};


class readerPort
{
private:
    std::string      estimatorName;
    std::string      portName;
    std::string      fullPortName;
    yarp::os::Port * inPort;
public:
    /**
     *  Constructor
     */
    readerPort();


    /**
     *  Destructor
     */
    virtual ~readerPort();


    /**
     *  Opens a reader port and connects it to its source.
     *
     *  @param className    Current estimator class name.
     *  @param pName        Reader port name.
     *  @param srcPort      Source port name.
     *  @param inputPort    Pointer to input port. This should be a private variable of the calling class.
     *
     *  @return true when ports creation and connections are successful.
     */
    bool configurePort(std::string className, std::string pName, std::string srcPort, yarp::os::Port * inputPort);


    /**
     *  This method is specific to an MTB board with gyroscope and accelerometer attached to an ethernet robot such as iCubGenova02. Provided that the following variables are somewhere defined: MTB_PORT_DATA_PACKAGE_OFFSET, CONVERSION_FACTOR_ACC, CONVERSION_FACTOR_GYRO. This method parses the masurements as streamed by the intertial unit and separates them into linear acceleration, angular velocity and orientation -if provided- as output.
     *
     *  @param boardNum       Currently specified in consntants.h
     *  @param sensorMeasPort Port from where measurements are read.
     *  @param measurements   (output) Separated measurements in a single object.
     *
     *  @return True when parsing is succesful, false otherwise.
     */
    bool extractMTBDatafromPort(int boardNum, yarp::os::Port * sensorMeasPort, measurementsStruct &measurements);
    
    
    /**
     *  Closes the reader port opened by this object.
     *
     *  @return True if successful, false otherwise.
     */
    bool closePort();

};


#endif
