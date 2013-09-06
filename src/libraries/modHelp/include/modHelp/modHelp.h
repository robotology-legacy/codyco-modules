/*
 * Copyright (C) 2012 MACSi - 2013 CoDyCo
 * Author: Serena Ivaldi
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

/**
 * \defgroup codyco_modHelp modHelp
 *
 * @ingroup codyco_libraries
 *
 * Classes/functions for helping writing modules.
 *
 * \section dep_sec Dependencies
 * Eigen
 *
 * \section intro_sec Description
 * This is a collection of functions to ease writing code for modules. There are functions to read 
 * variables from ini files; functions to create/open/close YARP ports; functions to ease the
 * conversion of numeric vectors from yarp to eigen and viceversa.
 *
 * \section tested_os_sec Tested OS
 *
 * Linux
 *
 *
 * \author Serena Ivaldi - serena.ivaldi@isir.upmc.fr
 *
 * Copyright (C) 2012-3 MACSI, 2013 CODYCO
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 *
 **/

#ifndef __MODHELP_H__
#define __MODHELP_H__

#include <yarp/os/all.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <deque>
#include <string>

#include <Eigen/Lgsm>
#include <Eigen/StdVector> //Mandatory for vector fixed-size Eigen element (like Twistd)
                           //otherwise it raises assertion of bad alignement

namespace modHelp
{
  
#define displayValue(V)         cout<<"  "<< #V <<" : "<<V<<endl;
#define displayNameValue(S,V)   cout<<"  "<< S <<" : "<<V<<endl;
#define displayVector(V)        cout<<"  "<< #V <<" : "<<V.toString()<<endl;
#define displayNameVector(S,V)  cout<<"  "<< S <<" : "<<V.toString()<<endl;
    
    /**
     * Find a parameter in a ResourceFinder and read it as a string.
     If the parameter is not found, the default value vdefault is set in the variable v.
     * @param rf a ResourceFinder
     * @param name the parameter, for example "robot", "part", ..
     * @param v the variable which will store the value of the parameter
     * @param vdefault the default value, for example "iCub", "right_arm", ..
     */
    void readString(yarp::os::ResourceFinder &rf, std::string name, std::string &v, std::string vdefault);
    
    /**
     * Find a parameter in a ResourceFinder and read it as a double.
     If the parameter is not found, the default value vdefault is set in the variable v.
     * @param rf a ResourceFinder
     * @param name the parameter, for example "tolerance", ..
     * @param v the variable which will store the value of the parameter
     * @param vdefault the default value, for example 0.01, ..
     */
    void readDouble(yarp::os::ResourceFinder &rf, std::string name, double &v, double vdefault);
    
    /**
     * Find a parameter in a ResourceFinder and read it as an integer.
     If the parameter is not found, the default value vdefault is set in the variable v.
     * @param rf a ResourceFinder
     * @param name the parameter, for example "trials", ..
     * @param v the variable which will store the value of the parameter
     * @param vdefault the default value, for example 20, ..
     */
    void readInt(yarp::os::ResourceFinder &rf, std::string name, int &v, int vdefault);
    
    /**
     * Find a parameter in a ResourceFinder and read it as a bool.
     The parameter is considered true if the string describing the value
     in the RF file is "true", "yes", "active" or "on"; any other string is interpreted as false.
     If the parameter is not found, the default value vdefault is set in the variable v.
     * @param rf a ResourceFinder
     * @param name the parameter, for example "enableTorso", ..
     * @param v the variable which will store the value of the parameter
     * @param vdefault the default value, true/false
     */
    void readBool(yarp::os::ResourceFinder &rf, std::string name, bool &v, bool vdefault);
    
    /**
     * Find a parameter in a ResourceFinder and read it as a vector of double.
     The length of the vector is also required.
     If the parameter is not found, by default the vector is set to zero.
     * @param rf a ResourceFinder
     * @param name the parameter, for example "njoints", ..
     * @param v the variable which will store the vector of values of the parameter
     * @param len the length of the vector, for example 5
     */
    void readVector(yarp::os::ResourceFinder &rf, std::string name, yarp::sig::Vector &v, int len);
    
    /**
     * Find a parameter in a Bottle and read it as a string.
     If the parameter is not found, the default value vdefault is set in the variable v.
     * @param rf a bottle
     * @param name the parameter, for example "robot", "part", ..
     * @param v the variable which will store the value of the parameter
     * @param vdefault the default value, for example "iCub", "right_arm", ..
     */
    void readString(yarp::os::Bottle &rf, std::string name, std::string &v, std::string vdefault);
    
    /**
     * Find a parameter in a Bottle and read it as a double.
     If the parameter is not found, the default value vdefault is set in the variable v.
     * @param rf a bottle
     * @param name the parameter, for example "tolerance", ..
     * @param v the variable which will store the value of the parameter
     * @param vdefault the default value, for example 0.01, ..
     */
    void readDouble(yarp::os::Bottle &rf, std::string name, double &v, double vdefault);
    
    
    /**
     * Find a parameter in a Bottle and read it as an integer.
     If the parameter is not found, the default value vdefault is set in the variable v.
     * @param rf a bottle
     * @param name the parameter, for example "trials", ..
     * @param v the variable which will store the value of the parameter
     * @param vdefault the default value, for example 20, ..
     */
    void readInt(yarp::os::Bottle &rf, std::string name, int &v, int vdefault);
    
    /**
     * Find a parameter in a Bottle and read it as a bool.
     The parameter is considered true if the string describing the value
     in the RF file is "true", "yes" or "on".
     If the parameter is not found, the default value vdefault is set in the variable v.
     * @param rf a bottle
     * @param name the parameter, for example "enableTorso", ..
     * @param v the variable which will store the value of the parameter
     * @param vdefault the default value, true/false
     */
    void readBool(yarp::os::Bottle &rf, std::string name, bool &v, bool vdefault);
    
    /**
     * Find a parameter in a Bottle and read it as a vector of double.
     The length of the vector is also required.
     If the parameter is not found, by default the vector is set to zero.
     * @param rf a bottle
     * @param name the parameter, for example "njoints", ..
     * @param v the variable which will store the vector of values of the parameter
     * @param len the length of the vector, for example 5
     */
    void readVector(yarp::os::Bottle &rf, std::string name, yarp::sig::Vector &v, int len);
    
    
    /**
     * Creates the polydriver through the pointer, using the specified options.
     * @param p the pointer to the polydriver
     * @param options the pointer to the polydriver
     * @return true if driver was created correctly, false if there were errors
     */
    bool createDriver(yarp::dev::PolyDriver *&_dd, yarp::os::Property &options);
        
    /**
     * Delete the polydriver through the pointer.
     * @param p the pointer to the polydriver
     */
    void deleteDriver(yarp::dev::PolyDriver *p);
    
    /**
     * Delete the encoder interface through the pointer.
     * @param ie the interface
     */
    void deleteInterfaceEncoders(yarp::dev::IEncoders *ie);
    
    /**
     * Delete the position interface through the pointer.
     * @param ie the interface
     */
    void deleteInterfacePosition(yarp::dev::IPositionControl * ip);
    
    /**
     * Delete the impedance interface through the pointer.
     * @param ie the interface
     */
    void deleteInterfaceImpedance(yarp::dev::IImpedanceControl *ii);
    
    /**
     * Delete the control interface through the pointer.
     * @param ie the interface
     */
    void deleteInterfaceControl(yarp::dev::IControlMode *ic);
    
    //template <typename T>
    //        void deleteInterface(T* i){ delete i;}
    
    /**
     * A simple utility function to flush cin - because fflush(stdin) works only in MS-C++.
     */
    void fflushCin();
    
    /**
     * Connects two ports.
     * @param portFrom the port streaming out
     * @param portTo the port receiving in the data 
     * @return true if ports were connected, false otherwise
     */
    bool connectPorts(const std::string &portFrom, const std::string &portTo, int retry=1);
    
    /**
     * Close a port.
     * @param port the port to be closed
     */
    void closePort(yarp::os::Contactable *port);
    
    /**
     * Turns a boolean flag into a string true/false (type=1), on/off (type=2), yes/no (type=3), enabled/disabled (type=4)
     * @param val the boolean value
     * @param type the format of the boolean flag
     * @return the string representing the value
     */
     std::string toString(bool val, int type=2);


    bool yarpToEigenVector(const yarp::sig::Vector &yarpVector, Eigen::VectorXd &eigenVector);

    bool eigenToYarpVector(const Eigen::VectorXd &eigenVector, yarp::sig::Vector &yarpVector);

    bool yarpToEigenMatrix(const yarp::sig::Matrix& yarpMatrix, Eigen::MatrixXd& eigenMatrix);

    bool eigenToYarpMatrix(const Eigen::MatrixXd& eigenMatrix, yarp::sig::Matrix& yarpMatrix);

    
}//end namespace modHelp


#endif



