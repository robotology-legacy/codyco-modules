/*
 * Copyright (C) 2013 RobotCub Consortium
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 *
 */

#ifndef __YARP_KDL_H__
#define __YARP_KDL_H__


/**
 * Convert a 3x3 yarp::sig::Matrix representing the Inertia matrix of the iDynLink object, to a KDL::RotationalInertia object
 * @param yarpInertia Inertia of the iDynLink object input
 * @param kdlRotationalInertia KDL::RigidBodyInertia object output
 * @return true if conversion was successful, false otherwise
 */
bool YarptoKDL(const yarp::sig::Matrix & yarpInertia,KDL::RotationalInertia & kdlRotationalInertia);

/**
 * Convert a 4x4 yarp::sig::Matrix rapresenting a rototranslation matrix (element of SE(3)), to a KDL::Frame object
 * @param yarpMatrix yarp::sig::Matrix input
 * @param kdlFrame KDL::Frame output
 * @return true if conversion was successful, false otherwise
 */
bool YarptoKDL(const yarp::sig::Matrix & yarpMatrix, KDL::Frame & kdlFrame);

/**
 * Convert a 3x3 yarp::sig::Matrix rapresenting a rotation matrix (element of SO(3)), to a KDL::Rotation object
 * @param yarpMatrix yarp::sig::Matrix input
 * @param kdlRotation KDL::Rotation output
 * @return true if conversion was successful, false otherwise
 */
bool YarptoKDL(const yarp::sig::Matrix & yarpMatrix, KDL::Rotation & kdlRotation);


/**
 * Convert a yarp::sig::Vector to a KDL::Vector
 * @param yarpVector yarp::sig::Vector input
 * @param kdlVector KDL::Vector output
 * @return true if conversion was successful, false otherwise
 */
bool YarptoKDL(const yarp::sig::Vector & yarpVector, KDL::Vector & kdlVector);



/**
 * Convert a yarp::sig::Vector to a KDL::JntArray
 * @return true if conversion was successful, false otherwise
 */
bool YarptoKDL(const yarp::sig::Vector & yarpVector, KDL::JntArray & kdlJntArray);


/**
 * Convert a KDL::Vector to a yarp::sig::Vector 
 * @param kdlVector KDL::Vector input
 * @param idynVector yarp::sig::Vector output
 * @return true if conversion was successful, false otherwise
 */
bool KDLtoYarp(const KDL::Vector & kdlVector,yarp::sig::Vector & yarpVector);

/**
 * Convert a KDL::JntArray to a yarp::sig::Vector
 * @return true if conversion was successful, false otherwise
 */
bool KDLtoYarp(const KDL::JntArray & kdlJntArray,yarp::sig::Vector & yarpVector);


#endif
