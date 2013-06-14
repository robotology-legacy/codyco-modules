/*
 * Copyright (C) 2013 IIT - Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 *
 */
 
#include "iCub/iDynTree/yarp_kdl.h"
#include <cstring>

bool YarptoKDL(const yarp::sig::Matrix & idynInertia,KDL::RotationalInertia & kdlRotationalInertia)
{
     if(idynInertia.cols() != 3 || idynInertia.rows() != 3 ) return false;
     if(idynInertia(0,1) != idynInertia(1,0) || idynInertia(0,2) != idynInertia(2,0) || idynInertia(1,2) != idynInertia(2,1)) return false;
     kdlRotationalInertia = KDL::RotationalInertia(idynInertia(0,0),idynInertia(1,1),idynInertia(2,2),idynInertia(0,1),idynInertia(0,2),idynInertia(1,2));
     return true;
}

bool YarptoKDL(const yarp::sig::Matrix & idynMatrix, KDL::Frame & kdlFrame)
{
    if( idynMatrix.cols() != 4 || idynMatrix.rows() != 4 ) return false;
    KDL::Rotation kdlRotation;
    KDL::Vector kdlVector;
    YarptoKDL(idynMatrix.submatrix(0,2,0,2),kdlRotation);
    YarptoKDL(idynMatrix.subcol(0,3,3),kdlVector);
    kdlFrame = KDL::Frame(kdlRotation,kdlVector);
    return true;
}

bool YarptoKDL(const yarp::sig::Matrix & idynMatrix, KDL::Rotation & kdlRotation)
{
    if(idynMatrix.cols() != 3 || idynMatrix.rows() != 3) return false;
    kdlRotation = KDL::Rotation(idynMatrix(0,0),idynMatrix(0,1),idynMatrix(0,2),
                                idynMatrix(1,0),idynMatrix(1,1),idynMatrix(1,2),
                                idynMatrix(2,0),idynMatrix(2,1),idynMatrix(2,2));
    return true;
}

bool YarptoKDL(const yarp::sig::Vector & yarpVector, KDL::Vector & kdlVector)
{
    if( yarpVector.size() != 3 ) return false;
    memcpy(kdlVector.data,yarpVector.data(),3*sizeof(double));
    return true;
}

bool YarptoKDL(const yarp::sig::Vector & yarpVector, KDL::JntArray & kdlJntArray)
{
    size_t nj = yarpVector.size();
    if( kdlJntArray.rows() != nj ) { kdlJntArray.resize(nj) }
    memcpy(kdlJntArray.data,yarpVector.data(),nj*sizeof(double));
    return true;
}


bool KDLtoYarp(const KDL::Vector & kdlVector,yarp::sig::Vector & yarpVector)
{
    if( yarpVector.size != 3 ) { yarpVector.resize(3); }
    memcpy(yarpVector.data(),kdlVector.data,3*sizeof(double));
    return true;
}

bool KDLtoYarp(const KDL::JntArray & kdlJntArray,yarp::sig::Vector & yarpVector)
{
    int nj = kdlJntArray.rows();
    if( yarpVector.size != nj ) { yarpVector.resize(nj); }
    memcpy(yarpVector.data(),kdlJntArray.data,nj*sizeof(double));
    return true;
}


