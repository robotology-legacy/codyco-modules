/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
#include "iDyn_KDL_emulation.h" 
#include <vector>
#include <iCub/iDyn/iDyn.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/api.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>
#include <iCub/skinDynLib/skinContactList.h>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>

#include <cassert>

yarp::sig::Matrix idynChainGetForces_usingKDL(iCub::iDyn::iDynChain & idynChain,yarp::sig::Vector & ddp0)
{
    KDL::Wrenches f_i;

    int nj = idynChain.getN();

    yarp::sig::Vector f(3);
    yarp::sig::Matrix F(3,nj);
    
    F.zero();
    
    f_i =  idynChainGet_usingKDL_aux(idynChain,ddp0);
    for(int i=0; i < nj; i++ ) {
        //iCub::iKin::iKinLink = armTorsoDyn.asChain()
        if( i != nj -1 ) {
            to_iDyn(f_i[i+1].force,f);
            f = ((idynChain)[i+1]).getR()*f;
        } else {
            f.zero();
        }
        F.setCol(i,f);
    }
    return F;
}

yarp::sig::Matrix idynChainGetMoments_usingKDL(iCub::iDyn::iDynChain & idynChain,yarp::sig::Vector & ddp0)
{
    KDL::Wrenches f_i;
    int nj = idynChain.getN();

    yarp::sig::Vector mu(3),f(3);
    yarp::sig::Matrix Mu(3,nj);
    
    Mu.zero();
    
    f_i =  idynChainGet_usingKDL_aux(idynChain,ddp0);
    //Debug
    /*
    std::cout << "f_i without sensor " << std::endl;
    for(int p=0;p<f_i.size();p++)
    {
        std::cout << f_i[p] << std::endl;
    }
    */
    
    for(int i=0; i < nj; i++ ) {
      if( i != nj -1 ) {  
        to_iDyn(f_i[i+1].force,f);
        f = (idynChain[i+1]).getR()*f;
        to_iDyn(f_i[i+1].torque,mu);
        yarp::sig::Vector r_n = ((idynChain)[i+1]).getH().subcol(0,3,3);
        mu = ((idynChain)[i+1]).getR()*mu+cross(r_n,f);
      } else {
        mu.zero();
      }
      Mu.setCol(i,mu);
    }
    return Mu;
}

yarp::sig::Vector idynChainGetTorques_usingKDL(iCub::iDyn::iDynChain & idynChain,yarp::sig::Vector & ddp0)
{
    yarp::sig::Vector q,dq,ddq;
    q = idynChain.getAng();
    dq = idynChain.getDAng();
    ddq = idynChain.getD2Ang();
    
    KDL::Chain kdlChain;
    
    idynChain2kdlChain(idynChain,kdlChain);
    
    int nj = idynChain.getN();
    
    KDL::JntArray jointpositions = KDL::JntArray(nj);
    KDL::JntArray jointvel = KDL::JntArray(nj);
    KDL::JntArray jointacc = KDL::JntArray(nj);
    KDL::JntArray torques = KDL::JntArray(nj);

    for(unsigned int i=0;i<nj;i++){
        jointpositions(i)=q[i];
        jointvel(i) = dq[i];
        jointacc(i) = ddq[i];
    }
    
    KDL::Wrenches f_ext(nj);
    KDL::Wrenches f_int(nj);
    
    KDL::Vector grav_kdl;
    idynVector2kdlVector(idynChain.getH0().submatrix(0,2,0,2).transposed()*ddp0,grav_kdl);
    
    KDL::ChainIdSolver_RNE_IW neSolver = KDL::ChainIdSolver_RNE_IW(kdlChain,-grav_kdl);
    
    assert(neSolver.CartToJnt_and_internal_wrenches(jointpositions,jointvel,jointacc,f_ext,torques,f_int) >= 0);
    
    yarp::sig::Vector ret_tau;
    kdlJntArray2idynVector(torques,ret_tau);
    return ret_tau;
}

KDL::Wrenches idynChainGet_usingKDL_aux(iCub::iDyn::iDynChain & idynChain,yarp::sig::Vector & ddp0)
{
    yarp::sig::Vector q,dq,ddq;
    q = idynChain.getAng();
    dq = idynChain.getDAng();
    ddq = idynChain.getD2Ang();
    
    KDL::Chain kdlChain;
    
    idynChain2kdlChain(idynChain,kdlChain);
    
    int nj = idynChain.getN();
    
    KDL::JntArray jointpositions = KDL::JntArray(nj);
    KDL::JntArray jointvel = KDL::JntArray(nj);
    KDL::JntArray jointacc = KDL::JntArray(nj);
    KDL::JntArray torques = KDL::JntArray(nj);

    for(unsigned int i=0;i<nj;i++){
        jointpositions(i)=q[i];
        jointvel(i) = dq[i];
        jointacc(i) = ddq[i];
    }
    
    KDL::Wrenches f_ext(nj);
    KDL::Wrenches f_int(nj);
    
    KDL::Vector grav_kdl;
    idynVector2kdlVector(idynChain.getH0().submatrix(0,2,0,2).transposed()*ddp0,grav_kdl);
    
    KDL::ChainIdSolver_RNE_IW neSolver = KDL::ChainIdSolver_RNE_IW(kdlChain,-grav_kdl);
    
    int ret = neSolver.CartToJnt_and_internal_wrenches(jointpositions,jointvel,jointacc,f_ext,torques,f_int);
    assert( ret >= 0);
    
    return f_int;
}


yarp::sig::Matrix idynChainGetForces_usingKDL(iCub::iDyn::iDynChain & idynChain,iCub::iDyn::iDynInvSensor & idynSensor,yarp::sig::Vector & ddp0)
{
    KDL::Wrenches f_i;

    int nj = idynChain.getN();
    int ns = nj+1;
    int sensor_link = idynSensor.getSensorLink();


    yarp::sig::Vector f(3);
    yarp::sig::Matrix F(3,nj);
    
    F.zero();
    
    f_i =  idynChainGet_usingKDL_aux(idynChain,idynSensor,ddp0);

    assert(f_i.size() == ns);
    
    int i,j;
    for(i=0,j=0; i < ns; i++) {
        if( i < sensor_link ) {
            assert(i != ns -1);
            //DEBUG
            //cout << "converting old, f_i " << i+1 << endl;
            to_iDyn(f_i[i+1].force,f);
            f = ((idynChain)[i+1]).getR()*f;
        }
    
        if( i == sensor_link-1 )
        {
            assert(i != ns -1);
            //pay attention to this, for KDL the wrench is referred to the sensor reference frame
            iCub::iKin::iKinLink link_sensor = idynChain[sensor_link];
            /*
            double angSensorLink = link_sensor.getAng();
            yarp::sig::Matrix  H_sensor_link = (link_sensor.getH(0.0));
            link_sensor.setAng(angSensorLink);
            */
             yarp::sig::Matrix  H_sensor_link = (link_sensor.getH());
            //H_0 = H_{i}^{i-1}*H_{s}^{i}
            yarp::sig::Matrix H_0 = H_sensor_link  * (idynSensor.getH());
            
            to_iDyn(f_i[i+1].force,f);
            f = H_0.submatrix(0,2,0,2)*f;
        }
        if( i > sensor_link ) 
        {
            if( i != ns-1 ) {
                //DEBUG
                //cout << "converting new, f_i " << i+1 << endl;
                to_iDyn(f_i[i+1].force,f);
                //printVector("f",f);
                f = ((idynChain)[i]).getR()*f;
                //printVector("f",f);

            } else {
                f.zero();
            }
            
        }
        if(i!=sensor_link) {
            //cout << "j" << j << endl;
            F.setCol(j,f);
            j++;
        }
    }
    return F;
}
yarp::sig::Matrix idynChainGetMoments_usingKDL(iCub::iDyn::iDynChain & idynChain,iCub::iDyn::iDynInvSensor & idynSensor,yarp::sig::Vector & ddp0)
{
    KDL::Wrenches f_i;
    int nj = idynChain.getN();
    
    int ns = nj+1;
    int sensor_link = idynSensor.getSensorLink();

    yarp::sig::Vector mu(3),f(3);
    yarp::sig::Matrix Mu(3,nj);
    
    Mu.zero();

    f_i =  idynChainGet_usingKDL_aux(idynChain,idynSensor,ddp0);
    
    //Debug
    /*
    std::cout << "f_i with sensor " << std::endl;
    for(int p=0;p<f_i.size();p++)
    {
        std::cout << f_i[p] << std::endl;
    }
    */
    
    int i,j;
    for(i=0,j=0; i < ns; i++) {
        mu.zero();
        if( i < sensor_link-1 ) {
            assert(i != ns -1);
            to_iDyn(f_i[i+1].force,f);
            f = ((idynChain)[i+1]).getR()*f;
            to_iDyn(f_i[i+1].torque,mu);
            yarp::sig::Vector r_n = ((idynChain)[i+1]).getH().subcol(0,3,3);
            mu = ((idynChain)[i+1]).getR()*mu+cross(r_n,f);
        }
        if( i == sensor_link-1 )
        {
            assert(i != ns -1);
            //pay attention to this, for KDL the wrench is referred to the sensor reference frame
            iCub::iKin::iKinLink link_sensor = idynChain[sensor_link];
            //double angSensorLink = link_sensor.getAng();
            yarp::sig::Matrix  H_sensor_link = (link_sensor.getH());
            //link_sensor.setAng(angSensorLink);
            yarp::sig::Matrix H_0 = H_sensor_link  * (idynSensor.getH());
            
            to_iDyn(f_i[i+1].force,f);
            f = H_0.submatrix(0,2,0,2)*f;
            to_iDyn(f_i[i+1].torque,mu);
            yarp::sig::Vector r_n = H_0.subcol(0,3,3);
            mu =  H_0.submatrix(0,2,0,2)*mu+cross(r_n,f);
        }
        if( i > sensor_link-1 ) 
        {
            if( i != ns-1 ) {
                to_iDyn(f_i[i+1].force,f);
                f = ((idynChain)[i]).getR()*f;
                to_iDyn(f_i[i+1].torque,mu);
                yarp::sig::Vector r_n = ((idynChain)[i]).getH().subcol(0,3,3);
                mu = ((idynChain)[i]).getR()*mu+yarp::math::cross(r_n,f);
            } else {
                mu.zero();
            }
            
        }
        if(i!=sensor_link) {
            Mu.setCol(j,mu);
            j++;
        }
    }
    
    /*for(int i=0; i < nj; i++ ) {
      if( i != nj -1 ) {  
        to_iDyn(f_i[i+1].force,f);
        f = (idynChain[i+1]).getR()*f;
        to_iDyn(f_i[i+1].torque,mu);
        Vector r_n = ((idynChain)[i+1]).getH().subcol(0,3,3);
        mu = ((idynChain)[i+1]).getR()*mu+cross(r_n,f);
      } else {
        mu.zero();
      }*/
    return Mu;
}

yarp::sig::Vector idynSensorGetSensorForce_usingKDL(iCub::iDyn::iDynChain & idynChain,iCub::iDyn::iDynInvSensor & idynSensor,yarp::sig::Vector & ddp0)
{
    //H_i^s
    yarp::sig::Matrix H = localSE3inv(idynSensor.getH());
    int sensor_link = idynSensor.getSensorLink();
    
    KDL::Wrenches f_i;

    int nj = idynChain.getN();
    int ns = nj+1;
    //int sensor_link = idynSensor.getSensorLink();


    yarp::sig::Vector f(3);
    
    f_i =  idynChainGet_usingKDL_aux(idynChain,idynSensor,ddp0);

    assert(f_i.size() == ns);
    

    to_iDyn(f_i[sensor_link+1].force,f);
    f = H.submatrix(0,2,0,2)*f;
    return f;
}

yarp::sig::Vector idynSensorGetSensorMoment_usingKDL(iCub::iDyn::iDynChain & idynChain,iCub::iDyn::iDynInvSensor & idynSensor,yarp::sig::Vector & ddp0)
{
    
    //H_i^s
    yarp::sig::Matrix H = iCub::ctrl::SE3inv(idynSensor.getH());
    int sensor_link = idynSensor.getSensorLink();
    
    KDL::Wrenches f_i;

    int nj = idynChain.getN();
    int ns = nj+1;
    //int sensor_link = idynSensor.getSensorLink();


    yarp::sig::Vector f(3),mu(3);
    
    f_i =  idynChainGet_usingKDL_aux(idynChain,idynSensor,ddp0);

    assert(f_i.size() == ns);
    

    to_iDyn(f_i[sensor_link+1].force,f);
    f = H.submatrix(0,2,0,2)*f;
    
    to_iDyn(f_i[sensor_link+1].torque,mu);
    yarp::sig::Vector r_n = H.subcol(0,3,3);
    mu = H.submatrix(0,2,0,2)*mu+yarp::math::cross(r_n,f);
    return mu;
}

yarp::sig::Vector idynChainGetTorques_usingKDL(iCub::iDyn::iDynChain & idynChain,iCub::iDyn::iDynInvSensor & idynSensor,yarp::sig::Vector & ddp0)
{
        yarp::sig::Vector q,dq,ddq;
    q = idynChain.getAng();
    dq = idynChain.getDAng();
    ddq = idynChain.getD2Ang();
    
    KDL::Chain kdlChain;
    
    idynSensorChain2kdlChain(idynChain,idynSensor,kdlChain);
    
    int nj = idynChain.getN();
    
    KDL::JntArray jointpositions = KDL::JntArray(nj);
    KDL::JntArray jointvel = KDL::JntArray(nj);
    KDL::JntArray jointacc = KDL::JntArray(nj);
    KDL::JntArray torques = KDL::JntArray(nj);

    for(unsigned int i=0;i<nj;i++){
        jointpositions(i)=q[i];
        jointvel(i) = dq[i];
        jointacc(i) = ddq[i];
    }
    
    KDL::Wrenches f_ext(nj+1);
    KDL::Wrenches f_int(nj+1);
    
    KDL::Vector grav_kdl;
    idynVector2kdlVector(idynChain.getH0().submatrix(0,2,0,2).transposed()*ddp0,grav_kdl);
    
    KDL::ChainIdSolver_RNE_IW neSolver = KDL::ChainIdSolver_RNE_IW(kdlChain,-grav_kdl);
    
    int ret = neSolver.CartToJnt_and_internal_wrenches(jointpositions,jointvel,jointacc,f_ext,torques,f_int);
    assert(ret >= 0);
    
    yarp::sig::Vector ret_tau;
    kdlJntArray2idynVector(torques,ret_tau);
    return ret_tau;
}

KDL::Wrenches idynChainGet_usingKDL_aux(iCub::iDyn::iDynChain & idynChain, iCub::iDyn::iDynInvSensor & idynSensor,yarp::sig::Vector & ddp0)
{
    yarp::sig::Vector q,dq,ddq;
    q = idynChain.getAng();
    dq = idynChain.getDAng();
    ddq = idynChain.getD2Ang();
    
    KDL::Chain kdlChain;
    
    std::vector<std::string> la_joints;
    la_joints.push_back("left_shoulder_pitch");
    la_joints.push_back("left_shoulder_roll");
    la_joints.push_back("left_arm_ft_sensor");
    la_joints.push_back("left_shoulder_yaw");
    la_joints.push_back("left_elbow");
    la_joints.push_back("left_wrist_prosup");
    la_joints.push_back("left_wrist_pitch");
    la_joints.push_back("left_wrist_yaw");


    
    idynSensorChain2kdlChain(idynChain,idynSensor,kdlChain,la_joints,la_joints);
    
    std::cout << kdlChain << std::endl;
    
    int nj = idynChain.getN();
    
    //cout << "idynChainGet_usingKDL_aux with sensor"  << " nrJoints " << kdlChain.getNrOfJoints() <<  " nrsegments " << kdlChain.getNrOfSegments() << endl;

    assert(nj==kdlChain.getNrOfJoints());
    assert(nj+1==kdlChain.getNrOfSegments());
    
    
    KDL::JntArray jointpositions = KDL::JntArray(nj);
    KDL::JntArray jointvel = KDL::JntArray(nj);
    KDL::JntArray jointacc = KDL::JntArray(nj);
    KDL::JntArray torques = KDL::JntArray(nj);

    for(unsigned int i=0;i<nj;i++){
        jointpositions(i)=q[i];
        jointvel(i) = dq[i];
        jointacc(i) = ddq[i];
    }
    
    KDL::Wrenches f_ext(nj+1);
    KDL::Wrenches f_int(nj+1);
    
    KDL::Vector grav_kdl;
    idynVector2kdlVector(idynChain.getH0().submatrix(0,2,0,2).transposed()*ddp0,grav_kdl);
    
    KDL::ChainIdSolver_RNE_IW neSolver = KDL::ChainIdSolver_RNE_IW(kdlChain,-grav_kdl);
    
    int ret = neSolver.CartToJnt_and_internal_wrenches(jointpositions,jointvel,jointacc,f_ext,torques,f_int);
    assert(ret >= 0);
    
    return f_int;
}

