/*
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * email: silvio.traversaro@iit.it
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

#include "wbiy/wbiy.h"
#include <string>

#include <cmath>

#include <iCub/skinDynLib/common.h>
#include <iCub/ctrl/math.h>

using namespace std;
using namespace wbi;
using namespace wbiy;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace iCub::skinDynLib;

// iterate over all body parts
#define FOR_ALL_BODY_PARTS(itBp)            FOR_ALL_BODY_PARTS_OF(itBp, jointIdList)
// iterate over all joints of all body parts
#define FOR_ALL(itBp, itJ)                  FOR_ALL_OF(itBp, itJ, jointIdList)
// print the floating point vector pointed by data of size size and name name
#define PRINT_VECTOR(name, size, data)  printf("%s: ",name); for(int i=0;i<size;i++) printf("%.1lf ",data[i]); printf("\n");

#define PRINT_MATRIX(name, rows, cols, data) printf("%s:\n",name); for(int i=0;i<rows;i++){ for(int j=0;j<cols;j++) printf("%.1lf ",data[i*(cols)+j]); printf("\n"); }

Vector dcm2quaternion(const Matrix &R, unsigned int verbose)
{
    if ((R.rows()<3) || (R.cols()<3))
    {
        if (verbose)
            fprintf(stderr,"dcm2quaternion() failed\n");

        return Vector(0);
    }

    Vector v(4);
    v[0]=R(2,1)-R(1,2);
    v[1]=R(0,2)-R(2,0);
    v[2]=R(1,0)-R(0,1);
    v[3]=0.0;
    double r=yarp::math::norm(v);
    double theta=atan2(0.5*r,0.5*(R(0,0)+R(1,1)+R(2,2)-1));
    
    double x,y,z,w;
    
    if (r<1e-9)
    {
        double trace = R[0][0] + R[1][1] + R[2][2]; // I removed + 1.0f; see discussion with Ethan
        if( trace > 0 ) {// I changed M_EPSILON to 0
            double s = 0.5f / sqrt(trace+ 1.0f);
            w = 0.25f / s;
            x = ( R(2,1) - R(1,2) ) * s;
            y = ( R(0,2) - R(2,0) ) * s;
            z = ( R(1,0) - R(0,1) ) * s;
        } else {
            if ( R(0,0) > R(1,1) && R(0,0) > R(2,2) ) {
                double s = 2.0f * sqrt( 1.0f + R(0,0) - R(1,1) - R(2,2));
                w = (R(2,1)- R(1,2) ) / s;
                x = 0.25f * s;
                y = (R(0,1) + R(1,0) ) / s;
                z = (R(0,2) + R(2,0) ) / s;
            } else if (R(1,1) > R(2,2)) {
                double s = 2.0f * sqrt( 1.0f + R(1,1) - R(0,0) - R(2,2));
                w = (R(0,2) - R(2,0) ) / s;
                x = (R(0,1) + R(1,0) ) / s;
                y = 0.25f * s;
                z = (R(1,2) + R(2,1) ) / s;
            } else {
                double s = 2.0f * sqrt( 1.0f + R(2,2) - R(0,0) - R(1,1) );
                w = (R(1,0) - R(0,1) ) / s;
                x = (R(0,2) + R(2,0) ) / s;
                y = (R(1,2) + R(2,1) ) / s;
                z = 0.25f * s;
            }
        }
    }
    
    v[0] = x;
    v[1] = y;
    v[2] = z;
    v[3] = w;

    return v;
}

Matrix quaternion2dcm(const Vector &v, unsigned int verbose=false)
{
    if (v.length()<4)
    {
        if (verbose)
            fprintf(stderr,"quaternion2dcm() failed\n");
    
        return Matrix(0,0);
    }

    Matrix R(4,4);
    R.eye();

    double x=v[0];
    double y=v[1];
    double z=v[2];
    double w=v[3];

    R(0,0)=1-2*(y*y+z*z);
    R(0,1)=2*(x*y+w*z);
    R(0,2)=2*(x*z-w*y);
    R(1,0)=2*(x*y-w*z);
    R(1,1)=1-2*(x*x+z*z);
    R(1,2)=2*(y*z+w*x);
    R(2,0)=2*(x*z+w*y);
    R(2,1)=2*(y*z-w*x);
    R(2,2)=1-2*(x*x+y*y);

    return R;
}


// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          ICUB WHOLE BODY MODEL
// *********************************************************************************************************************
// *********************************************************************************************************************
icubWholeBodyModel::icubWholeBodyModel(const char* _name, const char* _robotName, const std::vector<std::string> &_bodyPartNames, 
    int head_version, int legs_version, double* initial_q)
    : dof(0), name(_name), robot(_robotName), bodyPartNames(_bodyPartNames)
{
    version.head_version = head_version;
    version.legs_version = legs_version;
    p_icub_model = new iCub::iDynTree::iCubTree(version);
    all_q.resize(p_icub_model->getNrOfDOFs(),0.0);
    all_q_min = all_q_max = all_ddq = all_dq = all_q;
    
    world_base_transformation.resize(4,4);
    world_base_transformation.eye();
    
    v_base.resize(3,0.0);
    
    a_base = omega_base = domega_base = v_base;
    
    if( initial_q != 0 ) {
        memcpy(all_q.data(),initial_q,all_q.size()*sizeof(double));
    }
}

bool icubWholeBodyModel::init()
{
    bool initDone = true;
    FOR_ALL_BODY_PARTS(itBp)
        initDone = initDone && openDrivers(itBp->first);
    return initDone && (p_icub_model->getNrOfDOFs() > 0);
}

bool icubWholeBodyModel::openDrivers(int bp)
{
    ilim[bp]=0; dd[bp]=0;
    if(!openPolyDriver(name, robot, dd[bp], bodyPartNames[bp]))
        return false;
    bool ok = dd[bp]->view(ilim[bp]);   //if(!isRobotSimulator(robot))
    if(ok) 
        return true;
    fprintf(stderr, "Problem initializing drivers of %s\n", bodyPartNames[bp].c_str());
    return false;
}

bool icubWholeBodyModel::close()
{
    bool ok = true;
    FOR_ALL_BODY_PARTS(itBp)
    {
        assert(dd[itBp->first]!=NULL);
        ok = ok && dd[itBp->first]->close();
    }
    if(p_icub_model) { delete p_icub_model; p_icub_model = 0; }
    return ok;
}

int icubWholeBodyModel::getDoFs()
{
    return dof;
}

bool icubWholeBodyModel::removeJoint(const wbi::LocalId &j)
{
    if(!jointIdList.removeId(j))
        return false;
    dof--;
    return true;
}

bool icubWholeBodyModel::addJoint(const wbi::LocalId &j)
{
    if(!jointIdList.addId(j))
        return false;
    
    dof++;
    return true;
}

int icubWholeBodyModel::addJoints(const wbi::LocalIdList &j)
{
    int count = jointIdList.addIdList(j);
    dof += count;
    return count;
}

bool icubWholeBodyModel::getLinkId(const char *linkName, int &linkId)
{
    linkId = p_icub_model->getLinkIndex(linkName);
    return linkId>=0;
}

bool icubWholeBodyModel::convertBasePose(const Frame &xBase, yarp::sig::Matrix & H_world_base)
{
    if( H_world_base.cols() != 4 || H_world_base.rows() != 4 )
        H_world_base.resize(4,4);
    xBase.as4x4Matrix(H_world_base.data());
    return true;
}

bool icubWholeBodyModel::convertBaseVelocity(const double *dxB, yarp::sig::Vector & v_b, yarp::sig::Vector & omega_b)
{
    v_b[0] = dxB[0];
    v_b[1] = dxB[1];
    v_b[2] = dxB[2];
    omega_b[0] = dxB[3];
    omega_b[1] = dxB[4];
    omega_b[2] = dxB[5];
    return true;
}

bool icubWholeBodyModel::convertBaseAcceleration(const double *ddxB, yarp::sig::Vector & a_b, yarp::sig::Vector & domega_b)
{
    a_b[0] = ddxB[0];
    a_b[1] = ddxB[1];
    a_b[2] = ddxB[2];
    domega_b[0] = ddxB[3];
    domega_b[1] = ddxB[4];
    domega_b[2] = ddxB[5];
    return true;
}

bool icubWholeBodyModel::convertQ(const double *_q_input, yarp::sig::Vector & q_complete_output)
{
    int i=0;
    FOR_ALL_BODY_PARTS_OF(itBp, jointIdList) {
        FOR_ALL_JOINTS(itBp, itJ) {
            double tmp;
            tmp = _q_input[i];
            assert(p_icub_model->getDOFIndex(itBp->first,*itJ) >= 0);
            assert(p_icub_model->getDOFIndex(itBp->first,*itJ) < (int)q_complete_output.size());
            q_complete_output[p_icub_model->getDOFIndex(itBp->first,*itJ)] = tmp;
            i++;
        }
    }
    return true;
}

bool icubWholeBodyModel::convertQ(const yarp::sig::Vector & q_complete_input, double *_q_output )
{
    int i=0;
    FOR_ALL_BODY_PARTS_OF(itBp, jointIdList) {
        FOR_ALL_JOINTS(itBp, itJ) {
             _q_output[i] = q_complete_input[p_icub_model->getDOFIndex(itBp->first,*itJ)];
            i++;
        }
    }
    return true;
}

bool icubWholeBodyModel::convertDQ(const double *_dq_input, yarp::sig::Vector & dq_complete_output)
{
    int i=0;
    FOR_ALL_BODY_PARTS_OF(itBp, jointIdList) {
        FOR_ALL_JOINTS(itBp, itJ) {
            dq_complete_output[p_icub_model->getDOFIndex(itBp->first,*itJ)] = _dq_input[i];
            i++;
        }
    }
    return true;
}

bool icubWholeBodyModel::convertDDQ(const double *_ddq_input, yarp::sig::Vector & ddq_complete_output)
{
    int i=0;
    FOR_ALL_BODY_PARTS_OF(itBp, jointIdList) {
        FOR_ALL_JOINTS(itBp, itJ) {
            ddq_complete_output[p_icub_model->getDOFIndex(itBp->first,*itJ)] = _ddq_input[i];
            i++;
        }
    }
    return true;
}

bool icubWholeBodyModel::getJointLimits(double *qMin, double *qMax, int joint)
{
    if( (joint < 0 || joint >= (int)jointIdList.size()) && joint != -1 ) { return false; }

    if(joint>=0)
    {
        LocalId lid = jointIdList.globalToLocalId(joint);
        int index = lid.bodyPart==TORSO ? 2-lid.index : lid.index;
        assert(ilim[lid.bodyPart]!=NULL);
        bool res = ilim[lid.bodyPart]->getLimits(index, qMin, qMax);
        if(res)
        {
            *qMin = (*qMin) * CTRL_DEG2RAD;   // convert from deg to rad
            *qMax = (*qMax) * CTRL_DEG2RAD;   // convert from deg to rad
        }
        return res;
    }
    
    bool res = true;
    int n = jointIdList.size();
    for(int i=0; i<n; i++)
        res = res && getJointLimits(qMin+i, qMax+i, i);
    return res;

    // OLD IMPLEMENTATION
    //all_q_min = p_icub_model->getJointBoundMin();
    //all_q_max = p_icub_model->getJointBoundMax();

    //if( joint == -1 ) {
    //    //Get all joint limits
    //    convertQ(all_q_min,qMin);
    //    convertQ(all_q_max,qMax);
    //} else {
    //    //Get only a joint
    //    LocalId loc_id = jointIdList.globalToLocalId(joint);
    //    *qMin = all_q_min[p_icub_model->getDOFIndex(loc_id.bodyPart,loc_id.index)];
    //    *qMax = all_q_max[p_icub_model->getDOFIndex(loc_id.bodyPart,loc_id.index)];
    //}
    //return true;
}

bool icubWholeBodyModel::computeH(double *q, const Frame &xBase, int linkId, double *H)
{
    if( (linkId < 0 || linkId >= p_icub_model->getNrOfLinks()) && linkId != COM_LINK_ID ) return false;
    
    convertBasePose(xBase,world_base_transformation);
    convertQ(q,all_q);

    p_icub_model->setWorldBasePose(world_base_transformation);
    p_icub_model->setAng(all_q);
    
    Matrix H_result;
    if( linkId != COM_LINK_ID ) {
        H_result = p_icub_model->getPosition(linkId);
        if( H_result.cols() != 4 || H_result.rows() != 4 ) { return false; }
    } else {
       H_result = Matrix(4,4);
       H_result.eye();
       Vector com = p_icub_model->getCOM();
       if( com.size() == 0 ) { return false; }
       H_result.setSubcol(com,0,3);
    }

    memcpy(H,H_result.data(),4*4*sizeof(double));
    return true;
}


bool icubWholeBodyModel::computeJacobian(double *q, const Frame &xBase, int linkId, double *J, double *pos)
{
    if( (linkId < 0 || linkId >= p_icub_model->getNrOfLinks()) && linkId != COM_LINK_ID ) return false;
    
    if( pos != 0 ) return false; //not implemented yet
    
    bool ret_val;
    
    int dof_jacobian = dof+6;
    Matrix complete_jacobian(6,all_q.size()+6), reduced_jacobian(6,dof_jacobian);
    
    convertBasePose(xBase,world_base_transformation);
    convertQ(q,all_q);
    
    p_icub_model->setWorldBasePose(world_base_transformation);
    p_icub_model->setAng(all_q);
    
    //Get Jacobian, the one of the link or the one of the COM
    if( linkId != COM_LINK_ID ) {
         ret_val = p_icub_model->getJacobian(linkId,complete_jacobian);
         if( !ret_val ) return false;
    } else {
         ret_val = p_icub_model->getCOMJacobian(complete_jacobian);
         if( !ret_val ) return false;
    }

    
    int i=0;
    FOR_ALL_BODY_PARTS_OF(itBp, jointIdList) {
        FOR_ALL_JOINTS(itBp, itJ) {
            reduced_jacobian.setCol(i+6,complete_jacobian.getCol(6+p_icub_model->getDOFIndex(itBp->first,*itJ)));
            i++;
        }
    }
    reduced_jacobian.setSubmatrix(complete_jacobian.submatrix(0,5,0,5),0,0);
    memcpy(J,reduced_jacobian.data(),sizeof(double)*6*dof_jacobian);

#ifndef NDEBUG
    //printf("J of link %d:\n%s\n", linkId, reduced_jacobian.submatrix(0,5,0,9).toString(1).c_str());
    //printf("J of link %d:\n%s\n", linkId, reduced_jacobian.submatrix(0,5,10,19).toString(1).c_str());
    //printf("J of link %d:\n%s\n", linkId, reduced_jacobian.submatrix(0,5,20,29).toString(1).c_str());
#endif
    
    return true;
}

bool icubWholeBodyModel::computeDJdq(double *q, const Frame &xBase, double *dq, double *dxB, int linkId, double *dJdq, double *pos)
{
    return false;    
}

bool icubWholeBodyModel::forwardKinematics(double *q, const Frame &xB, int linkId, double *x)
{
    if( (linkId < 0 || linkId >= p_icub_model->getNrOfLinks()) && linkId != COM_LINK_ID ) return false;
    
    convertBasePose(xB,world_base_transformation);
    convertQ(q,all_q);

    p_icub_model->setWorldBasePose(world_base_transformation);
    p_icub_model->setAng(all_q);
    
    Matrix H_result;

    if( linkId != COM_LINK_ID ) {
        H_result = p_icub_model->getPosition(linkId);
        if( H_result.cols() != 4 || H_result.rows() != 4 ) { return false; }
    } else {
       H_result = Matrix(4,4);
       H_result.eye();
       Vector com = p_icub_model->getCOM();
       if( com.size() == 0 ) { return false; }
       H_result.setSubcol(com,0,3);
    }

    
    
    Vector axisangle(4);
    
    x[0] = H_result(0,3);
    x[1] = H_result(1,3);
    x[2] = H_result(2,3);
    
    axisangle = iCub::ctrl::dcm2axis(H_result.submatrix(0,2,0,2));
    
    x[3] = axisangle(0);
    x[4] = axisangle(1);
    x[5] = axisangle(2);
    x[6] = axisangle(3);

//#ifndef NDEBUG
//    PRINT_VECTOR("xB", 7, xB);
//    PRINT_VECTOR("q", jointIdList.size(), q);
//    PRINT_MATRIX("world_base_transformation", 4, 4, world_base_transformation.data());
//#endif
    
    return true;
}

bool icubWholeBodyModel::inverseDynamics(double *q, const Frame &xB, double *dq, double *dxB, double *ddq, double *ddxB, double *tau)
{
    return false;
}

bool icubWholeBodyModel::directDynamics(double *q, const Frame &xB, double *dq, double *dxB, double *M, double *h)
{
    return false;
}
