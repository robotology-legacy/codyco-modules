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

#include "wbiIcub/wholeBodyInterfaceIcub.h"
#include <string>

#include <cmath>

#include <iCub/skinDynLib/common.h>
#include <iCub/ctrl/math.h>

#include <yarp/math/Math.h>

#include <Eigen/Core>

using namespace std;
using namespace wbi;
using namespace wbiIcub;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::skinDynLib;

// iterate over all body parts
#define FOR_ALL_BODY_PARTS(itBp)            FOR_ALL_BODY_PARTS_OF(itBp, jointIdList)
// iterate over all joints of all body parts
#define FOR_ALL(itBp, itJ)                  FOR_ALL_OF(itBp, itJ, jointIdList)
// print the floating point vector pointed by data of size size and name name
#define PRINT_VECTOR(name, size, data)  printf("%s: ",name); for(int i=0;i<size;i++) printf("%.1lf ",data[i]); printf("\n");
// print the floating point matrix pointed by data of name "name"
#define PRINT_MATRIX(name, rows, cols, data) printf("%s:\n",name); for(int i=0;i<rows;i++){ for(int j=0;j<cols;j++) printf("%.1lf ",data[i*(cols)+j]); printf("\n"); }

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          ICUB WHOLE BODY MODEL
// *********************************************************************************************************************
// *********************************************************************************************************************
icubWholeBodyModel::icubWholeBodyModel(const char* _name, const char* _robotName, int head_version, int legs_version, 
    double* initial_q, const std::vector<std::string> &_bodyPartNames)
    : dof(0), six_elem_buffer(6,0.0), three_elem_buffer(3,0.0), name(_name), robot(_robotName), bodyPartNames(_bodyPartNames)
{
    std::string kinematic_base_link_name = "root_link";
    version.head_version = head_version;
    version.legs_version = legs_version;
    p_icub_model = new iCub::iDynTree::iCubTree(version,false,iCub::iDynTree::SKINDYNLIB_SERIALIZATION,0,kinematic_base_link_name);
    all_q.resize(p_icub_model->getNrOfDOFs(),0.0);
    all_q_min = all_q_max = all_ddq = all_dq = all_q;
    floating_base_mass_matrix.resize(p_icub_model->getNrOfDOFs(),p_icub_model->getNrOfDOFs());
    floating_base_mass_matrix.zero();
    
    world_base_transformation.resize(4,4);
    world_base_transformation.eye();
    
    v_base.resize(3,0.0);
    
    a_base = omega_base = domega_base = v_base;
    
    v_six_elems_base.resize(3,0.0);
    a_six_elems_base.resize(6,0.0);
    
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

bool icubWholeBodyModel::removeJoint(const wbi::LocalId &j)
{
    if(!jointIdList.removeId(j)) 
        return false;
    all_dq.zero();
    all_ddq.zero();
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

bool icubWholeBodyModel::convertBasePose(const Frame &xBase, yarp::sig::Matrix & H_world_base)
{
    if( H_world_base.cols() != 4 || H_world_base.rows() != 4 )
        H_world_base.resize(4,4);
    xBase.get4x4Matrix(H_world_base.data());
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

bool icubWholeBodyModel::convertBaseVelocity(const double *dxB, yarp::sig::Vector & v_b)
{
    v_b[0] = dxB[0];
    v_b[1] = dxB[1];
    v_b[2] = dxB[2];
    v_b[3] = dxB[3];
    v_b[4] = dxB[4];
    v_b[5] = dxB[5];
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

bool icubWholeBodyModel::convertBaseAcceleration(const double *ddxB, yarp::sig::Vector & a_b)
{
    a_b[0] = ddxB[0];
    a_b[1] = ddxB[1];
    a_b[2] = ddxB[2];
    a_b[3] = ddxB[3];
    a_b[4] = ddxB[4];
    a_b[5] = ddxB[5];
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

bool icubWholeBodyModel::convertGeneralizedTorques(yarp::sig::Vector idyntree_base_force, yarp::sig::Vector idyntree_torques, double * tau)
{
    if( idyntree_base_force.size() != 6 && idyntree_torques.size() != p_icub_model->getNrOfDOFs() ) { return false; }
    for(int j = 0; j < 6; j++ ) {
        tau[j] = idyntree_base_force[j];
    }
    int i = 0;
    FOR_ALL_BODY_PARTS_OF(itBp, jointIdList) {
        FOR_ALL_JOINTS(itBp, itJ) {
            tau[i+6] = idyntree_torques[p_icub_model->getDOFIndex(itBp->first,*itJ)];
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

bool icubWholeBodyModel::computeH(double *q, const Frame &xBase, int linkId, Frame &H)
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

    H.set4x4Matrix(H_result.data());
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

bool icubWholeBodyModel::computeDJdq(double *q, const Frame &xBase, double *dq, double *dxB, int linkID, double *dJdq, double *pos)
{
    if ((linkID < 0 || linkID >= p_icub_model->getNrOfLinks()) && linkID != COM_LINK_ID) return false;
    if (pos != 0) return false; //not implemented yet
    
    //joints
    convertQ(q, all_q);
    convertDQ(dq, all_dq);
    all_ddq.zero();
    
    //base
    convertBasePose(xBase, world_base_transformation);
    convertBaseVelocity(dxB, v_six_elems_base);
    a_six_elems_base.zero();
    
    p_icub_model->setAng(all_q);
    p_icub_model->setDAng(all_dq);
    p_icub_model->setD2Ang(all_ddq);
    
    p_icub_model->setWorldBasePose(world_base_transformation);
    
    //The setKinematicBaseVelAcc accepts the velocity and accelerations of the kinematic base in world orientation
    p_icub_model->setKinematicBaseVelAcc(v_six_elems_base,a_six_elems_base);
    
    p_icub_model->kinematicRNEA();
    
    bool ret;
    
    if( linkID != COM_LINK_ID ) {
        ret = p_icub_model->getAcc(linkID,six_elem_buffer);
    } else {
        // Only the linear part of DJdq is supported for the COM
        ret = p_icub_model->getAccCOM(three_elem_buffer);
        for(int i=0; i < 3; i++ ) {
            six_elem_buffer[i] = three_elem_buffer[i];
            six_elem_buffer[i+3] = 0.0;
        }
    }
    
    if( !ret ) {
        if( six_elem_buffer.size() != 6 ) {
            six_elem_buffer.resize(6,0.0);
        }
        return false;
    }
    
    //should I copy directly?
    memcpy(dJdq, six_elem_buffer.data(), sizeof(double) * six_elem_buffer.size());
    return true;
    
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
    /** \todo move all conversion (also the one relative to frames) in convert* functions */
    //Converting local wbi positions/velocity/acceleration to iDynTree one
    convertBasePose(xB,world_base_transformation);
    convertQ(q,all_q);
    convertBaseVelocity(dxB,v_base,omega_base);
    convertDQ(dq,all_dq);
    convertBaseAcceleration(ddxB,a_base,domega_base);
    convertDDQ(ddq,all_ddq);

    //Setting iDynTree variables
    p_icub_model->setWorldBasePose(world_base_transformation);
    p_icub_model->setAng(all_q);
    //The kinematic initial values are expressed in the imu link (in this case, the base) for iDynTree 
    yarp::sig::Matrix base_world_rotation = world_base_transformation.submatrix(0,2,0,2).transposed();
    
    p_icub_model->setInertialMeasure(base_world_rotation * omega_base,
                                     base_world_rotation * domega_base,
                                     base_world_rotation * a_base);
    p_icub_model->setDAng(all_dq);
    p_icub_model->setD2Ang(all_ddq);
    
    //Computing inverse dynamics
    p_icub_model->kinematicRNEA();
    p_icub_model->dynamicRNEA();
    
    //Get the output floating base torques and convert them to wbi generalized torques
    yarp::sig::Vector base_force = p_icub_model->getBaseForceTorque(iCub::iDynTree::WORLD_FRAME);
    
    convertGeneralizedTorques(base_force,p_icub_model->getTorques(),tau);
    
    return true;
}

bool icubWholeBodyModel::computeMassMatrix(double *q, const Frame &xBase, double *M)
{
    convertBasePose(xBase,world_base_transformation);
    convertQ(q,all_q);
    
    //Setting iDynTree variables
    p_icub_model->setWorldBasePose(world_base_transformation);
    p_icub_model->setAng(all_q);
    
    
    
    //iDynTree floating base mass matrix is already world orientation friendly 
    //(i.e. expects the base velocity to be expressed in world reference frame)
    p_icub_model->getFloatingBaseMassMatrix(floating_base_mass_matrix);
    
    if( reduced_floating_base_mass_matrix.cols() != 6+dof ||
        reduced_floating_base_mass_matrix.rows() != 6+dof ) {
        reduced_floating_base_mass_matrix.resize(6+dof,6+dof);
    }
    
    //Converting the iDynTree complete floating_base_mass_matrix to the reduced one
    //           that includes only the joint added in the wholeBodyModel interfacec
    
    //Given the structure of the wholeBodyModel, this manual quadratic loop is necessary
    //To speed-up the case in which all the joint are considered, it could be possible to add a check to directly copy the mass matrix
    
    //Using mapped eigen matrices to avoid the overhead of using setSubmatrix / submatrix methods in yarp
    Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > mapped_reduced_mm(reduced_floating_base_mass_matrix.data(),
                                                                                                           reduced_floating_base_mass_matrix.rows(),
                                                                                                           reduced_floating_base_mass_matrix.cols());

    Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > mapped_complete_mm(floating_base_mass_matrix.data(),
                                                                                                            floating_base_mass_matrix.rows(),
                                                                                                            floating_base_mass_matrix.cols());

    
    //Left top submatrix (spatial inertia matrix)
    mapped_reduced_mm.block<6,6>(0,0) = mapped_complete_mm.block<6,6>(0,0);
    
    //Rest of the matrix
    int reduced_dof_row=0;
    int reduced_dof_column = 0;
    FOR_ALL_BODY_PARTS_OF(row_bp, jointIdList) {
        FOR_ALL_JOINTS(row_bp, row_joint) {
            int complete_dof_row = p_icub_model->getDOFIndex(row_bp->first,*row_joint);
            
            //Left bottom submatrix 
            mapped_reduced_mm.block<1,6>(6+reduced_dof_row,0) = mapped_complete_mm.block<1,6>(6+complete_dof_row,0);
            
            ///Right top submatrix (using the row loop to avoid doing another loop)
            mapped_reduced_mm.block<6,1>(0,6+reduced_dof_row) =  mapped_reduced_mm.block<1,6>(6+reduced_dof_row,0).transpose();
            
            reduced_dof_column=0;
            FOR_ALL_BODY_PARTS_OF(column_bp, jointIdList) {
                FOR_ALL_JOINTS(column_bp, column_joint) {
                    int complete_dof_column = p_icub_model->getDOFIndex(column_bp->first,*column_joint);
                    mapped_reduced_mm(6+reduced_dof_row,6+reduced_dof_column) = 
                        mapped_complete_mm(6+complete_dof_row,6+complete_dof_column);
                    reduced_dof_column++;
                }
            }
            reduced_dof_row++;
        }
    }
    
    memcpy(M,reduced_floating_base_mass_matrix.data(),sizeof(double)*(6+dof)*(6+dof));
    
    return true;
}

bool icubWholeBodyModel::computeGeneralizedBiasForces(double *q, const Frame &xBase, double *dq, double *dxB, double *h)
{
/** \todo move all conversion (also the one relative to frames) in convert* functions */
    //Converting local wbi positions/velocity/acceleration to iDynTree one
    convertBasePose(xBase,world_base_transformation);
    convertQ(q,all_q);
    convertBaseVelocity(dxB,v_base,omega_base);
    convertDQ(dq,all_dq);
    yarp::sig::Vector ddxB(6, 0.0);
    convertBaseAcceleration(ddxB.data(),a_base,domega_base);
    yarp::sig::Vector ddq(dof, 0.0);
    convertDDQ(ddq.data(),all_ddq);

    //Setting iDynTree variables
    p_icub_model->setWorldBasePose(world_base_transformation);
    p_icub_model->setAng(all_q);
    //The kinematic initial values are expressed in the imu link (in this case, the base) for iDynTree 
    yarp::sig::Matrix base_world_rotation = world_base_transformation.submatrix(0,2,0,2).transposed();
    
    p_icub_model->setInertialMeasure(base_world_rotation * omega_base,
                                     base_world_rotation * domega_base,
                                     base_world_rotation * a_base);
    p_icub_model->setDAng(all_dq);
    p_icub_model->setD2Ang(all_ddq);
    
    //Computing inverse dynamics
    p_icub_model->kinematicRNEA();
    p_icub_model->dynamicRNEA();
    
    //Get the output floating base torques and convert them to wbi generalized torques
    yarp::sig::Vector base_force = p_icub_model->getBaseForceTorque(iCub::iDynTree::WORLD_FRAME);
    
    
    convertGeneralizedTorques(base_force,p_icub_model->getTorques(),h);
    
    return true;
}


    
