/*
 * Copyright (C) 2013  CoDyCo Consortium
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
 *
 * Authors: Andrea Del Prete, Marco Randazzo
 * email: andrea.delprete@iit.it - marco.randazzo@iit.it
 */

#include <wbi/wbi.h>
#include <sstream>

using namespace std;
using namespace wbi;

const double PI         = 3.1415926535897932384626433832795;
const double deg2rad    = 0.01745329251994329576923690768488;
const double rad2deg    = 57.2957795130823208767981548141052;
double epsilon          = 0.000001;



LocalIdList::LocalIdList()
{}
LocalIdList::LocalIdList(int bp, int j0)
{ pushId(bp,j0); }
LocalIdList::LocalIdList(int bp, int j0, int j1)
{ pushId(bp,j0); pushId(bp,j1);}
LocalIdList::LocalIdList(int bp, int j0, int j1, int j2)
{ pushId(bp,j0); pushId(bp,j1); pushId(bp,j2); }
LocalIdList::LocalIdList(int bp, int j0, int j1, int j2, int j3)
{ pushId(bp,j0); pushId(bp,j1); pushId(bp,j2); pushId(bp,j3); }
LocalIdList::LocalIdList(int bp, int j0, int j1, int j2, int j3, int j4)
{ pushId(bp,j0); pushId(bp,j1); pushId(bp,j2); pushId(bp,j3); pushId(bp,j4); }
LocalIdList::LocalIdList(int bp, int j0, int j1, int j2, int j3, int j4, int j5)
{ pushId(bp,j0); pushId(bp,j1); pushId(bp,j2); pushId(bp,j3); pushId(bp,j4); pushId(bp,j5); }
LocalIdList::LocalIdList(int bp, int j0, int j1, int j2, int j3, int j4, int j5, int j6)
{ pushId(bp,j0); pushId(bp,j1); pushId(bp,j2); pushId(bp,j3); pushId(bp,j4); pushId(bp,j5); pushId(bp,j6); }

LocalIdList::LocalIdList(const LocalIdList &lid1, const LocalIdList &lid2)
{ addIdList(lid1); addIdList(lid2); }
LocalIdList::LocalIdList(const LocalIdList &lid1, const LocalIdList &lid2, const LocalIdList &lid3)
{ addIdList(lid1); addIdList(lid2); addIdList(lid3); }
LocalIdList::LocalIdList(const LocalIdList &lid1, const LocalIdList &lid2, const LocalIdList &lid3, const LocalIdList &lid4)
{ addIdList(lid1); addIdList(lid2); addIdList(lid3); addIdList(lid4); }
LocalIdList::LocalIdList(const LocalIdList &lid1, const LocalIdList &lid2, const LocalIdList &lid3, const LocalIdList &lid4, const LocalIdList &lid5)
{ addIdList(lid1); addIdList(lid2); addIdList(lid3); addIdList(lid4); addIdList(lid5); }
LocalIdList::LocalIdList(const LocalIdList &lid1, const LocalIdList &lid2, const LocalIdList &lid3, const LocalIdList &lid4, const LocalIdList &lid5, const LocalIdList &lid6)
{ addIdList(lid1); addIdList(lid2); addIdList(lid3); addIdList(lid4); addIdList(lid5); addIdList(lid6); }

LocalIdList::~LocalIdList() {}

void LocalIdList::pushId(int bp, int i)
{
    //If the bodypart is not part of the LocalIdList, create it
    if( this->find(bp) == this->end() ) 
    {
        (*this)[bp] = std::vector<int>(0); 
    }
    (*this)[bp].push_back(i);
}

/** Convert a local id into a global id */
int LocalIdList::localToGlobalId(const LocalId &i) const
{
    int gid = 0;
    FOR_ALL_BODY_PARTS_OF(itBp, (*this))
        if(itBp->first == i.bodyPart)
        {
            FOR_ALL_JOINTS(itBp, itJ)
                if(i.index == *itJ)
                    return gid;
                else
                    gid++;
        }
        else
            gid += itBp->second.size();
    return gid;
}

/** Convert a global id into a local id */
LocalId LocalIdList::globalToLocalId(int globalId) const
{
    // iterate over list decreasing globalId for each id encountered
    // when globalId==0 => we found the local id
    FOR_ALL_BODY_PARTS_OF(itBp, (*this))
        if(globalId > (int)itBp->second.size())  // if globalId greater than # of ids of current body part 
            globalId -= itBp->second.size();
        else
        {
            FOR_ALL_JOINTS(itBp, itJ)
                if(globalId==0)
                    return LocalId(itBp->first, *itJ);
                else
                    globalId--;
        }
    return LocalId();
}

//Remove an existing joint 
bool LocalIdList::removeId(const LocalId &j)
{
    LocalIdList::iterator itBp = find(j.bodyPart);
    if(itBp == end())
        return false;
    FOR_ALL_JOINTS_NC(itBp, itJ)
    {
        if(j.index == *itJ)
        {
            itBp->second.erase(itJ);
            return true;
        }
    }
    return false;
}

bool LocalIdList::addId(const LocalId &i)
{
    if(containsId(i))
        return false;
    pushId(i.bodyPart, i.index);
    return true;
}

int LocalIdList::addIdList(const LocalIdList &jList)
{
    int count = 0;
    FOR_ALL_OF(itBp, itJ, jList)
        if(!containsId(LocalId(itBp->first,*itJ)))
        {
            pushId(itBp->first,*itJ);
            count++;
        }
    return count;
}

bool LocalIdList::containsId(const LocalId &i) const
{
    const_iterator it = find(i.bodyPart);
    if(it==end())
        return false;
    for(unsigned int j=0; j<it->second.size(); j++)
        if(it->second[j]==i.index)
            return true;
    return false;
}

// Get the number of ids in this list
unsigned int LocalIdList::size() const
{
    unsigned int s=0;
    FOR_ALL_BODY_PARTS_OF(itBp, (*this))
        s += itBp->second.size();
    return s;
}

string LocalIdList::toString() const
{
    stringstream s;
    FOR_ALL_BODY_PARTS_OF(itBp, (*this))
    {
        s << "(Body part "<< itBp->first<< " Joints ";
        FOR_ALL_JOINTS(itBp, itJ)
            s<< *itJ<< " ";
        s<< ") ";
    }
    return s.str();
}


/*******************************************************************************************************************/
/*******************************************************************************************************************/
/************************************************** ROTATION *******************************************************/
/*******************************************************************************************************************/
/*******************************************************************************************************************/

/************************************************************************************************/
/********************************************** GET *********************************************/
/************************************************************************************************/

void Rotation::getQuaternion(double& x, double& y, double& z, double& w) const
{
    double trace = (*this)(0,0) + (*this)(1,1) + (*this)(2,2) + 1.0;
    double epsilon=1E-12;
    if( trace > epsilon )
    {
        double s = 0.5 / sqrt(trace);
        w = 0.25 / s;
        x = ( (*this)(2,1) - (*this)(1,2) ) * s;
        y = ( (*this)(0,2) - (*this)(2,0) ) * s;
        z = ( (*this)(1,0) - (*this)(0,1) ) * s;
    }
    else
    {
        if ( (*this)(0,0) > (*this)(1,1) && (*this)(0,0) > (*this)(2,2) )
        {
            double s = 2.0 * sqrt( 1.0 + (*this)(0,0) - (*this)(1,1) - (*this)(2,2));
            w = ((*this)(2,1) - (*this)(1,2) ) / s;
            x = 0.25 * s;
            y = ((*this)(0,1) + (*this)(1,0) ) / s;
            z = ((*this)(0,2) + (*this)(2,0) ) / s;
        } 
        else if ((*this)(1,1) > (*this)(2,2)) 
        {
            double s = 2.0 * sqrt( 1.0 + (*this)(1,1) - (*this)(0,0) - (*this)(2,2));
            w = ((*this)(0,2) - (*this)(2,0) ) / s;
            x = ((*this)(0,1) + (*this)(1,0) ) / s;
            y = 0.25 * s;
            z = ((*this)(1,2) + (*this)(2,1) ) / s;
        }
        else 
        {
            double s = 2.0 * sqrt( 1.0 + (*this)(2,2) - (*this)(0,0) - (*this)(1,1) );
            w = ((*this)(1,0) - (*this)(0,1) ) / s;
            x = ((*this)(0,2) + (*this)(2,0) ) / s;
            y = ((*this)(1,2) + (*this)(2,1) ) / s;
            z = 0.25 * s;
        }
    }    
}

void Rotation::getRPY(double& roll,double& pitch,double& yaw) const
{
	double epsilon=1E-12;
	pitch = atan2(-data[6], sqrt( sqr(data[0]) +sqr(data[3]) )  );
    if ( fabs(pitch) > (PI/2.0-epsilon) ) 
    {
        yaw = atan2(-data[1], data[4]);
        roll  = 0.0;
    } 
    else 
    {
        roll  = atan2(data[7], data[8]);
        yaw   = atan2(data[3], data[0]);
    }
}

void Rotation::getEulerZYZ(double& alpha,double& beta,double& gamma) const 
{
    double epsilon = 1E-12;
    if (fabs(data[8]) > 1-epsilon  ) 
    {
        gamma=0.0;
        if (data[8]>0) 
        {
            beta = 0.0;
            alpha= atan2(data[3],data[0]);
        } 
        else 
        {
            beta = PI;
            alpha= atan2(-data[3],-data[0]);
        }
    } 
    else 
    {
        alpha=atan2(data[5], data[2]);
        beta=atan2(sqrt( sqr(data[6]) + sqr(data[7]) ), data[8]);
        gamma=atan2(data[7], -data[6]);
    }
}

void Rotation::getRotationVector(double &vX, double &vY, double &vZ) const
{
    double angle;
    getAxisAngle(vX, vY, vZ, angle);
    vX *= angle;
    vY *= angle;
    vZ *= angle;
}

void Rotation::getAxisAngle(double &axisX, double &axisY, double &axisZ, double &angle) const
{
    double x,y,z,w;
    getQuaternion(x,y,z,w);
    double norm = norm3d(x,y,z); //sqrt(x*x+y*y+z*z); 
    angle = 2*atan2(norm,w);
    if (norm>EPSILON){  axisX = x/norm;         axisY = y/norm;     axisZ = z/norm;   }
    else{               axisX = axisY = 0.0;    axisZ = 1.0;        angle = 0.0;      }
}

/************************************************************************************************/
/********************************************** STATIC ******************************************/
/************************************************************************************************/

Rotation Rotation::quaternion(double x,double y,double z, double w)
{
    double x2, y2, z2, w2;
    x2 = x*x;  y2 = y*y; z2 = z*z;  w2 = w*w;
    return Rotation(w2+x2-y2-z2, 2*x*y-2*w*z, 2*x*z+2*w*y,
                    2*x*y+2*w*z, w2-x2+y2-z2, 2*y*z-2*w*x,
                    2*x*z-2*w*y, 2*y*z+2*w*x, w2-x2-y2+z2);
}

Rotation Rotation::RPY(double roll,double pitch,double yaw)
{
    double ca1,cb1,cc1,sa1,sb1,sc1;
    ca1 = cos(yaw);   sa1 = sin(yaw);
    cb1 = cos(pitch); sb1 = sin(pitch);
    cc1 = cos(roll);  sc1 = sin(roll);
    return Rotation(ca1*cb1,  ca1*sb1*sc1 - sa1*cc1,   ca1*sb1*cc1 + sa1*sc1,
                    sa1*cb1,  sa1*sb1*sc1 + ca1*cc1,   sa1*sb1*cc1 - ca1*sc1,
                    -sb1,     cb1*sc1,                 cb1*cc1);
}

Rotation Rotation::eulerZYZ(double Alfa,double Beta,double Gamma) 
{
    double sa,ca,sb,cb,sg,cg;
    sa  = sin(Alfa);ca = cos(Alfa);
    sb  = sin(Beta);cb = cos(Beta);
    sg  = sin(Gamma);cg = cos(Gamma);
    return Rotation( ca*cb*cg-sa*sg,     -ca*cb*sg-sa*cg,        ca*sb,
                     sa*cb*cg+ca*sg,     -sa*cb*sg+ca*cg,        sa*sb,
                     -sb*cg ,                sb*sg,              cb );
}

Rotation Rotation::axisAngle(const double rotaxis[3], double angle) 
{
    // The formula is
    // V.(V.tr) + st*[V x] + ct*(I-V.(V.tr))
    // can be found by multiplying it with an arbitrary vector p
    // and noting that this vector is rotated.
    double v[3];
	double norm = norm3d(rotaxis);
    if(norm < EPSILON)
        return Rotation::identity();
    v[0] = rotaxis[0] / norm;
    v[1] = rotaxis[1] / norm;
    v[2] = rotaxis[2] / norm;
	return Rotation::axisAngle2(v, angle);
}

Rotation Rotation::axisAngle2(const double rotvec[3], double angle) 
{
    // rotvec should be normalized!
    double ct = cos(angle);             double st = sin(angle);             double vt = 1-ct;
    double m_vt_0=vt*rotvec[0];         double m_vt_1=vt*rotvec[1];         double m_vt_2=vt*rotvec[2];
    double m_st_0=rotvec[0]*st;         double m_st_1=rotvec[1]*st;         double m_st_2=rotvec[2]*st;
    double m_vt_0_1=m_vt_0*rotvec[1];   double m_vt_0_2=m_vt_0*rotvec[2];   double m_vt_1_2=m_vt_1*rotvec[2];
    return Rotation(ct      +  m_vt_0*rotvec[0],    -m_st_2 +  m_vt_0_1,            m_st_1  +  m_vt_0_2,
                    m_st_2  +  m_vt_0_1,            ct      +  m_vt_1*rotvec[1],    -m_st_0 +  m_vt_1_2,
                    -m_st_1 +  m_vt_0_2,            m_st_0  +  m_vt_1_2,            ct      +  m_vt_2*rotvec[2] );
}

Rotation Rotation::rotationVector(const double rotvec[3])
{
    double v[3];
	double angle = norm3d(rotvec);
    if(angle < EPSILON)
        return Rotation::identity();
    v[0] = rotvec[0] / angle;
    v[1] = rotvec[1] / angle;
    v[2] = rotvec[2] / angle;
	return Rotation::axisAngle2(v, angle);
}

/************************************************************************************************/
/****************************************** OPERATORS *******************************************/
/************************************************************************************************/

Rotation wbi::operator*(const Rotation& lhs,const Rotation& rhs) // Complexity : 27M+27A
{
    return Rotation(lhs.data[0]*rhs.data[0]+lhs.data[1]*rhs.data[3]+lhs.data[2]*rhs.data[6],
                    lhs.data[0]*rhs.data[1]+lhs.data[1]*rhs.data[4]+lhs.data[2]*rhs.data[7],
                    lhs.data[0]*rhs.data[2]+lhs.data[1]*rhs.data[5]+lhs.data[2]*rhs.data[8],
                    lhs.data[3]*rhs.data[0]+lhs.data[4]*rhs.data[3]+lhs.data[5]*rhs.data[6],
                    lhs.data[3]*rhs.data[1]+lhs.data[4]*rhs.data[4]+lhs.data[5]*rhs.data[7],
                    lhs.data[3]*rhs.data[2]+lhs.data[4]*rhs.data[5]+lhs.data[5]*rhs.data[8],
                    lhs.data[6]*rhs.data[0]+lhs.data[7]*rhs.data[3]+lhs.data[8]*rhs.data[6],
                    lhs.data[6]*rhs.data[1]+lhs.data[7]*rhs.data[4]+lhs.data[8]*rhs.data[7],
                    lhs.data[6]*rhs.data[2]+lhs.data[7]*rhs.data[5]+lhs.data[8]*rhs.data[8] );
}

bool wbi::isEqual(const Rotation& a, const Rotation& b, double eps) 
{
    return (wbi::equal(a.data[0],b.data[0],eps) &&
            wbi::equal(a.data[1],b.data[1],eps) &&
            wbi::equal(a.data[2],b.data[2],eps) &&
            wbi::equal(a.data[3],b.data[3],eps) &&
            wbi::equal(a.data[4],b.data[4],eps) &&
            wbi::equal(a.data[5],b.data[5],eps) &&
            wbi::equal(a.data[6],b.data[6],eps) &&
            wbi::equal(a.data[7],b.data[7],eps) &&
            wbi::equal(a.data[8],b.data[8],eps)    );
}

/************************************************************************************************/
/****************************************** FRAME ***********************************************/
/************************************************************************************************/

void Frame::get4x4Matrix(double d[16]) const
{
    for (int i=0;i<3;i++) 
    {
        for (int j=0;j<3;j++)
            d[i*4+j]=R(i,j);
        d[i*4+3] = p[i];
    }
    d[12]=d[13]=d[14]=0.0;
    d[15]=1.0;
}

Frame wbi::operator *(const Frame& lhs,const Frame& rhs)
// Complexity : 36M+36A
{
    double v[3];
    lhs.R.rotate(rhs.p, v);
    v[0]+=lhs.p[0]; v[1]+=lhs.p[1]; v[2]+=lhs.p[2]; 
    return Frame(lhs.R*rhs.R, v);
}

bool wbi::isEqual(const Frame& a, const Frame& b, double eps) 
{
    return (wbi::isEqual(a.R, b.R, eps) &&
            wbi::equal(a.p[0],b.p[0],eps) && 
            wbi::equal(a.p[1],b.p[1],eps) && 
            wbi::equal(a.p[2],b.p[2],eps));
}