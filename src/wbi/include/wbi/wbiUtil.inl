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

#include <sstream>
#include <math.h>

double wbi::norm3d(const double v[3])
{
    double tmp1=fabs(v[0]), tmp2=fabs(v[1]);
    if (tmp1 >= tmp2) 
    {
        tmp2=fabs(v[2]);
        if (tmp1 >= tmp2) 
        {
            if (tmp1 == 0) // only to everything exactly zero case, all other are handled correctly
                return 0.0;
            return tmp1*sqrt(1+sqr(v[1]/v[0])+sqr(v[2]/v[0]));
        } 
        else
            return tmp2*sqrt(1+sqr(v[0]/v[2])+sqr(v[1]/v[2]));
    } 
    else 
    {
        tmp1=fabs(v[2]);
        if (tmp2 > tmp1)
            return tmp2*sqrt(1+sqr(v[0]/v[1])+sqr(v[2]/v[1]));
        else
            return tmp1*sqrt(1+sqr(v[0]/v[2])+sqr(v[1]/v[2]));
    }
}

/*******************************************************************************************************************/
/*******************************************************************************************************************/
/************************************************** ROTATION *******************************************************/
/*******************************************************************************************************************/
/*******************************************************************************************************************/
Rotation::Rotation(const double R[9]) 
{
    for(int i=0;i<9;i++)
        data[i] = R[i];
}

Rotation::Rotation( double Xx,double Yx,double Zx,
                    double Xy,double Yy,double Zy,
                    double Xz,double Yz,double Zz)
{
    data[0]=Xx; data[1]=Yx; data[2]=Zx;
    data[3]=Xy; data[4]=Yy; data[5]=Zy;
    data[6]=Xz; data[7]=Yz; data[8]=Zz;
}

Rotation::Rotation(const double *x, const double *y, const double *z) 
{
    data[0] = x[0]; data[3] = x[1]; data[6] = x[2];
    data[1] = y[0]; data[4] = y[1]; data[7] = y[2];
    data[2] = z[0]; data[5] = z[1]; data[8] = z[2];
}

/************************************************************************************************/
/****************************************** OPERATORS *******************************************/
/************************************************************************************************/

double& Rotation::operator()(int i,int j) 
{
    FRAMES_CHECKI((0<=i)&&(i<=2)&&(0<=j)&&(j<=2));
    return data[i*3+j];
}

double Rotation::operator()(int i,int j) const 
{
    FRAMES_CHECKI((0<=i)&&(i<=2)&&(0<=j)&&(j<=2));
    return data[i*3+j];
}

//Rotation& Rotation::operator=(const Rotation& arg) 
//{
//    int count=9;
//    while (count--) data[count] = arg.data[count];
//    return *this;
//}

std::string Rotation::toString(int precision, const char *sep, const char *endRow) const
{
    std::stringstream res;
    if(precision>=0)
        res<<std::setprecision(precision);
    res<<data[0]<<sep<<data[1]<<sep<<data[2]<<endRow
        <<data[3]<<sep<<data[4]<<sep<<data[5]<<endRow
        <<data[6]<<sep<<data[7]<<sep<<data[8];
    return res.str();
}


/************************************************************************************************/
/*********************************** ROTATION AND INVERSE ***************************************/
/************************************************************************************************/

void Rotation::rotate(const double *v, double *out) const 
{
    out[0] = data[0]*v[0] + data[1]*v[1] + data[2]*v[2];
    out[1] = data[3]*v[0] + data[4]*v[1] + data[5]*v[2];
    out[2] = data[6]*v[0] + data[7]*v[1] + data[8]*v[2];
}

void Rotation::rotate(double *v) const 
{
    double v0=v[0], v1=v[1];
    v[0] = data[0]*v0 + data[1]*v1 + data[2]*v[2];
    v[1] = data[3]*v0 + data[4]*v1 + data[5]*v[2];
    v[2] = data[6]*v0 + data[7]*v1 + data[8]*v[2];
}

void Rotation::rotateInverse(const double *v, double *out) const 
{
    out[0] = data[0]*v[0] + data[3]*v[1] + data[6]*v[2];
    out[1] = data[1]*v[0] + data[4]*v[1] + data[7]*v[2];
    out[2] = data[2]*v[0] + data[5]*v[1] + data[8]*v[2];
}

void Rotation::rotateInverse(double *v) const 
{
    double v0=v[0], v1=v[1];
    v[0] = data[0]*v0 + data[3]*v1 + data[6]*v[2];
    v[1] = data[1]*v0 + data[4]*v1 + data[7]*v[2];
    v[2] = data[2]*v0 + data[5]*v1 + data[8]*v[2];
}

Rotation Rotation::rotateInverse(const Rotation &R) const
{
    return Rotation(data[0]*R.data[0] + data[3]*R.data[3] + data[6]*R.data[6],
                    data[0]*R.data[1] + data[3]*R.data[4] + data[6]*R.data[7],
                    data[0]*R.data[2] + data[3]*R.data[5] + data[6]*R.data[8],
                    data[1]*R.data[0] + data[4]*R.data[3] + data[7]*R.data[6],
                    data[1]*R.data[1] + data[4]*R.data[4] + data[7]*R.data[7],
                    data[1]*R.data[2] + data[4]*R.data[5] + data[7]*R.data[8],
                    data[2]*R.data[0] + data[5]*R.data[3] + data[8]*R.data[6],
                    data[2]*R.data[1] + data[5]*R.data[4] + data[8]*R.data[7],
                    data[2]*R.data[2] + data[5]*R.data[5] + data[8]*R.data[8] );
}

Rotation Rotation::getInverse() const
{
    Rotation tmp(*this);
    tmp.setToInverse();
    return tmp;
}

void Rotation::setToInverse()
{
    double tmp;
    tmp = data[1];data[1]=data[3];data[3]=tmp;
    tmp = data[2];data[2]=data[6];data[6]=tmp;
    tmp = data[5];data[5]=data[7];data[7]=tmp;
}

/************************************************************************************************/
/********************************************** SET ROTATIONS ***********************************/
/************************************************************************************************/

void Rotation::doRotX(double angle)
{
    // *this = *this * ROT(X,angle)
    double cs = cos(angle);
    double sn = sin(angle);
    double x1,x2,x3;
    x1  = cs* (*this)(0,1) + sn* (*this)(0,2);
    x2  = cs* (*this)(1,1) + sn* (*this)(1,2);
    x3  = cs* (*this)(2,1) + sn* (*this)(2,2);
    (*this)(0,2) = -sn* (*this)(0,1) + cs* (*this)(0,2);
    (*this)(1,2) = -sn* (*this)(1,1) + cs* (*this)(1,2);
    (*this)(2,2) = -sn* (*this)(2,1) + cs* (*this)(2,2);
    (*this)(0,1) = x1;
    (*this)(1,1) = x2;
    (*this)(2,1) = x3;
}

void Rotation::doRotY(double angle)
{
    double cs = cos(angle);
    double sn = sin(angle);
    double x1,x2,x3;
    x1  = cs* (*this)(0,0) - sn* (*this)(0,2);
    x2  = cs* (*this)(1,0) - sn* (*this)(1,2);
    x3  = cs* (*this)(2,0) - sn* (*this)(2,2);
    (*this)(0,2) = sn* (*this)(0,0) + cs* (*this)(0,2);
    (*this)(1,2) = sn* (*this)(1,0) + cs* (*this)(1,2);
    (*this)(2,2) = sn* (*this)(2,0) + cs* (*this)(2,2);
    (*this)(0,0) = x1;
    (*this)(1,0) = x2;
    (*this)(2,0) = x3;
}

void Rotation::doRotZ(double angle)
{
    double cs = cos(angle);
    double sn = sin(angle);
    double x1,x2,x3;
    x1  = cs* (*this)(0,0) + sn* (*this)(0,1);
    x2  = cs* (*this)(1,0) + sn* (*this)(1,1);
    x3  = cs* (*this)(2,0) + sn* (*this)(2,1);
    (*this)(0,1) = -sn* (*this)(0,0) + cs* (*this)(0,1);
    (*this)(1,1) = -sn* (*this)(1,0) + cs* (*this)(1,1);
    (*this)(2,1) = -sn* (*this)(2,0) + cs* (*this)(2,1);
    (*this)(0,0) = x1;
    (*this)(1,0) = x2;
    (*this)(2,0) = x3;
}

/************************************************************************************************/
/********************************************** STATIC ******************************************/
/************************************************************************************************/

Rotation Rotation::rotX(double angle) 
{
    double cs=cos(angle);
    double sn=sin(angle);
    return Rotation(1,0,0,0,cs,-sn,0,sn,cs);
}

Rotation Rotation::rotY(double angle) 
{
    double cs=cos(angle);
    double sn=sin(angle);
    return Rotation(cs,0,sn,0,1,0,-sn,0,cs);
}

Rotation Rotation::rotZ(double angle) 
{
    double cs=cos(angle);
    double sn=sin(angle);
    return Rotation(cs,-sn,0,sn,cs,0,0,0,1);
}
