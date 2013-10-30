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
 * Authors: Andrea Del Prete
 * email: andrea.delprete@iit.it
 */

#ifndef WBI_UTIL_H
#define WBI_UTIL_H

#include <vector>
#include <map>
#include <string>
#include <sstream>      // std::stringstream
#include <iomanip>      // std::setprecision
#include <cassert>

namespace wbi
{

// Turn on or off frames bounds checking
#ifdef NDEBUG
    #define FRAMES_CHECKI(a)
#else
    #define FRAMES_CHECKI(a) assert(a)
#endif
    
#ifdef _MSC_VER
    // Microsoft Visual C
    #define IMETHOD __forceinline
#else
    // Some other compiler, e.g. gcc
    #define IMETHOD inline
#endif

    // iterate over all body parts of the specified jointIds
#define FOR_ALL_BODY_PARTS_OF(itBp, jIds)   for (LocalIdList::const_iterator itBp=jIds.begin(); itBp!=jIds.end(); itBp++)
    // iterate over all body parts of the specified jointIds
#define FOR_ALL_BODY_PARTS_OF_NC(itBp, jIds)   for (LocalIdList::iterator itBp=jIds.begin(); itBp!=jIds.end(); itBp++)
    // iterate over all joints of the specified body part
#define FOR_ALL_JOINTS(itBp, itJ)           for(vector<int>::const_iterator itJ=itBp->second.begin(); itJ!=itBp->second.end(); itJ++)
    // as before, but it uses a nonconst iterator
#define FOR_ALL_JOINTS_NC(itBp, itJ)        for(vector<int>::iterator itJ=itBp->second.begin(); itJ!=itBp->second.end(); itJ++)
    // iterate over all joints of all body parts of the specified jointIds
#define FOR_ALL_OF(itBp, itJ, jIds)         FOR_ALL_BODY_PARTS_OF(itBp, jIds) FOR_ALL_JOINTS(itBp, itJ)
    // iterate over all joints of all body parts of the specified jointIds
#define FOR_ALL_OF_NC(itBp, itJ, jIds)      FOR_ALL_BODY_PARTS_OF_NC(itBp, jIds) FOR_ALL_JOINTS_NC(itBp, itJ)

#define EPSILON 1e-6        // tolerance used for zero detection

    /** Compute the square of the specified value. */
    inline double sqr(double arg) { return arg*arg; }

    /** Compare whether 2 doubles are equal in an eps-interval. On VC6, if a/b is -INF, it returns false. */
    inline bool equal(double a, double b, double eps=EPSILON){ return ((eps>=(a-b)) && ((a-b)>=-eps)); }

    inline double norm3d(const double v[3]);

    /**
     * Identifier composed by a body part identifier and a relative id that identifies the object locally (on the body part).
     */
    class LocalId
    {
    public:
        int bodyPart;               // body part id
        int index;                  // local id
        std::string description;    // description
        
        // CONSTRUCTORS
        LocalId(): bodyPart(0), index(0) {}
        LocalId(int _bp, unsigned int _j): bodyPart(_bp), index(_j) {}
        LocalId(int _bp, unsigned int _j, std::string &_desc): bodyPart(_bp), index(_j), description(_desc) {}

        // OPERATORS
        bool operator==(const LocalId &other) const { return (bodyPart==other.bodyPart && index==other.index); }
        bool operator> (const LocalId &o) const { return bodyPart>o.bodyPart || (bodyPart==o.bodyPart && index>o.index); }
        bool operator< (const LocalId &o) const { return bodyPart<o.bodyPart || (bodyPart==o.bodyPart && index<o.index); }
    };
    
    
    /**
     * List of identifiers, that is a map from body part identifiers to lists of numbers.
     */
    class LocalIdList : public std::map< int, std::vector<int> >
    {
    protected:
        /** Add the specified id without checking its existance. */
        void pushId(int bp, int i);
        
    public:
        LocalIdList();
        LocalIdList(int bp, int j0);
        /** Create an id list with the specified ids, all belonging to the same body part.
         * @param bp Body part
         * @param j0 First id
         * @param j1 Second id */
        LocalIdList(int bp, int j0, int j1);
        LocalIdList(int bp, int j0, int j1, int j2);
        LocalIdList(int bp, int j0, int j1, int j2, int j3);
        LocalIdList(int bp, int j0, int j1, int j2, int j3, int j4);
        LocalIdList(int bp, int j0, int j1, int j2, int j3, int j4, int j5);
        LocalIdList(int bp, int j0, int j1, int j2, int j3, int j4, int j5, int j6);

        LocalIdList(const LocalIdList &lid1, const LocalIdList &lid2);
        LocalIdList(const LocalIdList &lid1, const LocalIdList &lid2, const LocalIdList &lid3);
        LocalIdList(const LocalIdList &lid1, const LocalIdList &lid2, const LocalIdList &lid3, const LocalIdList &lid4);
        LocalIdList(const LocalIdList &lid1, const LocalIdList &lid2, const LocalIdList &lid3, const LocalIdList &lid4, const LocalIdList &lid5);
        
        /** Convert a local id to a global id */
        virtual int localToGlobalId(const LocalId &i) const;

        /** Convert a global id to a local id */
        virtual LocalId globalToLocalId(int globalId) const;
        
        /** Remove the specified joint from the list */
        virtual bool removeId(const LocalId &i);
        
        /** Add the specified id to the list.
         * @param i id to add
         * @return true if the id has been added, false if it was already present
         */
        virtual bool addId(const LocalId &i);
        
        /** Add the specified ids to the list.
         * @param i id list to add
         * @return the number of ids added to the list
         */
        virtual int addIdList(const LocalIdList &i);
        
        /** Get the number of ids in this list */
        virtual unsigned int size() const;
        
        /* Check whether the specified body part is present in this list. */
        virtual bool containsBodyPart(int bp) const{ return !(find(bp)==end()); }
        
        /* Check whether the specified id is present in this list. */        
        virtual bool containsId(const LocalId &i) const;
        
        virtual std::string toString() const;
    };


    
    /**
      \brief A rotation in 3 dimensional space.

      This class represents a rotation matrix with the following conventions:
     \verbatim
         Suppose V2 = R*V,                                    (1)
         V is expressed in frame B
         V2 is expressed in frame A
         This matrix R consists of 3 columns [ X,Y,Z ],
         X,Y, and Z contain the axes of frame B, expressed in frame A
         Because of linearity expr(1) is valid.
     \endverbatim
       This class only represents rotational_interpolation, not translation
     Two interpretations are possible for rotation angles.
     * if you rotate with angle around X frame A to have frame B,
       then the result of SetRotX is equal to frame B expressed wrt A.
         In code:
     \verbatim
          Rotation R;
          F_A_B = R.SetRotX(angle);
     \endverbatim
     * Secondly, if you take the following code :
     \verbatim
          Vector p,p2; Rotation R;
          R.SetRotX(angle);
          p2 = R*p;
     \endverbatim
       then the frame p2 is rotated around X axis with (-angle).
       Analogue reasonings can be applyd to SetRotY,SetRotZ,SetRot
     \par type
      Concrete implementation
    */
    class Rotation
    {
    public:
        double data[9];     // 9 elements of the 3x3 rotation matrix, stored by row (row major)

        /************************************************************************************************/
        /***************************************** CONSTRUCTORS *****************************************/
        /************************************************************************************************/
        
        /** Create an identity rotation matrix */
        inline Rotation() { *this = Rotation::identity(); }

        /** Create a rotation represented by the 9 specified values of the associated rotation matrix. */
        inline Rotation(const double R[9]);

        /** Create a rotation represented by the 9 specified values of the associated rotation matrix
         * @param x First column of the rotation matrix
         * @param y Second column of the rotation matrix
         * @param z Third column of the rotation matrix */
        inline Rotation(const double x[3], const double y[3], const double z[3]);

        /** Create a rotation represented by the 9 specified values of the associated rotation matrix
         * @param Xx Element of row 1 and column 1
         * @param Yx Element of row 1 and column 2
         * @param Zx Element of row 1 and column 3
         * @param Xy Element of row 2 and column 1
         * @param Yy Element of row 2 and column 2
         * @param Zy Element of row 2 and column 3
         * @param Xz Element of row 3 and column 1
         * @param Yz Element of row 3 and column 2
         * @param Zz Element of row 3 and column 3 */
        inline Rotation(double Xx,double Yx,double Zx,double Xy,double Yy,double Zy,double Xz,double Yz,double Zz);
        
        // default copy constructor and assignment operator are sufficient (they perform deep copies)

        /************************************************************************************************/
        /****************************************** OPERATORS *******************************************/
        /************************************************************************************************/
        //inline Rotation& operator=(const Rotation& arg);

        /** Access to elements 0..2,0..2, bounds are checked when NDEBUG is not set. */
        inline double& operator()(int i, int j);

        /** Access to elements 0..2,0..2, bounds are checked when NDEBUG is not set. */
        inline double operator() (int i, int j) const;

        /** Get a string representation of the 3x3 rotation matrix associated to this object.
          * @param precision Number of digits after the floating point
          * @param sep String used to separate the entries on the same row of the matrix
          * @param endRow String used to separate two consecutive rows of the matrix
          * @return A string representation of a 3x3 rotation matrix. */
        inline std::string toString(int precision=-1, const char *sep="\t", const char *endRow="\n") const;


        /************************************************************************************************/
        /*********************************** ROTATION AND INVERSE ***************************************/
        /************************************************************************************************/

        /** Rotate the specified vector with this rotation. */
        inline void rotate(const double v[3], double out[3]) const;

        /** Rotate the specified vector with this rotation. Modify the input vector. */
        inline void rotate(double v[3]) const;

        /** The same as this->getInverse().rotate(v, out) but more efficient. */
        inline void rotateInverse(const double v[3], double out[3]) const;

        /** The same as this->getInverse().rotate(v, out) but more efficient. Modify the input vector. */
        inline void rotateInverse(double v[3]) const;

        /** The same as this->Inverse()*R but more efficient. */
        inline Rotation rotateInverse(const Rotation &R) const;

        /** Gives back the inverse rotation matrix of *this. */
        inline Rotation getInverse() const;

        /** Sets the value of *this to its inverse. */
        inline void setToInverse();


        /************************************************************************************************/
        /********************************************** SET ROTATIONS ***********************************/
        /************************************************************************************************/

        //! The DoRot... functions apply a rotation R to *this,such that *this = *this * Rot..
        //! DoRot... functions are only defined when they can be executed more efficiently
        inline void doRotX(double angle);
        //! The DoRot... functions apply a rotation R to *this,such that *this = *this * Rot..
        //! DoRot... functions are only defined when they can be executed more efficiently
        inline void doRotY(double angle);
        //! The DoRot... functions apply a rotation R to *this,such that *this = *this * Rot..
        //! DoRot... functions are only defined when they can be executed more efficiently
        inline void doRotZ(double angle);

        //! Access to the underlying unitvectors of the rotation matrix
        inline void setUnitX(const double *x) { data[0]=x[0]; data[3]=x[1]; data[6]=x[2]; }
        //! Access to the underlying unitvectors of the rotation matrix
        inline void setUnitY(const double *y) { data[1]=y[0]; data[4]=y[1]; data[7]=y[2]; }
        //! Access to the underlying unitvectors of the rotation matrix
        inline void setUnitZ(const double *z) { data[2]=z[0]; data[5]=z[1]; data[8]=z[2]; }



        /************************************************************************************************/
        /********************************************** GET *********************************************/
        /************************************************************************************************/

        /** Returns a vector with the direction of the equiv. axis and with a norm equivalent to the rotation angle. */
        void getRotationVector(double &vX, double &vY, double &vZ) const;
        void getRotationVector(double v[3]) const { return getRotationVector(v[0], v[1], v[2]); };

	    void getAxisAngle(double &axisX, double &axisY, double &axisZ, double &angle) const;
        /** Returns the rotation angle around the equiv. axis.
         * Taken from Wikipedia http://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation 
	     * @param aa the rotation axis followed by the rotation angle (between [0..PI] )
	     */
        void getAxisAngle(double aa[4]) const{ return getAxisAngle(aa[0], aa[1], aa[2], aa[3]); }

        /** Get the quaternion of this matrix
         * \post the norm of (x,y,z,w) is 1
         * From the following sources:
         * http://web.archive.org/web/20041029003853/http:/www.j3d.org/matrix_faq/matrfaq_latest.html
         * http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
         * RobOOP::quaternion.cpp 
         */
        void getQuaternion(double& x, double& y, double& z, double& w) const;
        void getQuaternion(double q[4]) const { return getQuaternion(q[0], q[1], q[2], q[3]); }

        /**  Gives back a vector in RPY coordinates, variables are bound by
         -  -PI <= roll <= PI
         -   -PI <= Yaw  <= PI
         -  -PI/2 <= PITCH <= PI/2

	     convention :
	     - first rotate around X with roll,
	     - then around the old Y with pitch,
	     - then around old Z with yaw

	     if pitch == PI/2 or pitch == -PI/2, multiple solutions for gamma and alpha exist.  The solution where roll==0
	     is chosen.

	     Invariants:
	     - RPY(roll,pitch,yaw) == RPY( roll +/- PI, PI-pitch, yaw +/- PI )
	     - angles + 2*k*PI
        **/
        void getRPY(double& roll, double& pitch, double& yaw) const; 
        void getRPY(double rpy[3]) const { return getRPY(rpy[0], rpy[1], rpy[2]); } 

        /** Gives back the EulerZYZ convention description of the rotation matrix :
	     First rotate around Z with alpha,
	     then around the new Y with beta, then around
	     new Z with gamma.

	     Variables are bound by:
	     - (-PI <  alpha  <= PI),
	     - (0   <= beta  <= PI),
	     - (-PI <  gamma <= PI)

	     if beta==0 or beta==PI, then alpha and gamma are not unique, in this case gamma is chosen to be zero.
	     Invariants:
	       - EulerZYX(alpha,beta,gamma) == EulerZYX(alpha +/- PI, -beta, gamma +/- PI)
	       - angle + 2*k*PI
	     */
        void getEulerZYZ(double& alpha, double& beta, double& gamma) const;
        void getEulerZYZ(double zyz[3]) const { getEulerZYZ(zyz[0], zyz[1], zyz[2]); }

        /**   GetEulerZYX gets the euler ZYX parameters of a rotation :
         *   First rotate around Z with alfa,
         *   then around the new Y with beta, then around
         *   new X with gamma.
         *
         *  Range of the results of GetEulerZYX :
         *  -  -PI <= alfa <= PI
         *  -   -PI <= gamma <= PI
         *  -  -PI/2 <= beta <= PI/2
         *
         *  if beta == PI/2 or beta == -PI/2, multiple solutions for gamma and alpha exist.  The solution where gamma==0
         *  is chosen.
         *
         *
         *  Invariants:
         *  	- EulerZYX(alpha,beta,gamma) == EulerZYX(alpha +/- PI, PI-beta, gamma +/- PI)
         *  	- and also (angle + 2*k*PI)
         *
         *  Closely related to RPY-convention.
         **/
        inline void getEulerZYX(double& Alfa,double& Beta,double& Gamma) const { getRPY(Gamma,Beta,Alfa); }
        inline void getEulerZYX(double zyx[3]) const { getEulerZYX(zyx[0], zyx[1], zyx[2]); }

        /** Access to the underlying unitvectors of the rotation matrix. */
        inline void getUnitX(double x[3]) const { x[0]=data[0]; x[1]=data[3]; x[2]=data[6];}   

        /** Access to the underlying unitvectors of the rotation matrix. */
        inline void getUnitY(double y[3]) const { y[0]=data[1]; y[1]=data[4]; y[2]=data[7]; }
     
        /** Access to the underlying unitvectors of the rotation matrix. */
        inline void getUnitZ(double z[3]) const { z[0]=data[2]; z[1]=data[5]; z[2]=data[8]; }


        

        /************************************************************************************************/
        /********************************************** STATIC ******************************************/
        /************************************************************************************************/

        //! Gives back an identity rotaton matrix
        inline static Rotation identity(){ return Rotation(1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0); }

        //! The Rot... static functions give the value of the appropriate rotation matrix back.
        inline static Rotation rotX(double angle);
        //! The Rot... static functions give the value of the appropriate rotation matrix back.
        inline static Rotation rotY(double angle);
        //! The Rot... static functions give the value of the appropriate rotation matrix back.
        inline static Rotation rotZ(double angle);

        /** Along an arbitrary axes.  It is not necessary to normalize rotvec.
          * returns identity rotation matrix in the case that the norm of rotvec
          * is to small to be used.
          * @see Rot2 if you want to handle this error in another way. */
        static Rotation axisAngle(const double rotvec[3], double angle);
        static Rotation axisAngle(const double aa[4]){ return axisAngle(aa, aa[3]); }

        /** Along an arbitrary axis. rotvec should be normalized. */
        static Rotation axisAngle2(const double rotvec[3], double angle);
        static Rotation axisAngle2(const double aa[4]){ return axisAngle2(aa, aa[3]); }

        /* Set the value of this object to a rotation specified with RPY convention:
         * first rotate around X with roll, then around the old Y with pitch, 
         * then around old Z with yaw.
         *
         * Invariants:
         *  - RPY(roll,pitch,yaw) == RPY( roll +/- PI, PI-pitch, yaw +/- PI )
         */
        static Rotation RPY(double roll, double pitch, double yaw);
        static Rotation RPY(double rpy[3]){ return RPY(rpy[0], rpy[1], rpy[2]); }

        /** Gives back a rotation matrix specified with EulerZYZ convention :
	     *  - First rotate around Z with alfa,
	     *  - then around the new Y with beta,
	     *	- then around new Z with gamma.
	     *  Invariants:
	     *  - EulerZYX(alpha,beta,gamma) == EulerZYX(alpha +/- PHI, -beta, gamma +/- PI)
	     **/
        static Rotation eulerZYZ(double alfa, double beta, double gamma);
        static Rotation eulerZYZ(double zyz[3]){ return eulerZYZ(zyz[0], zyz[1], zyz[2]); }

        /**  EulerZYX constructs a Rotation from the Euler ZYX parameters:
         *   -  First rotate around Z with alfa,
         *   - then around the new Y with beta,
         *   - then around new X with gamma.
         *  Closely related to RPY-convention.
         *
         *  Invariants:
         *  	- EulerZYX(alpha,beta,gamma) == EulerZYX(alpha +/- PI, PI-beta, gamma +/- PI)
         **/
        inline static Rotation eulerZYX(double alfa, double beta, double gamma) { return RPY(gamma, beta, alfa); }
        inline static Rotation eulerZYX(double zyx[3]) { return eulerZYX(zyx[0], zyx[1], zyx[2]); }

        //! Sets the value of this object to a rotation specified with Quaternion convention.
        //! The norm of (x,y,z,w) should be equal to 1.
        static Rotation quaternion(double x, double y, double z, double w);
        inline static Rotation quaternion(double q[4]){ return quaternion(q[0], q[1], q[2], q[3]); }
    };

    //! The literal inequality operator!=()
    bool isEqual(const Rotation& a,const Rotation& b, double eps=EPSILON);

    //! The literal equality operator==(), also identical.
    inline bool operator==(const Rotation& a,const Rotation& b){ return isEqual(a,b,0.0); }

    //! The literal inequality operator!=()
    inline bool operator!=(const Rotation& a,const Rotation& b){ return !isEqual(a,b,0.0); }

    /** Multiplication between two rotations. */
    Rotation operator *(const Rotation& lhs, const Rotation& rhs);


// include inline function definitions
#include <wbi/wbiUtil.inl>
    
} // end namespace

#endif

