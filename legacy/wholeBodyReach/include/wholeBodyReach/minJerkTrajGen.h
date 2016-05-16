/**
 * Copyright (C) 2014 CoDyCo
 * @author: Francesco Romano
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

#ifndef WBR_MINIMUMJERKTRAJECTORYGENERATOR_H
#define WBR_MINIMUMJERKTRAJECTORYGENERATOR_H

#include <Eigen/Core>
#include <yarp/sig/Vector.h>

namespace Eigen
{
    typedef Matrix<double,Dynamic,Dynamic,RowMajor> MatrixRXd;     /// Dynamic matrix with row-major storage order
    typedef Matrix<double,6,6,RowMajor>             MatrixR6d;     /// 6x6 matrix with row-major storage order
    
    typedef Ref<VectorXd>                           VectorRef;      /// Type used to pass Eigen vectors by reference
    typedef Ref<MatrixRXd>                          MatrixRef;      /// Type used to pass Eigen matrices by reference
    typedef const Ref<const VectorXd>&              VectorConst;    /// Type used to pass Eigen vectors by const reference
    typedef const Ref<const MatrixRXd>&             MatrixConst;    /// Type used to pass Eigen matrices by const reference
}

namespace iCub {
    namespace ctrl {
        class minJerkTrajGen;
    }
}

namespace yarp {
    namespace sig {
        class Vector;
    }
}

namespace wholeBodyReach
{
    class MinJerkTrajGen
    {
    public:
        MinJerkTrajGen(int size, double sampleTime=1e-3, double trajDuration=1.0);
        
        virtual ~MinJerkTrajGen();
        
        virtual MinJerkTrajGen* clone() const;
        
        virtual bool initializeTimeParameters(double sampleTime, double trajDuration);
        
        virtual bool init(Eigen::VectorConst initialValue);
        
        /** Compute the next trajectory values (pos, vel, acc) using the last
          * set-point.
          */
        virtual bool computeNextValues();
        
        virtual bool computeNextValues(Eigen::VectorConst setPoint);
        
        virtual const Eigen::VectorXd& getPos();
        virtual const Eigen::VectorXd& getVel();
        virtual const Eigen::VectorXd& getAcc();
        
        virtual double getTrajectoryDuration(){ return _trajDuration; }
        virtual double getSampleTime(){ return _sampleTime; }
        virtual double setTrajectoryDuration(double trajDuration);
        virtual double setSampleTime(double sampleTime);
        
    private:
        int                             _size;
        double                          _sampleTime;
        double                          _trajDuration;
        iCub::ctrl::minJerkTrajGen*     _minimumJerkGenerator;
        
        Eigen::VectorXd     _posEig;
        Eigen::VectorXd     _velEig;
        Eigen::VectorXd     _accEig;

        yarp::sig::Vector               _setPointYarp;
        yarp::sig::Vector               _initialValueYarp;
    };

}

#endif /* end of include guard: WBR_MINIMUMJERKTRAJECTORYGENERATOR_H */
