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

#ifndef REFERENCEGENERATOR_H
#define REFERENCEGENERATOR_H

#include <yarp/os/RateThread.h>

#include <Eigen/Core>

#include <cmath>
#include <yarp/os/Mutex.h>

//Visual Studio prior to 2013 lacks NAN support
#ifdef _MSC_VER
#ifndef NAN
static const unsigned long __nan[2] = {0xffffffff, 0x7fffffff};
#define NAN (*(const float *) __nan)
#endif
#endif

namespace wbi {
    class wholeBodyIterface;
}

namespace codyco {
    namespace torquebalancing {

        class ReferenceGeneratorInputReader;
        class ReferenceFilter;
        class Reference;

        //TODO: change name to PIDController or something else
        /** This class is responsible of generating a proper reference signal.
         *
         * This class is agnostic of the reference type and dimension.
         * It generates a reference signal as the output of a PID (min jerk trajectory also in the future?). It reads the current value for the signal and signal derivatives from a class which implements the ReferenceGeneratorInputReader protocol.
         * It then writes the computed reference to the reference object.
         *
         * This class is thread-safe, i.e., every access to variables are guarded by a mutex.
         */
        class ReferenceGenerator: public ::yarp::os::RateThread
        {
        public:
            /** Constructor.
             *
             * @param period thread period
             * @param reference object in which to save the computed reference trajectory
             * @param reader object implementing the ReferenceGeneratorInputReader protocol used to obtain the current state of the system.
             * @param name custom name to be assigned to the controller. Useful while debugging
             */
            ReferenceGenerator(int period,
                               Reference& outputReference,
                               ReferenceGeneratorInputReader& reader,
                               const std::string& name = "");

            /**
             * Destructor
             */
            ~ReferenceGenerator();

            virtual bool threadInit();
            virtual void threadRelease();
            virtual void run();

#pragma mark - Getter and setter

            ReferenceGeneratorInputReader& inputReader();

            /** Returns the name of the controller
             * @return the name of the controller
             */
            const std::string& name() const;

            //Object is copied inside. Passed object can be deallocated.
            void setReferenceFilter(ReferenceFilter* referenceFilter);

            const ReferenceFilter* referenceFilter();


            /** Returns the current signal reference used by this controller
             * @return the current signal reference
             */
            const Eigen::VectorXd& signalReference();

            /** Returns the current actual reference used by this reference generator.
             * @return the current actual reference
             *
             * \note if setReferenceFilter was called, actualReference is the filtered version of signalReference.
             */
            const Eigen::VectorXd& actualReference();

            /** @brief Sets the new reference for the controller.
             *
             * The reference will be used for the computation of the current error.
             * @note if resetFilter is true the filter will be initialized to coincide with the reference
             * @param reference the new reference for the controller
             * @param resetFilter initialize the smoothing filter if present
             */
            void setSignalReference(const Eigen::VectorXd& reference, bool resetFilter = false);

            /** Returns the current reference for the signal derivative used by this controller
             * @return the current reference for the derivative
             */
            const Eigen::VectorXd signalDerivativeReference();

            /** @brief Sets the new reference for the derivative of the signal
             *
             * The reference will be used for the computation of the error on the signal derivative.
             * @param derivativeReference the new reference for the derivative.
             */
            void setSignalDerivativeReference(const Eigen::VectorXd& derivativeReference);

            /** Returns the current feedforward term used by this controller
             * @return the current feedforward term
             */
            const Eigen::VectorXd& signalFeedForward();

            /** @brief Sets the feedforward signal for the controller.
             *
             * The feedforward term will summed to the error contribution.
             * @param feedforward the new feedforward term for the controller
             */
            void setSignalFeedForward(const Eigen::VectorXd& feedforward);

            /** Sets all the controller references at once.
             *
             * Use this method if you need to set all the references of the controller.
             * @note if resetFilter is true the filter will be initialized to coincide with the reference
             * @param reference the reference of the signal
             * @param derivativeReference the reference of the derivative of the signal
             * @param feedforward the feedforward term
             * @param resetFilter initialize the smoothing filter if present
             */
            void setAllReferences(const Eigen::VectorXd& reference,
                                  const Eigen::VectorXd& derivativeReference,
                                  const Eigen::VectorXd& feedforward,
                                  bool resetFilter = false);

            /** Returns if the current thread is active or not
             *
             * @return true if the thread is active. False otherwise
             */
            bool isActiveState();

            /** Sets the state of the controller.
             *
             * If the state is active the controller starts to compute and to update the trajectory. If false the controller does nothing.
             * If the controller is already in the state requested as input the command is ignored.
             * @param isActive the new state of the controller
             */
            void setActiveState(bool isActive);

            /** Returns the current proportional gains used by this controller.
             *
             * @return the used proportional gains
             */
            const Eigen::VectorXd& proportionalGains();

            /** Sets the new proportional gains to be used in the trajectory generation
             *
             * @param proportionalGains the new proportional gains
             */
            void setProportionalGains(const Eigen::VectorXd& proportionalGains);

            /** Returns the current derivative gains used by this controller.
             *
             * @return the used derivative gains
             */
            const Eigen::VectorXd& derivativeGains();

            /** Sets the new derivative gains to be used in the trajectory generation
             *
             * @param derivativeGains the new derivative gains
             */
            void setDerivativeGains(const Eigen::VectorXd& derivativeGains);

            /** Returns the current integral gains used by this controller.
             *
             * @return the used integral gains
             */
            const Eigen::VectorXd& integralGains();

            /** Sets the new integral gains to be used in the trajectory generation
             *
             * @param integralGains the new integral gains
             */
            void setIntegralGains(const Eigen::VectorXd& integralGains);

            /** Returns the current bound on the integral term used by this controller.
             *
             * @return the absolute value of the bound on the integral term
             */
            double integralLimit();

            /** @brief Sets the new bound on the integral term to be used in the trajectory generation.
             *
             * only the absolute vale of the limit will be considered.
             * The integral term will be bounded in module, i.e., -|integralLimit| < integralTerm < |integralLimit|.
             * To disable the limit you can pass a big number, e.g. std::numeric_limits<double>::max()
             * @param integralLimit the new bound on the integral term
             */
            void setIntegralLimit(double integralLimit);

            /** Sets all gain for the PID controller.
             *
             * This function sets all the gains of the controller in one shot.
             * If the new integral limit is NAN (C99 cmath constant) then it is ignored. Pass std::numeric_limits<double>::max() to disable integral limit.
             * @param proportionalGains the new proportional gains
             * @param derivativeGains the new derivative gains
             * @param integralGains the new integral gains.
             * @param integralLimit the new integral limit or NAN to ignore it
             */
            void setAllGains(const Eigen::VectorXd& proportionalGains,
                             const Eigen::VectorXd& derivativeGains,
                             const Eigen::VectorXd& integralGains,
                             double integralLimit = NAN);

            /** Returns the computed reference signal by this controller
             *
             * @return the output of this controller
             */
            const Eigen::VectorXd& computedReference();

            /** Returns the instantaneous error between the signal reference and its measured value.
             * @return the instantaneous error
             */
            const Eigen::VectorXd& instantaneousError();

            /** Returns the integral of the error from the activation time to now
             * @return the integral of the error
             */
            const Eigen::VectorXd& errorIntegral();

        private:

            void limitIntegral(const Eigen::Ref<Eigen::VectorXd>& integral, Eigen::Ref<Eigen::VectorXd> limitedIntegral);

            const std::string m_name;

            Reference& m_outputReference;
            ReferenceGeneratorInputReader& m_reader;
            ReferenceFilter* m_referenceFilter;

            Eigen::VectorXd m_computedReference;
            Eigen::VectorXd m_integralTerm;
            Eigen::VectorXd m_error;

            Eigen::VectorXd m_proportionalGains;
            Eigen::VectorXd m_derivativeGains;
            Eigen::VectorXd m_integralGains;

            double m_integralLimit;

            Eigen::VectorXd m_signalReference;
            Eigen::VectorXd m_signalDerivativeReference;
            Eigen::VectorXd m_signalFeedForward;

            double m_previousTime;
            bool m_active;

            //Utility variables
            Eigen::VectorXd m_currentSignalValue;
            Eigen::VectorXd m_actualReference;

            yarp::os::Mutex m_mutex;
        };

        /**
         * This protocol specifies methods to obtain the current values
         * of a signal and its derivative.
         */
        class ReferenceGeneratorInputReader {
        public:
            /** Virtual destructor
             */
            virtual ~ReferenceGeneratorInputReader();

            /** Perform configuration time initialization.
             * By default this method returns true.
             */
            virtual bool init();

            /** Gets the current value of the signal
             * @param context addition information for the method. Varies in implementation
             * @return the current value of the signal
             */
            virtual const Eigen::VectorXd& getSignal(long context = 0) = 0;

            /** Gets the current value of the signal derivative
             * @param context addition information for the method. Varies in implementation
             * @return the current value of the signal derivative
             */
            virtual const Eigen::VectorXd& getSignalDerivative(long context = 0) = 0;

            /** Gets the size of the vector returned by the methods of this protocol
             * @return the size of the vectors returned by the implementation of this protocol
             */
            virtual int signalSize() const = 0;
        };

        /**
         * This protocol specifies methods to modify the reference signal
         * for the controller, e.g. minimum jerk, smoothing, ...
         */
        class ReferenceFilter {
        public:
            virtual ~ReferenceFilter();

            virtual ReferenceFilter* clone() const = 0;

            virtual bool initializeTimeParameters(double sampleTime,
                                                  double duration) = 0;

            virtual bool computeReference(const Eigen::VectorXd& setPoint,
                                          const Eigen::VectorXd& currentValue,
                                          double initialTime = 0.0,
                                          bool initFilter = false) = 0;

            virtual bool updateTrajectoryForCurrentTime(double currentTime) = 0;

            virtual const Eigen::VectorXd& getComputedValue() = 0;
            virtual const Eigen::VectorXd& getComputedDerivativeValue() = 0;
            virtual const Eigen::VectorXd& getComputedSecondDerivativeValue() = 0;

        };
    }
}


#endif /* end of include guard: REFERENCEGENERATOR_H */
