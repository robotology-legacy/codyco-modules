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

#include "Reference.h"
#include "config.h"

#include <yarp/os/Mutex.h>
#include <yarp/os/LockGuard.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Thread.h>
#include <set>


//C++ 11
//namespace std {
//    template <> struct hash<codyco::torquebalancing::ReferenceDelegate>
//    {
//        size_t operator()(const codyco::torquebalancing::ReferenceDelegate & x) const
//        {
//            return reinterpret_cast<size_t>(&x);
//        }
//    };
//}

namespace codyco {
    namespace torquebalancing {

#pragma mark - Reference implementation
        

        ReferenceDelegate::~ReferenceDelegate() {}
        void ReferenceDelegate::referenceWillChangeValue(const codyco::torquebalancing::Reference &, const Eigen::VectorXd& newValue) {}
        void ReferenceDelegate::referenceDidChangeValue(const codyco::torquebalancing::Reference &) {}

        bool ReferenceDelegate::operator==(const ReferenceDelegate &other) const {return &other == this;}
        bool ReferenceDelegate::operator<(const ReferenceDelegate &other) const { return &other < this; }

        //If we want to drop the dependency from YARP of this class it is easy:
        //mutex and guard are in C++11
        //the private part can be moved to a private header and different implementations can be created

        class ReferenceReader : public yarp::os::TypedReaderCallback<yarp::sig::Vector>{
            Reference &reference;
            Eigen::Map<Eigen::VectorXd> temporaryValue;

        public:
            ReferenceReader(Reference& reference)
            : reference(reference)
            , temporaryValue(NULL, reference.valueSize())
            { }

            virtual void onRead(yarp::sig::Vector &read) {
                if (read.size() == temporaryValue.size()) {
                    new (&temporaryValue) Eigen::Map<Eigen::VectorXd>(read.data(), read.size());
                    reference.setValue(temporaryValue);
                }
            }
        };

        struct ReferencePrivateImplementation {
            yarp::os::Mutex m_lock;
            std::set<ReferenceDelegate*> delegates;

            ReferenceReader reader;
            yarp::os::BufferedPort<yarp::sig::Vector> *readerPort;

            ReferencePrivateImplementation(Reference& reference): reader(reference), readerPort(NULL) {}

            ~ReferencePrivateImplementation() {
                if (readerPort) {
                    readerPort->interrupt();
                    readerPort->close();
                    delete readerPort;
                    readerPort = NULL;
                }
            }

        private:
            ReferencePrivateImplementation(const ReferencePrivateImplementation& other):reader(other.reader) {}
            ReferencePrivateImplementation& operator=(const ReferencePrivateImplementation &);
        };


        Reference::Reference(int referenceSize)
        : m_value(referenceSize)
        , m_valid(false)
        , m_valueSize(referenceSize)
        , m_implementation(new ReferencePrivateImplementation(*this)) {}
        
        Reference::~Reference()
        {
            this->tearDownReaderThread();
            if (m_implementation) {
                ReferencePrivateImplementation *implementation = static_cast<ReferencePrivateImplementation*>(m_implementation);
                delete implementation;
                m_implementation = 0;
            }
        }

        void Reference::addDelegate(ReferenceDelegate *delegate)
        {
            if (!delegate) return;
            ReferencePrivateImplementation *implementation = static_cast<ReferencePrivateImplementation*>(m_implementation);
            implementation->delegates.insert(delegate);
        }

        void Reference::removeDelegate(ReferenceDelegate *delegate)
        {
            if (!delegate) return;
            ReferencePrivateImplementation *implementation = static_cast<ReferencePrivateImplementation*>(m_implementation);
            implementation->delegates.erase(delegate);
        }

        bool Reference::setUpReaderThread(std::string portName)
        {
            ReferencePrivateImplementation *implementation = static_cast<ReferencePrivateImplementation*>(m_implementation);
            bool result = false;
            if (!implementation->readerPort) {
                implementation->readerPort = new yarp::os::BufferedPort<yarp::sig::Vector>();
                if (!implementation->readerPort) return false;
//                implementation->readerPort->useCallback(implementation->reader);
            }
//            if (!implementation->readerPort->isClosed()) {
//                return false;
//            }
            result = implementation->readerPort->open(portName);
            implementation->readerPort->useCallback(implementation->reader);

            return result;
        }

        bool Reference::tearDownReaderThread()
        {
            ReferencePrivateImplementation *implementation = static_cast<ReferencePrivateImplementation*>(m_implementation);
            if (implementation->readerPort) {
                implementation->readerPort->interrupt();
                implementation->readerPort->close();
                delete implementation->readerPort;
                implementation->readerPort = NULL;
            }
            return true;
        }

        const Eigen::VectorXd& Reference::value() const
        {
            ReferencePrivateImplementation *implementation = static_cast<ReferencePrivateImplementation*>(m_implementation);
            yarp::os::LockGuard guard(implementation->m_lock);
            return m_value;
        }
        
        void Reference::setValue(const Eigen::Ref<const Eigen::VectorXd>& _value)
        {
            ReferencePrivateImplementation *implementation = static_cast<ReferencePrivateImplementation*>(m_implementation);

            if (implementation->delegates.size() > 0)
                for (std::set<ReferenceDelegate*>::iterator delegate = implementation->delegates.begin(); delegate != implementation->delegates.end(); delegate++) {
                    (*delegate)->referenceWillChangeValue(*this, _value);
                }
            {
                yarp::os::LockGuard guard(implementation->m_lock);
                m_value = _value;
                m_valid = true;
            }
            if (implementation->delegates.size() > 0)
                for (std::set<ReferenceDelegate*>::iterator delegate = implementation->delegates.begin(); delegate != implementation->delegates.end(); delegate++) {
                    (*delegate)->referenceDidChangeValue(*this);
                }
        }

        void Reference::setValid(bool isValid)
        {
            ReferencePrivateImplementation *implementation = static_cast<ReferencePrivateImplementation*>(m_implementation);
            yarp::os::LockGuard guard(implementation->m_lock);
            m_valid = isValid;
        }
        
        bool Reference::isValid()
        {
            ReferencePrivateImplementation *implementation = static_cast<ReferencePrivateImplementation*>(m_implementation);
            yarp::os::LockGuard guard(implementation->m_lock);
            return m_valid;
        }
        
        int Reference::valueSize() const
        {
            return m_valueSize;
        }
        
#pragma mark - ControllerReferences implementation
        
        ControllerReferences::ControllerReferences(int actuatedDOFs)
        : m_desiredCOMPosition(3)
        , m_desiredJointsPosition(actuatedDOFs)
        , m_desiredCOMAcceleration(3)
        , m_desiredLeftHandPosition(7)
        , m_desiredRightHandPosition(7)
        , m_desiredLeftHandForce(6)
        , m_desiredRightHandForce(6)
        , m_desiredJointsConfiguration(actuatedDOFs) {}

        Reference& ControllerReferences::desiredCOMPosition()
        {
            return m_desiredCOMPosition;
        }

        Reference& ControllerReferences::desiredJointsPosition()
        {
            return m_desiredJointsPosition;
        }
        
        Reference& ControllerReferences::desiredCOMAcceleration()
        {
            return m_desiredCOMAcceleration;
        }
        
        Reference& ControllerReferences::desiredLeftHandPosition()
        {
            return m_desiredLeftHandPosition;
        }

        Reference& ControllerReferences::desiredRightHandPosition()
        {
            return m_desiredRightHandPosition;
        }
        
        Reference& ControllerReferences::desiredLeftHandForce()
        {
            return m_desiredLeftHandForce;
        }
        
        Reference& ControllerReferences::desiredRightHandForce()
        {
            return m_desiredRightHandForce;
        }
        
        Reference& ControllerReferences::desiredJointsConfiguration()
        {
            return m_desiredJointsConfiguration;
        }
    }
}
