#ifndef COORDINATOR_Y2_H
#define COORDINATOR_Y2_H

#include <yarp/os/RFModule.h>
#include <string>

namespace yarp {
    namespace os {
        template <class T>
        class BufferedPort;

        class Bottle;
        class Property;
        class RpcServer;
        class ResourceFinder;
    }
    namespace sig {
        class Vector;
    }
}

namespace codyco {
    namespace y2 {
        class Coordinator;
    }
}

class codyco::y2::Coordinator: public yarp::os::RFModule {
public:

    Coordinator();
    virtual ~Coordinator();

    virtual bool configure(yarp::os::ResourceFinder &);
    virtual bool updateModule();
    virtual double getPeriod();
    virtual bool close();
    virtual bool respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply);

private:
    double m_threadPeriod;
    std::string m_robotName;
    double m_motionDoneThreshold;
    yarp::os::BufferedPort<yarp::os::Property>* m_inputJointReferences;
    yarp::os::BufferedPort<yarp::sig::Vector>* m_outputTorqueControlledJointReferences;
    yarp::os::BufferedPort<yarp::sig::Vector>* m_outputComDesiredPosVelAcc;

    yarp::os::RpcServer *m_rpcServer;

    void * implementation;

    virtual bool cleanup();
    bool checkMotionDone(const yarp::sig::Vector &reference, const yarp::sig::Vector &actual);

};


#endif /* end of include guard: COORDINATOR_Y2_H */
