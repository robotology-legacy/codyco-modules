#include "Coordinator.h"

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Value.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/LockGuard.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IControlMode2.h>
#include <yarp/dev/IControlLimits2.h>
#include <yarp/math/Math.h>
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>

#include <iCub/ctrl/minJerkCtrl.h>

#include <vector>
#include <list>
#include <algorithm>

namespace codyco {
    namespace y2 {

        // Some const variables useful to increase code readability
        const int COM_SIZE = 3;

        class CoordinatorData;

        class Reader : public yarp::os::TypedReaderCallback<yarp::os::Property> {
        private:
            CoordinatorData &data;
            double limitInput(double value, double min, double max);
        public:
            Reader(CoordinatorData& _data);
            virtual void onRead(yarp::os::Property& read);
        };

        //use delegation here
        struct CoordinatorData {
            yarp::os::Mutex mutex;

            bool referencesChanged;
            //Full vectors. This are split into "torque" and "pos"
            yarp::sig::Vector torsoJointReferences;
            yarp::sig::Vector leftArmJointReferences;
            yarp::sig::Vector rightArmJointReferences;
            //Full vectors. The legs are always assume to be controlled in "torque"
            yarp::sig::Vector leftLegReferences;
            yarp::sig::Vector rightLegReferences;
            //This vector is the "sum" of the torque controlled torso and arms plus the legs
            yarp::sig::Vector torqueControlOutputReferences;
            //Full vector of the reference set point for the com
            yarp::sig::Vector comReferences;

            yarp::sig::Vector torsoTorqueControlledJointReferences;
            yarp::sig::Vector torsoPositionControlledJointReferences;
            yarp::sig::Vector leftArmTorqueControlledJointReferences;
            yarp::sig::Vector leftArmPositionControlledJointReferences;
            yarp::sig::Vector rightArmTorqueControlledJointReferences;
            yarp::sig::Vector rightArmPositionControlledJointReferences;

            //Trajectory generators
            iCub::ctrl::minJerkTrajGen* torsoGenerator;
            iCub::ctrl::minJerkTrajGen* leftArmGenerator;
            iCub::ctrl::minJerkTrajGen* rightArmGenerator;
            iCub::ctrl::minJerkTrajGen* torqueBalancingGenerator;
            //Trajectory generator for the com
            iCub::ctrl::minJerkTrajGen* comGenerator;
            /** if true, stream the output of the trajectory generator */
            bool comTrajGenActive;
            yarp::sig::Vector comDesPos;
            yarp::sig::Vector comDesVel;
            yarp::sig::Vector comDesAcc;

            yarp::sig::Vector torsoCurrentPosition;
            yarp::sig::Vector leftArmCurrentPosition;
            yarp::sig::Vector rightArmCurrentPosition;
            yarp::sig::Vector leftLegCurrentPosition;
            yarp::sig::Vector rightLegCurrentPosition;

            std::vector<int> armJointIDs;
            std::vector<int> torsoJointIDs;
            yarp::dev::PolyDriver leftArmDriver;
            yarp::dev::PolyDriver rightArmDriver;
            yarp::dev::PolyDriver torsoDriver;
            yarp::dev::PolyDriver leftLegDriver;
            yarp::dev::PolyDriver rightLegDriver;

            yarp::dev::IEncoders *torsoEncoders;
            yarp::dev::IPositionDirect *torsoPositionControl;
            yarp::dev::IControlLimits *torsoLimitsInterface;
            yarp::dev::IEncoders *leftArmEncoders;
            yarp::dev::IPositionDirect *leftArmPositionControl;
            yarp::dev::IControlLimits *leftArmLimitsInterface;
            yarp::dev::IEncoders *rightArmEncoders;
            yarp::dev::IPositionDirect *rightArmPositionControl;
            yarp::dev::IControlLimits *rightArmLimitsInterface;

            yarp::dev::IEncoders *leftLegEncoders;
            yarp::dev::IControlLimits *leftLegLimitsInterface;
            yarp::dev::IEncoders *rightLegEncoders;
            yarp::dev::IControlLimits *rightLegLimitsInterface;

            yarp::sig::Vector torsoMinLimits;
            yarp::sig::Vector torsoMaxLimits;
            yarp::sig::Vector leftArmMinLimits;
            yarp::sig::Vector leftArmMaxLimits;
            yarp::sig::Vector rightArmMinLimits;
            yarp::sig::Vector rightArmMaxLimits;
            yarp::sig::Vector leftLegMinLimits;
            yarp::sig::Vector leftLegMaxLimits;
            yarp::sig::Vector rightLegMinLimits;
            yarp::sig::Vector rightLegMaxLimits;

            std::list<int> torsoMappingInformationComplement;
            std::list<int> armMappingInformationComplement;

            void mapInput(std::vector<int> &map, std::list<int> &complementMap,
                          yarp::sig::Vector& input, yarp::sig::Vector &mapped, yarp::sig::Vector &complementMapped);


            Reader reader;

            CoordinatorData()
            : referencesChanged(false)
            , reader(*this)
            , torsoGenerator(0), leftArmGenerator(0), rightArmGenerator(0) {}

            void init()
            {
                //Do torso
                for (int i = 0; i < torsoPositionControlledJointReferences.size() + torsoTorqueControlledJointReferences.size(); i++) {
                    torsoMappingInformationComplement.push_back(i);
                }

                if (torsoJointIDs.size() > 0) {
                    std::list<int>::iterator it = torsoMappingInformationComplement.begin();
                    while (it != torsoMappingInformationComplement.end()) {
                        std::vector<int>::const_iterator found = std::find(torsoJointIDs.begin(), torsoJointIDs.end(), *it);
                        if (found != torsoJointIDs.end()) {
                            std::list<int>::iterator toBeRemoved = it;
                            it++;
                            torsoMappingInformationComplement.erase(toBeRemoved);
                        }
                        it++;
                    }
                }

                fprintf(stderr, "Torso IDs\n");
                for (int i = 0; i < torsoJointIDs.size(); i++) {
                    fprintf(stderr, "%d ", torsoJointIDs[i]);
                }
                fprintf(stderr, "\n");

                fprintf(stderr, "Complement Torso IDs\n");
                for (std::list<int>::iterator it = torsoMappingInformationComplement.begin();
                     it != torsoMappingInformationComplement.end(); it++) {
                    fprintf(stderr, "%d ", *it);
                }
                fprintf(stderr, "\n");


                //Do Arm
                for (int i = 0; i < leftArmPositionControlledJointReferences.size() + leftArmTorqueControlledJointReferences.size(); i++) {
                    armMappingInformationComplement.push_back(i);
                }

                if (armJointIDs.size() > 0) {
                    std::list<int>::iterator it = armMappingInformationComplement.begin();
                    while (it != armMappingInformationComplement.end()) {
                        std::vector<int>::const_iterator found = std::find(armJointIDs.begin(), armJointIDs.end(), *it);
                        if (found != armJointIDs.end()) {
                            std::list<int>::iterator toBeRemoved = it;
                            it++;
                            armMappingInformationComplement.erase(toBeRemoved);
                        }
                        it++;
                    }
                }

                fprintf(stderr, "Arm Joint IDs\n");
                for (int i = 0; i < armJointIDs.size(); i++) {
                    fprintf(stderr, "%d ", armJointIDs[i]);
                }
                fprintf(stderr, "\n");
                fprintf(stderr, "Complement Arm IDs\n");
                for (std::list<int>::iterator it = armMappingInformationComplement.begin();
                     it != armMappingInformationComplement.end(); it++) {
                    fprintf(stderr, "%d ", *it);
                }
                fprintf(stderr, "\n");

            }

            void copyReferencesForTorqueOutput()
            {
                //Assuming the following order:
                // 1) Torso
                // 2) Left Arm
                // 3) Right Arm
                // 4) Left Leg
                // 5) Right Leg
                //TODO: outside demo we should make this more robust (mapping joint names?)

                int index = 0;
                //Torso
                if (torsoTorqueControlledJointReferences.size() == 3) {
                    torqueControlOutputReferences(0) = torsoTorqueControlledJointReferences(2);
                    torqueControlOutputReferences(1) = torsoTorqueControlledJointReferences(1);
                    torqueControlOutputReferences(2) = torsoTorqueControlledJointReferences(0);
                } else {
                    yWarning("Case not allowed. Setting zero reference for torso");
                    for (int i = 0; i < torsoTorqueControlledJointReferences.size(); i++) {
                        torqueControlOutputReferences(i) = 0.0;
                    }
                }
                index += torsoTorqueControlledJointReferences.size();

                //Left Arm
                torqueControlOutputReferences.setSubvector(index, leftArmTorqueControlledJointReferences);
                index += leftArmTorqueControlledJointReferences.size();
                //Right Arm
                torqueControlOutputReferences.setSubvector(index, rightArmTorqueControlledJointReferences);
                index += rightArmTorqueControlledJointReferences.size();
                //Left Leg
                torqueControlOutputReferences.setSubvector(index, leftLegReferences);
                index += leftLegReferences.size();
                //Right Leg
                torqueControlOutputReferences.setSubvector(index, rightLegReferences);
                index += rightLegReferences.size();
            }

            ~CoordinatorData() {}

        };


        Reader::Reader(CoordinatorData& _data)
        : data(_data) { }

        double Reader::limitInput(double value, double min, double max)
        {
            if (std::isnan(value) || std::isinf(value)) {
                yWarning("NaN or Inf detected");
                return 0;
            }
            if (value > max) {
                yWarning("Read value(%lf) is outside joint limit(%lf)", value, max);
                return max;
            } else if (value < min) {
                yWarning("Read value(%lf) is outside joint limit(%lf)", value, min);
                return min;
            }
            return value;
        }

        void Reader::onRead(yarp::os::Property& read) {
            using namespace yarp::os;
            using yarp::sig::Vector;
            using namespace yarp::math;

            Value &torso    = read.find("torso");
            Value &leftArm  = read.find("left_arm");
            Value &rightArm = read.find("right_arm");
            Value &leftLeg  = read.find("left_leg");
            Value &rightLeg = read.find("right_leg");
            Value &com      = read.find("com");

            LockGuard guard(data.mutex);
            if (!torso.isNull() && torso.isList()) {
                Bottle *list = torso.asList();
                int torsoSize = std::min((size_t)list->size(), data.torsoJointReferences.size());
                if (torsoSize == 3) {
                    data.torsoJointReferences(2) = limitInput(list->get(0).asDouble(), data.torsoMinLimits(2), data.torsoMaxLimits(2));
                    data.torsoJointReferences(1) = limitInput(list->get(1).asDouble(), data.torsoMinLimits(1), data.torsoMaxLimits(1));
                    data.torsoJointReferences(0) = limitInput(list->get(2).asDouble(), data.torsoMinLimits(0), data.torsoMaxLimits(0));

                    //read in radians
                    data.mapInput(data.torsoJointIDs, data.torsoMappingInformationComplement,
                                  data.torsoJointReferences, data.torsoPositionControlledJointReferences, data.torsoTorqueControlledJointReferences);
                }
            }

            if (!leftArm.isNull() && leftArm.isList()) {
                Bottle *list = leftArm.asList();
                for (int i = 0; i < std::min((size_t)list->size(), data.leftArmJointReferences.size()); i++) {
                    data.leftArmJointReferences(i) = limitInput(list->get(i).asDouble(),
                                                                data.leftArmMinLimits(i), data.leftArmMaxLimits(i));
                }
                //read in radians
                data.mapInput(data.armJointIDs, data.armMappingInformationComplement,
                              data.leftArmJointReferences, data.leftArmPositionControlledJointReferences, data.leftArmTorqueControlledJointReferences);
            }

            if (!rightArm.isNull() && rightArm.isList()) {
                Bottle *list = rightArm.asList();
                for (int i = 0; i < std::min((size_t)list->size(), data.rightArmJointReferences.size()); i++) {
                    data.rightArmJointReferences(i) = limitInput(list->get(i).asDouble(),
                                                                 data.rightArmMinLimits(i), data.rightArmMaxLimits(i));
                }
                //read in radians
                data.mapInput(data.armJointIDs, data.armMappingInformationComplement,
                              data.rightArmJointReferences, data.rightArmPositionControlledJointReferences, data.rightArmTorqueControlledJointReferences);
            }

            if (!leftLeg.isNull() && leftLeg.isList()) {
                Bottle *list = leftLeg.asList();
                for (int i = 0; i < std::min((size_t)list->size(), data.leftLegReferences.size()); i++) {
                    data.leftLegReferences(i) = limitInput(list->get(i).asDouble(),
                                                           data.leftLegMinLimits(i), data.leftLegMaxLimits(i));
                }
            }

            if (!rightLeg.isNull() && rightLeg.isList()) {
                Bottle *list = rightLeg.asList();
                for (int i = 0; i < std::min((size_t)list->size(), data.rightLegReferences.size()); i++) {
                    data.rightLegReferences(i) = limitInput(list->get(i).asDouble(),
                                                           data.rightLegMinLimits(i), data.rightLegMaxLimits(i));
                }
            }

            if( !com.isNull() && com.isList() && com.asList()->size() == COM_SIZE ) {
                Bottle *list = com.asList();

                for (int i = 0; i < COM_SIZE; i++) {
                    data.comReferences(i) = list->get(i).asDouble();
                }

                // If this is the first com that we receive, reset the trajectory generator
                if( !data.comTrajGenActive ) {
                    data.comGenerator->init(data.comReferences);
                    data.comTrajGenActive = true;
                }

            }
        }

        void CoordinatorData::mapInput(std::vector<int> &map, std::list<int> &complementMap,
                                       yarp::sig::Vector& input, yarp::sig::Vector &mapped, yarp::sig::Vector &complementMapped)
        {
            //Now map this to the pos/torque
            for (int i = 0; i < map.size(); i++) {
                int jointID = map[i];
                mapped[i] = input[jointID];
            }
            int i = 0;
            for (std::list<int>::const_iterator it = complementMap.begin();
                 it != complementMap.end(); it++, i++) {
                complementMapped[i] = input[*it];
            }
        }

        Coordinator::Coordinator()
        : m_threadPeriod(0.01)
        , m_inputJointReferences(0)
        , m_outputTorqueControlledJointReferences(0)
        , m_outputComDesiredPosVelAcc(0)
        , m_rpcServer(0)
        , implementation(0) {}

        Coordinator::~Coordinator() { cleanup(); }

        double Coordinator::getPeriod() { return m_threadPeriod; }

        bool Coordinator::configure(yarp::os::ResourceFinder &rf)
        {
            using namespace yarp::os;
            using yarp::sig::Vector;
            using namespace yarp::math;

            CoordinatorData *data = new CoordinatorData();
            if (!data) {
                cleanup();
                return false;
            }
            implementation = data;

            setName(rf.check("moduleName", Value("/trajGenY2")).asString().c_str());
            m_threadPeriod = rf.check("modulePeriod", Value(0.01)).asDouble();
            m_robotName = rf.check("robot", Value("icub")).asString();

            double trajectoryTimeStep = rf.check("trajTs", Value(m_threadPeriod)).asDouble();
            double trajectoryTimeDuration = rf.check("trajTimeDuration", Value(3.0)).asDouble();

            m_inputJointReferences = new BufferedPort<Property>();
            if (!m_inputJointReferences
                || !m_inputJointReferences->open(getName("/refs:i"))) {
                cleanup();
                return false;
            }
            m_inputJointReferences->useCallback(data->reader);

            m_outputTorqueControlledJointReferences = new BufferedPort<Vector>();
            if (!m_outputTorqueControlledJointReferences
                || !m_outputTorqueControlledJointReferences->open(getName("/qdes:o"))) {
                cleanup();
                return false;
            }

            m_outputComDesiredPosVelAcc = new BufferedPort<Vector>();
            if (!m_outputComDesiredPosVelAcc
                || !m_outputComDesiredPosVelAcc->open(getName("/comdes:o"))) {
                cleanup();
                return false;
            }
            // By default, the com trajectory generation is disable until we get a ref com
            data->comTrajGenActive = false;


            m_rpcServer = new RpcServer();
            if (!m_rpcServer
                || !m_rpcServer->open(getName("/rpc:i"))) {
                cleanup();
                return false;
            }
            attach(*m_rpcServer);


            bool result = true;
            Property leftArmOptions("(device remote_controlboard)");
            leftArmOptions.put("remote", "/" + m_robotName + "/left_arm");
            leftArmOptions.put("local", getName("/left_arm"));
            result = result && data->leftArmDriver.open(leftArmOptions);

            Property rightArmOptions("(device remote_controlboard)");
            rightArmOptions.put("remote", "/" + m_robotName + "/right_arm");
            rightArmOptions.put("local", getName("/right_arm"));
            result = result && data->rightArmDriver.open(rightArmOptions);

            Property torsoOptions("(device remote_controlboard)");
            torsoOptions.put("remote", "/" + m_robotName + "/torso");
            torsoOptions.put("local", getName("/torso"));
            result = result && data->torsoDriver.open(torsoOptions);

            Property leftLegOptions("(device remote_controlboard)");
            leftLegOptions.put("remote", "/" + m_robotName + "/left_leg");
            leftLegOptions.put("local", getName("/left_leg"));
            result = result && data->leftLegDriver.open(leftLegOptions);

            Property rightLegOptions("(device remote_controlboard)");
            rightLegOptions.put("remote", "/" + m_robotName + "/right_leg");
            rightLegOptions.put("local", getName("/right_leg"));
            result = result && data->rightLegDriver.open(rightLegOptions);

            if (!result) {
                cleanup();
                yError("Could not open control boards");
                return false;
            }

            bool encoderRead = true;
            int readCount = 10;
            //Load initial configuration
            data->torsoDriver.view(data->torsoEncoders);
            data->torsoDriver.view(data->torsoPositionControl);
            int torsoAxes = 0;
            data->torsoPositionControl->getAxes(&torsoAxes);
            data->torsoCurrentPosition.resize(torsoAxes, 0.0);
            do {
                encoderRead = data->torsoEncoders->getEncoders(data->torsoCurrentPosition.data());
                readCount--;
                yarp::os::Time::delay(0.01);
            } while(!encoderRead && readCount > 0);
            result = result && encoderRead;
            data->torsoCurrentPosition *= CTRL_DEG2RAD;

            data->leftArmDriver.view(data->leftArmEncoders);
            data->leftArmDriver.view(data->leftArmPositionControl);
            int leftArmAxes = 0;
            data->leftArmPositionControl->getAxes(&leftArmAxes);
            data->leftArmCurrentPosition.resize(leftArmAxes, 0.0);

            encoderRead = true;
            readCount = 10;
            do {
                encoderRead = data->leftArmEncoders->getEncoders(data->leftArmCurrentPosition.data());
                readCount--;
                yarp::os::Time::delay(0.01);
            } while(!encoderRead && readCount > 0);
            result = result && encoderRead;

            data->leftArmCurrentPosition *= CTRL_DEG2RAD;

            data->rightArmDriver.view(data->rightArmEncoders);
            data->rightArmDriver.view(data->rightArmPositionControl);
            int rightArmAxes = 0;
            data->rightArmPositionControl->getAxes(&rightArmAxes);
            data->rightArmCurrentPosition.resize(rightArmAxes, 0.0);
            encoderRead = true;
            readCount = 10;
            do {
                encoderRead = data->rightArmEncoders->getEncoders(data->rightArmCurrentPosition.data());
                readCount--;
                yarp::os::Time::delay(0.01);
            } while(!encoderRead && readCount > 0);
            result = result && encoderRead;
            data->rightArmCurrentPosition *= CTRL_DEG2RAD;

            yarp::dev::IPositionControl *positionControl = NULL;
            data->leftLegDriver.view(positionControl);
            data->leftLegDriver.view(data->leftLegEncoders);
            int leftLegAxes = 0;
            positionControl->getAxes(&leftLegAxes);
            data->leftLegCurrentPosition.resize(leftLegAxes, 0.0);
            data->leftLegReferences.resize(leftLegAxes, 0.0);
            encoderRead = true;
            readCount = 10;
            do {
                encoderRead = data->leftLegEncoders->getEncoders(data->leftLegCurrentPosition.data());
                readCount--;
                yarp::os::Time::delay(0.01);
            } while(!encoderRead && readCount > 0);
            result = result && encoderRead;
            data->leftLegCurrentPosition *= CTRL_DEG2RAD;

            int rightLegAxes = 0;
            data->rightLegDriver.view(positionControl);
            data->rightLegDriver.view(data->rightLegEncoders);
            positionControl->getAxes(&rightLegAxes);
            data->rightLegCurrentPosition.resize(rightLegAxes, 0.0);
            data->rightLegReferences.resize(rightLegAxes, 0.0);
            encoderRead = true;
            readCount = 10;
            do {
                encoderRead = data->rightLegEncoders->getEncoders(data->rightLegCurrentPosition.data());
                readCount--;
                yarp::os::Time::delay(0.01);
            } while(!encoderRead && readCount > 0);
            result = result && encoderRead;
            data->rightLegCurrentPosition *= CTRL_DEG2RAD;

            if (!result) {
                yError("Could not read initial encoders");
                return false;
            }

            //Resize com reference vector
            data->comReferences.resize(COM_SIZE, 0.0);

            //Resize com desired trajectory helper buffers
            data->comDesPos.resize(COM_SIZE, 0.0);
            data->comDesVel.resize(COM_SIZE, 0.0);
            data->comDesAcc.resize(COM_SIZE, 0.0);

            //Load limits
            data->torsoMinLimits.resize(torsoAxes, 0.0);
            data->torsoMaxLimits.resize(torsoAxes, 0.0);
            data->leftArmMinLimits.resize((leftArmAxes > 7 ? 7 : leftArmAxes), 0.0);
            data->leftArmMaxLimits.resize((leftArmAxes > 7 ? 7 : leftArmAxes), 0.0);
            data->rightArmMinLimits.resize((rightArmAxes > 7 ? 7 : rightArmAxes), 0.0);
            data->rightArmMaxLimits.resize((rightArmAxes > 7 ? 7 : rightArmAxes), 0.0);
            data->leftLegMinLimits.resize(leftLegAxes, 0.0);
            data->leftLegMaxLimits.resize(leftLegAxes, 0.0);
            data->rightLegMinLimits.resize(rightLegAxes, 0.0);
            data->rightLegMaxLimits.resize(rightLegAxes, 0.0);

            data->torsoDriver.view(data->torsoLimitsInterface);
            data->leftArmDriver.view(data->leftArmLimitsInterface);
            data->rightArmDriver.view(data->rightArmLimitsInterface);
            data->leftLegDriver.view(data->leftLegLimitsInterface);
            data->rightLegDriver.view(data->rightLegLimitsInterface);

            for (int i = 0; i < torsoAxes; i++) {
                data->torsoLimitsInterface->getLimits(i, &data->torsoMinLimits(i), &data->torsoMaxLimits(i));
                data->torsoMinLimits(i) *= CTRL_DEG2RAD;
                data->torsoMaxLimits(i) *= CTRL_DEG2RAD;
            }
            for (int i = 0; i < (leftArmAxes > 7 ? 7 : leftArmAxes); i++) {
                data->leftArmLimitsInterface->getLimits(i, &data->leftArmMinLimits(i), &data->leftArmMaxLimits(i));
                data->leftArmMinLimits(i) *= CTRL_DEG2RAD;
                data->leftArmMaxLimits(i) *= CTRL_DEG2RAD;
            }
            for (int i = 0; i < (rightArmAxes > 7 ? 7 : rightArmAxes); i++) {
                data->rightArmLimitsInterface->getLimits(i, &data->rightArmMinLimits(i), &data->rightArmMaxLimits(i));
                data->rightArmMinLimits(i) *= CTRL_DEG2RAD;
                data->rightArmMaxLimits(i) *= CTRL_DEG2RAD;
            }
            for (int i = 0; i < leftLegAxes; i++) {
                data->leftLegLimitsInterface->getLimits(i, &data->leftLegMinLimits(i), &data->leftLegMaxLimits(i));
                data->leftLegMinLimits(i) *= CTRL_DEG2RAD;
                data->leftLegMaxLimits(i) *= CTRL_DEG2RAD;
            }
            for (int i = 0; i < rightLegAxes; i++) {
                data->rightLegLimitsInterface->getLimits(i, &data->rightLegMinLimits(i), &data->rightLegMaxLimits(i));
                data->rightLegMinLimits(i) *= CTRL_DEG2RAD;
                data->rightLegMaxLimits(i) *= CTRL_DEG2RAD;
            }
            yInfo("Read joint limits");
            yInfo("Torso[min,max]=%s,%s", data->torsoMinLimits.toString().c_str(), data->torsoMaxLimits.toString().c_str());
            yInfo("Left[min,max]=%s,%s", data->leftArmMinLimits.toString().c_str(), data->leftArmMaxLimits.toString().c_str());
            yInfo("Right[min,max]=%s,%s", data->rightArmMinLimits.toString().c_str(), data->rightArmMaxLimits.toString().c_str());

            yInfo("Read joint limits");
            yInfo("Torso[min,max]=%s,%s", data->torsoMinLimits.toString().c_str(), data->torsoMaxLimits.toString().c_str());
            yInfo("Left[min,max]=%s,%s", data->leftArmMinLimits.toString().c_str(), data->leftArmMaxLimits.toString().c_str());
            yInfo("Right[min,max]=%s,%s", data->rightArmMinLimits.toString().c_str(), data->rightArmMaxLimits.toString().c_str());

            //setup initial references to coincide with reads from encoders
            data->torsoJointReferences.resize(3, 0.0);
            for (int i = 0; i < 3; i++) {
                data->torsoJointReferences(i) = data->torsoCurrentPosition(i);
            }
            data->leftArmJointReferences.resize(7, 0.0);
            for (int i = 0; i < 7; i++) {
                data->leftArmJointReferences(i) = data->leftArmCurrentPosition(i);
            }
            data->rightArmJointReferences.resize(7, 0.0);
            for (int i = 0; i < 7; i++) {
                data->rightArmJointReferences(i) = data->rightArmCurrentPosition(i);
            }
            data->leftLegReferences = data->leftLegCurrentPosition;
            data->rightLegReferences = data->rightLegCurrentPosition;

            std::vector<std::string> torsoJoints;
            torsoJoints.push_back("torso_yaw");
            torsoJoints.push_back("torso_roll");
            torsoJoints.push_back("torso_pitch");

            std::vector<std::string> leftArmJoints;
            leftArmJoints.push_back("l_shoulder_pitch");
            leftArmJoints.push_back("l_shoulder_roll");
            leftArmJoints.push_back("l_shoulder_yaw");
            leftArmJoints.push_back("l_elbow");
            leftArmJoints.push_back("l_wrist_prosup");
            leftArmJoints.push_back("l_wrist_pitch");
            leftArmJoints.push_back("l_wrist_yaw");

            std::vector<std::string> rightArmJoints;
            rightArmJoints.push_back("r_shoulder_pitch");
            rightArmJoints.push_back("r_shoulder_roll");
            rightArmJoints.push_back("r_shoulder_yaw");
            rightArmJoints.push_back("r_elbow");
            rightArmJoints.push_back("r_wrist_prosup");
            rightArmJoints.push_back("r_wrist_pitch");
            rightArmJoints.push_back("r_wrist_yaw");

            //Loading joints information
            yarp::os::Property wbiProperties;
            if (!rf.check("wbi_file")) {
                yError("No WBI configuration file found.");
                cleanup();
                return false;
            }

            if (!rf.check("wbi_joint_list")) {
                yError("No joint list found. Please specify a joint list in \"wbi_joint_list\"");
                cleanup();
                return false;
            }

            if (!wbiProperties.fromConfigFile(rf.findFile("wbi_file"))) {
                yError("Not possible to load WBI properties from file.");
                cleanup();
                return false;
            }
            wbiProperties.fromString(rf.toString(), false);

            yarp::os::ConstString jointList = rf.find("wbi_joint_list").asString();
            //retrieve all main joints
            wbi::IDList iCubMainJoints;
            if (!yarpWbi::loadIdListFromConfig(jointList, wbiProperties, iCubMainJoints)) {
                yError("Cannot find joint list");
                cleanup();
                return false;
            }

            data->torqueControlOutputReferences.resize(iCubMainJoints.size(), 0.0);

            data->torsoJointIDs.clear(); data->torsoJointIDs.reserve(3);
            data->armJointIDs.clear(); data->armJointIDs.reserve(7);

            for (int i = 0; i < torsoJoints.size(); i++) {
                if (!iCubMainJoints.containsID(torsoJoints[i])) {
                    data->torsoJointIDs.push_back(i);
                }
            }

            for (int i = 0; i < leftArmJoints.size(); i++) {
                bool onLeft = iCubMainJoints.containsID(leftArmJoints[i]);
                bool onRight = iCubMainJoints.containsID(rightArmJoints[i]);

                if (onLeft != onRight) {
                    yError("For now only symmetric configuration is supported!");
                    cleanup();
                    return false;
                }
                if (!onLeft) {
                    data->armJointIDs.push_back(i);
                }
            }

            data->torsoPositionControlledJointReferences.resize(data->torsoJointIDs.size(), 0.0);
            data->torsoTorqueControlledJointReferences.resize(3 - data->torsoJointIDs.size(), 0.0);
            data->leftArmPositionControlledJointReferences.resize(data->armJointIDs.size(), 0.0);
            data->leftArmTorqueControlledJointReferences.resize(7 - data->armJointIDs.size(), 0.0);
            data->rightArmPositionControlledJointReferences.resize(data->armJointIDs.size(), 0.0);
            data->rightArmTorqueControlledJointReferences.resize(7 - data->armJointIDs.size(), 0.0);

            //Init mapping information
            data->init();

            //Setup generators
            data->torsoGenerator = new iCub::ctrl::minJerkTrajGen(data->torsoPositionControlledJointReferences.size(), trajectoryTimeStep, trajectoryTimeDuration);
            data->leftArmGenerator = new iCub::ctrl::minJerkTrajGen(data->leftArmPositionControlledJointReferences.size(), trajectoryTimeStep, trajectoryTimeDuration);
            data->rightArmGenerator = new iCub::ctrl::minJerkTrajGen(data->rightArmPositionControlledJointReferences.size(), trajectoryTimeStep, trajectoryTimeDuration);
            data->torqueBalancingGenerator = new iCub::ctrl::minJerkTrajGen(iCubMainJoints.size(), trajectoryTimeStep, trajectoryTimeDuration);
            data->comGenerator = new iCub::ctrl::minJerkTrajGen(3, trajectoryTimeStep,  trajectoryTimeDuration);

            if (!data->torsoGenerator ||
                !data->leftArmGenerator ||
                !data->rightArmGenerator ||
                !data->torqueBalancingGenerator ||
                !data->comGenerator) {
                yError("Could not create trajectory generator");
                cleanup();
                return false;
            }

            //Map initial values
            data->mapInput(data->torsoJointIDs, data->torsoMappingInformationComplement,
                           data->torsoJointReferences, data->torsoPositionControlledJointReferences, data->torsoTorqueControlledJointReferences);
            data->mapInput(data->armJointIDs, data->armMappingInformationComplement,
                           data->leftArmJointReferences, data->leftArmPositionControlledJointReferences, data->leftArmTorqueControlledJointReferences);
            data->mapInput(data->armJointIDs, data->armMappingInformationComplement,
                           data->rightArmJointReferences, data->rightArmPositionControlledJointReferences, data->rightArmTorqueControlledJointReferences);

            data->copyReferencesForTorqueOutput();

            data->torsoGenerator->init(data->torsoPositionControlledJointReferences);
            data->leftArmGenerator->init(data->leftArmPositionControlledJointReferences);
            data->rightArmGenerator->init(data->rightArmPositionControlledJointReferences);
            data->torqueBalancingGenerator->init(data->torqueControlOutputReferences);
            data->comGenerator->init(data->comReferences);

            yInfo("Coordinator ready");
            return true;
        }

        bool Coordinator::updateModule()
        {
            using namespace yarp::math;

            CoordinatorData *data = static_cast<CoordinatorData*>(implementation);
            if (!data) return false;
            if (!data->torsoGenerator || !data->leftArmGenerator ||
                !data->rightArmGenerator || !data->torqueBalancingGenerator ||
                !data->comGenerator ) return false;

            yarp::os::LockGuard guard(data->mutex);
            //            if (!data->referencesChanged) return true;
            //            data->referencesChanged = false;

            data->copyReferencesForTorqueOutput();

            data->torsoGenerator->computeNextValues(data->torsoPositionControlledJointReferences);
            data->leftArmGenerator->computeNextValues(data->leftArmPositionControlledJointReferences);
            data->rightArmGenerator->computeNextValues(data->rightArmPositionControlledJointReferences);
            data->torqueBalancingGenerator->computeNextValues(data->torqueControlOutputReferences);

            //send to robot
            data->torsoPositionControl->setPositions(data->torsoJointIDs.size(), data->torsoJointIDs.data(), (data->torsoGenerator->getPos() * CTRL_RAD2DEG).data());

            data->leftArmPositionControl->setPositions(data->armJointIDs.size(), data->armJointIDs.data(), (data->leftArmGenerator->getPos() * CTRL_RAD2DEG).data());

            data->rightArmPositionControl->setPositions(data->armJointIDs.size(), data->armJointIDs.data(), (data->rightArmGenerator->getPos() * CTRL_RAD2DEG).data());

            //send to torqueBalancing

            //send position impedance
            yarp::sig::Vector& torqueOutput = m_outputTorqueControlledJointReferences->prepare();
//            torqueOutput.resize(data->torsoTorqueControlledJointReferences.size(), 0.0);
            torqueOutput = data->torqueBalancingGenerator->getPos();

            m_outputTorqueControlledJointReferences->write();

            //send com desired position, velocity, acceleration
            if( data->comTrajGenActive ) {
                yarp::sig::Vector& comDesPosVelAcc = m_outputComDesiredPosVelAcc->prepare();

                comDesPosVelAcc.resize(3*COM_SIZE, 0.0);

                data->comDesPos = data->comGenerator->getPos();
                data->comDesVel = data->comGenerator->getVel();
                data->comDesAcc = data->comGenerator->getAcc();
                comDesPosVelAcc.setSubvector(0,data->comDesPos);
                comDesPosVelAcc.setSubvector(1*COM_SIZE,data->comDesVel);
                comDesPosVelAcc.setSubvector(2*COM_SIZE,data->comDesAcc);

                m_outputComDesiredPosVelAcc->write();
            }


            return true;
        }

        bool Coordinator::close()
        {
            yInfo("Closing coordinator");
            return cleanup();
        }

        bool Coordinator::cleanup()
        {
            if (m_inputJointReferences) {
                m_inputJointReferences->interrupt();
                m_inputJointReferences->close();
                delete m_inputJointReferences;
                m_inputJointReferences = 0;
            }
            if (m_outputTorqueControlledJointReferences) {
                m_outputTorqueControlledJointReferences->interrupt();
                m_outputTorqueControlledJointReferences->close();
                delete m_outputTorqueControlledJointReferences;
                m_outputTorqueControlledJointReferences = 0;
            }
           if (m_outputComDesiredPosVelAcc) {
                m_outputComDesiredPosVelAcc->interrupt();
                m_outputComDesiredPosVelAcc->close();
                delete m_outputComDesiredPosVelAcc;
                m_outputComDesiredPosVelAcc = 0;
            }

            if (m_rpcServer) {
                m_rpcServer->interrupt();
                m_rpcServer->close();
                delete m_rpcServer;
                m_rpcServer = 0;
            }
            CoordinatorData *data = static_cast<CoordinatorData*>(implementation);

            if (data) {
                if (data->torsoGenerator) {
                    delete data->torsoGenerator;
                    data->torsoGenerator = 0;
                }
                if (data->leftArmGenerator) {
                    delete data->leftArmGenerator;
                    data->leftArmGenerator = 0;
                }
                if (data->rightArmGenerator) {
                    delete data->rightArmGenerator;
                    data->rightArmGenerator = 0;
                }
                if (data->torqueBalancingGenerator) {
                    delete data->torqueBalancingGenerator;
                    data->torqueBalancingGenerator = 0;
                }
                if (data->comGenerator) {
                    delete data->comGenerator;
                    data->comGenerator = 0;
                }

                data->leftArmDriver.close();
                data->rightArmDriver.close();
                data->torsoDriver.close();
                data->leftLegDriver.close();
                data->rightLegDriver.close();
                delete data;
                implementation = 0;
            }

            return true;
        }

        bool Coordinator::respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply)
        {
            CoordinatorData *data = static_cast<CoordinatorData*>(implementation);
            if (!data) return false;
            static std::string doneCommand = "IS_DONE";
            using namespace yarp::os;
            using namespace yarp::math;

            reply.clear();

            std::string cmd = "";

            if (command.size() > 0) {
                Value &cmdVal = command.get(0);
                if (!cmdVal.isNull() && cmdVal.isString())
                    cmd = cmdVal.asString();
            }
            if (cmd.length() == 0 || cmd.compare(doneCommand) != 0) {
                reply.addString("Failed");
                return true;
            }

            std::string partName = "";
            if (command.size() > 1) {
                Value &cmdVal = command.get(1);
                if (!cmdVal.isNull() && cmdVal.isString())
                    partName = cmdVal.asString();
            }

            LockGuard guard(data->mutex);

            data->torsoEncoders->getEncoders(data->torsoCurrentPosition.data());
            data->torsoCurrentPosition *= CTRL_DEG2RAD;
            data->leftArmEncoders->getEncoders(data->leftArmCurrentPosition.data());
            data->leftArmCurrentPosition *= CTRL_DEG2RAD;
            data->rightArmEncoders->getEncoders(data->rightArmCurrentPosition.data());
            data->rightArmCurrentPosition *= CTRL_DEG2RAD;

            bool result = true;
            if (partName.length() == 0 || partName.compare("torso") == 0) {
                result = result && checkMotionDone(data->torsoJointReferences, data->torsoCurrentPosition);
            }

            if (partName.length() == 0 || partName.compare("left_arm") == 0) {
                result = result && checkMotionDone(data->leftArmJointReferences, data->leftArmCurrentPosition);
            }

            if (partName.length() == 0 || partName.compare("right_arm") == 0) {
                result = result && checkMotionDone(data->rightArmJointReferences, data->rightArmCurrentPosition);
            }
            reply.addInt(result ? 1 : 0);


            return true;
        }

        bool Coordinator::checkMotionDone(const yarp::sig::Vector &reference, const yarp::sig::Vector &actual) {
            using namespace yarp::math;
            double squaredNorm = 0;
            for (int i = 0; i < std::min(reference.size(), actual.size()); i++) {
                squaredNorm += (reference(i) - actual(i)) * (reference(i) - actual(i));
            }
            return squaredNorm < (4.0 * 4.0 * CTRL_DEG2RAD);
        }

    }
}
