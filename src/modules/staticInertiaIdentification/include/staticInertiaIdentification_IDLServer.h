// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_staticInertiaIdentification_IDLServer
#define YARP_THRIFT_GENERATOR_staticInertiaIdentification_IDLServer

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class staticInertiaIdentification_IDLServer;


/**
 * staticInertiaIdentification_IDLServer
 * Interface.
 */
class staticInertiaIdentification_IDLServer : public yarp::os::Wire {
public:
  staticInertiaIdentification_IDLServer() { yarp().setOwner(*this); }
/**
 * Start the estimation.
 * @return true/false on success/failure
 */
  virtual bool start();
/**
 * stop the estimation.
 * @return true/false on success/failure
 */
  virtual bool stop();
/**
 * Save the estimated inertial parameters in a URDF file.
 * @param fileName the name of the save file.
 * @param robotName the name of the robot model in the URDF file.
 * @return true/false on success/failure
 */
  virtual bool saveURDF(const std::string& fileName, const std::string& robotName = "test_icub");
/**
 * Quit the module.
 * @return true/false on success/failure
 */
  virtual bool quit();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif

