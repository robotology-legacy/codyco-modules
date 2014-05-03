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
 * Quit the module.
 * @return true/false on success/failure
 */
  virtual bool quit();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif

