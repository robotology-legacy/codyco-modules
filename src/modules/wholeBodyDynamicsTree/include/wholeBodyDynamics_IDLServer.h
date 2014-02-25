// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_wholeBodyDynamics_IDLServer
#define YARP_THRIFT_GENERATOR_wholeBodyDynamics_IDLServer

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class wholeBodyDynamics_IDLServer;


/**
 * wholeBodyDynamics_IDLServer
 * Interface.
 */
class wholeBodyDynamics_IDLServer : public yarp::os::Wire {
public:
  wholeBodyDynamics_IDLServer() { yarp().setOwner(*this); }
/**
 * Calibrate the force/torque sensors
 * (WARNING: calibrate the sensors when the only external forces acting on the robot are on the torso/waist)
 * @param calib_code argument to specify the sensors to calibrate (all,arms,legs,feets)
 * @return true/false on success/failure
 */
  virtual bool calib(const std::string& calib_code);
/**
 * Quit the module.
 * @return true/false on success/failure
 */
  virtual bool quit();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif

