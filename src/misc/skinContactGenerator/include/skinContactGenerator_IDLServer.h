// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_skinContactGenerator_IDLServer
#define YARP_THRIFT_GENERATOR_skinContactGenerator_IDLServer

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class skinContactGenerator_IDLServer;


/**
 * skinContactGenerator_IDLServer
 * Interface.
 */
class skinContactGenerator_IDLServer : public yarp::os::Wire {
public:
  skinContactGenerator_IDLServer() { yarp().setOwner(*this); }
/**
 * Configure the skinContactGenerator to produce a contact
 * at the origin of a given link.
 */
  virtual bool setContact(const int32_t bodyPart, const int32_t link);
/**
 * Configure the skinContactGenerator to produce a contact
 * at a prescribed position of a given link.
 */
  virtual bool setContactForce(const int32_t bodyPart, const int32_t link, const double f_x, const double f_y, const double f_z, const double p_x, const double p_y, const double p_z, const int32_t skinPart = 0);
/**
 * Configure the skinContactGenerator to produce a contact
 * at the origin of a given link.
 */
  virtual bool setContactName(const std::string& bodyPart, const std::string& link);
/**
 * Quit the module.
 * @return true/false on success/failure
 */
  virtual bool quit();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif

