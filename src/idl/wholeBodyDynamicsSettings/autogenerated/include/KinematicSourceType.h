// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_ENUM_KinematicSourceType
#define YARP_THRIFT_GENERATOR_ENUM_KinematicSourceType

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

enum KinematicSourceType {
  IMU = 0,
  FIXED_FRAME = 1
};

class KinematicSourceTypeVocab;

class KinematicSourceTypeVocab : public yarp::os::idl::WireVocab {
public:
  virtual int fromString(const std::string& input);
  virtual std::string toString(int input);
};


#endif
