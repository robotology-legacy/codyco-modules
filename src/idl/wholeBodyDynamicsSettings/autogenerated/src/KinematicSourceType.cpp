// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <KinematicSourceType.h>



int KinematicSourceTypeVocab::fromString(const std::string& input) {
  // definitely needs optimizing :-)
  if (input=="IMU") return (int)IMU;
  if (input=="FIXED_FRAME") return (int)FIXED_FRAME;
  return -1;
}
std::string KinematicSourceTypeVocab::toString(int input) {
  switch((KinematicSourceType)input) {
  case IMU:
    return "IMU";
  case FIXED_FRAME:
    return "FIXED_FRAME";
  }
  return "";
}


