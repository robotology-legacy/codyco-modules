// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <EstimationAlgorithmType.h>



int EstimationAlgorithmTypeVocab::fromString(const std::string& input) {
  // definitely needs optimizing :-)
  if (input=="DETERMINISTIC_ALGORITHM") return (int)DETERMINISTIC_ALGORITHM;
  if (input=="PROBABILISTIC_ALGORITHM") return (int)PROBABILISTIC_ALGORITHM;
  return -1;
}
std::string EstimationAlgorithmTypeVocab::toString(int input) {
  switch((EstimationAlgorithmType)input) {
  case DETERMINISTIC_ALGORITHM:
    return "DETERMINISTIC_ALGORITHM";
  case PROBABILISTIC_ALGORITHM:
    return "PROBABILISTIC_ALGORITHM";
  }
  return "";
}


