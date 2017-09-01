// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_ENUM_EstimationAlgorithmType
#define YARP_THRIFT_GENERATOR_ENUM_EstimationAlgorithmType

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

enum EstimationAlgorithmType {
  DETERMINISTIC_ALGORITHM = 0,
  PROBABILISTIC_ALGORITHM = 1
};

class EstimationAlgorithmTypeVocab;

class EstimationAlgorithmTypeVocab : public yarp::os::idl::WireVocab {
public:
  virtual int fromString(const std::string& input) override;
  virtual std::string toString(int input) override;
};


#endif
