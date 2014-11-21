// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_insituFTSensorCalibration_IDLServer
#define YARP_THRIFT_GENERATOR_insituFTSensorCalibration_IDLServer

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class insituFTSensorCalibration_IDLServer;


/**
 * insituFTSensorCalibration_IDLServer
 * Interface.
 */
class insituFTSensorCalibration_IDLServer : public yarp::os::Wire {
public:
  insituFTSensorCalibration_IDLServer() { yarp().setOwner(*this); }
  /**
   * Start the new dataset acquisition.
   * At the end of each dataset acquisition,
   * the module stops until the user changes
   * the added mass on the robot. This command
   * is used to start the dataset acquisition
   * after the added mass is correctly mounted
   * in the robot.
   * @return true/false on success/failure
   */
  virtual bool startNewDatasetAcquisition();
  /**
   * Quit the module.
   * @return true/false on success/failure
   */
  virtual bool quit();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif

