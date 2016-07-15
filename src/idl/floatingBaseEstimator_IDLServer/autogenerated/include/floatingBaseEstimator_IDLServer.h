// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_floatingBaseEstimator_IDLServer
#define YARP_THRIFT_GENERATOR_floatingBaseEstimator_IDLServer

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class floatingBaseEstimator_IDLServer;


/**
 * floatingBaseEstimator_IDLServer
 * Interface.
 */
class floatingBaseEstimator_IDLServer : public yarp::os::Wire {
public:
  floatingBaseEstimator_IDLServer();
  /**
   * Reset the odometry world to be (initially) a frame specified in the robot model,
   * and specify a frame that is assumed to be fixed in the odometry.
   * @param initial_world_frame the frame of the robot model that is assume to be initially
   *        coincident with the world/inertial frame.
   * @param new_fixed_frame the name of a frame attached to the link that should be considered fixed
   * @return true/false on success/failure (typically if the frame/link names are wrong)
   */
  virtual bool resetSimpleLeggedOdometry(const std::string& initial_world_frame, const std::string& initial_fixed_frame);
  /**
   * Change the link that is considered fixed by the odometry.
   * @param new_fixed_frame the name of a frame attached to the link that should be considered fixed
   * @return true/false on success/failure (typically if the frame names are wrong)
   */
  virtual bool changeFixedLinkSimpleLeggedOdometry(const std::string& new_fixed_frame);
  /**
   * Get the current settings in the form of a string.
   * @return the current settings as a human readable string.
   */
  virtual std::string getCurrentSettingsString();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
