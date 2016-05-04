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
  wholeBodyDynamics_IDLServer();
  /**
   * Calibrate the force/torque sensors
   * (WARNING: calibrate the sensors when the only external forces acting on the robot are on the torso/waist)
   * @param calib_code argument to specify the sensors to calibrate (all,arms,legs,feet)
   * @param nr_of_samples number of samples
   * @return true/false on success/failure
   */
  virtual bool calib(const std::string& calib_code, const int32_t nr_of_samples = 100);
  /**
   * Calibrate the force/torque sensors when on double support
   * (WARNING: calibrate the sensors when the only external forces acting on the robot are on the sole).
   * For this calibration the strong assumption of simmetry of the robot and its pose is done.
   * @param calib_code argument to specify the sensors to calibrate (all,arms,legs,feet)
   * @param nr_of_samples number of samples
   * @return true/false on success/failure
   */
  virtual bool calibStanding(const std::string& calib_code, const int32_t nr_of_samples = 100);
  /**
   * Calibrate the force/torque sensors when on single support on left foot
   * (WARNING: calibrate the sensors when the only external forces acting on the robot are on the left sole).
   * @param calib_code argument to specify the sensors to calibrate (all,arms,legs,feet)
   * @param nr_of_samples number of samples
   * @return true/false on success/failure
   */
  virtual bool calibStandingLeftFoot(const std::string& calib_code, const int32_t nr_of_samples = 100);
  /**
   * Calibrate the force/torque sensors when on single support on right foot
   * (WARNING: calibrate the sensors when the only external forces acting on the robot are on the right sole).
   * @param calib_code argument to specify the sensors to calibrate (all,arms,legs,feet)
   * @param nr_of_samples number of samples
   * @return true/false on success/failure
   */
  virtual bool calibStandingRightFoot(const std::string& calib_code, const int32_t nr_of_samples = 100);
  /**
   * Reset the sensor offset to 0 0 0 0 0 0 (six zeros).
   * @param calib_code argument to specify the sensors to reset (all,arms,legs,feet)
   * @return true/false on success/failure
   */
  virtual bool resetOffset(const std::string& calib_code);
  /**
   * Quit the module.
   * @return true/false on success/failure
   */
  virtual bool quit();
  /**
   * Reset the odometry world to be (initially) a frame specified in the robot model,
   * and specify a link that is assumed to be fixed in the odometry.
   * @param initial_world_frame the frame of the robot model that is assume to be initially
   *        coincident with the world/inertial frame.
   * @param new_fixed_link the name of the link that should be initially fixed
   * @return true/false on success/failure (typically if the frame/link names are wrong)
   */
  virtual bool resetSimpleLeggedOdometry(const std::string& initial_world_frame, const std::string& initial_fixed_link);
  /**
   * Change the link that is considered fixed by the odometry.
   * @param new_fixed_link the name of the new link that should be considered fixed
   * @return true/false on success/failure (typically if the frame/link names are wrong)
   */
  virtual bool changeFixedLinkSimpleLeggedOdometry(const std::string& new_fixed_link);
  /**
   * Set the cutoff frequency (in Hz) for IMU measurements
   * @return true/false on success/failure
   */
  virtual bool set_imuFilterCutoffInHz(const double newCutoff);
  /**
   * Get the cutoff frequency (in Hz) for IMU measurements
   * @return the cutoff frequency (in Hz)
   */
  virtual double get_imuFilterCutoffInHz();
  /**
   * Set the cutoff frequency (in Hz) for FT measurements
   * @return true/false on success/failure
   */
  virtual bool set_forceTorqueFilterCutoffInHz(const double newCutoff);
  /**
   * Get the cutoff frequency (in Hz) for FT measurements
   * @return the cutoff frequency (in Hz)
   */
  virtual double get_forceTorqueFilterCutoffInHz();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
