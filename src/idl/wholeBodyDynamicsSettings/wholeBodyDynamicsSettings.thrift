struct Gravity {
    1: double x; /* x gravity component [m/s^2] */
    2: double y; /* y gravity component [m/s^2] */
    3: double z; /* z gravity component [m/s^2] */
}

struct ContactPoint {
    1: double x; /* x component [m] */
    2: double y; /* y component [m] */
    3: double z; /* z component [m] */
}

enum KinematicSourceType {
      IMU,
      FIXED_FRAME
}

struct wholeBodyDynamicsSettings {
    1: KinematicSourceType kinematicSource; /** Specify the source of the kinematic information for one link, see KinematicSourceType information for more info. */
    2: string fixedFrameName; /** If kinematicSource is FIXED_LINK, specify the frame of the robot that we know to be fixed (i.e. not moving with respect to an inertial frame) */
    3: Gravity fixedFrameGravity; /** If kinematicSource is FIXED_LINK, specify the gravity vector (in m/s^2) in the fixedFrame */
    4: double imuFilterCutoffInHz; /** Cutoff frequency (in Hz) of the first order filter of the IMU */
    5: double forceTorqueFilterCutoffInHz; /** Cutoff frequency(in Hz) of the first order filter of the F/T sensors */
    6: double jointVelFilterCutoffInHz;    /** Cutoff frequency(in Hz) of the first order filter of the joint velocities */
    7: double jointAccFilterCutoffInHz;    /** Cutoff frequency(in Hz) of the first order filter of the joint accelerations */
    8: bool useJointVelocity; /** Use the joint velocity measurement if this is true, assume they are zero otherwise. */
    9: bool useJointAcceleration; /** Use the joint acceleration measurment if this is true, assume they are zero otherwise. */
}
