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
    3: Gravity gravity; /** If kinematicSource is FIXED_LINK, specify the gravity vector in the fixedFrame */
    4: double imuFilterCutoff; /** Cutoff frequency of the first order filter of the IMU */
    4: double forceTorqueFilterCutoff; /** Cutoff frequency of the first order filter of the F/T sensors */
    5: bool useJointVelocity; /** Use the joint velocity measurement if this is true, assume they are zero otherwise. */
    6: bool useJointAcceleration; /** Use the joint acceleration measurment if this is true, assume they are zero otherwise. */
}
