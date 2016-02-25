#ifndef CONSTANTS_H_
#define CONSTANTS_H_

// TEMPORARY
/**
 *  ID number of the first board in the right foot with accelerometer plus gyro as specified in:
 *
 *  @return 32.0
 */
#define MTB_RIGHT_FOOT_ACC_PLUS_GYRO_1_ID 32.0
/**
 *  ID number of the second board in the right foot with accelerometer plus gyro as specified in:
 *
 *  @return 33.0
 *  @note: This is the board to which the palm skin is currently connected.
 */
#define MTB_RIGHT_FOOT_ACC_PLUS_GYRO_2_ID 33.0 // The board to which the skin is connected
/**
 *  ID number of the MTB board in the palm of iCubGenova01 with accelerometer plys gyro as specified in:
 *
 *  @return 25.0
 */
#define MTB_RIGHT_HAND_ACC_PLUS_GYRO_1_ID 25.0
/**
 *  Position in the measurement vector as read from the port where a package data is expected.
 *  Recall that before the actual data, there are some other numbers and identifiers that could change.
 *
 *  @return 6
 */
#define MTB_PORT_DATA_PACKAGE_OFFSET 6
// Accelerometer conversion factor in m/s^2
/**
 *  MTB accelerometer conversion factor.
 *
 *  @return 5.9855e-04
 */
#define CONVERSION_FACTOR_ACC 5.9855e-04
// Gyroscope conversion factor in deg/sec
/**
 *  MTB gyroscope conversion factor.
 *
 *  @return 7.6274e-03
 */
#define CONVERSION_FACTOR_GYRO 7.6274e-03
#define PI 3.141592654


#endif /* constants.h */
