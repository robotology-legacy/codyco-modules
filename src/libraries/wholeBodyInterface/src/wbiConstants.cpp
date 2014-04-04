#include <wbi/wbiConstants.h>

namespace wbi {
    
    const SensorTypeDescription sensorTypeDescriptions[SENSOR_TYPE_SIZE] =
    {
        SensorTypeDescription(SENSOR_ENCODER,       "encoder",          1, true,  "Joint position"),
        SensorTypeDescription(SENSOR_PWM,           "PWM",              1, true,  "Motor PWM"),
        SensorTypeDescription(SENSOR_CURRENT,       "current",          1, true,  "Motor current"),
        SensorTypeDescription(SENSOR_TORQUE,        "torque",           1, true,  "Joint torque"),
        SensorTypeDescription(SENSOR_IMU,           "IMU",              13, false, "Inertial Measurement Unit"),
        SensorTypeDescription(SENSOR_FORCE_TORQUE,  "force-torque",     6, false, "6-axis force torque"),
        SensorTypeDescription(SENSOR_ACCELEROMETER, "accelerometer",    3, false, "3d linear acceleration"),
    };
    
    
    SensorTypeDescription::SensorTypeDescription(SensorType _id, std::string _name, int _dataSize, bool _isJoint, std::string _descr)
    : id(_id)
    , name(_name)
    , description(_descr)
    , dataSize(_dataSize)
    , isJointSensor(_isJoint)
    {}
    
    bool SensorTypeDescription::operator ==(const SensorTypeDescription &st)
    {
        return st.id == this->id;
    }
    
}