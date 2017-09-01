// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <wholeBodyDynamicsSettings.h>

bool wholeBodyDynamicsSettings::read_kinematicSource(yarp::os::idl::WireReader& reader) {
  int32_t ecast0;
  KinematicSourceTypeVocab cvrt1;
  if (!reader.readEnum(ecast0,cvrt1)) {
    reader.fail();
    return false;
  } else {
    kinematicSource = (KinematicSourceType)ecast0;
  }
  return true;
}
bool wholeBodyDynamicsSettings::nested_read_kinematicSource(yarp::os::idl::WireReader& reader) {
  int32_t ecast2;
  KinematicSourceTypeVocab cvrt3;
  if (!reader.readEnum(ecast2,cvrt3)) {
    reader.fail();
    return false;
  } else {
    kinematicSource = (KinematicSourceType)ecast2;
  }
  return true;
}
bool wholeBodyDynamicsSettings::read_fixedFrameName(yarp::os::idl::WireReader& reader) {
  if (!reader.readString(fixedFrameName)) {
    reader.fail();
    return false;
  }
  return true;
}
bool wholeBodyDynamicsSettings::nested_read_fixedFrameName(yarp::os::idl::WireReader& reader) {
  if (!reader.readString(fixedFrameName)) {
    reader.fail();
    return false;
  }
  return true;
}
bool wholeBodyDynamicsSettings::read_fixedFrameGravity(yarp::os::idl::WireReader& reader) {
  if (!reader.read(fixedFrameGravity)) {
    reader.fail();
    return false;
  }
  return true;
}
bool wholeBodyDynamicsSettings::nested_read_fixedFrameGravity(yarp::os::idl::WireReader& reader) {
  if (!reader.readNested(fixedFrameGravity)) {
    reader.fail();
    return false;
  }
  return true;
}
bool wholeBodyDynamicsSettings::read_imuFrameName(yarp::os::idl::WireReader& reader) {
  if (!reader.readString(imuFrameName)) {
    reader.fail();
    return false;
  }
  return true;
}
bool wholeBodyDynamicsSettings::nested_read_imuFrameName(yarp::os::idl::WireReader& reader) {
  if (!reader.readString(imuFrameName)) {
    reader.fail();
    return false;
  }
  return true;
}
bool wholeBodyDynamicsSettings::read_imuFilterCutoffInHz(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(imuFilterCutoffInHz)) {
    reader.fail();
    return false;
  }
  return true;
}
bool wholeBodyDynamicsSettings::nested_read_imuFilterCutoffInHz(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(imuFilterCutoffInHz)) {
    reader.fail();
    return false;
  }
  return true;
}
bool wholeBodyDynamicsSettings::read_forceTorqueFilterCutoffInHz(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(forceTorqueFilterCutoffInHz)) {
    reader.fail();
    return false;
  }
  return true;
}
bool wholeBodyDynamicsSettings::nested_read_forceTorqueFilterCutoffInHz(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(forceTorqueFilterCutoffInHz)) {
    reader.fail();
    return false;
  }
  return true;
}
bool wholeBodyDynamicsSettings::read_jointVelFilterCutoffInHz(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(jointVelFilterCutoffInHz)) {
    reader.fail();
    return false;
  }
  return true;
}
bool wholeBodyDynamicsSettings::nested_read_jointVelFilterCutoffInHz(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(jointVelFilterCutoffInHz)) {
    reader.fail();
    return false;
  }
  return true;
}
bool wholeBodyDynamicsSettings::read_jointAccFilterCutoffInHz(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(jointAccFilterCutoffInHz)) {
    reader.fail();
    return false;
  }
  return true;
}
bool wholeBodyDynamicsSettings::nested_read_jointAccFilterCutoffInHz(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(jointAccFilterCutoffInHz)) {
    reader.fail();
    return false;
  }
  return true;
}
bool wholeBodyDynamicsSettings::read_useJointVelocity(yarp::os::idl::WireReader& reader) {
  if (!reader.readBool(useJointVelocity)) {
    reader.fail();
    return false;
  }
  return true;
}
bool wholeBodyDynamicsSettings::nested_read_useJointVelocity(yarp::os::idl::WireReader& reader) {
  if (!reader.readBool(useJointVelocity)) {
    reader.fail();
    return false;
  }
  return true;
}
bool wholeBodyDynamicsSettings::read_useJointAcceleration(yarp::os::idl::WireReader& reader) {
  if (!reader.readBool(useJointAcceleration)) {
    reader.fail();
    return false;
  }
  return true;
}
bool wholeBodyDynamicsSettings::nested_read_useJointAcceleration(yarp::os::idl::WireReader& reader) {
  if (!reader.readBool(useJointAcceleration)) {
    reader.fail();
    return false;
  }
  return true;
}
bool wholeBodyDynamicsSettings::read_estimationAlgorithm(yarp::os::idl::WireReader& reader) {
  int32_t ecast4;
  EstimationAlgorithmTypeVocab cvrt5;
  if (!reader.readEnum(ecast4,cvrt5)) {
    reader.fail();
    return false;
  } else {
    estimationAlgorithm = (EstimationAlgorithmType)ecast4;
  }
  return true;
}
bool wholeBodyDynamicsSettings::nested_read_estimationAlgorithm(yarp::os::idl::WireReader& reader) {
  int32_t ecast6;
  EstimationAlgorithmTypeVocab cvrt7;
  if (!reader.readEnum(ecast6,cvrt7)) {
    reader.fail();
    return false;
  } else {
    estimationAlgorithm = (EstimationAlgorithmType)ecast6;
  }
  return true;
}
bool wholeBodyDynamicsSettings::read(yarp::os::idl::WireReader& reader) {
  if (!read_kinematicSource(reader)) return false;
  if (!read_fixedFrameName(reader)) return false;
  if (!read_fixedFrameGravity(reader)) return false;
  if (!read_imuFrameName(reader)) return false;
  if (!read_imuFilterCutoffInHz(reader)) return false;
  if (!read_forceTorqueFilterCutoffInHz(reader)) return false;
  if (!read_jointVelFilterCutoffInHz(reader)) return false;
  if (!read_jointAccFilterCutoffInHz(reader)) return false;
  if (!read_useJointVelocity(reader)) return false;
  if (!read_useJointAcceleration(reader)) return false;
  if (!read_estimationAlgorithm(reader)) return false;
  return !reader.isError();
}

bool wholeBodyDynamicsSettings::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListHeader(13)) return false;
  return read(reader);
}

bool wholeBodyDynamicsSettings::write_kinematicSource(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeI32((int32_t)kinematicSource)) return false;
  return true;
}
bool wholeBodyDynamicsSettings::nested_write_kinematicSource(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeI32((int32_t)kinematicSource)) return false;
  return true;
}
bool wholeBodyDynamicsSettings::write_fixedFrameName(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeString(fixedFrameName)) return false;
  return true;
}
bool wholeBodyDynamicsSettings::nested_write_fixedFrameName(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeString(fixedFrameName)) return false;
  return true;
}
bool wholeBodyDynamicsSettings::write_fixedFrameGravity(yarp::os::idl::WireWriter& writer) {
  if (!writer.write(fixedFrameGravity)) return false;
  return true;
}
bool wholeBodyDynamicsSettings::nested_write_fixedFrameGravity(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeNested(fixedFrameGravity)) return false;
  return true;
}
bool wholeBodyDynamicsSettings::write_imuFrameName(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeString(imuFrameName)) return false;
  return true;
}
bool wholeBodyDynamicsSettings::nested_write_imuFrameName(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeString(imuFrameName)) return false;
  return true;
}
bool wholeBodyDynamicsSettings::write_imuFilterCutoffInHz(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(imuFilterCutoffInHz)) return false;
  return true;
}
bool wholeBodyDynamicsSettings::nested_write_imuFilterCutoffInHz(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(imuFilterCutoffInHz)) return false;
  return true;
}
bool wholeBodyDynamicsSettings::write_forceTorqueFilterCutoffInHz(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(forceTorqueFilterCutoffInHz)) return false;
  return true;
}
bool wholeBodyDynamicsSettings::nested_write_forceTorqueFilterCutoffInHz(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(forceTorqueFilterCutoffInHz)) return false;
  return true;
}
bool wholeBodyDynamicsSettings::write_jointVelFilterCutoffInHz(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(jointVelFilterCutoffInHz)) return false;
  return true;
}
bool wholeBodyDynamicsSettings::nested_write_jointVelFilterCutoffInHz(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(jointVelFilterCutoffInHz)) return false;
  return true;
}
bool wholeBodyDynamicsSettings::write_jointAccFilterCutoffInHz(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(jointAccFilterCutoffInHz)) return false;
  return true;
}
bool wholeBodyDynamicsSettings::nested_write_jointAccFilterCutoffInHz(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(jointAccFilterCutoffInHz)) return false;
  return true;
}
bool wholeBodyDynamicsSettings::write_useJointVelocity(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeBool(useJointVelocity)) return false;
  return true;
}
bool wholeBodyDynamicsSettings::nested_write_useJointVelocity(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeBool(useJointVelocity)) return false;
  return true;
}
bool wholeBodyDynamicsSettings::write_useJointAcceleration(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeBool(useJointAcceleration)) return false;
  return true;
}
bool wholeBodyDynamicsSettings::nested_write_useJointAcceleration(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeBool(useJointAcceleration)) return false;
  return true;
}
bool wholeBodyDynamicsSettings::write_estimationAlgorithm(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeI32((int32_t)estimationAlgorithm)) return false;
  return true;
}
bool wholeBodyDynamicsSettings::nested_write_estimationAlgorithm(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeI32((int32_t)estimationAlgorithm)) return false;
  return true;
}
bool wholeBodyDynamicsSettings::write(yarp::os::idl::WireWriter& writer) {
  if (!write_kinematicSource(writer)) return false;
  if (!write_fixedFrameName(writer)) return false;
  if (!write_fixedFrameGravity(writer)) return false;
  if (!write_imuFrameName(writer)) return false;
  if (!write_imuFilterCutoffInHz(writer)) return false;
  if (!write_forceTorqueFilterCutoffInHz(writer)) return false;
  if (!write_jointVelFilterCutoffInHz(writer)) return false;
  if (!write_jointAccFilterCutoffInHz(writer)) return false;
  if (!write_useJointVelocity(writer)) return false;
  if (!write_useJointAcceleration(writer)) return false;
  if (!write_estimationAlgorithm(writer)) return false;
  return !writer.isError();
}

bool wholeBodyDynamicsSettings::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(13)) return false;
  return write(writer);
}
bool wholeBodyDynamicsSettings::Editor::write(yarp::os::ConnectionWriter& connection) {
  if (!isValid()) return false;
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(dirty_count+1)) return false;
  if (!writer.writeString("patch")) return false;
  if (is_dirty_kinematicSource) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("kinematicSource")) return false;
    if (!obj->nested_write_kinematicSource(writer)) return false;
  }
  if (is_dirty_fixedFrameName) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("fixedFrameName")) return false;
    if (!obj->nested_write_fixedFrameName(writer)) return false;
  }
  if (is_dirty_fixedFrameGravity) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("fixedFrameGravity")) return false;
    if (!obj->nested_write_fixedFrameGravity(writer)) return false;
  }
  if (is_dirty_imuFrameName) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("imuFrameName")) return false;
    if (!obj->nested_write_imuFrameName(writer)) return false;
  }
  if (is_dirty_imuFilterCutoffInHz) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("imuFilterCutoffInHz")) return false;
    if (!obj->nested_write_imuFilterCutoffInHz(writer)) return false;
  }
  if (is_dirty_forceTorqueFilterCutoffInHz) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("forceTorqueFilterCutoffInHz")) return false;
    if (!obj->nested_write_forceTorqueFilterCutoffInHz(writer)) return false;
  }
  if (is_dirty_jointVelFilterCutoffInHz) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("jointVelFilterCutoffInHz")) return false;
    if (!obj->nested_write_jointVelFilterCutoffInHz(writer)) return false;
  }
  if (is_dirty_jointAccFilterCutoffInHz) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("jointAccFilterCutoffInHz")) return false;
    if (!obj->nested_write_jointAccFilterCutoffInHz(writer)) return false;
  }
  if (is_dirty_useJointVelocity) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("useJointVelocity")) return false;
    if (!obj->nested_write_useJointVelocity(writer)) return false;
  }
  if (is_dirty_useJointAcceleration) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("useJointAcceleration")) return false;
    if (!obj->nested_write_useJointAcceleration(writer)) return false;
  }
  if (is_dirty_estimationAlgorithm) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("estimationAlgorithm")) return false;
    if (!obj->nested_write_estimationAlgorithm(writer)) return false;
  }
  return !writer.isError();
}
bool wholeBodyDynamicsSettings::Editor::read(yarp::os::ConnectionReader& connection) {
  if (!isValid()) return false;
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) return false;
  int len = reader.getLength();
  if (len==0) {
    yarp::os::idl::WireWriter writer(reader);
    if (writer.isNull()) return true;
    if (!writer.writeListHeader(1)) return false;
    writer.writeString("send: 'help' or 'patch (param1 val1) (param2 val2)'");
    return true;
  }
  yarp::os::ConstString tag;
  if (!reader.readString(tag)) return false;
  if (tag=="help") {
    yarp::os::idl::WireWriter writer(reader);
    if (writer.isNull()) return true;
    if (!writer.writeListHeader(2)) return false;
    if (!writer.writeTag("many",1, 0)) return false;
    if (reader.getLength()>0) {
      yarp::os::ConstString field;
      if (!reader.readString(field)) return false;
      if (field=="kinematicSource") {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString("KinematicSourceType kinematicSource")) return false;
      }
      if (field=="fixedFrameName") {
        if (!writer.writeListHeader(2)) return false;
        if (!writer.writeString("std::string fixedFrameName")) return false;
        if (!writer.writeString("Specify the source of the kinematic information for one link, see KinematicSourceType information for more info.")) return false;
      }
      if (field=="fixedFrameGravity") {
        if (!writer.writeListHeader(2)) return false;
        if (!writer.writeString("Gravity fixedFrameGravity")) return false;
        if (!writer.writeString("If kinematicSource is FIXED_LINK, specify the frame of the robot that we know to be fixed (i.e. not moving with respect to an inertial frame)")) return false;
      }
      if (field=="imuFrameName") {
        if (!writer.writeListHeader(2)) return false;
        if (!writer.writeString("std::string imuFrameName")) return false;
        if (!writer.writeString("If kinematicSource is FIXED_LINK, specify the gravity vector (in m/s^2) in the fixedFrame")) return false;
      }
      if (field=="imuFilterCutoffInHz") {
        if (!writer.writeListHeader(2)) return false;
        if (!writer.writeString("double imuFilterCutoffInHz")) return false;
        if (!writer.writeString("If kinematicSource is IMU, specify the frame name of the imu")) return false;
      }
      if (field=="forceTorqueFilterCutoffInHz") {
        if (!writer.writeListHeader(2)) return false;
        if (!writer.writeString("double forceTorqueFilterCutoffInHz")) return false;
        if (!writer.writeString("Cutoff frequency (in Hz) of the first order filter of the IMU")) return false;
      }
      if (field=="jointVelFilterCutoffInHz") {
        if (!writer.writeListHeader(2)) return false;
        if (!writer.writeString("double jointVelFilterCutoffInHz")) return false;
        if (!writer.writeString("Cutoff frequency(in Hz) of the first order filter of the F/T sensors")) return false;
      }
      if (field=="jointAccFilterCutoffInHz") {
        if (!writer.writeListHeader(2)) return false;
        if (!writer.writeString("double jointAccFilterCutoffInHz")) return false;
        if (!writer.writeString("Cutoff frequency(in Hz) of the first order filter of the joint velocities")) return false;
      }
      if (field=="useJointVelocity") {
        if (!writer.writeListHeader(2)) return false;
        if (!writer.writeString("bool useJointVelocity")) return false;
        if (!writer.writeString("Cutoff frequency(in Hz) of the first order filter of the joint accelerations")) return false;
      }
      if (field=="useJointAcceleration") {
        if (!writer.writeListHeader(2)) return false;
        if (!writer.writeString("bool useJointAcceleration")) return false;
        if (!writer.writeString("Use the joint velocity measurement if this is true, assume they are zero otherwise.")) return false;
      }
      if (field=="estimationAlgorithm") {
        if (!writer.writeListHeader(2)) return false;
        if (!writer.writeString("EstimationAlgorithmType estimationAlgorithm")) return false;
        if (!writer.writeString("Use the joint acceleration measurment if this is true, assume they are zero otherwise.")) return false;
      }
    }
    if (!writer.writeListHeader(12)) return false;
    writer.writeString("*** Available fields:");
    writer.writeString("kinematicSource");
    writer.writeString("fixedFrameName");
    writer.writeString("fixedFrameGravity");
    writer.writeString("imuFrameName");
    writer.writeString("imuFilterCutoffInHz");
    writer.writeString("forceTorqueFilterCutoffInHz");
    writer.writeString("jointVelFilterCutoffInHz");
    writer.writeString("jointAccFilterCutoffInHz");
    writer.writeString("useJointVelocity");
    writer.writeString("useJointAcceleration");
    writer.writeString("estimationAlgorithm");
    return true;
  }
  bool nested = true;
  bool have_act = false;
  if (tag!="patch") {
    if ((len-1)%2 != 0) return false;
    len = 1 + ((len-1)/2);
    nested = false;
    have_act = true;
  }
  for (int i=1; i<len; i++) {
    if (nested && !reader.readListHeader(3)) return false;
    yarp::os::ConstString act;
    yarp::os::ConstString key;
    if (have_act) {
      act = tag;
    } else {
      if (!reader.readString(act)) return false;
    }
    if (!reader.readString(key)) return false;
    // inefficient code follows, bug paulfitz to improve it
    if (key == "kinematicSource") {
      will_set_kinematicSource();
      if (!obj->nested_read_kinematicSource(reader)) return false;
      did_set_kinematicSource();
    } else if (key == "fixedFrameName") {
      will_set_fixedFrameName();
      if (!obj->nested_read_fixedFrameName(reader)) return false;
      did_set_fixedFrameName();
    } else if (key == "fixedFrameGravity") {
      will_set_fixedFrameGravity();
      if (!obj->nested_read_fixedFrameGravity(reader)) return false;
      did_set_fixedFrameGravity();
    } else if (key == "imuFrameName") {
      will_set_imuFrameName();
      if (!obj->nested_read_imuFrameName(reader)) return false;
      did_set_imuFrameName();
    } else if (key == "imuFilterCutoffInHz") {
      will_set_imuFilterCutoffInHz();
      if (!obj->nested_read_imuFilterCutoffInHz(reader)) return false;
      did_set_imuFilterCutoffInHz();
    } else if (key == "forceTorqueFilterCutoffInHz") {
      will_set_forceTorqueFilterCutoffInHz();
      if (!obj->nested_read_forceTorqueFilterCutoffInHz(reader)) return false;
      did_set_forceTorqueFilterCutoffInHz();
    } else if (key == "jointVelFilterCutoffInHz") {
      will_set_jointVelFilterCutoffInHz();
      if (!obj->nested_read_jointVelFilterCutoffInHz(reader)) return false;
      did_set_jointVelFilterCutoffInHz();
    } else if (key == "jointAccFilterCutoffInHz") {
      will_set_jointAccFilterCutoffInHz();
      if (!obj->nested_read_jointAccFilterCutoffInHz(reader)) return false;
      did_set_jointAccFilterCutoffInHz();
    } else if (key == "useJointVelocity") {
      will_set_useJointVelocity();
      if (!obj->nested_read_useJointVelocity(reader)) return false;
      did_set_useJointVelocity();
    } else if (key == "useJointAcceleration") {
      will_set_useJointAcceleration();
      if (!obj->nested_read_useJointAcceleration(reader)) return false;
      did_set_useJointAcceleration();
    } else if (key == "estimationAlgorithm") {
      will_set_estimationAlgorithm();
      if (!obj->nested_read_estimationAlgorithm(reader)) return false;
      did_set_estimationAlgorithm();
    } else {
      // would be useful to have a fallback here
    }
  }
  reader.accept();
  yarp::os::idl::WireWriter writer(reader);
  if (writer.isNull()) return true;
  writer.writeListHeader(1);
  writer.writeVocab(VOCAB2('o','k'));
  return true;
}

yarp::os::ConstString wholeBodyDynamicsSettings::toString() {
  yarp::os::Bottle b;
  b.read(*this);
  return b.toString();
}
