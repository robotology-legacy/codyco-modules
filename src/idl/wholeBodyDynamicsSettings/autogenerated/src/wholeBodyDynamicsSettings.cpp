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
bool wholeBodyDynamicsSettings::read_gravity(yarp::os::idl::WireReader& reader) {
  if (!reader.read(gravity)) {
    reader.fail();
    return false;
  }
  return true;
}
bool wholeBodyDynamicsSettings::nested_read_gravity(yarp::os::idl::WireReader& reader) {
  if (!reader.readNested(gravity)) {
    reader.fail();
    return false;
  }
  return true;
}
bool wholeBodyDynamicsSettings::read_imuFilterCutoff(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(imuFilterCutoff)) {
    reader.fail();
    return false;
  }
  return true;
}
bool wholeBodyDynamicsSettings::nested_read_imuFilterCutoff(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(imuFilterCutoff)) {
    reader.fail();
    return false;
  }
  return true;
}
bool wholeBodyDynamicsSettings::read_forceTorqueFilterCutoff(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(forceTorqueFilterCutoff)) {
    reader.fail();
    return false;
  }
  return true;
}
bool wholeBodyDynamicsSettings::nested_read_forceTorqueFilterCutoff(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(forceTorqueFilterCutoff)) {
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
bool wholeBodyDynamicsSettings::read(yarp::os::idl::WireReader& reader) {
  if (!read_kinematicSource(reader)) return false;
  if (!read_fixedFrameName(reader)) return false;
  if (!read_gravity(reader)) return false;
  if (!read_imuFilterCutoff(reader)) return false;
  if (!read_forceTorqueFilterCutoff(reader)) return false;
  if (!read_useJointVelocity(reader)) return false;
  if (!read_useJointAcceleration(reader)) return false;
  return !reader.isError();
}

bool wholeBodyDynamicsSettings::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListHeader(9)) return false;
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
bool wholeBodyDynamicsSettings::write_gravity(yarp::os::idl::WireWriter& writer) {
  if (!writer.write(gravity)) return false;
  return true;
}
bool wholeBodyDynamicsSettings::nested_write_gravity(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeNested(gravity)) return false;
  return true;
}
bool wholeBodyDynamicsSettings::write_imuFilterCutoff(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(imuFilterCutoff)) return false;
  return true;
}
bool wholeBodyDynamicsSettings::nested_write_imuFilterCutoff(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(imuFilterCutoff)) return false;
  return true;
}
bool wholeBodyDynamicsSettings::write_forceTorqueFilterCutoff(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(forceTorqueFilterCutoff)) return false;
  return true;
}
bool wholeBodyDynamicsSettings::nested_write_forceTorqueFilterCutoff(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(forceTorqueFilterCutoff)) return false;
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
bool wholeBodyDynamicsSettings::write(yarp::os::idl::WireWriter& writer) {
  if (!write_kinematicSource(writer)) return false;
  if (!write_fixedFrameName(writer)) return false;
  if (!write_gravity(writer)) return false;
  if (!write_imuFilterCutoff(writer)) return false;
  if (!write_forceTorqueFilterCutoff(writer)) return false;
  if (!write_useJointVelocity(writer)) return false;
  if (!write_useJointAcceleration(writer)) return false;
  return !writer.isError();
}

bool wholeBodyDynamicsSettings::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(9)) return false;
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
  if (is_dirty_gravity) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("gravity")) return false;
    if (!obj->nested_write_gravity(writer)) return false;
  }
  if (is_dirty_imuFilterCutoff) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("imuFilterCutoff")) return false;
    if (!obj->nested_write_imuFilterCutoff(writer)) return false;
  }
  if (is_dirty_forceTorqueFilterCutoff) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("forceTorqueFilterCutoff")) return false;
    if (!obj->nested_write_forceTorqueFilterCutoff(writer)) return false;
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
      if (field=="gravity") {
        if (!writer.writeListHeader(2)) return false;
        if (!writer.writeString("Gravity gravity")) return false;
        if (!writer.writeString("If kinematicSource is FIXED_LINK, specify the frame of the robot that we know to be fixed (i.e. not moving with respect to an inertial frame)")) return false;
      }
      if (field=="imuFilterCutoff") {
        if (!writer.writeListHeader(2)) return false;
        if (!writer.writeString("double imuFilterCutoff")) return false;
        if (!writer.writeString("If kinematicSource is FIXED_LINK, specify the gravity vector in the fixedFrame")) return false;
      }
      if (field=="forceTorqueFilterCutoff") {
        if (!writer.writeListHeader(2)) return false;
        if (!writer.writeString("double forceTorqueFilterCutoff")) return false;
        if (!writer.writeString("Cutoff frequency of the first order filter of the IMU")) return false;
      }
      if (field=="useJointVelocity") {
        if (!writer.writeListHeader(2)) return false;
        if (!writer.writeString("bool useJointVelocity")) return false;
        if (!writer.writeString("Cutoff frequency of the first order filter of the F/T sensors")) return false;
      }
      if (field=="useJointAcceleration") {
        if (!writer.writeListHeader(2)) return false;
        if (!writer.writeString("bool useJointAcceleration")) return false;
        if (!writer.writeString("Use the joint velocity measurement if this is true, assume they are zero otherwise.")) return false;
      }
    }
    if (!writer.writeListHeader(8)) return false;
    writer.writeString("*** Available fields:");
    writer.writeString("kinematicSource");
    writer.writeString("fixedFrameName");
    writer.writeString("gravity");
    writer.writeString("imuFilterCutoff");
    writer.writeString("forceTorqueFilterCutoff");
    writer.writeString("useJointVelocity");
    writer.writeString("useJointAcceleration");
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
    } else if (key == "gravity") {
      will_set_gravity();
      if (!obj->nested_read_gravity(reader)) return false;
      did_set_gravity();
    } else if (key == "imuFilterCutoff") {
      will_set_imuFilterCutoff();
      if (!obj->nested_read_imuFilterCutoff(reader)) return false;
      did_set_imuFilterCutoff();
    } else if (key == "forceTorqueFilterCutoff") {
      will_set_forceTorqueFilterCutoff();
      if (!obj->nested_read_forceTorqueFilterCutoff(reader)) return false;
      did_set_forceTorqueFilterCutoff();
    } else if (key == "useJointVelocity") {
      will_set_useJointVelocity();
      if (!obj->nested_read_useJointVelocity(reader)) return false;
      did_set_useJointVelocity();
    } else if (key == "useJointAcceleration") {
      will_set_useJointAcceleration();
      if (!obj->nested_read_useJointAcceleration(reader)) return false;
      did_set_useJointAcceleration();
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
