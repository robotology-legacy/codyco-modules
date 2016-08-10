// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <codyco/HomTransform.h>

namespace codyco {
bool HomTransform::read_x(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(x)) {
    reader.fail();
    return false;
  }
  return true;
}
bool HomTransform::nested_read_x(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(x)) {
    reader.fail();
    return false;
  }
  return true;
}
bool HomTransform::read_y(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(y)) {
    reader.fail();
    return false;
  }
  return true;
}
bool HomTransform::nested_read_y(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(y)) {
    reader.fail();
    return false;
  }
  return true;
}
bool HomTransform::read_z(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(z)) {
    reader.fail();
    return false;
  }
  return true;
}
bool HomTransform::nested_read_z(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(z)) {
    reader.fail();
    return false;
  }
  return true;
}
bool HomTransform::read_xx(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(xx)) {
    reader.fail();
    return false;
  }
  return true;
}
bool HomTransform::nested_read_xx(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(xx)) {
    reader.fail();
    return false;
  }
  return true;
}
bool HomTransform::read_xy(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(xy)) {
    reader.fail();
    return false;
  }
  return true;
}
bool HomTransform::nested_read_xy(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(xy)) {
    reader.fail();
    return false;
  }
  return true;
}
bool HomTransform::read_xz(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(xz)) {
    reader.fail();
    return false;
  }
  return true;
}
bool HomTransform::nested_read_xz(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(xz)) {
    reader.fail();
    return false;
  }
  return true;
}
bool HomTransform::read_yx(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(yx)) {
    reader.fail();
    return false;
  }
  return true;
}
bool HomTransform::nested_read_yx(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(yx)) {
    reader.fail();
    return false;
  }
  return true;
}
bool HomTransform::read_yy(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(yy)) {
    reader.fail();
    return false;
  }
  return true;
}
bool HomTransform::nested_read_yy(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(yy)) {
    reader.fail();
    return false;
  }
  return true;
}
bool HomTransform::read_yz(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(yz)) {
    reader.fail();
    return false;
  }
  return true;
}
bool HomTransform::nested_read_yz(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(yz)) {
    reader.fail();
    return false;
  }
  return true;
}
bool HomTransform::read_zx(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(zx)) {
    reader.fail();
    return false;
  }
  return true;
}
bool HomTransform::nested_read_zx(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(zx)) {
    reader.fail();
    return false;
  }
  return true;
}
bool HomTransform::read_zy(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(zy)) {
    reader.fail();
    return false;
  }
  return true;
}
bool HomTransform::nested_read_zy(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(zy)) {
    reader.fail();
    return false;
  }
  return true;
}
bool HomTransform::read_zz(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(zz)) {
    reader.fail();
    return false;
  }
  return true;
}
bool HomTransform::nested_read_zz(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(zz)) {
    reader.fail();
    return false;
  }
  return true;
}
bool HomTransform::read(yarp::os::idl::WireReader& reader) {
  if (!read_x(reader)) return false;
  if (!read_y(reader)) return false;
  if (!read_z(reader)) return false;
  if (!read_xx(reader)) return false;
  if (!read_xy(reader)) return false;
  if (!read_xz(reader)) return false;
  if (!read_yx(reader)) return false;
  if (!read_yy(reader)) return false;
  if (!read_yz(reader)) return false;
  if (!read_zx(reader)) return false;
  if (!read_zy(reader)) return false;
  if (!read_zz(reader)) return false;
  return !reader.isError();
}

bool HomTransform::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListHeader(12)) return false;
  return read(reader);
}

bool HomTransform::write_x(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(x)) return false;
  return true;
}
bool HomTransform::nested_write_x(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(x)) return false;
  return true;
}
bool HomTransform::write_y(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(y)) return false;
  return true;
}
bool HomTransform::nested_write_y(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(y)) return false;
  return true;
}
bool HomTransform::write_z(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(z)) return false;
  return true;
}
bool HomTransform::nested_write_z(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(z)) return false;
  return true;
}
bool HomTransform::write_xx(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(xx)) return false;
  return true;
}
bool HomTransform::nested_write_xx(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(xx)) return false;
  return true;
}
bool HomTransform::write_xy(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(xy)) return false;
  return true;
}
bool HomTransform::nested_write_xy(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(xy)) return false;
  return true;
}
bool HomTransform::write_xz(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(xz)) return false;
  return true;
}
bool HomTransform::nested_write_xz(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(xz)) return false;
  return true;
}
bool HomTransform::write_yx(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(yx)) return false;
  return true;
}
bool HomTransform::nested_write_yx(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(yx)) return false;
  return true;
}
bool HomTransform::write_yy(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(yy)) return false;
  return true;
}
bool HomTransform::nested_write_yy(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(yy)) return false;
  return true;
}
bool HomTransform::write_yz(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(yz)) return false;
  return true;
}
bool HomTransform::nested_write_yz(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(yz)) return false;
  return true;
}
bool HomTransform::write_zx(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(zx)) return false;
  return true;
}
bool HomTransform::nested_write_zx(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(zx)) return false;
  return true;
}
bool HomTransform::write_zy(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(zy)) return false;
  return true;
}
bool HomTransform::nested_write_zy(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(zy)) return false;
  return true;
}
bool HomTransform::write_zz(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(zz)) return false;
  return true;
}
bool HomTransform::nested_write_zz(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(zz)) return false;
  return true;
}
bool HomTransform::write(yarp::os::idl::WireWriter& writer) {
  if (!write_x(writer)) return false;
  if (!write_y(writer)) return false;
  if (!write_z(writer)) return false;
  if (!write_xx(writer)) return false;
  if (!write_xy(writer)) return false;
  if (!write_xz(writer)) return false;
  if (!write_yx(writer)) return false;
  if (!write_yy(writer)) return false;
  if (!write_yz(writer)) return false;
  if (!write_zx(writer)) return false;
  if (!write_zy(writer)) return false;
  if (!write_zz(writer)) return false;
  return !writer.isError();
}

bool HomTransform::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(12)) return false;
  return write(writer);
}
bool HomTransform::Editor::write(yarp::os::ConnectionWriter& connection) {
  if (!isValid()) return false;
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(dirty_count+1)) return false;
  if (!writer.writeString("patch")) return false;
  if (is_dirty_x) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("x")) return false;
    if (!obj->nested_write_x(writer)) return false;
  }
  if (is_dirty_y) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("y")) return false;
    if (!obj->nested_write_y(writer)) return false;
  }
  if (is_dirty_z) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("z")) return false;
    if (!obj->nested_write_z(writer)) return false;
  }
  if (is_dirty_xx) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("xx")) return false;
    if (!obj->nested_write_xx(writer)) return false;
  }
  if (is_dirty_xy) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("xy")) return false;
    if (!obj->nested_write_xy(writer)) return false;
  }
  if (is_dirty_xz) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("xz")) return false;
    if (!obj->nested_write_xz(writer)) return false;
  }
  if (is_dirty_yx) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("yx")) return false;
    if (!obj->nested_write_yx(writer)) return false;
  }
  if (is_dirty_yy) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("yy")) return false;
    if (!obj->nested_write_yy(writer)) return false;
  }
  if (is_dirty_yz) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("yz")) return false;
    if (!obj->nested_write_yz(writer)) return false;
  }
  if (is_dirty_zx) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("zx")) return false;
    if (!obj->nested_write_zx(writer)) return false;
  }
  if (is_dirty_zy) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("zy")) return false;
    if (!obj->nested_write_zy(writer)) return false;
  }
  if (is_dirty_zz) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("zz")) return false;
    if (!obj->nested_write_zz(writer)) return false;
  }
  return !writer.isError();
}
bool HomTransform::Editor::read(yarp::os::ConnectionReader& connection) {
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
      if (field=="x") {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString("double x")) return false;
      }
      if (field=="y") {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString("double y")) return false;
      }
      if (field=="z") {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString("double z")) return false;
      }
      if (field=="xx") {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString("double xx")) return false;
      }
      if (field=="xy") {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString("double xy")) return false;
      }
      if (field=="xz") {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString("double xz")) return false;
      }
      if (field=="yx") {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString("double yx")) return false;
      }
      if (field=="yy") {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString("double yy")) return false;
      }
      if (field=="yz") {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString("double yz")) return false;
      }
      if (field=="zx") {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString("double zx")) return false;
      }
      if (field=="zy") {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString("double zy")) return false;
      }
      if (field=="zz") {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString("double zz")) return false;
      }
    }
    if (!writer.writeListHeader(13)) return false;
    writer.writeString("*** Available fields:");
    writer.writeString("x");
    writer.writeString("y");
    writer.writeString("z");
    writer.writeString("xx");
    writer.writeString("xy");
    writer.writeString("xz");
    writer.writeString("yx");
    writer.writeString("yy");
    writer.writeString("yz");
    writer.writeString("zx");
    writer.writeString("zy");
    writer.writeString("zz");
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
    if (key == "x") {
      will_set_x();
      if (!obj->nested_read_x(reader)) return false;
      did_set_x();
    } else if (key == "y") {
      will_set_y();
      if (!obj->nested_read_y(reader)) return false;
      did_set_y();
    } else if (key == "z") {
      will_set_z();
      if (!obj->nested_read_z(reader)) return false;
      did_set_z();
    } else if (key == "xx") {
      will_set_xx();
      if (!obj->nested_read_xx(reader)) return false;
      did_set_xx();
    } else if (key == "xy") {
      will_set_xy();
      if (!obj->nested_read_xy(reader)) return false;
      did_set_xy();
    } else if (key == "xz") {
      will_set_xz();
      if (!obj->nested_read_xz(reader)) return false;
      did_set_xz();
    } else if (key == "yx") {
      will_set_yx();
      if (!obj->nested_read_yx(reader)) return false;
      did_set_yx();
    } else if (key == "yy") {
      will_set_yy();
      if (!obj->nested_read_yy(reader)) return false;
      did_set_yy();
    } else if (key == "yz") {
      will_set_yz();
      if (!obj->nested_read_yz(reader)) return false;
      did_set_yz();
    } else if (key == "zx") {
      will_set_zx();
      if (!obj->nested_read_zx(reader)) return false;
      did_set_zx();
    } else if (key == "zy") {
      will_set_zy();
      if (!obj->nested_read_zy(reader)) return false;
      did_set_zy();
    } else if (key == "zz") {
      will_set_zz();
      if (!obj->nested_read_zz(reader)) return false;
      did_set_zz();
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

yarp::os::ConstString HomTransform::toString() {
  yarp::os::Bottle b;
  b.read(*this);
  return b.toString();
}
} // namespace
