// Minimal msgpack codec for the AirSim RPC bridge.
//
// Only the subset needed by AirSim's msgpack-rpc protocol is implemented:
// integers, booleans, floats, strings, binary blobs, arrays and maps with
// string keys.  AirSim replies are decoded with binary blobs kept intact so
// that PNG images and float point clouds survive untouched.

#ifndef AIRSIM_ROS2_BRIDGE__MSGPACK_HPP_
#define AIRSIM_ROS2_BRIDGE__MSGPACK_HPP_

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace airsim_bridge
{
namespace msgpack
{

class Error : public std::runtime_error
{
public:
  explicit Error(const std::string & what)
  : std::runtime_error(what) {}
};

class Value
{
public:
  enum class Type
  {
    Nil,
    Bool,
    Int,
    UInt,
    Float,
    String,
    Bin,
    Array,
    Map,
  };

  Value() = default;

  static Value make_bool(bool value) {Value v; v.type_ = Type::Bool; v.b_ = value; return v;}
  static Value make_int(int64_t value) {Value v; v.type_ = Type::Int; v.i_ = value; return v;}
  static Value make_uint(uint64_t value) {Value v; v.type_ = Type::UInt; v.u_ = value; return v;}
  static Value make_double(double value) {Value v; v.type_ = Type::Float; v.f_ = value; return v;}
  static Value make_string(std::string value)
  {
    Value v; v.type_ = Type::String; v.s_ = std::move(value); return v;
  }
  static Value make_bin(std::string value)
  {
    Value v; v.type_ = Type::Bin; v.s_ = std::move(value); return v;
  }
  static Value make_array() {Value v; v.type_ = Type::Array; return v;}
  static Value make_map() {Value v; v.type_ = Type::Map; return v;}

  Type type() const {return type_;}
  bool is_nil() const {return type_ == Type::Nil;}
  bool is_array() const {return type_ == Type::Array;}
  bool is_map() const {return type_ == Type::Map;}

  bool as_bool() const
  {
    require(Type::Bool, "boolean");
    return b_;
  }

  int64_t as_int() const
  {
    if (type_ == Type::Int) {return i_;}
    if (type_ == Type::UInt) {return static_cast<int64_t>(u_);}
    if (type_ == Type::Float) {return static_cast<int64_t>(f_);}
    throw Error("value is not an integer");
  }

  uint64_t as_uint() const
  {
    if (type_ == Type::UInt) {return u_;}
    if (type_ == Type::Int) {return static_cast<uint64_t>(i_);}
    if (type_ == Type::Float) {return static_cast<uint64_t>(f_);}
    throw Error("value is not an unsigned integer");
  }

  double as_double() const
  {
    if (type_ == Type::Float) {return f_;}
    if (type_ == Type::Int) {return static_cast<double>(i_);}
    if (type_ == Type::UInt) {return static_cast<double>(u_);}
    throw Error("value is not a number");
  }

  // String and Bin payloads are both returned as raw bytes.
  const std::string & as_string() const
  {
    if (type_ != Type::String && type_ != Type::Bin) {
      throw Error("value is not a string or binary blob");
    }
    return s_;
  }

  const std::vector<Value> & as_array() const
  {
    require(Type::Array, "array");
    return a_;
  }

  size_t size() const
  {
    if (type_ == Type::Array) {return a_.size();}
    if (type_ == Type::Map) {return m_.size();}
    throw Error("value has no size");
  }

  const Value & at(size_t index) const
  {
    require(Type::Array, "array");
    if (index >= a_.size()) {
      throw Error("array index out of range");
    }
    return a_[index];
  }

  const Value * find(const std::string & key) const
  {
    if (type_ != Type::Map) {return nullptr;}
    for (const auto & entry : m_) {
      if (entry.first == key) {return &entry.second;}
    }
    return nullptr;
  }

  const Value & at(const std::string & key) const
  {
    const Value * found = find(key);
    if (found == nullptr) {
      throw Error("missing map key: " + key);
    }
    return *found;
  }

  const std::vector<std::pair<std::string, Value>> & entries() const
  {
    require(Type::Map, "map");
    return m_;
  }

  void push(Value value) {a_.push_back(std::move(value));}
  void reserve(size_t count) {a_.reserve(count);}
  void set(std::string key, Value value) {m_.emplace_back(std::move(key), std::move(value));}

private:
  void require(Type expected, const char * what) const
  {
    if (type_ != expected) {
      throw Error(std::string("value is not a ") + what);
    }
  }

  Type type_ = Type::Nil;
  bool b_ = false;
  int64_t i_ = 0;
  uint64_t u_ = 0;
  double f_ = 0.0;
  std::string s_;
  std::vector<Value> a_;
  std::vector<std::pair<std::string, Value>> m_;
};

class Decoder
{
public:
  explicit Decoder(const std::string & data)
  : data_(data) {}

  // Parses one value at the start of `data_`.  Returns false when the buffer
  // holds only part of a value; otherwise `consumed` receives its byte count.
  bool next(Value & out, size_t & consumed) const
  {
    size_t pos = 0;
    try {
      out = decode(pos);
    } catch (const Incomplete &) {
      return false;
    }
    consumed = pos;
    return true;
  }

private:
  struct Incomplete {};

  void check(size_t pos, size_t count) const
  {
    if (pos + count > data_.size()) {throw Incomplete();}
  }

  uint8_t u8(size_t & pos) const
  {
    check(pos, 1);
    return static_cast<uint8_t>(data_[pos++]);
  }

  uint16_t u16(size_t & pos) const
  {
    check(pos, 2);
    uint16_t value = static_cast<uint16_t>(
      (static_cast<uint8_t>(data_[pos]) << 8) | static_cast<uint8_t>(data_[pos + 1]));
    pos += 2;
    return value;
  }

  uint32_t u32(size_t & pos) const
  {
    check(pos, 4);
    uint32_t value = 0;
    for (int i = 0; i < 4; ++i) {
      value = (value << 8) | static_cast<uint8_t>(data_[pos + i]);
    }
    pos += 4;
    return value;
  }

  uint64_t u64(size_t & pos) const
  {
    check(pos, 8);
    uint64_t value = 0;
    for (int i = 0; i < 8; ++i) {
      value = (value << 8) | static_cast<uint8_t>(data_[pos + i]);
    }
    pos += 8;
    return value;
  }

  float f32(size_t & pos) const
  {
    uint32_t bits = u32(pos);
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
  }

  double f64(size_t & pos) const
  {
    uint64_t bits = u64(pos);
    double value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
  }

  std::string bytes(size_t & pos, size_t count) const
  {
    check(pos, count);
    std::string value(data_, pos, count);
    pos += count;
    return value;
  }

  Value decode(size_t & pos) const
  {
    const uint8_t code = u8(pos);
    if (code <= 0x7f) {return Value::make_uint(code);}
    if (code >= 0xe0) {return Value::make_int(static_cast<int8_t>(code));}
    if ((code & 0xf0) == 0x80) {return decode_map(pos, code & 0x0f);}
    if ((code & 0xf0) == 0x90) {return decode_array(pos, code & 0x0f);}
    if ((code & 0xe0) == 0xa0) {return Value::make_string(bytes(pos, code & 0x1f));}

    switch (code) {
      case 0xc0: return Value();
      case 0xc2: return Value::make_bool(false);
      case 0xc3: return Value::make_bool(true);
      case 0xc4: return Value::make_bin(bytes(pos, u8(pos)));
      case 0xc5: return Value::make_bin(bytes(pos, u16(pos)));
      case 0xc6: return Value::make_bin(bytes(pos, u32(pos)));
      case 0xca: return Value::make_double(f32(pos));
      case 0xcb: return Value::make_double(f64(pos));
      case 0xcc: return Value::make_uint(u8(pos));
      case 0xcd: return Value::make_uint(u16(pos));
      case 0xce: return Value::make_uint(u32(pos));
      case 0xcf: return Value::make_uint(u64(pos));
      case 0xd0: return Value::make_int(static_cast<int8_t>(u8(pos)));
      case 0xd1: return Value::make_int(static_cast<int16_t>(u16(pos)));
      case 0xd2: return Value::make_int(static_cast<int32_t>(u32(pos)));
      case 0xd3: return Value::make_int(static_cast<int64_t>(u64(pos)));
      case 0xd9: return Value::make_string(bytes(pos, u8(pos)));
      case 0xda: return Value::make_string(bytes(pos, u16(pos)));
      case 0xdb: return Value::make_string(bytes(pos, u32(pos)));
      case 0xdc: return decode_array(pos, u16(pos));
      case 0xdd: return decode_array(pos, u32(pos));
      case 0xde: return decode_map(pos, u16(pos));
      case 0xdf: return decode_map(pos, u32(pos));
      default:
        throw Error("unsupported msgpack code " + std::to_string(code));
    }
  }

  Value decode_array(size_t & pos, size_t count) const
  {
    Value value = Value::make_array();
    value.reserve(count);
    for (size_t i = 0; i < count; ++i) {
      value.push(decode(pos));
    }
    return value;
  }

  Value decode_map(size_t & pos, size_t count) const
  {
    Value value = Value::make_map();
    for (size_t i = 0; i < count; ++i) {
      Value key = decode(pos);
      if (key.type() != Value::Type::String) {
        throw Error("msgpack map key is not a string");
      }
      value.set(key.as_string(), decode(pos));
    }
    return value;
  }

  const std::string & data_;
};

class Writer
{
public:
  void value(const Value & input)
  {
    switch (input.type()) {
      case Value::Type::Nil:
        u8(0xc0);
        break;
      case Value::Type::Bool:
        u8(input.as_bool() ? 0xc3 : 0xc2);
        break;
      case Value::Type::Int:
        write_int(input.as_int());
        break;
      case Value::Type::UInt:
        write_uint(input.as_uint());
        break;
      case Value::Type::Float:
        write_double(input.as_double());
        break;
      case Value::Type::String:
        write_string(input.as_string());
        break;
      case Value::Type::Bin:
        write_bin(input.as_string());
        break;
      case Value::Type::Array:
        write_array_header(input.size());
        for (const auto & item : input.as_array()) {
          value(item);
        }
        break;
      case Value::Type::Map:
        write_map_header(input.size());
        for (size_t i = 0; i < input.size(); ++i) {
          // Map entries are encoded from the same accessor used by as_string().
          const auto & entry = input.entries().at(i);
          write_string(entry.first);
          value(entry.second);
        }
        break;
    }
  }

  const std::string & buffer() const {return buffer_;}

private:
  void u8(uint8_t byte) {buffer_.push_back(static_cast<char>(byte));}

  void be16(uint16_t value)
  {
    u8(static_cast<uint8_t>(value >> 8));
    u8(static_cast<uint8_t>(value));
  }

  void be32(uint32_t value)
  {
    for (int shift = 24; shift >= 0; shift -= 8) {
      u8(static_cast<uint8_t>(value >> shift));
    }
  }

  void be64(uint64_t value)
  {
    for (int shift = 56; shift >= 0; shift -= 8) {
      u8(static_cast<uint8_t>(value >> shift));
    }
  }

  void write_uint(uint64_t value)
  {
    if (value <= 0x7f) {
      u8(static_cast<uint8_t>(value));
    } else if (value <= 0xff) {
      u8(0xcc);
      u8(static_cast<uint8_t>(value));
    } else if (value <= 0xffff) {
      u8(0xcd);
      be16(static_cast<uint16_t>(value));
    } else if (value <= 0xffffffffULL) {
      u8(0xce);
      be32(static_cast<uint32_t>(value));
    } else {
      u8(0xcf);
      be64(value);
    }
  }

  void write_int(int64_t value)
  {
    if (value >= 0) {
      write_uint(static_cast<uint64_t>(value));
    } else if (value >= -32) {
      u8(static_cast<uint8_t>(value));
    } else if (value >= INT8_MIN) {
      u8(0xd0);
      u8(static_cast<uint8_t>(value));
    } else if (value >= INT16_MIN) {
      u8(0xd1);
      be16(static_cast<uint16_t>(value));
    } else if (value >= INT32_MIN) {
      u8(0xd2);
      be32(static_cast<uint32_t>(value));
    } else {
      u8(0xd3);
      be64(static_cast<uint64_t>(value));
    }
  }

  void write_double(double value)
  {
    uint64_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    u8(0xcb);
    be64(bits);
  }

  void write_string(const std::string & value)
  {
    const size_t count = value.size();
    if (count <= 31) {
      u8(static_cast<uint8_t>(0xa0 | count));
    } else if (count <= 0xff) {
      u8(0xd9);
      u8(static_cast<uint8_t>(count));
    } else if (count <= 0xffff) {
      u8(0xda);
      be16(static_cast<uint16_t>(count));
    } else {
      u8(0xdb);
      be32(static_cast<uint32_t>(count));
    }
    buffer_.append(value);
  }

  void write_bin(const std::string & value)
  {
    const size_t count = value.size();
    if (count <= 0xff) {
      u8(0xc4);
      u8(static_cast<uint8_t>(count));
    } else if (count <= 0xffff) {
      u8(0xc5);
      be16(static_cast<uint16_t>(count));
    } else {
      u8(0xc6);
      be32(static_cast<uint32_t>(count));
    }
    buffer_.append(value);
  }

  void write_array_header(size_t count)
  {
    if (count <= 15) {
      u8(static_cast<uint8_t>(0x90 | count));
    } else if (count <= 0xffff) {
      u8(0xdc);
      be16(static_cast<uint16_t>(count));
    } else {
      u8(0xdd);
      be32(static_cast<uint32_t>(count));
    }
  }

  void write_map_header(size_t count)
  {
    if (count <= 15) {
      u8(static_cast<uint8_t>(0x80 | count));
    } else if (count <= 0xffff) {
      u8(0xde);
      be16(static_cast<uint16_t>(count));
    } else {
      u8(0xdf);
      be32(static_cast<uint32_t>(count));
    }
  }

  std::string buffer_;
};

}  // namespace msgpack
}  // namespace airsim_bridge

#endif  // AIRSIM_ROS2_BRIDGE__MSGPACK_HPP_
