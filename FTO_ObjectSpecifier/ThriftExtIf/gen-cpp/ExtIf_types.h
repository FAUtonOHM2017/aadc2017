/**
 * Autogenerated by Thrift Compiler (0.9.1)
 *
 * DO NOT EDIT UNLESS YOU ARE SURE THAT YOU KNOW WHAT YOU ARE DOING
 *  @generated
 */
#ifndef ExtIf_TYPES_H
#define ExtIf_TYPES_H

#include <thrift/Thrift.h>
#include <thrift/TApplicationException.h>
#include <thrift/protocol/TProtocol.h>
#include <thrift/transport/TTransport.h>

#include <thrift/cxxfunctional.h>


namespace ext_iface {

struct TransportDef {
  enum type {
    IMAGEDATA = 0
  };
};

extern const std::map<int, const char*> _TransportDef_VALUES_TO_NAMES;

typedef std::vector<class TDataResult>  TDataResultList;


class TROI {
 public:

  static const char* ascii_fingerprint; // = "FCD9BA0161383B6B58D8659284FB9442";
  static const uint8_t binary_fingerprint[16]; // = {0xFC,0xD9,0xBA,0x01,0x61,0x38,0x3B,0x6B,0x58,0xD8,0x65,0x92,0x84,0xFB,0x94,0x42};

  TROI() : x(0), y(0), width(0), height(0), name() {
  }

  virtual ~TROI() throw() {}

  int16_t x;
  int16_t y;
  int16_t width;
  int16_t height;
  std::string name;

  void __set_x(const int16_t val) {
    x = val;
  }

  void __set_y(const int16_t val) {
    y = val;
  }

  void __set_width(const int16_t val) {
    width = val;
  }

  void __set_height(const int16_t val) {
    height = val;
  }

  void __set_name(const std::string& val) {
    name = val;
  }

  bool operator == (const TROI & rhs) const
  {
    if (!(x == rhs.x))
      return false;
    if (!(y == rhs.y))
      return false;
    if (!(width == rhs.width))
      return false;
    if (!(height == rhs.height))
      return false;
    if (!(name == rhs.name))
      return false;
    return true;
  }
  bool operator != (const TROI &rhs) const {
    return !(*this == rhs);
  }

  bool operator < (const TROI & ) const;

  uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
  uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

};

void swap(TROI &a, TROI &b);


class TDataRaw {
 public:

  static const char* ascii_fingerprint; // = "164D6961738D9688907EDB2C60DFEC00";
  static const uint8_t binary_fingerprint[16]; // = {0x16,0x4D,0x69,0x61,0x73,0x8D,0x96,0x88,0x90,0x7E,0xDB,0x2C,0x60,0xDF,0xEC,0x00};

  TDataRaw() : raw_data() {
  }

  virtual ~TDataRaw() throw() {}

  std::string raw_data;
  std::vector<TROI>  rois;

  void __set_raw_data(const std::string& val) {
    raw_data = val;
  }

  void __set_rois(const std::vector<TROI> & val) {
    rois = val;
  }

  bool operator == (const TDataRaw & rhs) const
  {
    if (!(raw_data == rhs.raw_data))
      return false;
    if (!(rois == rhs.rois))
      return false;
    return true;
  }
  bool operator != (const TDataRaw &rhs) const {
    return !(*this == rhs);
  }

  bool operator < (const TDataRaw & ) const;

  uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
  uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

};

void swap(TDataRaw &a, TDataRaw &b);


class TDataClassification {
 public:

  static const char* ascii_fingerprint; // = "C712EF0DA8599E55DF4D0F13415232EF";
  static const uint8_t binary_fingerprint[16]; // = {0xC7,0x12,0xEF,0x0D,0xA8,0x59,0x9E,0x55,0xDF,0x4D,0x0F,0x13,0x41,0x52,0x32,0xEF};

  TDataClassification() : classification(), probability(0) {
  }

  virtual ~TDataClassification() throw() {}

  std::string classification;
  double probability;

  void __set_classification(const std::string& val) {
    classification = val;
  }

  void __set_probability(const double val) {
    probability = val;
  }

  bool operator == (const TDataClassification & rhs) const
  {
    if (!(classification == rhs.classification))
      return false;
    if (!(probability == rhs.probability))
      return false;
    return true;
  }
  bool operator != (const TDataClassification &rhs) const {
    return !(*this == rhs);
  }

  bool operator < (const TDataClassification & ) const;

  uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
  uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

};

void swap(TDataClassification &a, TDataClassification &b);


class TDataResult {
 public:

  static const char* ascii_fingerprint; // = "97B35AFE0B91EB82ACDED8D1F93B9A9C";
  static const uint8_t binary_fingerprint[16]; // = {0x97,0xB3,0x5A,0xFE,0x0B,0x91,0xEB,0x82,0xAC,0xDE,0xD8,0xD1,0xF9,0x3B,0x9A,0x9C};

  TDataResult() {
  }

  virtual ~TDataResult() throw() {}

  TROI roi;
  std::vector<TDataClassification>  results;

  void __set_roi(const TROI& val) {
    roi = val;
  }

  void __set_results(const std::vector<TDataClassification> & val) {
    results = val;
  }

  bool operator == (const TDataResult & rhs) const
  {
    if (!(roi == rhs.roi))
      return false;
    if (!(results == rhs.results))
      return false;
    return true;
  }
  bool operator != (const TDataResult &rhs) const {
    return !(*this == rhs);
  }

  bool operator < (const TDataResult & ) const;

  uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
  uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

};

void swap(TDataResult &a, TDataResult &b);


class TImageParams {
 public:

  static const char* ascii_fingerprint; // = "EEBEE5F2DAE75B1AB615147C163DCA93";
  static const uint8_t binary_fingerprint[16]; // = {0xEE,0xBE,0xE5,0xF2,0xDA,0xE7,0x5B,0x1A,0xB6,0x15,0x14,0x7C,0x16,0x3D,0xCA,0x93};

  TImageParams() : height(0), width(0), bytesPerPixel(0) {
  }

  virtual ~TImageParams() throw() {}

  int16_t height;
  int16_t width;
  int16_t bytesPerPixel;

  void __set_height(const int16_t val) {
    height = val;
  }

  void __set_width(const int16_t val) {
    width = val;
  }

  void __set_bytesPerPixel(const int16_t val) {
    bytesPerPixel = val;
  }

  bool operator == (const TImageParams & rhs) const
  {
    if (!(height == rhs.height))
      return false;
    if (!(width == rhs.width))
      return false;
    if (!(bytesPerPixel == rhs.bytesPerPixel))
      return false;
    return true;
  }
  bool operator != (const TImageParams &rhs) const {
    return !(*this == rhs);
  }

  bool operator < (const TImageParams & ) const;

  uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
  uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

};

void swap(TImageParams &a, TImageParams &b);

typedef struct _TIoException__isset {
  _TIoException__isset() : message(false) {}
  bool message;
} _TIoException__isset;

class TIoException : public ::apache::thrift::TException {
 public:

  static const char* ascii_fingerprint; // = "EFB929595D312AC8F305D5A794CFEDA1";
  static const uint8_t binary_fingerprint[16]; // = {0xEF,0xB9,0x29,0x59,0x5D,0x31,0x2A,0xC8,0xF3,0x05,0xD5,0xA7,0x94,0xCF,0xED,0xA1};

  TIoException() : message() {
  }

  virtual ~TIoException() throw() {}

  std::string message;

  _TIoException__isset __isset;

  void __set_message(const std::string& val) {
    message = val;
  }

  bool operator == (const TIoException & rhs) const
  {
    if (!(message == rhs.message))
      return false;
    return true;
  }
  bool operator != (const TIoException &rhs) const {
    return !(*this == rhs);
  }

  bool operator < (const TIoException & ) const;

  uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
  uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

};

void swap(TIoException &a, TIoException &b);

} // namespace

#endif