// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/perception/lidar/lib/segmentation/cnnseg/proto/cnnseg_config.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2fperception_2flidar_2flib_2fsegmentation_2fcnnseg_2fproto_2fcnnseg_5fconfig_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2fperception_2flidar_2flib_2fsegmentation_2fcnnseg_2fproto_2fcnnseg_5fconfig_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3011000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3011004 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_modules_2fperception_2flidar_2flib_2fsegmentation_2fcnnseg_2fproto_2fcnnseg_5fconfig_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2fperception_2flidar_2flib_2fsegmentation_2fcnnseg_2fproto_2fcnnseg_5fconfig_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxillaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[1]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fperception_2flidar_2flib_2fsegmentation_2fcnnseg_2fproto_2fcnnseg_5fconfig_2eproto;
namespace apollo {
namespace perception {
namespace lidar {
class CNNSegConfig;
class CNNSegConfigDefaultTypeInternal;
extern CNNSegConfigDefaultTypeInternal _CNNSegConfig_default_instance_;
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::perception::lidar::CNNSegConfig* Arena::CreateMaybeMessage<::apollo::perception::lidar::CNNSegConfig>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace perception {
namespace lidar {

// ===================================================================

class CNNSegConfig :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.perception.lidar.CNNSegConfig) */ {
 public:
  CNNSegConfig();
  virtual ~CNNSegConfig();

  CNNSegConfig(const CNNSegConfig& from);
  CNNSegConfig(CNNSegConfig&& from) noexcept
    : CNNSegConfig() {
    *this = ::std::move(from);
  }

  inline CNNSegConfig& operator=(const CNNSegConfig& from) {
    CopyFrom(from);
    return *this;
  }
  inline CNNSegConfig& operator=(CNNSegConfig&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  inline const ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return GetMetadataStatic().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return GetMetadataStatic().reflection;
  }
  static const CNNSegConfig& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const CNNSegConfig* internal_default_instance() {
    return reinterpret_cast<const CNNSegConfig*>(
               &_CNNSegConfig_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(CNNSegConfig& a, CNNSegConfig& b) {
    a.Swap(&b);
  }
  inline void Swap(CNNSegConfig* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline CNNSegConfig* New() const final {
    return CreateMaybeMessage<CNNSegConfig>(nullptr);
  }

  CNNSegConfig* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<CNNSegConfig>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const CNNSegConfig& from);
  void MergeFrom(const CNNSegConfig& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  inline void SharedCtor();
  inline void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(CNNSegConfig* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.perception.lidar.CNNSegConfig";
  }
  private:
  inline ::PROTOBUF_NAMESPACE_ID::Arena* GetArenaNoVirtual() const {
    return nullptr;
  }
  inline void* MaybeArenaPtr() const {
    return nullptr;
  }
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2fperception_2flidar_2flib_2fsegmentation_2fcnnseg_2fproto_2fcnnseg_5fconfig_2eproto);
    return ::descriptor_table_modules_2fperception_2flidar_2flib_2fsegmentation_2fcnnseg_2fproto_2fcnnseg_5fconfig_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kParamFileFieldNumber = 1,
    kProtoFileFieldNumber = 2,
    kWeightFileFieldNumber = 3,
    kEngineFileFieldNumber = 4,
  };
  // optional string param_file = 1 [default = "./data/models/cnnseg/param.conf"];
  bool has_param_file() const;
  private:
  bool _internal_has_param_file() const;
  public:
  void clear_param_file();
  const std::string& param_file() const;
  void set_param_file(const std::string& value);
  void set_param_file(std::string&& value);
  void set_param_file(const char* value);
  void set_param_file(const char* value, size_t size);
  std::string* mutable_param_file();
  std::string* release_param_file();
  void set_allocated_param_file(std::string* param_file);
  private:
  const std::string& _internal_param_file() const;
  void _internal_set_param_file(const std::string& value);
  std::string* _internal_mutable_param_file();
  public:

  // optional string proto_file = 2 [default = "./data/models/cnnseg/deploy.prototxt"];
  bool has_proto_file() const;
  private:
  bool _internal_has_proto_file() const;
  public:
  void clear_proto_file();
  const std::string& proto_file() const;
  void set_proto_file(const std::string& value);
  void set_proto_file(std::string&& value);
  void set_proto_file(const char* value);
  void set_proto_file(const char* value, size_t size);
  std::string* mutable_proto_file();
  std::string* release_proto_file();
  void set_allocated_proto_file(std::string* proto_file);
  private:
  const std::string& _internal_proto_file() const;
  void _internal_set_proto_file(const std::string& value);
  std::string* _internal_mutable_proto_file();
  public:

  // optional string weight_file = 3 [default = "./data/models/cnnseg/deploy.caffemodel"];
  bool has_weight_file() const;
  private:
  bool _internal_has_weight_file() const;
  public:
  void clear_weight_file();
  const std::string& weight_file() const;
  void set_weight_file(const std::string& value);
  void set_weight_file(std::string&& value);
  void set_weight_file(const char* value);
  void set_weight_file(const char* value, size_t size);
  std::string* mutable_weight_file();
  std::string* release_weight_file();
  void set_allocated_weight_file(std::string* weight_file);
  private:
  const std::string& _internal_weight_file() const;
  void _internal_set_weight_file(const std::string& value);
  std::string* _internal_mutable_weight_file();
  public:

  // optional string engine_file = 4 [default = "./data/models/cnnseg/engine.conf"];
  bool has_engine_file() const;
  private:
  bool _internal_has_engine_file() const;
  public:
  void clear_engine_file();
  const std::string& engine_file() const;
  void set_engine_file(const std::string& value);
  void set_engine_file(std::string&& value);
  void set_engine_file(const char* value);
  void set_engine_file(const char* value, size_t size);
  std::string* mutable_engine_file();
  std::string* release_engine_file();
  void set_allocated_engine_file(std::string* engine_file);
  private:
  const std::string& _internal_engine_file() const;
  void _internal_set_engine_file(const std::string& value);
  std::string* _internal_mutable_engine_file();
  public:

  // @@protoc_insertion_point(class_scope:apollo.perception.lidar.CNNSegConfig)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  public:
  static ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<std::string> _i_give_permission_to_break_this_code_default_param_file_;
  private:
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr param_file_;
  public:
  static ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<std::string> _i_give_permission_to_break_this_code_default_proto_file_;
  private:
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr proto_file_;
  public:
  static ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<std::string> _i_give_permission_to_break_this_code_default_weight_file_;
  private:
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr weight_file_;
  public:
  static ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<std::string> _i_give_permission_to_break_this_code_default_engine_file_;
  private:
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr engine_file_;
  friend struct ::TableStruct_modules_2fperception_2flidar_2flib_2fsegmentation_2fcnnseg_2fproto_2fcnnseg_5fconfig_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// CNNSegConfig

// optional string param_file = 1 [default = "./data/models/cnnseg/param.conf"];
inline bool CNNSegConfig::_internal_has_param_file() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool CNNSegConfig::has_param_file() const {
  return _internal_has_param_file();
}
inline void CNNSegConfig::clear_param_file() {
  param_file_.ClearToDefaultNoArena(&::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_param_file_.get());
  _has_bits_[0] &= ~0x00000001u;
}
inline const std::string& CNNSegConfig::param_file() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.CNNSegConfig.param_file)
  return _internal_param_file();
}
inline void CNNSegConfig::set_param_file(const std::string& value) {
  _internal_set_param_file(value);
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.CNNSegConfig.param_file)
}
inline std::string* CNNSegConfig::mutable_param_file() {
  // @@protoc_insertion_point(field_mutable:apollo.perception.lidar.CNNSegConfig.param_file)
  return _internal_mutable_param_file();
}
inline const std::string& CNNSegConfig::_internal_param_file() const {
  return param_file_.GetNoArena();
}
inline void CNNSegConfig::_internal_set_param_file(const std::string& value) {
  _has_bits_[0] |= 0x00000001u;
  param_file_.SetNoArena(&::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_param_file_.get(), value);
}
inline void CNNSegConfig::set_param_file(std::string&& value) {
  _has_bits_[0] |= 0x00000001u;
  param_file_.SetNoArena(
    &::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_param_file_.get(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:apollo.perception.lidar.CNNSegConfig.param_file)
}
inline void CNNSegConfig::set_param_file(const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  _has_bits_[0] |= 0x00000001u;
  param_file_.SetNoArena(&::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_param_file_.get(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:apollo.perception.lidar.CNNSegConfig.param_file)
}
inline void CNNSegConfig::set_param_file(const char* value, size_t size) {
  _has_bits_[0] |= 0x00000001u;
  param_file_.SetNoArena(&::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_param_file_.get(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:apollo.perception.lidar.CNNSegConfig.param_file)
}
inline std::string* CNNSegConfig::_internal_mutable_param_file() {
  _has_bits_[0] |= 0x00000001u;
  return param_file_.MutableNoArena(&::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_param_file_.get());
}
inline std::string* CNNSegConfig::release_param_file() {
  // @@protoc_insertion_point(field_release:apollo.perception.lidar.CNNSegConfig.param_file)
  if (!_internal_has_param_file()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000001u;
  return param_file_.ReleaseNonDefaultNoArena(&::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_param_file_.get());
}
inline void CNNSegConfig::set_allocated_param_file(std::string* param_file) {
  if (param_file != nullptr) {
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  param_file_.SetAllocatedNoArena(&::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_param_file_.get(), param_file);
  // @@protoc_insertion_point(field_set_allocated:apollo.perception.lidar.CNNSegConfig.param_file)
}

// optional string proto_file = 2 [default = "./data/models/cnnseg/deploy.prototxt"];
inline bool CNNSegConfig::_internal_has_proto_file() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool CNNSegConfig::has_proto_file() const {
  return _internal_has_proto_file();
}
inline void CNNSegConfig::clear_proto_file() {
  proto_file_.ClearToDefaultNoArena(&::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_proto_file_.get());
  _has_bits_[0] &= ~0x00000002u;
}
inline const std::string& CNNSegConfig::proto_file() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.CNNSegConfig.proto_file)
  return _internal_proto_file();
}
inline void CNNSegConfig::set_proto_file(const std::string& value) {
  _internal_set_proto_file(value);
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.CNNSegConfig.proto_file)
}
inline std::string* CNNSegConfig::mutable_proto_file() {
  // @@protoc_insertion_point(field_mutable:apollo.perception.lidar.CNNSegConfig.proto_file)
  return _internal_mutable_proto_file();
}
inline const std::string& CNNSegConfig::_internal_proto_file() const {
  return proto_file_.GetNoArena();
}
inline void CNNSegConfig::_internal_set_proto_file(const std::string& value) {
  _has_bits_[0] |= 0x00000002u;
  proto_file_.SetNoArena(&::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_proto_file_.get(), value);
}
inline void CNNSegConfig::set_proto_file(std::string&& value) {
  _has_bits_[0] |= 0x00000002u;
  proto_file_.SetNoArena(
    &::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_proto_file_.get(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:apollo.perception.lidar.CNNSegConfig.proto_file)
}
inline void CNNSegConfig::set_proto_file(const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  _has_bits_[0] |= 0x00000002u;
  proto_file_.SetNoArena(&::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_proto_file_.get(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:apollo.perception.lidar.CNNSegConfig.proto_file)
}
inline void CNNSegConfig::set_proto_file(const char* value, size_t size) {
  _has_bits_[0] |= 0x00000002u;
  proto_file_.SetNoArena(&::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_proto_file_.get(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:apollo.perception.lidar.CNNSegConfig.proto_file)
}
inline std::string* CNNSegConfig::_internal_mutable_proto_file() {
  _has_bits_[0] |= 0x00000002u;
  return proto_file_.MutableNoArena(&::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_proto_file_.get());
}
inline std::string* CNNSegConfig::release_proto_file() {
  // @@protoc_insertion_point(field_release:apollo.perception.lidar.CNNSegConfig.proto_file)
  if (!_internal_has_proto_file()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000002u;
  return proto_file_.ReleaseNonDefaultNoArena(&::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_proto_file_.get());
}
inline void CNNSegConfig::set_allocated_proto_file(std::string* proto_file) {
  if (proto_file != nullptr) {
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  proto_file_.SetAllocatedNoArena(&::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_proto_file_.get(), proto_file);
  // @@protoc_insertion_point(field_set_allocated:apollo.perception.lidar.CNNSegConfig.proto_file)
}

// optional string weight_file = 3 [default = "./data/models/cnnseg/deploy.caffemodel"];
inline bool CNNSegConfig::_internal_has_weight_file() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool CNNSegConfig::has_weight_file() const {
  return _internal_has_weight_file();
}
inline void CNNSegConfig::clear_weight_file() {
  weight_file_.ClearToDefaultNoArena(&::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_weight_file_.get());
  _has_bits_[0] &= ~0x00000004u;
}
inline const std::string& CNNSegConfig::weight_file() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.CNNSegConfig.weight_file)
  return _internal_weight_file();
}
inline void CNNSegConfig::set_weight_file(const std::string& value) {
  _internal_set_weight_file(value);
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.CNNSegConfig.weight_file)
}
inline std::string* CNNSegConfig::mutable_weight_file() {
  // @@protoc_insertion_point(field_mutable:apollo.perception.lidar.CNNSegConfig.weight_file)
  return _internal_mutable_weight_file();
}
inline const std::string& CNNSegConfig::_internal_weight_file() const {
  return weight_file_.GetNoArena();
}
inline void CNNSegConfig::_internal_set_weight_file(const std::string& value) {
  _has_bits_[0] |= 0x00000004u;
  weight_file_.SetNoArena(&::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_weight_file_.get(), value);
}
inline void CNNSegConfig::set_weight_file(std::string&& value) {
  _has_bits_[0] |= 0x00000004u;
  weight_file_.SetNoArena(
    &::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_weight_file_.get(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:apollo.perception.lidar.CNNSegConfig.weight_file)
}
inline void CNNSegConfig::set_weight_file(const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  _has_bits_[0] |= 0x00000004u;
  weight_file_.SetNoArena(&::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_weight_file_.get(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:apollo.perception.lidar.CNNSegConfig.weight_file)
}
inline void CNNSegConfig::set_weight_file(const char* value, size_t size) {
  _has_bits_[0] |= 0x00000004u;
  weight_file_.SetNoArena(&::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_weight_file_.get(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:apollo.perception.lidar.CNNSegConfig.weight_file)
}
inline std::string* CNNSegConfig::_internal_mutable_weight_file() {
  _has_bits_[0] |= 0x00000004u;
  return weight_file_.MutableNoArena(&::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_weight_file_.get());
}
inline std::string* CNNSegConfig::release_weight_file() {
  // @@protoc_insertion_point(field_release:apollo.perception.lidar.CNNSegConfig.weight_file)
  if (!_internal_has_weight_file()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000004u;
  return weight_file_.ReleaseNonDefaultNoArena(&::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_weight_file_.get());
}
inline void CNNSegConfig::set_allocated_weight_file(std::string* weight_file) {
  if (weight_file != nullptr) {
    _has_bits_[0] |= 0x00000004u;
  } else {
    _has_bits_[0] &= ~0x00000004u;
  }
  weight_file_.SetAllocatedNoArena(&::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_weight_file_.get(), weight_file);
  // @@protoc_insertion_point(field_set_allocated:apollo.perception.lidar.CNNSegConfig.weight_file)
}

// optional string engine_file = 4 [default = "./data/models/cnnseg/engine.conf"];
inline bool CNNSegConfig::_internal_has_engine_file() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool CNNSegConfig::has_engine_file() const {
  return _internal_has_engine_file();
}
inline void CNNSegConfig::clear_engine_file() {
  engine_file_.ClearToDefaultNoArena(&::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_engine_file_.get());
  _has_bits_[0] &= ~0x00000008u;
}
inline const std::string& CNNSegConfig::engine_file() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.CNNSegConfig.engine_file)
  return _internal_engine_file();
}
inline void CNNSegConfig::set_engine_file(const std::string& value) {
  _internal_set_engine_file(value);
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.CNNSegConfig.engine_file)
}
inline std::string* CNNSegConfig::mutable_engine_file() {
  // @@protoc_insertion_point(field_mutable:apollo.perception.lidar.CNNSegConfig.engine_file)
  return _internal_mutable_engine_file();
}
inline const std::string& CNNSegConfig::_internal_engine_file() const {
  return engine_file_.GetNoArena();
}
inline void CNNSegConfig::_internal_set_engine_file(const std::string& value) {
  _has_bits_[0] |= 0x00000008u;
  engine_file_.SetNoArena(&::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_engine_file_.get(), value);
}
inline void CNNSegConfig::set_engine_file(std::string&& value) {
  _has_bits_[0] |= 0x00000008u;
  engine_file_.SetNoArena(
    &::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_engine_file_.get(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:apollo.perception.lidar.CNNSegConfig.engine_file)
}
inline void CNNSegConfig::set_engine_file(const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  _has_bits_[0] |= 0x00000008u;
  engine_file_.SetNoArena(&::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_engine_file_.get(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:apollo.perception.lidar.CNNSegConfig.engine_file)
}
inline void CNNSegConfig::set_engine_file(const char* value, size_t size) {
  _has_bits_[0] |= 0x00000008u;
  engine_file_.SetNoArena(&::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_engine_file_.get(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:apollo.perception.lidar.CNNSegConfig.engine_file)
}
inline std::string* CNNSegConfig::_internal_mutable_engine_file() {
  _has_bits_[0] |= 0x00000008u;
  return engine_file_.MutableNoArena(&::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_engine_file_.get());
}
inline std::string* CNNSegConfig::release_engine_file() {
  // @@protoc_insertion_point(field_release:apollo.perception.lidar.CNNSegConfig.engine_file)
  if (!_internal_has_engine_file()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000008u;
  return engine_file_.ReleaseNonDefaultNoArena(&::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_engine_file_.get());
}
inline void CNNSegConfig::set_allocated_engine_file(std::string* engine_file) {
  if (engine_file != nullptr) {
    _has_bits_[0] |= 0x00000008u;
  } else {
    _has_bits_[0] &= ~0x00000008u;
  }
  engine_file_.SetAllocatedNoArena(&::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_engine_file_.get(), engine_file);
  // @@protoc_insertion_point(field_set_allocated:apollo.perception.lidar.CNNSegConfig.engine_file)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2fperception_2flidar_2flib_2fsegmentation_2fcnnseg_2fproto_2fcnnseg_5fconfig_2eproto
