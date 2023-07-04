// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/v2x/proto/v2x_service_obu_to_car.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2fv2x_2fproto_2fv2x_5fservice_5fobu_5fto_5fcar_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2fv2x_2fproto_2fv2x_5fservice_5fobu_5fto_5fcar_2eproto

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
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/v2x/proto/v2x_traffic_light.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_modules_2fv2x_2fproto_2fv2x_5fservice_5fobu_5fto_5fcar_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2fv2x_2fproto_2fv2x_5fservice_5fobu_5fto_5fcar_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fv2x_2fproto_2fv2x_5fservice_5fobu_5fto_5fcar_2eproto;
namespace apollo {
namespace v2x {
class StatusResponse;
class StatusResponseDefaultTypeInternal;
extern StatusResponseDefaultTypeInternal _StatusResponse_default_instance_;
}  // namespace v2x
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::v2x::StatusResponse* Arena::CreateMaybeMessage<::apollo::v2x::StatusResponse>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace v2x {

// ===================================================================

class StatusResponse :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.v2x.StatusResponse) */ {
 public:
  StatusResponse();
  virtual ~StatusResponse();

  StatusResponse(const StatusResponse& from);
  StatusResponse(StatusResponse&& from) noexcept
    : StatusResponse() {
    *this = ::std::move(from);
  }

  inline StatusResponse& operator=(const StatusResponse& from) {
    CopyFrom(from);
    return *this;
  }
  inline StatusResponse& operator=(StatusResponse&& from) noexcept {
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
  static const StatusResponse& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const StatusResponse* internal_default_instance() {
    return reinterpret_cast<const StatusResponse*>(
               &_StatusResponse_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(StatusResponse& a, StatusResponse& b) {
    a.Swap(&b);
  }
  inline void Swap(StatusResponse* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline StatusResponse* New() const final {
    return CreateMaybeMessage<StatusResponse>(nullptr);
  }

  StatusResponse* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<StatusResponse>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const StatusResponse& from);
  void MergeFrom(const StatusResponse& from);
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
  void InternalSwap(StatusResponse* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.v2x.StatusResponse";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2fv2x_2fproto_2fv2x_5fservice_5fobu_5fto_5fcar_2eproto);
    return ::descriptor_table_modules_2fv2x_2fproto_2fv2x_5fservice_5fobu_5fto_5fcar_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kInfoFieldNumber = 2,
    kErrorCodeFieldNumber = 3,
    kStatusFieldNumber = 1,
  };
  // optional string info = 2;
  bool has_info() const;
  private:
  bool _internal_has_info() const;
  public:
  void clear_info();
  const std::string& info() const;
  void set_info(const std::string& value);
  void set_info(std::string&& value);
  void set_info(const char* value);
  void set_info(const char* value, size_t size);
  std::string* mutable_info();
  std::string* release_info();
  void set_allocated_info(std::string* info);
  private:
  const std::string& _internal_info() const;
  void _internal_set_info(const std::string& value);
  std::string* _internal_mutable_info();
  public:

  // optional int64 error_code = 3;
  bool has_error_code() const;
  private:
  bool _internal_has_error_code() const;
  public:
  void clear_error_code();
  ::PROTOBUF_NAMESPACE_ID::int64 error_code() const;
  void set_error_code(::PROTOBUF_NAMESPACE_ID::int64 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int64 _internal_error_code() const;
  void _internal_set_error_code(::PROTOBUF_NAMESPACE_ID::int64 value);
  public:

  // required bool status = 1 [default = false];
  bool has_status() const;
  private:
  bool _internal_has_status() const;
  public:
  void clear_status();
  bool status() const;
  void set_status(bool value);
  private:
  bool _internal_status() const;
  void _internal_set_status(bool value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.v2x.StatusResponse)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr info_;
  ::PROTOBUF_NAMESPACE_ID::int64 error_code_;
  bool status_;
  friend struct ::TableStruct_modules_2fv2x_2fproto_2fv2x_5fservice_5fobu_5fto_5fcar_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// StatusResponse

// required bool status = 1 [default = false];
inline bool StatusResponse::_internal_has_status() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool StatusResponse::has_status() const {
  return _internal_has_status();
}
inline void StatusResponse::clear_status() {
  status_ = false;
  _has_bits_[0] &= ~0x00000004u;
}
inline bool StatusResponse::_internal_status() const {
  return status_;
}
inline bool StatusResponse::status() const {
  // @@protoc_insertion_point(field_get:apollo.v2x.StatusResponse.status)
  return _internal_status();
}
inline void StatusResponse::_internal_set_status(bool value) {
  _has_bits_[0] |= 0x00000004u;
  status_ = value;
}
inline void StatusResponse::set_status(bool value) {
  _internal_set_status(value);
  // @@protoc_insertion_point(field_set:apollo.v2x.StatusResponse.status)
}

// optional string info = 2;
inline bool StatusResponse::_internal_has_info() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool StatusResponse::has_info() const {
  return _internal_has_info();
}
inline void StatusResponse::clear_info() {
  info_.ClearToEmptyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  _has_bits_[0] &= ~0x00000001u;
}
inline const std::string& StatusResponse::info() const {
  // @@protoc_insertion_point(field_get:apollo.v2x.StatusResponse.info)
  return _internal_info();
}
inline void StatusResponse::set_info(const std::string& value) {
  _internal_set_info(value);
  // @@protoc_insertion_point(field_set:apollo.v2x.StatusResponse.info)
}
inline std::string* StatusResponse::mutable_info() {
  // @@protoc_insertion_point(field_mutable:apollo.v2x.StatusResponse.info)
  return _internal_mutable_info();
}
inline const std::string& StatusResponse::_internal_info() const {
  return info_.GetNoArena();
}
inline void StatusResponse::_internal_set_info(const std::string& value) {
  _has_bits_[0] |= 0x00000001u;
  info_.SetNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), value);
}
inline void StatusResponse::set_info(std::string&& value) {
  _has_bits_[0] |= 0x00000001u;
  info_.SetNoArena(
    &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:apollo.v2x.StatusResponse.info)
}
inline void StatusResponse::set_info(const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  _has_bits_[0] |= 0x00000001u;
  info_.SetNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:apollo.v2x.StatusResponse.info)
}
inline void StatusResponse::set_info(const char* value, size_t size) {
  _has_bits_[0] |= 0x00000001u;
  info_.SetNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:apollo.v2x.StatusResponse.info)
}
inline std::string* StatusResponse::_internal_mutable_info() {
  _has_bits_[0] |= 0x00000001u;
  return info_.MutableNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}
inline std::string* StatusResponse::release_info() {
  // @@protoc_insertion_point(field_release:apollo.v2x.StatusResponse.info)
  if (!_internal_has_info()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000001u;
  return info_.ReleaseNonDefaultNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}
inline void StatusResponse::set_allocated_info(std::string* info) {
  if (info != nullptr) {
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  info_.SetAllocatedNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), info);
  // @@protoc_insertion_point(field_set_allocated:apollo.v2x.StatusResponse.info)
}

// optional int64 error_code = 3;
inline bool StatusResponse::_internal_has_error_code() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool StatusResponse::has_error_code() const {
  return _internal_has_error_code();
}
inline void StatusResponse::clear_error_code() {
  error_code_ = PROTOBUF_LONGLONG(0);
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::int64 StatusResponse::_internal_error_code() const {
  return error_code_;
}
inline ::PROTOBUF_NAMESPACE_ID::int64 StatusResponse::error_code() const {
  // @@protoc_insertion_point(field_get:apollo.v2x.StatusResponse.error_code)
  return _internal_error_code();
}
inline void StatusResponse::_internal_set_error_code(::PROTOBUF_NAMESPACE_ID::int64 value) {
  _has_bits_[0] |= 0x00000002u;
  error_code_ = value;
}
inline void StatusResponse::set_error_code(::PROTOBUF_NAMESPACE_ID::int64 value) {
  _internal_set_error_code(value);
  // @@protoc_insertion_point(field_set:apollo.v2x.StatusResponse.error_code)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace v2x
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2fv2x_2fproto_2fv2x_5fservice_5fobu_5fto_5fcar_2eproto
