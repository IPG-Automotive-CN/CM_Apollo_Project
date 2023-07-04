// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/drivers/canbus/proto/sensor_canbus_conf.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2fdrivers_2fcanbus_2fproto_2fsensor_5fcanbus_5fconf_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2fdrivers_2fcanbus_2fproto_2fsensor_5fcanbus_5fconf_2eproto

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
#include "modules/drivers/canbus/proto/can_card_parameter.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_modules_2fdrivers_2fcanbus_2fproto_2fsensor_5fcanbus_5fconf_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2fdrivers_2fcanbus_2fproto_2fsensor_5fcanbus_5fconf_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fdrivers_2fcanbus_2fproto_2fsensor_5fcanbus_5fconf_2eproto;
namespace apollo {
namespace drivers {
namespace canbus {
class SensorCanbusConf;
class SensorCanbusConfDefaultTypeInternal;
extern SensorCanbusConfDefaultTypeInternal _SensorCanbusConf_default_instance_;
}  // namespace canbus
}  // namespace drivers
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::drivers::canbus::SensorCanbusConf* Arena::CreateMaybeMessage<::apollo::drivers::canbus::SensorCanbusConf>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace drivers {
namespace canbus {

// ===================================================================

class SensorCanbusConf :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.drivers.canbus.SensorCanbusConf) */ {
 public:
  SensorCanbusConf();
  virtual ~SensorCanbusConf();

  SensorCanbusConf(const SensorCanbusConf& from);
  SensorCanbusConf(SensorCanbusConf&& from) noexcept
    : SensorCanbusConf() {
    *this = ::std::move(from);
  }

  inline SensorCanbusConf& operator=(const SensorCanbusConf& from) {
    CopyFrom(from);
    return *this;
  }
  inline SensorCanbusConf& operator=(SensorCanbusConf&& from) noexcept {
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
  static const SensorCanbusConf& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const SensorCanbusConf* internal_default_instance() {
    return reinterpret_cast<const SensorCanbusConf*>(
               &_SensorCanbusConf_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(SensorCanbusConf& a, SensorCanbusConf& b) {
    a.Swap(&b);
  }
  inline void Swap(SensorCanbusConf* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline SensorCanbusConf* New() const final {
    return CreateMaybeMessage<SensorCanbusConf>(nullptr);
  }

  SensorCanbusConf* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<SensorCanbusConf>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const SensorCanbusConf& from);
  void MergeFrom(const SensorCanbusConf& from);
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
  void InternalSwap(SensorCanbusConf* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.drivers.canbus.SensorCanbusConf";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2fdrivers_2fcanbus_2fproto_2fsensor_5fcanbus_5fconf_2eproto);
    return ::descriptor_table_modules_2fdrivers_2fcanbus_2fproto_2fsensor_5fcanbus_5fconf_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kCanCardParameterFieldNumber = 1,
    kEnableDebugModeFieldNumber = 2,
    kEnableReceiverLogFieldNumber = 3,
  };
  // optional .apollo.drivers.canbus.CANCardParameter can_card_parameter = 1;
  bool has_can_card_parameter() const;
  private:
  bool _internal_has_can_card_parameter() const;
  public:
  void clear_can_card_parameter();
  const ::apollo::drivers::canbus::CANCardParameter& can_card_parameter() const;
  ::apollo::drivers::canbus::CANCardParameter* release_can_card_parameter();
  ::apollo::drivers::canbus::CANCardParameter* mutable_can_card_parameter();
  void set_allocated_can_card_parameter(::apollo::drivers::canbus::CANCardParameter* can_card_parameter);
  private:
  const ::apollo::drivers::canbus::CANCardParameter& _internal_can_card_parameter() const;
  ::apollo::drivers::canbus::CANCardParameter* _internal_mutable_can_card_parameter();
  public:

  // optional bool enable_debug_mode = 2 [default = false];
  bool has_enable_debug_mode() const;
  private:
  bool _internal_has_enable_debug_mode() const;
  public:
  void clear_enable_debug_mode();
  bool enable_debug_mode() const;
  void set_enable_debug_mode(bool value);
  private:
  bool _internal_enable_debug_mode() const;
  void _internal_set_enable_debug_mode(bool value);
  public:

  // optional bool enable_receiver_log = 3 [default = false];
  bool has_enable_receiver_log() const;
  private:
  bool _internal_has_enable_receiver_log() const;
  public:
  void clear_enable_receiver_log();
  bool enable_receiver_log() const;
  void set_enable_receiver_log(bool value);
  private:
  bool _internal_enable_receiver_log() const;
  void _internal_set_enable_receiver_log(bool value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.drivers.canbus.SensorCanbusConf)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::apollo::drivers::canbus::CANCardParameter* can_card_parameter_;
  bool enable_debug_mode_;
  bool enable_receiver_log_;
  friend struct ::TableStruct_modules_2fdrivers_2fcanbus_2fproto_2fsensor_5fcanbus_5fconf_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// SensorCanbusConf

// optional .apollo.drivers.canbus.CANCardParameter can_card_parameter = 1;
inline bool SensorCanbusConf::_internal_has_can_card_parameter() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || can_card_parameter_ != nullptr);
  return value;
}
inline bool SensorCanbusConf::has_can_card_parameter() const {
  return _internal_has_can_card_parameter();
}
inline const ::apollo::drivers::canbus::CANCardParameter& SensorCanbusConf::_internal_can_card_parameter() const {
  const ::apollo::drivers::canbus::CANCardParameter* p = can_card_parameter_;
  return p != nullptr ? *p : *reinterpret_cast<const ::apollo::drivers::canbus::CANCardParameter*>(
      &::apollo::drivers::canbus::_CANCardParameter_default_instance_);
}
inline const ::apollo::drivers::canbus::CANCardParameter& SensorCanbusConf::can_card_parameter() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.canbus.SensorCanbusConf.can_card_parameter)
  return _internal_can_card_parameter();
}
inline ::apollo::drivers::canbus::CANCardParameter* SensorCanbusConf::release_can_card_parameter() {
  // @@protoc_insertion_point(field_release:apollo.drivers.canbus.SensorCanbusConf.can_card_parameter)
  _has_bits_[0] &= ~0x00000001u;
  ::apollo::drivers::canbus::CANCardParameter* temp = can_card_parameter_;
  can_card_parameter_ = nullptr;
  return temp;
}
inline ::apollo::drivers::canbus::CANCardParameter* SensorCanbusConf::_internal_mutable_can_card_parameter() {
  _has_bits_[0] |= 0x00000001u;
  if (can_card_parameter_ == nullptr) {
    auto* p = CreateMaybeMessage<::apollo::drivers::canbus::CANCardParameter>(GetArenaNoVirtual());
    can_card_parameter_ = p;
  }
  return can_card_parameter_;
}
inline ::apollo::drivers::canbus::CANCardParameter* SensorCanbusConf::mutable_can_card_parameter() {
  // @@protoc_insertion_point(field_mutable:apollo.drivers.canbus.SensorCanbusConf.can_card_parameter)
  return _internal_mutable_can_card_parameter();
}
inline void SensorCanbusConf::set_allocated_can_card_parameter(::apollo::drivers::canbus::CANCardParameter* can_card_parameter) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(can_card_parameter_);
  }
  if (can_card_parameter) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      can_card_parameter = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, can_card_parameter, submessage_arena);
    }
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  can_card_parameter_ = can_card_parameter;
  // @@protoc_insertion_point(field_set_allocated:apollo.drivers.canbus.SensorCanbusConf.can_card_parameter)
}

// optional bool enable_debug_mode = 2 [default = false];
inline bool SensorCanbusConf::_internal_has_enable_debug_mode() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool SensorCanbusConf::has_enable_debug_mode() const {
  return _internal_has_enable_debug_mode();
}
inline void SensorCanbusConf::clear_enable_debug_mode() {
  enable_debug_mode_ = false;
  _has_bits_[0] &= ~0x00000002u;
}
inline bool SensorCanbusConf::_internal_enable_debug_mode() const {
  return enable_debug_mode_;
}
inline bool SensorCanbusConf::enable_debug_mode() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.canbus.SensorCanbusConf.enable_debug_mode)
  return _internal_enable_debug_mode();
}
inline void SensorCanbusConf::_internal_set_enable_debug_mode(bool value) {
  _has_bits_[0] |= 0x00000002u;
  enable_debug_mode_ = value;
}
inline void SensorCanbusConf::set_enable_debug_mode(bool value) {
  _internal_set_enable_debug_mode(value);
  // @@protoc_insertion_point(field_set:apollo.drivers.canbus.SensorCanbusConf.enable_debug_mode)
}

// optional bool enable_receiver_log = 3 [default = false];
inline bool SensorCanbusConf::_internal_has_enable_receiver_log() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool SensorCanbusConf::has_enable_receiver_log() const {
  return _internal_has_enable_receiver_log();
}
inline void SensorCanbusConf::clear_enable_receiver_log() {
  enable_receiver_log_ = false;
  _has_bits_[0] &= ~0x00000004u;
}
inline bool SensorCanbusConf::_internal_enable_receiver_log() const {
  return enable_receiver_log_;
}
inline bool SensorCanbusConf::enable_receiver_log() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.canbus.SensorCanbusConf.enable_receiver_log)
  return _internal_enable_receiver_log();
}
inline void SensorCanbusConf::_internal_set_enable_receiver_log(bool value) {
  _has_bits_[0] |= 0x00000004u;
  enable_receiver_log_ = value;
}
inline void SensorCanbusConf::set_enable_receiver_log(bool value) {
  _internal_set_enable_receiver_log(value);
  // @@protoc_insertion_point(field_set:apollo.drivers.canbus.SensorCanbusConf.enable_receiver_log)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace canbus
}  // namespace drivers
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2fdrivers_2fcanbus_2fproto_2fsensor_5fcanbus_5fconf_2eproto
