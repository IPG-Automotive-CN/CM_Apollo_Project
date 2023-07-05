// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/planning/proto/piecewise_jerk_speed_config.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2fplanning_2fproto_2fpiecewise_5fjerk_5fspeed_5fconfig_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2fplanning_2fproto_2fpiecewise_5fjerk_5fspeed_5fconfig_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_modules_2fplanning_2fproto_2fpiecewise_5fjerk_5fspeed_5fconfig_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2fplanning_2fproto_2fpiecewise_5fjerk_5fspeed_5fconfig_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fplanning_2fproto_2fpiecewise_5fjerk_5fspeed_5fconfig_2eproto;
namespace apollo {
namespace planning {
class PiecewiseJerkSpeedConfig;
class PiecewiseJerkSpeedConfigDefaultTypeInternal;
extern PiecewiseJerkSpeedConfigDefaultTypeInternal _PiecewiseJerkSpeedConfig_default_instance_;
}  // namespace planning
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::planning::PiecewiseJerkSpeedConfig* Arena::CreateMaybeMessage<::apollo::planning::PiecewiseJerkSpeedConfig>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace planning {

// ===================================================================

class PiecewiseJerkSpeedConfig :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.planning.PiecewiseJerkSpeedConfig) */ {
 public:
  PiecewiseJerkSpeedConfig();
  virtual ~PiecewiseJerkSpeedConfig();

  PiecewiseJerkSpeedConfig(const PiecewiseJerkSpeedConfig& from);
  PiecewiseJerkSpeedConfig(PiecewiseJerkSpeedConfig&& from) noexcept
    : PiecewiseJerkSpeedConfig() {
    *this = ::std::move(from);
  }

  inline PiecewiseJerkSpeedConfig& operator=(const PiecewiseJerkSpeedConfig& from) {
    CopyFrom(from);
    return *this;
  }
  inline PiecewiseJerkSpeedConfig& operator=(PiecewiseJerkSpeedConfig&& from) noexcept {
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
  static const PiecewiseJerkSpeedConfig& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const PiecewiseJerkSpeedConfig* internal_default_instance() {
    return reinterpret_cast<const PiecewiseJerkSpeedConfig*>(
               &_PiecewiseJerkSpeedConfig_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(PiecewiseJerkSpeedConfig& a, PiecewiseJerkSpeedConfig& b) {
    a.Swap(&b);
  }
  inline void Swap(PiecewiseJerkSpeedConfig* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline PiecewiseJerkSpeedConfig* New() const final {
    return CreateMaybeMessage<PiecewiseJerkSpeedConfig>(nullptr);
  }

  PiecewiseJerkSpeedConfig* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<PiecewiseJerkSpeedConfig>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const PiecewiseJerkSpeedConfig& from);
  void MergeFrom(const PiecewiseJerkSpeedConfig& from);
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
  void InternalSwap(PiecewiseJerkSpeedConfig* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.planning.PiecewiseJerkSpeedConfig";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2fplanning_2fproto_2fpiecewise_5fjerk_5fspeed_5fconfig_2eproto);
    return ::descriptor_table_modules_2fplanning_2fproto_2fpiecewise_5fjerk_5fspeed_5fconfig_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kAccWeightFieldNumber = 1,
    kJerkWeightFieldNumber = 2,
    kKappaPenaltyWeightFieldNumber = 3,
    kRefSWeightFieldNumber = 4,
    kRefVWeightFieldNumber = 5,
  };
  // optional double acc_weight = 1 [default = 1];
  bool has_acc_weight() const;
  private:
  bool _internal_has_acc_weight() const;
  public:
  void clear_acc_weight();
  double acc_weight() const;
  void set_acc_weight(double value);
  private:
  double _internal_acc_weight() const;
  void _internal_set_acc_weight(double value);
  public:

  // optional double jerk_weight = 2 [default = 10];
  bool has_jerk_weight() const;
  private:
  bool _internal_has_jerk_weight() const;
  public:
  void clear_jerk_weight();
  double jerk_weight() const;
  void set_jerk_weight(double value);
  private:
  double _internal_jerk_weight() const;
  void _internal_set_jerk_weight(double value);
  public:

  // optional double kappa_penalty_weight = 3 [default = 1000];
  bool has_kappa_penalty_weight() const;
  private:
  bool _internal_has_kappa_penalty_weight() const;
  public:
  void clear_kappa_penalty_weight();
  double kappa_penalty_weight() const;
  void set_kappa_penalty_weight(double value);
  private:
  double _internal_kappa_penalty_weight() const;
  void _internal_set_kappa_penalty_weight(double value);
  public:

  // optional double ref_s_weight = 4 [default = 10];
  bool has_ref_s_weight() const;
  private:
  bool _internal_has_ref_s_weight() const;
  public:
  void clear_ref_s_weight();
  double ref_s_weight() const;
  void set_ref_s_weight(double value);
  private:
  double _internal_ref_s_weight() const;
  void _internal_set_ref_s_weight(double value);
  public:

  // optional double ref_v_weight = 5 [default = 10];
  bool has_ref_v_weight() const;
  private:
  bool _internal_has_ref_v_weight() const;
  public:
  void clear_ref_v_weight();
  double ref_v_weight() const;
  void set_ref_v_weight(double value);
  private:
  double _internal_ref_v_weight() const;
  void _internal_set_ref_v_weight(double value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.planning.PiecewiseJerkSpeedConfig)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  double acc_weight_;
  double jerk_weight_;
  double kappa_penalty_weight_;
  double ref_s_weight_;
  double ref_v_weight_;
  friend struct ::TableStruct_modules_2fplanning_2fproto_2fpiecewise_5fjerk_5fspeed_5fconfig_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// PiecewiseJerkSpeedConfig

// optional double acc_weight = 1 [default = 1];
inline bool PiecewiseJerkSpeedConfig::_internal_has_acc_weight() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool PiecewiseJerkSpeedConfig::has_acc_weight() const {
  return _internal_has_acc_weight();
}
inline void PiecewiseJerkSpeedConfig::clear_acc_weight() {
  acc_weight_ = 1;
  _has_bits_[0] &= ~0x00000001u;
}
inline double PiecewiseJerkSpeedConfig::_internal_acc_weight() const {
  return acc_weight_;
}
inline double PiecewiseJerkSpeedConfig::acc_weight() const {
  // @@protoc_insertion_point(field_get:apollo.planning.PiecewiseJerkSpeedConfig.acc_weight)
  return _internal_acc_weight();
}
inline void PiecewiseJerkSpeedConfig::_internal_set_acc_weight(double value) {
  _has_bits_[0] |= 0x00000001u;
  acc_weight_ = value;
}
inline void PiecewiseJerkSpeedConfig::set_acc_weight(double value) {
  _internal_set_acc_weight(value);
  // @@protoc_insertion_point(field_set:apollo.planning.PiecewiseJerkSpeedConfig.acc_weight)
}

// optional double jerk_weight = 2 [default = 10];
inline bool PiecewiseJerkSpeedConfig::_internal_has_jerk_weight() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool PiecewiseJerkSpeedConfig::has_jerk_weight() const {
  return _internal_has_jerk_weight();
}
inline void PiecewiseJerkSpeedConfig::clear_jerk_weight() {
  jerk_weight_ = 10;
  _has_bits_[0] &= ~0x00000002u;
}
inline double PiecewiseJerkSpeedConfig::_internal_jerk_weight() const {
  return jerk_weight_;
}
inline double PiecewiseJerkSpeedConfig::jerk_weight() const {
  // @@protoc_insertion_point(field_get:apollo.planning.PiecewiseJerkSpeedConfig.jerk_weight)
  return _internal_jerk_weight();
}
inline void PiecewiseJerkSpeedConfig::_internal_set_jerk_weight(double value) {
  _has_bits_[0] |= 0x00000002u;
  jerk_weight_ = value;
}
inline void PiecewiseJerkSpeedConfig::set_jerk_weight(double value) {
  _internal_set_jerk_weight(value);
  // @@protoc_insertion_point(field_set:apollo.planning.PiecewiseJerkSpeedConfig.jerk_weight)
}

// optional double kappa_penalty_weight = 3 [default = 1000];
inline bool PiecewiseJerkSpeedConfig::_internal_has_kappa_penalty_weight() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool PiecewiseJerkSpeedConfig::has_kappa_penalty_weight() const {
  return _internal_has_kappa_penalty_weight();
}
inline void PiecewiseJerkSpeedConfig::clear_kappa_penalty_weight() {
  kappa_penalty_weight_ = 1000;
  _has_bits_[0] &= ~0x00000004u;
}
inline double PiecewiseJerkSpeedConfig::_internal_kappa_penalty_weight() const {
  return kappa_penalty_weight_;
}
inline double PiecewiseJerkSpeedConfig::kappa_penalty_weight() const {
  // @@protoc_insertion_point(field_get:apollo.planning.PiecewiseJerkSpeedConfig.kappa_penalty_weight)
  return _internal_kappa_penalty_weight();
}
inline void PiecewiseJerkSpeedConfig::_internal_set_kappa_penalty_weight(double value) {
  _has_bits_[0] |= 0x00000004u;
  kappa_penalty_weight_ = value;
}
inline void PiecewiseJerkSpeedConfig::set_kappa_penalty_weight(double value) {
  _internal_set_kappa_penalty_weight(value);
  // @@protoc_insertion_point(field_set:apollo.planning.PiecewiseJerkSpeedConfig.kappa_penalty_weight)
}

// optional double ref_s_weight = 4 [default = 10];
inline bool PiecewiseJerkSpeedConfig::_internal_has_ref_s_weight() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool PiecewiseJerkSpeedConfig::has_ref_s_weight() const {
  return _internal_has_ref_s_weight();
}
inline void PiecewiseJerkSpeedConfig::clear_ref_s_weight() {
  ref_s_weight_ = 10;
  _has_bits_[0] &= ~0x00000008u;
}
inline double PiecewiseJerkSpeedConfig::_internal_ref_s_weight() const {
  return ref_s_weight_;
}
inline double PiecewiseJerkSpeedConfig::ref_s_weight() const {
  // @@protoc_insertion_point(field_get:apollo.planning.PiecewiseJerkSpeedConfig.ref_s_weight)
  return _internal_ref_s_weight();
}
inline void PiecewiseJerkSpeedConfig::_internal_set_ref_s_weight(double value) {
  _has_bits_[0] |= 0x00000008u;
  ref_s_weight_ = value;
}
inline void PiecewiseJerkSpeedConfig::set_ref_s_weight(double value) {
  _internal_set_ref_s_weight(value);
  // @@protoc_insertion_point(field_set:apollo.planning.PiecewiseJerkSpeedConfig.ref_s_weight)
}

// optional double ref_v_weight = 5 [default = 10];
inline bool PiecewiseJerkSpeedConfig::_internal_has_ref_v_weight() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool PiecewiseJerkSpeedConfig::has_ref_v_weight() const {
  return _internal_has_ref_v_weight();
}
inline void PiecewiseJerkSpeedConfig::clear_ref_v_weight() {
  ref_v_weight_ = 10;
  _has_bits_[0] &= ~0x00000010u;
}
inline double PiecewiseJerkSpeedConfig::_internal_ref_v_weight() const {
  return ref_v_weight_;
}
inline double PiecewiseJerkSpeedConfig::ref_v_weight() const {
  // @@protoc_insertion_point(field_get:apollo.planning.PiecewiseJerkSpeedConfig.ref_v_weight)
  return _internal_ref_v_weight();
}
inline void PiecewiseJerkSpeedConfig::_internal_set_ref_v_weight(double value) {
  _has_bits_[0] |= 0x00000010u;
  ref_v_weight_ = value;
}
inline void PiecewiseJerkSpeedConfig::set_ref_v_weight(double value) {
  _internal_set_ref_v_weight(value);
  // @@protoc_insertion_point(field_set:apollo.planning.PiecewiseJerkSpeedConfig.ref_v_weight)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace planning
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2fplanning_2fproto_2fpiecewise_5fjerk_5fspeed_5fconfig_2eproto