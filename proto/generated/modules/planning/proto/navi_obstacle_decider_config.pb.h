// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/planning/proto/navi_obstacle_decider_config.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2fplanning_2fproto_2fnavi_5fobstacle_5fdecider_5fconfig_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2fplanning_2fproto_2fnavi_5fobstacle_5fdecider_5fconfig_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_modules_2fplanning_2fproto_2fnavi_5fobstacle_5fdecider_5fconfig_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2fplanning_2fproto_2fnavi_5fobstacle_5fdecider_5fconfig_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fplanning_2fproto_2fnavi_5fobstacle_5fdecider_5fconfig_2eproto;
namespace apollo {
namespace planning {
class NaviObstacleDeciderConfig;
class NaviObstacleDeciderConfigDefaultTypeInternal;
extern NaviObstacleDeciderConfigDefaultTypeInternal _NaviObstacleDeciderConfig_default_instance_;
}  // namespace planning
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::planning::NaviObstacleDeciderConfig* Arena::CreateMaybeMessage<::apollo::planning::NaviObstacleDeciderConfig>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace planning {

// ===================================================================

class NaviObstacleDeciderConfig :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.planning.NaviObstacleDeciderConfig) */ {
 public:
  NaviObstacleDeciderConfig();
  virtual ~NaviObstacleDeciderConfig();

  NaviObstacleDeciderConfig(const NaviObstacleDeciderConfig& from);
  NaviObstacleDeciderConfig(NaviObstacleDeciderConfig&& from) noexcept
    : NaviObstacleDeciderConfig() {
    *this = ::std::move(from);
  }

  inline NaviObstacleDeciderConfig& operator=(const NaviObstacleDeciderConfig& from) {
    CopyFrom(from);
    return *this;
  }
  inline NaviObstacleDeciderConfig& operator=(NaviObstacleDeciderConfig&& from) noexcept {
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
  static const NaviObstacleDeciderConfig& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const NaviObstacleDeciderConfig* internal_default_instance() {
    return reinterpret_cast<const NaviObstacleDeciderConfig*>(
               &_NaviObstacleDeciderConfig_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(NaviObstacleDeciderConfig& a, NaviObstacleDeciderConfig& b) {
    a.Swap(&b);
  }
  inline void Swap(NaviObstacleDeciderConfig* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline NaviObstacleDeciderConfig* New() const final {
    return CreateMaybeMessage<NaviObstacleDeciderConfig>(nullptr);
  }

  NaviObstacleDeciderConfig* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<NaviObstacleDeciderConfig>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const NaviObstacleDeciderConfig& from);
  void MergeFrom(const NaviObstacleDeciderConfig& from);
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
  void InternalSwap(NaviObstacleDeciderConfig* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.planning.NaviObstacleDeciderConfig";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2fplanning_2fproto_2fnavi_5fobstacle_5fdecider_5fconfig_2eproto);
    return ::descriptor_table_modules_2fplanning_2fproto_2fnavi_5fobstacle_5fdecider_5fconfig_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kMinNudgeDistanceFieldNumber = 1,
    kMaxNudgeDistanceFieldNumber = 2,
    kMaxAllowNudgeSpeedFieldNumber = 3,
    kSafeDistanceFieldNumber = 4,
    kNudgeAllowToleranceFieldNumber = 5,
    kJudgeDisCoeffFieldNumber = 7,
    kBasisDisValueFieldNumber = 8,
    kCyclesNumberFieldNumber = 6,
    kMaxKeepNudgeCyclesFieldNumber = 11,
    kLateralVelocityValueFieldNumber = 9,
    kSpeedDeciderDetectRangeFieldNumber = 10,
  };
  // optional double min_nudge_distance = 1 [default = 0.2];
  bool has_min_nudge_distance() const;
  private:
  bool _internal_has_min_nudge_distance() const;
  public:
  void clear_min_nudge_distance();
  double min_nudge_distance() const;
  void set_min_nudge_distance(double value);
  private:
  double _internal_min_nudge_distance() const;
  void _internal_set_min_nudge_distance(double value);
  public:

  // optional double max_nudge_distance = 2 [default = 1.2];
  bool has_max_nudge_distance() const;
  private:
  bool _internal_has_max_nudge_distance() const;
  public:
  void clear_max_nudge_distance();
  double max_nudge_distance() const;
  void set_max_nudge_distance(double value);
  private:
  double _internal_max_nudge_distance() const;
  void _internal_set_max_nudge_distance(double value);
  public:

  // optional double max_allow_nudge_speed = 3 [default = 16.667];
  bool has_max_allow_nudge_speed() const;
  private:
  bool _internal_has_max_allow_nudge_speed() const;
  public:
  void clear_max_allow_nudge_speed();
  double max_allow_nudge_speed() const;
  void set_max_allow_nudge_speed(double value);
  private:
  double _internal_max_allow_nudge_speed() const;
  void _internal_set_max_allow_nudge_speed(double value);
  public:

  // optional double safe_distance = 4 [default = 0.2];
  bool has_safe_distance() const;
  private:
  bool _internal_has_safe_distance() const;
  public:
  void clear_safe_distance();
  double safe_distance() const;
  void set_safe_distance(double value);
  private:
  double _internal_safe_distance() const;
  void _internal_set_safe_distance(double value);
  public:

  // optional double nudge_allow_tolerance = 5 [default = 0.05];
  bool has_nudge_allow_tolerance() const;
  private:
  bool _internal_has_nudge_allow_tolerance() const;
  public:
  void clear_nudge_allow_tolerance();
  double nudge_allow_tolerance() const;
  void set_nudge_allow_tolerance(double value);
  private:
  double _internal_nudge_allow_tolerance() const;
  void _internal_set_nudge_allow_tolerance(double value);
  public:

  // optional double judge_dis_coeff = 7 [default = 2];
  bool has_judge_dis_coeff() const;
  private:
  bool _internal_has_judge_dis_coeff() const;
  public:
  void clear_judge_dis_coeff();
  double judge_dis_coeff() const;
  void set_judge_dis_coeff(double value);
  private:
  double _internal_judge_dis_coeff() const;
  void _internal_set_judge_dis_coeff(double value);
  public:

  // optional double basis_dis_value = 8 [default = 30];
  bool has_basis_dis_value() const;
  private:
  bool _internal_has_basis_dis_value() const;
  public:
  void clear_basis_dis_value();
  double basis_dis_value() const;
  void set_basis_dis_value(double value);
  private:
  double _internal_basis_dis_value() const;
  void _internal_set_basis_dis_value(double value);
  public:

  // optional uint32 cycles_number = 6 [default = 3];
  bool has_cycles_number() const;
  private:
  bool _internal_has_cycles_number() const;
  public:
  void clear_cycles_number();
  ::PROTOBUF_NAMESPACE_ID::uint32 cycles_number() const;
  void set_cycles_number(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_cycles_number() const;
  void _internal_set_cycles_number(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 max_keep_nudge_cycles = 11 [default = 100];
  bool has_max_keep_nudge_cycles() const;
  private:
  bool _internal_has_max_keep_nudge_cycles() const;
  public:
  void clear_max_keep_nudge_cycles();
  ::PROTOBUF_NAMESPACE_ID::uint32 max_keep_nudge_cycles() const;
  void set_max_keep_nudge_cycles(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_max_keep_nudge_cycles() const;
  void _internal_set_max_keep_nudge_cycles(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional double lateral_velocity_value = 9 [default = 0.5];
  bool has_lateral_velocity_value() const;
  private:
  bool _internal_has_lateral_velocity_value() const;
  public:
  void clear_lateral_velocity_value();
  double lateral_velocity_value() const;
  void set_lateral_velocity_value(double value);
  private:
  double _internal_lateral_velocity_value() const;
  void _internal_set_lateral_velocity_value(double value);
  public:

  // optional double speed_decider_detect_range = 10 [default = 1];
  bool has_speed_decider_detect_range() const;
  private:
  bool _internal_has_speed_decider_detect_range() const;
  public:
  void clear_speed_decider_detect_range();
  double speed_decider_detect_range() const;
  void set_speed_decider_detect_range(double value);
  private:
  double _internal_speed_decider_detect_range() const;
  void _internal_set_speed_decider_detect_range(double value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.planning.NaviObstacleDeciderConfig)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  double min_nudge_distance_;
  double max_nudge_distance_;
  double max_allow_nudge_speed_;
  double safe_distance_;
  double nudge_allow_tolerance_;
  double judge_dis_coeff_;
  double basis_dis_value_;
  ::PROTOBUF_NAMESPACE_ID::uint32 cycles_number_;
  ::PROTOBUF_NAMESPACE_ID::uint32 max_keep_nudge_cycles_;
  double lateral_velocity_value_;
  double speed_decider_detect_range_;
  friend struct ::TableStruct_modules_2fplanning_2fproto_2fnavi_5fobstacle_5fdecider_5fconfig_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// NaviObstacleDeciderConfig

// optional double min_nudge_distance = 1 [default = 0.2];
inline bool NaviObstacleDeciderConfig::_internal_has_min_nudge_distance() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool NaviObstacleDeciderConfig::has_min_nudge_distance() const {
  return _internal_has_min_nudge_distance();
}
inline void NaviObstacleDeciderConfig::clear_min_nudge_distance() {
  min_nudge_distance_ = 0.2;
  _has_bits_[0] &= ~0x00000001u;
}
inline double NaviObstacleDeciderConfig::_internal_min_nudge_distance() const {
  return min_nudge_distance_;
}
inline double NaviObstacleDeciderConfig::min_nudge_distance() const {
  // @@protoc_insertion_point(field_get:apollo.planning.NaviObstacleDeciderConfig.min_nudge_distance)
  return _internal_min_nudge_distance();
}
inline void NaviObstacleDeciderConfig::_internal_set_min_nudge_distance(double value) {
  _has_bits_[0] |= 0x00000001u;
  min_nudge_distance_ = value;
}
inline void NaviObstacleDeciderConfig::set_min_nudge_distance(double value) {
  _internal_set_min_nudge_distance(value);
  // @@protoc_insertion_point(field_set:apollo.planning.NaviObstacleDeciderConfig.min_nudge_distance)
}

// optional double max_nudge_distance = 2 [default = 1.2];
inline bool NaviObstacleDeciderConfig::_internal_has_max_nudge_distance() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool NaviObstacleDeciderConfig::has_max_nudge_distance() const {
  return _internal_has_max_nudge_distance();
}
inline void NaviObstacleDeciderConfig::clear_max_nudge_distance() {
  max_nudge_distance_ = 1.2;
  _has_bits_[0] &= ~0x00000002u;
}
inline double NaviObstacleDeciderConfig::_internal_max_nudge_distance() const {
  return max_nudge_distance_;
}
inline double NaviObstacleDeciderConfig::max_nudge_distance() const {
  // @@protoc_insertion_point(field_get:apollo.planning.NaviObstacleDeciderConfig.max_nudge_distance)
  return _internal_max_nudge_distance();
}
inline void NaviObstacleDeciderConfig::_internal_set_max_nudge_distance(double value) {
  _has_bits_[0] |= 0x00000002u;
  max_nudge_distance_ = value;
}
inline void NaviObstacleDeciderConfig::set_max_nudge_distance(double value) {
  _internal_set_max_nudge_distance(value);
  // @@protoc_insertion_point(field_set:apollo.planning.NaviObstacleDeciderConfig.max_nudge_distance)
}

// optional double max_allow_nudge_speed = 3 [default = 16.667];
inline bool NaviObstacleDeciderConfig::_internal_has_max_allow_nudge_speed() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool NaviObstacleDeciderConfig::has_max_allow_nudge_speed() const {
  return _internal_has_max_allow_nudge_speed();
}
inline void NaviObstacleDeciderConfig::clear_max_allow_nudge_speed() {
  max_allow_nudge_speed_ = 16.667;
  _has_bits_[0] &= ~0x00000004u;
}
inline double NaviObstacleDeciderConfig::_internal_max_allow_nudge_speed() const {
  return max_allow_nudge_speed_;
}
inline double NaviObstacleDeciderConfig::max_allow_nudge_speed() const {
  // @@protoc_insertion_point(field_get:apollo.planning.NaviObstacleDeciderConfig.max_allow_nudge_speed)
  return _internal_max_allow_nudge_speed();
}
inline void NaviObstacleDeciderConfig::_internal_set_max_allow_nudge_speed(double value) {
  _has_bits_[0] |= 0x00000004u;
  max_allow_nudge_speed_ = value;
}
inline void NaviObstacleDeciderConfig::set_max_allow_nudge_speed(double value) {
  _internal_set_max_allow_nudge_speed(value);
  // @@protoc_insertion_point(field_set:apollo.planning.NaviObstacleDeciderConfig.max_allow_nudge_speed)
}

// optional double safe_distance = 4 [default = 0.2];
inline bool NaviObstacleDeciderConfig::_internal_has_safe_distance() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool NaviObstacleDeciderConfig::has_safe_distance() const {
  return _internal_has_safe_distance();
}
inline void NaviObstacleDeciderConfig::clear_safe_distance() {
  safe_distance_ = 0.2;
  _has_bits_[0] &= ~0x00000008u;
}
inline double NaviObstacleDeciderConfig::_internal_safe_distance() const {
  return safe_distance_;
}
inline double NaviObstacleDeciderConfig::safe_distance() const {
  // @@protoc_insertion_point(field_get:apollo.planning.NaviObstacleDeciderConfig.safe_distance)
  return _internal_safe_distance();
}
inline void NaviObstacleDeciderConfig::_internal_set_safe_distance(double value) {
  _has_bits_[0] |= 0x00000008u;
  safe_distance_ = value;
}
inline void NaviObstacleDeciderConfig::set_safe_distance(double value) {
  _internal_set_safe_distance(value);
  // @@protoc_insertion_point(field_set:apollo.planning.NaviObstacleDeciderConfig.safe_distance)
}

// optional double nudge_allow_tolerance = 5 [default = 0.05];
inline bool NaviObstacleDeciderConfig::_internal_has_nudge_allow_tolerance() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool NaviObstacleDeciderConfig::has_nudge_allow_tolerance() const {
  return _internal_has_nudge_allow_tolerance();
}
inline void NaviObstacleDeciderConfig::clear_nudge_allow_tolerance() {
  nudge_allow_tolerance_ = 0.05;
  _has_bits_[0] &= ~0x00000010u;
}
inline double NaviObstacleDeciderConfig::_internal_nudge_allow_tolerance() const {
  return nudge_allow_tolerance_;
}
inline double NaviObstacleDeciderConfig::nudge_allow_tolerance() const {
  // @@protoc_insertion_point(field_get:apollo.planning.NaviObstacleDeciderConfig.nudge_allow_tolerance)
  return _internal_nudge_allow_tolerance();
}
inline void NaviObstacleDeciderConfig::_internal_set_nudge_allow_tolerance(double value) {
  _has_bits_[0] |= 0x00000010u;
  nudge_allow_tolerance_ = value;
}
inline void NaviObstacleDeciderConfig::set_nudge_allow_tolerance(double value) {
  _internal_set_nudge_allow_tolerance(value);
  // @@protoc_insertion_point(field_set:apollo.planning.NaviObstacleDeciderConfig.nudge_allow_tolerance)
}

// optional uint32 cycles_number = 6 [default = 3];
inline bool NaviObstacleDeciderConfig::_internal_has_cycles_number() const {
  bool value = (_has_bits_[0] & 0x00000080u) != 0;
  return value;
}
inline bool NaviObstacleDeciderConfig::has_cycles_number() const {
  return _internal_has_cycles_number();
}
inline void NaviObstacleDeciderConfig::clear_cycles_number() {
  cycles_number_ = 3u;
  _has_bits_[0] &= ~0x00000080u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 NaviObstacleDeciderConfig::_internal_cycles_number() const {
  return cycles_number_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 NaviObstacleDeciderConfig::cycles_number() const {
  // @@protoc_insertion_point(field_get:apollo.planning.NaviObstacleDeciderConfig.cycles_number)
  return _internal_cycles_number();
}
inline void NaviObstacleDeciderConfig::_internal_set_cycles_number(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000080u;
  cycles_number_ = value;
}
inline void NaviObstacleDeciderConfig::set_cycles_number(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_cycles_number(value);
  // @@protoc_insertion_point(field_set:apollo.planning.NaviObstacleDeciderConfig.cycles_number)
}

// optional double judge_dis_coeff = 7 [default = 2];
inline bool NaviObstacleDeciderConfig::_internal_has_judge_dis_coeff() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool NaviObstacleDeciderConfig::has_judge_dis_coeff() const {
  return _internal_has_judge_dis_coeff();
}
inline void NaviObstacleDeciderConfig::clear_judge_dis_coeff() {
  judge_dis_coeff_ = 2;
  _has_bits_[0] &= ~0x00000020u;
}
inline double NaviObstacleDeciderConfig::_internal_judge_dis_coeff() const {
  return judge_dis_coeff_;
}
inline double NaviObstacleDeciderConfig::judge_dis_coeff() const {
  // @@protoc_insertion_point(field_get:apollo.planning.NaviObstacleDeciderConfig.judge_dis_coeff)
  return _internal_judge_dis_coeff();
}
inline void NaviObstacleDeciderConfig::_internal_set_judge_dis_coeff(double value) {
  _has_bits_[0] |= 0x00000020u;
  judge_dis_coeff_ = value;
}
inline void NaviObstacleDeciderConfig::set_judge_dis_coeff(double value) {
  _internal_set_judge_dis_coeff(value);
  // @@protoc_insertion_point(field_set:apollo.planning.NaviObstacleDeciderConfig.judge_dis_coeff)
}

// optional double basis_dis_value = 8 [default = 30];
inline bool NaviObstacleDeciderConfig::_internal_has_basis_dis_value() const {
  bool value = (_has_bits_[0] & 0x00000040u) != 0;
  return value;
}
inline bool NaviObstacleDeciderConfig::has_basis_dis_value() const {
  return _internal_has_basis_dis_value();
}
inline void NaviObstacleDeciderConfig::clear_basis_dis_value() {
  basis_dis_value_ = 30;
  _has_bits_[0] &= ~0x00000040u;
}
inline double NaviObstacleDeciderConfig::_internal_basis_dis_value() const {
  return basis_dis_value_;
}
inline double NaviObstacleDeciderConfig::basis_dis_value() const {
  // @@protoc_insertion_point(field_get:apollo.planning.NaviObstacleDeciderConfig.basis_dis_value)
  return _internal_basis_dis_value();
}
inline void NaviObstacleDeciderConfig::_internal_set_basis_dis_value(double value) {
  _has_bits_[0] |= 0x00000040u;
  basis_dis_value_ = value;
}
inline void NaviObstacleDeciderConfig::set_basis_dis_value(double value) {
  _internal_set_basis_dis_value(value);
  // @@protoc_insertion_point(field_set:apollo.planning.NaviObstacleDeciderConfig.basis_dis_value)
}

// optional double lateral_velocity_value = 9 [default = 0.5];
inline bool NaviObstacleDeciderConfig::_internal_has_lateral_velocity_value() const {
  bool value = (_has_bits_[0] & 0x00000200u) != 0;
  return value;
}
inline bool NaviObstacleDeciderConfig::has_lateral_velocity_value() const {
  return _internal_has_lateral_velocity_value();
}
inline void NaviObstacleDeciderConfig::clear_lateral_velocity_value() {
  lateral_velocity_value_ = 0.5;
  _has_bits_[0] &= ~0x00000200u;
}
inline double NaviObstacleDeciderConfig::_internal_lateral_velocity_value() const {
  return lateral_velocity_value_;
}
inline double NaviObstacleDeciderConfig::lateral_velocity_value() const {
  // @@protoc_insertion_point(field_get:apollo.planning.NaviObstacleDeciderConfig.lateral_velocity_value)
  return _internal_lateral_velocity_value();
}
inline void NaviObstacleDeciderConfig::_internal_set_lateral_velocity_value(double value) {
  _has_bits_[0] |= 0x00000200u;
  lateral_velocity_value_ = value;
}
inline void NaviObstacleDeciderConfig::set_lateral_velocity_value(double value) {
  _internal_set_lateral_velocity_value(value);
  // @@protoc_insertion_point(field_set:apollo.planning.NaviObstacleDeciderConfig.lateral_velocity_value)
}

// optional double speed_decider_detect_range = 10 [default = 1];
inline bool NaviObstacleDeciderConfig::_internal_has_speed_decider_detect_range() const {
  bool value = (_has_bits_[0] & 0x00000400u) != 0;
  return value;
}
inline bool NaviObstacleDeciderConfig::has_speed_decider_detect_range() const {
  return _internal_has_speed_decider_detect_range();
}
inline void NaviObstacleDeciderConfig::clear_speed_decider_detect_range() {
  speed_decider_detect_range_ = 1;
  _has_bits_[0] &= ~0x00000400u;
}
inline double NaviObstacleDeciderConfig::_internal_speed_decider_detect_range() const {
  return speed_decider_detect_range_;
}
inline double NaviObstacleDeciderConfig::speed_decider_detect_range() const {
  // @@protoc_insertion_point(field_get:apollo.planning.NaviObstacleDeciderConfig.speed_decider_detect_range)
  return _internal_speed_decider_detect_range();
}
inline void NaviObstacleDeciderConfig::_internal_set_speed_decider_detect_range(double value) {
  _has_bits_[0] |= 0x00000400u;
  speed_decider_detect_range_ = value;
}
inline void NaviObstacleDeciderConfig::set_speed_decider_detect_range(double value) {
  _internal_set_speed_decider_detect_range(value);
  // @@protoc_insertion_point(field_set:apollo.planning.NaviObstacleDeciderConfig.speed_decider_detect_range)
}

// optional uint32 max_keep_nudge_cycles = 11 [default = 100];
inline bool NaviObstacleDeciderConfig::_internal_has_max_keep_nudge_cycles() const {
  bool value = (_has_bits_[0] & 0x00000100u) != 0;
  return value;
}
inline bool NaviObstacleDeciderConfig::has_max_keep_nudge_cycles() const {
  return _internal_has_max_keep_nudge_cycles();
}
inline void NaviObstacleDeciderConfig::clear_max_keep_nudge_cycles() {
  max_keep_nudge_cycles_ = 100u;
  _has_bits_[0] &= ~0x00000100u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 NaviObstacleDeciderConfig::_internal_max_keep_nudge_cycles() const {
  return max_keep_nudge_cycles_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 NaviObstacleDeciderConfig::max_keep_nudge_cycles() const {
  // @@protoc_insertion_point(field_get:apollo.planning.NaviObstacleDeciderConfig.max_keep_nudge_cycles)
  return _internal_max_keep_nudge_cycles();
}
inline void NaviObstacleDeciderConfig::_internal_set_max_keep_nudge_cycles(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000100u;
  max_keep_nudge_cycles_ = value;
}
inline void NaviObstacleDeciderConfig::set_max_keep_nudge_cycles(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_max_keep_nudge_cycles(value);
  // @@protoc_insertion_point(field_set:apollo.planning.NaviObstacleDeciderConfig.max_keep_nudge_cycles)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace planning
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2fplanning_2fproto_2fnavi_5fobstacle_5fdecider_5fconfig_2eproto
