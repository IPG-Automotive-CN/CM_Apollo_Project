// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/planning/proto/lane_change_decider_config.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2fplanning_2fproto_2flane_5fchange_5fdecider_5fconfig_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2fplanning_2fproto_2flane_5fchange_5fdecider_5fconfig_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_modules_2fplanning_2fproto_2flane_5fchange_5fdecider_5fconfig_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2fplanning_2fproto_2flane_5fchange_5fdecider_5fconfig_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fplanning_2fproto_2flane_5fchange_5fdecider_5fconfig_2eproto;
namespace apollo {
namespace planning {
class LaneChangeDeciderConfig;
class LaneChangeDeciderConfigDefaultTypeInternal;
extern LaneChangeDeciderConfigDefaultTypeInternal _LaneChangeDeciderConfig_default_instance_;
}  // namespace planning
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::planning::LaneChangeDeciderConfig* Arena::CreateMaybeMessage<::apollo::planning::LaneChangeDeciderConfig>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace planning {

// ===================================================================

class LaneChangeDeciderConfig :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.planning.LaneChangeDeciderConfig) */ {
 public:
  LaneChangeDeciderConfig();
  virtual ~LaneChangeDeciderConfig();

  LaneChangeDeciderConfig(const LaneChangeDeciderConfig& from);
  LaneChangeDeciderConfig(LaneChangeDeciderConfig&& from) noexcept
    : LaneChangeDeciderConfig() {
    *this = ::std::move(from);
  }

  inline LaneChangeDeciderConfig& operator=(const LaneChangeDeciderConfig& from) {
    CopyFrom(from);
    return *this;
  }
  inline LaneChangeDeciderConfig& operator=(LaneChangeDeciderConfig&& from) noexcept {
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
  static const LaneChangeDeciderConfig& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const LaneChangeDeciderConfig* internal_default_instance() {
    return reinterpret_cast<const LaneChangeDeciderConfig*>(
               &_LaneChangeDeciderConfig_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(LaneChangeDeciderConfig& a, LaneChangeDeciderConfig& b) {
    a.Swap(&b);
  }
  inline void Swap(LaneChangeDeciderConfig* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline LaneChangeDeciderConfig* New() const final {
    return CreateMaybeMessage<LaneChangeDeciderConfig>(nullptr);
  }

  LaneChangeDeciderConfig* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<LaneChangeDeciderConfig>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const LaneChangeDeciderConfig& from);
  void MergeFrom(const LaneChangeDeciderConfig& from);
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
  void InternalSwap(LaneChangeDeciderConfig* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.planning.LaneChangeDeciderConfig";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2fplanning_2fproto_2flane_5fchange_5fdecider_5fconfig_2eproto);
    return ::descriptor_table_modules_2fplanning_2fproto_2flane_5fchange_5fdecider_5fconfig_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kEnableLaneChangeUrgencyCheckFieldNumber = 1,
  };
  // optional bool enable_lane_change_urgency_check = 1;
  bool has_enable_lane_change_urgency_check() const;
  private:
  bool _internal_has_enable_lane_change_urgency_check() const;
  public:
  void clear_enable_lane_change_urgency_check();
  bool enable_lane_change_urgency_check() const;
  void set_enable_lane_change_urgency_check(bool value);
  private:
  bool _internal_enable_lane_change_urgency_check() const;
  void _internal_set_enable_lane_change_urgency_check(bool value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.planning.LaneChangeDeciderConfig)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  bool enable_lane_change_urgency_check_;
  friend struct ::TableStruct_modules_2fplanning_2fproto_2flane_5fchange_5fdecider_5fconfig_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// LaneChangeDeciderConfig

// optional bool enable_lane_change_urgency_check = 1;
inline bool LaneChangeDeciderConfig::_internal_has_enable_lane_change_urgency_check() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool LaneChangeDeciderConfig::has_enable_lane_change_urgency_check() const {
  return _internal_has_enable_lane_change_urgency_check();
}
inline void LaneChangeDeciderConfig::clear_enable_lane_change_urgency_check() {
  enable_lane_change_urgency_check_ = false;
  _has_bits_[0] &= ~0x00000001u;
}
inline bool LaneChangeDeciderConfig::_internal_enable_lane_change_urgency_check() const {
  return enable_lane_change_urgency_check_;
}
inline bool LaneChangeDeciderConfig::enable_lane_change_urgency_check() const {
  // @@protoc_insertion_point(field_get:apollo.planning.LaneChangeDeciderConfig.enable_lane_change_urgency_check)
  return _internal_enable_lane_change_urgency_check();
}
inline void LaneChangeDeciderConfig::_internal_set_enable_lane_change_urgency_check(bool value) {
  _has_bits_[0] |= 0x00000001u;
  enable_lane_change_urgency_check_ = value;
}
inline void LaneChangeDeciderConfig::set_enable_lane_change_urgency_check(bool value) {
  _internal_set_enable_lane_change_urgency_check(value);
  // @@protoc_insertion_point(field_set:apollo.planning.LaneChangeDeciderConfig.enable_lane_change_urgency_check)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace planning
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2fplanning_2fproto_2flane_5fchange_5fdecider_5fconfig_2eproto
