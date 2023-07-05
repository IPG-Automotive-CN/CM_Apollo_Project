// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/planning/proto/path_decider_info.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2fplanning_2fproto_2fpath_5fdecider_5finfo_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2fplanning_2fproto_2fpath_5fdecider_5finfo_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_modules_2fplanning_2fproto_2fpath_5fdecider_5finfo_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2fplanning_2fproto_2fpath_5fdecider_5finfo_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fplanning_2fproto_2fpath_5fdecider_5finfo_2eproto;
namespace apollo {
namespace planning {
class PathDeciderInfo;
class PathDeciderInfoDefaultTypeInternal;
extern PathDeciderInfoDefaultTypeInternal _PathDeciderInfo_default_instance_;
}  // namespace planning
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::planning::PathDeciderInfo* Arena::CreateMaybeMessage<::apollo::planning::PathDeciderInfo>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace planning {

// ===================================================================

class PathDeciderInfo :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.planning.PathDeciderInfo) */ {
 public:
  PathDeciderInfo();
  virtual ~PathDeciderInfo();

  PathDeciderInfo(const PathDeciderInfo& from);
  PathDeciderInfo(PathDeciderInfo&& from) noexcept
    : PathDeciderInfo() {
    *this = ::std::move(from);
  }

  inline PathDeciderInfo& operator=(const PathDeciderInfo& from) {
    CopyFrom(from);
    return *this;
  }
  inline PathDeciderInfo& operator=(PathDeciderInfo&& from) noexcept {
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
  static const PathDeciderInfo& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const PathDeciderInfo* internal_default_instance() {
    return reinterpret_cast<const PathDeciderInfo*>(
               &_PathDeciderInfo_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(PathDeciderInfo& a, PathDeciderInfo& b) {
    a.Swap(&b);
  }
  inline void Swap(PathDeciderInfo* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline PathDeciderInfo* New() const final {
    return CreateMaybeMessage<PathDeciderInfo>(nullptr);
  }

  PathDeciderInfo* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<PathDeciderInfo>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const PathDeciderInfo& from);
  void MergeFrom(const PathDeciderInfo& from);
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
  void InternalSwap(PathDeciderInfo* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.planning.PathDeciderInfo";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2fplanning_2fproto_2fpath_5fdecider_5finfo_2eproto);
    return ::descriptor_table_modules_2fplanning_2fproto_2fpath_5fdecider_5finfo_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kFrontStaticObstacleIdFieldNumber = 4,
    kFrontStaticObstacleCycleCounterFieldNumber = 1,
    kAbleToUseSelfLaneCounterFieldNumber = 2,
    kIsInPathLaneBorrowScenarioFieldNumber = 3,
    kDecidedSidePassDirectionFieldNumber = 5,
  };
  // optional string front_static_obstacle_id = 4 [default = ""];
  bool has_front_static_obstacle_id() const;
  private:
  bool _internal_has_front_static_obstacle_id() const;
  public:
  void clear_front_static_obstacle_id();
  const std::string& front_static_obstacle_id() const;
  void set_front_static_obstacle_id(const std::string& value);
  void set_front_static_obstacle_id(std::string&& value);
  void set_front_static_obstacle_id(const char* value);
  void set_front_static_obstacle_id(const char* value, size_t size);
  std::string* mutable_front_static_obstacle_id();
  std::string* release_front_static_obstacle_id();
  void set_allocated_front_static_obstacle_id(std::string* front_static_obstacle_id);
  private:
  const std::string& _internal_front_static_obstacle_id() const;
  void _internal_set_front_static_obstacle_id(const std::string& value);
  std::string* _internal_mutable_front_static_obstacle_id();
  public:

  // optional int32 front_static_obstacle_cycle_counter = 1 [default = 0];
  bool has_front_static_obstacle_cycle_counter() const;
  private:
  bool _internal_has_front_static_obstacle_cycle_counter() const;
  public:
  void clear_front_static_obstacle_cycle_counter();
  ::PROTOBUF_NAMESPACE_ID::int32 front_static_obstacle_cycle_counter() const;
  void set_front_static_obstacle_cycle_counter(::PROTOBUF_NAMESPACE_ID::int32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int32 _internal_front_static_obstacle_cycle_counter() const;
  void _internal_set_front_static_obstacle_cycle_counter(::PROTOBUF_NAMESPACE_ID::int32 value);
  public:

  // optional int32 able_to_use_self_lane_counter = 2 [default = 0];
  bool has_able_to_use_self_lane_counter() const;
  private:
  bool _internal_has_able_to_use_self_lane_counter() const;
  public:
  void clear_able_to_use_self_lane_counter();
  ::PROTOBUF_NAMESPACE_ID::int32 able_to_use_self_lane_counter() const;
  void set_able_to_use_self_lane_counter(::PROTOBUF_NAMESPACE_ID::int32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int32 _internal_able_to_use_self_lane_counter() const;
  void _internal_set_able_to_use_self_lane_counter(::PROTOBUF_NAMESPACE_ID::int32 value);
  public:

  // optional bool is_in_path_lane_borrow_scenario = 3 [default = false];
  bool has_is_in_path_lane_borrow_scenario() const;
  private:
  bool _internal_has_is_in_path_lane_borrow_scenario() const;
  public:
  void clear_is_in_path_lane_borrow_scenario();
  bool is_in_path_lane_borrow_scenario() const;
  void set_is_in_path_lane_borrow_scenario(bool value);
  private:
  bool _internal_is_in_path_lane_borrow_scenario() const;
  void _internal_set_is_in_path_lane_borrow_scenario(bool value);
  public:

  // optional int32 decided_side_pass_direction = 5 [default = 0];
  bool has_decided_side_pass_direction() const;
  private:
  bool _internal_has_decided_side_pass_direction() const;
  public:
  void clear_decided_side_pass_direction();
  ::PROTOBUF_NAMESPACE_ID::int32 decided_side_pass_direction() const;
  void set_decided_side_pass_direction(::PROTOBUF_NAMESPACE_ID::int32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int32 _internal_decided_side_pass_direction() const;
  void _internal_set_decided_side_pass_direction(::PROTOBUF_NAMESPACE_ID::int32 value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.planning.PathDeciderInfo)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr front_static_obstacle_id_;
  ::PROTOBUF_NAMESPACE_ID::int32 front_static_obstacle_cycle_counter_;
  ::PROTOBUF_NAMESPACE_ID::int32 able_to_use_self_lane_counter_;
  bool is_in_path_lane_borrow_scenario_;
  ::PROTOBUF_NAMESPACE_ID::int32 decided_side_pass_direction_;
  friend struct ::TableStruct_modules_2fplanning_2fproto_2fpath_5fdecider_5finfo_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// PathDeciderInfo

// optional int32 front_static_obstacle_cycle_counter = 1 [default = 0];
inline bool PathDeciderInfo::_internal_has_front_static_obstacle_cycle_counter() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool PathDeciderInfo::has_front_static_obstacle_cycle_counter() const {
  return _internal_has_front_static_obstacle_cycle_counter();
}
inline void PathDeciderInfo::clear_front_static_obstacle_cycle_counter() {
  front_static_obstacle_cycle_counter_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 PathDeciderInfo::_internal_front_static_obstacle_cycle_counter() const {
  return front_static_obstacle_cycle_counter_;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 PathDeciderInfo::front_static_obstacle_cycle_counter() const {
  // @@protoc_insertion_point(field_get:apollo.planning.PathDeciderInfo.front_static_obstacle_cycle_counter)
  return _internal_front_static_obstacle_cycle_counter();
}
inline void PathDeciderInfo::_internal_set_front_static_obstacle_cycle_counter(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00000002u;
  front_static_obstacle_cycle_counter_ = value;
}
inline void PathDeciderInfo::set_front_static_obstacle_cycle_counter(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _internal_set_front_static_obstacle_cycle_counter(value);
  // @@protoc_insertion_point(field_set:apollo.planning.PathDeciderInfo.front_static_obstacle_cycle_counter)
}

// optional int32 able_to_use_self_lane_counter = 2 [default = 0];
inline bool PathDeciderInfo::_internal_has_able_to_use_self_lane_counter() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool PathDeciderInfo::has_able_to_use_self_lane_counter() const {
  return _internal_has_able_to_use_self_lane_counter();
}
inline void PathDeciderInfo::clear_able_to_use_self_lane_counter() {
  able_to_use_self_lane_counter_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 PathDeciderInfo::_internal_able_to_use_self_lane_counter() const {
  return able_to_use_self_lane_counter_;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 PathDeciderInfo::able_to_use_self_lane_counter() const {
  // @@protoc_insertion_point(field_get:apollo.planning.PathDeciderInfo.able_to_use_self_lane_counter)
  return _internal_able_to_use_self_lane_counter();
}
inline void PathDeciderInfo::_internal_set_able_to_use_self_lane_counter(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00000004u;
  able_to_use_self_lane_counter_ = value;
}
inline void PathDeciderInfo::set_able_to_use_self_lane_counter(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _internal_set_able_to_use_self_lane_counter(value);
  // @@protoc_insertion_point(field_set:apollo.planning.PathDeciderInfo.able_to_use_self_lane_counter)
}

// optional bool is_in_path_lane_borrow_scenario = 3 [default = false];
inline bool PathDeciderInfo::_internal_has_is_in_path_lane_borrow_scenario() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool PathDeciderInfo::has_is_in_path_lane_borrow_scenario() const {
  return _internal_has_is_in_path_lane_borrow_scenario();
}
inline void PathDeciderInfo::clear_is_in_path_lane_borrow_scenario() {
  is_in_path_lane_borrow_scenario_ = false;
  _has_bits_[0] &= ~0x00000008u;
}
inline bool PathDeciderInfo::_internal_is_in_path_lane_borrow_scenario() const {
  return is_in_path_lane_borrow_scenario_;
}
inline bool PathDeciderInfo::is_in_path_lane_borrow_scenario() const {
  // @@protoc_insertion_point(field_get:apollo.planning.PathDeciderInfo.is_in_path_lane_borrow_scenario)
  return _internal_is_in_path_lane_borrow_scenario();
}
inline void PathDeciderInfo::_internal_set_is_in_path_lane_borrow_scenario(bool value) {
  _has_bits_[0] |= 0x00000008u;
  is_in_path_lane_borrow_scenario_ = value;
}
inline void PathDeciderInfo::set_is_in_path_lane_borrow_scenario(bool value) {
  _internal_set_is_in_path_lane_borrow_scenario(value);
  // @@protoc_insertion_point(field_set:apollo.planning.PathDeciderInfo.is_in_path_lane_borrow_scenario)
}

// optional string front_static_obstacle_id = 4 [default = ""];
inline bool PathDeciderInfo::_internal_has_front_static_obstacle_id() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool PathDeciderInfo::has_front_static_obstacle_id() const {
  return _internal_has_front_static_obstacle_id();
}
inline void PathDeciderInfo::clear_front_static_obstacle_id() {
  front_static_obstacle_id_.ClearToEmptyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  _has_bits_[0] &= ~0x00000001u;
}
inline const std::string& PathDeciderInfo::front_static_obstacle_id() const {
  // @@protoc_insertion_point(field_get:apollo.planning.PathDeciderInfo.front_static_obstacle_id)
  return _internal_front_static_obstacle_id();
}
inline void PathDeciderInfo::set_front_static_obstacle_id(const std::string& value) {
  _internal_set_front_static_obstacle_id(value);
  // @@protoc_insertion_point(field_set:apollo.planning.PathDeciderInfo.front_static_obstacle_id)
}
inline std::string* PathDeciderInfo::mutable_front_static_obstacle_id() {
  // @@protoc_insertion_point(field_mutable:apollo.planning.PathDeciderInfo.front_static_obstacle_id)
  return _internal_mutable_front_static_obstacle_id();
}
inline const std::string& PathDeciderInfo::_internal_front_static_obstacle_id() const {
  return front_static_obstacle_id_.GetNoArena();
}
inline void PathDeciderInfo::_internal_set_front_static_obstacle_id(const std::string& value) {
  _has_bits_[0] |= 0x00000001u;
  front_static_obstacle_id_.SetNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), value);
}
inline void PathDeciderInfo::set_front_static_obstacle_id(std::string&& value) {
  _has_bits_[0] |= 0x00000001u;
  front_static_obstacle_id_.SetNoArena(
    &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:apollo.planning.PathDeciderInfo.front_static_obstacle_id)
}
inline void PathDeciderInfo::set_front_static_obstacle_id(const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  _has_bits_[0] |= 0x00000001u;
  front_static_obstacle_id_.SetNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:apollo.planning.PathDeciderInfo.front_static_obstacle_id)
}
inline void PathDeciderInfo::set_front_static_obstacle_id(const char* value, size_t size) {
  _has_bits_[0] |= 0x00000001u;
  front_static_obstacle_id_.SetNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:apollo.planning.PathDeciderInfo.front_static_obstacle_id)
}
inline std::string* PathDeciderInfo::_internal_mutable_front_static_obstacle_id() {
  _has_bits_[0] |= 0x00000001u;
  return front_static_obstacle_id_.MutableNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}
inline std::string* PathDeciderInfo::release_front_static_obstacle_id() {
  // @@protoc_insertion_point(field_release:apollo.planning.PathDeciderInfo.front_static_obstacle_id)
  if (!_internal_has_front_static_obstacle_id()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000001u;
  return front_static_obstacle_id_.ReleaseNonDefaultNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}
inline void PathDeciderInfo::set_allocated_front_static_obstacle_id(std::string* front_static_obstacle_id) {
  if (front_static_obstacle_id != nullptr) {
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  front_static_obstacle_id_.SetAllocatedNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), front_static_obstacle_id);
  // @@protoc_insertion_point(field_set_allocated:apollo.planning.PathDeciderInfo.front_static_obstacle_id)
}

// optional int32 decided_side_pass_direction = 5 [default = 0];
inline bool PathDeciderInfo::_internal_has_decided_side_pass_direction() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool PathDeciderInfo::has_decided_side_pass_direction() const {
  return _internal_has_decided_side_pass_direction();
}
inline void PathDeciderInfo::clear_decided_side_pass_direction() {
  decided_side_pass_direction_ = 0;
  _has_bits_[0] &= ~0x00000010u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 PathDeciderInfo::_internal_decided_side_pass_direction() const {
  return decided_side_pass_direction_;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 PathDeciderInfo::decided_side_pass_direction() const {
  // @@protoc_insertion_point(field_get:apollo.planning.PathDeciderInfo.decided_side_pass_direction)
  return _internal_decided_side_pass_direction();
}
inline void PathDeciderInfo::_internal_set_decided_side_pass_direction(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00000010u;
  decided_side_pass_direction_ = value;
}
inline void PathDeciderInfo::set_decided_side_pass_direction(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _internal_set_decided_side_pass_direction(value);
  // @@protoc_insertion_point(field_set:apollo.planning.PathDeciderInfo.decided_side_pass_direction)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace planning
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2fplanning_2fproto_2fpath_5fdecider_5finfo_2eproto
