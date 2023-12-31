// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/perception/lidar/app/proto/lidar_obstacle_tracking_config.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2fperception_2flidar_2fapp_2fproto_2flidar_5fobstacle_5ftracking_5fconfig_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2fperception_2flidar_2fapp_2fproto_2flidar_5fobstacle_5ftracking_5fconfig_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_modules_2fperception_2flidar_2fapp_2fproto_2flidar_5fobstacle_5ftracking_5fconfig_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2fperception_2flidar_2fapp_2fproto_2flidar_5fobstacle_5ftracking_5fconfig_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fperception_2flidar_2fapp_2fproto_2flidar_5fobstacle_5ftracking_5fconfig_2eproto;
namespace apollo {
namespace perception {
namespace lidar {
class LidarObstacleTrackingConfig;
class LidarObstacleTrackingConfigDefaultTypeInternal;
extern LidarObstacleTrackingConfigDefaultTypeInternal _LidarObstacleTrackingConfig_default_instance_;
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::perception::lidar::LidarObstacleTrackingConfig* Arena::CreateMaybeMessage<::apollo::perception::lidar::LidarObstacleTrackingConfig>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace perception {
namespace lidar {

// ===================================================================

class LidarObstacleTrackingConfig :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.perception.lidar.LidarObstacleTrackingConfig) */ {
 public:
  LidarObstacleTrackingConfig();
  virtual ~LidarObstacleTrackingConfig();

  LidarObstacleTrackingConfig(const LidarObstacleTrackingConfig& from);
  LidarObstacleTrackingConfig(LidarObstacleTrackingConfig&& from) noexcept
    : LidarObstacleTrackingConfig() {
    *this = ::std::move(from);
  }

  inline LidarObstacleTrackingConfig& operator=(const LidarObstacleTrackingConfig& from) {
    CopyFrom(from);
    return *this;
  }
  inline LidarObstacleTrackingConfig& operator=(LidarObstacleTrackingConfig&& from) noexcept {
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
  static const LidarObstacleTrackingConfig& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const LidarObstacleTrackingConfig* internal_default_instance() {
    return reinterpret_cast<const LidarObstacleTrackingConfig*>(
               &_LidarObstacleTrackingConfig_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(LidarObstacleTrackingConfig& a, LidarObstacleTrackingConfig& b) {
    a.Swap(&b);
  }
  inline void Swap(LidarObstacleTrackingConfig* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline LidarObstacleTrackingConfig* New() const final {
    return CreateMaybeMessage<LidarObstacleTrackingConfig>(nullptr);
  }

  LidarObstacleTrackingConfig* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<LidarObstacleTrackingConfig>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const LidarObstacleTrackingConfig& from);
  void MergeFrom(const LidarObstacleTrackingConfig& from);
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
  void InternalSwap(LidarObstacleTrackingConfig* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.perception.lidar.LidarObstacleTrackingConfig";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2fperception_2flidar_2fapp_2fproto_2flidar_5fobstacle_5ftracking_5fconfig_2eproto);
    return ::descriptor_table_modules_2fperception_2flidar_2fapp_2fproto_2flidar_5fobstacle_5ftracking_5fconfig_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kMultiTargetTrackerFieldNumber = 1,
    kFrameClassifierFieldNumber = 2,
    kFusionClassifierFieldNumber = 3,
  };
  // optional string multi_target_tracker = 1 [default = "DummyMultiTargetTracker"];
  bool has_multi_target_tracker() const;
  private:
  bool _internal_has_multi_target_tracker() const;
  public:
  void clear_multi_target_tracker();
  const std::string& multi_target_tracker() const;
  void set_multi_target_tracker(const std::string& value);
  void set_multi_target_tracker(std::string&& value);
  void set_multi_target_tracker(const char* value);
  void set_multi_target_tracker(const char* value, size_t size);
  std::string* mutable_multi_target_tracker();
  std::string* release_multi_target_tracker();
  void set_allocated_multi_target_tracker(std::string* multi_target_tracker);
  private:
  const std::string& _internal_multi_target_tracker() const;
  void _internal_set_multi_target_tracker(const std::string& value);
  std::string* _internal_mutable_multi_target_tracker();
  public:

  // optional string frame_classifier = 2 [default = "DummyClassifier"];
  bool has_frame_classifier() const;
  private:
  bool _internal_has_frame_classifier() const;
  public:
  void clear_frame_classifier();
  const std::string& frame_classifier() const;
  void set_frame_classifier(const std::string& value);
  void set_frame_classifier(std::string&& value);
  void set_frame_classifier(const char* value);
  void set_frame_classifier(const char* value, size_t size);
  std::string* mutable_frame_classifier();
  std::string* release_frame_classifier();
  void set_allocated_frame_classifier(std::string* frame_classifier);
  private:
  const std::string& _internal_frame_classifier() const;
  void _internal_set_frame_classifier(const std::string& value);
  std::string* _internal_mutable_frame_classifier();
  public:

  // optional string fusion_classifier = 3 [default = "DummyClassifier"];
  bool has_fusion_classifier() const;
  private:
  bool _internal_has_fusion_classifier() const;
  public:
  void clear_fusion_classifier();
  const std::string& fusion_classifier() const;
  void set_fusion_classifier(const std::string& value);
  void set_fusion_classifier(std::string&& value);
  void set_fusion_classifier(const char* value);
  void set_fusion_classifier(const char* value, size_t size);
  std::string* mutable_fusion_classifier();
  std::string* release_fusion_classifier();
  void set_allocated_fusion_classifier(std::string* fusion_classifier);
  private:
  const std::string& _internal_fusion_classifier() const;
  void _internal_set_fusion_classifier(const std::string& value);
  std::string* _internal_mutable_fusion_classifier();
  public:

  // @@protoc_insertion_point(class_scope:apollo.perception.lidar.LidarObstacleTrackingConfig)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  public:
  static ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<std::string> _i_give_permission_to_break_this_code_default_multi_target_tracker_;
  private:
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr multi_target_tracker_;
  public:
  static ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<std::string> _i_give_permission_to_break_this_code_default_frame_classifier_;
  private:
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr frame_classifier_;
  public:
  static ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<std::string> _i_give_permission_to_break_this_code_default_fusion_classifier_;
  private:
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr fusion_classifier_;
  friend struct ::TableStruct_modules_2fperception_2flidar_2fapp_2fproto_2flidar_5fobstacle_5ftracking_5fconfig_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// LidarObstacleTrackingConfig

// optional string multi_target_tracker = 1 [default = "DummyMultiTargetTracker"];
inline bool LidarObstacleTrackingConfig::_internal_has_multi_target_tracker() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool LidarObstacleTrackingConfig::has_multi_target_tracker() const {
  return _internal_has_multi_target_tracker();
}
inline void LidarObstacleTrackingConfig::clear_multi_target_tracker() {
  multi_target_tracker_.ClearToDefaultNoArena(&::apollo::perception::lidar::LidarObstacleTrackingConfig::_i_give_permission_to_break_this_code_default_multi_target_tracker_.get());
  _has_bits_[0] &= ~0x00000001u;
}
inline const std::string& LidarObstacleTrackingConfig::multi_target_tracker() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.LidarObstacleTrackingConfig.multi_target_tracker)
  return _internal_multi_target_tracker();
}
inline void LidarObstacleTrackingConfig::set_multi_target_tracker(const std::string& value) {
  _internal_set_multi_target_tracker(value);
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.LidarObstacleTrackingConfig.multi_target_tracker)
}
inline std::string* LidarObstacleTrackingConfig::mutable_multi_target_tracker() {
  // @@protoc_insertion_point(field_mutable:apollo.perception.lidar.LidarObstacleTrackingConfig.multi_target_tracker)
  return _internal_mutable_multi_target_tracker();
}
inline const std::string& LidarObstacleTrackingConfig::_internal_multi_target_tracker() const {
  return multi_target_tracker_.GetNoArena();
}
inline void LidarObstacleTrackingConfig::_internal_set_multi_target_tracker(const std::string& value) {
  _has_bits_[0] |= 0x00000001u;
  multi_target_tracker_.SetNoArena(&::apollo::perception::lidar::LidarObstacleTrackingConfig::_i_give_permission_to_break_this_code_default_multi_target_tracker_.get(), value);
}
inline void LidarObstacleTrackingConfig::set_multi_target_tracker(std::string&& value) {
  _has_bits_[0] |= 0x00000001u;
  multi_target_tracker_.SetNoArena(
    &::apollo::perception::lidar::LidarObstacleTrackingConfig::_i_give_permission_to_break_this_code_default_multi_target_tracker_.get(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:apollo.perception.lidar.LidarObstacleTrackingConfig.multi_target_tracker)
}
inline void LidarObstacleTrackingConfig::set_multi_target_tracker(const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  _has_bits_[0] |= 0x00000001u;
  multi_target_tracker_.SetNoArena(&::apollo::perception::lidar::LidarObstacleTrackingConfig::_i_give_permission_to_break_this_code_default_multi_target_tracker_.get(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:apollo.perception.lidar.LidarObstacleTrackingConfig.multi_target_tracker)
}
inline void LidarObstacleTrackingConfig::set_multi_target_tracker(const char* value, size_t size) {
  _has_bits_[0] |= 0x00000001u;
  multi_target_tracker_.SetNoArena(&::apollo::perception::lidar::LidarObstacleTrackingConfig::_i_give_permission_to_break_this_code_default_multi_target_tracker_.get(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:apollo.perception.lidar.LidarObstacleTrackingConfig.multi_target_tracker)
}
inline std::string* LidarObstacleTrackingConfig::_internal_mutable_multi_target_tracker() {
  _has_bits_[0] |= 0x00000001u;
  return multi_target_tracker_.MutableNoArena(&::apollo::perception::lidar::LidarObstacleTrackingConfig::_i_give_permission_to_break_this_code_default_multi_target_tracker_.get());
}
inline std::string* LidarObstacleTrackingConfig::release_multi_target_tracker() {
  // @@protoc_insertion_point(field_release:apollo.perception.lidar.LidarObstacleTrackingConfig.multi_target_tracker)
  if (!_internal_has_multi_target_tracker()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000001u;
  return multi_target_tracker_.ReleaseNonDefaultNoArena(&::apollo::perception::lidar::LidarObstacleTrackingConfig::_i_give_permission_to_break_this_code_default_multi_target_tracker_.get());
}
inline void LidarObstacleTrackingConfig::set_allocated_multi_target_tracker(std::string* multi_target_tracker) {
  if (multi_target_tracker != nullptr) {
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  multi_target_tracker_.SetAllocatedNoArena(&::apollo::perception::lidar::LidarObstacleTrackingConfig::_i_give_permission_to_break_this_code_default_multi_target_tracker_.get(), multi_target_tracker);
  // @@protoc_insertion_point(field_set_allocated:apollo.perception.lidar.LidarObstacleTrackingConfig.multi_target_tracker)
}

// optional string frame_classifier = 2 [default = "DummyClassifier"];
inline bool LidarObstacleTrackingConfig::_internal_has_frame_classifier() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool LidarObstacleTrackingConfig::has_frame_classifier() const {
  return _internal_has_frame_classifier();
}
inline void LidarObstacleTrackingConfig::clear_frame_classifier() {
  frame_classifier_.ClearToDefaultNoArena(&::apollo::perception::lidar::LidarObstacleTrackingConfig::_i_give_permission_to_break_this_code_default_frame_classifier_.get());
  _has_bits_[0] &= ~0x00000002u;
}
inline const std::string& LidarObstacleTrackingConfig::frame_classifier() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.LidarObstacleTrackingConfig.frame_classifier)
  return _internal_frame_classifier();
}
inline void LidarObstacleTrackingConfig::set_frame_classifier(const std::string& value) {
  _internal_set_frame_classifier(value);
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.LidarObstacleTrackingConfig.frame_classifier)
}
inline std::string* LidarObstacleTrackingConfig::mutable_frame_classifier() {
  // @@protoc_insertion_point(field_mutable:apollo.perception.lidar.LidarObstacleTrackingConfig.frame_classifier)
  return _internal_mutable_frame_classifier();
}
inline const std::string& LidarObstacleTrackingConfig::_internal_frame_classifier() const {
  return frame_classifier_.GetNoArena();
}
inline void LidarObstacleTrackingConfig::_internal_set_frame_classifier(const std::string& value) {
  _has_bits_[0] |= 0x00000002u;
  frame_classifier_.SetNoArena(&::apollo::perception::lidar::LidarObstacleTrackingConfig::_i_give_permission_to_break_this_code_default_frame_classifier_.get(), value);
}
inline void LidarObstacleTrackingConfig::set_frame_classifier(std::string&& value) {
  _has_bits_[0] |= 0x00000002u;
  frame_classifier_.SetNoArena(
    &::apollo::perception::lidar::LidarObstacleTrackingConfig::_i_give_permission_to_break_this_code_default_frame_classifier_.get(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:apollo.perception.lidar.LidarObstacleTrackingConfig.frame_classifier)
}
inline void LidarObstacleTrackingConfig::set_frame_classifier(const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  _has_bits_[0] |= 0x00000002u;
  frame_classifier_.SetNoArena(&::apollo::perception::lidar::LidarObstacleTrackingConfig::_i_give_permission_to_break_this_code_default_frame_classifier_.get(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:apollo.perception.lidar.LidarObstacleTrackingConfig.frame_classifier)
}
inline void LidarObstacleTrackingConfig::set_frame_classifier(const char* value, size_t size) {
  _has_bits_[0] |= 0x00000002u;
  frame_classifier_.SetNoArena(&::apollo::perception::lidar::LidarObstacleTrackingConfig::_i_give_permission_to_break_this_code_default_frame_classifier_.get(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:apollo.perception.lidar.LidarObstacleTrackingConfig.frame_classifier)
}
inline std::string* LidarObstacleTrackingConfig::_internal_mutable_frame_classifier() {
  _has_bits_[0] |= 0x00000002u;
  return frame_classifier_.MutableNoArena(&::apollo::perception::lidar::LidarObstacleTrackingConfig::_i_give_permission_to_break_this_code_default_frame_classifier_.get());
}
inline std::string* LidarObstacleTrackingConfig::release_frame_classifier() {
  // @@protoc_insertion_point(field_release:apollo.perception.lidar.LidarObstacleTrackingConfig.frame_classifier)
  if (!_internal_has_frame_classifier()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000002u;
  return frame_classifier_.ReleaseNonDefaultNoArena(&::apollo::perception::lidar::LidarObstacleTrackingConfig::_i_give_permission_to_break_this_code_default_frame_classifier_.get());
}
inline void LidarObstacleTrackingConfig::set_allocated_frame_classifier(std::string* frame_classifier) {
  if (frame_classifier != nullptr) {
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  frame_classifier_.SetAllocatedNoArena(&::apollo::perception::lidar::LidarObstacleTrackingConfig::_i_give_permission_to_break_this_code_default_frame_classifier_.get(), frame_classifier);
  // @@protoc_insertion_point(field_set_allocated:apollo.perception.lidar.LidarObstacleTrackingConfig.frame_classifier)
}

// optional string fusion_classifier = 3 [default = "DummyClassifier"];
inline bool LidarObstacleTrackingConfig::_internal_has_fusion_classifier() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool LidarObstacleTrackingConfig::has_fusion_classifier() const {
  return _internal_has_fusion_classifier();
}
inline void LidarObstacleTrackingConfig::clear_fusion_classifier() {
  fusion_classifier_.ClearToDefaultNoArena(&::apollo::perception::lidar::LidarObstacleTrackingConfig::_i_give_permission_to_break_this_code_default_fusion_classifier_.get());
  _has_bits_[0] &= ~0x00000004u;
}
inline const std::string& LidarObstacleTrackingConfig::fusion_classifier() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.LidarObstacleTrackingConfig.fusion_classifier)
  return _internal_fusion_classifier();
}
inline void LidarObstacleTrackingConfig::set_fusion_classifier(const std::string& value) {
  _internal_set_fusion_classifier(value);
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.LidarObstacleTrackingConfig.fusion_classifier)
}
inline std::string* LidarObstacleTrackingConfig::mutable_fusion_classifier() {
  // @@protoc_insertion_point(field_mutable:apollo.perception.lidar.LidarObstacleTrackingConfig.fusion_classifier)
  return _internal_mutable_fusion_classifier();
}
inline const std::string& LidarObstacleTrackingConfig::_internal_fusion_classifier() const {
  return fusion_classifier_.GetNoArena();
}
inline void LidarObstacleTrackingConfig::_internal_set_fusion_classifier(const std::string& value) {
  _has_bits_[0] |= 0x00000004u;
  fusion_classifier_.SetNoArena(&::apollo::perception::lidar::LidarObstacleTrackingConfig::_i_give_permission_to_break_this_code_default_fusion_classifier_.get(), value);
}
inline void LidarObstacleTrackingConfig::set_fusion_classifier(std::string&& value) {
  _has_bits_[0] |= 0x00000004u;
  fusion_classifier_.SetNoArena(
    &::apollo::perception::lidar::LidarObstacleTrackingConfig::_i_give_permission_to_break_this_code_default_fusion_classifier_.get(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:apollo.perception.lidar.LidarObstacleTrackingConfig.fusion_classifier)
}
inline void LidarObstacleTrackingConfig::set_fusion_classifier(const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  _has_bits_[0] |= 0x00000004u;
  fusion_classifier_.SetNoArena(&::apollo::perception::lidar::LidarObstacleTrackingConfig::_i_give_permission_to_break_this_code_default_fusion_classifier_.get(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:apollo.perception.lidar.LidarObstacleTrackingConfig.fusion_classifier)
}
inline void LidarObstacleTrackingConfig::set_fusion_classifier(const char* value, size_t size) {
  _has_bits_[0] |= 0x00000004u;
  fusion_classifier_.SetNoArena(&::apollo::perception::lidar::LidarObstacleTrackingConfig::_i_give_permission_to_break_this_code_default_fusion_classifier_.get(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:apollo.perception.lidar.LidarObstacleTrackingConfig.fusion_classifier)
}
inline std::string* LidarObstacleTrackingConfig::_internal_mutable_fusion_classifier() {
  _has_bits_[0] |= 0x00000004u;
  return fusion_classifier_.MutableNoArena(&::apollo::perception::lidar::LidarObstacleTrackingConfig::_i_give_permission_to_break_this_code_default_fusion_classifier_.get());
}
inline std::string* LidarObstacleTrackingConfig::release_fusion_classifier() {
  // @@protoc_insertion_point(field_release:apollo.perception.lidar.LidarObstacleTrackingConfig.fusion_classifier)
  if (!_internal_has_fusion_classifier()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000004u;
  return fusion_classifier_.ReleaseNonDefaultNoArena(&::apollo::perception::lidar::LidarObstacleTrackingConfig::_i_give_permission_to_break_this_code_default_fusion_classifier_.get());
}
inline void LidarObstacleTrackingConfig::set_allocated_fusion_classifier(std::string* fusion_classifier) {
  if (fusion_classifier != nullptr) {
    _has_bits_[0] |= 0x00000004u;
  } else {
    _has_bits_[0] &= ~0x00000004u;
  }
  fusion_classifier_.SetAllocatedNoArena(&::apollo::perception::lidar::LidarObstacleTrackingConfig::_i_give_permission_to_break_this_code_default_fusion_classifier_.get(), fusion_classifier);
  // @@protoc_insertion_point(field_set_allocated:apollo.perception.lidar.LidarObstacleTrackingConfig.fusion_classifier)
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
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2fperception_2flidar_2fapp_2fproto_2flidar_5fobstacle_5ftracking_5fconfig_2eproto
