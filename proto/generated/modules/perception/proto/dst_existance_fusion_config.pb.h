// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/perception/proto/dst_existance_fusion_config.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2fperception_2fproto_2fdst_5fexistance_5ffusion_5fconfig_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2fperception_2fproto_2fdst_5fexistance_5ffusion_5fconfig_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_modules_2fperception_2fproto_2fdst_5fexistance_5ffusion_5fconfig_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2fperception_2fproto_2fdst_5fexistance_5ffusion_5fconfig_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxillaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[2]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fperception_2fproto_2fdst_5fexistance_5ffusion_5fconfig_2eproto;
namespace apollo {
namespace perception {
class CameraValidDist;
class CameraValidDistDefaultTypeInternal;
extern CameraValidDistDefaultTypeInternal _CameraValidDist_default_instance_;
class DstExistanceFusionConfig;
class DstExistanceFusionConfigDefaultTypeInternal;
extern DstExistanceFusionConfigDefaultTypeInternal _DstExistanceFusionConfig_default_instance_;
}  // namespace perception
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::perception::CameraValidDist* Arena::CreateMaybeMessage<::apollo::perception::CameraValidDist>(Arena*);
template<> ::apollo::perception::DstExistanceFusionConfig* Arena::CreateMaybeMessage<::apollo::perception::DstExistanceFusionConfig>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace perception {

// ===================================================================

class CameraValidDist :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.perception.CameraValidDist) */ {
 public:
  CameraValidDist();
  virtual ~CameraValidDist();

  CameraValidDist(const CameraValidDist& from);
  CameraValidDist(CameraValidDist&& from) noexcept
    : CameraValidDist() {
    *this = ::std::move(from);
  }

  inline CameraValidDist& operator=(const CameraValidDist& from) {
    CopyFrom(from);
    return *this;
  }
  inline CameraValidDist& operator=(CameraValidDist&& from) noexcept {
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
  static const CameraValidDist& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const CameraValidDist* internal_default_instance() {
    return reinterpret_cast<const CameraValidDist*>(
               &_CameraValidDist_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(CameraValidDist& a, CameraValidDist& b) {
    a.Swap(&b);
  }
  inline void Swap(CameraValidDist* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline CameraValidDist* New() const final {
    return CreateMaybeMessage<CameraValidDist>(nullptr);
  }

  CameraValidDist* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<CameraValidDist>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const CameraValidDist& from);
  void MergeFrom(const CameraValidDist& from);
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
  void InternalSwap(CameraValidDist* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.perception.CameraValidDist";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2fperception_2fproto_2fdst_5fexistance_5ffusion_5fconfig_2eproto);
    return ::descriptor_table_modules_2fperception_2fproto_2fdst_5fexistance_5ffusion_5fconfig_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kCameraNameFieldNumber = 1,
    kValidDistFieldNumber = 2,
  };
  // optional string camera_name = 1 [default = ""];
  bool has_camera_name() const;
  private:
  bool _internal_has_camera_name() const;
  public:
  void clear_camera_name();
  const std::string& camera_name() const;
  void set_camera_name(const std::string& value);
  void set_camera_name(std::string&& value);
  void set_camera_name(const char* value);
  void set_camera_name(const char* value, size_t size);
  std::string* mutable_camera_name();
  std::string* release_camera_name();
  void set_allocated_camera_name(std::string* camera_name);
  private:
  const std::string& _internal_camera_name() const;
  void _internal_set_camera_name(const std::string& value);
  std::string* _internal_mutable_camera_name();
  public:

  // optional double valid_dist = 2 [default = 0];
  bool has_valid_dist() const;
  private:
  bool _internal_has_valid_dist() const;
  public:
  void clear_valid_dist();
  double valid_dist() const;
  void set_valid_dist(double value);
  private:
  double _internal_valid_dist() const;
  void _internal_set_valid_dist(double value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.perception.CameraValidDist)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr camera_name_;
  double valid_dist_;
  friend struct ::TableStruct_modules_2fperception_2fproto_2fdst_5fexistance_5ffusion_5fconfig_2eproto;
};
// -------------------------------------------------------------------

class DstExistanceFusionConfig :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.perception.DstExistanceFusionConfig) */ {
 public:
  DstExistanceFusionConfig();
  virtual ~DstExistanceFusionConfig();

  DstExistanceFusionConfig(const DstExistanceFusionConfig& from);
  DstExistanceFusionConfig(DstExistanceFusionConfig&& from) noexcept
    : DstExistanceFusionConfig() {
    *this = ::std::move(from);
  }

  inline DstExistanceFusionConfig& operator=(const DstExistanceFusionConfig& from) {
    CopyFrom(from);
    return *this;
  }
  inline DstExistanceFusionConfig& operator=(DstExistanceFusionConfig&& from) noexcept {
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
  static const DstExistanceFusionConfig& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const DstExistanceFusionConfig* internal_default_instance() {
    return reinterpret_cast<const DstExistanceFusionConfig*>(
               &_DstExistanceFusionConfig_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(DstExistanceFusionConfig& a, DstExistanceFusionConfig& b) {
    a.Swap(&b);
  }
  inline void Swap(DstExistanceFusionConfig* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline DstExistanceFusionConfig* New() const final {
    return CreateMaybeMessage<DstExistanceFusionConfig>(nullptr);
  }

  DstExistanceFusionConfig* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<DstExistanceFusionConfig>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const DstExistanceFusionConfig& from);
  void MergeFrom(const DstExistanceFusionConfig& from);
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
  void InternalSwap(DstExistanceFusionConfig* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.perception.DstExistanceFusionConfig";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2fperception_2fproto_2fdst_5fexistance_5ffusion_5fconfig_2eproto);
    return ::descriptor_table_modules_2fperception_2fproto_2fdst_5fexistance_5ffusion_5fconfig_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kCameraValidDistFieldNumber = 2,
    kTrackObjectMaxMatchDistanceFieldNumber = 1,
  };
  // repeated .apollo.perception.CameraValidDist camera_valid_dist = 2;
  int camera_valid_dist_size() const;
  private:
  int _internal_camera_valid_dist_size() const;
  public:
  void clear_camera_valid_dist();
  ::apollo::perception::CameraValidDist* mutable_camera_valid_dist(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::perception::CameraValidDist >*
      mutable_camera_valid_dist();
  private:
  const ::apollo::perception::CameraValidDist& _internal_camera_valid_dist(int index) const;
  ::apollo::perception::CameraValidDist* _internal_add_camera_valid_dist();
  public:
  const ::apollo::perception::CameraValidDist& camera_valid_dist(int index) const;
  ::apollo::perception::CameraValidDist* add_camera_valid_dist();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::perception::CameraValidDist >&
      camera_valid_dist() const;

  // optional double track_object_max_match_distance = 1 [default = 4];
  bool has_track_object_max_match_distance() const;
  private:
  bool _internal_has_track_object_max_match_distance() const;
  public:
  void clear_track_object_max_match_distance();
  double track_object_max_match_distance() const;
  void set_track_object_max_match_distance(double value);
  private:
  double _internal_track_object_max_match_distance() const;
  void _internal_set_track_object_max_match_distance(double value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.perception.DstExistanceFusionConfig)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::perception::CameraValidDist > camera_valid_dist_;
  double track_object_max_match_distance_;
  friend struct ::TableStruct_modules_2fperception_2fproto_2fdst_5fexistance_5ffusion_5fconfig_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// CameraValidDist

// optional string camera_name = 1 [default = ""];
inline bool CameraValidDist::_internal_has_camera_name() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool CameraValidDist::has_camera_name() const {
  return _internal_has_camera_name();
}
inline void CameraValidDist::clear_camera_name() {
  camera_name_.ClearToEmptyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  _has_bits_[0] &= ~0x00000001u;
}
inline const std::string& CameraValidDist::camera_name() const {
  // @@protoc_insertion_point(field_get:apollo.perception.CameraValidDist.camera_name)
  return _internal_camera_name();
}
inline void CameraValidDist::set_camera_name(const std::string& value) {
  _internal_set_camera_name(value);
  // @@protoc_insertion_point(field_set:apollo.perception.CameraValidDist.camera_name)
}
inline std::string* CameraValidDist::mutable_camera_name() {
  // @@protoc_insertion_point(field_mutable:apollo.perception.CameraValidDist.camera_name)
  return _internal_mutable_camera_name();
}
inline const std::string& CameraValidDist::_internal_camera_name() const {
  return camera_name_.GetNoArena();
}
inline void CameraValidDist::_internal_set_camera_name(const std::string& value) {
  _has_bits_[0] |= 0x00000001u;
  camera_name_.SetNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), value);
}
inline void CameraValidDist::set_camera_name(std::string&& value) {
  _has_bits_[0] |= 0x00000001u;
  camera_name_.SetNoArena(
    &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:apollo.perception.CameraValidDist.camera_name)
}
inline void CameraValidDist::set_camera_name(const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  _has_bits_[0] |= 0x00000001u;
  camera_name_.SetNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:apollo.perception.CameraValidDist.camera_name)
}
inline void CameraValidDist::set_camera_name(const char* value, size_t size) {
  _has_bits_[0] |= 0x00000001u;
  camera_name_.SetNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:apollo.perception.CameraValidDist.camera_name)
}
inline std::string* CameraValidDist::_internal_mutable_camera_name() {
  _has_bits_[0] |= 0x00000001u;
  return camera_name_.MutableNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}
inline std::string* CameraValidDist::release_camera_name() {
  // @@protoc_insertion_point(field_release:apollo.perception.CameraValidDist.camera_name)
  if (!_internal_has_camera_name()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000001u;
  return camera_name_.ReleaseNonDefaultNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}
inline void CameraValidDist::set_allocated_camera_name(std::string* camera_name) {
  if (camera_name != nullptr) {
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  camera_name_.SetAllocatedNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), camera_name);
  // @@protoc_insertion_point(field_set_allocated:apollo.perception.CameraValidDist.camera_name)
}

// optional double valid_dist = 2 [default = 0];
inline bool CameraValidDist::_internal_has_valid_dist() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool CameraValidDist::has_valid_dist() const {
  return _internal_has_valid_dist();
}
inline void CameraValidDist::clear_valid_dist() {
  valid_dist_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline double CameraValidDist::_internal_valid_dist() const {
  return valid_dist_;
}
inline double CameraValidDist::valid_dist() const {
  // @@protoc_insertion_point(field_get:apollo.perception.CameraValidDist.valid_dist)
  return _internal_valid_dist();
}
inline void CameraValidDist::_internal_set_valid_dist(double value) {
  _has_bits_[0] |= 0x00000002u;
  valid_dist_ = value;
}
inline void CameraValidDist::set_valid_dist(double value) {
  _internal_set_valid_dist(value);
  // @@protoc_insertion_point(field_set:apollo.perception.CameraValidDist.valid_dist)
}

// -------------------------------------------------------------------

// DstExistanceFusionConfig

// optional double track_object_max_match_distance = 1 [default = 4];
inline bool DstExistanceFusionConfig::_internal_has_track_object_max_match_distance() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool DstExistanceFusionConfig::has_track_object_max_match_distance() const {
  return _internal_has_track_object_max_match_distance();
}
inline void DstExistanceFusionConfig::clear_track_object_max_match_distance() {
  track_object_max_match_distance_ = 4;
  _has_bits_[0] &= ~0x00000001u;
}
inline double DstExistanceFusionConfig::_internal_track_object_max_match_distance() const {
  return track_object_max_match_distance_;
}
inline double DstExistanceFusionConfig::track_object_max_match_distance() const {
  // @@protoc_insertion_point(field_get:apollo.perception.DstExistanceFusionConfig.track_object_max_match_distance)
  return _internal_track_object_max_match_distance();
}
inline void DstExistanceFusionConfig::_internal_set_track_object_max_match_distance(double value) {
  _has_bits_[0] |= 0x00000001u;
  track_object_max_match_distance_ = value;
}
inline void DstExistanceFusionConfig::set_track_object_max_match_distance(double value) {
  _internal_set_track_object_max_match_distance(value);
  // @@protoc_insertion_point(field_set:apollo.perception.DstExistanceFusionConfig.track_object_max_match_distance)
}

// repeated .apollo.perception.CameraValidDist camera_valid_dist = 2;
inline int DstExistanceFusionConfig::_internal_camera_valid_dist_size() const {
  return camera_valid_dist_.size();
}
inline int DstExistanceFusionConfig::camera_valid_dist_size() const {
  return _internal_camera_valid_dist_size();
}
inline void DstExistanceFusionConfig::clear_camera_valid_dist() {
  camera_valid_dist_.Clear();
}
inline ::apollo::perception::CameraValidDist* DstExistanceFusionConfig::mutable_camera_valid_dist(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.perception.DstExistanceFusionConfig.camera_valid_dist)
  return camera_valid_dist_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::perception::CameraValidDist >*
DstExistanceFusionConfig::mutable_camera_valid_dist() {
  // @@protoc_insertion_point(field_mutable_list:apollo.perception.DstExistanceFusionConfig.camera_valid_dist)
  return &camera_valid_dist_;
}
inline const ::apollo::perception::CameraValidDist& DstExistanceFusionConfig::_internal_camera_valid_dist(int index) const {
  return camera_valid_dist_.Get(index);
}
inline const ::apollo::perception::CameraValidDist& DstExistanceFusionConfig::camera_valid_dist(int index) const {
  // @@protoc_insertion_point(field_get:apollo.perception.DstExistanceFusionConfig.camera_valid_dist)
  return _internal_camera_valid_dist(index);
}
inline ::apollo::perception::CameraValidDist* DstExistanceFusionConfig::_internal_add_camera_valid_dist() {
  return camera_valid_dist_.Add();
}
inline ::apollo::perception::CameraValidDist* DstExistanceFusionConfig::add_camera_valid_dist() {
  // @@protoc_insertion_point(field_add:apollo.perception.DstExistanceFusionConfig.camera_valid_dist)
  return _internal_add_camera_valid_dist();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::perception::CameraValidDist >&
DstExistanceFusionConfig::camera_valid_dist() const {
  // @@protoc_insertion_point(field_list:apollo.perception.DstExistanceFusionConfig.camera_valid_dist)
  return camera_valid_dist_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace perception
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2fperception_2fproto_2fdst_5fexistance_5ffusion_5fconfig_2eproto
