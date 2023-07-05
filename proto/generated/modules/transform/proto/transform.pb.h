// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/transform/proto/transform.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2ftransform_2fproto_2ftransform_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2ftransform_2fproto_2ftransform_2eproto

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
#include "modules/common/proto/header.pb.h"
#include "modules/common/proto/geometry.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_modules_2ftransform_2fproto_2ftransform_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2ftransform_2fproto_2ftransform_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxillaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[3]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2ftransform_2fproto_2ftransform_2eproto;
namespace apollo {
namespace transform {
class Transform;
class TransformDefaultTypeInternal;
extern TransformDefaultTypeInternal _Transform_default_instance_;
class TransformStamped;
class TransformStampedDefaultTypeInternal;
extern TransformStampedDefaultTypeInternal _TransformStamped_default_instance_;
class TransformStampeds;
class TransformStampedsDefaultTypeInternal;
extern TransformStampedsDefaultTypeInternal _TransformStampeds_default_instance_;
}  // namespace transform
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::transform::Transform* Arena::CreateMaybeMessage<::apollo::transform::Transform>(Arena*);
template<> ::apollo::transform::TransformStamped* Arena::CreateMaybeMessage<::apollo::transform::TransformStamped>(Arena*);
template<> ::apollo::transform::TransformStampeds* Arena::CreateMaybeMessage<::apollo::transform::TransformStampeds>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace transform {

// ===================================================================

class Transform :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.transform.Transform) */ {
 public:
  Transform();
  virtual ~Transform();

  Transform(const Transform& from);
  Transform(Transform&& from) noexcept
    : Transform() {
    *this = ::std::move(from);
  }

  inline Transform& operator=(const Transform& from) {
    CopyFrom(from);
    return *this;
  }
  inline Transform& operator=(Transform&& from) noexcept {
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
  static const Transform& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Transform* internal_default_instance() {
    return reinterpret_cast<const Transform*>(
               &_Transform_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(Transform& a, Transform& b) {
    a.Swap(&b);
  }
  inline void Swap(Transform* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline Transform* New() const final {
    return CreateMaybeMessage<Transform>(nullptr);
  }

  Transform* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<Transform>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const Transform& from);
  void MergeFrom(const Transform& from);
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
  void InternalSwap(Transform* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.transform.Transform";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2ftransform_2fproto_2ftransform_2eproto);
    return ::descriptor_table_modules_2ftransform_2fproto_2ftransform_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kTranslationFieldNumber = 1,
    kRotationFieldNumber = 2,
  };
  // optional .apollo.common.Point3D translation = 1;
  bool has_translation() const;
  private:
  bool _internal_has_translation() const;
  public:
  void clear_translation();
  const ::apollo::common::Point3D& translation() const;
  ::apollo::common::Point3D* release_translation();
  ::apollo::common::Point3D* mutable_translation();
  void set_allocated_translation(::apollo::common::Point3D* translation);
  private:
  const ::apollo::common::Point3D& _internal_translation() const;
  ::apollo::common::Point3D* _internal_mutable_translation();
  public:

  // optional .apollo.common.Quaternion rotation = 2;
  bool has_rotation() const;
  private:
  bool _internal_has_rotation() const;
  public:
  void clear_rotation();
  const ::apollo::common::Quaternion& rotation() const;
  ::apollo::common::Quaternion* release_rotation();
  ::apollo::common::Quaternion* mutable_rotation();
  void set_allocated_rotation(::apollo::common::Quaternion* rotation);
  private:
  const ::apollo::common::Quaternion& _internal_rotation() const;
  ::apollo::common::Quaternion* _internal_mutable_rotation();
  public:

  // @@protoc_insertion_point(class_scope:apollo.transform.Transform)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::apollo::common::Point3D* translation_;
  ::apollo::common::Quaternion* rotation_;
  friend struct ::TableStruct_modules_2ftransform_2fproto_2ftransform_2eproto;
};
// -------------------------------------------------------------------

class TransformStamped :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.transform.TransformStamped) */ {
 public:
  TransformStamped();
  virtual ~TransformStamped();

  TransformStamped(const TransformStamped& from);
  TransformStamped(TransformStamped&& from) noexcept
    : TransformStamped() {
    *this = ::std::move(from);
  }

  inline TransformStamped& operator=(const TransformStamped& from) {
    CopyFrom(from);
    return *this;
  }
  inline TransformStamped& operator=(TransformStamped&& from) noexcept {
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
  static const TransformStamped& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const TransformStamped* internal_default_instance() {
    return reinterpret_cast<const TransformStamped*>(
               &_TransformStamped_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(TransformStamped& a, TransformStamped& b) {
    a.Swap(&b);
  }
  inline void Swap(TransformStamped* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline TransformStamped* New() const final {
    return CreateMaybeMessage<TransformStamped>(nullptr);
  }

  TransformStamped* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<TransformStamped>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const TransformStamped& from);
  void MergeFrom(const TransformStamped& from);
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
  void InternalSwap(TransformStamped* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.transform.TransformStamped";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2ftransform_2fproto_2ftransform_2eproto);
    return ::descriptor_table_modules_2ftransform_2fproto_2ftransform_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kChildFrameIdFieldNumber = 2,
    kHeaderFieldNumber = 1,
    kTransformFieldNumber = 3,
  };
  // optional string child_frame_id = 2;
  bool has_child_frame_id() const;
  private:
  bool _internal_has_child_frame_id() const;
  public:
  void clear_child_frame_id();
  const std::string& child_frame_id() const;
  void set_child_frame_id(const std::string& value);
  void set_child_frame_id(std::string&& value);
  void set_child_frame_id(const char* value);
  void set_child_frame_id(const char* value, size_t size);
  std::string* mutable_child_frame_id();
  std::string* release_child_frame_id();
  void set_allocated_child_frame_id(std::string* child_frame_id);
  private:
  const std::string& _internal_child_frame_id() const;
  void _internal_set_child_frame_id(const std::string& value);
  std::string* _internal_mutable_child_frame_id();
  public:

  // optional .apollo.common.Header header = 1;
  bool has_header() const;
  private:
  bool _internal_has_header() const;
  public:
  void clear_header();
  const ::apollo::common::Header& header() const;
  ::apollo::common::Header* release_header();
  ::apollo::common::Header* mutable_header();
  void set_allocated_header(::apollo::common::Header* header);
  private:
  const ::apollo::common::Header& _internal_header() const;
  ::apollo::common::Header* _internal_mutable_header();
  public:

  // optional .apollo.transform.Transform transform = 3;
  bool has_transform() const;
  private:
  bool _internal_has_transform() const;
  public:
  void clear_transform();
  const ::apollo::transform::Transform& transform() const;
  ::apollo::transform::Transform* release_transform();
  ::apollo::transform::Transform* mutable_transform();
  void set_allocated_transform(::apollo::transform::Transform* transform);
  private:
  const ::apollo::transform::Transform& _internal_transform() const;
  ::apollo::transform::Transform* _internal_mutable_transform();
  public:

  // @@protoc_insertion_point(class_scope:apollo.transform.TransformStamped)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr child_frame_id_;
  ::apollo::common::Header* header_;
  ::apollo::transform::Transform* transform_;
  friend struct ::TableStruct_modules_2ftransform_2fproto_2ftransform_2eproto;
};
// -------------------------------------------------------------------

class TransformStampeds :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.transform.TransformStampeds) */ {
 public:
  TransformStampeds();
  virtual ~TransformStampeds();

  TransformStampeds(const TransformStampeds& from);
  TransformStampeds(TransformStampeds&& from) noexcept
    : TransformStampeds() {
    *this = ::std::move(from);
  }

  inline TransformStampeds& operator=(const TransformStampeds& from) {
    CopyFrom(from);
    return *this;
  }
  inline TransformStampeds& operator=(TransformStampeds&& from) noexcept {
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
  static const TransformStampeds& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const TransformStampeds* internal_default_instance() {
    return reinterpret_cast<const TransformStampeds*>(
               &_TransformStampeds_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    2;

  friend void swap(TransformStampeds& a, TransformStampeds& b) {
    a.Swap(&b);
  }
  inline void Swap(TransformStampeds* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline TransformStampeds* New() const final {
    return CreateMaybeMessage<TransformStampeds>(nullptr);
  }

  TransformStampeds* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<TransformStampeds>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const TransformStampeds& from);
  void MergeFrom(const TransformStampeds& from);
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
  void InternalSwap(TransformStampeds* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.transform.TransformStampeds";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2ftransform_2fproto_2ftransform_2eproto);
    return ::descriptor_table_modules_2ftransform_2fproto_2ftransform_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kTransformsFieldNumber = 2,
    kHeaderFieldNumber = 1,
  };
  // repeated .apollo.transform.TransformStamped transforms = 2;
  int transforms_size() const;
  private:
  int _internal_transforms_size() const;
  public:
  void clear_transforms();
  ::apollo::transform::TransformStamped* mutable_transforms(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::transform::TransformStamped >*
      mutable_transforms();
  private:
  const ::apollo::transform::TransformStamped& _internal_transforms(int index) const;
  ::apollo::transform::TransformStamped* _internal_add_transforms();
  public:
  const ::apollo::transform::TransformStamped& transforms(int index) const;
  ::apollo::transform::TransformStamped* add_transforms();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::transform::TransformStamped >&
      transforms() const;

  // optional .apollo.common.Header header = 1;
  bool has_header() const;
  private:
  bool _internal_has_header() const;
  public:
  void clear_header();
  const ::apollo::common::Header& header() const;
  ::apollo::common::Header* release_header();
  ::apollo::common::Header* mutable_header();
  void set_allocated_header(::apollo::common::Header* header);
  private:
  const ::apollo::common::Header& _internal_header() const;
  ::apollo::common::Header* _internal_mutable_header();
  public:

  // @@protoc_insertion_point(class_scope:apollo.transform.TransformStampeds)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::transform::TransformStamped > transforms_;
  ::apollo::common::Header* header_;
  friend struct ::TableStruct_modules_2ftransform_2fproto_2ftransform_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Transform

// optional .apollo.common.Point3D translation = 1;
inline bool Transform::_internal_has_translation() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || translation_ != nullptr);
  return value;
}
inline bool Transform::has_translation() const {
  return _internal_has_translation();
}
inline const ::apollo::common::Point3D& Transform::_internal_translation() const {
  const ::apollo::common::Point3D* p = translation_;
  return p != nullptr ? *p : *reinterpret_cast<const ::apollo::common::Point3D*>(
      &::apollo::common::_Point3D_default_instance_);
}
inline const ::apollo::common::Point3D& Transform::translation() const {
  // @@protoc_insertion_point(field_get:apollo.transform.Transform.translation)
  return _internal_translation();
}
inline ::apollo::common::Point3D* Transform::release_translation() {
  // @@protoc_insertion_point(field_release:apollo.transform.Transform.translation)
  _has_bits_[0] &= ~0x00000001u;
  ::apollo::common::Point3D* temp = translation_;
  translation_ = nullptr;
  return temp;
}
inline ::apollo::common::Point3D* Transform::_internal_mutable_translation() {
  _has_bits_[0] |= 0x00000001u;
  if (translation_ == nullptr) {
    auto* p = CreateMaybeMessage<::apollo::common::Point3D>(GetArenaNoVirtual());
    translation_ = p;
  }
  return translation_;
}
inline ::apollo::common::Point3D* Transform::mutable_translation() {
  // @@protoc_insertion_point(field_mutable:apollo.transform.Transform.translation)
  return _internal_mutable_translation();
}
inline void Transform::set_allocated_translation(::apollo::common::Point3D* translation) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(translation_);
  }
  if (translation) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      translation = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, translation, submessage_arena);
    }
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  translation_ = translation;
  // @@protoc_insertion_point(field_set_allocated:apollo.transform.Transform.translation)
}

// optional .apollo.common.Quaternion rotation = 2;
inline bool Transform::_internal_has_rotation() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  PROTOBUF_ASSUME(!value || rotation_ != nullptr);
  return value;
}
inline bool Transform::has_rotation() const {
  return _internal_has_rotation();
}
inline const ::apollo::common::Quaternion& Transform::_internal_rotation() const {
  const ::apollo::common::Quaternion* p = rotation_;
  return p != nullptr ? *p : *reinterpret_cast<const ::apollo::common::Quaternion*>(
      &::apollo::common::_Quaternion_default_instance_);
}
inline const ::apollo::common::Quaternion& Transform::rotation() const {
  // @@protoc_insertion_point(field_get:apollo.transform.Transform.rotation)
  return _internal_rotation();
}
inline ::apollo::common::Quaternion* Transform::release_rotation() {
  // @@protoc_insertion_point(field_release:apollo.transform.Transform.rotation)
  _has_bits_[0] &= ~0x00000002u;
  ::apollo::common::Quaternion* temp = rotation_;
  rotation_ = nullptr;
  return temp;
}
inline ::apollo::common::Quaternion* Transform::_internal_mutable_rotation() {
  _has_bits_[0] |= 0x00000002u;
  if (rotation_ == nullptr) {
    auto* p = CreateMaybeMessage<::apollo::common::Quaternion>(GetArenaNoVirtual());
    rotation_ = p;
  }
  return rotation_;
}
inline ::apollo::common::Quaternion* Transform::mutable_rotation() {
  // @@protoc_insertion_point(field_mutable:apollo.transform.Transform.rotation)
  return _internal_mutable_rotation();
}
inline void Transform::set_allocated_rotation(::apollo::common::Quaternion* rotation) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(rotation_);
  }
  if (rotation) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      rotation = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, rotation, submessage_arena);
    }
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  rotation_ = rotation;
  // @@protoc_insertion_point(field_set_allocated:apollo.transform.Transform.rotation)
}

// -------------------------------------------------------------------

// TransformStamped

// optional .apollo.common.Header header = 1;
inline bool TransformStamped::_internal_has_header() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  PROTOBUF_ASSUME(!value || header_ != nullptr);
  return value;
}
inline bool TransformStamped::has_header() const {
  return _internal_has_header();
}
inline const ::apollo::common::Header& TransformStamped::_internal_header() const {
  const ::apollo::common::Header* p = header_;
  return p != nullptr ? *p : *reinterpret_cast<const ::apollo::common::Header*>(
      &::apollo::common::_Header_default_instance_);
}
inline const ::apollo::common::Header& TransformStamped::header() const {
  // @@protoc_insertion_point(field_get:apollo.transform.TransformStamped.header)
  return _internal_header();
}
inline ::apollo::common::Header* TransformStamped::release_header() {
  // @@protoc_insertion_point(field_release:apollo.transform.TransformStamped.header)
  _has_bits_[0] &= ~0x00000002u;
  ::apollo::common::Header* temp = header_;
  header_ = nullptr;
  return temp;
}
inline ::apollo::common::Header* TransformStamped::_internal_mutable_header() {
  _has_bits_[0] |= 0x00000002u;
  if (header_ == nullptr) {
    auto* p = CreateMaybeMessage<::apollo::common::Header>(GetArenaNoVirtual());
    header_ = p;
  }
  return header_;
}
inline ::apollo::common::Header* TransformStamped::mutable_header() {
  // @@protoc_insertion_point(field_mutable:apollo.transform.TransformStamped.header)
  return _internal_mutable_header();
}
inline void TransformStamped::set_allocated_header(::apollo::common::Header* header) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(header_);
  }
  if (header) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      header = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, header, submessage_arena);
    }
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  header_ = header;
  // @@protoc_insertion_point(field_set_allocated:apollo.transform.TransformStamped.header)
}

// optional string child_frame_id = 2;
inline bool TransformStamped::_internal_has_child_frame_id() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool TransformStamped::has_child_frame_id() const {
  return _internal_has_child_frame_id();
}
inline void TransformStamped::clear_child_frame_id() {
  child_frame_id_.ClearToEmptyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  _has_bits_[0] &= ~0x00000001u;
}
inline const std::string& TransformStamped::child_frame_id() const {
  // @@protoc_insertion_point(field_get:apollo.transform.TransformStamped.child_frame_id)
  return _internal_child_frame_id();
}
inline void TransformStamped::set_child_frame_id(const std::string& value) {
  _internal_set_child_frame_id(value);
  // @@protoc_insertion_point(field_set:apollo.transform.TransformStamped.child_frame_id)
}
inline std::string* TransformStamped::mutable_child_frame_id() {
  // @@protoc_insertion_point(field_mutable:apollo.transform.TransformStamped.child_frame_id)
  return _internal_mutable_child_frame_id();
}
inline const std::string& TransformStamped::_internal_child_frame_id() const {
  return child_frame_id_.GetNoArena();
}
inline void TransformStamped::_internal_set_child_frame_id(const std::string& value) {
  _has_bits_[0] |= 0x00000001u;
  child_frame_id_.SetNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), value);
}
inline void TransformStamped::set_child_frame_id(std::string&& value) {
  _has_bits_[0] |= 0x00000001u;
  child_frame_id_.SetNoArena(
    &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:apollo.transform.TransformStamped.child_frame_id)
}
inline void TransformStamped::set_child_frame_id(const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  _has_bits_[0] |= 0x00000001u;
  child_frame_id_.SetNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:apollo.transform.TransformStamped.child_frame_id)
}
inline void TransformStamped::set_child_frame_id(const char* value, size_t size) {
  _has_bits_[0] |= 0x00000001u;
  child_frame_id_.SetNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:apollo.transform.TransformStamped.child_frame_id)
}
inline std::string* TransformStamped::_internal_mutable_child_frame_id() {
  _has_bits_[0] |= 0x00000001u;
  return child_frame_id_.MutableNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}
inline std::string* TransformStamped::release_child_frame_id() {
  // @@protoc_insertion_point(field_release:apollo.transform.TransformStamped.child_frame_id)
  if (!_internal_has_child_frame_id()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000001u;
  return child_frame_id_.ReleaseNonDefaultNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}
inline void TransformStamped::set_allocated_child_frame_id(std::string* child_frame_id) {
  if (child_frame_id != nullptr) {
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  child_frame_id_.SetAllocatedNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), child_frame_id);
  // @@protoc_insertion_point(field_set_allocated:apollo.transform.TransformStamped.child_frame_id)
}

// optional .apollo.transform.Transform transform = 3;
inline bool TransformStamped::_internal_has_transform() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  PROTOBUF_ASSUME(!value || transform_ != nullptr);
  return value;
}
inline bool TransformStamped::has_transform() const {
  return _internal_has_transform();
}
inline void TransformStamped::clear_transform() {
  if (transform_ != nullptr) transform_->Clear();
  _has_bits_[0] &= ~0x00000004u;
}
inline const ::apollo::transform::Transform& TransformStamped::_internal_transform() const {
  const ::apollo::transform::Transform* p = transform_;
  return p != nullptr ? *p : *reinterpret_cast<const ::apollo::transform::Transform*>(
      &::apollo::transform::_Transform_default_instance_);
}
inline const ::apollo::transform::Transform& TransformStamped::transform() const {
  // @@protoc_insertion_point(field_get:apollo.transform.TransformStamped.transform)
  return _internal_transform();
}
inline ::apollo::transform::Transform* TransformStamped::release_transform() {
  // @@protoc_insertion_point(field_release:apollo.transform.TransformStamped.transform)
  _has_bits_[0] &= ~0x00000004u;
  ::apollo::transform::Transform* temp = transform_;
  transform_ = nullptr;
  return temp;
}
inline ::apollo::transform::Transform* TransformStamped::_internal_mutable_transform() {
  _has_bits_[0] |= 0x00000004u;
  if (transform_ == nullptr) {
    auto* p = CreateMaybeMessage<::apollo::transform::Transform>(GetArenaNoVirtual());
    transform_ = p;
  }
  return transform_;
}
inline ::apollo::transform::Transform* TransformStamped::mutable_transform() {
  // @@protoc_insertion_point(field_mutable:apollo.transform.TransformStamped.transform)
  return _internal_mutable_transform();
}
inline void TransformStamped::set_allocated_transform(::apollo::transform::Transform* transform) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete transform_;
  }
  if (transform) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      transform = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, transform, submessage_arena);
    }
    _has_bits_[0] |= 0x00000004u;
  } else {
    _has_bits_[0] &= ~0x00000004u;
  }
  transform_ = transform;
  // @@protoc_insertion_point(field_set_allocated:apollo.transform.TransformStamped.transform)
}

// -------------------------------------------------------------------

// TransformStampeds

// optional .apollo.common.Header header = 1;
inline bool TransformStampeds::_internal_has_header() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || header_ != nullptr);
  return value;
}
inline bool TransformStampeds::has_header() const {
  return _internal_has_header();
}
inline const ::apollo::common::Header& TransformStampeds::_internal_header() const {
  const ::apollo::common::Header* p = header_;
  return p != nullptr ? *p : *reinterpret_cast<const ::apollo::common::Header*>(
      &::apollo::common::_Header_default_instance_);
}
inline const ::apollo::common::Header& TransformStampeds::header() const {
  // @@protoc_insertion_point(field_get:apollo.transform.TransformStampeds.header)
  return _internal_header();
}
inline ::apollo::common::Header* TransformStampeds::release_header() {
  // @@protoc_insertion_point(field_release:apollo.transform.TransformStampeds.header)
  _has_bits_[0] &= ~0x00000001u;
  ::apollo::common::Header* temp = header_;
  header_ = nullptr;
  return temp;
}
inline ::apollo::common::Header* TransformStampeds::_internal_mutable_header() {
  _has_bits_[0] |= 0x00000001u;
  if (header_ == nullptr) {
    auto* p = CreateMaybeMessage<::apollo::common::Header>(GetArenaNoVirtual());
    header_ = p;
  }
  return header_;
}
inline ::apollo::common::Header* TransformStampeds::mutable_header() {
  // @@protoc_insertion_point(field_mutable:apollo.transform.TransformStampeds.header)
  return _internal_mutable_header();
}
inline void TransformStampeds::set_allocated_header(::apollo::common::Header* header) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(header_);
  }
  if (header) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      header = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, header, submessage_arena);
    }
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  header_ = header;
  // @@protoc_insertion_point(field_set_allocated:apollo.transform.TransformStampeds.header)
}

// repeated .apollo.transform.TransformStamped transforms = 2;
inline int TransformStampeds::_internal_transforms_size() const {
  return transforms_.size();
}
inline int TransformStampeds::transforms_size() const {
  return _internal_transforms_size();
}
inline void TransformStampeds::clear_transforms() {
  transforms_.Clear();
}
inline ::apollo::transform::TransformStamped* TransformStampeds::mutable_transforms(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.transform.TransformStampeds.transforms)
  return transforms_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::transform::TransformStamped >*
TransformStampeds::mutable_transforms() {
  // @@protoc_insertion_point(field_mutable_list:apollo.transform.TransformStampeds.transforms)
  return &transforms_;
}
inline const ::apollo::transform::TransformStamped& TransformStampeds::_internal_transforms(int index) const {
  return transforms_.Get(index);
}
inline const ::apollo::transform::TransformStamped& TransformStampeds::transforms(int index) const {
  // @@protoc_insertion_point(field_get:apollo.transform.TransformStampeds.transforms)
  return _internal_transforms(index);
}
inline ::apollo::transform::TransformStamped* TransformStampeds::_internal_add_transforms() {
  return transforms_.Add();
}
inline ::apollo::transform::TransformStamped* TransformStampeds::add_transforms() {
  // @@protoc_insertion_point(field_add:apollo.transform.TransformStampeds.transforms)
  return _internal_add_transforms();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::transform::TransformStamped >&
TransformStampeds::transforms() const {
  // @@protoc_insertion_point(field_list:apollo.transform.TransformStampeds.transforms)
  return transforms_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------

// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace transform
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2ftransform_2fproto_2ftransform_2eproto