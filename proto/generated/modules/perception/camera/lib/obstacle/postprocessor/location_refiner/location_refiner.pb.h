// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/perception/camera/lib/obstacle/postprocessor/location_refiner/location_refiner.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2fperception_2fcamera_2flib_2fobstacle_2fpostprocessor_2flocation_5frefiner_2flocation_5frefiner_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2fperception_2fcamera_2flib_2fobstacle_2fpostprocessor_2flocation_5frefiner_2flocation_5frefiner_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_modules_2fperception_2fcamera_2flib_2fobstacle_2fpostprocessor_2flocation_5frefiner_2flocation_5frefiner_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2fperception_2fcamera_2flib_2fobstacle_2fpostprocessor_2flocation_5frefiner_2flocation_5frefiner_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fperception_2fcamera_2flib_2fobstacle_2fpostprocessor_2flocation_5frefiner_2flocation_5frefiner_2eproto;
namespace apollo {
namespace perception {
namespace camera {
namespace location_refiner {
class LocationRefinerParam;
class LocationRefinerParamDefaultTypeInternal;
extern LocationRefinerParamDefaultTypeInternal _LocationRefinerParam_default_instance_;
}  // namespace location_refiner
}  // namespace camera
}  // namespace perception
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::perception::camera::location_refiner::LocationRefinerParam* Arena::CreateMaybeMessage<::apollo::perception::camera::location_refiner::LocationRefinerParam>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace perception {
namespace camera {
namespace location_refiner {

// ===================================================================

class LocationRefinerParam :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.perception.camera.location_refiner.LocationRefinerParam) */ {
 public:
  LocationRefinerParam();
  virtual ~LocationRefinerParam();

  LocationRefinerParam(const LocationRefinerParam& from);
  LocationRefinerParam(LocationRefinerParam&& from) noexcept
    : LocationRefinerParam() {
    *this = ::std::move(from);
  }

  inline LocationRefinerParam& operator=(const LocationRefinerParam& from) {
    CopyFrom(from);
    return *this;
  }
  inline LocationRefinerParam& operator=(LocationRefinerParam&& from) noexcept {
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
  static const LocationRefinerParam& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const LocationRefinerParam* internal_default_instance() {
    return reinterpret_cast<const LocationRefinerParam*>(
               &_LocationRefinerParam_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(LocationRefinerParam& a, LocationRefinerParam& b) {
    a.Swap(&b);
  }
  inline void Swap(LocationRefinerParam* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline LocationRefinerParam* New() const final {
    return CreateMaybeMessage<LocationRefinerParam>(nullptr);
  }

  LocationRefinerParam* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<LocationRefinerParam>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const LocationRefinerParam& from);
  void MergeFrom(const LocationRefinerParam& from);
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
  void InternalSwap(LocationRefinerParam* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.perception.camera.location_refiner.LocationRefinerParam";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2fperception_2fcamera_2flib_2fobstacle_2fpostprocessor_2flocation_5frefiner_2flocation_5frefiner_2eproto);
    return ::descriptor_table_modules_2fperception_2fcamera_2flib_2fobstacle_2fpostprocessor_2flocation_5frefiner_2flocation_5frefiner_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kMinDistToCameraFieldNumber = 1,
    kRoiH2BottomScaleFieldNumber = 2,
  };
  // optional float min_dist_to_camera = 1 [default = 30];
  bool has_min_dist_to_camera() const;
  private:
  bool _internal_has_min_dist_to_camera() const;
  public:
  void clear_min_dist_to_camera();
  float min_dist_to_camera() const;
  void set_min_dist_to_camera(float value);
  private:
  float _internal_min_dist_to_camera() const;
  void _internal_set_min_dist_to_camera(float value);
  public:

  // optional float roi_h2bottom_scale = 2 [default = 0.5];
  bool has_roi_h2bottom_scale() const;
  private:
  bool _internal_has_roi_h2bottom_scale() const;
  public:
  void clear_roi_h2bottom_scale();
  float roi_h2bottom_scale() const;
  void set_roi_h2bottom_scale(float value);
  private:
  float _internal_roi_h2bottom_scale() const;
  void _internal_set_roi_h2bottom_scale(float value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.perception.camera.location_refiner.LocationRefinerParam)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  float min_dist_to_camera_;
  float roi_h2bottom_scale_;
  friend struct ::TableStruct_modules_2fperception_2fcamera_2flib_2fobstacle_2fpostprocessor_2flocation_5frefiner_2flocation_5frefiner_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// LocationRefinerParam

// optional float min_dist_to_camera = 1 [default = 30];
inline bool LocationRefinerParam::_internal_has_min_dist_to_camera() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool LocationRefinerParam::has_min_dist_to_camera() const {
  return _internal_has_min_dist_to_camera();
}
inline void LocationRefinerParam::clear_min_dist_to_camera() {
  min_dist_to_camera_ = 30;
  _has_bits_[0] &= ~0x00000001u;
}
inline float LocationRefinerParam::_internal_min_dist_to_camera() const {
  return min_dist_to_camera_;
}
inline float LocationRefinerParam::min_dist_to_camera() const {
  // @@protoc_insertion_point(field_get:apollo.perception.camera.location_refiner.LocationRefinerParam.min_dist_to_camera)
  return _internal_min_dist_to_camera();
}
inline void LocationRefinerParam::_internal_set_min_dist_to_camera(float value) {
  _has_bits_[0] |= 0x00000001u;
  min_dist_to_camera_ = value;
}
inline void LocationRefinerParam::set_min_dist_to_camera(float value) {
  _internal_set_min_dist_to_camera(value);
  // @@protoc_insertion_point(field_set:apollo.perception.camera.location_refiner.LocationRefinerParam.min_dist_to_camera)
}

// optional float roi_h2bottom_scale = 2 [default = 0.5];
inline bool LocationRefinerParam::_internal_has_roi_h2bottom_scale() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool LocationRefinerParam::has_roi_h2bottom_scale() const {
  return _internal_has_roi_h2bottom_scale();
}
inline void LocationRefinerParam::clear_roi_h2bottom_scale() {
  roi_h2bottom_scale_ = 0.5f;
  _has_bits_[0] &= ~0x00000002u;
}
inline float LocationRefinerParam::_internal_roi_h2bottom_scale() const {
  return roi_h2bottom_scale_;
}
inline float LocationRefinerParam::roi_h2bottom_scale() const {
  // @@protoc_insertion_point(field_get:apollo.perception.camera.location_refiner.LocationRefinerParam.roi_h2bottom_scale)
  return _internal_roi_h2bottom_scale();
}
inline void LocationRefinerParam::_internal_set_roi_h2bottom_scale(float value) {
  _has_bits_[0] |= 0x00000002u;
  roi_h2bottom_scale_ = value;
}
inline void LocationRefinerParam::set_roi_h2bottom_scale(float value) {
  _internal_set_roi_h2bottom_scale(value);
  // @@protoc_insertion_point(field_set:apollo.perception.camera.location_refiner.LocationRefinerParam.roi_h2bottom_scale)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace location_refiner
}  // namespace camera
}  // namespace perception
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2fperception_2fcamera_2flib_2fobstacle_2fpostprocessor_2flocation_5frefiner_2flocation_5frefiner_2eproto
