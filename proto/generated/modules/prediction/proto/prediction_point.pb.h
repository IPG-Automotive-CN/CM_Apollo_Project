// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/prediction/proto/prediction_point.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2fprediction_2fproto_2fprediction_5fpoint_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2fprediction_2fproto_2fprediction_5fpoint_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_modules_2fprediction_2fproto_2fprediction_5fpoint_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2fprediction_2fproto_2fprediction_5fpoint_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fprediction_2fproto_2fprediction_5fpoint_2eproto;
namespace apollo {
namespace prediction {
class PredictionPathPoint;
class PredictionPathPointDefaultTypeInternal;
extern PredictionPathPointDefaultTypeInternal _PredictionPathPoint_default_instance_;
class PredictionTrajectoryPoint;
class PredictionTrajectoryPointDefaultTypeInternal;
extern PredictionTrajectoryPointDefaultTypeInternal _PredictionTrajectoryPoint_default_instance_;
}  // namespace prediction
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::prediction::PredictionPathPoint* Arena::CreateMaybeMessage<::apollo::prediction::PredictionPathPoint>(Arena*);
template<> ::apollo::prediction::PredictionTrajectoryPoint* Arena::CreateMaybeMessage<::apollo::prediction::PredictionTrajectoryPoint>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace prediction {

// ===================================================================

class PredictionPathPoint :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.prediction.PredictionPathPoint) */ {
 public:
  PredictionPathPoint();
  virtual ~PredictionPathPoint();

  PredictionPathPoint(const PredictionPathPoint& from);
  PredictionPathPoint(PredictionPathPoint&& from) noexcept
    : PredictionPathPoint() {
    *this = ::std::move(from);
  }

  inline PredictionPathPoint& operator=(const PredictionPathPoint& from) {
    CopyFrom(from);
    return *this;
  }
  inline PredictionPathPoint& operator=(PredictionPathPoint&& from) noexcept {
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
  static const PredictionPathPoint& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const PredictionPathPoint* internal_default_instance() {
    return reinterpret_cast<const PredictionPathPoint*>(
               &_PredictionPathPoint_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(PredictionPathPoint& a, PredictionPathPoint& b) {
    a.Swap(&b);
  }
  inline void Swap(PredictionPathPoint* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline PredictionPathPoint* New() const final {
    return CreateMaybeMessage<PredictionPathPoint>(nullptr);
  }

  PredictionPathPoint* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<PredictionPathPoint>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const PredictionPathPoint& from);
  void MergeFrom(const PredictionPathPoint& from);
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
  void InternalSwap(PredictionPathPoint* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.prediction.PredictionPathPoint";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2fprediction_2fproto_2fprediction_5fpoint_2eproto);
    return ::descriptor_table_modules_2fprediction_2fproto_2fprediction_5fpoint_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kXFieldNumber = 1,
    kYFieldNumber = 2,
    kVelocityHeadingFieldNumber = 3,
  };
  // required double x = 1;
  bool has_x() const;
  private:
  bool _internal_has_x() const;
  public:
  void clear_x();
  double x() const;
  void set_x(double value);
  private:
  double _internal_x() const;
  void _internal_set_x(double value);
  public:

  // required double y = 2;
  bool has_y() const;
  private:
  bool _internal_has_y() const;
  public:
  void clear_y();
  double y() const;
  void set_y(double value);
  private:
  double _internal_y() const;
  void _internal_set_y(double value);
  public:

  // optional double velocity_heading = 3;
  bool has_velocity_heading() const;
  private:
  bool _internal_has_velocity_heading() const;
  public:
  void clear_velocity_heading();
  double velocity_heading() const;
  void set_velocity_heading(double value);
  private:
  double _internal_velocity_heading() const;
  void _internal_set_velocity_heading(double value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.prediction.PredictionPathPoint)
 private:
  class _Internal;

  // helper for ByteSizeLong()
  size_t RequiredFieldsByteSizeFallback() const;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  double x_;
  double y_;
  double velocity_heading_;
  friend struct ::TableStruct_modules_2fprediction_2fproto_2fprediction_5fpoint_2eproto;
};
// -------------------------------------------------------------------

class PredictionTrajectoryPoint :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.prediction.PredictionTrajectoryPoint) */ {
 public:
  PredictionTrajectoryPoint();
  virtual ~PredictionTrajectoryPoint();

  PredictionTrajectoryPoint(const PredictionTrajectoryPoint& from);
  PredictionTrajectoryPoint(PredictionTrajectoryPoint&& from) noexcept
    : PredictionTrajectoryPoint() {
    *this = ::std::move(from);
  }

  inline PredictionTrajectoryPoint& operator=(const PredictionTrajectoryPoint& from) {
    CopyFrom(from);
    return *this;
  }
  inline PredictionTrajectoryPoint& operator=(PredictionTrajectoryPoint&& from) noexcept {
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
  static const PredictionTrajectoryPoint& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const PredictionTrajectoryPoint* internal_default_instance() {
    return reinterpret_cast<const PredictionTrajectoryPoint*>(
               &_PredictionTrajectoryPoint_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(PredictionTrajectoryPoint& a, PredictionTrajectoryPoint& b) {
    a.Swap(&b);
  }
  inline void Swap(PredictionTrajectoryPoint* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline PredictionTrajectoryPoint* New() const final {
    return CreateMaybeMessage<PredictionTrajectoryPoint>(nullptr);
  }

  PredictionTrajectoryPoint* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<PredictionTrajectoryPoint>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const PredictionTrajectoryPoint& from);
  void MergeFrom(const PredictionTrajectoryPoint& from);
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
  void InternalSwap(PredictionTrajectoryPoint* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.prediction.PredictionTrajectoryPoint";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2fprediction_2fproto_2fprediction_5fpoint_2eproto);
    return ::descriptor_table_modules_2fprediction_2fproto_2fprediction_5fpoint_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kPathPointFieldNumber = 1,
    kTimestampFieldNumber = 2,
  };
  // required .apollo.prediction.PredictionPathPoint path_point = 1;
  bool has_path_point() const;
  private:
  bool _internal_has_path_point() const;
  public:
  void clear_path_point();
  const ::apollo::prediction::PredictionPathPoint& path_point() const;
  ::apollo::prediction::PredictionPathPoint* release_path_point();
  ::apollo::prediction::PredictionPathPoint* mutable_path_point();
  void set_allocated_path_point(::apollo::prediction::PredictionPathPoint* path_point);
  private:
  const ::apollo::prediction::PredictionPathPoint& _internal_path_point() const;
  ::apollo::prediction::PredictionPathPoint* _internal_mutable_path_point();
  public:

  // required double timestamp = 2;
  bool has_timestamp() const;
  private:
  bool _internal_has_timestamp() const;
  public:
  void clear_timestamp();
  double timestamp() const;
  void set_timestamp(double value);
  private:
  double _internal_timestamp() const;
  void _internal_set_timestamp(double value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.prediction.PredictionTrajectoryPoint)
 private:
  class _Internal;

  // helper for ByteSizeLong()
  size_t RequiredFieldsByteSizeFallback() const;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::apollo::prediction::PredictionPathPoint* path_point_;
  double timestamp_;
  friend struct ::TableStruct_modules_2fprediction_2fproto_2fprediction_5fpoint_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// PredictionPathPoint

// required double x = 1;
inline bool PredictionPathPoint::_internal_has_x() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool PredictionPathPoint::has_x() const {
  return _internal_has_x();
}
inline void PredictionPathPoint::clear_x() {
  x_ = 0;
  _has_bits_[0] &= ~0x00000001u;
}
inline double PredictionPathPoint::_internal_x() const {
  return x_;
}
inline double PredictionPathPoint::x() const {
  // @@protoc_insertion_point(field_get:apollo.prediction.PredictionPathPoint.x)
  return _internal_x();
}
inline void PredictionPathPoint::_internal_set_x(double value) {
  _has_bits_[0] |= 0x00000001u;
  x_ = value;
}
inline void PredictionPathPoint::set_x(double value) {
  _internal_set_x(value);
  // @@protoc_insertion_point(field_set:apollo.prediction.PredictionPathPoint.x)
}

// required double y = 2;
inline bool PredictionPathPoint::_internal_has_y() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool PredictionPathPoint::has_y() const {
  return _internal_has_y();
}
inline void PredictionPathPoint::clear_y() {
  y_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline double PredictionPathPoint::_internal_y() const {
  return y_;
}
inline double PredictionPathPoint::y() const {
  // @@protoc_insertion_point(field_get:apollo.prediction.PredictionPathPoint.y)
  return _internal_y();
}
inline void PredictionPathPoint::_internal_set_y(double value) {
  _has_bits_[0] |= 0x00000002u;
  y_ = value;
}
inline void PredictionPathPoint::set_y(double value) {
  _internal_set_y(value);
  // @@protoc_insertion_point(field_set:apollo.prediction.PredictionPathPoint.y)
}

// optional double velocity_heading = 3;
inline bool PredictionPathPoint::_internal_has_velocity_heading() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool PredictionPathPoint::has_velocity_heading() const {
  return _internal_has_velocity_heading();
}
inline void PredictionPathPoint::clear_velocity_heading() {
  velocity_heading_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline double PredictionPathPoint::_internal_velocity_heading() const {
  return velocity_heading_;
}
inline double PredictionPathPoint::velocity_heading() const {
  // @@protoc_insertion_point(field_get:apollo.prediction.PredictionPathPoint.velocity_heading)
  return _internal_velocity_heading();
}
inline void PredictionPathPoint::_internal_set_velocity_heading(double value) {
  _has_bits_[0] |= 0x00000004u;
  velocity_heading_ = value;
}
inline void PredictionPathPoint::set_velocity_heading(double value) {
  _internal_set_velocity_heading(value);
  // @@protoc_insertion_point(field_set:apollo.prediction.PredictionPathPoint.velocity_heading)
}

// -------------------------------------------------------------------

// PredictionTrajectoryPoint

// required .apollo.prediction.PredictionPathPoint path_point = 1;
inline bool PredictionTrajectoryPoint::_internal_has_path_point() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || path_point_ != nullptr);
  return value;
}
inline bool PredictionTrajectoryPoint::has_path_point() const {
  return _internal_has_path_point();
}
inline void PredictionTrajectoryPoint::clear_path_point() {
  if (path_point_ != nullptr) path_point_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
inline const ::apollo::prediction::PredictionPathPoint& PredictionTrajectoryPoint::_internal_path_point() const {
  const ::apollo::prediction::PredictionPathPoint* p = path_point_;
  return p != nullptr ? *p : *reinterpret_cast<const ::apollo::prediction::PredictionPathPoint*>(
      &::apollo::prediction::_PredictionPathPoint_default_instance_);
}
inline const ::apollo::prediction::PredictionPathPoint& PredictionTrajectoryPoint::path_point() const {
  // @@protoc_insertion_point(field_get:apollo.prediction.PredictionTrajectoryPoint.path_point)
  return _internal_path_point();
}
inline ::apollo::prediction::PredictionPathPoint* PredictionTrajectoryPoint::release_path_point() {
  // @@protoc_insertion_point(field_release:apollo.prediction.PredictionTrajectoryPoint.path_point)
  _has_bits_[0] &= ~0x00000001u;
  ::apollo::prediction::PredictionPathPoint* temp = path_point_;
  path_point_ = nullptr;
  return temp;
}
inline ::apollo::prediction::PredictionPathPoint* PredictionTrajectoryPoint::_internal_mutable_path_point() {
  _has_bits_[0] |= 0x00000001u;
  if (path_point_ == nullptr) {
    auto* p = CreateMaybeMessage<::apollo::prediction::PredictionPathPoint>(GetArenaNoVirtual());
    path_point_ = p;
  }
  return path_point_;
}
inline ::apollo::prediction::PredictionPathPoint* PredictionTrajectoryPoint::mutable_path_point() {
  // @@protoc_insertion_point(field_mutable:apollo.prediction.PredictionTrajectoryPoint.path_point)
  return _internal_mutable_path_point();
}
inline void PredictionTrajectoryPoint::set_allocated_path_point(::apollo::prediction::PredictionPathPoint* path_point) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete path_point_;
  }
  if (path_point) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      path_point = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, path_point, submessage_arena);
    }
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  path_point_ = path_point;
  // @@protoc_insertion_point(field_set_allocated:apollo.prediction.PredictionTrajectoryPoint.path_point)
}

// required double timestamp = 2;
inline bool PredictionTrajectoryPoint::_internal_has_timestamp() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool PredictionTrajectoryPoint::has_timestamp() const {
  return _internal_has_timestamp();
}
inline void PredictionTrajectoryPoint::clear_timestamp() {
  timestamp_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline double PredictionTrajectoryPoint::_internal_timestamp() const {
  return timestamp_;
}
inline double PredictionTrajectoryPoint::timestamp() const {
  // @@protoc_insertion_point(field_get:apollo.prediction.PredictionTrajectoryPoint.timestamp)
  return _internal_timestamp();
}
inline void PredictionTrajectoryPoint::_internal_set_timestamp(double value) {
  _has_bits_[0] |= 0x00000002u;
  timestamp_ = value;
}
inline void PredictionTrajectoryPoint::set_timestamp(double value) {
  _internal_set_timestamp(value);
  // @@protoc_insertion_point(field_set:apollo.prediction.PredictionTrajectoryPoint.timestamp)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace prediction
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2fprediction_2fproto_2fprediction_5fpoint_2eproto
