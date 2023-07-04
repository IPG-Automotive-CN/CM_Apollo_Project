// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/localization/proto/sins_pva.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2flocalization_2fproto_2fsins_5fpva_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2flocalization_2fproto_2fsins_5fpva_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_modules_2flocalization_2fproto_2fsins_5fpva_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2flocalization_2fproto_2fsins_5fpva_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2flocalization_2fproto_2fsins_5fpva_2eproto;
namespace apollo {
namespace localization {
class IntegSinsPva;
class IntegSinsPvaDefaultTypeInternal;
extern IntegSinsPvaDefaultTypeInternal _IntegSinsPva_default_instance_;
}  // namespace localization
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::localization::IntegSinsPva* Arena::CreateMaybeMessage<::apollo::localization::IntegSinsPva>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace localization {

// ===================================================================

class IntegSinsPva :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.localization.IntegSinsPva) */ {
 public:
  IntegSinsPva();
  virtual ~IntegSinsPva();

  IntegSinsPva(const IntegSinsPva& from);
  IntegSinsPva(IntegSinsPva&& from) noexcept
    : IntegSinsPva() {
    *this = ::std::move(from);
  }

  inline IntegSinsPva& operator=(const IntegSinsPva& from) {
    CopyFrom(from);
    return *this;
  }
  inline IntegSinsPva& operator=(IntegSinsPva&& from) noexcept {
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
  static const IntegSinsPva& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const IntegSinsPva* internal_default_instance() {
    return reinterpret_cast<const IntegSinsPva*>(
               &_IntegSinsPva_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(IntegSinsPva& a, IntegSinsPva& b) {
    a.Swap(&b);
  }
  inline void Swap(IntegSinsPva* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline IntegSinsPva* New() const final {
    return CreateMaybeMessage<IntegSinsPva>(nullptr);
  }

  IntegSinsPva* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<IntegSinsPva>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const IntegSinsPva& from);
  void MergeFrom(const IntegSinsPva& from);
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
  void InternalSwap(IntegSinsPva* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.localization.IntegSinsPva";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2flocalization_2fproto_2fsins_5fpva_2eproto);
    return ::descriptor_table_modules_2flocalization_2fproto_2fsins_5fpva_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kPvaCovarFieldNumber = 5,
    kHeaderFieldNumber = 1,
    kPositionFieldNumber = 2,
    kVelocityFieldNumber = 3,
    kAttitudeFieldNumber = 4,
    kInitAndAlignmentFieldNumber = 6,
  };
  // repeated double pva_covar = 5 [packed = true];
  int pva_covar_size() const;
  private:
  int _internal_pva_covar_size() const;
  public:
  void clear_pva_covar();
  private:
  double _internal_pva_covar(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      _internal_pva_covar() const;
  void _internal_add_pva_covar(double value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      _internal_mutable_pva_covar();
  public:
  double pva_covar(int index) const;
  void set_pva_covar(int index, double value);
  void add_pva_covar(double value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      pva_covar() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      mutable_pva_covar();

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

  // optional .apollo.common.PointLLH position = 2;
  bool has_position() const;
  private:
  bool _internal_has_position() const;
  public:
  void clear_position();
  const ::apollo::common::PointLLH& position() const;
  ::apollo::common::PointLLH* release_position();
  ::apollo::common::PointLLH* mutable_position();
  void set_allocated_position(::apollo::common::PointLLH* position);
  private:
  const ::apollo::common::PointLLH& _internal_position() const;
  ::apollo::common::PointLLH* _internal_mutable_position();
  public:

  // optional .apollo.common.Point3D velocity = 3;
  bool has_velocity() const;
  private:
  bool _internal_has_velocity() const;
  public:
  void clear_velocity();
  const ::apollo::common::Point3D& velocity() const;
  ::apollo::common::Point3D* release_velocity();
  ::apollo::common::Point3D* mutable_velocity();
  void set_allocated_velocity(::apollo::common::Point3D* velocity);
  private:
  const ::apollo::common::Point3D& _internal_velocity() const;
  ::apollo::common::Point3D* _internal_mutable_velocity();
  public:

  // optional .apollo.common.Point3D attitude = 4;
  bool has_attitude() const;
  private:
  bool _internal_has_attitude() const;
  public:
  void clear_attitude();
  const ::apollo::common::Point3D& attitude() const;
  ::apollo::common::Point3D* release_attitude();
  ::apollo::common::Point3D* mutable_attitude();
  void set_allocated_attitude(::apollo::common::Point3D* attitude);
  private:
  const ::apollo::common::Point3D& _internal_attitude() const;
  ::apollo::common::Point3D* _internal_mutable_attitude();
  public:

  // optional bool init_and_alignment = 6;
  bool has_init_and_alignment() const;
  private:
  bool _internal_has_init_and_alignment() const;
  public:
  void clear_init_and_alignment();
  bool init_and_alignment() const;
  void set_init_and_alignment(bool value);
  private:
  bool _internal_init_and_alignment() const;
  void _internal_set_init_and_alignment(bool value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.localization.IntegSinsPva)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double > pva_covar_;
  mutable std::atomic<int> _pva_covar_cached_byte_size_;
  ::apollo::common::Header* header_;
  ::apollo::common::PointLLH* position_;
  ::apollo::common::Point3D* velocity_;
  ::apollo::common::Point3D* attitude_;
  bool init_and_alignment_;
  friend struct ::TableStruct_modules_2flocalization_2fproto_2fsins_5fpva_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// IntegSinsPva

// optional .apollo.common.Header header = 1;
inline bool IntegSinsPva::_internal_has_header() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || header_ != nullptr);
  return value;
}
inline bool IntegSinsPva::has_header() const {
  return _internal_has_header();
}
inline const ::apollo::common::Header& IntegSinsPva::_internal_header() const {
  const ::apollo::common::Header* p = header_;
  return p != nullptr ? *p : *reinterpret_cast<const ::apollo::common::Header*>(
      &::apollo::common::_Header_default_instance_);
}
inline const ::apollo::common::Header& IntegSinsPva::header() const {
  // @@protoc_insertion_point(field_get:apollo.localization.IntegSinsPva.header)
  return _internal_header();
}
inline ::apollo::common::Header* IntegSinsPva::release_header() {
  // @@protoc_insertion_point(field_release:apollo.localization.IntegSinsPva.header)
  _has_bits_[0] &= ~0x00000001u;
  ::apollo::common::Header* temp = header_;
  header_ = nullptr;
  return temp;
}
inline ::apollo::common::Header* IntegSinsPva::_internal_mutable_header() {
  _has_bits_[0] |= 0x00000001u;
  if (header_ == nullptr) {
    auto* p = CreateMaybeMessage<::apollo::common::Header>(GetArenaNoVirtual());
    header_ = p;
  }
  return header_;
}
inline ::apollo::common::Header* IntegSinsPva::mutable_header() {
  // @@protoc_insertion_point(field_mutable:apollo.localization.IntegSinsPva.header)
  return _internal_mutable_header();
}
inline void IntegSinsPva::set_allocated_header(::apollo::common::Header* header) {
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
  // @@protoc_insertion_point(field_set_allocated:apollo.localization.IntegSinsPva.header)
}

// optional .apollo.common.PointLLH position = 2;
inline bool IntegSinsPva::_internal_has_position() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  PROTOBUF_ASSUME(!value || position_ != nullptr);
  return value;
}
inline bool IntegSinsPva::has_position() const {
  return _internal_has_position();
}
inline const ::apollo::common::PointLLH& IntegSinsPva::_internal_position() const {
  const ::apollo::common::PointLLH* p = position_;
  return p != nullptr ? *p : *reinterpret_cast<const ::apollo::common::PointLLH*>(
      &::apollo::common::_PointLLH_default_instance_);
}
inline const ::apollo::common::PointLLH& IntegSinsPva::position() const {
  // @@protoc_insertion_point(field_get:apollo.localization.IntegSinsPva.position)
  return _internal_position();
}
inline ::apollo::common::PointLLH* IntegSinsPva::release_position() {
  // @@protoc_insertion_point(field_release:apollo.localization.IntegSinsPva.position)
  _has_bits_[0] &= ~0x00000002u;
  ::apollo::common::PointLLH* temp = position_;
  position_ = nullptr;
  return temp;
}
inline ::apollo::common::PointLLH* IntegSinsPva::_internal_mutable_position() {
  _has_bits_[0] |= 0x00000002u;
  if (position_ == nullptr) {
    auto* p = CreateMaybeMessage<::apollo::common::PointLLH>(GetArenaNoVirtual());
    position_ = p;
  }
  return position_;
}
inline ::apollo::common::PointLLH* IntegSinsPva::mutable_position() {
  // @@protoc_insertion_point(field_mutable:apollo.localization.IntegSinsPva.position)
  return _internal_mutable_position();
}
inline void IntegSinsPva::set_allocated_position(::apollo::common::PointLLH* position) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(position_);
  }
  if (position) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      position = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, position, submessage_arena);
    }
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  position_ = position;
  // @@protoc_insertion_point(field_set_allocated:apollo.localization.IntegSinsPva.position)
}

// optional .apollo.common.Point3D velocity = 3;
inline bool IntegSinsPva::_internal_has_velocity() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  PROTOBUF_ASSUME(!value || velocity_ != nullptr);
  return value;
}
inline bool IntegSinsPva::has_velocity() const {
  return _internal_has_velocity();
}
inline const ::apollo::common::Point3D& IntegSinsPva::_internal_velocity() const {
  const ::apollo::common::Point3D* p = velocity_;
  return p != nullptr ? *p : *reinterpret_cast<const ::apollo::common::Point3D*>(
      &::apollo::common::_Point3D_default_instance_);
}
inline const ::apollo::common::Point3D& IntegSinsPva::velocity() const {
  // @@protoc_insertion_point(field_get:apollo.localization.IntegSinsPva.velocity)
  return _internal_velocity();
}
inline ::apollo::common::Point3D* IntegSinsPva::release_velocity() {
  // @@protoc_insertion_point(field_release:apollo.localization.IntegSinsPva.velocity)
  _has_bits_[0] &= ~0x00000004u;
  ::apollo::common::Point3D* temp = velocity_;
  velocity_ = nullptr;
  return temp;
}
inline ::apollo::common::Point3D* IntegSinsPva::_internal_mutable_velocity() {
  _has_bits_[0] |= 0x00000004u;
  if (velocity_ == nullptr) {
    auto* p = CreateMaybeMessage<::apollo::common::Point3D>(GetArenaNoVirtual());
    velocity_ = p;
  }
  return velocity_;
}
inline ::apollo::common::Point3D* IntegSinsPva::mutable_velocity() {
  // @@protoc_insertion_point(field_mutable:apollo.localization.IntegSinsPva.velocity)
  return _internal_mutable_velocity();
}
inline void IntegSinsPva::set_allocated_velocity(::apollo::common::Point3D* velocity) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(velocity_);
  }
  if (velocity) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      velocity = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, velocity, submessage_arena);
    }
    _has_bits_[0] |= 0x00000004u;
  } else {
    _has_bits_[0] &= ~0x00000004u;
  }
  velocity_ = velocity;
  // @@protoc_insertion_point(field_set_allocated:apollo.localization.IntegSinsPva.velocity)
}

// optional .apollo.common.Point3D attitude = 4;
inline bool IntegSinsPva::_internal_has_attitude() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  PROTOBUF_ASSUME(!value || attitude_ != nullptr);
  return value;
}
inline bool IntegSinsPva::has_attitude() const {
  return _internal_has_attitude();
}
inline const ::apollo::common::Point3D& IntegSinsPva::_internal_attitude() const {
  const ::apollo::common::Point3D* p = attitude_;
  return p != nullptr ? *p : *reinterpret_cast<const ::apollo::common::Point3D*>(
      &::apollo::common::_Point3D_default_instance_);
}
inline const ::apollo::common::Point3D& IntegSinsPva::attitude() const {
  // @@protoc_insertion_point(field_get:apollo.localization.IntegSinsPva.attitude)
  return _internal_attitude();
}
inline ::apollo::common::Point3D* IntegSinsPva::release_attitude() {
  // @@protoc_insertion_point(field_release:apollo.localization.IntegSinsPva.attitude)
  _has_bits_[0] &= ~0x00000008u;
  ::apollo::common::Point3D* temp = attitude_;
  attitude_ = nullptr;
  return temp;
}
inline ::apollo::common::Point3D* IntegSinsPva::_internal_mutable_attitude() {
  _has_bits_[0] |= 0x00000008u;
  if (attitude_ == nullptr) {
    auto* p = CreateMaybeMessage<::apollo::common::Point3D>(GetArenaNoVirtual());
    attitude_ = p;
  }
  return attitude_;
}
inline ::apollo::common::Point3D* IntegSinsPva::mutable_attitude() {
  // @@protoc_insertion_point(field_mutable:apollo.localization.IntegSinsPva.attitude)
  return _internal_mutable_attitude();
}
inline void IntegSinsPva::set_allocated_attitude(::apollo::common::Point3D* attitude) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(attitude_);
  }
  if (attitude) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      attitude = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, attitude, submessage_arena);
    }
    _has_bits_[0] |= 0x00000008u;
  } else {
    _has_bits_[0] &= ~0x00000008u;
  }
  attitude_ = attitude;
  // @@protoc_insertion_point(field_set_allocated:apollo.localization.IntegSinsPva.attitude)
}

// repeated double pva_covar = 5 [packed = true];
inline int IntegSinsPva::_internal_pva_covar_size() const {
  return pva_covar_.size();
}
inline int IntegSinsPva::pva_covar_size() const {
  return _internal_pva_covar_size();
}
inline void IntegSinsPva::clear_pva_covar() {
  pva_covar_.Clear();
}
inline double IntegSinsPva::_internal_pva_covar(int index) const {
  return pva_covar_.Get(index);
}
inline double IntegSinsPva::pva_covar(int index) const {
  // @@protoc_insertion_point(field_get:apollo.localization.IntegSinsPva.pva_covar)
  return _internal_pva_covar(index);
}
inline void IntegSinsPva::set_pva_covar(int index, double value) {
  pva_covar_.Set(index, value);
  // @@protoc_insertion_point(field_set:apollo.localization.IntegSinsPva.pva_covar)
}
inline void IntegSinsPva::_internal_add_pva_covar(double value) {
  pva_covar_.Add(value);
}
inline void IntegSinsPva::add_pva_covar(double value) {
  _internal_add_pva_covar(value);
  // @@protoc_insertion_point(field_add:apollo.localization.IntegSinsPva.pva_covar)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
IntegSinsPva::_internal_pva_covar() const {
  return pva_covar_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
IntegSinsPva::pva_covar() const {
  // @@protoc_insertion_point(field_list:apollo.localization.IntegSinsPva.pva_covar)
  return _internal_pva_covar();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
IntegSinsPva::_internal_mutable_pva_covar() {
  return &pva_covar_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
IntegSinsPva::mutable_pva_covar() {
  // @@protoc_insertion_point(field_mutable_list:apollo.localization.IntegSinsPva.pva_covar)
  return _internal_mutable_pva_covar();
}

// optional bool init_and_alignment = 6;
inline bool IntegSinsPva::_internal_has_init_and_alignment() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool IntegSinsPva::has_init_and_alignment() const {
  return _internal_has_init_and_alignment();
}
inline void IntegSinsPva::clear_init_and_alignment() {
  init_and_alignment_ = false;
  _has_bits_[0] &= ~0x00000010u;
}
inline bool IntegSinsPva::_internal_init_and_alignment() const {
  return init_and_alignment_;
}
inline bool IntegSinsPva::init_and_alignment() const {
  // @@protoc_insertion_point(field_get:apollo.localization.IntegSinsPva.init_and_alignment)
  return _internal_init_and_alignment();
}
inline void IntegSinsPva::_internal_set_init_and_alignment(bool value) {
  _has_bits_[0] |= 0x00000010u;
  init_and_alignment_ = value;
}
inline void IntegSinsPva::set_init_and_alignment(bool value) {
  _internal_set_init_and_alignment(value);
  // @@protoc_insertion_point(field_set:apollo.localization.IntegSinsPva.init_and_alignment)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace localization
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2flocalization_2fproto_2fsins_5fpva_2eproto
