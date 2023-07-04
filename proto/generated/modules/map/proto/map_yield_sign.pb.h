// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/map/proto/map_yield_sign.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2fmap_2fproto_2fmap_5fyield_5fsign_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2fmap_2fproto_2fmap_5fyield_5fsign_2eproto

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
#include "modules/map/proto/map_id.pb.h"
#include "modules/map/proto/map_geometry.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_modules_2fmap_2fproto_2fmap_5fyield_5fsign_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2fmap_2fproto_2fmap_5fyield_5fsign_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fmap_2fproto_2fmap_5fyield_5fsign_2eproto;
namespace apollo {
namespace hdmap {
class YieldSign;
class YieldSignDefaultTypeInternal;
extern YieldSignDefaultTypeInternal _YieldSign_default_instance_;
}  // namespace hdmap
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::hdmap::YieldSign* Arena::CreateMaybeMessage<::apollo::hdmap::YieldSign>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace hdmap {

// ===================================================================

class YieldSign :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.hdmap.YieldSign) */ {
 public:
  YieldSign();
  virtual ~YieldSign();

  YieldSign(const YieldSign& from);
  YieldSign(YieldSign&& from) noexcept
    : YieldSign() {
    *this = ::std::move(from);
  }

  inline YieldSign& operator=(const YieldSign& from) {
    CopyFrom(from);
    return *this;
  }
  inline YieldSign& operator=(YieldSign&& from) noexcept {
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
  static const YieldSign& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const YieldSign* internal_default_instance() {
    return reinterpret_cast<const YieldSign*>(
               &_YieldSign_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(YieldSign& a, YieldSign& b) {
    a.Swap(&b);
  }
  inline void Swap(YieldSign* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline YieldSign* New() const final {
    return CreateMaybeMessage<YieldSign>(nullptr);
  }

  YieldSign* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<YieldSign>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const YieldSign& from);
  void MergeFrom(const YieldSign& from);
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
  void InternalSwap(YieldSign* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.hdmap.YieldSign";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2fmap_2fproto_2fmap_5fyield_5fsign_2eproto);
    return ::descriptor_table_modules_2fmap_2fproto_2fmap_5fyield_5fsign_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kStopLineFieldNumber = 2,
    kOverlapIdFieldNumber = 3,
    kIdFieldNumber = 1,
  };
  // repeated .apollo.hdmap.Curve stop_line = 2;
  int stop_line_size() const;
  private:
  int _internal_stop_line_size() const;
  public:
  void clear_stop_line();
  ::apollo::hdmap::Curve* mutable_stop_line(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::hdmap::Curve >*
      mutable_stop_line();
  private:
  const ::apollo::hdmap::Curve& _internal_stop_line(int index) const;
  ::apollo::hdmap::Curve* _internal_add_stop_line();
  public:
  const ::apollo::hdmap::Curve& stop_line(int index) const;
  ::apollo::hdmap::Curve* add_stop_line();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::hdmap::Curve >&
      stop_line() const;

  // repeated .apollo.hdmap.Id overlap_id = 3;
  int overlap_id_size() const;
  private:
  int _internal_overlap_id_size() const;
  public:
  void clear_overlap_id();
  ::apollo::hdmap::Id* mutable_overlap_id(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::hdmap::Id >*
      mutable_overlap_id();
  private:
  const ::apollo::hdmap::Id& _internal_overlap_id(int index) const;
  ::apollo::hdmap::Id* _internal_add_overlap_id();
  public:
  const ::apollo::hdmap::Id& overlap_id(int index) const;
  ::apollo::hdmap::Id* add_overlap_id();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::hdmap::Id >&
      overlap_id() const;

  // optional .apollo.hdmap.Id id = 1;
  bool has_id() const;
  private:
  bool _internal_has_id() const;
  public:
  void clear_id();
  const ::apollo::hdmap::Id& id() const;
  ::apollo::hdmap::Id* release_id();
  ::apollo::hdmap::Id* mutable_id();
  void set_allocated_id(::apollo::hdmap::Id* id);
  private:
  const ::apollo::hdmap::Id& _internal_id() const;
  ::apollo::hdmap::Id* _internal_mutable_id();
  public:

  // @@protoc_insertion_point(class_scope:apollo.hdmap.YieldSign)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::hdmap::Curve > stop_line_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::hdmap::Id > overlap_id_;
  ::apollo::hdmap::Id* id_;
  friend struct ::TableStruct_modules_2fmap_2fproto_2fmap_5fyield_5fsign_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// YieldSign

// optional .apollo.hdmap.Id id = 1;
inline bool YieldSign::_internal_has_id() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || id_ != nullptr);
  return value;
}
inline bool YieldSign::has_id() const {
  return _internal_has_id();
}
inline const ::apollo::hdmap::Id& YieldSign::_internal_id() const {
  const ::apollo::hdmap::Id* p = id_;
  return p != nullptr ? *p : *reinterpret_cast<const ::apollo::hdmap::Id*>(
      &::apollo::hdmap::_Id_default_instance_);
}
inline const ::apollo::hdmap::Id& YieldSign::id() const {
  // @@protoc_insertion_point(field_get:apollo.hdmap.YieldSign.id)
  return _internal_id();
}
inline ::apollo::hdmap::Id* YieldSign::release_id() {
  // @@protoc_insertion_point(field_release:apollo.hdmap.YieldSign.id)
  _has_bits_[0] &= ~0x00000001u;
  ::apollo::hdmap::Id* temp = id_;
  id_ = nullptr;
  return temp;
}
inline ::apollo::hdmap::Id* YieldSign::_internal_mutable_id() {
  _has_bits_[0] |= 0x00000001u;
  if (id_ == nullptr) {
    auto* p = CreateMaybeMessage<::apollo::hdmap::Id>(GetArenaNoVirtual());
    id_ = p;
  }
  return id_;
}
inline ::apollo::hdmap::Id* YieldSign::mutable_id() {
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.YieldSign.id)
  return _internal_mutable_id();
}
inline void YieldSign::set_allocated_id(::apollo::hdmap::Id* id) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(id_);
  }
  if (id) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      id = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, id, submessage_arena);
    }
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  id_ = id;
  // @@protoc_insertion_point(field_set_allocated:apollo.hdmap.YieldSign.id)
}

// repeated .apollo.hdmap.Curve stop_line = 2;
inline int YieldSign::_internal_stop_line_size() const {
  return stop_line_.size();
}
inline int YieldSign::stop_line_size() const {
  return _internal_stop_line_size();
}
inline ::apollo::hdmap::Curve* YieldSign::mutable_stop_line(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.YieldSign.stop_line)
  return stop_line_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::hdmap::Curve >*
YieldSign::mutable_stop_line() {
  // @@protoc_insertion_point(field_mutable_list:apollo.hdmap.YieldSign.stop_line)
  return &stop_line_;
}
inline const ::apollo::hdmap::Curve& YieldSign::_internal_stop_line(int index) const {
  return stop_line_.Get(index);
}
inline const ::apollo::hdmap::Curve& YieldSign::stop_line(int index) const {
  // @@protoc_insertion_point(field_get:apollo.hdmap.YieldSign.stop_line)
  return _internal_stop_line(index);
}
inline ::apollo::hdmap::Curve* YieldSign::_internal_add_stop_line() {
  return stop_line_.Add();
}
inline ::apollo::hdmap::Curve* YieldSign::add_stop_line() {
  // @@protoc_insertion_point(field_add:apollo.hdmap.YieldSign.stop_line)
  return _internal_add_stop_line();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::hdmap::Curve >&
YieldSign::stop_line() const {
  // @@protoc_insertion_point(field_list:apollo.hdmap.YieldSign.stop_line)
  return stop_line_;
}

// repeated .apollo.hdmap.Id overlap_id = 3;
inline int YieldSign::_internal_overlap_id_size() const {
  return overlap_id_.size();
}
inline int YieldSign::overlap_id_size() const {
  return _internal_overlap_id_size();
}
inline ::apollo::hdmap::Id* YieldSign::mutable_overlap_id(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.YieldSign.overlap_id)
  return overlap_id_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::hdmap::Id >*
YieldSign::mutable_overlap_id() {
  // @@protoc_insertion_point(field_mutable_list:apollo.hdmap.YieldSign.overlap_id)
  return &overlap_id_;
}
inline const ::apollo::hdmap::Id& YieldSign::_internal_overlap_id(int index) const {
  return overlap_id_.Get(index);
}
inline const ::apollo::hdmap::Id& YieldSign::overlap_id(int index) const {
  // @@protoc_insertion_point(field_get:apollo.hdmap.YieldSign.overlap_id)
  return _internal_overlap_id(index);
}
inline ::apollo::hdmap::Id* YieldSign::_internal_add_overlap_id() {
  return overlap_id_.Add();
}
inline ::apollo::hdmap::Id* YieldSign::add_overlap_id() {
  // @@protoc_insertion_point(field_add:apollo.hdmap.YieldSign.overlap_id)
  return _internal_add_overlap_id();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::hdmap::Id >&
YieldSign::overlap_id() const {
  // @@protoc_insertion_point(field_list:apollo.hdmap.YieldSign.overlap_id)
  return overlap_id_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace hdmap
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2fmap_2fproto_2fmap_5fyield_5fsign_2eproto
