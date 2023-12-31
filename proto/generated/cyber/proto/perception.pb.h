// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cyber/proto/perception.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_cyber_2fproto_2fperception_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_cyber_2fproto_2fperception_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_cyber_2fproto_2fperception_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_cyber_2fproto_2fperception_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_cyber_2fproto_2fperception_2eproto;
namespace apollo {
namespace cyber {
namespace proto {
class Perception;
class PerceptionDefaultTypeInternal;
extern PerceptionDefaultTypeInternal _Perception_default_instance_;
class Perception_Header;
class Perception_HeaderDefaultTypeInternal;
extern Perception_HeaderDefaultTypeInternal _Perception_Header_default_instance_;
}  // namespace proto
}  // namespace cyber
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::cyber::proto::Perception* Arena::CreateMaybeMessage<::apollo::cyber::proto::Perception>(Arena*);
template<> ::apollo::cyber::proto::Perception_Header* Arena::CreateMaybeMessage<::apollo::cyber::proto::Perception_Header>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace cyber {
namespace proto {

// ===================================================================

class Perception_Header :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.cyber.proto.Perception.Header) */ {
 public:
  Perception_Header();
  virtual ~Perception_Header();

  Perception_Header(const Perception_Header& from);
  Perception_Header(Perception_Header&& from) noexcept
    : Perception_Header() {
    *this = ::std::move(from);
  }

  inline Perception_Header& operator=(const Perception_Header& from) {
    CopyFrom(from);
    return *this;
  }
  inline Perception_Header& operator=(Perception_Header&& from) noexcept {
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
  static const Perception_Header& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Perception_Header* internal_default_instance() {
    return reinterpret_cast<const Perception_Header*>(
               &_Perception_Header_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(Perception_Header& a, Perception_Header& b) {
    a.Swap(&b);
  }
  inline void Swap(Perception_Header* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline Perception_Header* New() const final {
    return CreateMaybeMessage<Perception_Header>(nullptr);
  }

  Perception_Header* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<Perception_Header>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const Perception_Header& from);
  void MergeFrom(const Perception_Header& from);
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
  void InternalSwap(Perception_Header* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.cyber.proto.Perception.Header";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_cyber_2fproto_2fperception_2eproto);
    return ::descriptor_table_cyber_2fproto_2fperception_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kTimestampFieldNumber = 1,
  };
  // optional uint64 timestamp = 1;
  bool has_timestamp() const;
  private:
  bool _internal_has_timestamp() const;
  public:
  void clear_timestamp();
  ::PROTOBUF_NAMESPACE_ID::uint64 timestamp() const;
  void set_timestamp(::PROTOBUF_NAMESPACE_ID::uint64 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint64 _internal_timestamp() const;
  void _internal_set_timestamp(::PROTOBUF_NAMESPACE_ID::uint64 value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.cyber.proto.Perception.Header)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::uint64 timestamp_;
  friend struct ::TableStruct_cyber_2fproto_2fperception_2eproto;
};
// -------------------------------------------------------------------

class Perception :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.cyber.proto.Perception) */ {
 public:
  Perception();
  virtual ~Perception();

  Perception(const Perception& from);
  Perception(Perception&& from) noexcept
    : Perception() {
    *this = ::std::move(from);
  }

  inline Perception& operator=(const Perception& from) {
    CopyFrom(from);
    return *this;
  }
  inline Perception& operator=(Perception&& from) noexcept {
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
  static const Perception& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Perception* internal_default_instance() {
    return reinterpret_cast<const Perception*>(
               &_Perception_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(Perception& a, Perception& b) {
    a.Swap(&b);
  }
  inline void Swap(Perception* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline Perception* New() const final {
    return CreateMaybeMessage<Perception>(nullptr);
  }

  Perception* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<Perception>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const Perception& from);
  void MergeFrom(const Perception& from);
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
  void InternalSwap(Perception* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.cyber.proto.Perception";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_cyber_2fproto_2fperception_2eproto);
    return ::descriptor_table_cyber_2fproto_2fperception_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  typedef Perception_Header Header;

  // accessors -------------------------------------------------------

  enum : int {
    kHeaderFieldNumber = 1,
    kMsgIdFieldNumber = 2,
    kResultFieldNumber = 3,
  };
  // optional .apollo.cyber.proto.Perception.Header header = 1;
  bool has_header() const;
  private:
  bool _internal_has_header() const;
  public:
  void clear_header();
  const ::apollo::cyber::proto::Perception_Header& header() const;
  ::apollo::cyber::proto::Perception_Header* release_header();
  ::apollo::cyber::proto::Perception_Header* mutable_header();
  void set_allocated_header(::apollo::cyber::proto::Perception_Header* header);
  private:
  const ::apollo::cyber::proto::Perception_Header& _internal_header() const;
  ::apollo::cyber::proto::Perception_Header* _internal_mutable_header();
  public:

  // optional uint64 msg_id = 2;
  bool has_msg_id() const;
  private:
  bool _internal_has_msg_id() const;
  public:
  void clear_msg_id();
  ::PROTOBUF_NAMESPACE_ID::uint64 msg_id() const;
  void set_msg_id(::PROTOBUF_NAMESPACE_ID::uint64 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint64 _internal_msg_id() const;
  void _internal_set_msg_id(::PROTOBUF_NAMESPACE_ID::uint64 value);
  public:

  // optional double result = 3;
  bool has_result() const;
  private:
  bool _internal_has_result() const;
  public:
  void clear_result();
  double result() const;
  void set_result(double value);
  private:
  double _internal_result() const;
  void _internal_set_result(double value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.cyber.proto.Perception)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::apollo::cyber::proto::Perception_Header* header_;
  ::PROTOBUF_NAMESPACE_ID::uint64 msg_id_;
  double result_;
  friend struct ::TableStruct_cyber_2fproto_2fperception_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Perception_Header

// optional uint64 timestamp = 1;
inline bool Perception_Header::_internal_has_timestamp() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool Perception_Header::has_timestamp() const {
  return _internal_has_timestamp();
}
inline void Perception_Header::clear_timestamp() {
  timestamp_ = PROTOBUF_ULONGLONG(0);
  _has_bits_[0] &= ~0x00000001u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint64 Perception_Header::_internal_timestamp() const {
  return timestamp_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint64 Perception_Header::timestamp() const {
  // @@protoc_insertion_point(field_get:apollo.cyber.proto.Perception.Header.timestamp)
  return _internal_timestamp();
}
inline void Perception_Header::_internal_set_timestamp(::PROTOBUF_NAMESPACE_ID::uint64 value) {
  _has_bits_[0] |= 0x00000001u;
  timestamp_ = value;
}
inline void Perception_Header::set_timestamp(::PROTOBUF_NAMESPACE_ID::uint64 value) {
  _internal_set_timestamp(value);
  // @@protoc_insertion_point(field_set:apollo.cyber.proto.Perception.Header.timestamp)
}

// -------------------------------------------------------------------

// Perception

// optional .apollo.cyber.proto.Perception.Header header = 1;
inline bool Perception::_internal_has_header() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || header_ != nullptr);
  return value;
}
inline bool Perception::has_header() const {
  return _internal_has_header();
}
inline void Perception::clear_header() {
  if (header_ != nullptr) header_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
inline const ::apollo::cyber::proto::Perception_Header& Perception::_internal_header() const {
  const ::apollo::cyber::proto::Perception_Header* p = header_;
  return p != nullptr ? *p : *reinterpret_cast<const ::apollo::cyber::proto::Perception_Header*>(
      &::apollo::cyber::proto::_Perception_Header_default_instance_);
}
inline const ::apollo::cyber::proto::Perception_Header& Perception::header() const {
  // @@protoc_insertion_point(field_get:apollo.cyber.proto.Perception.header)
  return _internal_header();
}
inline ::apollo::cyber::proto::Perception_Header* Perception::release_header() {
  // @@protoc_insertion_point(field_release:apollo.cyber.proto.Perception.header)
  _has_bits_[0] &= ~0x00000001u;
  ::apollo::cyber::proto::Perception_Header* temp = header_;
  header_ = nullptr;
  return temp;
}
inline ::apollo::cyber::proto::Perception_Header* Perception::_internal_mutable_header() {
  _has_bits_[0] |= 0x00000001u;
  if (header_ == nullptr) {
    auto* p = CreateMaybeMessage<::apollo::cyber::proto::Perception_Header>(GetArenaNoVirtual());
    header_ = p;
  }
  return header_;
}
inline ::apollo::cyber::proto::Perception_Header* Perception::mutable_header() {
  // @@protoc_insertion_point(field_mutable:apollo.cyber.proto.Perception.header)
  return _internal_mutable_header();
}
inline void Perception::set_allocated_header(::apollo::cyber::proto::Perception_Header* header) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete header_;
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
  // @@protoc_insertion_point(field_set_allocated:apollo.cyber.proto.Perception.header)
}

// optional uint64 msg_id = 2;
inline bool Perception::_internal_has_msg_id() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool Perception::has_msg_id() const {
  return _internal_has_msg_id();
}
inline void Perception::clear_msg_id() {
  msg_id_ = PROTOBUF_ULONGLONG(0);
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint64 Perception::_internal_msg_id() const {
  return msg_id_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint64 Perception::msg_id() const {
  // @@protoc_insertion_point(field_get:apollo.cyber.proto.Perception.msg_id)
  return _internal_msg_id();
}
inline void Perception::_internal_set_msg_id(::PROTOBUF_NAMESPACE_ID::uint64 value) {
  _has_bits_[0] |= 0x00000002u;
  msg_id_ = value;
}
inline void Perception::set_msg_id(::PROTOBUF_NAMESPACE_ID::uint64 value) {
  _internal_set_msg_id(value);
  // @@protoc_insertion_point(field_set:apollo.cyber.proto.Perception.msg_id)
}

// optional double result = 3;
inline bool Perception::_internal_has_result() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool Perception::has_result() const {
  return _internal_has_result();
}
inline void Perception::clear_result() {
  result_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline double Perception::_internal_result() const {
  return result_;
}
inline double Perception::result() const {
  // @@protoc_insertion_point(field_get:apollo.cyber.proto.Perception.result)
  return _internal_result();
}
inline void Perception::_internal_set_result(double value) {
  _has_bits_[0] |= 0x00000004u;
  result_ = value;
}
inline void Perception::set_result(double value) {
  _internal_set_result(value);
  // @@protoc_insertion_point(field_set:apollo.cyber.proto.Perception.result)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace cyber
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_cyber_2fproto_2fperception_2eproto
