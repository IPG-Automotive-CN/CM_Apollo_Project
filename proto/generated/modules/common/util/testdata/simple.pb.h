// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/common/util/testdata/simple.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2fcommon_2futil_2ftestdata_2fsimple_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2fcommon_2futil_2ftestdata_2fsimple_2eproto

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
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_modules_2fcommon_2futil_2ftestdata_2fsimple_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2fcommon_2futil_2ftestdata_2fsimple_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fcommon_2futil_2ftestdata_2fsimple_2eproto;
namespace apollo {
namespace common {
namespace util {
namespace test {
class SimpleMessage;
class SimpleMessageDefaultTypeInternal;
extern SimpleMessageDefaultTypeInternal _SimpleMessage_default_instance_;
class SimpleRepeatedMessage;
class SimpleRepeatedMessageDefaultTypeInternal;
extern SimpleRepeatedMessageDefaultTypeInternal _SimpleRepeatedMessage_default_instance_;
}  // namespace test
}  // namespace util
}  // namespace common
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::common::util::test::SimpleMessage* Arena::CreateMaybeMessage<::apollo::common::util::test::SimpleMessage>(Arena*);
template<> ::apollo::common::util::test::SimpleRepeatedMessage* Arena::CreateMaybeMessage<::apollo::common::util::test::SimpleRepeatedMessage>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace common {
namespace util {
namespace test {

// ===================================================================

class SimpleMessage :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.common.util.test.SimpleMessage) */ {
 public:
  SimpleMessage();
  virtual ~SimpleMessage();

  SimpleMessage(const SimpleMessage& from);
  SimpleMessage(SimpleMessage&& from) noexcept
    : SimpleMessage() {
    *this = ::std::move(from);
  }

  inline SimpleMessage& operator=(const SimpleMessage& from) {
    CopyFrom(from);
    return *this;
  }
  inline SimpleMessage& operator=(SimpleMessage&& from) noexcept {
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
  static const SimpleMessage& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const SimpleMessage* internal_default_instance() {
    return reinterpret_cast<const SimpleMessage*>(
               &_SimpleMessage_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(SimpleMessage& a, SimpleMessage& b) {
    a.Swap(&b);
  }
  inline void Swap(SimpleMessage* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline SimpleMessage* New() const final {
    return CreateMaybeMessage<SimpleMessage>(nullptr);
  }

  SimpleMessage* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<SimpleMessage>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const SimpleMessage& from);
  void MergeFrom(const SimpleMessage& from);
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
  void InternalSwap(SimpleMessage* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.common.util.test.SimpleMessage";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2fcommon_2futil_2ftestdata_2fsimple_2eproto);
    return ::descriptor_table_modules_2fcommon_2futil_2ftestdata_2fsimple_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kTextFieldNumber = 2,
    kHeaderFieldNumber = 3,
    kIntegerFieldNumber = 1,
  };
  // optional string text = 2;
  bool has_text() const;
  private:
  bool _internal_has_text() const;
  public:
  void clear_text();
  const std::string& text() const;
  void set_text(const std::string& value);
  void set_text(std::string&& value);
  void set_text(const char* value);
  void set_text(const char* value, size_t size);
  std::string* mutable_text();
  std::string* release_text();
  void set_allocated_text(std::string* text);
  private:
  const std::string& _internal_text() const;
  void _internal_set_text(const std::string& value);
  std::string* _internal_mutable_text();
  public:

  // optional .apollo.common.Header header = 3;
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

  // optional int32 integer = 1;
  bool has_integer() const;
  private:
  bool _internal_has_integer() const;
  public:
  void clear_integer();
  ::PROTOBUF_NAMESPACE_ID::int32 integer() const;
  void set_integer(::PROTOBUF_NAMESPACE_ID::int32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int32 _internal_integer() const;
  void _internal_set_integer(::PROTOBUF_NAMESPACE_ID::int32 value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.common.util.test.SimpleMessage)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr text_;
  ::apollo::common::Header* header_;
  ::PROTOBUF_NAMESPACE_ID::int32 integer_;
  friend struct ::TableStruct_modules_2fcommon_2futil_2ftestdata_2fsimple_2eproto;
};
// -------------------------------------------------------------------

class SimpleRepeatedMessage :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.common.util.test.SimpleRepeatedMessage) */ {
 public:
  SimpleRepeatedMessage();
  virtual ~SimpleRepeatedMessage();

  SimpleRepeatedMessage(const SimpleRepeatedMessage& from);
  SimpleRepeatedMessage(SimpleRepeatedMessage&& from) noexcept
    : SimpleRepeatedMessage() {
    *this = ::std::move(from);
  }

  inline SimpleRepeatedMessage& operator=(const SimpleRepeatedMessage& from) {
    CopyFrom(from);
    return *this;
  }
  inline SimpleRepeatedMessage& operator=(SimpleRepeatedMessage&& from) noexcept {
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
  static const SimpleRepeatedMessage& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const SimpleRepeatedMessage* internal_default_instance() {
    return reinterpret_cast<const SimpleRepeatedMessage*>(
               &_SimpleRepeatedMessage_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(SimpleRepeatedMessage& a, SimpleRepeatedMessage& b) {
    a.Swap(&b);
  }
  inline void Swap(SimpleRepeatedMessage* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline SimpleRepeatedMessage* New() const final {
    return CreateMaybeMessage<SimpleRepeatedMessage>(nullptr);
  }

  SimpleRepeatedMessage* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<SimpleRepeatedMessage>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const SimpleRepeatedMessage& from);
  void MergeFrom(const SimpleRepeatedMessage& from);
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
  void InternalSwap(SimpleRepeatedMessage* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.common.util.test.SimpleRepeatedMessage";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2fcommon_2futil_2ftestdata_2fsimple_2eproto);
    return ::descriptor_table_modules_2fcommon_2futil_2ftestdata_2fsimple_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kMessageFieldNumber = 1,
  };
  // repeated .apollo.common.util.test.SimpleMessage message = 1;
  int message_size() const;
  private:
  int _internal_message_size() const;
  public:
  void clear_message();
  ::apollo::common::util::test::SimpleMessage* mutable_message(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::common::util::test::SimpleMessage >*
      mutable_message();
  private:
  const ::apollo::common::util::test::SimpleMessage& _internal_message(int index) const;
  ::apollo::common::util::test::SimpleMessage* _internal_add_message();
  public:
  const ::apollo::common::util::test::SimpleMessage& message(int index) const;
  ::apollo::common::util::test::SimpleMessage* add_message();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::common::util::test::SimpleMessage >&
      message() const;

  // @@protoc_insertion_point(class_scope:apollo.common.util.test.SimpleRepeatedMessage)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::common::util::test::SimpleMessage > message_;
  friend struct ::TableStruct_modules_2fcommon_2futil_2ftestdata_2fsimple_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// SimpleMessage

// optional int32 integer = 1;
inline bool SimpleMessage::_internal_has_integer() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool SimpleMessage::has_integer() const {
  return _internal_has_integer();
}
inline void SimpleMessage::clear_integer() {
  integer_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 SimpleMessage::_internal_integer() const {
  return integer_;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 SimpleMessage::integer() const {
  // @@protoc_insertion_point(field_get:apollo.common.util.test.SimpleMessage.integer)
  return _internal_integer();
}
inline void SimpleMessage::_internal_set_integer(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00000004u;
  integer_ = value;
}
inline void SimpleMessage::set_integer(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _internal_set_integer(value);
  // @@protoc_insertion_point(field_set:apollo.common.util.test.SimpleMessage.integer)
}

// optional string text = 2;
inline bool SimpleMessage::_internal_has_text() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool SimpleMessage::has_text() const {
  return _internal_has_text();
}
inline void SimpleMessage::clear_text() {
  text_.ClearToEmptyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  _has_bits_[0] &= ~0x00000001u;
}
inline const std::string& SimpleMessage::text() const {
  // @@protoc_insertion_point(field_get:apollo.common.util.test.SimpleMessage.text)
  return _internal_text();
}
inline void SimpleMessage::set_text(const std::string& value) {
  _internal_set_text(value);
  // @@protoc_insertion_point(field_set:apollo.common.util.test.SimpleMessage.text)
}
inline std::string* SimpleMessage::mutable_text() {
  // @@protoc_insertion_point(field_mutable:apollo.common.util.test.SimpleMessage.text)
  return _internal_mutable_text();
}
inline const std::string& SimpleMessage::_internal_text() const {
  return text_.GetNoArena();
}
inline void SimpleMessage::_internal_set_text(const std::string& value) {
  _has_bits_[0] |= 0x00000001u;
  text_.SetNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), value);
}
inline void SimpleMessage::set_text(std::string&& value) {
  _has_bits_[0] |= 0x00000001u;
  text_.SetNoArena(
    &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:apollo.common.util.test.SimpleMessage.text)
}
inline void SimpleMessage::set_text(const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  _has_bits_[0] |= 0x00000001u;
  text_.SetNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:apollo.common.util.test.SimpleMessage.text)
}
inline void SimpleMessage::set_text(const char* value, size_t size) {
  _has_bits_[0] |= 0x00000001u;
  text_.SetNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:apollo.common.util.test.SimpleMessage.text)
}
inline std::string* SimpleMessage::_internal_mutable_text() {
  _has_bits_[0] |= 0x00000001u;
  return text_.MutableNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}
inline std::string* SimpleMessage::release_text() {
  // @@protoc_insertion_point(field_release:apollo.common.util.test.SimpleMessage.text)
  if (!_internal_has_text()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000001u;
  return text_.ReleaseNonDefaultNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}
inline void SimpleMessage::set_allocated_text(std::string* text) {
  if (text != nullptr) {
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  text_.SetAllocatedNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), text);
  // @@protoc_insertion_point(field_set_allocated:apollo.common.util.test.SimpleMessage.text)
}

// optional .apollo.common.Header header = 3;
inline bool SimpleMessage::_internal_has_header() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  PROTOBUF_ASSUME(!value || header_ != nullptr);
  return value;
}
inline bool SimpleMessage::has_header() const {
  return _internal_has_header();
}
inline const ::apollo::common::Header& SimpleMessage::_internal_header() const {
  const ::apollo::common::Header* p = header_;
  return p != nullptr ? *p : *reinterpret_cast<const ::apollo::common::Header*>(
      &::apollo::common::_Header_default_instance_);
}
inline const ::apollo::common::Header& SimpleMessage::header() const {
  // @@protoc_insertion_point(field_get:apollo.common.util.test.SimpleMessage.header)
  return _internal_header();
}
inline ::apollo::common::Header* SimpleMessage::release_header() {
  // @@protoc_insertion_point(field_release:apollo.common.util.test.SimpleMessage.header)
  _has_bits_[0] &= ~0x00000002u;
  ::apollo::common::Header* temp = header_;
  header_ = nullptr;
  return temp;
}
inline ::apollo::common::Header* SimpleMessage::_internal_mutable_header() {
  _has_bits_[0] |= 0x00000002u;
  if (header_ == nullptr) {
    auto* p = CreateMaybeMessage<::apollo::common::Header>(GetArenaNoVirtual());
    header_ = p;
  }
  return header_;
}
inline ::apollo::common::Header* SimpleMessage::mutable_header() {
  // @@protoc_insertion_point(field_mutable:apollo.common.util.test.SimpleMessage.header)
  return _internal_mutable_header();
}
inline void SimpleMessage::set_allocated_header(::apollo::common::Header* header) {
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
  // @@protoc_insertion_point(field_set_allocated:apollo.common.util.test.SimpleMessage.header)
}

// -------------------------------------------------------------------

// SimpleRepeatedMessage

// repeated .apollo.common.util.test.SimpleMessage message = 1;
inline int SimpleRepeatedMessage::_internal_message_size() const {
  return message_.size();
}
inline int SimpleRepeatedMessage::message_size() const {
  return _internal_message_size();
}
inline void SimpleRepeatedMessage::clear_message() {
  message_.Clear();
}
inline ::apollo::common::util::test::SimpleMessage* SimpleRepeatedMessage::mutable_message(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.common.util.test.SimpleRepeatedMessage.message)
  return message_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::common::util::test::SimpleMessage >*
SimpleRepeatedMessage::mutable_message() {
  // @@protoc_insertion_point(field_mutable_list:apollo.common.util.test.SimpleRepeatedMessage.message)
  return &message_;
}
inline const ::apollo::common::util::test::SimpleMessage& SimpleRepeatedMessage::_internal_message(int index) const {
  return message_.Get(index);
}
inline const ::apollo::common::util::test::SimpleMessage& SimpleRepeatedMessage::message(int index) const {
  // @@protoc_insertion_point(field_get:apollo.common.util.test.SimpleRepeatedMessage.message)
  return _internal_message(index);
}
inline ::apollo::common::util::test::SimpleMessage* SimpleRepeatedMessage::_internal_add_message() {
  return message_.Add();
}
inline ::apollo::common::util::test::SimpleMessage* SimpleRepeatedMessage::add_message() {
  // @@protoc_insertion_point(field_add:apollo.common.util.test.SimpleRepeatedMessage.message)
  return _internal_add_message();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::common::util::test::SimpleMessage >&
SimpleRepeatedMessage::message() const {
  // @@protoc_insertion_point(field_list:apollo.common.util.test.SimpleRepeatedMessage.message)
  return message_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace test
}  // namespace util
}  // namespace common
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2fcommon_2futil_2ftestdata_2fsimple_2eproto
