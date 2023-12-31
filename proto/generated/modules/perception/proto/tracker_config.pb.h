// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/perception/proto/tracker_config.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2fperception_2fproto_2ftracker_5fconfig_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2fperception_2fproto_2ftracker_5fconfig_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_modules_2fperception_2fproto_2ftracker_5fconfig_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2fperception_2fproto_2ftracker_5fconfig_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fperception_2fproto_2ftracker_5fconfig_2eproto;
class MatcherConfig;
class MatcherConfigDefaultTypeInternal;
extern MatcherConfigDefaultTypeInternal _MatcherConfig_default_instance_;
PROTOBUF_NAMESPACE_OPEN
template<> ::MatcherConfig* Arena::CreateMaybeMessage<::MatcherConfig>(Arena*);
PROTOBUF_NAMESPACE_CLOSE

// ===================================================================

class MatcherConfig :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:MatcherConfig) */ {
 public:
  MatcherConfig();
  virtual ~MatcherConfig();

  MatcherConfig(const MatcherConfig& from);
  MatcherConfig(MatcherConfig&& from) noexcept
    : MatcherConfig() {
    *this = ::std::move(from);
  }

  inline MatcherConfig& operator=(const MatcherConfig& from) {
    CopyFrom(from);
    return *this;
  }
  inline MatcherConfig& operator=(MatcherConfig&& from) noexcept {
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
  static const MatcherConfig& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const MatcherConfig* internal_default_instance() {
    return reinterpret_cast<const MatcherConfig*>(
               &_MatcherConfig_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(MatcherConfig& a, MatcherConfig& b) {
    a.Swap(&b);
  }
  inline void Swap(MatcherConfig* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline MatcherConfig* New() const final {
    return CreateMaybeMessage<MatcherConfig>(nullptr);
  }

  MatcherConfig* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<MatcherConfig>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const MatcherConfig& from);
  void MergeFrom(const MatcherConfig& from);
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
  void InternalSwap(MatcherConfig* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "MatcherConfig";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2fperception_2fproto_2ftracker_5fconfig_2eproto);
    return ::descriptor_table_modules_2fperception_2fproto_2ftracker_5fconfig_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kMaxMatchDistanceFieldNumber = 1,
    kBoundMatchDistanceFieldNumber = 2,
  };
  // optional double max_match_distance = 1 [default = 2.5];
  bool has_max_match_distance() const;
  private:
  bool _internal_has_max_match_distance() const;
  public:
  void clear_max_match_distance();
  double max_match_distance() const;
  void set_max_match_distance(double value);
  private:
  double _internal_max_match_distance() const;
  void _internal_set_max_match_distance(double value);
  public:

  // optional double bound_match_distance = 2 [default = 10];
  bool has_bound_match_distance() const;
  private:
  bool _internal_has_bound_match_distance() const;
  public:
  void clear_bound_match_distance();
  double bound_match_distance() const;
  void set_bound_match_distance(double value);
  private:
  double _internal_bound_match_distance() const;
  void _internal_set_bound_match_distance(double value);
  public:

  // @@protoc_insertion_point(class_scope:MatcherConfig)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  double max_match_distance_;
  double bound_match_distance_;
  friend struct ::TableStruct_modules_2fperception_2fproto_2ftracker_5fconfig_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// MatcherConfig

// optional double max_match_distance = 1 [default = 2.5];
inline bool MatcherConfig::_internal_has_max_match_distance() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool MatcherConfig::has_max_match_distance() const {
  return _internal_has_max_match_distance();
}
inline void MatcherConfig::clear_max_match_distance() {
  max_match_distance_ = 2.5;
  _has_bits_[0] &= ~0x00000001u;
}
inline double MatcherConfig::_internal_max_match_distance() const {
  return max_match_distance_;
}
inline double MatcherConfig::max_match_distance() const {
  // @@protoc_insertion_point(field_get:MatcherConfig.max_match_distance)
  return _internal_max_match_distance();
}
inline void MatcherConfig::_internal_set_max_match_distance(double value) {
  _has_bits_[0] |= 0x00000001u;
  max_match_distance_ = value;
}
inline void MatcherConfig::set_max_match_distance(double value) {
  _internal_set_max_match_distance(value);
  // @@protoc_insertion_point(field_set:MatcherConfig.max_match_distance)
}

// optional double bound_match_distance = 2 [default = 10];
inline bool MatcherConfig::_internal_has_bound_match_distance() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool MatcherConfig::has_bound_match_distance() const {
  return _internal_has_bound_match_distance();
}
inline void MatcherConfig::clear_bound_match_distance() {
  bound_match_distance_ = 10;
  _has_bits_[0] &= ~0x00000002u;
}
inline double MatcherConfig::_internal_bound_match_distance() const {
  return bound_match_distance_;
}
inline double MatcherConfig::bound_match_distance() const {
  // @@protoc_insertion_point(field_get:MatcherConfig.bound_match_distance)
  return _internal_bound_match_distance();
}
inline void MatcherConfig::_internal_set_bound_match_distance(double value) {
  _has_bits_[0] |= 0x00000002u;
  bound_match_distance_ = value;
}
inline void MatcherConfig::set_bound_match_distance(double value) {
  _internal_set_bound_match_distance(value);
  // @@protoc_insertion_point(field_set:MatcherConfig.bound_match_distance)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)


// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2fperception_2fproto_2ftracker_5fconfig_2eproto
