// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/drivers/tools/image_decompress/proto/config.proto

#include "modules/drivers/tools/image_decompress/proto/config.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
namespace apollo {
namespace image_decompress {
class ConfigDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<Config> _instance;
} _Config_default_instance_;
}  // namespace image_decompress
}  // namespace apollo
static void InitDefaultsscc_info_Config_modules_2fdrivers_2ftools_2fimage_5fdecompress_2fproto_2fconfig_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::image_decompress::_Config_default_instance_;
    new (ptr) ::apollo::image_decompress::Config();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::image_decompress::Config::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_Config_modules_2fdrivers_2ftools_2fimage_5fdecompress_2fproto_2fconfig_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_Config_modules_2fdrivers_2ftools_2fimage_5fdecompress_2fproto_2fconfig_2eproto}, {}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fdrivers_2ftools_2fimage_5fdecompress_2fproto_2fconfig_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2fdrivers_2ftools_2fimage_5fdecompress_2fproto_2fconfig_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fdrivers_2ftools_2fimage_5fdecompress_2fproto_2fconfig_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fdrivers_2ftools_2fimage_5fdecompress_2fproto_2fconfig_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::image_decompress::Config, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::image_decompress::Config, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::apollo::image_decompress::Config, channel_name_),
  0,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 6, sizeof(::apollo::image_decompress::Config)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::image_decompress::_Config_default_instance_),
};

const char descriptor_table_protodef_modules_2fdrivers_2ftools_2fimage_5fdecompress_2fproto_2fconfig_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n9modules/drivers/tools/image_decompress"
  "/proto/config.proto\022\027apollo.image_decomp"
  "ress\"\036\n\006Config\022\024\n\014channel_name\030\001 \001(\t"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_modules_2fdrivers_2ftools_2fimage_5fdecompress_2fproto_2fconfig_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_modules_2fdrivers_2ftools_2fimage_5fdecompress_2fproto_2fconfig_2eproto_sccs[1] = {
  &scc_info_Config_modules_2fdrivers_2ftools_2fimage_5fdecompress_2fproto_2fconfig_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fdrivers_2ftools_2fimage_5fdecompress_2fproto_2fconfig_2eproto_once;
static bool descriptor_table_modules_2fdrivers_2ftools_2fimage_5fdecompress_2fproto_2fconfig_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fdrivers_2ftools_2fimage_5fdecompress_2fproto_2fconfig_2eproto = {
  &descriptor_table_modules_2fdrivers_2ftools_2fimage_5fdecompress_2fproto_2fconfig_2eproto_initialized, descriptor_table_protodef_modules_2fdrivers_2ftools_2fimage_5fdecompress_2fproto_2fconfig_2eproto, "modules/drivers/tools/image_decompress/proto/config.proto", 116,
  &descriptor_table_modules_2fdrivers_2ftools_2fimage_5fdecompress_2fproto_2fconfig_2eproto_once, descriptor_table_modules_2fdrivers_2ftools_2fimage_5fdecompress_2fproto_2fconfig_2eproto_sccs, descriptor_table_modules_2fdrivers_2ftools_2fimage_5fdecompress_2fproto_2fconfig_2eproto_deps, 1, 0,
  schemas, file_default_instances, TableStruct_modules_2fdrivers_2ftools_2fimage_5fdecompress_2fproto_2fconfig_2eproto::offsets,
  file_level_metadata_modules_2fdrivers_2ftools_2fimage_5fdecompress_2fproto_2fconfig_2eproto, 1, file_level_enum_descriptors_modules_2fdrivers_2ftools_2fimage_5fdecompress_2fproto_2fconfig_2eproto, file_level_service_descriptors_modules_2fdrivers_2ftools_2fimage_5fdecompress_2fproto_2fconfig_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_modules_2fdrivers_2ftools_2fimage_5fdecompress_2fproto_2fconfig_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_modules_2fdrivers_2ftools_2fimage_5fdecompress_2fproto_2fconfig_2eproto), true);
namespace apollo {
namespace image_decompress {

// ===================================================================

void Config::InitAsDefaultInstance() {
}
class Config::_Internal {
 public:
  using HasBits = decltype(std::declval<Config>()._has_bits_);
  static void set_has_channel_name(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
};

Config::Config()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.image_decompress.Config)
}
Config::Config(const Config& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  channel_name_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (from._internal_has_channel_name()) {
    channel_name_.AssignWithDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), from.channel_name_);
  }
  // @@protoc_insertion_point(copy_constructor:apollo.image_decompress.Config)
}

void Config::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_Config_modules_2fdrivers_2ftools_2fimage_5fdecompress_2fproto_2fconfig_2eproto.base);
  channel_name_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}

Config::~Config() {
  // @@protoc_insertion_point(destructor:apollo.image_decompress.Config)
  SharedDtor();
}

void Config::SharedDtor() {
  channel_name_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}

void Config::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const Config& Config::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_Config_modules_2fdrivers_2ftools_2fimage_5fdecompress_2fproto_2fconfig_2eproto.base);
  return *internal_default_instance();
}


void Config::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.image_decompress.Config)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    channel_name_.ClearNonDefaultToEmptyNoArena();
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* Config::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional string channel_name = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          auto str = _internal_mutable_channel_name();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          #ifndef NDEBUG
          ::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "apollo.image_decompress.Config.channel_name");
          #endif  // !NDEBUG
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag, &_internal_metadata_, ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  _has_bits_.Or(has_bits);
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* Config::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.image_decompress.Config)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional string channel_name = 1;
  if (cached_has_bits & 0x00000001u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_channel_name().data(), static_cast<int>(this->_internal_channel_name().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.image_decompress.Config.channel_name");
    target = stream->WriteStringMaybeAliased(
        1, this->_internal_channel_name(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.image_decompress.Config)
  return target;
}

size_t Config::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.image_decompress.Config)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // optional string channel_name = 1;
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
        this->_internal_channel_name());
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void Config::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.image_decompress.Config)
  GOOGLE_DCHECK_NE(&from, this);
  const Config* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<Config>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.image_decompress.Config)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.image_decompress.Config)
    MergeFrom(*source);
  }
}

void Config::MergeFrom(const Config& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.image_decompress.Config)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from._internal_has_channel_name()) {
    _has_bits_[0] |= 0x00000001u;
    channel_name_.AssignWithDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), from.channel_name_);
  }
}

void Config::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.image_decompress.Config)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Config::CopyFrom(const Config& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.image_decompress.Config)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Config::IsInitialized() const {
  return true;
}

void Config::InternalSwap(Config* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  channel_name_.Swap(&other->channel_name_, &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
    GetArenaNoVirtual());
}

::PROTOBUF_NAMESPACE_ID::Metadata Config::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace image_decompress
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::image_decompress::Config* Arena::CreateMaybeMessage< ::apollo::image_decompress::Config >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::image_decompress::Config >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>