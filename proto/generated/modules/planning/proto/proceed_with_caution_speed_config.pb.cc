// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/planning/proto/proceed_with_caution_speed_config.proto

#include "modules/planning/proto/proceed_with_caution_speed_config.pb.h"

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
namespace planning {
class ProceedWithCautionSpeedConfigDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<ProceedWithCautionSpeedConfig> _instance;
} _ProceedWithCautionSpeedConfig_default_instance_;
}  // namespace planning
}  // namespace apollo
static void InitDefaultsscc_info_ProceedWithCautionSpeedConfig_modules_2fplanning_2fproto_2fproceed_5fwith_5fcaution_5fspeed_5fconfig_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::planning::_ProceedWithCautionSpeedConfig_default_instance_;
    new (ptr) ::apollo::planning::ProceedWithCautionSpeedConfig();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::planning::ProceedWithCautionSpeedConfig::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_ProceedWithCautionSpeedConfig_modules_2fplanning_2fproto_2fproceed_5fwith_5fcaution_5fspeed_5fconfig_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_ProceedWithCautionSpeedConfig_modules_2fplanning_2fproto_2fproceed_5fwith_5fcaution_5fspeed_5fconfig_2eproto}, {}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fplanning_2fproto_2fproceed_5fwith_5fcaution_5fspeed_5fconfig_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2fplanning_2fproto_2fproceed_5fwith_5fcaution_5fspeed_5fconfig_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fplanning_2fproto_2fproceed_5fwith_5fcaution_5fspeed_5fconfig_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fplanning_2fproto_2fproceed_5fwith_5fcaution_5fspeed_5fconfig_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::planning::ProceedWithCautionSpeedConfig, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::planning::ProceedWithCautionSpeedConfig, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::apollo::planning::ProceedWithCautionSpeedConfig, max_distance_),
  0,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 6, sizeof(::apollo::planning::ProceedWithCautionSpeedConfig)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::planning::_ProceedWithCautionSpeedConfig_default_instance_),
};

const char descriptor_table_protodef_modules_2fplanning_2fproto_2fproceed_5fwith_5fcaution_5fspeed_5fconfig_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n>modules/planning/proto/proceed_with_ca"
  "ution_speed_config.proto\022\017apollo.plannin"
  "g\"8\n\035ProceedWithCautionSpeedConfig\022\027\n\014ma"
  "x_distance\030\001 \001(\001:\0015"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_modules_2fplanning_2fproto_2fproceed_5fwith_5fcaution_5fspeed_5fconfig_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_modules_2fplanning_2fproto_2fproceed_5fwith_5fcaution_5fspeed_5fconfig_2eproto_sccs[1] = {
  &scc_info_ProceedWithCautionSpeedConfig_modules_2fplanning_2fproto_2fproceed_5fwith_5fcaution_5fspeed_5fconfig_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fplanning_2fproto_2fproceed_5fwith_5fcaution_5fspeed_5fconfig_2eproto_once;
static bool descriptor_table_modules_2fplanning_2fproto_2fproceed_5fwith_5fcaution_5fspeed_5fconfig_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fplanning_2fproto_2fproceed_5fwith_5fcaution_5fspeed_5fconfig_2eproto = {
  &descriptor_table_modules_2fplanning_2fproto_2fproceed_5fwith_5fcaution_5fspeed_5fconfig_2eproto_initialized, descriptor_table_protodef_modules_2fplanning_2fproto_2fproceed_5fwith_5fcaution_5fspeed_5fconfig_2eproto, "modules/planning/proto/proceed_with_caution_speed_config.proto", 139,
  &descriptor_table_modules_2fplanning_2fproto_2fproceed_5fwith_5fcaution_5fspeed_5fconfig_2eproto_once, descriptor_table_modules_2fplanning_2fproto_2fproceed_5fwith_5fcaution_5fspeed_5fconfig_2eproto_sccs, descriptor_table_modules_2fplanning_2fproto_2fproceed_5fwith_5fcaution_5fspeed_5fconfig_2eproto_deps, 1, 0,
  schemas, file_default_instances, TableStruct_modules_2fplanning_2fproto_2fproceed_5fwith_5fcaution_5fspeed_5fconfig_2eproto::offsets,
  file_level_metadata_modules_2fplanning_2fproto_2fproceed_5fwith_5fcaution_5fspeed_5fconfig_2eproto, 1, file_level_enum_descriptors_modules_2fplanning_2fproto_2fproceed_5fwith_5fcaution_5fspeed_5fconfig_2eproto, file_level_service_descriptors_modules_2fplanning_2fproto_2fproceed_5fwith_5fcaution_5fspeed_5fconfig_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_modules_2fplanning_2fproto_2fproceed_5fwith_5fcaution_5fspeed_5fconfig_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_modules_2fplanning_2fproto_2fproceed_5fwith_5fcaution_5fspeed_5fconfig_2eproto), true);
namespace apollo {
namespace planning {

// ===================================================================

void ProceedWithCautionSpeedConfig::InitAsDefaultInstance() {
}
class ProceedWithCautionSpeedConfig::_Internal {
 public:
  using HasBits = decltype(std::declval<ProceedWithCautionSpeedConfig>()._has_bits_);
  static void set_has_max_distance(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
};

ProceedWithCautionSpeedConfig::ProceedWithCautionSpeedConfig()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.planning.ProceedWithCautionSpeedConfig)
}
ProceedWithCautionSpeedConfig::ProceedWithCautionSpeedConfig(const ProceedWithCautionSpeedConfig& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  max_distance_ = from.max_distance_;
  // @@protoc_insertion_point(copy_constructor:apollo.planning.ProceedWithCautionSpeedConfig)
}

void ProceedWithCautionSpeedConfig::SharedCtor() {
  max_distance_ = 5;
}

ProceedWithCautionSpeedConfig::~ProceedWithCautionSpeedConfig() {
  // @@protoc_insertion_point(destructor:apollo.planning.ProceedWithCautionSpeedConfig)
  SharedDtor();
}

void ProceedWithCautionSpeedConfig::SharedDtor() {
}

void ProceedWithCautionSpeedConfig::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ProceedWithCautionSpeedConfig& ProceedWithCautionSpeedConfig::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_ProceedWithCautionSpeedConfig_modules_2fplanning_2fproto_2fproceed_5fwith_5fcaution_5fspeed_5fconfig_2eproto.base);
  return *internal_default_instance();
}


void ProceedWithCautionSpeedConfig::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.planning.ProceedWithCautionSpeedConfig)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  max_distance_ = 5;
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* ProceedWithCautionSpeedConfig::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional double max_distance = 1 [default = 5];
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 9)) {
          _Internal::set_has_max_distance(&has_bits);
          max_distance_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
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

::PROTOBUF_NAMESPACE_ID::uint8* ProceedWithCautionSpeedConfig::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.planning.ProceedWithCautionSpeedConfig)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional double max_distance = 1 [default = 5];
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(1, this->_internal_max_distance(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.planning.ProceedWithCautionSpeedConfig)
  return target;
}

size_t ProceedWithCautionSpeedConfig::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.planning.ProceedWithCautionSpeedConfig)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // optional double max_distance = 1 [default = 5];
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    total_size += 1 + 8;
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void ProceedWithCautionSpeedConfig::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.planning.ProceedWithCautionSpeedConfig)
  GOOGLE_DCHECK_NE(&from, this);
  const ProceedWithCautionSpeedConfig* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<ProceedWithCautionSpeedConfig>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.planning.ProceedWithCautionSpeedConfig)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.planning.ProceedWithCautionSpeedConfig)
    MergeFrom(*source);
  }
}

void ProceedWithCautionSpeedConfig::MergeFrom(const ProceedWithCautionSpeedConfig& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.planning.ProceedWithCautionSpeedConfig)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from._internal_has_max_distance()) {
    _internal_set_max_distance(from._internal_max_distance());
  }
}

void ProceedWithCautionSpeedConfig::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.planning.ProceedWithCautionSpeedConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ProceedWithCautionSpeedConfig::CopyFrom(const ProceedWithCautionSpeedConfig& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.planning.ProceedWithCautionSpeedConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ProceedWithCautionSpeedConfig::IsInitialized() const {
  return true;
}

void ProceedWithCautionSpeedConfig::InternalSwap(ProceedWithCautionSpeedConfig* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(max_distance_, other->max_distance_);
}

::PROTOBUF_NAMESPACE_ID::Metadata ProceedWithCautionSpeedConfig::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace planning
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::planning::ProceedWithCautionSpeedConfig* Arena::CreateMaybeMessage< ::apollo::planning::ProceedWithCautionSpeedConfig >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::planning::ProceedWithCautionSpeedConfig >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>