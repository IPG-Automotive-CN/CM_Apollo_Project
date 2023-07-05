// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/planning/proto/decider_config.proto

#include "modules/planning/proto/decider_config.pb.h"

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
class CreepDeciderConfigDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<CreepDeciderConfig> _instance;
} _CreepDeciderConfig_default_instance_;
class SidePassSafetyConfigDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<SidePassSafetyConfig> _instance;
} _SidePassSafetyConfig_default_instance_;
}  // namespace planning
}  // namespace apollo
static void InitDefaultsscc_info_CreepDeciderConfig_modules_2fplanning_2fproto_2fdecider_5fconfig_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::planning::_CreepDeciderConfig_default_instance_;
    new (ptr) ::apollo::planning::CreepDeciderConfig();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::planning::CreepDeciderConfig::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_CreepDeciderConfig_modules_2fplanning_2fproto_2fdecider_5fconfig_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_CreepDeciderConfig_modules_2fplanning_2fproto_2fdecider_5fconfig_2eproto}, {}};

static void InitDefaultsscc_info_SidePassSafetyConfig_modules_2fplanning_2fproto_2fdecider_5fconfig_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::planning::_SidePassSafetyConfig_default_instance_;
    new (ptr) ::apollo::planning::SidePassSafetyConfig();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::planning::SidePassSafetyConfig::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_SidePassSafetyConfig_modules_2fplanning_2fproto_2fdecider_5fconfig_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_SidePassSafetyConfig_modules_2fplanning_2fproto_2fdecider_5fconfig_2eproto}, {}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fplanning_2fproto_2fdecider_5fconfig_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2fplanning_2fproto_2fdecider_5fconfig_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fplanning_2fproto_2fdecider_5fconfig_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fplanning_2fproto_2fdecider_5fconfig_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::planning::CreepDeciderConfig, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::planning::CreepDeciderConfig, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::apollo::planning::CreepDeciderConfig, stop_distance_),
  PROTOBUF_FIELD_OFFSET(::apollo::planning::CreepDeciderConfig, speed_limit_),
  PROTOBUF_FIELD_OFFSET(::apollo::planning::CreepDeciderConfig, max_valid_stop_distance_),
  PROTOBUF_FIELD_OFFSET(::apollo::planning::CreepDeciderConfig, min_boundary_t_),
  PROTOBUF_FIELD_OFFSET(::apollo::planning::CreepDeciderConfig, ignore_max_st_min_t_),
  PROTOBUF_FIELD_OFFSET(::apollo::planning::CreepDeciderConfig, ignore_min_st_min_s_),
  0,
  1,
  2,
  3,
  4,
  5,
  PROTOBUF_FIELD_OFFSET(::apollo::planning::SidePassSafetyConfig, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::planning::SidePassSafetyConfig, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::apollo::planning::SidePassSafetyConfig, min_obstacle_lateral_distance_),
  PROTOBUF_FIELD_OFFSET(::apollo::planning::SidePassSafetyConfig, max_overlap_s_range_),
  PROTOBUF_FIELD_OFFSET(::apollo::planning::SidePassSafetyConfig, safe_duration_reach_ref_line_),
  0,
  1,
  2,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 11, sizeof(::apollo::planning::CreepDeciderConfig)},
  { 17, 25, sizeof(::apollo::planning::SidePassSafetyConfig)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::planning::_CreepDeciderConfig_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::planning::_SidePassSafetyConfig_default_instance_),
};

const char descriptor_table_protodef_modules_2fplanning_2fproto_2fdecider_5fconfig_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n+modules/planning/proto/decider_config."
  "proto\022\017apollo.planning\"\314\001\n\022CreepDeciderC"
  "onfig\022\032\n\rstop_distance\030\001 \001(\001:\0030.5\022\026\n\013spe"
  "ed_limit\030\002 \001(\001:\0011\022$\n\027max_valid_stop_dist"
  "ance\030\003 \001(\001:\0030.3\022\031\n\016min_boundary_t\030\004 \001(\001:"
  "\0016\022 \n\023ignore_max_st_min_t\030\005 \001(\001:\0030.1\022\037\n\023"
  "ignore_min_st_min_s\030\006 \001(\001:\00215\"\211\001\n\024SidePa"
  "ssSafetyConfig\022(\n\035min_obstacle_lateral_d"
  "istance\030\001 \001(\001:\0011\022\036\n\023max_overlap_s_range\030"
  "\002 \001(\001:\0015\022\'\n\034safe_duration_reach_ref_line"
  "\030\003 \001(\001:\0015"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_modules_2fplanning_2fproto_2fdecider_5fconfig_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_modules_2fplanning_2fproto_2fdecider_5fconfig_2eproto_sccs[2] = {
  &scc_info_CreepDeciderConfig_modules_2fplanning_2fproto_2fdecider_5fconfig_2eproto.base,
  &scc_info_SidePassSafetyConfig_modules_2fplanning_2fproto_2fdecider_5fconfig_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fplanning_2fproto_2fdecider_5fconfig_2eproto_once;
static bool descriptor_table_modules_2fplanning_2fproto_2fdecider_5fconfig_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fplanning_2fproto_2fdecider_5fconfig_2eproto = {
  &descriptor_table_modules_2fplanning_2fproto_2fdecider_5fconfig_2eproto_initialized, descriptor_table_protodef_modules_2fplanning_2fproto_2fdecider_5fconfig_2eproto, "modules/planning/proto/decider_config.proto", 409,
  &descriptor_table_modules_2fplanning_2fproto_2fdecider_5fconfig_2eproto_once, descriptor_table_modules_2fplanning_2fproto_2fdecider_5fconfig_2eproto_sccs, descriptor_table_modules_2fplanning_2fproto_2fdecider_5fconfig_2eproto_deps, 2, 0,
  schemas, file_default_instances, TableStruct_modules_2fplanning_2fproto_2fdecider_5fconfig_2eproto::offsets,
  file_level_metadata_modules_2fplanning_2fproto_2fdecider_5fconfig_2eproto, 2, file_level_enum_descriptors_modules_2fplanning_2fproto_2fdecider_5fconfig_2eproto, file_level_service_descriptors_modules_2fplanning_2fproto_2fdecider_5fconfig_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_modules_2fplanning_2fproto_2fdecider_5fconfig_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_modules_2fplanning_2fproto_2fdecider_5fconfig_2eproto), true);
namespace apollo {
namespace planning {

// ===================================================================

void CreepDeciderConfig::InitAsDefaultInstance() {
}
class CreepDeciderConfig::_Internal {
 public:
  using HasBits = decltype(std::declval<CreepDeciderConfig>()._has_bits_);
  static void set_has_stop_distance(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_speed_limit(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_max_valid_stop_distance(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_min_boundary_t(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
  static void set_has_ignore_max_st_min_t(HasBits* has_bits) {
    (*has_bits)[0] |= 16u;
  }
  static void set_has_ignore_min_st_min_s(HasBits* has_bits) {
    (*has_bits)[0] |= 32u;
  }
};

CreepDeciderConfig::CreepDeciderConfig()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.planning.CreepDeciderConfig)
}
CreepDeciderConfig::CreepDeciderConfig(const CreepDeciderConfig& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&stop_distance_, &from.stop_distance_,
    static_cast<size_t>(reinterpret_cast<char*>(&ignore_min_st_min_s_) -
    reinterpret_cast<char*>(&stop_distance_)) + sizeof(ignore_min_st_min_s_));
  // @@protoc_insertion_point(copy_constructor:apollo.planning.CreepDeciderConfig)
}

void CreepDeciderConfig::SharedCtor() {
  stop_distance_ = 0.5;
  speed_limit_ = 1;
  max_valid_stop_distance_ = 0.3;
  min_boundary_t_ = 6;
  ignore_max_st_min_t_ = 0.1;
  ignore_min_st_min_s_ = 15;
}

CreepDeciderConfig::~CreepDeciderConfig() {
  // @@protoc_insertion_point(destructor:apollo.planning.CreepDeciderConfig)
  SharedDtor();
}

void CreepDeciderConfig::SharedDtor() {
}

void CreepDeciderConfig::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const CreepDeciderConfig& CreepDeciderConfig::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_CreepDeciderConfig_modules_2fplanning_2fproto_2fdecider_5fconfig_2eproto.base);
  return *internal_default_instance();
}


void CreepDeciderConfig::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.planning.CreepDeciderConfig)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000003fu) {
    stop_distance_ = 0.5;
    speed_limit_ = 1;
    max_valid_stop_distance_ = 0.3;
    min_boundary_t_ = 6;
    ignore_max_st_min_t_ = 0.1;
    ignore_min_st_min_s_ = 15;
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* CreepDeciderConfig::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional double stop_distance = 1 [default = 0.5];
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 9)) {
          _Internal::set_has_stop_distance(&has_bits);
          stop_distance_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else goto handle_unusual;
        continue;
      // optional double speed_limit = 2 [default = 1];
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 17)) {
          _Internal::set_has_speed_limit(&has_bits);
          speed_limit_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else goto handle_unusual;
        continue;
      // optional double max_valid_stop_distance = 3 [default = 0.3];
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 25)) {
          _Internal::set_has_max_valid_stop_distance(&has_bits);
          max_valid_stop_distance_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else goto handle_unusual;
        continue;
      // optional double min_boundary_t = 4 [default = 6];
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 33)) {
          _Internal::set_has_min_boundary_t(&has_bits);
          min_boundary_t_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else goto handle_unusual;
        continue;
      // optional double ignore_max_st_min_t = 5 [default = 0.1];
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 41)) {
          _Internal::set_has_ignore_max_st_min_t(&has_bits);
          ignore_max_st_min_t_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else goto handle_unusual;
        continue;
      // optional double ignore_min_st_min_s = 6 [default = 15];
      case 6:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 49)) {
          _Internal::set_has_ignore_min_st_min_s(&has_bits);
          ignore_min_st_min_s_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* CreepDeciderConfig::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.planning.CreepDeciderConfig)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional double stop_distance = 1 [default = 0.5];
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(1, this->_internal_stop_distance(), target);
  }

  // optional double speed_limit = 2 [default = 1];
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(2, this->_internal_speed_limit(), target);
  }

  // optional double max_valid_stop_distance = 3 [default = 0.3];
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(3, this->_internal_max_valid_stop_distance(), target);
  }

  // optional double min_boundary_t = 4 [default = 6];
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(4, this->_internal_min_boundary_t(), target);
  }

  // optional double ignore_max_st_min_t = 5 [default = 0.1];
  if (cached_has_bits & 0x00000010u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(5, this->_internal_ignore_max_st_min_t(), target);
  }

  // optional double ignore_min_st_min_s = 6 [default = 15];
  if (cached_has_bits & 0x00000020u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(6, this->_internal_ignore_min_st_min_s(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.planning.CreepDeciderConfig)
  return target;
}

size_t CreepDeciderConfig::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.planning.CreepDeciderConfig)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000003fu) {
    // optional double stop_distance = 1 [default = 0.5];
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 + 8;
    }

    // optional double speed_limit = 2 [default = 1];
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 + 8;
    }

    // optional double max_valid_stop_distance = 3 [default = 0.3];
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 + 8;
    }

    // optional double min_boundary_t = 4 [default = 6];
    if (cached_has_bits & 0x00000008u) {
      total_size += 1 + 8;
    }

    // optional double ignore_max_st_min_t = 5 [default = 0.1];
    if (cached_has_bits & 0x00000010u) {
      total_size += 1 + 8;
    }

    // optional double ignore_min_st_min_s = 6 [default = 15];
    if (cached_has_bits & 0x00000020u) {
      total_size += 1 + 8;
    }

  }
  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void CreepDeciderConfig::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.planning.CreepDeciderConfig)
  GOOGLE_DCHECK_NE(&from, this);
  const CreepDeciderConfig* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<CreepDeciderConfig>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.planning.CreepDeciderConfig)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.planning.CreepDeciderConfig)
    MergeFrom(*source);
  }
}

void CreepDeciderConfig::MergeFrom(const CreepDeciderConfig& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.planning.CreepDeciderConfig)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x0000003fu) {
    if (cached_has_bits & 0x00000001u) {
      stop_distance_ = from.stop_distance_;
    }
    if (cached_has_bits & 0x00000002u) {
      speed_limit_ = from.speed_limit_;
    }
    if (cached_has_bits & 0x00000004u) {
      max_valid_stop_distance_ = from.max_valid_stop_distance_;
    }
    if (cached_has_bits & 0x00000008u) {
      min_boundary_t_ = from.min_boundary_t_;
    }
    if (cached_has_bits & 0x00000010u) {
      ignore_max_st_min_t_ = from.ignore_max_st_min_t_;
    }
    if (cached_has_bits & 0x00000020u) {
      ignore_min_st_min_s_ = from.ignore_min_st_min_s_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void CreepDeciderConfig::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.planning.CreepDeciderConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void CreepDeciderConfig::CopyFrom(const CreepDeciderConfig& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.planning.CreepDeciderConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool CreepDeciderConfig::IsInitialized() const {
  return true;
}

void CreepDeciderConfig::InternalSwap(CreepDeciderConfig* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(stop_distance_, other->stop_distance_);
  swap(speed_limit_, other->speed_limit_);
  swap(max_valid_stop_distance_, other->max_valid_stop_distance_);
  swap(min_boundary_t_, other->min_boundary_t_);
  swap(ignore_max_st_min_t_, other->ignore_max_st_min_t_);
  swap(ignore_min_st_min_s_, other->ignore_min_st_min_s_);
}

::PROTOBUF_NAMESPACE_ID::Metadata CreepDeciderConfig::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void SidePassSafetyConfig::InitAsDefaultInstance() {
}
class SidePassSafetyConfig::_Internal {
 public:
  using HasBits = decltype(std::declval<SidePassSafetyConfig>()._has_bits_);
  static void set_has_min_obstacle_lateral_distance(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_max_overlap_s_range(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_safe_duration_reach_ref_line(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
};

SidePassSafetyConfig::SidePassSafetyConfig()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.planning.SidePassSafetyConfig)
}
SidePassSafetyConfig::SidePassSafetyConfig(const SidePassSafetyConfig& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&min_obstacle_lateral_distance_, &from.min_obstacle_lateral_distance_,
    static_cast<size_t>(reinterpret_cast<char*>(&safe_duration_reach_ref_line_) -
    reinterpret_cast<char*>(&min_obstacle_lateral_distance_)) + sizeof(safe_duration_reach_ref_line_));
  // @@protoc_insertion_point(copy_constructor:apollo.planning.SidePassSafetyConfig)
}

void SidePassSafetyConfig::SharedCtor() {
  min_obstacle_lateral_distance_ = 1;
  max_overlap_s_range_ = 5;
  safe_duration_reach_ref_line_ = 5;
}

SidePassSafetyConfig::~SidePassSafetyConfig() {
  // @@protoc_insertion_point(destructor:apollo.planning.SidePassSafetyConfig)
  SharedDtor();
}

void SidePassSafetyConfig::SharedDtor() {
}

void SidePassSafetyConfig::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const SidePassSafetyConfig& SidePassSafetyConfig::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_SidePassSafetyConfig_modules_2fplanning_2fproto_2fdecider_5fconfig_2eproto.base);
  return *internal_default_instance();
}


void SidePassSafetyConfig::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.planning.SidePassSafetyConfig)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    min_obstacle_lateral_distance_ = 1;
    max_overlap_s_range_ = 5;
    safe_duration_reach_ref_line_ = 5;
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* SidePassSafetyConfig::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional double min_obstacle_lateral_distance = 1 [default = 1];
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 9)) {
          _Internal::set_has_min_obstacle_lateral_distance(&has_bits);
          min_obstacle_lateral_distance_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else goto handle_unusual;
        continue;
      // optional double max_overlap_s_range = 2 [default = 5];
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 17)) {
          _Internal::set_has_max_overlap_s_range(&has_bits);
          max_overlap_s_range_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else goto handle_unusual;
        continue;
      // optional double safe_duration_reach_ref_line = 3 [default = 5];
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 25)) {
          _Internal::set_has_safe_duration_reach_ref_line(&has_bits);
          safe_duration_reach_ref_line_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* SidePassSafetyConfig::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.planning.SidePassSafetyConfig)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional double min_obstacle_lateral_distance = 1 [default = 1];
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(1, this->_internal_min_obstacle_lateral_distance(), target);
  }

  // optional double max_overlap_s_range = 2 [default = 5];
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(2, this->_internal_max_overlap_s_range(), target);
  }

  // optional double safe_duration_reach_ref_line = 3 [default = 5];
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(3, this->_internal_safe_duration_reach_ref_line(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.planning.SidePassSafetyConfig)
  return target;
}

size_t SidePassSafetyConfig::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.planning.SidePassSafetyConfig)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    // optional double min_obstacle_lateral_distance = 1 [default = 1];
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 + 8;
    }

    // optional double max_overlap_s_range = 2 [default = 5];
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 + 8;
    }

    // optional double safe_duration_reach_ref_line = 3 [default = 5];
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 + 8;
    }

  }
  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void SidePassSafetyConfig::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.planning.SidePassSafetyConfig)
  GOOGLE_DCHECK_NE(&from, this);
  const SidePassSafetyConfig* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<SidePassSafetyConfig>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.planning.SidePassSafetyConfig)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.planning.SidePassSafetyConfig)
    MergeFrom(*source);
  }
}

void SidePassSafetyConfig::MergeFrom(const SidePassSafetyConfig& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.planning.SidePassSafetyConfig)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      min_obstacle_lateral_distance_ = from.min_obstacle_lateral_distance_;
    }
    if (cached_has_bits & 0x00000002u) {
      max_overlap_s_range_ = from.max_overlap_s_range_;
    }
    if (cached_has_bits & 0x00000004u) {
      safe_duration_reach_ref_line_ = from.safe_duration_reach_ref_line_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void SidePassSafetyConfig::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.planning.SidePassSafetyConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void SidePassSafetyConfig::CopyFrom(const SidePassSafetyConfig& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.planning.SidePassSafetyConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool SidePassSafetyConfig::IsInitialized() const {
  return true;
}

void SidePassSafetyConfig::InternalSwap(SidePassSafetyConfig* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(min_obstacle_lateral_distance_, other->min_obstacle_lateral_distance_);
  swap(max_overlap_s_range_, other->max_overlap_s_range_);
  swap(safe_duration_reach_ref_line_, other->safe_duration_reach_ref_line_);
}

::PROTOBUF_NAMESPACE_ID::Metadata SidePassSafetyConfig::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace planning
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::planning::CreepDeciderConfig* Arena::CreateMaybeMessage< ::apollo::planning::CreepDeciderConfig >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::planning::CreepDeciderConfig >(arena);
}
template<> PROTOBUF_NOINLINE ::apollo::planning::SidePassSafetyConfig* Arena::CreateMaybeMessage< ::apollo::planning::SidePassSafetyConfig >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::planning::SidePassSafetyConfig >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>