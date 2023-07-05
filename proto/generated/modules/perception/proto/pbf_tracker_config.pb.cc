// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/perception/proto/pbf_tracker_config.proto

#include "modules/perception/proto/pbf_tracker_config.pb.h"

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
namespace perception {
namespace fusion {
class PbfTrackerConfigDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<PbfTrackerConfig> _instance;
} _PbfTrackerConfig_default_instance_;
}  // namespace fusion
}  // namespace perception
}  // namespace apollo
static void InitDefaultsscc_info_PbfTrackerConfig_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_type_fusion_method_.DefaultConstruct();
  *::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_type_fusion_method_.get_mutable() = std::string("DstTypeFusion", 13);
  ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyString(
      ::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_type_fusion_method_.get_mutable());
  ::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_motion_fusion_method_.DefaultConstruct();
  *::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_motion_fusion_method_.get_mutable() = std::string("KalmanMotionFusion", 18);
  ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyString(
      ::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_motion_fusion_method_.get_mutable());
  ::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_shape_fusion_method_.DefaultConstruct();
  *::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_shape_fusion_method_.get_mutable() = std::string("PbfShapeFusion", 14);
  ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyString(
      ::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_shape_fusion_method_.get_mutable());
  ::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_existance_fusion_method_.DefaultConstruct();
  *::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_existance_fusion_method_.get_mutable() = std::string("DstExistanceFusion", 18);
  ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyString(
      ::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_existance_fusion_method_.get_mutable());
  {
    void* ptr = &::apollo::perception::fusion::_PbfTrackerConfig_default_instance_;
    new (ptr) ::apollo::perception::fusion::PbfTrackerConfig();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::perception::fusion::PbfTrackerConfig::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_PbfTrackerConfig_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_PbfTrackerConfig_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto}, {}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::perception::fusion::PbfTrackerConfig, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::fusion::PbfTrackerConfig, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::apollo::perception::fusion::PbfTrackerConfig, type_fusion_method_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::fusion::PbfTrackerConfig, motion_fusion_method_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::fusion::PbfTrackerConfig, shape_fusion_method_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::fusion::PbfTrackerConfig, existance_fusion_method_),
  0,
  1,
  2,
  3,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 9, sizeof(::apollo::perception::fusion::PbfTrackerConfig)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::perception::fusion::_PbfTrackerConfig_default_instance_),
};

const char descriptor_table_protodef_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n1modules/perception/proto/pbf_tracker_c"
  "onfig.proto\022\030apollo.perception.fusion\"\321\001"
  "\n\020PbfTrackerConfig\022)\n\022type_fusion_method"
  "\030\001 \001(\t:\rDstTypeFusion\0220\n\024motion_fusion_m"
  "ethod\030\002 \001(\t:\022KalmanMotionFusion\022+\n\023shape"
  "_fusion_method\030\003 \001(\t:\016PbfShapeFusion\0223\n\027"
  "existance_fusion_method\030\004 \001(\t:\022DstExista"
  "nceFusion"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto_sccs[1] = {
  &scc_info_PbfTrackerConfig_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto_once;
static bool descriptor_table_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto = {
  &descriptor_table_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto_initialized, descriptor_table_protodef_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto, "modules/perception/proto/pbf_tracker_config.proto", 289,
  &descriptor_table_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto_once, descriptor_table_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto_sccs, descriptor_table_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto_deps, 1, 0,
  schemas, file_default_instances, TableStruct_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto::offsets,
  file_level_metadata_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto, 1, file_level_enum_descriptors_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto, file_level_service_descriptors_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto), true);
namespace apollo {
namespace perception {
namespace fusion {

// ===================================================================

void PbfTrackerConfig::InitAsDefaultInstance() {
}
class PbfTrackerConfig::_Internal {
 public:
  using HasBits = decltype(std::declval<PbfTrackerConfig>()._has_bits_);
  static void set_has_type_fusion_method(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_motion_fusion_method(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_shape_fusion_method(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_existance_fusion_method(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
};

::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<std::string> PbfTrackerConfig::_i_give_permission_to_break_this_code_default_type_fusion_method_;
::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<std::string> PbfTrackerConfig::_i_give_permission_to_break_this_code_default_motion_fusion_method_;
::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<std::string> PbfTrackerConfig::_i_give_permission_to_break_this_code_default_shape_fusion_method_;
::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<std::string> PbfTrackerConfig::_i_give_permission_to_break_this_code_default_existance_fusion_method_;
PbfTrackerConfig::PbfTrackerConfig()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.perception.fusion.PbfTrackerConfig)
}
PbfTrackerConfig::PbfTrackerConfig(const PbfTrackerConfig& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  type_fusion_method_.UnsafeSetDefault(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_type_fusion_method_.get());
  if (from._internal_has_type_fusion_method()) {
    type_fusion_method_.AssignWithDefault(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_type_fusion_method_.get(), from.type_fusion_method_);
  }
  motion_fusion_method_.UnsafeSetDefault(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_motion_fusion_method_.get());
  if (from._internal_has_motion_fusion_method()) {
    motion_fusion_method_.AssignWithDefault(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_motion_fusion_method_.get(), from.motion_fusion_method_);
  }
  shape_fusion_method_.UnsafeSetDefault(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_shape_fusion_method_.get());
  if (from._internal_has_shape_fusion_method()) {
    shape_fusion_method_.AssignWithDefault(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_shape_fusion_method_.get(), from.shape_fusion_method_);
  }
  existance_fusion_method_.UnsafeSetDefault(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_existance_fusion_method_.get());
  if (from._internal_has_existance_fusion_method()) {
    existance_fusion_method_.AssignWithDefault(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_existance_fusion_method_.get(), from.existance_fusion_method_);
  }
  // @@protoc_insertion_point(copy_constructor:apollo.perception.fusion.PbfTrackerConfig)
}

void PbfTrackerConfig::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_PbfTrackerConfig_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto.base);
  type_fusion_method_.UnsafeSetDefault(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_type_fusion_method_.get());
  motion_fusion_method_.UnsafeSetDefault(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_motion_fusion_method_.get());
  shape_fusion_method_.UnsafeSetDefault(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_shape_fusion_method_.get());
  existance_fusion_method_.UnsafeSetDefault(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_existance_fusion_method_.get());
}

PbfTrackerConfig::~PbfTrackerConfig() {
  // @@protoc_insertion_point(destructor:apollo.perception.fusion.PbfTrackerConfig)
  SharedDtor();
}

void PbfTrackerConfig::SharedDtor() {
  type_fusion_method_.DestroyNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_type_fusion_method_.get());
  motion_fusion_method_.DestroyNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_motion_fusion_method_.get());
  shape_fusion_method_.DestroyNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_shape_fusion_method_.get());
  existance_fusion_method_.DestroyNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_existance_fusion_method_.get());
}

void PbfTrackerConfig::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const PbfTrackerConfig& PbfTrackerConfig::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_PbfTrackerConfig_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto.base);
  return *internal_default_instance();
}


void PbfTrackerConfig::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.perception.fusion.PbfTrackerConfig)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    if (cached_has_bits & 0x00000001u) {
      type_fusion_method_.UnsafeMutablePointer()->assign(*&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_type_fusion_method_.get());
    }
    if (cached_has_bits & 0x00000002u) {
      motion_fusion_method_.UnsafeMutablePointer()->assign(*&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_motion_fusion_method_.get());
    }
    if (cached_has_bits & 0x00000004u) {
      shape_fusion_method_.UnsafeMutablePointer()->assign(*&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_shape_fusion_method_.get());
    }
    if (cached_has_bits & 0x00000008u) {
      existance_fusion_method_.UnsafeMutablePointer()->assign(*&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_existance_fusion_method_.get());
    }
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* PbfTrackerConfig::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional string type_fusion_method = 1 [default = "DstTypeFusion"];
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          auto str = _internal_mutable_type_fusion_method();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          #ifndef NDEBUG
          ::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "apollo.perception.fusion.PbfTrackerConfig.type_fusion_method");
          #endif  // !NDEBUG
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional string motion_fusion_method = 2 [default = "KalmanMotionFusion"];
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          auto str = _internal_mutable_motion_fusion_method();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          #ifndef NDEBUG
          ::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "apollo.perception.fusion.PbfTrackerConfig.motion_fusion_method");
          #endif  // !NDEBUG
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional string shape_fusion_method = 3 [default = "PbfShapeFusion"];
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 26)) {
          auto str = _internal_mutable_shape_fusion_method();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          #ifndef NDEBUG
          ::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "apollo.perception.fusion.PbfTrackerConfig.shape_fusion_method");
          #endif  // !NDEBUG
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional string existance_fusion_method = 4 [default = "DstExistanceFusion"];
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 34)) {
          auto str = _internal_mutable_existance_fusion_method();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          #ifndef NDEBUG
          ::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "apollo.perception.fusion.PbfTrackerConfig.existance_fusion_method");
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

::PROTOBUF_NAMESPACE_ID::uint8* PbfTrackerConfig::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.perception.fusion.PbfTrackerConfig)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional string type_fusion_method = 1 [default = "DstTypeFusion"];
  if (cached_has_bits & 0x00000001u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_type_fusion_method().data(), static_cast<int>(this->_internal_type_fusion_method().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.perception.fusion.PbfTrackerConfig.type_fusion_method");
    target = stream->WriteStringMaybeAliased(
        1, this->_internal_type_fusion_method(), target);
  }

  // optional string motion_fusion_method = 2 [default = "KalmanMotionFusion"];
  if (cached_has_bits & 0x00000002u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_motion_fusion_method().data(), static_cast<int>(this->_internal_motion_fusion_method().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.perception.fusion.PbfTrackerConfig.motion_fusion_method");
    target = stream->WriteStringMaybeAliased(
        2, this->_internal_motion_fusion_method(), target);
  }

  // optional string shape_fusion_method = 3 [default = "PbfShapeFusion"];
  if (cached_has_bits & 0x00000004u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_shape_fusion_method().data(), static_cast<int>(this->_internal_shape_fusion_method().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.perception.fusion.PbfTrackerConfig.shape_fusion_method");
    target = stream->WriteStringMaybeAliased(
        3, this->_internal_shape_fusion_method(), target);
  }

  // optional string existance_fusion_method = 4 [default = "DstExistanceFusion"];
  if (cached_has_bits & 0x00000008u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_existance_fusion_method().data(), static_cast<int>(this->_internal_existance_fusion_method().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.perception.fusion.PbfTrackerConfig.existance_fusion_method");
    target = stream->WriteStringMaybeAliased(
        4, this->_internal_existance_fusion_method(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.perception.fusion.PbfTrackerConfig)
  return target;
}

size_t PbfTrackerConfig::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.perception.fusion.PbfTrackerConfig)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    // optional string type_fusion_method = 1 [default = "DstTypeFusion"];
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_type_fusion_method());
    }

    // optional string motion_fusion_method = 2 [default = "KalmanMotionFusion"];
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_motion_fusion_method());
    }

    // optional string shape_fusion_method = 3 [default = "PbfShapeFusion"];
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_shape_fusion_method());
    }

    // optional string existance_fusion_method = 4 [default = "DstExistanceFusion"];
    if (cached_has_bits & 0x00000008u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_existance_fusion_method());
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

void PbfTrackerConfig::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.perception.fusion.PbfTrackerConfig)
  GOOGLE_DCHECK_NE(&from, this);
  const PbfTrackerConfig* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<PbfTrackerConfig>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.perception.fusion.PbfTrackerConfig)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.perception.fusion.PbfTrackerConfig)
    MergeFrom(*source);
  }
}

void PbfTrackerConfig::MergeFrom(const PbfTrackerConfig& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.perception.fusion.PbfTrackerConfig)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    if (cached_has_bits & 0x00000001u) {
      _has_bits_[0] |= 0x00000001u;
      type_fusion_method_.AssignWithDefault(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_type_fusion_method_.get(), from.type_fusion_method_);
    }
    if (cached_has_bits & 0x00000002u) {
      _has_bits_[0] |= 0x00000002u;
      motion_fusion_method_.AssignWithDefault(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_motion_fusion_method_.get(), from.motion_fusion_method_);
    }
    if (cached_has_bits & 0x00000004u) {
      _has_bits_[0] |= 0x00000004u;
      shape_fusion_method_.AssignWithDefault(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_shape_fusion_method_.get(), from.shape_fusion_method_);
    }
    if (cached_has_bits & 0x00000008u) {
      _has_bits_[0] |= 0x00000008u;
      existance_fusion_method_.AssignWithDefault(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_existance_fusion_method_.get(), from.existance_fusion_method_);
    }
  }
}

void PbfTrackerConfig::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.perception.fusion.PbfTrackerConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void PbfTrackerConfig::CopyFrom(const PbfTrackerConfig& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.perception.fusion.PbfTrackerConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool PbfTrackerConfig::IsInitialized() const {
  return true;
}

void PbfTrackerConfig::InternalSwap(PbfTrackerConfig* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  type_fusion_method_.Swap(&other->type_fusion_method_, &::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_type_fusion_method_.get(),
    GetArenaNoVirtual());
  motion_fusion_method_.Swap(&other->motion_fusion_method_, &::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_motion_fusion_method_.get(),
    GetArenaNoVirtual());
  shape_fusion_method_.Swap(&other->shape_fusion_method_, &::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_shape_fusion_method_.get(),
    GetArenaNoVirtual());
  existance_fusion_method_.Swap(&other->existance_fusion_method_, &::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_existance_fusion_method_.get(),
    GetArenaNoVirtual());
}

::PROTOBUF_NAMESPACE_ID::Metadata PbfTrackerConfig::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace fusion
}  // namespace perception
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::perception::fusion::PbfTrackerConfig* Arena::CreateMaybeMessage< ::apollo::perception::fusion::PbfTrackerConfig >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::perception::fusion::PbfTrackerConfig >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
