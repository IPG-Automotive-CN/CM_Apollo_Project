// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/perception/proto/probabilistic_fusion_config.proto

#include "modules/perception/proto/probabilistic_fusion_config.pb.h"

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
class ProbabilisticFusionConfigDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<ProbabilisticFusionConfig> _instance;
} _ProbabilisticFusionConfig_default_instance_;
}  // namespace fusion
}  // namespace perception
}  // namespace apollo
static void InitDefaultsscc_info_ProbabilisticFusionConfig_modules_2fperception_2fproto_2fprobabilistic_5ffusion_5fconfig_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::apollo::perception::fusion::ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_tracker_method_.DefaultConstruct();
  *::apollo::perception::fusion::ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_tracker_method_.get_mutable() = std::string("PbfTracker", 10);
  ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyString(
      ::apollo::perception::fusion::ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_tracker_method_.get_mutable());
  ::apollo::perception::fusion::ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_data_association_method_.DefaultConstruct();
  *::apollo::perception::fusion::ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_data_association_method_.get_mutable() = std::string("HMAssociation", 13);
  ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyString(
      ::apollo::perception::fusion::ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_data_association_method_.get_mutable());
  ::apollo::perception::fusion::ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_gate_keeper_method_.DefaultConstruct();
  *::apollo::perception::fusion::ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_gate_keeper_method_.get_mutable() = std::string("PbfGatekeeper", 13);
  ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyString(
      ::apollo::perception::fusion::ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_gate_keeper_method_.get_mutable());
  {
    void* ptr = &::apollo::perception::fusion::_ProbabilisticFusionConfig_default_instance_;
    new (ptr) ::apollo::perception::fusion::ProbabilisticFusionConfig();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::perception::fusion::ProbabilisticFusionConfig::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_ProbabilisticFusionConfig_modules_2fperception_2fproto_2fprobabilistic_5ffusion_5fconfig_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_ProbabilisticFusionConfig_modules_2fperception_2fproto_2fprobabilistic_5ffusion_5fconfig_2eproto}, {}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fperception_2fproto_2fprobabilistic_5ffusion_5fconfig_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2fperception_2fproto_2fprobabilistic_5ffusion_5fconfig_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fperception_2fproto_2fprobabilistic_5ffusion_5fconfig_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fperception_2fproto_2fprobabilistic_5ffusion_5fconfig_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::perception::fusion::ProbabilisticFusionConfig, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::fusion::ProbabilisticFusionConfig, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::apollo::perception::fusion::ProbabilisticFusionConfig, use_lidar_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::fusion::ProbabilisticFusionConfig, use_radar_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::fusion::ProbabilisticFusionConfig, use_camera_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::fusion::ProbabilisticFusionConfig, tracker_method_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::fusion::ProbabilisticFusionConfig, data_association_method_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::fusion::ProbabilisticFusionConfig, gate_keeper_method_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::fusion::ProbabilisticFusionConfig, prohibition_sensors_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::fusion::ProbabilisticFusionConfig, max_lidar_invisible_period_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::fusion::ProbabilisticFusionConfig, max_radar_invisible_period_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::fusion::ProbabilisticFusionConfig, max_camera_invisible_period_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::fusion::ProbabilisticFusionConfig, max_cached_frame_num_),
  3,
  4,
  5,
  0,
  1,
  2,
  ~0u,
  6,
  7,
  8,
  9,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 16, sizeof(::apollo::perception::fusion::ProbabilisticFusionConfig)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::perception::fusion::_ProbabilisticFusionConfig_default_instance_),
};

const char descriptor_table_protodef_modules_2fperception_2fproto_2fprobabilistic_5ffusion_5fconfig_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n:modules/perception/proto/probabilistic"
  "_fusion_config.proto\022\030apollo.perception."
  "fusion\"\243\003\n\031ProbabilisticFusionConfig\022\027\n\t"
  "use_lidar\030\001 \001(\010:\004true\022\027\n\tuse_radar\030\002 \001(\010"
  ":\004true\022\030\n\nuse_camera\030\003 \001(\010:\004true\022\"\n\016trac"
  "ker_method\030\004 \001(\t:\nPbfTracker\022.\n\027data_ass"
  "ociation_method\030\005 \001(\t:\rHMAssociation\022)\n\022"
  "gate_keeper_method\030\006 \001(\t:\rPbfGatekeeper\022"
  "\033\n\023prohibition_sensors\030\007 \003(\t\022(\n\032max_lida"
  "r_invisible_period\030\010 \001(\001:\0040.25\022\'\n\032max_ra"
  "dar_invisible_period\030\t \001(\001:\0030.5\022)\n\033max_c"
  "amera_invisible_period\030\n \001(\001:\0040.75\022 \n\024ma"
  "x_cached_frame_num\030\013 \001(\003:\00250"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_modules_2fperception_2fproto_2fprobabilistic_5ffusion_5fconfig_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_modules_2fperception_2fproto_2fprobabilistic_5ffusion_5fconfig_2eproto_sccs[1] = {
  &scc_info_ProbabilisticFusionConfig_modules_2fperception_2fproto_2fprobabilistic_5ffusion_5fconfig_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fperception_2fproto_2fprobabilistic_5ffusion_5fconfig_2eproto_once;
static bool descriptor_table_modules_2fperception_2fproto_2fprobabilistic_5ffusion_5fconfig_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fperception_2fproto_2fprobabilistic_5ffusion_5fconfig_2eproto = {
  &descriptor_table_modules_2fperception_2fproto_2fprobabilistic_5ffusion_5fconfig_2eproto_initialized, descriptor_table_protodef_modules_2fperception_2fproto_2fprobabilistic_5ffusion_5fconfig_2eproto, "modules/perception/proto/probabilistic_fusion_config.proto", 508,
  &descriptor_table_modules_2fperception_2fproto_2fprobabilistic_5ffusion_5fconfig_2eproto_once, descriptor_table_modules_2fperception_2fproto_2fprobabilistic_5ffusion_5fconfig_2eproto_sccs, descriptor_table_modules_2fperception_2fproto_2fprobabilistic_5ffusion_5fconfig_2eproto_deps, 1, 0,
  schemas, file_default_instances, TableStruct_modules_2fperception_2fproto_2fprobabilistic_5ffusion_5fconfig_2eproto::offsets,
  file_level_metadata_modules_2fperception_2fproto_2fprobabilistic_5ffusion_5fconfig_2eproto, 1, file_level_enum_descriptors_modules_2fperception_2fproto_2fprobabilistic_5ffusion_5fconfig_2eproto, file_level_service_descriptors_modules_2fperception_2fproto_2fprobabilistic_5ffusion_5fconfig_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_modules_2fperception_2fproto_2fprobabilistic_5ffusion_5fconfig_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_modules_2fperception_2fproto_2fprobabilistic_5ffusion_5fconfig_2eproto), true);
namespace apollo {
namespace perception {
namespace fusion {

// ===================================================================

void ProbabilisticFusionConfig::InitAsDefaultInstance() {
}
class ProbabilisticFusionConfig::_Internal {
 public:
  using HasBits = decltype(std::declval<ProbabilisticFusionConfig>()._has_bits_);
  static void set_has_use_lidar(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
  static void set_has_use_radar(HasBits* has_bits) {
    (*has_bits)[0] |= 16u;
  }
  static void set_has_use_camera(HasBits* has_bits) {
    (*has_bits)[0] |= 32u;
  }
  static void set_has_tracker_method(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_data_association_method(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_gate_keeper_method(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_max_lidar_invisible_period(HasBits* has_bits) {
    (*has_bits)[0] |= 64u;
  }
  static void set_has_max_radar_invisible_period(HasBits* has_bits) {
    (*has_bits)[0] |= 128u;
  }
  static void set_has_max_camera_invisible_period(HasBits* has_bits) {
    (*has_bits)[0] |= 256u;
  }
  static void set_has_max_cached_frame_num(HasBits* has_bits) {
    (*has_bits)[0] |= 512u;
  }
};

::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<std::string> ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_tracker_method_;
::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<std::string> ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_data_association_method_;
::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<std::string> ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_gate_keeper_method_;
ProbabilisticFusionConfig::ProbabilisticFusionConfig()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.perception.fusion.ProbabilisticFusionConfig)
}
ProbabilisticFusionConfig::ProbabilisticFusionConfig(const ProbabilisticFusionConfig& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      prohibition_sensors_(from.prohibition_sensors_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  tracker_method_.UnsafeSetDefault(&::apollo::perception::fusion::ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_tracker_method_.get());
  if (from._internal_has_tracker_method()) {
    tracker_method_.AssignWithDefault(&::apollo::perception::fusion::ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_tracker_method_.get(), from.tracker_method_);
  }
  data_association_method_.UnsafeSetDefault(&::apollo::perception::fusion::ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_data_association_method_.get());
  if (from._internal_has_data_association_method()) {
    data_association_method_.AssignWithDefault(&::apollo::perception::fusion::ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_data_association_method_.get(), from.data_association_method_);
  }
  gate_keeper_method_.UnsafeSetDefault(&::apollo::perception::fusion::ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_gate_keeper_method_.get());
  if (from._internal_has_gate_keeper_method()) {
    gate_keeper_method_.AssignWithDefault(&::apollo::perception::fusion::ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_gate_keeper_method_.get(), from.gate_keeper_method_);
  }
  ::memcpy(&use_lidar_, &from.use_lidar_,
    static_cast<size_t>(reinterpret_cast<char*>(&max_cached_frame_num_) -
    reinterpret_cast<char*>(&use_lidar_)) + sizeof(max_cached_frame_num_));
  // @@protoc_insertion_point(copy_constructor:apollo.perception.fusion.ProbabilisticFusionConfig)
}

void ProbabilisticFusionConfig::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_ProbabilisticFusionConfig_modules_2fperception_2fproto_2fprobabilistic_5ffusion_5fconfig_2eproto.base);
  tracker_method_.UnsafeSetDefault(&::apollo::perception::fusion::ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_tracker_method_.get());
  data_association_method_.UnsafeSetDefault(&::apollo::perception::fusion::ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_data_association_method_.get());
  gate_keeper_method_.UnsafeSetDefault(&::apollo::perception::fusion::ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_gate_keeper_method_.get());
  use_lidar_ = true;
  use_radar_ = true;
  use_camera_ = true;
  max_lidar_invisible_period_ = 0.25;
  max_radar_invisible_period_ = 0.5;
  max_camera_invisible_period_ = 0.75;
  max_cached_frame_num_ = PROTOBUF_LONGLONG(50);
}

ProbabilisticFusionConfig::~ProbabilisticFusionConfig() {
  // @@protoc_insertion_point(destructor:apollo.perception.fusion.ProbabilisticFusionConfig)
  SharedDtor();
}

void ProbabilisticFusionConfig::SharedDtor() {
  tracker_method_.DestroyNoArena(&::apollo::perception::fusion::ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_tracker_method_.get());
  data_association_method_.DestroyNoArena(&::apollo::perception::fusion::ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_data_association_method_.get());
  gate_keeper_method_.DestroyNoArena(&::apollo::perception::fusion::ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_gate_keeper_method_.get());
}

void ProbabilisticFusionConfig::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ProbabilisticFusionConfig& ProbabilisticFusionConfig::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_ProbabilisticFusionConfig_modules_2fperception_2fproto_2fprobabilistic_5ffusion_5fconfig_2eproto.base);
  return *internal_default_instance();
}


void ProbabilisticFusionConfig::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.perception.fusion.ProbabilisticFusionConfig)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  prohibition_sensors_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x000000ffu) {
    if (cached_has_bits & 0x00000001u) {
      tracker_method_.UnsafeMutablePointer()->assign(*&::apollo::perception::fusion::ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_tracker_method_.get());
    }
    if (cached_has_bits & 0x00000002u) {
      data_association_method_.UnsafeMutablePointer()->assign(*&::apollo::perception::fusion::ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_data_association_method_.get());
    }
    if (cached_has_bits & 0x00000004u) {
      gate_keeper_method_.UnsafeMutablePointer()->assign(*&::apollo::perception::fusion::ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_gate_keeper_method_.get());
    }
    use_lidar_ = true;
    use_radar_ = true;
    use_camera_ = true;
    max_lidar_invisible_period_ = 0.25;
    max_radar_invisible_period_ = 0.5;
  }
  if (cached_has_bits & 0x00000300u) {
    max_camera_invisible_period_ = 0.75;
    max_cached_frame_num_ = PROTOBUF_LONGLONG(50);
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* ProbabilisticFusionConfig::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional bool use_lidar = 1 [default = true];
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 8)) {
          _Internal::set_has_use_lidar(&has_bits);
          use_lidar_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional bool use_radar = 2 [default = true];
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 16)) {
          _Internal::set_has_use_radar(&has_bits);
          use_radar_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional bool use_camera = 3 [default = true];
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 24)) {
          _Internal::set_has_use_camera(&has_bits);
          use_camera_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional string tracker_method = 4 [default = "PbfTracker"];
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 34)) {
          auto str = _internal_mutable_tracker_method();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          #ifndef NDEBUG
          ::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "apollo.perception.fusion.ProbabilisticFusionConfig.tracker_method");
          #endif  // !NDEBUG
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional string data_association_method = 5 [default = "HMAssociation"];
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 42)) {
          auto str = _internal_mutable_data_association_method();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          #ifndef NDEBUG
          ::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "apollo.perception.fusion.ProbabilisticFusionConfig.data_association_method");
          #endif  // !NDEBUG
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional string gate_keeper_method = 6 [default = "PbfGatekeeper"];
      case 6:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 50)) {
          auto str = _internal_mutable_gate_keeper_method();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          #ifndef NDEBUG
          ::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "apollo.perception.fusion.ProbabilisticFusionConfig.gate_keeper_method");
          #endif  // !NDEBUG
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // repeated string prohibition_sensors = 7;
      case 7:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 58)) {
          ptr -= 1;
          do {
            ptr += 1;
            auto str = _internal_add_prohibition_sensors();
            ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
            #ifndef NDEBUG
            ::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "apollo.perception.fusion.ProbabilisticFusionConfig.prohibition_sensors");
            #endif  // !NDEBUG
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<58>(ptr));
        } else goto handle_unusual;
        continue;
      // optional double max_lidar_invisible_period = 8 [default = 0.25];
      case 8:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 65)) {
          _Internal::set_has_max_lidar_invisible_period(&has_bits);
          max_lidar_invisible_period_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else goto handle_unusual;
        continue;
      // optional double max_radar_invisible_period = 9 [default = 0.5];
      case 9:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 73)) {
          _Internal::set_has_max_radar_invisible_period(&has_bits);
          max_radar_invisible_period_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else goto handle_unusual;
        continue;
      // optional double max_camera_invisible_period = 10 [default = 0.75];
      case 10:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 81)) {
          _Internal::set_has_max_camera_invisible_period(&has_bits);
          max_camera_invisible_period_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else goto handle_unusual;
        continue;
      // optional int64 max_cached_frame_num = 11 [default = 50];
      case 11:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 88)) {
          _Internal::set_has_max_cached_frame_num(&has_bits);
          max_cached_frame_num_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* ProbabilisticFusionConfig::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.perception.fusion.ProbabilisticFusionConfig)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional bool use_lidar = 1 [default = true];
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(1, this->_internal_use_lidar(), target);
  }

  // optional bool use_radar = 2 [default = true];
  if (cached_has_bits & 0x00000010u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(2, this->_internal_use_radar(), target);
  }

  // optional bool use_camera = 3 [default = true];
  if (cached_has_bits & 0x00000020u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(3, this->_internal_use_camera(), target);
  }

  // optional string tracker_method = 4 [default = "PbfTracker"];
  if (cached_has_bits & 0x00000001u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_tracker_method().data(), static_cast<int>(this->_internal_tracker_method().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.perception.fusion.ProbabilisticFusionConfig.tracker_method");
    target = stream->WriteStringMaybeAliased(
        4, this->_internal_tracker_method(), target);
  }

  // optional string data_association_method = 5 [default = "HMAssociation"];
  if (cached_has_bits & 0x00000002u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_data_association_method().data(), static_cast<int>(this->_internal_data_association_method().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.perception.fusion.ProbabilisticFusionConfig.data_association_method");
    target = stream->WriteStringMaybeAliased(
        5, this->_internal_data_association_method(), target);
  }

  // optional string gate_keeper_method = 6 [default = "PbfGatekeeper"];
  if (cached_has_bits & 0x00000004u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_gate_keeper_method().data(), static_cast<int>(this->_internal_gate_keeper_method().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.perception.fusion.ProbabilisticFusionConfig.gate_keeper_method");
    target = stream->WriteStringMaybeAliased(
        6, this->_internal_gate_keeper_method(), target);
  }

  // repeated string prohibition_sensors = 7;
  for (int i = 0, n = this->_internal_prohibition_sensors_size(); i < n; i++) {
    const auto& s = this->_internal_prohibition_sensors(i);
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      s.data(), static_cast<int>(s.length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.perception.fusion.ProbabilisticFusionConfig.prohibition_sensors");
    target = stream->WriteString(7, s, target);
  }

  // optional double max_lidar_invisible_period = 8 [default = 0.25];
  if (cached_has_bits & 0x00000040u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(8, this->_internal_max_lidar_invisible_period(), target);
  }

  // optional double max_radar_invisible_period = 9 [default = 0.5];
  if (cached_has_bits & 0x00000080u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(9, this->_internal_max_radar_invisible_period(), target);
  }

  // optional double max_camera_invisible_period = 10 [default = 0.75];
  if (cached_has_bits & 0x00000100u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(10, this->_internal_max_camera_invisible_period(), target);
  }

  // optional int64 max_cached_frame_num = 11 [default = 50];
  if (cached_has_bits & 0x00000200u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt64ToArray(11, this->_internal_max_cached_frame_num(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.perception.fusion.ProbabilisticFusionConfig)
  return target;
}

size_t ProbabilisticFusionConfig::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.perception.fusion.ProbabilisticFusionConfig)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated string prohibition_sensors = 7;
  total_size += 1 *
      ::PROTOBUF_NAMESPACE_ID::internal::FromIntSize(prohibition_sensors_.size());
  for (int i = 0, n = prohibition_sensors_.size(); i < n; i++) {
    total_size += ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
      prohibition_sensors_.Get(i));
  }

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x000000ffu) {
    // optional string tracker_method = 4 [default = "PbfTracker"];
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_tracker_method());
    }

    // optional string data_association_method = 5 [default = "HMAssociation"];
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_data_association_method());
    }

    // optional string gate_keeper_method = 6 [default = "PbfGatekeeper"];
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_gate_keeper_method());
    }

    // optional bool use_lidar = 1 [default = true];
    if (cached_has_bits & 0x00000008u) {
      total_size += 1 + 1;
    }

    // optional bool use_radar = 2 [default = true];
    if (cached_has_bits & 0x00000010u) {
      total_size += 1 + 1;
    }

    // optional bool use_camera = 3 [default = true];
    if (cached_has_bits & 0x00000020u) {
      total_size += 1 + 1;
    }

    // optional double max_lidar_invisible_period = 8 [default = 0.25];
    if (cached_has_bits & 0x00000040u) {
      total_size += 1 + 8;
    }

    // optional double max_radar_invisible_period = 9 [default = 0.5];
    if (cached_has_bits & 0x00000080u) {
      total_size += 1 + 8;
    }

  }
  if (cached_has_bits & 0x00000300u) {
    // optional double max_camera_invisible_period = 10 [default = 0.75];
    if (cached_has_bits & 0x00000100u) {
      total_size += 1 + 8;
    }

    // optional int64 max_cached_frame_num = 11 [default = 50];
    if (cached_has_bits & 0x00000200u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int64Size(
          this->_internal_max_cached_frame_num());
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

void ProbabilisticFusionConfig::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.perception.fusion.ProbabilisticFusionConfig)
  GOOGLE_DCHECK_NE(&from, this);
  const ProbabilisticFusionConfig* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<ProbabilisticFusionConfig>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.perception.fusion.ProbabilisticFusionConfig)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.perception.fusion.ProbabilisticFusionConfig)
    MergeFrom(*source);
  }
}

void ProbabilisticFusionConfig::MergeFrom(const ProbabilisticFusionConfig& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.perception.fusion.ProbabilisticFusionConfig)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  prohibition_sensors_.MergeFrom(from.prohibition_sensors_);
  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x000000ffu) {
    if (cached_has_bits & 0x00000001u) {
      _has_bits_[0] |= 0x00000001u;
      tracker_method_.AssignWithDefault(&::apollo::perception::fusion::ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_tracker_method_.get(), from.tracker_method_);
    }
    if (cached_has_bits & 0x00000002u) {
      _has_bits_[0] |= 0x00000002u;
      data_association_method_.AssignWithDefault(&::apollo::perception::fusion::ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_data_association_method_.get(), from.data_association_method_);
    }
    if (cached_has_bits & 0x00000004u) {
      _has_bits_[0] |= 0x00000004u;
      gate_keeper_method_.AssignWithDefault(&::apollo::perception::fusion::ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_gate_keeper_method_.get(), from.gate_keeper_method_);
    }
    if (cached_has_bits & 0x00000008u) {
      use_lidar_ = from.use_lidar_;
    }
    if (cached_has_bits & 0x00000010u) {
      use_radar_ = from.use_radar_;
    }
    if (cached_has_bits & 0x00000020u) {
      use_camera_ = from.use_camera_;
    }
    if (cached_has_bits & 0x00000040u) {
      max_lidar_invisible_period_ = from.max_lidar_invisible_period_;
    }
    if (cached_has_bits & 0x00000080u) {
      max_radar_invisible_period_ = from.max_radar_invisible_period_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
  if (cached_has_bits & 0x00000300u) {
    if (cached_has_bits & 0x00000100u) {
      max_camera_invisible_period_ = from.max_camera_invisible_period_;
    }
    if (cached_has_bits & 0x00000200u) {
      max_cached_frame_num_ = from.max_cached_frame_num_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void ProbabilisticFusionConfig::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.perception.fusion.ProbabilisticFusionConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ProbabilisticFusionConfig::CopyFrom(const ProbabilisticFusionConfig& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.perception.fusion.ProbabilisticFusionConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ProbabilisticFusionConfig::IsInitialized() const {
  return true;
}

void ProbabilisticFusionConfig::InternalSwap(ProbabilisticFusionConfig* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  prohibition_sensors_.InternalSwap(&other->prohibition_sensors_);
  tracker_method_.Swap(&other->tracker_method_, &::apollo::perception::fusion::ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_tracker_method_.get(),
    GetArenaNoVirtual());
  data_association_method_.Swap(&other->data_association_method_, &::apollo::perception::fusion::ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_data_association_method_.get(),
    GetArenaNoVirtual());
  gate_keeper_method_.Swap(&other->gate_keeper_method_, &::apollo::perception::fusion::ProbabilisticFusionConfig::_i_give_permission_to_break_this_code_default_gate_keeper_method_.get(),
    GetArenaNoVirtual());
  swap(use_lidar_, other->use_lidar_);
  swap(use_radar_, other->use_radar_);
  swap(use_camera_, other->use_camera_);
  swap(max_lidar_invisible_period_, other->max_lidar_invisible_period_);
  swap(max_radar_invisible_period_, other->max_radar_invisible_period_);
  swap(max_camera_invisible_period_, other->max_camera_invisible_period_);
  swap(max_cached_frame_num_, other->max_cached_frame_num_);
}

::PROTOBUF_NAMESPACE_ID::Metadata ProbabilisticFusionConfig::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace fusion
}  // namespace perception
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::perception::fusion::ProbabilisticFusionConfig* Arena::CreateMaybeMessage< ::apollo::perception::fusion::ProbabilisticFusionConfig >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::perception::fusion::ProbabilisticFusionConfig >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>