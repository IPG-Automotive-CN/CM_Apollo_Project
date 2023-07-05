// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/canbus/proto/canbus_conf.proto

#include "modules/canbus/proto/canbus_conf.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_modules_2fcanbus_2fproto_2fvehicle_5fparameter_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_VehicleParameter_modules_2fcanbus_2fproto_2fvehicle_5fparameter_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_modules_2fdrivers_2fcanbus_2fproto_2fcan_5fcard_5fparameter_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_CANCardParameter_modules_2fdrivers_2fcanbus_2fproto_2fcan_5fcard_5fparameter_2eproto;
namespace apollo {
namespace canbus {
class CanbusConfDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<CanbusConf> _instance;
} _CanbusConf_default_instance_;
}  // namespace canbus
}  // namespace apollo
static void InitDefaultsscc_info_CanbusConf_modules_2fcanbus_2fproto_2fcanbus_5fconf_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::canbus::_CanbusConf_default_instance_;
    new (ptr) ::apollo::canbus::CanbusConf();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::canbus::CanbusConf::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<2> scc_info_CanbusConf_modules_2fcanbus_2fproto_2fcanbus_5fconf_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 2, 0, InitDefaultsscc_info_CanbusConf_modules_2fcanbus_2fproto_2fcanbus_5fconf_2eproto}, {
      &scc_info_VehicleParameter_modules_2fcanbus_2fproto_2fvehicle_5fparameter_2eproto.base,
      &scc_info_CANCardParameter_modules_2fdrivers_2fcanbus_2fproto_2fcan_5fcard_5fparameter_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fcanbus_2fproto_2fcanbus_5fconf_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2fcanbus_2fproto_2fcanbus_5fconf_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fcanbus_2fproto_2fcanbus_5fconf_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fcanbus_2fproto_2fcanbus_5fconf_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::canbus::CanbusConf, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::canbus::CanbusConf, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::apollo::canbus::CanbusConf, vehicle_parameter_),
  PROTOBUF_FIELD_OFFSET(::apollo::canbus::CanbusConf, can_card_parameter_),
  PROTOBUF_FIELD_OFFSET(::apollo::canbus::CanbusConf, enable_debug_mode_),
  PROTOBUF_FIELD_OFFSET(::apollo::canbus::CanbusConf, enable_receiver_log_),
  PROTOBUF_FIELD_OFFSET(::apollo::canbus::CanbusConf, enable_sender_log_),
  0,
  1,
  2,
  3,
  4,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 10, sizeof(::apollo::canbus::CanbusConf)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::canbus::_CanbusConf_default_instance_),
};

const char descriptor_table_protodef_modules_2fcanbus_2fproto_2fcanbus_5fconf_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n&modules/canbus/proto/canbus_conf.proto"
  "\022\rapollo.canbus\0325modules/drivers/canbus/"
  "proto/can_card_parameter.proto\032,modules/"
  "canbus/proto/vehicle_parameter.proto\"\365\001\n"
  "\nCanbusConf\022:\n\021vehicle_parameter\030\001 \001(\0132\037"
  ".apollo.canbus.VehicleParameter\022C\n\022can_c"
  "ard_parameter\030\002 \001(\0132\'.apollo.drivers.can"
  "bus.CANCardParameter\022 \n\021enable_debug_mod"
  "e\030\003 \001(\010:\005false\022\"\n\023enable_receiver_log\030\004 "
  "\001(\010:\005false\022 \n\021enable_sender_log\030\005 \001(\010:\005f"
  "alse"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_modules_2fcanbus_2fproto_2fcanbus_5fconf_2eproto_deps[2] = {
  &::descriptor_table_modules_2fcanbus_2fproto_2fvehicle_5fparameter_2eproto,
  &::descriptor_table_modules_2fdrivers_2fcanbus_2fproto_2fcan_5fcard_5fparameter_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_modules_2fcanbus_2fproto_2fcanbus_5fconf_2eproto_sccs[1] = {
  &scc_info_CanbusConf_modules_2fcanbus_2fproto_2fcanbus_5fconf_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fcanbus_2fproto_2fcanbus_5fconf_2eproto_once;
static bool descriptor_table_modules_2fcanbus_2fproto_2fcanbus_5fconf_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fcanbus_2fproto_2fcanbus_5fconf_2eproto = {
  &descriptor_table_modules_2fcanbus_2fproto_2fcanbus_5fconf_2eproto_initialized, descriptor_table_protodef_modules_2fcanbus_2fproto_2fcanbus_5fconf_2eproto, "modules/canbus/proto/canbus_conf.proto", 404,
  &descriptor_table_modules_2fcanbus_2fproto_2fcanbus_5fconf_2eproto_once, descriptor_table_modules_2fcanbus_2fproto_2fcanbus_5fconf_2eproto_sccs, descriptor_table_modules_2fcanbus_2fproto_2fcanbus_5fconf_2eproto_deps, 1, 2,
  schemas, file_default_instances, TableStruct_modules_2fcanbus_2fproto_2fcanbus_5fconf_2eproto::offsets,
  file_level_metadata_modules_2fcanbus_2fproto_2fcanbus_5fconf_2eproto, 1, file_level_enum_descriptors_modules_2fcanbus_2fproto_2fcanbus_5fconf_2eproto, file_level_service_descriptors_modules_2fcanbus_2fproto_2fcanbus_5fconf_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_modules_2fcanbus_2fproto_2fcanbus_5fconf_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_modules_2fcanbus_2fproto_2fcanbus_5fconf_2eproto), true);
namespace apollo {
namespace canbus {

// ===================================================================

void CanbusConf::InitAsDefaultInstance() {
  ::apollo::canbus::_CanbusConf_default_instance_._instance.get_mutable()->vehicle_parameter_ = const_cast< ::apollo::canbus::VehicleParameter*>(
      ::apollo::canbus::VehicleParameter::internal_default_instance());
  ::apollo::canbus::_CanbusConf_default_instance_._instance.get_mutable()->can_card_parameter_ = const_cast< ::apollo::drivers::canbus::CANCardParameter*>(
      ::apollo::drivers::canbus::CANCardParameter::internal_default_instance());
}
class CanbusConf::_Internal {
 public:
  using HasBits = decltype(std::declval<CanbusConf>()._has_bits_);
  static const ::apollo::canbus::VehicleParameter& vehicle_parameter(const CanbusConf* msg);
  static void set_has_vehicle_parameter(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static const ::apollo::drivers::canbus::CANCardParameter& can_card_parameter(const CanbusConf* msg);
  static void set_has_can_card_parameter(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_enable_debug_mode(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_enable_receiver_log(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
  static void set_has_enable_sender_log(HasBits* has_bits) {
    (*has_bits)[0] |= 16u;
  }
};

const ::apollo::canbus::VehicleParameter&
CanbusConf::_Internal::vehicle_parameter(const CanbusConf* msg) {
  return *msg->vehicle_parameter_;
}
const ::apollo::drivers::canbus::CANCardParameter&
CanbusConf::_Internal::can_card_parameter(const CanbusConf* msg) {
  return *msg->can_card_parameter_;
}
void CanbusConf::clear_vehicle_parameter() {
  if (vehicle_parameter_ != nullptr) vehicle_parameter_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
void CanbusConf::clear_can_card_parameter() {
  if (can_card_parameter_ != nullptr) can_card_parameter_->Clear();
  _has_bits_[0] &= ~0x00000002u;
}
CanbusConf::CanbusConf()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.canbus.CanbusConf)
}
CanbusConf::CanbusConf(const CanbusConf& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from._internal_has_vehicle_parameter()) {
    vehicle_parameter_ = new ::apollo::canbus::VehicleParameter(*from.vehicle_parameter_);
  } else {
    vehicle_parameter_ = nullptr;
  }
  if (from._internal_has_can_card_parameter()) {
    can_card_parameter_ = new ::apollo::drivers::canbus::CANCardParameter(*from.can_card_parameter_);
  } else {
    can_card_parameter_ = nullptr;
  }
  ::memcpy(&enable_debug_mode_, &from.enable_debug_mode_,
    static_cast<size_t>(reinterpret_cast<char*>(&enable_sender_log_) -
    reinterpret_cast<char*>(&enable_debug_mode_)) + sizeof(enable_sender_log_));
  // @@protoc_insertion_point(copy_constructor:apollo.canbus.CanbusConf)
}

void CanbusConf::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_CanbusConf_modules_2fcanbus_2fproto_2fcanbus_5fconf_2eproto.base);
  ::memset(&vehicle_parameter_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&enable_sender_log_) -
      reinterpret_cast<char*>(&vehicle_parameter_)) + sizeof(enable_sender_log_));
}

CanbusConf::~CanbusConf() {
  // @@protoc_insertion_point(destructor:apollo.canbus.CanbusConf)
  SharedDtor();
}

void CanbusConf::SharedDtor() {
  if (this != internal_default_instance()) delete vehicle_parameter_;
  if (this != internal_default_instance()) delete can_card_parameter_;
}

void CanbusConf::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const CanbusConf& CanbusConf::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_CanbusConf_modules_2fcanbus_2fproto_2fcanbus_5fconf_2eproto.base);
  return *internal_default_instance();
}


void CanbusConf::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.canbus.CanbusConf)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      GOOGLE_DCHECK(vehicle_parameter_ != nullptr);
      vehicle_parameter_->Clear();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(can_card_parameter_ != nullptr);
      can_card_parameter_->Clear();
    }
  }
  ::memset(&enable_debug_mode_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&enable_sender_log_) -
      reinterpret_cast<char*>(&enable_debug_mode_)) + sizeof(enable_sender_log_));
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* CanbusConf::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional .apollo.canbus.VehicleParameter vehicle_parameter = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_vehicle_parameter(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional .apollo.drivers.canbus.CANCardParameter can_card_parameter = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          ptr = ctx->ParseMessage(_internal_mutable_can_card_parameter(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional bool enable_debug_mode = 3 [default = false];
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 24)) {
          _Internal::set_has_enable_debug_mode(&has_bits);
          enable_debug_mode_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional bool enable_receiver_log = 4 [default = false];
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 32)) {
          _Internal::set_has_enable_receiver_log(&has_bits);
          enable_receiver_log_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional bool enable_sender_log = 5 [default = false];
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 40)) {
          _Internal::set_has_enable_sender_log(&has_bits);
          enable_sender_log_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* CanbusConf::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.canbus.CanbusConf)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.canbus.VehicleParameter vehicle_parameter = 1;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::vehicle_parameter(this), target, stream);
  }

  // optional .apollo.drivers.canbus.CANCardParameter can_card_parameter = 2;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        2, _Internal::can_card_parameter(this), target, stream);
  }

  // optional bool enable_debug_mode = 3 [default = false];
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(3, this->_internal_enable_debug_mode(), target);
  }

  // optional bool enable_receiver_log = 4 [default = false];
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(4, this->_internal_enable_receiver_log(), target);
  }

  // optional bool enable_sender_log = 5 [default = false];
  if (cached_has_bits & 0x00000010u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(5, this->_internal_enable_sender_log(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.canbus.CanbusConf)
  return target;
}

size_t CanbusConf::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.canbus.CanbusConf)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000001fu) {
    // optional .apollo.canbus.VehicleParameter vehicle_parameter = 1;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *vehicle_parameter_);
    }

    // optional .apollo.drivers.canbus.CANCardParameter can_card_parameter = 2;
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *can_card_parameter_);
    }

    // optional bool enable_debug_mode = 3 [default = false];
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 + 1;
    }

    // optional bool enable_receiver_log = 4 [default = false];
    if (cached_has_bits & 0x00000008u) {
      total_size += 1 + 1;
    }

    // optional bool enable_sender_log = 5 [default = false];
    if (cached_has_bits & 0x00000010u) {
      total_size += 1 + 1;
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

void CanbusConf::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.canbus.CanbusConf)
  GOOGLE_DCHECK_NE(&from, this);
  const CanbusConf* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<CanbusConf>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.canbus.CanbusConf)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.canbus.CanbusConf)
    MergeFrom(*source);
  }
}

void CanbusConf::MergeFrom(const CanbusConf& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.canbus.CanbusConf)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x0000001fu) {
    if (cached_has_bits & 0x00000001u) {
      _internal_mutable_vehicle_parameter()->::apollo::canbus::VehicleParameter::MergeFrom(from._internal_vehicle_parameter());
    }
    if (cached_has_bits & 0x00000002u) {
      _internal_mutable_can_card_parameter()->::apollo::drivers::canbus::CANCardParameter::MergeFrom(from._internal_can_card_parameter());
    }
    if (cached_has_bits & 0x00000004u) {
      enable_debug_mode_ = from.enable_debug_mode_;
    }
    if (cached_has_bits & 0x00000008u) {
      enable_receiver_log_ = from.enable_receiver_log_;
    }
    if (cached_has_bits & 0x00000010u) {
      enable_sender_log_ = from.enable_sender_log_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void CanbusConf::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.canbus.CanbusConf)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void CanbusConf::CopyFrom(const CanbusConf& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.canbus.CanbusConf)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool CanbusConf::IsInitialized() const {
  return true;
}

void CanbusConf::InternalSwap(CanbusConf* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(vehicle_parameter_, other->vehicle_parameter_);
  swap(can_card_parameter_, other->can_card_parameter_);
  swap(enable_debug_mode_, other->enable_debug_mode_);
  swap(enable_receiver_log_, other->enable_receiver_log_);
  swap(enable_sender_log_, other->enable_sender_log_);
}

::PROTOBUF_NAMESPACE_ID::Metadata CanbusConf::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace canbus
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::canbus::CanbusConf* Arena::CreateMaybeMessage< ::apollo::canbus::CanbusConf >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::canbus::CanbusConf >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>