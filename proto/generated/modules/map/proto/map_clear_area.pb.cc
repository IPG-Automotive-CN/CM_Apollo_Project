// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/map/proto/map_clear_area.proto

#include "modules/map/proto/map_clear_area.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_modules_2fmap_2fproto_2fmap_5fid_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_Id_modules_2fmap_2fproto_2fmap_5fid_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_modules_2fmap_2fproto_2fmap_5fgeometry_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_Polygon_modules_2fmap_2fproto_2fmap_5fgeometry_2eproto;
namespace apollo {
namespace hdmap {
class ClearAreaDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<ClearArea> _instance;
} _ClearArea_default_instance_;
}  // namespace hdmap
}  // namespace apollo
static void InitDefaultsscc_info_ClearArea_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::hdmap::_ClearArea_default_instance_;
    new (ptr) ::apollo::hdmap::ClearArea();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::hdmap::ClearArea::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<2> scc_info_ClearArea_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 2, 0, InitDefaultsscc_info_ClearArea_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto}, {
      &scc_info_Id_modules_2fmap_2fproto_2fmap_5fid_2eproto.base,
      &scc_info_Polygon_modules_2fmap_2fproto_2fmap_5fgeometry_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::hdmap::ClearArea, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::hdmap::ClearArea, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::apollo::hdmap::ClearArea, id_),
  PROTOBUF_FIELD_OFFSET(::apollo::hdmap::ClearArea, overlap_id_),
  PROTOBUF_FIELD_OFFSET(::apollo::hdmap::ClearArea, polygon_),
  0,
  ~0u,
  1,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, sizeof(::apollo::hdmap::ClearArea)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::hdmap::_ClearArea_default_instance_),
};

const char descriptor_table_protodef_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n&modules/map/proto/map_clear_area.proto"
  "\022\014apollo.hdmap\032\036modules/map/proto/map_id"
  ".proto\032$modules/map/proto/map_geometry.p"
  "roto\"w\n\tClearArea\022\034\n\002id\030\001 \001(\0132\020.apollo.h"
  "dmap.Id\022$\n\noverlap_id\030\002 \003(\0132\020.apollo.hdm"
  "ap.Id\022&\n\007polygon\030\003 \001(\0132\025.apollo.hdmap.Po"
  "lygon"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto_deps[2] = {
  &::descriptor_table_modules_2fmap_2fproto_2fmap_5fgeometry_2eproto,
  &::descriptor_table_modules_2fmap_2fproto_2fmap_5fid_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto_sccs[1] = {
  &scc_info_ClearArea_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto_once;
static bool descriptor_table_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto = {
  &descriptor_table_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto_initialized, descriptor_table_protodef_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto, "modules/map/proto/map_clear_area.proto", 245,
  &descriptor_table_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto_once, descriptor_table_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto_sccs, descriptor_table_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto_deps, 1, 2,
  schemas, file_default_instances, TableStruct_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto::offsets,
  file_level_metadata_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto, 1, file_level_enum_descriptors_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto, file_level_service_descriptors_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto), true);
namespace apollo {
namespace hdmap {

// ===================================================================

void ClearArea::InitAsDefaultInstance() {
  ::apollo::hdmap::_ClearArea_default_instance_._instance.get_mutable()->id_ = const_cast< ::apollo::hdmap::Id*>(
      ::apollo::hdmap::Id::internal_default_instance());
  ::apollo::hdmap::_ClearArea_default_instance_._instance.get_mutable()->polygon_ = const_cast< ::apollo::hdmap::Polygon*>(
      ::apollo::hdmap::Polygon::internal_default_instance());
}
class ClearArea::_Internal {
 public:
  using HasBits = decltype(std::declval<ClearArea>()._has_bits_);
  static const ::apollo::hdmap::Id& id(const ClearArea* msg);
  static void set_has_id(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static const ::apollo::hdmap::Polygon& polygon(const ClearArea* msg);
  static void set_has_polygon(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
};

const ::apollo::hdmap::Id&
ClearArea::_Internal::id(const ClearArea* msg) {
  return *msg->id_;
}
const ::apollo::hdmap::Polygon&
ClearArea::_Internal::polygon(const ClearArea* msg) {
  return *msg->polygon_;
}
void ClearArea::clear_id() {
  if (id_ != nullptr) id_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
void ClearArea::clear_overlap_id() {
  overlap_id_.Clear();
}
void ClearArea::clear_polygon() {
  if (polygon_ != nullptr) polygon_->Clear();
  _has_bits_[0] &= ~0x00000002u;
}
ClearArea::ClearArea()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.hdmap.ClearArea)
}
ClearArea::ClearArea(const ClearArea& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      overlap_id_(from.overlap_id_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from._internal_has_id()) {
    id_ = new ::apollo::hdmap::Id(*from.id_);
  } else {
    id_ = nullptr;
  }
  if (from._internal_has_polygon()) {
    polygon_ = new ::apollo::hdmap::Polygon(*from.polygon_);
  } else {
    polygon_ = nullptr;
  }
  // @@protoc_insertion_point(copy_constructor:apollo.hdmap.ClearArea)
}

void ClearArea::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_ClearArea_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto.base);
  ::memset(&id_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&polygon_) -
      reinterpret_cast<char*>(&id_)) + sizeof(polygon_));
}

ClearArea::~ClearArea() {
  // @@protoc_insertion_point(destructor:apollo.hdmap.ClearArea)
  SharedDtor();
}

void ClearArea::SharedDtor() {
  if (this != internal_default_instance()) delete id_;
  if (this != internal_default_instance()) delete polygon_;
}

void ClearArea::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ClearArea& ClearArea::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_ClearArea_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto.base);
  return *internal_default_instance();
}


void ClearArea::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.hdmap.ClearArea)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  overlap_id_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      GOOGLE_DCHECK(id_ != nullptr);
      id_->Clear();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(polygon_ != nullptr);
      polygon_->Clear();
    }
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* ClearArea::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional .apollo.hdmap.Id id = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_id(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // repeated .apollo.hdmap.Id overlap_id = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_overlap_id(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<18>(ptr));
        } else goto handle_unusual;
        continue;
      // optional .apollo.hdmap.Polygon polygon = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 26)) {
          ptr = ctx->ParseMessage(_internal_mutable_polygon(), ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* ClearArea::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.hdmap.ClearArea)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.hdmap.Id id = 1;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::id(this), target, stream);
  }

  // repeated .apollo.hdmap.Id overlap_id = 2;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_overlap_id_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(2, this->_internal_overlap_id(i), target, stream);
  }

  // optional .apollo.hdmap.Polygon polygon = 3;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        3, _Internal::polygon(this), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.hdmap.ClearArea)
  return target;
}

size_t ClearArea::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.hdmap.ClearArea)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .apollo.hdmap.Id overlap_id = 2;
  total_size += 1UL * this->_internal_overlap_id_size();
  for (const auto& msg : this->overlap_id_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    // optional .apollo.hdmap.Id id = 1;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *id_);
    }

    // optional .apollo.hdmap.Polygon polygon = 3;
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *polygon_);
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

void ClearArea::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.hdmap.ClearArea)
  GOOGLE_DCHECK_NE(&from, this);
  const ClearArea* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<ClearArea>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.hdmap.ClearArea)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.hdmap.ClearArea)
    MergeFrom(*source);
  }
}

void ClearArea::MergeFrom(const ClearArea& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.hdmap.ClearArea)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  overlap_id_.MergeFrom(from.overlap_id_);
  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      _internal_mutable_id()->::apollo::hdmap::Id::MergeFrom(from._internal_id());
    }
    if (cached_has_bits & 0x00000002u) {
      _internal_mutable_polygon()->::apollo::hdmap::Polygon::MergeFrom(from._internal_polygon());
    }
  }
}

void ClearArea::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.hdmap.ClearArea)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ClearArea::CopyFrom(const ClearArea& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.hdmap.ClearArea)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ClearArea::IsInitialized() const {
  return true;
}

void ClearArea::InternalSwap(ClearArea* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  overlap_id_.InternalSwap(&other->overlap_id_);
  swap(id_, other->id_);
  swap(polygon_, other->polygon_);
}

::PROTOBUF_NAMESPACE_ID::Metadata ClearArea::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace hdmap
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::hdmap::ClearArea* Arena::CreateMaybeMessage< ::apollo::hdmap::ClearArea >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::hdmap::ClearArea >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
