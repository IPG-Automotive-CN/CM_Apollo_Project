// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/prediction/proto/fnn_model_base.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto

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
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxillaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[3]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto;
namespace apollo {
namespace prediction {
class Layer;
class LayerDefaultTypeInternal;
extern LayerDefaultTypeInternal _Layer_default_instance_;
class Matrix;
class MatrixDefaultTypeInternal;
extern MatrixDefaultTypeInternal _Matrix_default_instance_;
class Vector;
class VectorDefaultTypeInternal;
extern VectorDefaultTypeInternal _Vector_default_instance_;
}  // namespace prediction
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::prediction::Layer* Arena::CreateMaybeMessage<::apollo::prediction::Layer>(Arena*);
template<> ::apollo::prediction::Matrix* Arena::CreateMaybeMessage<::apollo::prediction::Matrix>(Arena*);
template<> ::apollo::prediction::Vector* Arena::CreateMaybeMessage<::apollo::prediction::Vector>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace prediction {

enum Layer_ActivationFunc : int {
  Layer_ActivationFunc_RELU = 0,
  Layer_ActivationFunc_TANH = 1,
  Layer_ActivationFunc_SIGMOID = 2,
  Layer_ActivationFunc_SOFTMAX = 3
};
bool Layer_ActivationFunc_IsValid(int value);
constexpr Layer_ActivationFunc Layer_ActivationFunc_ActivationFunc_MIN = Layer_ActivationFunc_RELU;
constexpr Layer_ActivationFunc Layer_ActivationFunc_ActivationFunc_MAX = Layer_ActivationFunc_SOFTMAX;
constexpr int Layer_ActivationFunc_ActivationFunc_ARRAYSIZE = Layer_ActivationFunc_ActivationFunc_MAX + 1;

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* Layer_ActivationFunc_descriptor();
template<typename T>
inline const std::string& Layer_ActivationFunc_Name(T enum_t_value) {
  static_assert(::std::is_same<T, Layer_ActivationFunc>::value ||
    ::std::is_integral<T>::value,
    "Incorrect type passed to function Layer_ActivationFunc_Name.");
  return ::PROTOBUF_NAMESPACE_ID::internal::NameOfEnum(
    Layer_ActivationFunc_descriptor(), enum_t_value);
}
inline bool Layer_ActivationFunc_Parse(
    const std::string& name, Layer_ActivationFunc* value) {
  return ::PROTOBUF_NAMESPACE_ID::internal::ParseNamedEnum<Layer_ActivationFunc>(
    Layer_ActivationFunc_descriptor(), name, value);
}
// ===================================================================

class Vector :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.prediction.Vector) */ {
 public:
  Vector();
  virtual ~Vector();

  Vector(const Vector& from);
  Vector(Vector&& from) noexcept
    : Vector() {
    *this = ::std::move(from);
  }

  inline Vector& operator=(const Vector& from) {
    CopyFrom(from);
    return *this;
  }
  inline Vector& operator=(Vector&& from) noexcept {
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
  static const Vector& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Vector* internal_default_instance() {
    return reinterpret_cast<const Vector*>(
               &_Vector_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(Vector& a, Vector& b) {
    a.Swap(&b);
  }
  inline void Swap(Vector* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline Vector* New() const final {
    return CreateMaybeMessage<Vector>(nullptr);
  }

  Vector* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<Vector>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const Vector& from);
  void MergeFrom(const Vector& from);
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
  void InternalSwap(Vector* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.prediction.Vector";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto);
    return ::descriptor_table_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kColumnsFieldNumber = 1,
  };
  // repeated double columns = 1;
  int columns_size() const;
  private:
  int _internal_columns_size() const;
  public:
  void clear_columns();
  private:
  double _internal_columns(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      _internal_columns() const;
  void _internal_add_columns(double value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      _internal_mutable_columns();
  public:
  double columns(int index) const;
  void set_columns(int index, double value);
  void add_columns(double value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      columns() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      mutable_columns();

  // @@protoc_insertion_point(class_scope:apollo.prediction.Vector)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double > columns_;
  friend struct ::TableStruct_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto;
};
// -------------------------------------------------------------------

class Matrix :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.prediction.Matrix) */ {
 public:
  Matrix();
  virtual ~Matrix();

  Matrix(const Matrix& from);
  Matrix(Matrix&& from) noexcept
    : Matrix() {
    *this = ::std::move(from);
  }

  inline Matrix& operator=(const Matrix& from) {
    CopyFrom(from);
    return *this;
  }
  inline Matrix& operator=(Matrix&& from) noexcept {
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
  static const Matrix& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Matrix* internal_default_instance() {
    return reinterpret_cast<const Matrix*>(
               &_Matrix_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(Matrix& a, Matrix& b) {
    a.Swap(&b);
  }
  inline void Swap(Matrix* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline Matrix* New() const final {
    return CreateMaybeMessage<Matrix>(nullptr);
  }

  Matrix* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<Matrix>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const Matrix& from);
  void MergeFrom(const Matrix& from);
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
  void InternalSwap(Matrix* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.prediction.Matrix";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto);
    return ::descriptor_table_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kRowsFieldNumber = 1,
  };
  // repeated .apollo.prediction.Vector rows = 1;
  int rows_size() const;
  private:
  int _internal_rows_size() const;
  public:
  void clear_rows();
  ::apollo::prediction::Vector* mutable_rows(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::prediction::Vector >*
      mutable_rows();
  private:
  const ::apollo::prediction::Vector& _internal_rows(int index) const;
  ::apollo::prediction::Vector* _internal_add_rows();
  public:
  const ::apollo::prediction::Vector& rows(int index) const;
  ::apollo::prediction::Vector* add_rows();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::prediction::Vector >&
      rows() const;

  // @@protoc_insertion_point(class_scope:apollo.prediction.Matrix)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::prediction::Vector > rows_;
  friend struct ::TableStruct_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto;
};
// -------------------------------------------------------------------

class Layer :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.prediction.Layer) */ {
 public:
  Layer();
  virtual ~Layer();

  Layer(const Layer& from);
  Layer(Layer&& from) noexcept
    : Layer() {
    *this = ::std::move(from);
  }

  inline Layer& operator=(const Layer& from) {
    CopyFrom(from);
    return *this;
  }
  inline Layer& operator=(Layer&& from) noexcept {
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
  static const Layer& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Layer* internal_default_instance() {
    return reinterpret_cast<const Layer*>(
               &_Layer_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    2;

  friend void swap(Layer& a, Layer& b) {
    a.Swap(&b);
  }
  inline void Swap(Layer* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline Layer* New() const final {
    return CreateMaybeMessage<Layer>(nullptr);
  }

  Layer* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<Layer>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const Layer& from);
  void MergeFrom(const Layer& from);
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
  void InternalSwap(Layer* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.prediction.Layer";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto);
    return ::descriptor_table_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  typedef Layer_ActivationFunc ActivationFunc;
  static constexpr ActivationFunc RELU =
    Layer_ActivationFunc_RELU;
  static constexpr ActivationFunc TANH =
    Layer_ActivationFunc_TANH;
  static constexpr ActivationFunc SIGMOID =
    Layer_ActivationFunc_SIGMOID;
  static constexpr ActivationFunc SOFTMAX =
    Layer_ActivationFunc_SOFTMAX;
  static inline bool ActivationFunc_IsValid(int value) {
    return Layer_ActivationFunc_IsValid(value);
  }
  static constexpr ActivationFunc ActivationFunc_MIN =
    Layer_ActivationFunc_ActivationFunc_MIN;
  static constexpr ActivationFunc ActivationFunc_MAX =
    Layer_ActivationFunc_ActivationFunc_MAX;
  static constexpr int ActivationFunc_ARRAYSIZE =
    Layer_ActivationFunc_ActivationFunc_ARRAYSIZE;
  static inline const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor*
  ActivationFunc_descriptor() {
    return Layer_ActivationFunc_descriptor();
  }
  template<typename T>
  static inline const std::string& ActivationFunc_Name(T enum_t_value) {
    static_assert(::std::is_same<T, ActivationFunc>::value ||
      ::std::is_integral<T>::value,
      "Incorrect type passed to function ActivationFunc_Name.");
    return Layer_ActivationFunc_Name(enum_t_value);
  }
  static inline bool ActivationFunc_Parse(const std::string& name,
      ActivationFunc* value) {
    return Layer_ActivationFunc_Parse(name, value);
  }

  // accessors -------------------------------------------------------

  enum : int {
    kLayerInputWeightFieldNumber = 3,
    kLayerBiasFieldNumber = 4,
    kLayerInputDimFieldNumber = 1,
    kLayerOutputDimFieldNumber = 2,
    kLayerActivationFuncFieldNumber = 5,
  };
  // optional .apollo.prediction.Matrix layer_input_weight = 3;
  bool has_layer_input_weight() const;
  private:
  bool _internal_has_layer_input_weight() const;
  public:
  void clear_layer_input_weight();
  const ::apollo::prediction::Matrix& layer_input_weight() const;
  ::apollo::prediction::Matrix* release_layer_input_weight();
  ::apollo::prediction::Matrix* mutable_layer_input_weight();
  void set_allocated_layer_input_weight(::apollo::prediction::Matrix* layer_input_weight);
  private:
  const ::apollo::prediction::Matrix& _internal_layer_input_weight() const;
  ::apollo::prediction::Matrix* _internal_mutable_layer_input_weight();
  public:

  // optional .apollo.prediction.Vector layer_bias = 4;
  bool has_layer_bias() const;
  private:
  bool _internal_has_layer_bias() const;
  public:
  void clear_layer_bias();
  const ::apollo::prediction::Vector& layer_bias() const;
  ::apollo::prediction::Vector* release_layer_bias();
  ::apollo::prediction::Vector* mutable_layer_bias();
  void set_allocated_layer_bias(::apollo::prediction::Vector* layer_bias);
  private:
  const ::apollo::prediction::Vector& _internal_layer_bias() const;
  ::apollo::prediction::Vector* _internal_mutable_layer_bias();
  public:

  // optional int32 layer_input_dim = 1;
  bool has_layer_input_dim() const;
  private:
  bool _internal_has_layer_input_dim() const;
  public:
  void clear_layer_input_dim();
  ::PROTOBUF_NAMESPACE_ID::int32 layer_input_dim() const;
  void set_layer_input_dim(::PROTOBUF_NAMESPACE_ID::int32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int32 _internal_layer_input_dim() const;
  void _internal_set_layer_input_dim(::PROTOBUF_NAMESPACE_ID::int32 value);
  public:

  // optional int32 layer_output_dim = 2;
  bool has_layer_output_dim() const;
  private:
  bool _internal_has_layer_output_dim() const;
  public:
  void clear_layer_output_dim();
  ::PROTOBUF_NAMESPACE_ID::int32 layer_output_dim() const;
  void set_layer_output_dim(::PROTOBUF_NAMESPACE_ID::int32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int32 _internal_layer_output_dim() const;
  void _internal_set_layer_output_dim(::PROTOBUF_NAMESPACE_ID::int32 value);
  public:

  // optional .apollo.prediction.Layer.ActivationFunc layer_activation_func = 5;
  bool has_layer_activation_func() const;
  private:
  bool _internal_has_layer_activation_func() const;
  public:
  void clear_layer_activation_func();
  ::apollo::prediction::Layer_ActivationFunc layer_activation_func() const;
  void set_layer_activation_func(::apollo::prediction::Layer_ActivationFunc value);
  private:
  ::apollo::prediction::Layer_ActivationFunc _internal_layer_activation_func() const;
  void _internal_set_layer_activation_func(::apollo::prediction::Layer_ActivationFunc value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.prediction.Layer)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::apollo::prediction::Matrix* layer_input_weight_;
  ::apollo::prediction::Vector* layer_bias_;
  ::PROTOBUF_NAMESPACE_ID::int32 layer_input_dim_;
  ::PROTOBUF_NAMESPACE_ID::int32 layer_output_dim_;
  int layer_activation_func_;
  friend struct ::TableStruct_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Vector

// repeated double columns = 1;
inline int Vector::_internal_columns_size() const {
  return columns_.size();
}
inline int Vector::columns_size() const {
  return _internal_columns_size();
}
inline void Vector::clear_columns() {
  columns_.Clear();
}
inline double Vector::_internal_columns(int index) const {
  return columns_.Get(index);
}
inline double Vector::columns(int index) const {
  // @@protoc_insertion_point(field_get:apollo.prediction.Vector.columns)
  return _internal_columns(index);
}
inline void Vector::set_columns(int index, double value) {
  columns_.Set(index, value);
  // @@protoc_insertion_point(field_set:apollo.prediction.Vector.columns)
}
inline void Vector::_internal_add_columns(double value) {
  columns_.Add(value);
}
inline void Vector::add_columns(double value) {
  _internal_add_columns(value);
  // @@protoc_insertion_point(field_add:apollo.prediction.Vector.columns)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
Vector::_internal_columns() const {
  return columns_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
Vector::columns() const {
  // @@protoc_insertion_point(field_list:apollo.prediction.Vector.columns)
  return _internal_columns();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
Vector::_internal_mutable_columns() {
  return &columns_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
Vector::mutable_columns() {
  // @@protoc_insertion_point(field_mutable_list:apollo.prediction.Vector.columns)
  return _internal_mutable_columns();
}

// -------------------------------------------------------------------

// Matrix

// repeated .apollo.prediction.Vector rows = 1;
inline int Matrix::_internal_rows_size() const {
  return rows_.size();
}
inline int Matrix::rows_size() const {
  return _internal_rows_size();
}
inline void Matrix::clear_rows() {
  rows_.Clear();
}
inline ::apollo::prediction::Vector* Matrix::mutable_rows(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.prediction.Matrix.rows)
  return rows_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::prediction::Vector >*
Matrix::mutable_rows() {
  // @@protoc_insertion_point(field_mutable_list:apollo.prediction.Matrix.rows)
  return &rows_;
}
inline const ::apollo::prediction::Vector& Matrix::_internal_rows(int index) const {
  return rows_.Get(index);
}
inline const ::apollo::prediction::Vector& Matrix::rows(int index) const {
  // @@protoc_insertion_point(field_get:apollo.prediction.Matrix.rows)
  return _internal_rows(index);
}
inline ::apollo::prediction::Vector* Matrix::_internal_add_rows() {
  return rows_.Add();
}
inline ::apollo::prediction::Vector* Matrix::add_rows() {
  // @@protoc_insertion_point(field_add:apollo.prediction.Matrix.rows)
  return _internal_add_rows();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::prediction::Vector >&
Matrix::rows() const {
  // @@protoc_insertion_point(field_list:apollo.prediction.Matrix.rows)
  return rows_;
}

// -------------------------------------------------------------------

// Layer

// optional int32 layer_input_dim = 1;
inline bool Layer::_internal_has_layer_input_dim() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool Layer::has_layer_input_dim() const {
  return _internal_has_layer_input_dim();
}
inline void Layer::clear_layer_input_dim() {
  layer_input_dim_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 Layer::_internal_layer_input_dim() const {
  return layer_input_dim_;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 Layer::layer_input_dim() const {
  // @@protoc_insertion_point(field_get:apollo.prediction.Layer.layer_input_dim)
  return _internal_layer_input_dim();
}
inline void Layer::_internal_set_layer_input_dim(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00000004u;
  layer_input_dim_ = value;
}
inline void Layer::set_layer_input_dim(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _internal_set_layer_input_dim(value);
  // @@protoc_insertion_point(field_set:apollo.prediction.Layer.layer_input_dim)
}

// optional int32 layer_output_dim = 2;
inline bool Layer::_internal_has_layer_output_dim() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool Layer::has_layer_output_dim() const {
  return _internal_has_layer_output_dim();
}
inline void Layer::clear_layer_output_dim() {
  layer_output_dim_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 Layer::_internal_layer_output_dim() const {
  return layer_output_dim_;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 Layer::layer_output_dim() const {
  // @@protoc_insertion_point(field_get:apollo.prediction.Layer.layer_output_dim)
  return _internal_layer_output_dim();
}
inline void Layer::_internal_set_layer_output_dim(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00000008u;
  layer_output_dim_ = value;
}
inline void Layer::set_layer_output_dim(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _internal_set_layer_output_dim(value);
  // @@protoc_insertion_point(field_set:apollo.prediction.Layer.layer_output_dim)
}

// optional .apollo.prediction.Matrix layer_input_weight = 3;
inline bool Layer::_internal_has_layer_input_weight() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || layer_input_weight_ != nullptr);
  return value;
}
inline bool Layer::has_layer_input_weight() const {
  return _internal_has_layer_input_weight();
}
inline void Layer::clear_layer_input_weight() {
  if (layer_input_weight_ != nullptr) layer_input_weight_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
inline const ::apollo::prediction::Matrix& Layer::_internal_layer_input_weight() const {
  const ::apollo::prediction::Matrix* p = layer_input_weight_;
  return p != nullptr ? *p : *reinterpret_cast<const ::apollo::prediction::Matrix*>(
      &::apollo::prediction::_Matrix_default_instance_);
}
inline const ::apollo::prediction::Matrix& Layer::layer_input_weight() const {
  // @@protoc_insertion_point(field_get:apollo.prediction.Layer.layer_input_weight)
  return _internal_layer_input_weight();
}
inline ::apollo::prediction::Matrix* Layer::release_layer_input_weight() {
  // @@protoc_insertion_point(field_release:apollo.prediction.Layer.layer_input_weight)
  _has_bits_[0] &= ~0x00000001u;
  ::apollo::prediction::Matrix* temp = layer_input_weight_;
  layer_input_weight_ = nullptr;
  return temp;
}
inline ::apollo::prediction::Matrix* Layer::_internal_mutable_layer_input_weight() {
  _has_bits_[0] |= 0x00000001u;
  if (layer_input_weight_ == nullptr) {
    auto* p = CreateMaybeMessage<::apollo::prediction::Matrix>(GetArenaNoVirtual());
    layer_input_weight_ = p;
  }
  return layer_input_weight_;
}
inline ::apollo::prediction::Matrix* Layer::mutable_layer_input_weight() {
  // @@protoc_insertion_point(field_mutable:apollo.prediction.Layer.layer_input_weight)
  return _internal_mutable_layer_input_weight();
}
inline void Layer::set_allocated_layer_input_weight(::apollo::prediction::Matrix* layer_input_weight) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete layer_input_weight_;
  }
  if (layer_input_weight) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      layer_input_weight = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, layer_input_weight, submessage_arena);
    }
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  layer_input_weight_ = layer_input_weight;
  // @@protoc_insertion_point(field_set_allocated:apollo.prediction.Layer.layer_input_weight)
}

// optional .apollo.prediction.Vector layer_bias = 4;
inline bool Layer::_internal_has_layer_bias() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  PROTOBUF_ASSUME(!value || layer_bias_ != nullptr);
  return value;
}
inline bool Layer::has_layer_bias() const {
  return _internal_has_layer_bias();
}
inline void Layer::clear_layer_bias() {
  if (layer_bias_ != nullptr) layer_bias_->Clear();
  _has_bits_[0] &= ~0x00000002u;
}
inline const ::apollo::prediction::Vector& Layer::_internal_layer_bias() const {
  const ::apollo::prediction::Vector* p = layer_bias_;
  return p != nullptr ? *p : *reinterpret_cast<const ::apollo::prediction::Vector*>(
      &::apollo::prediction::_Vector_default_instance_);
}
inline const ::apollo::prediction::Vector& Layer::layer_bias() const {
  // @@protoc_insertion_point(field_get:apollo.prediction.Layer.layer_bias)
  return _internal_layer_bias();
}
inline ::apollo::prediction::Vector* Layer::release_layer_bias() {
  // @@protoc_insertion_point(field_release:apollo.prediction.Layer.layer_bias)
  _has_bits_[0] &= ~0x00000002u;
  ::apollo::prediction::Vector* temp = layer_bias_;
  layer_bias_ = nullptr;
  return temp;
}
inline ::apollo::prediction::Vector* Layer::_internal_mutable_layer_bias() {
  _has_bits_[0] |= 0x00000002u;
  if (layer_bias_ == nullptr) {
    auto* p = CreateMaybeMessage<::apollo::prediction::Vector>(GetArenaNoVirtual());
    layer_bias_ = p;
  }
  return layer_bias_;
}
inline ::apollo::prediction::Vector* Layer::mutable_layer_bias() {
  // @@protoc_insertion_point(field_mutable:apollo.prediction.Layer.layer_bias)
  return _internal_mutable_layer_bias();
}
inline void Layer::set_allocated_layer_bias(::apollo::prediction::Vector* layer_bias) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete layer_bias_;
  }
  if (layer_bias) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      layer_bias = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, layer_bias, submessage_arena);
    }
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  layer_bias_ = layer_bias;
  // @@protoc_insertion_point(field_set_allocated:apollo.prediction.Layer.layer_bias)
}

// optional .apollo.prediction.Layer.ActivationFunc layer_activation_func = 5;
inline bool Layer::_internal_has_layer_activation_func() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool Layer::has_layer_activation_func() const {
  return _internal_has_layer_activation_func();
}
inline void Layer::clear_layer_activation_func() {
  layer_activation_func_ = 0;
  _has_bits_[0] &= ~0x00000010u;
}
inline ::apollo::prediction::Layer_ActivationFunc Layer::_internal_layer_activation_func() const {
  return static_cast< ::apollo::prediction::Layer_ActivationFunc >(layer_activation_func_);
}
inline ::apollo::prediction::Layer_ActivationFunc Layer::layer_activation_func() const {
  // @@protoc_insertion_point(field_get:apollo.prediction.Layer.layer_activation_func)
  return _internal_layer_activation_func();
}
inline void Layer::_internal_set_layer_activation_func(::apollo::prediction::Layer_ActivationFunc value) {
  assert(::apollo::prediction::Layer_ActivationFunc_IsValid(value));
  _has_bits_[0] |= 0x00000010u;
  layer_activation_func_ = value;
}
inline void Layer::set_layer_activation_func(::apollo::prediction::Layer_ActivationFunc value) {
  _internal_set_layer_activation_func(value);
  // @@protoc_insertion_point(field_set:apollo.prediction.Layer.layer_activation_func)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------

// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace prediction
}  // namespace apollo

PROTOBUF_NAMESPACE_OPEN

template <> struct is_proto_enum< ::apollo::prediction::Layer_ActivationFunc> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::apollo::prediction::Layer_ActivationFunc>() {
  return ::apollo::prediction::Layer_ActivationFunc_descriptor();
}

PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto
