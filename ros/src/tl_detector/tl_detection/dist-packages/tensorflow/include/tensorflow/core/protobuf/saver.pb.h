// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: tensorflow/core/protobuf/saver.proto

#ifndef PROTOBUF_tensorflow_2fcore_2fprotobuf_2fsaver_2eproto__INCLUDED
#define PROTOBUF_tensorflow_2fcore_2fprotobuf_2fsaver_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3002000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3002000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
namespace tensorflow {
class SaverDef;
class SaverDefDefaultTypeInternal;
extern SaverDefDefaultTypeInternal _SaverDef_default_instance_;
}  // namespace tensorflow

namespace tensorflow {

namespace protobuf_tensorflow_2fcore_2fprotobuf_2fsaver_2eproto {
// Internal implementation detail -- do not call these.
struct TableStruct {
  static const ::google::protobuf::uint32 offsets[];
  static void InitDefaultsImpl();
  static void Shutdown();
};
void AddDescriptors();
void InitDefaults();
}  // namespace protobuf_tensorflow_2fcore_2fprotobuf_2fsaver_2eproto

enum SaverDef_CheckpointFormatVersion {
  SaverDef_CheckpointFormatVersion_LEGACY = 0,
  SaverDef_CheckpointFormatVersion_V1 = 1,
  SaverDef_CheckpointFormatVersion_V2 = 2,
  SaverDef_CheckpointFormatVersion_SaverDef_CheckpointFormatVersion_INT_MIN_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32min,
  SaverDef_CheckpointFormatVersion_SaverDef_CheckpointFormatVersion_INT_MAX_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32max
};
bool SaverDef_CheckpointFormatVersion_IsValid(int value);
const SaverDef_CheckpointFormatVersion SaverDef_CheckpointFormatVersion_CheckpointFormatVersion_MIN = SaverDef_CheckpointFormatVersion_LEGACY;
const SaverDef_CheckpointFormatVersion SaverDef_CheckpointFormatVersion_CheckpointFormatVersion_MAX = SaverDef_CheckpointFormatVersion_V2;
const int SaverDef_CheckpointFormatVersion_CheckpointFormatVersion_ARRAYSIZE = SaverDef_CheckpointFormatVersion_CheckpointFormatVersion_MAX + 1;

const ::google::protobuf::EnumDescriptor* SaverDef_CheckpointFormatVersion_descriptor();
inline const ::std::string& SaverDef_CheckpointFormatVersion_Name(SaverDef_CheckpointFormatVersion value) {
  return ::google::protobuf::internal::NameOfEnum(
    SaverDef_CheckpointFormatVersion_descriptor(), value);
}
inline bool SaverDef_CheckpointFormatVersion_Parse(
    const ::std::string& name, SaverDef_CheckpointFormatVersion* value) {
  return ::google::protobuf::internal::ParseNamedEnum<SaverDef_CheckpointFormatVersion>(
    SaverDef_CheckpointFormatVersion_descriptor(), name, value);
}
// ===================================================================

class SaverDef : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:tensorflow.SaverDef) */ {
 public:
  SaverDef();
  virtual ~SaverDef();

  SaverDef(const SaverDef& from);

  inline SaverDef& operator=(const SaverDef& from) {
    CopyFrom(from);
    return *this;
  }

  inline ::google::protobuf::Arena* GetArena() const PROTOBUF_FINAL {
    return GetArenaNoVirtual();
  }
  inline void* GetMaybeArenaPointer() const PROTOBUF_FINAL {
    return MaybeArenaPtr();
  }
  static const ::google::protobuf::Descriptor* descriptor();
  static const SaverDef& default_instance();

  static inline const SaverDef* internal_default_instance() {
    return reinterpret_cast<const SaverDef*>(
               &_SaverDef_default_instance_);
  }

  void UnsafeArenaSwap(SaverDef* other);
  void Swap(SaverDef* other);

  // implements Message ----------------------------------------------

  inline SaverDef* New() const PROTOBUF_FINAL { return New(NULL); }

  SaverDef* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const SaverDef& from);
  void MergeFrom(const SaverDef& from);
  void Clear() PROTOBUF_FINAL;
  bool IsInitialized() const PROTOBUF_FINAL;

  size_t ByteSizeLong() const PROTOBUF_FINAL;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) PROTOBUF_FINAL;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output)
      const PROTOBUF_FINAL {
    return InternalSerializeWithCachedSizesToArray(
        ::google::protobuf::io::CodedOutputStream::IsDefaultSerializationDeterministic(), output);
  }
  int GetCachedSize() const PROTOBUF_FINAL { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const PROTOBUF_FINAL;
  void InternalSwap(SaverDef* other);
  protected:
  explicit SaverDef(::google::protobuf::Arena* arena);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::google::protobuf::Arena* arena);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return _internal_metadata_.arena();
  }
  inline void* MaybeArenaPtr() const {
    return _internal_metadata_.raw_arena_ptr();
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const PROTOBUF_FINAL;

  // nested types ----------------------------------------------------

  typedef SaverDef_CheckpointFormatVersion CheckpointFormatVersion;
  static const CheckpointFormatVersion LEGACY =
    SaverDef_CheckpointFormatVersion_LEGACY;
  static const CheckpointFormatVersion V1 =
    SaverDef_CheckpointFormatVersion_V1;
  static const CheckpointFormatVersion V2 =
    SaverDef_CheckpointFormatVersion_V2;
  static inline bool CheckpointFormatVersion_IsValid(int value) {
    return SaverDef_CheckpointFormatVersion_IsValid(value);
  }
  static const CheckpointFormatVersion CheckpointFormatVersion_MIN =
    SaverDef_CheckpointFormatVersion_CheckpointFormatVersion_MIN;
  static const CheckpointFormatVersion CheckpointFormatVersion_MAX =
    SaverDef_CheckpointFormatVersion_CheckpointFormatVersion_MAX;
  static const int CheckpointFormatVersion_ARRAYSIZE =
    SaverDef_CheckpointFormatVersion_CheckpointFormatVersion_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  CheckpointFormatVersion_descriptor() {
    return SaverDef_CheckpointFormatVersion_descriptor();
  }
  static inline const ::std::string& CheckpointFormatVersion_Name(CheckpointFormatVersion value) {
    return SaverDef_CheckpointFormatVersion_Name(value);
  }
  static inline bool CheckpointFormatVersion_Parse(const ::std::string& name,
      CheckpointFormatVersion* value) {
    return SaverDef_CheckpointFormatVersion_Parse(name, value);
  }

  // accessors -------------------------------------------------------

  // string filename_tensor_name = 1;
  void clear_filename_tensor_name();
  static const int kFilenameTensorNameFieldNumber = 1;
  const ::std::string& filename_tensor_name() const;
  void set_filename_tensor_name(const ::std::string& value);
  void set_filename_tensor_name(const char* value);
  void set_filename_tensor_name(const char* value, size_t size);
  ::std::string* mutable_filename_tensor_name();
  ::std::string* release_filename_tensor_name();
  void set_allocated_filename_tensor_name(::std::string* filename_tensor_name);
  ::std::string* unsafe_arena_release_filename_tensor_name();
  void unsafe_arena_set_allocated_filename_tensor_name(
      ::std::string* filename_tensor_name);

  // string save_tensor_name = 2;
  void clear_save_tensor_name();
  static const int kSaveTensorNameFieldNumber = 2;
  const ::std::string& save_tensor_name() const;
  void set_save_tensor_name(const ::std::string& value);
  void set_save_tensor_name(const char* value);
  void set_save_tensor_name(const char* value, size_t size);
  ::std::string* mutable_save_tensor_name();
  ::std::string* release_save_tensor_name();
  void set_allocated_save_tensor_name(::std::string* save_tensor_name);
  ::std::string* unsafe_arena_release_save_tensor_name();
  void unsafe_arena_set_allocated_save_tensor_name(
      ::std::string* save_tensor_name);

  // string restore_op_name = 3;
  void clear_restore_op_name();
  static const int kRestoreOpNameFieldNumber = 3;
  const ::std::string& restore_op_name() const;
  void set_restore_op_name(const ::std::string& value);
  void set_restore_op_name(const char* value);
  void set_restore_op_name(const char* value, size_t size);
  ::std::string* mutable_restore_op_name();
  ::std::string* release_restore_op_name();
  void set_allocated_restore_op_name(::std::string* restore_op_name);
  ::std::string* unsafe_arena_release_restore_op_name();
  void unsafe_arena_set_allocated_restore_op_name(
      ::std::string* restore_op_name);

  // int32 max_to_keep = 4;
  void clear_max_to_keep();
  static const int kMaxToKeepFieldNumber = 4;
  ::google::protobuf::int32 max_to_keep() const;
  void set_max_to_keep(::google::protobuf::int32 value);

  // bool sharded = 5;
  void clear_sharded();
  static const int kShardedFieldNumber = 5;
  bool sharded() const;
  void set_sharded(bool value);

  // float keep_checkpoint_every_n_hours = 6;
  void clear_keep_checkpoint_every_n_hours();
  static const int kKeepCheckpointEveryNHoursFieldNumber = 6;
  float keep_checkpoint_every_n_hours() const;
  void set_keep_checkpoint_every_n_hours(float value);

  // .tensorflow.SaverDef.CheckpointFormatVersion version = 7;
  void clear_version();
  static const int kVersionFieldNumber = 7;
  ::tensorflow::SaverDef_CheckpointFormatVersion version() const;
  void set_version(::tensorflow::SaverDef_CheckpointFormatVersion value);

  // @@protoc_insertion_point(class_scope:tensorflow.SaverDef)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  friend class ::google::protobuf::Arena;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::google::protobuf::internal::ArenaStringPtr filename_tensor_name_;
  ::google::protobuf::internal::ArenaStringPtr save_tensor_name_;
  ::google::protobuf::internal::ArenaStringPtr restore_op_name_;
  ::google::protobuf::int32 max_to_keep_;
  bool sharded_;
  float keep_checkpoint_every_n_hours_;
  int version_;
  mutable int _cached_size_;
  friend struct  protobuf_tensorflow_2fcore_2fprotobuf_2fsaver_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// SaverDef

// string filename_tensor_name = 1;
inline void SaverDef::clear_filename_tensor_name() {
  filename_tensor_name_.ClearToEmpty(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), GetArenaNoVirtual());
}
inline const ::std::string& SaverDef::filename_tensor_name() const {
  // @@protoc_insertion_point(field_get:tensorflow.SaverDef.filename_tensor_name)
  return filename_tensor_name_.Get();
}
inline void SaverDef::set_filename_tensor_name(const ::std::string& value) {
  
  filename_tensor_name_.Set(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value, GetArenaNoVirtual());
  // @@protoc_insertion_point(field_set:tensorflow.SaverDef.filename_tensor_name)
}
inline void SaverDef::set_filename_tensor_name(const char* value) {
  
  filename_tensor_name_.Set(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value),
              GetArenaNoVirtual());
  // @@protoc_insertion_point(field_set_char:tensorflow.SaverDef.filename_tensor_name)
}
inline void SaverDef::set_filename_tensor_name(const char* value,
    size_t size) {
  
  filename_tensor_name_.Set(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(
      reinterpret_cast<const char*>(value), size), GetArenaNoVirtual());
  // @@protoc_insertion_point(field_set_pointer:tensorflow.SaverDef.filename_tensor_name)
}
inline ::std::string* SaverDef::mutable_filename_tensor_name() {
  
  // @@protoc_insertion_point(field_mutable:tensorflow.SaverDef.filename_tensor_name)
  return filename_tensor_name_.Mutable(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), GetArenaNoVirtual());
}
inline ::std::string* SaverDef::release_filename_tensor_name() {
  // @@protoc_insertion_point(field_release:tensorflow.SaverDef.filename_tensor_name)
  
  return filename_tensor_name_.Release(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), GetArenaNoVirtual());
}
inline ::std::string* SaverDef::unsafe_arena_release_filename_tensor_name() {
  // @@protoc_insertion_point(field_unsafe_arena_release:tensorflow.SaverDef.filename_tensor_name)
  GOOGLE_DCHECK(GetArenaNoVirtual() != NULL);
  
  return filename_tensor_name_.UnsafeArenaRelease(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      GetArenaNoVirtual());
}
inline void SaverDef::set_allocated_filename_tensor_name(::std::string* filename_tensor_name) {
  if (filename_tensor_name != NULL) {
    
  } else {
    
  }
  filename_tensor_name_.SetAllocated(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), filename_tensor_name,
      GetArenaNoVirtual());
  // @@protoc_insertion_point(field_set_allocated:tensorflow.SaverDef.filename_tensor_name)
}
inline void SaverDef::unsafe_arena_set_allocated_filename_tensor_name(
    ::std::string* filename_tensor_name) {
  GOOGLE_DCHECK(GetArenaNoVirtual() != NULL);
  if (filename_tensor_name != NULL) {
    
  } else {
    
  }
  filename_tensor_name_.UnsafeArenaSetAllocated(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      filename_tensor_name, GetArenaNoVirtual());
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:tensorflow.SaverDef.filename_tensor_name)
}

// string save_tensor_name = 2;
inline void SaverDef::clear_save_tensor_name() {
  save_tensor_name_.ClearToEmpty(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), GetArenaNoVirtual());
}
inline const ::std::string& SaverDef::save_tensor_name() const {
  // @@protoc_insertion_point(field_get:tensorflow.SaverDef.save_tensor_name)
  return save_tensor_name_.Get();
}
inline void SaverDef::set_save_tensor_name(const ::std::string& value) {
  
  save_tensor_name_.Set(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value, GetArenaNoVirtual());
  // @@protoc_insertion_point(field_set:tensorflow.SaverDef.save_tensor_name)
}
inline void SaverDef::set_save_tensor_name(const char* value) {
  
  save_tensor_name_.Set(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value),
              GetArenaNoVirtual());
  // @@protoc_insertion_point(field_set_char:tensorflow.SaverDef.save_tensor_name)
}
inline void SaverDef::set_save_tensor_name(const char* value,
    size_t size) {
  
  save_tensor_name_.Set(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(
      reinterpret_cast<const char*>(value), size), GetArenaNoVirtual());
  // @@protoc_insertion_point(field_set_pointer:tensorflow.SaverDef.save_tensor_name)
}
inline ::std::string* SaverDef::mutable_save_tensor_name() {
  
  // @@protoc_insertion_point(field_mutable:tensorflow.SaverDef.save_tensor_name)
  return save_tensor_name_.Mutable(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), GetArenaNoVirtual());
}
inline ::std::string* SaverDef::release_save_tensor_name() {
  // @@protoc_insertion_point(field_release:tensorflow.SaverDef.save_tensor_name)
  
  return save_tensor_name_.Release(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), GetArenaNoVirtual());
}
inline ::std::string* SaverDef::unsafe_arena_release_save_tensor_name() {
  // @@protoc_insertion_point(field_unsafe_arena_release:tensorflow.SaverDef.save_tensor_name)
  GOOGLE_DCHECK(GetArenaNoVirtual() != NULL);
  
  return save_tensor_name_.UnsafeArenaRelease(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      GetArenaNoVirtual());
}
inline void SaverDef::set_allocated_save_tensor_name(::std::string* save_tensor_name) {
  if (save_tensor_name != NULL) {
    
  } else {
    
  }
  save_tensor_name_.SetAllocated(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), save_tensor_name,
      GetArenaNoVirtual());
  // @@protoc_insertion_point(field_set_allocated:tensorflow.SaverDef.save_tensor_name)
}
inline void SaverDef::unsafe_arena_set_allocated_save_tensor_name(
    ::std::string* save_tensor_name) {
  GOOGLE_DCHECK(GetArenaNoVirtual() != NULL);
  if (save_tensor_name != NULL) {
    
  } else {
    
  }
  save_tensor_name_.UnsafeArenaSetAllocated(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      save_tensor_name, GetArenaNoVirtual());
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:tensorflow.SaverDef.save_tensor_name)
}

// string restore_op_name = 3;
inline void SaverDef::clear_restore_op_name() {
  restore_op_name_.ClearToEmpty(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), GetArenaNoVirtual());
}
inline const ::std::string& SaverDef::restore_op_name() const {
  // @@protoc_insertion_point(field_get:tensorflow.SaverDef.restore_op_name)
  return restore_op_name_.Get();
}
inline void SaverDef::set_restore_op_name(const ::std::string& value) {
  
  restore_op_name_.Set(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value, GetArenaNoVirtual());
  // @@protoc_insertion_point(field_set:tensorflow.SaverDef.restore_op_name)
}
inline void SaverDef::set_restore_op_name(const char* value) {
  
  restore_op_name_.Set(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value),
              GetArenaNoVirtual());
  // @@protoc_insertion_point(field_set_char:tensorflow.SaverDef.restore_op_name)
}
inline void SaverDef::set_restore_op_name(const char* value,
    size_t size) {
  
  restore_op_name_.Set(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(
      reinterpret_cast<const char*>(value), size), GetArenaNoVirtual());
  // @@protoc_insertion_point(field_set_pointer:tensorflow.SaverDef.restore_op_name)
}
inline ::std::string* SaverDef::mutable_restore_op_name() {
  
  // @@protoc_insertion_point(field_mutable:tensorflow.SaverDef.restore_op_name)
  return restore_op_name_.Mutable(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), GetArenaNoVirtual());
}
inline ::std::string* SaverDef::release_restore_op_name() {
  // @@protoc_insertion_point(field_release:tensorflow.SaverDef.restore_op_name)
  
  return restore_op_name_.Release(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), GetArenaNoVirtual());
}
inline ::std::string* SaverDef::unsafe_arena_release_restore_op_name() {
  // @@protoc_insertion_point(field_unsafe_arena_release:tensorflow.SaverDef.restore_op_name)
  GOOGLE_DCHECK(GetArenaNoVirtual() != NULL);
  
  return restore_op_name_.UnsafeArenaRelease(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      GetArenaNoVirtual());
}
inline void SaverDef::set_allocated_restore_op_name(::std::string* restore_op_name) {
  if (restore_op_name != NULL) {
    
  } else {
    
  }
  restore_op_name_.SetAllocated(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), restore_op_name,
      GetArenaNoVirtual());
  // @@protoc_insertion_point(field_set_allocated:tensorflow.SaverDef.restore_op_name)
}
inline void SaverDef::unsafe_arena_set_allocated_restore_op_name(
    ::std::string* restore_op_name) {
  GOOGLE_DCHECK(GetArenaNoVirtual() != NULL);
  if (restore_op_name != NULL) {
    
  } else {
    
  }
  restore_op_name_.UnsafeArenaSetAllocated(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      restore_op_name, GetArenaNoVirtual());
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:tensorflow.SaverDef.restore_op_name)
}

// int32 max_to_keep = 4;
inline void SaverDef::clear_max_to_keep() {
  max_to_keep_ = 0;
}
inline ::google::protobuf::int32 SaverDef::max_to_keep() const {
  // @@protoc_insertion_point(field_get:tensorflow.SaverDef.max_to_keep)
  return max_to_keep_;
}
inline void SaverDef::set_max_to_keep(::google::protobuf::int32 value) {
  
  max_to_keep_ = value;
  // @@protoc_insertion_point(field_set:tensorflow.SaverDef.max_to_keep)
}

// bool sharded = 5;
inline void SaverDef::clear_sharded() {
  sharded_ = false;
}
inline bool SaverDef::sharded() const {
  // @@protoc_insertion_point(field_get:tensorflow.SaverDef.sharded)
  return sharded_;
}
inline void SaverDef::set_sharded(bool value) {
  
  sharded_ = value;
  // @@protoc_insertion_point(field_set:tensorflow.SaverDef.sharded)
}

// float keep_checkpoint_every_n_hours = 6;
inline void SaverDef::clear_keep_checkpoint_every_n_hours() {
  keep_checkpoint_every_n_hours_ = 0;
}
inline float SaverDef::keep_checkpoint_every_n_hours() const {
  // @@protoc_insertion_point(field_get:tensorflow.SaverDef.keep_checkpoint_every_n_hours)
  return keep_checkpoint_every_n_hours_;
}
inline void SaverDef::set_keep_checkpoint_every_n_hours(float value) {
  
  keep_checkpoint_every_n_hours_ = value;
  // @@protoc_insertion_point(field_set:tensorflow.SaverDef.keep_checkpoint_every_n_hours)
}

// .tensorflow.SaverDef.CheckpointFormatVersion version = 7;
inline void SaverDef::clear_version() {
  version_ = 0;
}
inline ::tensorflow::SaverDef_CheckpointFormatVersion SaverDef::version() const {
  // @@protoc_insertion_point(field_get:tensorflow.SaverDef.version)
  return static_cast< ::tensorflow::SaverDef_CheckpointFormatVersion >(version_);
}
inline void SaverDef::set_version(::tensorflow::SaverDef_CheckpointFormatVersion value) {
  
  version_ = value;
  // @@protoc_insertion_point(field_set:tensorflow.SaverDef.version)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)


}  // namespace tensorflow

#ifndef SWIG
namespace google {
namespace protobuf {

template <> struct is_proto_enum< ::tensorflow::SaverDef_CheckpointFormatVersion> : ::google::protobuf::internal::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::tensorflow::SaverDef_CheckpointFormatVersion>() {
  return ::tensorflow::SaverDef_CheckpointFormatVersion_descriptor();
}

}  // namespace protobuf
}  // namespace google
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_tensorflow_2fcore_2fprotobuf_2fsaver_2eproto__INCLUDED
