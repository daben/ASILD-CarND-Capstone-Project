# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: tensorflow/python/framework/cpp_shape_inference.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from tensorflow.core.framework import types_pb2 as tensorflow_dot_core_dot_framework_dot_types__pb2
from tensorflow.core.framework import tensor_shape_pb2 as tensorflow_dot_core_dot_framework_dot_tensor__shape__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='tensorflow/python/framework/cpp_shape_inference.proto',
  package='tensorflow',
  syntax='proto3',
  serialized_pb=_b('\n5tensorflow/python/framework/cpp_shape_inference.proto\x12\ntensorflow\x1a%tensorflow/core/framework/types.proto\x1a,tensorflow/core/framework/tensor_shape.proto\"\xa6\x01\n\x17\x43ppShapeInferenceResult\x12+\n\x05shape\x18\x01 \x01(\x0b\x32\x1c.tensorflow.TensorShapeProto\x12\x32\n\x0chandle_shape\x18\x02 \x01(\x0b\x32\x1c.tensorflow.TensorShapeProto\x12*\n\x0chandle_dtype\x18\x03 \x01(\x0e\x32\x14.tensorflow.DataType\"e\n\x1d\x43ppShapeInferenceInputsNeeded\x12\x1c\n\x14input_tensors_needed\x18\x01 \x03(\x05\x12&\n\x1einput_tensors_as_shapes_needed\x18\x02 \x03(\x05\x42\x03\xf8\x01\x01\x62\x06proto3')
  ,
  dependencies=[tensorflow_dot_core_dot_framework_dot_types__pb2.DESCRIPTOR,tensorflow_dot_core_dot_framework_dot_tensor__shape__pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_CPPSHAPEINFERENCERESULT = _descriptor.Descriptor(
  name='CppShapeInferenceResult',
  full_name='tensorflow.CppShapeInferenceResult',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='shape', full_name='tensorflow.CppShapeInferenceResult.shape', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='handle_shape', full_name='tensorflow.CppShapeInferenceResult.handle_shape', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='handle_dtype', full_name='tensorflow.CppShapeInferenceResult.handle_dtype', index=2,
      number=3, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=155,
  serialized_end=321,
)


_CPPSHAPEINFERENCEINPUTSNEEDED = _descriptor.Descriptor(
  name='CppShapeInferenceInputsNeeded',
  full_name='tensorflow.CppShapeInferenceInputsNeeded',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='input_tensors_needed', full_name='tensorflow.CppShapeInferenceInputsNeeded.input_tensors_needed', index=0,
      number=1, type=5, cpp_type=1, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='input_tensors_as_shapes_needed', full_name='tensorflow.CppShapeInferenceInputsNeeded.input_tensors_as_shapes_needed', index=1,
      number=2, type=5, cpp_type=1, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=323,
  serialized_end=424,
)

_CPPSHAPEINFERENCERESULT.fields_by_name['shape'].message_type = tensorflow_dot_core_dot_framework_dot_tensor__shape__pb2._TENSORSHAPEPROTO
_CPPSHAPEINFERENCERESULT.fields_by_name['handle_shape'].message_type = tensorflow_dot_core_dot_framework_dot_tensor__shape__pb2._TENSORSHAPEPROTO
_CPPSHAPEINFERENCERESULT.fields_by_name['handle_dtype'].enum_type = tensorflow_dot_core_dot_framework_dot_types__pb2._DATATYPE
DESCRIPTOR.message_types_by_name['CppShapeInferenceResult'] = _CPPSHAPEINFERENCERESULT
DESCRIPTOR.message_types_by_name['CppShapeInferenceInputsNeeded'] = _CPPSHAPEINFERENCEINPUTSNEEDED

CppShapeInferenceResult = _reflection.GeneratedProtocolMessageType('CppShapeInferenceResult', (_message.Message,), dict(
  DESCRIPTOR = _CPPSHAPEINFERENCERESULT,
  __module__ = 'tensorflow.python.framework.cpp_shape_inference_pb2'
  # @@protoc_insertion_point(class_scope:tensorflow.CppShapeInferenceResult)
  ))
_sym_db.RegisterMessage(CppShapeInferenceResult)

CppShapeInferenceInputsNeeded = _reflection.GeneratedProtocolMessageType('CppShapeInferenceInputsNeeded', (_message.Message,), dict(
  DESCRIPTOR = _CPPSHAPEINFERENCEINPUTSNEEDED,
  __module__ = 'tensorflow.python.framework.cpp_shape_inference_pb2'
  # @@protoc_insertion_point(class_scope:tensorflow.CppShapeInferenceInputsNeeded)
  ))
_sym_db.RegisterMessage(CppShapeInferenceInputsNeeded)


DESCRIPTOR.has_options = True
DESCRIPTOR._options = _descriptor._ParseOptions(descriptor_pb2.FileOptions(), _b('\370\001\001'))
# @@protoc_insertion_point(module_scope)
