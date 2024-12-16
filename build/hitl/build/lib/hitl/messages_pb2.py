# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: messages.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='messages.proto',
  package='HWIL',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\x0emessages.proto\x12\x04HWIL\"\xa5\x04\n\x03msg\x12\x1e\n\x05gains\x18\x01 \x01(\x0b\x32\x0f.HWIL.msg.Gains\x12\'\n\x08\x63ontrols\x18\x02 \x01(\x0b\x32\x15.HWIL.msg.ControlData\x12%\n\x07sensors\x18\x03 \x01(\x0b\x32\x14.HWIL.msg.SensorData\x12\"\n\x07\x63ommand\x18\x04 \x01(\x0b\x32\x11.HWIL.msg.Command\x1ak\n\x05Gains\x12\x0b\n\x03\x61lt\x18\x01 \x02(\x01\x12\n\n\x02vz\x18\x02 \x02(\x01\x12\x0c\n\x04roll\x18\x03 \x02(\x01\x12\t\n\x01p\x18\x04 \x02(\x01\x12\r\n\x05pitch\x18\x05 \x02(\x01\x12\t\n\x01q\x18\x06 \x02(\x01\x12\x0b\n\x03yaw\x18\x07 \x02(\x01\x12\t\n\x01r\x18\x08 \x02(\x01\x1a\x8d\x01\n\nSensorData\x12\x0c\n\x04roll\x18\x01 \x02(\x01\x12\r\n\x05pitch\x18\x02 \x02(\x01\x12\x0b\n\x03yaw\x18\x03 \x02(\x01\x12\x15\n\rangular_vel_x\x18\x04 \x02(\x01\x12\x15\n\rangular_vel_y\x18\x05 \x02(\x01\x12\x15\n\rangular_vel_z\x18\x06 \x02(\x01\x12\x10\n\x08\x61ltitude\x18\x07 \x02(\x01\x1aX\n\x0b\x43ontrolData\x12\x10\n\x08torque_x\x18\x01 \x02(\x01\x12\x10\n\x08torque_y\x18\x02 \x02(\x01\x12\x10\n\x08torque_z\x18\x03 \x02(\x01\x12\x13\n\x0btotal_force\x18\x04 \x02(\x01\x1a\x33\n\x07\x43ommand\x12\x0c\n\x04roll\x18\x01 \x02(\x01\x12\r\n\x05pitch\x18\x02 \x02(\x01\x12\x0b\n\x03yaw\x18\x03 \x02(\x01'
)




_MSG_GAINS = _descriptor.Descriptor(
  name='Gains',
  full_name='HWIL.msg.Gains',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='alt', full_name='HWIL.msg.Gains.alt', index=0,
      number=1, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='vz', full_name='HWIL.msg.Gains.vz', index=1,
      number=2, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='roll', full_name='HWIL.msg.Gains.roll', index=2,
      number=3, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='p', full_name='HWIL.msg.Gains.p', index=3,
      number=4, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='pitch', full_name='HWIL.msg.Gains.pitch', index=4,
      number=5, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='q', full_name='HWIL.msg.Gains.q', index=5,
      number=6, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='yaw', full_name='HWIL.msg.Gains.yaw', index=6,
      number=7, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='r', full_name='HWIL.msg.Gains.r', index=7,
      number=8, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=180,
  serialized_end=287,
)

_MSG_SENSORDATA = _descriptor.Descriptor(
  name='SensorData',
  full_name='HWIL.msg.SensorData',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='roll', full_name='HWIL.msg.SensorData.roll', index=0,
      number=1, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='pitch', full_name='HWIL.msg.SensorData.pitch', index=1,
      number=2, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='yaw', full_name='HWIL.msg.SensorData.yaw', index=2,
      number=3, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='angular_vel_x', full_name='HWIL.msg.SensorData.angular_vel_x', index=3,
      number=4, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='angular_vel_y', full_name='HWIL.msg.SensorData.angular_vel_y', index=4,
      number=5, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='angular_vel_z', full_name='HWIL.msg.SensorData.angular_vel_z', index=5,
      number=6, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='altitude', full_name='HWIL.msg.SensorData.altitude', index=6,
      number=7, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=290,
  serialized_end=431,
)

_MSG_CONTROLDATA = _descriptor.Descriptor(
  name='ControlData',
  full_name='HWIL.msg.ControlData',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='torque_x', full_name='HWIL.msg.ControlData.torque_x', index=0,
      number=1, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='torque_y', full_name='HWIL.msg.ControlData.torque_y', index=1,
      number=2, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='torque_z', full_name='HWIL.msg.ControlData.torque_z', index=2,
      number=3, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='total_force', full_name='HWIL.msg.ControlData.total_force', index=3,
      number=4, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=433,
  serialized_end=521,
)

_MSG_COMMAND = _descriptor.Descriptor(
  name='Command',
  full_name='HWIL.msg.Command',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='roll', full_name='HWIL.msg.Command.roll', index=0,
      number=1, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='pitch', full_name='HWIL.msg.Command.pitch', index=1,
      number=2, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='yaw', full_name='HWIL.msg.Command.yaw', index=2,
      number=3, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=523,
  serialized_end=574,
)

_MSG = _descriptor.Descriptor(
  name='msg',
  full_name='HWIL.msg',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='gains', full_name='HWIL.msg.gains', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='controls', full_name='HWIL.msg.controls', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='sensors', full_name='HWIL.msg.sensors', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='command', full_name='HWIL.msg.command', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[_MSG_GAINS, _MSG_SENSORDATA, _MSG_CONTROLDATA, _MSG_COMMAND, ],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=25,
  serialized_end=574,
)

_MSG_GAINS.containing_type = _MSG
_MSG_SENSORDATA.containing_type = _MSG
_MSG_CONTROLDATA.containing_type = _MSG
_MSG_COMMAND.containing_type = _MSG
_MSG.fields_by_name['gains'].message_type = _MSG_GAINS
_MSG.fields_by_name['controls'].message_type = _MSG_CONTROLDATA
_MSG.fields_by_name['sensors'].message_type = _MSG_SENSORDATA
_MSG.fields_by_name['command'].message_type = _MSG_COMMAND
DESCRIPTOR.message_types_by_name['msg'] = _MSG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

msg = _reflection.GeneratedProtocolMessageType('msg', (_message.Message,), {

  'Gains' : _reflection.GeneratedProtocolMessageType('Gains', (_message.Message,), {
    'DESCRIPTOR' : _MSG_GAINS,
    '__module__' : 'messages_pb2'
    # @@protoc_insertion_point(class_scope:HWIL.msg.Gains)
    })
  ,

  'SensorData' : _reflection.GeneratedProtocolMessageType('SensorData', (_message.Message,), {
    'DESCRIPTOR' : _MSG_SENSORDATA,
    '__module__' : 'messages_pb2'
    # @@protoc_insertion_point(class_scope:HWIL.msg.SensorData)
    })
  ,

  'ControlData' : _reflection.GeneratedProtocolMessageType('ControlData', (_message.Message,), {
    'DESCRIPTOR' : _MSG_CONTROLDATA,
    '__module__' : 'messages_pb2'
    # @@protoc_insertion_point(class_scope:HWIL.msg.ControlData)
    })
  ,

  'Command' : _reflection.GeneratedProtocolMessageType('Command', (_message.Message,), {
    'DESCRIPTOR' : _MSG_COMMAND,
    '__module__' : 'messages_pb2'
    # @@protoc_insertion_point(class_scope:HWIL.msg.Command)
    })
  ,
  'DESCRIPTOR' : _MSG,
  '__module__' : 'messages_pb2'
  # @@protoc_insertion_point(class_scope:HWIL.msg)
  })
_sym_db.RegisterMessage(msg)
_sym_db.RegisterMessage(msg.Gains)
_sym_db.RegisterMessage(msg.SensorData)
_sym_db.RegisterMessage(msg.ControlData)
_sym_db.RegisterMessage(msg.Command)


# @@protoc_insertion_point(module_scope)
