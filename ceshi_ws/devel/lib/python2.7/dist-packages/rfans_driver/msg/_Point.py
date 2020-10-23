# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from rfans_driver/Point.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class Point(genpy.Message):
  _md5sum = "c81b1cf288dd128754d09c9e4bbb83cf"
  _type = "rfans_driver/Point"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """#存储距离雷达最近的两对锥桶及其中心点的坐标

float64 x1
float64 y1
float64 x2
float64 y2
float64 a1
float64 a2
float64 a3
float64 a4
float64 b1
float64 b2
float64 b3
float64 b4






"""
  __slots__ = ['x1','y1','x2','y2','a1','a2','a3','a4','b1','b2','b3','b4']
  _slot_types = ['float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       x1,y1,x2,y2,a1,a2,a3,a4,b1,b2,b3,b4

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Point, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.x1 is None:
        self.x1 = 0.
      if self.y1 is None:
        self.y1 = 0.
      if self.x2 is None:
        self.x2 = 0.
      if self.y2 is None:
        self.y2 = 0.
      if self.a1 is None:
        self.a1 = 0.
      if self.a2 is None:
        self.a2 = 0.
      if self.a3 is None:
        self.a3 = 0.
      if self.a4 is None:
        self.a4 = 0.
      if self.b1 is None:
        self.b1 = 0.
      if self.b2 is None:
        self.b2 = 0.
      if self.b3 is None:
        self.b3 = 0.
      if self.b4 is None:
        self.b4 = 0.
    else:
      self.x1 = 0.
      self.y1 = 0.
      self.x2 = 0.
      self.y2 = 0.
      self.a1 = 0.
      self.a2 = 0.
      self.a3 = 0.
      self.a4 = 0.
      self.b1 = 0.
      self.b2 = 0.
      self.b3 = 0.
      self.b4 = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_12d().pack(_x.x1, _x.y1, _x.x2, _x.y2, _x.a1, _x.a2, _x.a3, _x.a4, _x.b1, _x.b2, _x.b3, _x.b4))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 96
      (_x.x1, _x.y1, _x.x2, _x.y2, _x.a1, _x.a2, _x.a3, _x.a4, _x.b1, _x.b2, _x.b3, _x.b4,) = _get_struct_12d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_12d().pack(_x.x1, _x.y1, _x.x2, _x.y2, _x.a1, _x.a2, _x.a3, _x.a4, _x.b1, _x.b2, _x.b3, _x.b4))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 96
      (_x.x1, _x.y1, _x.x2, _x.y2, _x.a1, _x.a2, _x.a3, _x.a4, _x.b1, _x.b2, _x.b3, _x.b4,) = _get_struct_12d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_12d = None
def _get_struct_12d():
    global _struct_12d
    if _struct_12d is None:
        _struct_12d = struct.Struct("<12d")
    return _struct_12d
