"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class pose_xyt_t(object):
    __slots__ = ["utime", "x", "y", "theta"]

    __typenames__ = ["int64_t", "float", "float", "float"]

    __dimensions__ = [None, None, None, None]

    def __init__(self):
        self.utime = 0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def encode(self):
        buf = BytesIO()
        buf.write(pose_xyt_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qfff", self.utime, self.x, self.y, self.theta))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != pose_xyt_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return pose_xyt_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = pose_xyt_t()
        self.utime, self.x, self.y, self.theta = struct.unpack(">qfff", buf.read(20))
        return self
    _decode_one = staticmethod(_decode_one)

    def _get_hash_recursive(parents):
        if pose_xyt_t in parents: return 0
        tmphash = (0xf98bd7892313b56) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if pose_xyt_t._packed_fingerprint is None:
            pose_xyt_t._packed_fingerprint = struct.pack(">Q", pose_xyt_t._get_hash_recursive([]))
        return pose_xyt_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", pose_xyt_t._get_packed_fingerprint())[0]

