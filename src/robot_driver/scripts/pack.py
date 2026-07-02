import struct
from enum import Enum
from typing import List
import traceback
import os

class PackType(Enum):
    PackTypeCMD = "CMD"
    PackTypeMessage = "Message"


class PackContent:
    Magic = b"das\r\n"


class Opcode(Enum):
    ReadSingle = 0x01
    ReadBatch = 0x02
    WriteDrive = 0x03
    Echo = 0x04
    CalibEncoder = 0x05
    DisableDrive = 0x06


class RecordType(Enum):
    Tactile = 0x01
    Encoder = 0x02
    Drive = 0x03
    Echo = 0x04


class Record(object):
    def __init__(self, record_type: RecordType, record_data: bytes):
        self.record_type = record_type
        self.record_content_length = len(record_data)
        self.record_data = record_data

    def pack(self) -> bytes:
        packet = b""
        packet += struct.pack("B", self.record_type.value)
        packet += struct.pack(
            "<Q", self.record_content_length
        )  # <Q little-endian uint64
        packet += self.record_data
        return packet

    def __repr__(self):
        return "record_type: {}, record_content_length: {}, record_data: {}\n".format(
            self.record_type, self.record_content_length, self.record_data
        )


class Pack(object):
    def __init__(
        self,
        pack_type: PackType = PackType.PackTypeCMD,
        data: bytes = b"",
        opcode=None,
    ):
        self.data = data
        self.pack_type_ = pack_type
        self.opcode_ = opcode

    @classmethod
    def parse(cls, data: bytes, target_type: PackType = PackType.PackTypeCMD) -> "Pack":
        if not cls.check_head(data) or not cls.check_tail(data):
            return None

    @classmethod
    def check_head(cls, data) -> bool:
        if not cls.check_magic(data[0 : len(PackContent.Magic)]):
            return False

        return True

    @classmethod
    def check_tail(cls, data) -> bool:
        if not cls.check_magic(data[-len(PackContent.Magic) :]):
            return False

        return True

    @classmethod
    def check_magic(cls, data) -> bool:
        if len(data) < len(PackContent.Magic):
            return False

        if data != PackContent.Magic:
            return False
        return True


class CmdPack(Pack):
    def __init__(
        self,
        pack_type=PackType.PackTypeCMD,
        data=b"",
        opcode=None,
        record_type=None,
        record_data=None,
    ):
        super().__init__(pack_type, data, opcode)
        self.record_type_ = record_type
        self.record_data_ = record_data

    def __repr__(self):
        return "pack_type: {}, opcode: {}, record_type: {}, record data: {}".format(
            self.pack_type_, self.opcode_, self.record_type_, self.record_data_
        )

    @classmethod
    def pack(
        cls,
        opcode: Opcode,
        record_type: RecordType,
        record=b"",
    ):
        record_content_length = len(record)

        packet = b""
        packet += PackContent.Magic
        packet += struct.pack("B", opcode.value)
        packet += struct.pack("B", record_type.value)
        # packet += struct.pack("<Q", record_content_length)  # <Q little-endian uint64
        packet += struct.pack(">I", record_content_length)  # >I big-endian uint32
        packet += struct.pack("B", 0)
        packet += struct.pack("B", 0)
        # packet += struct.pack("B", 0)
        # packet += struct.pack("B", 0)
        toque_value = 80
        packet += struct.pack(">H", toque_value)  # b'\x01\xf4'
        packet += record
        packet += struct.pack("B", 0)
        packet += struct.pack("B", 1)
        packet += PackContent.Magic

        return CmdPack(
            data=packet, opcode=opcode, record_data=record, record_type=record_type
        )
    
    @classmethod
    def pack_calib(
        cls,
        record=b"",
    ):
        record_content_length = len(record)

        packet = b""
        packet += record

        return CmdPack(
            data=packet, opcode=None, record_data=record, record_type=None
        )
    
    @classmethod
    def unpack(cls, data: bytes) -> "CmdPack":
        if not cls.check_head(data) or not cls.check_tail(data):
            return None

        # Offsets inside the packet
        magic_len = len(PackContent.Magic)
        header_end = magic_len
        opcode_pos = header_end
        record_type_pos = opcode_pos + 1
        length_pos = record_type_pos + 1
        record_start = length_pos + 8
        record_end = -magic_len

        opcode_value = data[opcode_pos]
        record_type_value = data[record_type_pos]
        record_content_length = struct.unpack("<Q", data[length_pos : length_pos + 8])[
            0
        ]
        record_data = data[record_start:record_end]

        if len(record_data) != record_content_length:
            print("ERROR! record data length not match!")
            return None

        try:
            opcode = Opcode(opcode_value)
            record_type = RecordType(record_type_value)
        except ValueError as e:
            return None

        return CmdPack(
            data=data, opcode=opcode, record_type=record_type, record_data=record_data
        )


class MessagePack(Pack):
    def __init__(
        self,
        pack_type=PackType.PackTypeMessage,
        data=b"",
        opcode=None,
        records: List[Record] = [],
    ):
        super().__init__(pack_type, data, opcode)
        self.records_ = records

    def __repr__(self):
        return "pack_type: {}, opcode: {}, records: \n {}".format(
            self.pack_type_, self.opcode_, self.records_
        )

    @classmethod
    def pack(
        cls,
        opcode: Opcode,
        records: List[Record] = [],
    ):

        packet = b""
        packet += PackContent.Magic
        packet += struct.pack("B", opcode.value)

        for record in records:
            packet += struct.pack("B", record.record_type.value)
            packet += struct.pack(
                "<Q", record.record_content_length
            )  # <Q little-endian uint64
            packet += record.record_data

        packet += PackContent.Magic
        return packet

    @classmethod
    def unpack(cls, data: bytes) -> "MessagePack":
        try:
            # print("unpack data: {}".format(data.hex()))
            if not cls.check_head(data) or not cls.check_tail(data):
                return None

            records: List[Record] = []

            magic_len = len(PackContent.Magic)
            header_end = magic_len
            opcode_pos = header_end

            i = opcode_pos + 1
            opcode_value = data[opcode_pos]

            data_end = len(data) - magic_len
            while i < data_end:
                record_type_pos = i
                record_type_value = data[record_type_pos]
                length_pos = record_type_pos + 1
                record_content_length = struct.unpack(
                    ">Q", data[length_pos : length_pos + 8]
                )[0]

                record_start = length_pos + 8
                record_end = record_start + record_content_length
                record_data = data[record_start:record_end]

                records.append(Record(RecordType(record_type_value), record_data))

                i = record_end

            return MessagePack(data=data, opcode=Opcode(opcode_value), records=records)
        except Exception  as e:
            traceback.print_exc()
            return None

    @classmethod
    def unpack_camera_calib(cls, data: bytes) -> bool:
        try:
            if not cls.check_head(data) or not cls.check_tail(data):
                return False

            magic_len = len(PackContent.Magic)
                      # The structure seems to be:
            # [Magic (5)] + [Length (2, Big Endian)] + [Opcode (1)] + [Payload] + [Tail Magic (5)]
            # Based on user input: 'das\r\n' (5) + '\x01\x91' (2) + '\n' (1, Opcode 0x0A) + ...

            length_pos = magic_len
            # Read 2 bytes length (Big Endian based on 0x0191 = 401)
            payload_length = struct.unpack(">H", data[length_pos : length_pos + 2])[0]

            opcode_pos = length_pos + 2
            opcode_value = data[opcode_pos]

            # Payload starts after Opcode
            payload_start = opcode_pos + 1
            # Payload ends before Tail Magic
            # Or we can use the parsed length to determine end?
            # Let's stick to the structure: Magic + Length + Opcode + Payload + Magic
            # Total length check?

            payload = data[payload_start : -magic_len]

            # Parse the protobuf-like payload
            calib_info = cls._parse_protobuf_calib(payload)

            # Optional YAML export
            yaml_filename = os.environ.get("CALIB_YAML_FILENAME")
            if yaml_filename:
                yaml_content = cls._generate_yaml(calib_info)
                print("Generated YAML content: ", yaml_content)
                try:
                    script_dir = os.path.dirname(os.path.abspath(__file__))
                    workspace_dir = os.path.abspath(os.path.join(script_dir, "../../../"))
                    result_dir = os.path.join(workspace_dir, "calib_result")

                    if not os.path.exists(result_dir):
                        os.makedirs(result_dir)

                    file_path = os.path.join(result_dir, yaml_filename)
                    with open(file_path, "w") as f:
                        f.write(yaml_content)
                    
                    print(f"Camera calib file saved: {file_path}")
                    return True
                except Exception as e:
                    print(f"Failed to save calib file: {e}")
                    return False
            else:
                cmd_name = os.environ.get("CALIB_CMD_NAME", "")
                prefix = f"Device response ({cmd_name})" if cmd_name else "Device response"
                try:
                    text = payload.decode("ascii")
                    print(f"{prefix}: {text}")
                except Exception:
                    print(f"{prefix}: {payload.hex()}")
                return True

        except Exception as e:
            traceback.print_exc()
            return False

    @staticmethod
    def _read_varint(data, pos):
        result = 0
        shift = 0
        while True:
            if pos >= len(data):
                print(f"pos: {pos}, data: {data}, len: {len(data)}")
                raise Exception("Varint overflow")
            b = data[pos]
            pos += 1
            result |= (b & 0x7f) << shift
            if not (b & 0x80):
                return result, pos
            shift += 7

    @classmethod
    def _parse_protobuf_calib(cls, data):
        pos = 0
        info = {
            "width": 0,
            "height": 0,
            "model": "",
            "distortion": [],
            "intrinsics": [],
            "extrinsics": [],  # tx, ty, tz, qx, qy, qz, qw
        }

        try:
            while pos < len(data):
                tag, pos = cls._read_varint(data, pos)
                field_num = tag >> 3
                wire_type = tag & 0x07

                if field_num == 2 and wire_type == 5:  # Width (Fixed32)
                    info["width"] = struct.unpack("<I", data[pos : pos + 4])[0]
                    print("info width: ", info["width"])
                    pos += 4
                elif field_num == 3 and wire_type == 5:  # Height (Fixed32)
                    info["height"] = struct.unpack("<I", data[pos : pos + 4])[0]
                    print("info height: ", info["height"])
                    pos += 4
                elif field_num == 4 and wire_type == 2:  # Model (String)
                    length, pos = cls._read_varint(data, pos)
                    info["model"] = data[pos : pos + length].decode("utf-8")
                    print("info model: ", info["model"])
                    pos += length
                elif field_num == 5 and wire_type == 2:  # Distortion Coeffs (Packed Doubles)
                    length, pos = cls._read_varint(data, pos)
                    count = length // 8
                    info["distortion"] = list(
                        struct.unpack(f"<{count}d", data[pos : pos + length])
                    )
                    print("info distortion: ", info["distortion"])
                    pos += length
                elif field_num == 6 and wire_type == 2:  # Intrinsics Matrix (Packed Doubles)
                    length, pos = cls._read_varint(data, pos)
                    count = length // 8
                    info["intrinsics"] = list(
                        struct.unpack(f"<{count}d", data[pos : pos + length])
                    )
                    print("info intrinsics: ", info["intrinsics"])
                    pos += length
                elif field_num == 10 and wire_type == 2:  # Extrinsics (Packed Doubles)
                    length, pos = cls._read_varint(data, pos)
                    count = length // 8
                    info["extrinsics"] = list(
                        struct.unpack(f"<{count}d", data[pos : pos + length])
                    )
                    print("info extrinsics: ", info["extrinsics"])
                    pos += length
                else:
                    # Skip unknown fields
                    if wire_type == 0:  # Varint
                        _, pos = cls._read_varint(data, pos)
                    elif wire_type == 1:  # Fixed64
                        pos += 8
                    elif wire_type == 2:  # Length Delimited
                        length, pos = cls._read_varint(data, pos)
                        pos += length
                    elif wire_type == 3:  # Start Group (Deprecated)
                        # Just skip the tag, hope we can parse inner fields or find End Group
                        pass
                    elif wire_type == 4:  # End Group (Deprecated)
                        # Just skip the tag
                        pass
                    elif wire_type == 5:  # Fixed32
                        pos += 4
                    else:
                        # Unknown wire type, skip
                        break
        
        except Exception as e:
            print(f"Protobuf parse error: {e}")                
        return info

    @staticmethod
    def _generate_yaml(info):
        # Map model name
        model = "kb4" if info["model"] == "equidistant" else info["model"]

        # Extract intrinsics: [fx, 0, cx, 0, fy, cy, 0, 0, 1] -> [fx, fy, cx, cy]
        K = info["intrinsics"]
        intrinsics = [K[0], K[4], K[2], K[5]] if len(K) >= 6 else []

        # Calculate Extrinsics Matrix T_BS
        # Data: tx, ty, tz, qx, qy, qz, qw
        E = info["extrinsics"]
        if len(E) >= 7:
            tx, ty, tz = E[0], E[1], E[2]
            qx, qy, qz, qw = E[3], E[4], E[5], E[6]

            # Quaternion to Rotation Matrix
            # R = [ 1-2y^2-2z^2, 2xy-2zw,     2xz+2yw ]
            #     [ 2xy+2zw,     1-2x^2-2z^2, 2yz-2xw ]
            #     [ 2xz-2yw,     2yz+2xw,     1-2x^2-2y^2 ]

            xx, yy, zz = qx * qx, qy * qy, qz * qz
            xy, xz, yz = qx * qy, qx * qz, qy * qz
            wx, wy, wz = qw * qx, qw * qy, qw * qz

            r00 = 1 - 2 * (yy + zz)
            r01 = 2 * (xy - wz)
            r02 = 2 * (xz + wy)

            r10 = 2 * (xy + wz)
            r11 = 1 - 2 * (xx + zz)
            r12 = 2 * (yz - wx)

            r20 = 2 * (xz - wy)
            r21 = 2 * (yz + wx)
            r22 = 1 - 2 * (xx + yy)

            t_bs_data = [
                r00,
                r01,
                r02,
                tx,
                r10,
                r11,
                r12,
                ty,
                r20,
                r21,
                r22,
                tz,
                0.0,
                0.0,
                0.0,
                1.0,
            ]
        else:
            t_bs_data = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]

        # Format YAML string
        yaml_str = "# General sensor definitions.\n"
        yaml_str += "sensor_type: camera\n"
        yaml_str += "comment: DAS Camera cam0\n\n"

        yaml_str += "# Sensor extrinsics wrt. the body-frame.\n"
        yaml_str += "T_BS:\n"
        yaml_str += "  cols: 4\n"
        yaml_str += "  rows: 4\n"
        yaml_str += "  data: ["
        yaml_str += ", ".join(f"{x:g}" for x in t_bs_data)
        yaml_str += "]\n\n"

        yaml_str += "# Camera specific definitions.\n"
        yaml_str += "rate_hz: 30\n"
        yaml_str += f"resolution: [{info['width']}, {info['height']}]\n"
        yaml_str += f"camera_model: {model}\n"
        yaml_str += (
            "intrinsics: ["
            + ", ".join(f"{x}" for x in intrinsics)
            + "] #fu, fv, cu, cv\n"
        )
        yaml_str += f"distortion_model: {model}\n"
        yaml_str += (
            "distortion_coefficients: ["
            + ", ".join(f"{x}" for x in info["distortion"])
            + "] #k1, k2, k3, k4\n"
        )

        return yaml_str
    

if __name__ == "__main__":
    cmd_l = CmdPack.pack_calib(record=b"camerarl")  # read camera left 

    cmd_c = CmdPack.pack_calib(record=b"camerarc")

    cmd_r = CmdPack.pack_calib(record=b"camerarr")

    # l_cmd_pack = CmdPack.unpack(data=cmd_l.data)
    # c_cmd_pack = CmdPack.unpack(data=cmd_l.data)
    # r_cmd_pack = CmdPack.unpack(data=cmd_l.data)

    # print(l_cmd_pack)
    # print(c_cmd_pack)
    # print(r_cmd_pack)

    msg_pack_data = MessagePack.pack(
        opcode=Opcode.ReadBatch,
        records=[
            Record(record_type=RecordType.Tactile, record_data=b"tactile"),
            Record(record_type=RecordType.Encoder, record_data=b"encoder"),
        ],
    )

    msg_pack = MessagePack.unpack(msg_pack_data)
    print(msg_pack)