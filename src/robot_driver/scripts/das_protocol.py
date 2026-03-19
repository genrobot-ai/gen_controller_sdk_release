import struct
from datetime import datetime
from typing import List, Tuple, Optional, Dict, Any
import logging
import threading


class DASProtocol:
    MAGIC = b"das\r\n"
    MAGIC_LENGTH = len(MAGIC)

    MAX_PACKET_SIZE = 4096  # max single packet size
    MAX_BUFFER_SIZE = 8192  # max parse buffer

    def __init__(self):
        self.logger = logging.getLogger("DASProtocol")

    @classmethod
    def find_packet(cls, data: bytes) -> Tuple[List[bytes], bytes]:
        packets = []
        buffer = data

        if len(buffer) > cls.MAX_BUFFER_SIZE:
            cls._log_warning(f"Buffer too large ({len(buffer)} bytes), clearing")
            return [], b""

        search_start = 0
        processed_count = 0

        while len(buffer) - search_start >= cls.MAGIC_LENGTH * 2:
            processed_count += 1
            if processed_count > 1000:
                cls._log_warning("Possible infinite loop, aborting parse")
                break

            # Find header magic
            header_pos = buffer.find(cls.MAGIC, search_start)
            if header_pos == -1:
                remaining_data = buffer[search_start:]
                cls._log_warning("not found header from index: {}".format(search_start))
                break

            # Back-to-back magic
            next_magic_pos = buffer.find(cls.MAGIC, header_pos + cls.MAGIC_LENGTH)
            if next_magic_pos == header_pos + cls.MAGIC_LENGTH:
                cls._log_warning(f"Consecutive magic at {header_pos}")
                search_start = header_pos + cls.MAGIC_LENGTH
                continue

            if header_pos > len(buffer) - cls.MAGIC_LENGTH * 2:
                cls._log_warning(f"Bad header position: {header_pos}")
                remaining_data = buffer[search_start:]
                break

            footer_search_start = header_pos + cls.MAGIC_LENGTH
            footer_pos = buffer.find(cls.MAGIC, footer_search_start)
            if footer_pos == -1:
                remaining_data = buffer[header_pos:]
                cls._log_info(
                    "not found footer from index: {}".format(footer_search_start)
                )
                break

            if footer_pos > len(buffer) - cls.MAGIC_LENGTH:
                cls._log_warning(
                    f"Bad footer position: {footer_pos}, len buffer:{len(buffer)}"
                )
                remaining_data = buffer[header_pos:]
                break

            next_after_footer = footer_pos + cls.MAGIC_LENGTH
            if (
                next_after_footer < len(buffer)
                and buffer.find(cls.MAGIC, next_after_footer) == next_after_footer
            ):
                cls._log_warning("Consecutive footer magic (possible glued packets)")

            packet_end = footer_pos + cls.MAGIC_LENGTH
            full_packet = buffer[header_pos:packet_end]

            if len(full_packet) > cls.MAX_PACKET_SIZE:
                cls._log_warning(f"Packet too large ({len(full_packet)} bytes), skip")
                search_start = header_pos + cls.MAGIC_LENGTH
                continue
            if cls._validate_packet_structure(full_packet):
                packets.append(full_packet)
                search_start = packet_end
            else:
                cls._log_warning("Invalid packet structure, skip")
                search_start = header_pos + cls.MAGIC_LENGTH
        else:
            remaining_data = buffer[search_start:]

        if packets:
            cls._log_debug(
                f"Found {len(packets)} packet(s), {len(remaining_data)} bytes left"
            )

        return packets, remaining_data

    @classmethod
    def _validate_packet_structure(cls, packet: bytes) -> bool:
        """Return True if packet has valid framing."""
        try:
            if len(packet) < cls.MAGIC_LENGTH * 2:
                cls._log_warning(f"Packet too short: {len(packet)} bytes")
                return False

            header = packet[: cls.MAGIC_LENGTH]
            footer = packet[-cls.MAGIC_LENGTH :]

            if header != cls.MAGIC:
                cls._log_warning("Header magic mismatch")
                return False

            if footer != cls.MAGIC:
                cls._log_warning("Footer magic mismatch")
                return False

            content = packet[cls.MAGIC_LENGTH : -cls.MAGIC_LENGTH]
            if len(content) < 1:
                cls._log_warning("Empty packet payload")
                return False

            opcode = content[0]
            if opcode > 0xFF:
                cls._log_warning(f"Invalid opcode: {opcode}")
                return False

            return True

        except Exception as e:
            cls._log_error(f"Packet validation error: {e}")
            return False

    @classmethod
    def parse_packet(cls, packet: bytes) -> Optional[Dict[str, Any]]:
        """Parse one framed packet."""
        try:
            if not cls._validate_packet_structure(packet):
                return None

            content = packet[cls.MAGIC_LENGTH : -cls.MAGIC_LENGTH]

            opcode = content[0]

            data_section = content[1:] if len(content) > 1 else b""

            return {
                "opcode": opcode,
                "data_section": data_section,
                "data_length": len(data_section),
                "raw_packet": packet,
                "packet_length": len(packet),
                "timestamp": datetime.now(),
            }

        except Exception as e:
            cls._log_error(f"Parse error: {e}")
            return None

    @classmethod
    def create_packet(cls, opcode: int, data: bytes = b"") -> bytes:
        try:
            if not isinstance(opcode, int) or opcode < 0 or opcode > 255:
                raise ValueError("opcode must be int 0-255")

            if not isinstance(data, bytes):
                raise ValueError("data must be bytes")

            if len(data) > 1024:
                raise ValueError("data too long")

            return cls.MAGIC + bytes([opcode]) + data + cls.MAGIC

        except Exception as e:
            cls._log_error(f"create_packet error: {e}")
            raise

    @classmethod
    def _log_info(cls, message: str):
        logging.info(f"[DASProtocol] {message}")

    @classmethod
    def _log_debug(cls, message: str):
        logging.debug(f"[DASProtocol] {message}")

    @classmethod
    def _log_warning(cls, message: str):
        logging.warning(f"[DASProtocol] {message}")

    @classmethod
    def _log_error(cls, message: str):
        logging.error(f"[DASProtocol] {message}")


class DASController:
    def __init__(self):
        self.buffer = b""
        self.buffer_lock = threading.Lock()
        self.error_count = 0
        self.max_consecutive_errors = 10
        self.consecutive_errors = 0
        self.stats = {
            "total_packets": 0,
            "valid_packets": 0,
            "invalid_packets": 0,
            "recovered_packets": 0,
            "buffer_resets": 0,
        }

    def process_received_data(self, new_data: bytes):
        try:
            with self.buffer_lock:
                self.buffer += new_data

                if not self._check_buffer_health():
                    self._reset_buffer()
                    return

                packets, remaining_data = DASProtocol.find_packet(self.buffer)

                self.buffer = remaining_data

            for packet in packets:
                self._handle_packet_with_retry(packet)

            self._log_warning("find packets num: {}".format(len(packets)))

        except Exception as e:
            self._handle_processing_error(e)

    def _check_buffer_health(self) -> bool:
        if len(self.buffer) > DASProtocol.MAX_BUFFER_SIZE:
            self._log_error(f"Buffer too large: {len(self.buffer)} bytes")
            return False
        return True

    def _reset_buffer(self):
        """Clear internal buffer."""
        self._log_warning("Reset buffer")
        self.buffer = b""
        self.consecutive_errors = 0
        self.stats["buffer_resets"] += 1

    def _handle_packet_with_retry(self, packet: bytes, max_retries: int = 3):
        """Parse packet with retries."""
        for attempt in range(max_retries):
            try:
                parsed = DASProtocol.parse_packet(packet)
                if parsed:
                    self._handle_valid_packet(parsed)
                    return
                else:
                    self._handle_invalid_packet(packet, attempt)
            except Exception as e:
                self._log_error(f"Packet handling error (try {attempt+1}): {e}")

        self.stats["invalid_packets"] += 1

    def _handle_valid_packet(self, parsed_packet: Dict):
        """Dispatch valid packet."""
        self.stats["valid_packets"] += 1
        self.stats["total_packets"] += 1

        print(
            f"[{parsed_packet['timestamp'].strftime('%H:%M:%S.%f')}] "
            f"Valid packet - OPCODE: 0x{parsed_packet['opcode']:02X}, "
            f"payload len: {parsed_packet['data_length']}"
        )

        self._dispatch_by_opcode(parsed_packet["opcode"], parsed_packet["data_section"])

    def _handle_invalid_packet(self, packet: bytes, attempt: int):
        self._log_warning(f"Invalid packet (try {attempt+1}): len={len(packet)} bytes")

        if attempt == 0:
            self._log_debug(f"Invalid hex preview: {packet.hex()[:100]}...")

    def _handle_processing_error(self, error: Exception):
        """Handle runtime errors during processing."""
        self.error_count += 1
        self.consecutive_errors += 1

        self._log_error(f"Processing error: {error}")

        if self.consecutive_errors >= self.max_consecutive_errors:
            self._log_error("Too many errors, recovering")
            self._reset_buffer()

    def _dispatch_by_opcode(self, opcode: int, data: bytes):
        """Route by opcode."""
        try:
            if opcode == 0x01:
                self._handle_sensor_data(data)
            elif opcode == 0x02:
                self._handle_config_data(data)
            elif opcode == 0x03:
                self._handle_status_data(data)
            else:
                self._log_warning(f"Unknown OPCODE: 0x{opcode:02X}")
        except Exception as e:
            self._log_error(f"Opcode dispatch error: {e}")

    def _handle_sensor_data(self, data: bytes):
        """Decode sensor payload (example)."""
        try:
            if len(data) >= 4:
                timestamp = int.from_bytes(data[:4], "little")
                sensor_value = data[4:] if len(data) > 4 else b""
                print(f"Sensor - ts: {timestamp}, value: {sensor_value.hex()}")
        except Exception as e:
            self._log_error(f"Sensor handler error: {e}")

    def _handle_config_data(self, data: bytes):
        print("Handling config data")

    def _handle_status_data(self, data: bytes):
        print("Handling status data")

    def get_statistics(self) -> Dict:
        return self.stats.copy()

    def _log_info(self, message: str):
        logging.info(f"[DASController] {message}")

    def _log_debug(self, message: str):
        logging.debug(f"[DASController] {message}")

    def _log_warning(self, message: str):
        logging.warning(f"[DASController] {message}")

    def _log_error(self, message: str):
        logging.error(f"[DASController] {message}")


def test_error_handling():
    print("=== DAS protocol error-handling test ===")

    logging.basicConfig(
        level=logging.DEBUG, format="%(asctime)s - %(levelname)s - %(message)s"
    )

    controller = DASController()

    test_cases = [
        DASProtocol.create_packet(0x02, b"good_data")
        + b"garbage"
        + DASProtocol.create_packet(0x03, b"end_data")
    ]

    for i, test_data in enumerate(test_cases):
        print(f"\n--- Case {i+1} ---")
        print(f"Input length: {len(test_data)} bytes")

        controller.process_received_data(test_data)


if __name__ == "__main__":
    test_error_handling()
