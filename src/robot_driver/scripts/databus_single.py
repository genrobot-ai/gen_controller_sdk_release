#!/usr/bin/env python3
# Path setup at file start
import sys
import os

# Add script directory to Python path
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

import serial
import serial.tools.list_ports
import threading
import time
import json
import logging
import queue
import traceback
import struct
import os
import subprocess
from datetime import datetime
from pack import CmdPack, MessagePack, Opcode, RecordType
from das_protocol import DASProtocol

# ROS imports
import rospy
from std_msgs.msg import Float32, Int8MultiArray, Int16MultiArray, MultiArrayLayout, MultiArrayDimension

# Global topic name placeholders
TOPIC_LEFT_TACTILE = '/das_controller/tactile_single_l'
TOPIC_RIGHT_TACTILE = '/das_controller/tactile_single_r'
TOPIC_ENCODER = '/das_controller/encoder_data'
TOPIC_TARGET_DISTANCE = '/das_controller/target_dis'

def load_ros_params():
    """Load topic names from the ROS parameter server."""
    global TOPIC_LEFT_TACTILE, TOPIC_RIGHT_TACTILE, TOPIC_ENCODER, TOPIC_TARGET_DISTANCE
    
    try:
        # Private namespace parameters
        TOPIC_LEFT_TACTILE = rospy.get_param('~topic_left_tactile', '/das_controller/tactile_single_l')
        TOPIC_RIGHT_TACTILE = rospy.get_param('~topic_right_tactile', '/das_controller/tactile_single_r')
        TOPIC_ENCODER = rospy.get_param('~topic_encoder', '/das_controller/encoder_data')
        TOPIC_TARGET_DISTANCE = rospy.get_param('~topic_target_distance', '/das_controller/target_dis')
        
        # rospy.loginfo("DAS topic config loaded:")
        # rospy.loginfo(f"  left tactile: {TOPIC_LEFT_TACTILE}")
        # rospy.loginfo(f"  right tactile: {TOPIC_RIGHT_TACTILE}")
        # rospy.loginfo(f"  encoder: {TOPIC_ENCODER}")
        # rospy.loginfo(f"  target distance: {TOPIC_TARGET_DISTANCE}")
        
    except Exception as e:
        # rospy.logwarn(f"Failed to load ROS params, using defaults: {e}")
        pass

def tactile_callback(record_data: bytes):
    # print("tactile data: len: {}".format(len(record_data)))
    # Hardware expects exactly 448 bytes
    if len(record_data) != 448:
        print(f"Bad data length: expected 448 bytes, got {len(record_data)}")
        return

    if not hasattr(tactile_callback, "pub_initialized"):
        # Topic names from globals
        load_ros_params()
        tactile_callback.left_publisher_uint8 = rospy.Publisher(TOPIC_LEFT_TACTILE, Int8MultiArray, queue_size=10)
        tactile_callback.right_publisher_uint8 = rospy.Publisher(TOPIC_RIGHT_TACTILE, Int8MultiArray, queue_size=10)
        tactile_callback.pub_initialized = True
        # print(f"Tactile publishers initialized")
        # print(f"  left topic: {TOPIC_LEFT_TACTILE}")
        # print(f"  right topic: {TOPIC_RIGHT_TACTILE}")
    
    try:
        # 1. Parse 448 raw bytes: left 224 (0-223), right 224 (224-447)
        raw_left_224 = [struct.unpack("B", record_data[i:i+1])[0] for i in range(0, 224)]
        raw_right_224 = [struct.unpack("B", record_data[i:i+1])[0] for i in range(224, 448)]
        
        # print(f"raw left: {len(raw_left_224)} vals, range {min(raw_left_224)}~{max(raw_left_224)}")
        # print(f"raw right: {len(raw_right_224)} vals, range {min(raw_right_224)}~{max(raw_right_224)}")

        # 2. Duplicate each sample (1->2) for 896 values total
        left_expanded_448 = []
        for val in raw_left_224:
            left_expanded_448.append(val)  # original
            left_expanded_448.append(val)  # duplicate (sync press)
        
        right_expanded_448 = []
        for val in raw_right_224:
            right_expanded_448.append(val)  # original
            right_expanded_448.append(val)  # duplicate (sync press)

        # 3. 100x10 grid: rows 0-49 left, 50-99 right
        total_grid = [[0 for _ in range(10)] for _ in range(100)]

        # 4. Coordinates filled with -1 (no tactile change)
        # 4.1 Left sensor (rows 0-49)
        left_neg_coords = [
            # top-left
            (0,0), (0,1), (0,2), (1,0), (1,1), (2,0),
            # top-right
            (0,7), (0,8), (0,9), (1,8), (1,9), (2,9),
            # bottom-left
            (49,0), (49,1), (49,2), (49,3),
            (48,0), (48,1), (48,2), (48,3),
            (47,0), (47,1), (47,2),
            (46,0), (46,1), (46,2),
            (45,0), (45,1), (45,2),
            (44,0), (44,1),
            (43,0),
            # bottom-right
            (49,6), (49,7), (49,8), (49,9),
            (48,6), (48,7), (48,8), (48,9),
            (47,7), (47,8), (47,9),
            (46,7), (46,8), (46,9),
            (45,7), (45,8), (45,9),
            (44,8), (44,9),
            (43,9)
        ]

        # 4.2 Right sensor (rows 50-99)
        right_neg_coords = [
            # top-left
            (50,0), (50,1), (50,2), (51,0), (51,1), (52,0),
            # top-right
            (50,7), (50,8), (50,9), (51,8), (51,9), (52,9),
            # bottom-left
            (99,0), (99,1), (99,2), (99,3),
            (98,0), (98,1), (98,2), (98,3),
            (97,0), (97,1), (97,2),
            (96,0), (96,1), (96,2),
            (95,0), (95,1), (95,2),
            (94,0), (94,1),
            (93,0),
            # bottom-right
            (99,6), (99,7), (99,8), (99,9),
            (98,6), (98,7), (98,8), (98,9),
            (97,7), (97,8), (97,9),
            (96,7), (96,8), (96,9),
            (95,7), (95,8), (95,9),
            (94,8), (94,9),
            (93,9)
        ]

        # 5. Fill -1 at masked coordinates
        for (r, c) in left_neg_coords:
            total_grid[r][c] = -1
        for (r, c) in right_neg_coords:
            total_grid[r][c] = -1

        # 6. Left-to-right, top-to-bottom fill with expanded data
        # 6.1 Left sensor (rows 0-49)
        left_idx = 0
        for row in range(50):          # rows 0-49
            for col in range(10):      # cols 0-9
                if total_grid[row][col] != -1 and left_idx < len(left_expanded_448):
                    total_grid[row][col] = left_expanded_448[left_idx]
                    left_idx += 1

        # 6.2 Right sensor (rows 50-99)
        right_idx = 0
        for row in range(50, 100):     # rows 50-99
            for col in range(10):      # cols 0-9
                if total_grid[row][col] != -1 and right_idx < len(right_expanded_448):
                    total_grid[row][col] = right_expanded_448[right_idx]
                    right_idx += 1

        # 8. Split grid and flatten for ROS
        # 8.1 Left flat (rows 0-49 -> 500 values)
        left_flat = []
        for row in range(50):
            left_flat.extend(total_grid[row])
        
        # 8.2 Right flat (rows 50-99 -> 500 values)
        right_flat = []
        for row in range(50, 100):
            right_flat.extend(total_grid[row])

        # Signed int8 (no UInt8MultiArray; keep -1)
        msg_left = Int8MultiArray()
        msg_left.data = [x if x == -1 else (x if x < 128 else x - 256) for x in left_flat]
        
        msg_right = Int8MultiArray()
        msg_right.data = [x if x == -1 else (x if x < 128 else x - 256) for x in right_flat]

        # Publish ROS messages
        tactile_callback.left_publisher_uint8.publish(msg_left)
        tactile_callback.right_publisher_uint8.publish(msg_right)

        # Publish stats (optional debug)
        left_neg_count = sum(1 for x in left_flat if x == -1)
        right_neg_count = sum(1 for x in right_flat if x == -1)
        
    except Exception as e:
        print(f"Tactile processing error: {e}")
        traceback.print_exc()

def encoder_callback(record_data: bytes):
    # ROS publisher singleton
    if not hasattr(encoder_callback, "pub_initialized"):
        # Topic names from globals
        load_ros_params()
        encoder_callback.publisher = rospy.Publisher(TOPIC_ENCODER, Float32, queue_size=10)
        encoder_callback.pub_initialized = True
        # print(f"Encoder publisher initialized: {TOPIC_ENCODER}")
    
    encoder_value = struct.unpack(">f", record_data)[0]
    # print(
    #     "encoder data: {}, len: {}, encoder_value: {}".format(
    #         record_data, len(record_data), encoder_value
    #     )
    # )
    
    # Publish encoder to ROS
    try:
        msg = Float32()
        msg.data = encoder_value
        encoder_callback.publisher.publish(msg)
        # print(f"Published encoder: {encoder_value}")
    except Exception as e:
        print(f"Error publishing encoder: {e}")

def echo_callback(record_data: bytes):
    print("echo data: {}".format(record_data))

def camera_calib_callback(camera_pack):
    """Camera calibration data callback."""
    # print("camera_pack received: ", camera_pack)
    # if camera_pack:
    #     print("Camera calibration parse OK")
    # else:
    #     print("Camera calibration parse failed")

class DataBus:
    def __init__(
        self,
        tty_port="/dev/ttyUSB0",
        baudrate=115200,
        timeout=0.5,
        is_calib_cmd=False,
        calib_cmd_name: str = None,
        encoder_freq: float = None,
        tactile_freq: float = None,
        gripper_type: str = None,
    ):
        # Init ROS node
        try:
            rospy.init_node('das_ros_interface', anonymous=True)
        except rospy.exceptions.ROSException:
            # Already initialized
            pass
        
        # Load topics
        load_ros_params()
        
        self.tty_port = tty_port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.is_running = False

        self._open_serial_success = False
        self.protocol: DASProtocol = DASProtocol()
        self.data_buffer: bytes = b""
        self.data_buffer_lock = threading.Lock()
        self.serial_lock = threading.Lock()

        self.cmd_queue = queue.Queue(1000)

        self.read_thread: threading.Thread = None
        self.parse_thread: threading.Thread = None
        self.send_thread: threading.Thread = None

        self.encoder_freq = encoder_freq
        self.tactile_freq = tactile_freq
        self.gripper_type = gripper_type or rospy.get_param('~gripper_type', 'default_gripper')
        self.encoder_thread: threading.Thread = None
        self.tactile_thread: threading.Thread = None
        
        self.gripper_dis = 0.0
        self.angle_lock = threading.Lock()
        # Camera calib flag / name (e.g. MCUID prints payload between das\\r\\n)
        self.is_calib_cmd = is_calib_cmd  # default False
        self.calib_cmd_name = calib_cmd_name  # e.g. "MCUID"
        if calib_cmd_name:
            os.environ["CALIB_CMD_NAME"] = calib_cmd_name
        self.tactile_callback = tactile_callback
        self.encoder_callback = encoder_callback
        self.echo_callback = echo_callback
        self.camera_calib_callback = camera_calib_callback

        # ROS subscribers
        self._init_ros_subscribers()

        self._open_serial()
        self.is_running = True
        self._start_reading()
        self._start_parsing()
        self._start_sending()
        
        # Start periodic threads
        if self.encoder_freq:
            self._start_encoder_loop()
        if self.tactile_freq:
            self._start_tactile_loop()

    def _init_ros_subscribers(self):
        """Init ROS subscribers."""
        # Gripper open distance command
        self.motor_cmd_subscriber = rospy.Subscriber(
            TOPIC_TARGET_DISTANCE, 
            Float32, 
            self._motor_command_callback
        )
        # print(f"ROS subscriber ready: {TOPIC_TARGET_DISTANCE}")

    def _motor_command_callback(self, msg):
        try:
            with self.angle_lock:
                self.gripper_dis = msg.data
            # print(f"Target distance cmd: {msg.data} m")
        except Exception as e:
            print(f"Motor command handling error: {e}")

    def drive_motor(self, angle_dgree: float):
        self.add_cmd(
            CmdPack.pack(
                opcode=Opcode.WriteDrive,
                record_type=RecordType.Drive,
                record=struct.pack(">f", angle_dgree),
                gripper_type=self.gripper_type,
            )
        )

    def disable_motor(self):
        self.add_cmd(
            CmdPack.pack(
                opcode=Opcode.DisableDrive,
                record_type=RecordType.Drive,
                gripper_type=self.gripper_type,
            )
        )
    
    def calib_encoder(self):
        self.add_cmd(
            CmdPack.pack(
                opcode=Opcode.CalibEncoder,
                record_type=RecordType.Drive,
                gripper_type=self.gripper_type,
            )
        )

    def send_camera_calib_cmd(self, camera_cmd: str):
        """Send camera calibration command."""
        try:
            cmd = CmdPack.pack_calib(
                record=camera_cmd.encode('utf-8')
            )
            success = self.add_cmd(cmd)
            if success:
                print(f"Sent camera calib command: {camera_cmd}")
            else:
                print(f"Failed to queue camera calib command: {camera_cmd}")
            return success
        except Exception as e:
            print(f"Error sending camera calib command: {e}")
            return False

    def add_cmd(self, cmd: CmdPack) -> bool:
        try:
            self.cmd_queue.put(cmd, block=True, timeout=1)
            # print(f"Cmd queued, size: {self.cmd_queue.qsize()}")
            return True
        except queue.Full:
            print("Command queue full, add failed")
            return False

    def is_opend(self):
        return self._open_serial_success

    def register_tactile_callback(self, callback):
        self.tactile_callback = callback

    def register_encoder_callback(self, callback):
        self.encoder_callback = callback

    def register_camera_calib_callback(self, callback):
        """Register camera calibration callback."""
        self.camera_calib_callback = callback

    def _open_serial(self):
        self.ser = serial.Serial(
            port=self.tty_port,
            baudrate=self.baudrate,
            timeout=self.timeout,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
        )

        if self.ser.is_open:
            print(f"open {self.tty_port} success!, baudrate: {self.baudrate}")
            self._open_serial_success = True
        else:
            print(f"open {self.tty_port} failed!, baudrate: {self.baudrate}")
            self._open_serial_success = False

    def _start_reading(self):
        self.read_thread = threading.Thread(target=self._reading_loop)
        self.read_thread.daemon = True
        self.read_thread.start()
        # print("Read thread started")
        return True

    def _start_parsing(self):
        self.parse_thread = threading.Thread(target=self._parsing_loop)
        self.parse_thread.daemon = True
        self.parse_thread.start()
        # print("Parse thread started")
        return True

    def _start_encoder_loop(self):
        """Start encoder polling thread."""
        self.encoder_thread = threading.Thread(target=self._send_encoder_loop)
        self.encoder_thread.daemon = True
        self.encoder_thread.start()
        # print("Encoder loop thread started")
        return True

    def _start_tactile_loop(self):
        """Start tactile polling thread."""
        self.tactile_thread = threading.Thread(target=self._send_tactile_loop)
        self.tactile_thread.daemon = True
        self.tactile_thread.start()
        # print("Tactile loop thread started")
        return True

    def _start_sending(self):
        self.send_thread = threading.Thread(target=self._sending_loop)
        self.send_thread.daemon = True
        self.send_thread.start()
        # print("Send thread started")
        return True

    def _sending_loop(self):
        """Send thread main loop."""
        while self.is_running and not rospy.is_shutdown():
            try:
                cmd: CmdPack = self.cmd_queue.get(block=True, timeout=0.1)
                with self.serial_lock:
                    if self.ser and self.ser.is_open:
                        self.ser.write(cmd.data)
                        self.ser.flush()
                        # print(f"Sent cmd, queue left: {self.cmd_queue.qsize()}")
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"Send error: {e}")
                time.sleep(0.01)

        # Exit send thread (silent)

    def _reading_loop(self):
        """Read thread main loop."""
        while self.is_running and not rospy.is_shutdown():
            try:
                with self.serial_lock:
                    if self.ser and self.ser.is_open:
                        n = self.ser.inWaiting()
                        if n:
                            data = self.ser.read(n)
                            with self.data_buffer_lock:
                                self.data_buffer = self.data_buffer + data
                                # print(f"Read {len(data)} bytes, buffer {len(self.data_buffer)}")

            except Exception as e:
                print(f"Read loop error: {e}")
                time.sleep(0.1)

            time.sleep(0.001)  # yield CPU

        # Exit read thread (silent)

    def _parsing_loop(self):
        """Parse thread: handles multiple packet types."""
        while self.is_running and not rospy.is_shutdown():
            with self.data_buffer_lock:
                if len(self.data_buffer) > 0:
                    # print(f"Parse buffer: {len(self.data_buffer)} bytes")
                    
                    # Use DASProtocol.find_packet on instance buffer
                    packets, remain = DASProtocol.find_packet(self.data_buffer)
                    self.data_buffer = remain
                    
                    # print("packets_biaoding: ", packets)
                    for packet in packets:
                        if self.is_calib_cmd:
                            magic = DASProtocol.MAGIC  # b"das\r\n"
                            is_camera_cmd = (
                                self.calib_cmd_name
                                and self.calib_cmd_name.startswith("camera")
                            )

                            # Camera calib uses das\\r\\n framing but carries protobuf payload.
                            if is_camera_cmd:
                                camera_pack = MessagePack.unpack_camera_calib(packet)
                                if camera_pack:
                                    if self.camera_calib_callback:
                                        self.camera_calib_callback(camera_pack)
                                    self.is_calib_cmd = False
                                    continue

                            if (
                                len(packet) > 2 * len(magic)
                                and packet.startswith(magic)
                                and packet.endswith(magic)
                            ):
                                middle = packet[len(magic) : -len(magic)]
                                try:
                                    text = middle.decode("ascii")
                                except Exception:
                                    text = middle.hex()
                                if self.calib_cmd_name == "MCUID":
                                    print("MCUID:", text)
                                else:
                                    print(f"Device response ({self.calib_cmd_name}): {text}")
                                self.is_calib_cmd = False
                                continue

                            if not is_camera_cmd:
                                camera_pack = MessagePack.unpack_camera_calib(packet)
                                if camera_pack:
                                    if self.camera_calib_callback:
                                        self.camera_calib_callback(camera_pack)
                                    self.is_calib_cmd = False
                                    continue

                            pack = MessagePack.unpack(packet)
                            if pack:
                                for record in pack.records_:
                                    if record.record_type == RecordType.Echo:
                                        try:
                                            text = record.record_data.decode("utf-8")
                                        except Exception:
                                            text = record.record_data.hex()
                                        print(f"Device response ({self.calib_cmd_name}): {text}")
                                        self.is_calib_cmd = False
                                        break
                        else:
                            pack = MessagePack.unpack(packet)
                            # print("pack_normal: ", pack)
                            if not pack:
                                continue

                            for record in pack.records_:
                                if record.record_type == RecordType.Tactile:
                                    self.tactile_callback(record.record_data)
                                elif record.record_type == RecordType.Encoder:
                                    self.encoder_callback(record.record_data)
                                elif record.record_type == RecordType.Echo:
                                    self.echo_callback(record.record_data)
                                else:
                                    logging.error(
                                        "record type:{} invalid !".format(record.record_type)
                                    )

            time.sleep(0.01)  # yield CPU

        # Exit parse thread (silent)

    def _send_encoder_loop(self):
        """Encoder loop using ROS-subscribed target distance."""
        if not self.encoder_freq:
            return
            
        interval = 1.0 / self.encoder_freq
        # print(f"Encoder loop {self.encoder_freq} Hz, interval {interval:.3f}s")
        
        while self.is_running and not rospy.is_shutdown():
            start_time = time.time()
            
            # Distance command to motor
            with self.angle_lock:
                dis_target = self.gripper_dis
            
            self.add_cmd(
                CmdPack.pack(
                    opcode=Opcode.ReadBatch, 
                    record_type=RecordType.Encoder, 
                    record=struct.pack(">f", dis_target),
                    gripper_type=self.gripper_type,
                ),
            )
            
            # Pace to target period
            elapsed = time.time() - start_time
            sleep_time = max(0, interval - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)

        print("Encoder loop thread exiting")

    def _send_tactile_loop(self):
        """Tactile polling loop."""
        if not self.tactile_freq:
            return
            
        interval = 1.0 / self.tactile_freq
        print(f"Tactile loop started, {self.tactile_freq} Hz, interval {interval:.3f}s")
        
        while self._should_run():
            start_time = time.time()
            self.add_cmd(
                CmdPack.pack(
                    opcode=Opcode.ReadSingle,
                    record_type=RecordType.Tactile,
                    record=struct.pack(">f", 0.0),
                    gripper_type=self.gripper_type,
                )
            )
            
            # Pace to target period
            elapsed = time.time() - start_time
            sleep_time = max(0, interval - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)

        print("Tactile loop thread exiting")

    def wait_for_calib_response(self, timeout=3.0, poll_interval=0.05):
        """Wait until calib response is received or timeout expires."""
        if not self.is_calib_cmd:
            return True
        print("Waiting for device response...")
        deadline = time.time() + timeout
        while time.time() < deadline:
            if not self.is_calib_cmd:
                return True
            time.sleep(poll_interval)
        return not self.is_calib_cmd

    def stop(self):
        """Stop all threads."""
        self.is_running = False
        
        # Join threads (no status prints)
        threads_to_join = []
        if self.read_thread and self.read_thread.is_alive():
            threads_to_join.append(self.read_thread)
        if self.send_thread and self.send_thread.is_alive():
            threads_to_join.append(self.send_thread)
        if self.parse_thread and self.parse_thread.is_alive():
            threads_to_join.append(self.parse_thread)
        if self.encoder_thread and self.encoder_thread.is_alive():
            threads_to_join.append(self.encoder_thread)
        if self.tactile_thread and self.tactile_thread.is_alive():
            threads_to_join.append(self.tactile_thread)
        
        for thread in threads_to_join:
            thread.join(timeout=2)
        
        if self.ser and self.ser.is_open:
            self.ser.close()

    def get_serial_info(self):
        """Serial port info dict."""
        if self.ser and self.ser.is_open:
            info = {
                "tty_port": self.tty_port,  # stored port name
                "baudrate": self.ser.baudrate,
                "bytesize": self.ser.bytesize,
                "parity": self.ser.parity,
                "stopbits": self.ser.stopbits,
                "timeout": self.ser.timeout,
                "in_waiting": self.ser.in_waiting,
            }
            return info
        return None

def check_and_fix_permission(port):
    """Check/fix serial device permissions."""
    if not os.path.exists(port):
        return False
    
    if os.access(port, os.R_OK | os.W_OK):
        return True
    
    print(f"Trying to fix permissions on {port}...")
    try:
        subprocess.run(['sudo', 'chmod', '666', port], check=True)
        print(f"Permissions fixed: {port}")
        return True
    except subprocess.CalledProcessError:
        print(f"Permission fix failed; run manually: sudo chmod 666 {port}")
        return False

def find_serial_port(pattern="ttyUSB", max_retries=3, retry_interval=2, side=None):
    """
    Find serial ports matching pattern with retries.
    side: 'left' or 'right' for mapped device nodes.
    """
    # Prefer udev-mapped devices
    if side == "left":
        left_port = "/dev/ttyDeviceLeft"
        if os.path.exists(left_port):
            if check_and_fix_permission(left_port):
                print(f"Using left mapped device: {left_port}")
                return left_port
    elif side == "right":
        right_port = "/dev/ttyDeviceRight"
        if os.path.exists(right_port):
            if check_and_fix_permission(right_port):
                print(f"Using right mapped device: {right_port}")
                return right_port
    
    # Fall back to auto-detect if no mapped node
    for attempt in range(max_retries):
        ports = list(serial.tools.list_ports.comports())
        
        matching_ports = []
        for port in ports:
            if pattern in port.device:
                matching_ports.append(port.device)
        
        # Sort by numeric suffix
        matching_ports.sort(key=lambda x: int(x.replace(f"/dev/{pattern}", "")) if x.replace(f"/dev/{pattern}", "").isdigit() else -1)
        
        if matching_ports:
            print(f"Found serial ports: {matching_ports}")
            
            if side:
                print(f"Warning: no {side} mapped device, using first auto-detected port")
            
            for port in matching_ports:
                if check_and_fix_permission(port):
                    return port
            # No permission on any: return last and let user fix
            return matching_ports[-1]
        
        if attempt < max_retries - 1:
            print(f"Attempt {attempt + 1}: no serial port, retry in {retry_interval}s...")
            time.sleep(retry_interval)
    
    print(f"No serial port found after {max_retries} attempts")
    return None

def main():
    # Strip ROS-injected argv
    ros_args = ['__name:=', '__log:=', '__master:=', '__ip:=']
    filtered_args = []
    
    for arg in sys.argv[1:]:
        if not any(arg.startswith(ros_arg) for ros_arg in ros_args):
            filtered_args.append(arg)
    
    sys.argv = [sys.argv[0]] + filtered_args
    
    # CLI args
    import argparse
    parser = argparse.ArgumentParser(description="DAS interface")
    parser.add_argument("--serial-port", type=str, default="", 
                       help="Serial port device (e.g., /dev/ttyUSB0)")
    parser.add_argument("--camera-cmd", type=str, default="",
                       help="Camera calibration command (e.g., 'MCUID' for camera ID query)")
    parser.add_argument("--side", type=str, default="", choices=["left", "right"],
                       help="Device side: left or right (uses mapped ports)")
    args, unknown = parser.parse_known_args()
    
    side = args.side
    
    # ROS param override
    if not side:
        try:
            rospy.init_node('das_ros_interface', anonymous=True)
        except:
            pass
        
        side = rospy.get_param('~side', '')
    
    # Serial port selection
    serial_port = None
    if args.serial_port and args.serial_port != "":
        serial_port = args.serial_port
        print(f"Using CLI serial port: {serial_port}")
    elif side:
        if side == "left":
            serial_port = "/dev/ttyDeviceLeft"
        elif side == "right":
            serial_port = "/dev/ttyDeviceRight"
        print(f"Using {side}-side mapped device: {serial_port}")
    else:
        # ROS param
        try:
            if not rospy.core.is_initialized():
                rospy.init_node('das_ros_interface', anonymous=True)
        except:
            pass
        
        serial_port = rospy.get_param('~serial_port', '')
        if serial_port and serial_port != "":
            # print(f"Using ROS param serial: {serial_port}")
            pass
    # Auto-discover
    if not serial_port or serial_port == "":
        serial_port = find_serial_port("ttyUSB", side=side)
        if serial_port is None:
            # print("No serial port; check device connection")
            return
    
    # print(f"Serial port in use: {serial_port}")
    
    # DataBus
    bus = DataBus(
        tty_port=serial_port,
        baudrate=921600,
        encoder_freq=30,
        is_calib_cmd=False,
    )

    time.sleep(1)
    
    if args.camera_cmd and args.camera_cmd != "":
        # bus.is_calib_cmd = True
        print(f"Sending camera calib command: {args.camera_cmd}")
        bus.send_camera_calib_cmd(args.camera_cmd)
    
    try:
        label = f"{side}-side" if side else "single-device"
        print(f"\nDevice initialized; {label} running (Ctrl+C to exit)...")
        rospy.spin()
            
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
        traceback.print_exc()
    finally:
        print("\nStopping device and closing serial...")
        bus.stop()
        print("Shutdown complete")

if __name__ == "__main__":
    main() 