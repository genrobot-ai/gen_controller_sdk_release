#!/usr/bin/env python3
# Path setup at file start
import sys
import os

# Add script directory to Python path
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

import cv2
import os
import time
import glob
import subprocess
import signal
import sys
import rospy
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import threading
from std_msgs.msg import Header

class CameraCaptureROS:
    def __init__(self):
        try:
            rospy.init_node('camera_capture_node', anonymous=True)
        except rospy.exceptions.ROSException:
            pass
        
        self._load_ros_params()
        
        self.node_name = rospy.get_name().replace('/', '_')
        if self.node_name == '_unnamed':
            self.node_name = 'camera_default'
        
        # rospy.loginfo(f"Node id: {self.node_name}")
        
        self.cameras = []
        self.running = True
        
        # CvBridge disabled (libffi issues)
        # self.bridge = CvBridge()
        
        self.image_publishers = []
        
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        self._init_cameras()
        self._init_ros_publishers()

    def _load_ros_params(self):
        """Load config from ROS parameter server."""
        self.show_preview = rospy.get_param('~show_preview', True)
        
        # Parse resolution list
        resolutions_str = rospy.get_param('~resolutions', "640x480,320x240,800x600")
        self.resolutions = []
        for res_str in resolutions_str.split(','):
            try:
                width, height = map(int, res_str.strip().split('x'))
                self.resolutions.append((width, height))
            except:
                rospy.logwarn(f"Cannot parse resolution string: {res_str}")
        
        if not self.resolutions:
            self.resolutions = [(640, 480), (320, 240), (800, 600)]
        
        self.topic_base = rospy.get_param('~topic_base', '/camera_fisheye')
        
        # launch may pass camera_count as string
        self.max_cameras = int(rospy.get_param('~camera_count', 3))
        
        # USB port filter for v4l devices
        self.usb_port = rospy.get_param('~usb_port', '')

        self.video0_main = rospy.get_param('~video_0_main', '')
        self.video0_sec = rospy.get_param('~video_0_sec', '')
        self.video1_main = rospy.get_param('~video_1_main', '')
        self.video1_sec = rospy.get_param('~video_1_sec', '')
        self.video2_main = rospy.get_param('~video_2_main', '')
        self.video2_sec = rospy.get_param('~video_2_sec', '')
        
        # rospy.loginfo(f"Camera config loaded:")
        # rospy.loginfo(f"  node: {rospy.get_name()}")
        # rospy.loginfo(f"  preview: {self.show_preview}")
        # rospy.loginfo(f"  resolutions: {self.resolutions}")
        # rospy.loginfo(f"  topic_base: {self.topic_base}")
        # rospy.loginfo(f"  max_cameras: {self.max_cameras}")
        # rospy.loginfo(f"  usb_port: {self.usb_port}")
        # rospy.loginfo(f"  - video0_main: {self.video0_main}")
        # rospy.loginfo(f"  - video0_sec: {self.video0_sec}")
        # rospy.loginfo(f"  - video1_main: {self.video1_main}")
        # rospy.loginfo(f"  - video1_sec: {self.video1_sec}")
        # rospy.loginfo(f"  - video2_main: {self.video2_main}")
        # rospy.loginfo(f"  - video2_sec: {self.video2_sec}")

    def _signal_handler(self, signum, frame):
        rospy.loginfo(f"\nSignal {signum} received, stopping capture...")
        self.running = False
        rospy.signal_shutdown("User requested shutdown")

    def _get_physical_devices(self):
        try:
            result = subprocess.run(['v4l2-ctl', '--list-devices'], 
                                 capture_output=True, text=True)
            devices = []
            current_dev = ""
            device_names = {}
            
            for line in result.stdout.split('\n'):
                if not line.strip():
                    continue
                if ':' in line and not line.startswith('/dev/'):
                    # Device name line
                    current_dev = line.split(':')[0].strip()
                elif line.startswith('/dev/video'):
                    # Video device path
                    dev_path = line.strip()
                    if os.path.exists(dev_path):
                        devices.append(dev_path)
                        device_names[dev_path] = current_dev
            
            # rospy.loginfo(f"All video devices: {devices}")
            # rospy.loginfo(f"name map: {device_names}")
            
            if self.usb_port and self.usb_port != "":
                # rospy.loginfo(f"Filter by USB port: {self.usb_port}")
                
                # USB index from path
                usb_number = None
                try:
                    if 'ttyUSB' in self.usb_port:
                        usb_number = int(self.usb_port.replace('/dev/ttyUSB', ''))
                    elif 'ttyACM' in self.usb_port:
                        usb_number = int(self.usb_port.replace('/dev/ttyACM', ''))
                except:
                    pass
                
                filtered_devices = []
                
                for dev in devices:
                    # Match 1: device name
                    device_name = device_names.get(dev, '')
                    
                    if self.usb_port in device_name or device_name in self.usb_port:
                        filtered_devices.append(dev)
                        # rospy.loginfo(f"Matched by name: {dev} ({device_name})")
                        continue
                    
                    # Match 2: numeric index
                    if usb_number is not None:
                        try:
                            video_number = int(dev.replace('/dev/video', ''))
                            if video_number == usb_number or video_number == usb_number + 1 or video_number == usb_number - 1:
                                filtered_devices.append(dev)
                                # rospy.loginfo(f"Matched by index: {dev} usb={usb_number} video={video_number}")
                                continue
                        except:
                            pass
                    
                    # Match 3: same USB bus via udev
                    try:
                        udev_cmd = ['udevadm', 'info', '-q', 'path', '-n', dev]
                        udev_result = subprocess.run(udev_cmd, capture_output=True, text=True)
                        udev_path = udev_result.stdout.strip()
                        
                        if udev_path and 'usb' in udev_path:
                            usb_info = udev_path.split('/')
                            for part in usb_info:
                                if 'usb' in part and len(part) > 3:
                                    # Compare USB bus path
                                    usb_cmd = ['udevadm', 'info', '-q', 'path', '-n', self.usb_port]
                                    usb_result = subprocess.run(usb_cmd, capture_output=True, text=True)
                                    usb_path = usb_result.stdout.strip()
                                    
                                    if part in usb_path:
                                        filtered_devices.append(dev)
                                        rospy.loginfo(f"Matched video device by USB bus: {dev}")
                                        break
                    except:
                        pass
                
                if filtered_devices:
                    devices = filtered_devices
                    rospy.loginfo(f"Filtered video devices: {devices}")
                else:
                    pass
            else:
                pass
            
            if len(devices) > self.max_cameras:
                devices = devices[:self.max_cameras]
                # rospy.loginfo(f"Limited devices: {devices}")
            
            return sorted(list(set(devices))) if devices else sorted(glob.glob('/dev/video*'))
        except Exception as e:
            # rospy.logerr(f"Error listing video devices: {e}")
            return sorted(glob.glob('/dev/video*'))

    def _try_reset_device(self, dev_path):
        try:
            udev_info = subprocess.run(
                ['udevadm', 'info', '-q', 'path', '-n', dev_path],
                capture_output=True, text=True
            ).stdout.strip()
            
            if udev_info:
                usb_path = f"/sys{udev_info}/../reset"
                if os.path.exists(usb_path):
                    with open(usb_path, 'w') as f:
                        f.write('1')
                    time.sleep(2)
                    return True
        except:
            pass
        return False

    def _init_camera(self, dev_path, cam_id):
        for attempt in range(3):
            try:
                if not os.path.exists(dev_path):
                    rospy.logwarn(f"Device {dev_path} does not exist")
                    continue

                if attempt > 0:
                    self._try_reset_device(dev_path)
                    os.system(f'sudo chmod 666 {dev_path}')
                    os.system(f'sudo fuser -k {dev_path} 2>/dev/null')

                # Unique camera id
                unique_cam_id = f"{self.node_name}_cam{cam_id}"
                cap = cv2.VideoCapture(dev_path, cv2.CAP_V4L2)
                if not cap.isOpened():
                    # rospy.logwarn('OpenCV cannot open device')
                    return False

                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
                
                # Try requested resolutions
                success = False
                actual_width = 0
                actual_height = 0
                
                for res in self.resolutions:
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, res[0])
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, res[1])
                    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    
                    if actual_width == res[0] and actual_height == res[1]:
                        # rospy.loginfo(f"Camera {unique_cam_id} set to {actual_width}x{actual_height}")
                        success = True
                        break
                    else:
                        # rospy.logwarn(f"Camera {unique_cam_id}: wanted {res[0]}x{res[1]}, got {actual_width}x{actual_height}")
                        pass
                
                if not success:
                    # rospy.logwarn(f"Camera {unique_cam_id}: using default {actual_width}x{actual_height}")
                    pass
                
                for _ in range(5):
                    cap.grab()
                    time.sleep(0.01)

                # Refresh actual resolution
                actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                
                # Unique OpenCV window name
                window_name = f'{self.node_name}_{cam_id}_{actual_width}x{actual_height}'
                
                self.cameras.append({
                    'id': cam_id,
                    'unique_id': unique_cam_id,
                    'cap': cap,
                    'dev': dev_path,
                    'frame_count': 0,
                    'width': actual_width,
                    'height': actual_height,
                    'window_name': window_name
                })
                return True

            except Exception as e:
                rospy.logerr(f"Attempt #{attempt+1} init {dev_path} failed: {str(e)}")
                if 'cap' in locals() and cap.isOpened():
                    cap.release()
                time.sleep(1)
        return False

    def _init_cameras(self):
        devices = self._get_physical_devices()
        # rospy.loginfo(f"Video devices: {devices}")
        # Init slots up to camera_count
        if self.max_cameras >= 1:
            self._init_main_or_second_camera(self.video0_main, self.video0_sec, 0)
        if self.max_cameras >= 2:
            self._init_main_or_second_camera(self.video1_main, self.video1_sec, 1)
        if self.max_cameras >= 3:
            self._init_main_or_second_camera(self.video2_main, self.video2_sec, 2)


        if not self.cameras:
            # rospy.logerr("No camera available")
            # rospy.loginfo("Diagnostics:")
            # rospy.loginfo("1. ls /dev/video*")
            # rospy.loginfo("2. v4l2-ctl --list-devices")
            # rospy.loginfo("3. sudo chmod 666 /dev/video*")
            # rospy.loginfo("4. reset USB if needed")
            exit(1)

    def _init_main_or_second_camera(self, dev_main, dev_second, index):
        if self._init_camera(dev_main, index):
            # rospy.loginfo(f"Initialized {dev_main} as {self.cameras[-1]['unique_id']}")
            pass
        else:
            # rospy.logwarn(f"Skip device {dev_main}")
            pass

        if self._init_camera(dev_second, index):
            # rospy.loginfo(f"Initialized {dev_second} as {self.cameras[-1]['unique_id']}")
            pass
        else:
            # rospy.logwarn(f"Skip device {dev_second}")
            pass

    def _init_ros_publishers(self):
        """Create ROS image publishers."""
        for cam in self.cameras:
            # One topic per camera
            if self.topic_base:
                if cam['id'] == 0:
                    topic_name = f'{self.topic_base}/color/image_raw'
                else:
                    topic_name = f'{self.topic_base}_{cam["id"]}/color/image_raw'
            else:
                # Legacy topic names
                if cam['id'] == 0:
                    topic_name = '/camera_fisheye/color/image_raw'
                else:
                    topic_name = f'/camera_fisheye/color/image_raw_{cam["id"] + 1}'
            
            publisher = rospy.Publisher(topic_name, Image, queue_size=10)
            self.image_publishers.append({
                'publisher': publisher,
                'cam_id': cam['id'],
                'unique_id': cam['unique_id'],
                'topic_name': topic_name
            })
            # rospy.loginfo(f"Camera {cam['unique_id']} -> {topic_name}")

    def _publish_frame(self, cam_id, frame, timestamp_ns):
        """Publish one frame to ROS Image topic."""
        try:
            if frame is None or frame.size == 0:
                # rospy.logwarn(f"Camera {cam_id}: empty frame")
                return
            
            if len(frame.shape) != 3 or frame.shape[2] != 3:
                # rospy.logwarn(f"Camera {cam_id}: bad shape {frame.shape}")
                return
                
            ros_image = Image()
            
            ros_image.header.stamp = rospy.Time.from_sec(timestamp_ns / 1e9)
            
            # frame_id
            for cam in self.cameras:
                if cam['id'] == cam_id:
                    ros_image.header.frame_id = cam['unique_id']
                    break
            
            height, width = frame.shape[:2]
            ros_image.height = height
            ros_image.width = width
            
            ros_image.encoding = 'bgr8'
            ros_image.step = width * 3
            ros_image.is_bigendian = 0
            
            if not frame.flags['C_CONTIGUOUS']:
                frame = np.ascontiguousarray(frame)
            
            ros_image.data = frame.tobytes()
            
            for pub_info in self.image_publishers:
                if pub_info['cam_id'] == cam_id:
                    pub_info['publisher'].publish(ros_image)
                    rospy.logdebug_once(f"Published camera {cam_id} {width}x{height}")
                    break
                    
        except Exception as e:
            # rospy.logerr(f"Publish camera {cam_id} failed: {str(e)}")
            pass

    def _display_frames(self, frames_data):
        """Show preview windows."""
        for cam, frame in frames_data:
            if frame is not None:
                # Overlay timestamp and frame count
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                info_text = f"{cam['unique_id']} | {timestamp} | Frames: {cam['frame_count']}"
                cv2.putText(frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                          0.7, (0, 255, 0), 2)
                
                cv2.imshow(cam['window_name'], frame)
        
        # ESC to quit preview
        if cv2.waitKey(1) == 27:
            self.running = False

    def capture_frames(self):
        # rospy.loginfo(f"Capturing from {len(self.cameras)} camera(s)...")
        # rospy.loginfo("ESC or Ctrl+C to stop")
        
        # Preview windows
        if self.show_preview:
            for cam in self.cameras:
                RESIZE_WIDTH = 640
                RESIZE_HEIGHT = 480
                # Unique OpenCV window name
                cv2.namedWindow(cam['window_name'], cv2.WINDOW_NORMAL)
                cv2.resizeWindow(cam['window_name'], RESIZE_WIDTH, RESIZE_HEIGHT)
        
        frame_num = 0
        rate = rospy.Rate(30)  # 30Hz
        
        try:
            while self.running and not rospy.is_shutdown():
                frames_data = []
                timestamp_ns = time.time_ns()
                
                for cam in self.cameras:
                    ret, frame = cam['cap'].read()
                    if not ret:
                        # rospy.logwarn(f"Camera {cam['unique_id']} frame {frame_num} grab failed")
                        frame = None
                    else:
                        # ROS publish
                        self._publish_frame(cam['id'], frame, timestamp_ns)
                        cam['frame_count'] += 1
                    
                    frames_data.append((cam, frame))
                
                if self.show_preview:
                    self._display_frames(frames_data)
                
                # if frame_num % 30 == 0:
                #     status = " | ".join([f"{cam['unique_id']}:{cam['frame_count']}f" for cam in self.cameras])
                #     rospy.loginfo(f"frame {frame_num+1} | {status}")
                
                frame_num += 1
                rate.sleep()
                
        except Exception as e:
            rospy.logerr(f"Capture error: {e}")
        finally:
            self._release_resources()
            # self._generate_report()

    def _release_resources(self):
        """Release cameras and windows."""
        for cam in self.cameras:
            try:
                cam['cap'].release()
            except:
                pass
        
        # Close this node's preview windows
        if self.show_preview:
            for cam in self.cameras:
                try:
                    cv2.destroyWindow(cam['window_name'])
                except:
                    pass

    def _generate_report(self):
        """Log capture summary."""
        rospy.loginfo("\n=== Capture report ===")
        for cam in self.cameras:
            rospy.loginfo(f"Camera {cam['unique_id']}[{cam['dev']}]")
            rospy.loginfo(f"  resolution: {cam['width']}x{cam['height']}")
            rospy.loginfo(f"  frames published: {cam['frame_count']}")
            for pub_info in self.image_publishers:
                if pub_info['cam_id'] == cam['id']:
                    rospy.loginfo(f"  ROS Topic: {pub_info['topic_name']}")
                    break

if __name__ == "__main__":
    try:
        os.nice(-20)
    except:
        pass

    cv2.setNumThreads(1)
    cv2.setUseOptimized(True)

    # Strip ROS-injected argv
    import argparse
    
    ros_args = ['__name:=', '__log:=', '__master:=', '__ip:=']
    filtered_args = []
    
    for arg in sys.argv[1:]:
        if not any(arg.startswith(ros_arg) for ros_arg in ros_args):
            filtered_args.append(arg)
    
    sys.argv = [sys.argv[0]] + filtered_args
    
    # Debug CLI (optional)
    parser = argparse.ArgumentParser(description="Camera capture script with ROS")
    parser.add_argument("--no-preview", dest="show_preview", action="store_false",
                       help="Disable preview window")
    parser.add_argument("--usb-port", type=str, default="",
                       help="USB port for filtering video devices")
    parser.add_argument("--video_0_main", type=str, default="",
                       help="center video devices")
    parser.add_argument("--video_1_main", type=str, default="",
                       help="video devices")
    parser.add_argument("--video_2_main", type=str, default="",
                       help="video devices")
    
    parser.add_argument("--video_0_sec", type=str, default="",
                       help="center video devices")
    parser.add_argument("--video_1_sec", type=str, default="",
                       help="left video devices")
    parser.add_argument("--video_2_sec", type=str, default="",
                       help="video devices")
    parser.set_defaults(show_preview=True)
    args = parser.parse_args()
    
    try:
        recorder = CameraCaptureROS()
        recorder.capture_frames()
    except rospy.ROSInterruptException:
        print("ROS interrupted")
    except Exception as e:
        print(f"Fatal error: {str(e)}")
        print("\nRecovery hints:")
        print("1. Power-cycle the camera")
        print("2. sudo rmmod uvcvideo && sudo modprobe uvcvideo")
        print("3. Try another USB port")