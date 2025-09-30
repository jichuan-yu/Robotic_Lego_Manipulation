#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import os
import datetime
import threading
import yaml
from std_msgs.msg import String

class ROSVideoRecorder:
    def __init__(self, config_file=None):
        rospy.init_node('lego_video_recorder', anonymous=True)
        
        self.bridge = CvBridge()
        self.recording = False
        self.video_writers = {} # VideoWriter objects for each camera
        self.latest_frames = {}
        self.frame_counts = {}
        self.last_received_time = {} # Track last received time for each camera
        self.lock = threading.Lock()
        
        self.config = self._default_config()
        
        # Create output directory
        self.output_dir = self.config['output_dir']
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Initialize subscribers
        self._setup_subscribers()
        
        
        rospy.loginfo("LEGO video recorder started")
        rospy.loginfo(f"Output directory: {self.output_dir}")
        
    def _default_config(self):
        """Default configuration"""
        return {
            'output_dir': os.path.expanduser('./data/'),
            'cameras': {
                'r1_color': {
                    'topic': '/r1/wrist_camera/color/image_raw',
                    'fps': 15.0,
                    'codec': 'mp4v',
                    'resize': None # [width, height]
                },
                'r2_color': {
                    'topic': '/r2/wrist_camera/color/image_raw',
                    'fps': 15.0,
                    'codec': 'mp4v',
                    'resize': None # [width, height]
                },
                'fixed_camera': {
                    'topic': '/camera1/fixed_camera/image_raw',
                    'fps': 15.0,
                    'codec': 'mp4v',
                    'resize': None # [width, height]
                }
            }
        }
    
    def _setup_subscribers(self):
        """Setup ROS subscribers"""
        self.subscribers = {}
        for camera_name, camera_config in self.config['cameras'].items():
            topic = camera_config['topic']

            sub = rospy.Subscriber(
                topic, Image,
                lambda msg, name=camera_name: self._image_callback(msg, name),
                queue_size=10  # Increased queue size to prevent message loss
            )
            
            self.subscribers[camera_name] = sub
            self.latest_frames[camera_name] = None
            self.frame_counts[camera_name] = 0

            rospy.loginfo(f"Subscribing to: {camera_name} -> {topic}")
            
        # Initialize last received time tracking
        for camera_name in self.config['cameras'].keys():
            self.last_received_time[camera_name] = 0.0

    def _image_callback(self, msg, camera_name):
        """Handle Image messages - optimized for recording performance"""
        try:
            # Update last received time
            self.last_received_time[camera_name] = rospy.get_time()
            
            # Color image processing
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # For better performance: record first, then update frames
            if self.recording and camera_name in self.video_writers:
                self._write_frame(camera_name, cv_image)
            
            # Only update latest_frames when needed (for recording initialization or occasionally)
            if (not self.recording and self.latest_frames[camera_name] is None) or \
               (self.recording and self.frame_counts[camera_name] % 60 == 0):
                # Use non-blocking lock to avoid delays
                if self.lock.acquire(blocking=False):
                    try:
                        self.latest_frames[camera_name] = cv_image.copy()
                    finally:
                        self.lock.release()
                    
        except Exception as e:
            rospy.logerr(f"Image processing error {camera_name}: {e}")
            import traceback
            rospy.logerr(f"Traceback: {traceback.format_exc()}")
    
    
    def _write_frame(self, camera_name, frame):
        """Write video frame"""
        if camera_name not in self.video_writers:
            return
            
        camera_config = self.config['cameras'][camera_name]
        
        # Resize if needed
        if camera_config.get('resize'):
            width, height = camera_config['resize']
            frame = cv2.resize(frame, (width, height))
        
        self.video_writers[camera_name].write(frame)
        self.frame_counts[camera_name] += 1
        
        # Log every 30 frames (roughly every second at 30fps)
        if self.frame_counts[camera_name] % 30 == 0:
            rospy.loginfo(f"{camera_name}: {self.frame_counts[camera_name]} frames recorded")
    

    def start_recording(self, session_name=None):
        """Start recording"""
        if self.recording:
            rospy.logwarn("Already recording...")
            return False
        
        # Generate session name
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        if session_name:
            session_dir = f"{session_name}_{timestamp}"
        else:
            session_dir = f"lego_recording_{timestamp}"
        
        full_session_dir = os.path.join(self.output_dir, session_dir)
        os.makedirs(full_session_dir, exist_ok=True)
        
        # Create video writers for each camera
        for camera_name, camera_config in self.config['cameras'].items():
            if camera_name not in self.latest_frames or self.latest_frames[camera_name] is None:
                rospy.logwarn(f"Camera {camera_name} has no image data, skipping recording")
                continue
            
            frame = self.latest_frames[camera_name]
            
            # Get video parameters
            if camera_config.get('resize'):
                width, height = camera_config['resize']
            else:
                height, width = frame.shape[:2]
            
            fps = camera_config.get('fps', 30.0)
            codec = camera_config.get('codec', 'mp4v')
            
            # Create video file
            video_filename = f"{camera_name}.mp4"
            video_path = os.path.join(full_session_dir, video_filename)
            
            fourcc = cv2.VideoWriter_fourcc(*codec)
            video_writer = cv2.VideoWriter(video_path, fourcc, fps, (width, height))
            
            if not video_writer.isOpened():
                rospy.logerr(f"Unable to create video file: {video_path}")
                continue
            
            self.video_writers[camera_name] = video_writer
            self.frame_counts[camera_name] = 0
            
            rospy.loginfo(f"Started recording {camera_name}: {video_path}")
        
        self.recording = True
        self.session_dir = full_session_dir
        
        # Save configuration file
        config_path = os.path.join(full_session_dir, "recording_config.yaml")
        with open(config_path, 'w') as f:
            yaml.dump(self.config, f, default_flow_style=False)
        
        rospy.loginfo(f"Recording session started: {session_dir}")
        return True
    
    def stop_recording(self):
        """Stop recording"""
        if not self.recording:
            rospy.logwarn("Currently not recording")
            return False
        
        self.recording = False
        
        # Release all video writers
        for camera_name, writer in self.video_writers.items():
            writer.release()
            frame_count = self.frame_counts[camera_name]
            rospy.loginfo(f"Camera {camera_name} recording completed: {frame_count} frames")
        
        self.video_writers.clear()
        
        # Reset frame counts
        for camera_name in self.config['cameras'].keys():
            self.frame_counts[camera_name] = 0
        
        rospy.loginfo(f"Recording session completed: {self.session_dir}")
        return True
    
    def preview_cameras(self):
        """Show camera preview"""
        rospy.loginfo("Showing camera preview (press 'q' to exit, 'r' to start/stop recording)")
        
        while not rospy.is_shutdown():
            with self.lock:
                for camera_name, frame in self.latest_frames.items():
                    if frame is not None:
                        # Add status information
                        display_frame = frame.copy()
                        
                        # Recording status
                        status_text = "REC" if self.recording else "STANDBY"
                        color = (0, 0, 255) if self.recording else (0, 255, 0)
                        cv2.putText(display_frame, status_text, (10, 30),
                                  cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
                        
                        # Frame count
                        if self.recording:
                            frame_count = self.frame_counts.get(camera_name, 0)
                            cv2.putText(display_frame, f"Frames: {frame_count}",
                                      (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                        
                        # Timestamp
                        timestamp = rospy.get_time()
                        cv2.putText(display_frame, f"Time: {timestamp:.2f}",
                                  (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                        
                        cv2.imshow(f"LEGO_{camera_name}", display_frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):
                if self.recording:
                    self.stop_recording()
                else:
                    self.start_recording("lego_record_sim")

        cv2.destroyAllWindows()
    

def main():
    try:
        # Create recorder with default configuration
        recorder = ROSVideoRecorder()
        
        # Wait a bit for topics to be available
        rospy.sleep(3)
        
        # Start preview mode (press 'r' to start/stop recording)
        recorder.preview_cameras()
            
    except KeyboardInterrupt:
        rospy.loginfo("Received interrupt signal, stopping recording...")
    except Exception as e:
        rospy.logerr(f"Recorder error: {e}")
    finally:
        if 'recorder' in locals() and recorder.recording:
            recorder.stop_recording()

if __name__ == '__main__':
    main()