#!/usr/bin/env python3
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##    RealSense Color Video Recorder         ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
import time
import datetime
import os
import argparse

class RealSenseVideoRecorder:
    def __init__(self, width=1280, height=720, fps=15, output_fps=15):
        """
        Initialize RealSense video recorder
        
        Args:
            width: Image width
            height: Image height  
            fps: RealSense capture fps
            output_fps: Output video fps
        """
        self.width = width
        self.height = height
        self.fps = fps
        self.output_fps = output_fps
        self.pipeline = None
        self.video_writer = None
        self.frame_count = 0
        
        # Create output directory
        self.output_dir = "./data/realsense_videos"
        os.makedirs(self.output_dir, exist_ok=True)
        
    def initialize_camera(self):
        """Initialize RealSense camera"""
        try:
            # Configure depth and color streams
            self.pipeline = rs.pipeline()
            config = rs.config()
            
            # Get device info
            pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
            pipeline_profile = config.resolve(pipeline_wrapper)
            device = pipeline_profile.get_device()
            device_product_line = str(device.get_info(rs.camera_info.product_line))
            
            print(f"Device: {device_product_line}")
            
            # Check for RGB camera
            found_rgb = False
            for s in device.sensors:
                if s.get_info(rs.camera_info.name) == 'RGB Camera':
                    found_rgb = True
                    break
            
            if not found_rgb:
                raise RuntimeError("RGB Camera not found")
            
            # Enable color stream
            config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
            
            # Start streaming
            print(f"Starting camera: {self.width}x{self.height} @ {self.fps}fps")
            self.pipeline.start(config)
            
            # Let camera warm up
            for _ in range(10):
                frames = self.pipeline.wait_for_frames()
                
            print("Camera initialized successfully")
            return True
            
        except Exception as e:
            print(f"Failed to initialize camera: {e}")
            return False
    
    def initialize_video_writer(self, filename):
        """Initialize video writer"""
        try:
            # Video codec
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            
            # Create video writer
            self.video_writer = cv2.VideoWriter(
                filename, fourcc, self.output_fps, (self.width, self.height)
            )
            
            if not self.video_writer.isOpened():
                raise RuntimeError("Failed to open video writer")
            
            print(f"Video writer initialized: {filename}")
            print(f"Output format: {self.width}x{self.height} @ {self.output_fps}fps")
            return True
            
        except Exception as e:
            print(f"Failed to initialize video writer: {e}")
            return False
    
    def record_video(self, duration_seconds=None, max_frames=None):
        """
        Record video for specified duration or frame count
        
        Args:
            duration_seconds: Recording duration in seconds (None for manual stop)
            max_frames: Maximum number of frames (None for no limit)
        """
        # Generate filename with timestamp
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(self.output_dir, f"realsense_color_{timestamp}.mp4")
        
        if not self.initialize_video_writer(filename):
            return False
        
        print("\n=== Recording Started ===")
        print("Press 'q' to stop recording")
        if duration_seconds:
            print(f"Auto-stop after {duration_seconds} seconds")
        if max_frames:
            print(f"Auto-stop after {max_frames} frames")
        
        start_time = time.time()
        last_log_time = start_time
        frame_interval = 1.0 / self.output_fps  # Target interval between frames
        next_frame_time = start_time
        
        try:
            while True:
                current_time = time.time()
                
                # Check stopping conditions
                if duration_seconds and (current_time - start_time) >= duration_seconds:
                    print(f"\nRecording completed: {duration_seconds} seconds reached")
                    break
                
                if max_frames and self.frame_count >= max_frames:
                    print(f"\nRecording completed: {max_frames} frames reached")
                    break
                
                # Check if it's time for next frame (fixed frequency)
                if current_time >= next_frame_time:
                    # Get frames from RealSense
                    frames = self.pipeline.wait_for_frames()
                    color_frame = frames.get_color_frame()
                    
                    if not color_frame:
                        continue
                    
                    # Convert to numpy array
                    color_image = np.asanyarray(color_frame.get_data())
                    
                    # Write frame to video
                    self.video_writer.write(color_image)
                    self.frame_count += 1
                    
                    # Update next frame time
                    next_frame_time += frame_interval
                    
                    # Display preview (optional)
                    cv2.imshow('RealSense Recording', color_image)
                    
                    # Log progress every second
                    if current_time - last_log_time >= 1.0:
                        elapsed = current_time - start_time
                        actual_fps = self.frame_count / elapsed if elapsed > 0 else 0
                        print(f"Recording: {self.frame_count} frames, "
                              f"Elapsed: {elapsed:.1f}s, "
                              f"Actual FPS: {actual_fps:.1f}")
                        last_log_time = current_time
                
                # Check for quit key
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("\nRecording stopped by user")
                    break
                
                # Small sleep to prevent busy waiting
                time.sleep(0.001)
                
        except KeyboardInterrupt:
            print("\nRecording interrupted by user")
        
        except Exception as e:
            print(f"\nRecording error: {e}")
            return False
        
        finally:
            # Cleanup
            if self.video_writer:
                self.video_writer.release()
            cv2.destroyAllWindows()
            
            # Final statistics
            total_time = time.time() - start_time
            avg_fps = self.frame_count / total_time if total_time > 0 else 0
            
            print(f"\n=== Recording Summary ===")
            print(f"Output file: {filename}")
            print(f"Total frames: {self.frame_count}")
            print(f"Total time: {total_time:.2f} seconds")
            print(f"Average FPS: {avg_fps:.2f}")
            print(f"Target FPS: {self.output_fps}")
            print(f"File size: {os.path.getsize(filename) / (1024*1024):.2f} MB")
        
        return True
    
    def cleanup(self):
        """Cleanup resources"""
        if self.pipeline:
            self.pipeline.stop()
        if self.video_writer:
            self.video_writer.release()
        cv2.destroyAllWindows()

def main():
    parser = argparse.ArgumentParser(description='RealSense Color Video Recorder')
    parser.add_argument('--width', type=int, default=1280, help='Image width (default: 1280)')
    parser.add_argument('--height', type=int, default=720, help='Image height (default: 720)')
    parser.add_argument('--fps', type=int, default=15, help='Camera FPS (default: 15)')
    parser.add_argument('--output-fps', type=int, default=15, help='Output video FPS (default: 15)')
    parser.add_argument('--duration', type=int, default=None, help='Recording duration in seconds')
    parser.add_argument('--frames', type=int, default=None, help='Maximum number of frames')
    
    args = parser.parse_args()
    
    # Create recorder
    recorder = RealSenseVideoRecorder(
        width=args.width,
        height=args.height, 
        fps=args.fps,
        output_fps=args.output_fps
    )
    
    try:
        # Initialize camera
        if not recorder.initialize_camera():
            print("Failed to initialize camera")
            return 1
        
        # Start recording
        success = recorder.record_video(
            duration_seconds=args.duration,
            max_frames=args.frames
        )
        
        if success:
            print("Recording completed successfully")
            return 0
        else:
            print("Recording failed")
            return 1
            
    except Exception as e:
        print(f"Error: {e}")
        return 1
        
    finally:
        recorder.cleanup()

if __name__ == "__main__":
    exit(main())
