import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from camera_to_opensim_interfaces.msg import PersonArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from datetime import datetime
from std_srvs.srv import Trigger
# from image_geometry import PinholeCameraModel # Requires python3-image-geometry

class DataProcessingNode(Node):
    def __init__(self):
        super().__init__('data_processing_node')
        
        # Parameters
        self.declare_parameter('output_dir', os.path.expanduser('~/opensim_data'))
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.bridge = CvBridge()
        self.camera_model = None
        self.recorded_data = [] # List of dicts
        self.recording = False

        # Subscribers
        self.keypoints_sub = message_filters.Subscriber(self, PersonArray, '/body_tracking/keypoints_2d')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/aligned_depth_to_color/image_raw')
        
        # Synchronizer (Approximate Time)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.keypoints_sub, self.depth_sub], 10, 0.1)
        self.ts.registerCallback(self.sync_callback)
        
        # Camera Info Subscriber (One shot or continuous)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/aligned_depth_to_color/camera_info', self.info_callback, 10)

        # Services
        self.start_srv = self.create_service(Trigger, 'start_recording', self.start_recording)
        self.stop_srv = self.create_service(Trigger, 'stop_recording', self.stop_recording)
        
        self.get_logger().info('Data Processing Node Initialized. Waiting for CameraInfo...')

    def info_callback(self, msg):
        if self.camera_model is None:
            # Simple Intrinsic Matrix extraction since image_geometry might not be available or setup
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.camera_model = True
            self.get_logger().info(f'Camera Info Received. fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}')
            # We can unregister if we assume constant intrinsics, but keeping it for now is safe

    def start_recording(self, request, response):
        self.recording = True
        self.recorded_data = []
        response.success = True
        response.message = "Recording started."
        self.get_logger().info("Recording started.")
        return response

    def stop_recording(self, request, response):
        self.recording = False
        if not self.recorded_data:
            response.success = False
            response.message = "No data recorded."
            return response
            
        filename = self.save_trc()
        self.get_logger().info(f"Recording stopped and saved to {filename}")

        # IK Step
        try:
            # We assume pose2sim path is provided via parameter if not in PYTHONPATH
            # Or assume run_ik handles it based on environment
            from .pose2sim_wrapper import run_ik
            # run_ik expects output_dir (where 'pose-3d' will be)
            output_dir_parent = os.path.dirname(self.output_dir) # If structure is output_dir/pose-3d, we pass output_dir?
            # data_processing_node writes to self.output_dir/recording...trc
            # Pose2Sim expects project_dir/pose-3d/file.trc
            # So if self.output_dir is 'opensim_data/pose-3d', project_dir is 'opensim_data'
            # Let's adjust self.output_dir logic
            
            project_dir = self.output_dir # Assuming output_dir is the PROJECT directory
            # save_trc should create pose-3d subdir
            
            run_ik(project_dir, use_simple_model=True)
            response.message = f"Recording saved to {filename} and IK run."
            self.get_logger().info(f"IK run successfully.")
        except Exception as e:
            response.message = f"Saved {filename}, but IK failed: {e}"
            self.get_logger().error(f"IK failed: {e}")
            
        response.success = True
        return response

    def sync_callback(self, kps_msg, depth_msg):
        if not self.recording or self.camera_model is None:
            return
            
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1') # mm
        except Exception as e:
            self.get_logger().error(f'CV Bridge failed: {e}')
            return

        timestamp = kps_msg.header.stamp.sec + kps_msg.header.stamp.nanosec * 1e-9
        
        # Process first person for now (Single Subject assumption for TRC conversion simplicity)
        if len(kps_msg.persons) > 0:
            person = kps_msg.persons[0]
            
            frame_data = {'Time': timestamp}
            
            for kp in person.keypoints:
                u, v = int(kp.x), int(kp.y)
                
                # Boundary check
                if 0 <= u < cv_depth.shape[1] and 0 <= v < cv_depth.shape[0]:
                    depth_val = cv_depth[v, u] # mm
                    
                    if depth_val > 0:
                        z = depth_val / 1000.0 # meters
                        x = (u - self.cx) * z / self.fx
                        y = (v - self.cy) * z / self.fy
                        
                        # OpenSim Coordinate System Conversion might be needed here
                        # Typically OpenSim Y is UP. RealSense Y is DOWN.
                        # RealSense: X Right, Y Down, Z Forward
                        # OpenSim: X Forward, Y Up, Z Right (Model dependent, but usually Y is up)
                        # Let's keep camera frame for TRC and handle rotation in IK tool or here.
                        # Standard TRC is usually Y-up.
                        # Rot: X_trc = Z_cam, Y_trc = -Y_cam, Z_trc = X_cam ??? 
                        # This depends on the OpenSim model. 
                        # Let's just raw log first.
                        
                        frame_data[f'{kp.name}_X'] = x
                        frame_data[f'{kp.name}_Y'] = y
                        frame_data[f'{kp.name}_Z'] = z
                    else:
                        frame_data[f'{kp.name}_X'] = np.nan
                        frame_data[f'{kp.name}_Y'] = np.nan
                        frame_data[f'{kp.name}_Z'] = np.nan
                else:
                    frame_data[f'{kp.name}_X'] = np.nan
                    frame_data[f'{kp.name}_Y'] = np.nan
                    frame_data[f'{kp.name}_Z'] = np.nan
            
            self.recorded_data.append(frame_data)

    def save_trc(self):
        if not self.recorded_data:
            return None
            
        # Ensure project structure: project/pose-3d
        pose3d_dir = os.path.join(self.output_dir, 'pose-3d')
        os.makedirs(pose3d_dir, exist_ok=True)
        
        # Simple TRC Header generation
        path = os.path.join(pose3d_dir, f'recording_{datetime.now().strftime("%Y%m%d_%H%M%S")}.trc')
        
        # Metadata
        data_rate = 30.0 # Estimate or calculate
        camera_rate = 30.0
        num_frames = len(self.recorded_data)
        
        # Extract marker names from the first frame or accumulate all seen? 
        # Assuming all frames have same markers for now based on logic
        if num_frames == 0:
            return None
            
        # Get keys that end with _X
        first_frame = self.recorded_data[0]
        marker_names = [key[:-2] for key in first_frame.keys() if key.endswith('_X')]
        marker_names.sort() # Ensure deterministic order
        
        num_markers = len(marker_names)
        
        with open(path, 'w') as f:
            f.write(f"PathFileType\t4\t(X/Y/Z)\t{os.path.basename(path)}\n")
            f.write(f"DataRate\tCameraRate\tNumFrames\tNumMarkers\tUnits\tOrigDataRate\tOrigDataStartFrame\tOrigNumFrames\n")
            f.write(f"{data_rate}\t{camera_rate}\t{num_frames}\t{num_markers}\tm\t{data_rate}\t1\t{num_frames}\n")
            
            # Marker Names Header
            f.write("Frame#\tTime\t" + "\t".join([f"{name}\t\t" for name in marker_names]) + "\n")
            f.write("\t\t" + "\t".join([f"X{i+1}\tY{i+1}\tZ{i+1}" for i in range(num_markers)]) + "\n")
            
            # Data
            for idx, row in enumerate(self.recorded_data):
                line = [str(idx+1), f"{row['Time']:.4f}"]
                for name in marker_names:
                    x = row.get(f'{name}_X', np.nan)
                    y = row.get(f'{name}_Y', np.nan)
                    z = row.get(f'{name}_Z', np.nan)
                    # Handle NaNs
                    line.append(f"{x:.5f}" if not np.isnan(x) else "")
                    line.append(f"{y:.5f}" if not np.isnan(y) else "")
                    line.append(f"{z:.5f}" if not np.isnan(z) else "")
                f.write("\t".join(line) + "\n")
                
        return path

def main(args=None):
    rclpy.init(args=args)
    node = DataProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
