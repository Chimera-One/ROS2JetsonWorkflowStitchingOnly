#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSPresetProfiles, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.context import Context

#Executors
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

#Msgs 
from std_msgs.msg import String
from sensor_msgs.msg import Image
from custom_interfaces.msg import StitchData, StitchedData

import os, glob
import re
import yaml
from lightglue import LightGlue, SuperPoint, ALIKED
from lightglue.utils import rbd

import cv2
import numpy as np

import torch
from concurrent.futures import ThreadPoolExecutor
from torchvision import transforms

#Images 
import cv2
from cv_bridge import CvBridge

#Threads
import threading
import time

#Resetting
import subprocess

#Math
import math

#Transforms
from transforms3d import _gohlketransforms

#Garbage Collector
import gc


INCLUDE_DIR = "/home/chimera/ros2_ws/src/chimera_stitching/include"
SCRIPT_PATH = "/home/chimera/scripts/mission_reset.sh"
CONFIG_PATH = os.path.join(INCLUDE_DIR, "config.yaml")

R_EARTH = 6378137.0  # meters
CALIBRATED_FOCAL_LENGTH = 3725.151611


class StitchingNode(Node):


    def __init__(self):
        super().__init__('stitching')

        self.get_logger().info("Hello World")


        self.is_dir_reset = True


        self.is_stitch_ready = False
        self.is_stitch_done = False

        self.to_be_pose_filtered = []

        self.iteration = 1

        # A vector of the orientation of the mission
        self.orientation_vector = np.array([[0, 0], [0, 0]], dtype=np.float32)
        self.stitched_orientation_vector = np.array([[0, 0], [0, 0]], dtype=np.float32)
        self.orientation_angle_between_original_images_and_stitched_images = 0.0

        self.gps_lat = []
        self.gps_lon = []

        self.bridge = CvBridge()

        qos_settings = QoSProfile(history=QoSHistoryPolicy.KEEP_ALL, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.stitch_signaller = self.create_subscription(StitchData, 'mission_completion', self.stitch_signal, qos_profile=qos_settings)
        self.pose_filter_subscription = self.create_subscription(String, 'pose_filter', self.pose_filter, qos_profile=qos_settings)
        self.stitched_rgb = self.create_publisher(StitchedData, 'stitched_rgb', qos_profile=qos_settings)
        self.stitched_mask = self.create_publisher(StitchedData, 'stitched_mask', qos_profile=qos_settings)
        self.stitched_heatmap = self.create_publisher(StitchedData, 'stitched_heatmap', qos_profile=qos_settings)

        startup_thread = threading.Thread(target=self.waiting_for_startup)
        startup_thread.start()

        # List of GPSs for StitchData
        self.image_data = None

        self.extractor = None

        #@TODO: Need to destroy threads in stop sequence


    def pose_filter(self, msg):
        self.get_logger().info(f"Pose filtering {msg.data}")
        self.to_be_pose_filtered.append(msg.data)


    def stitch_signal(self, msg):
        self.get_logger().info("Received stitching msg")

        if msg.data != "MISSION FINISHED":
            return

        self.image_data = {}

        for name, lat, lon, alt in zip(
            msg.names,
            msg.gps_latitude,
            msg.gps_longitude,
            msg.gps_altitude
        ):
            self.image_data[name] = {
                "lat": lat,
                "lon": lon,
                "alt": alt,
                "image_path": None,
                "mask_path": None,
                "heatmap_path": None
            }


        self.get_logger().info(f"Stitch request: {self.image_data}")

        self.is_stitch_ready = True




    def waiting_for_startup(self):

        try: 

            while not self.is_stitch_done:


                if self.is_stitch_ready:

                    self.original_images = []
                    self.original_sizes = []
                    self.feats_list = []
                    self.mask_images = []
                    self.heatmap_images = []

                    self.config = self.load_config(CONFIG_PATH)
                    # self.image_paths, self.mask_paths, self.heatmap_paths = self.get_image_paths(self.config)

                    if self.extractor is None:
                        self.get_logger().info(f"Loading {self.config['extractor']}...")
                        if self.config['extractor'] == 'superpoint':
                            self.extractor = SuperPoint(max_num_keypoints=2048).eval().to(self.config["device"])

                    print(f"Iteration: {self.iteration}")

                    self.get_image_paths(self.config)
                    self.apply_pose_filtering()
                    self.validate_image_data()

                    # Create ordered lists ONLY for stitching pipeline
                    ordered_items = sorted(self.image_data.items())

                    self.image_paths = [data["image_path"] for _, data in ordered_items]
                    self.mask_paths = [data["mask_path"] for _, data in ordered_items]
                    self.heatmap_paths = [data["heatmap_path"] for _, data in ordered_items]
                    self.image_stitch_paths = []
                    self.gps_lat = [data["lat"] for _, data in ordered_items]
                    self.gps_lon = [data["lon"] for _, data in ordered_items]

                    orientation_vector = np.array([[self.gps_lat[0], self.gps_lon[0], 0], [self.gps_lat[-1], self.gps_lon[-1], 0]], dtype=np.float32)

                    print(f"Orientation vector: {orientation_vector}")
                    print("processing bounding box...")

                    bb_x_min, bb_x_max, bb_y_min, bb_y_max, stitch_orientation = self.process_bounding_box()

                    print(f"Bounding box processed. {bb_x_max - bb_x_min}, {bb_y_max - bb_y_min} bb_x_min: {bb_x_min}, bb_x_max: {bb_x_max}, bb_y_min: {bb_y_min}, bb_y_max: {bb_y_max}")

                    stitch_vec = stitch_orientation[1] - stitch_orientation[0]
                    gps_vec = orientation_vector[1] - orientation_vector[0]

                    self.orientation_angle_between_original_images_and_stitched_images = _gohlketransforms.angle_between_vectors(stitch_vec, gps_vec)
                    print(f"Rotation angle: {self.orientation_angle_between_original_images_and_stitched_images}")
                    print("processing images...")

                    processed_count = 0
                    total_images = len(self.image_paths)

                    # with ThreadPoolExecutor(max_workers=2) as executor:
                    #     results = executor.map(lambda path: self.process_image(path, self.config), self.image_paths)
                    
                    results = []
                    for path in self.image_paths:
                        print(f"Extracting features: {path}")
                        size, multi_feats = self.process_image(path, self.config) # Process one by one
                        results.append((size, multi_feats))
                    
                    for (size, multi_feats), mask_path, heatmap_path in zip(results, self.mask_paths, self.heatmap_paths):
                        processed_count += 1
                        print(f"Processed {processed_count}/{total_images} images.")
                        
                        if multi_feats is not None:
                            # Load the mask as a numpy array
                            # using imread to get the np array (default BGR)
                            # Add cv2.IMREAD_GRAYSCALE if your stitching pipeline expects 1-channel masks
                            mask_np = cv2.imread(mask_path)
                            heatmap_np = cv2.imread(heatmap_path)
                            
                            if mask_np is None:
                                print(f"Warning: Could not load mask at {mask_path}. Skipping corresponding image.")
                                continue

                            if heatmap_np is None:
                                print(f"Warning: Could not load heatmap at {heatmap_path}. Skipping corresponding image.")
                                continue

                            # Only append data if both image and mask are valid
                            self.original_sizes.append(size)
                            self.feats_list.append(multi_feats)
                            self.mask_images.append(mask_np)
                            self.heatmap_images.append(heatmap_np)

                    gc.collect()
                    torch.cuda.empty_cache()

                    print("Matching keypoints and estimating transformations...")
                    transformations, all_corners = self.match_keypoints(self.feats_list, self.original_sizes, self.config)

                    print("Keypoints matched clear memory")
                    self.original_images.clear()
                    self.mask_images.clear()
                    self.heatmap_images.clear()
                    self.feats_list.clear()
                    del self.original_images, self.mask_images, self.heatmap_images, self.feats_list
                    gc.collect()

                    # panorama = self.stitch_images(self.original_images, transformations, all_corners)
                    panorama, panorama_mask, panorama_heatmap = self.stitch_images(transformations, all_corners)
                    print("Stitching Pipeline completed.")

                    panorama_height, panorama_width = panorama.shape[:2]
                    bb_width = int(np.ceil(bb_x_max - bb_x_min))
                    bb_height = int(np.ceil(bb_y_max - bb_y_min))

                    if (panorama_width <= bb_width and panorama_height <= bb_height) or (panorama_height <= bb_width and panorama_width <= bb_height) or self.iteration >= 1:
                        print(f"--- Panorama fits inside the bounding box. Iteration: {self.iteration}")
                        print(f"Panorama size: ({panorama_width}, {panorama_height})")
                        print(f"Bounding box size: ({bb_width}, {bb_height})")
                        panorama_msg = StitchedData()
                        panorama_msg.image = self.bridge.cv2_to_imgmsg(panorama, encoding="bgra8")
                        panorama_msg.image.header.stamp = self.get_clock().now().to_msg()
                        panorama_msg.image.header.frame_id = f"RGB_{self.iteration}"
                        panorama_msg.rotation_degree = float(self.orientation_angle_between_original_images_and_stitched_images)

                        self.stitched_rgb.publish(panorama_msg)

                        panorama_msg = StitchedData()
                        panorama_msg.image = self.bridge.cv2_to_imgmsg(panorama_mask, encoding="bgra8")
                        panorama_msg.image.header.stamp = self.get_clock().now().to_msg()
                        panorama_msg.image.header.frame_id = f"MASK_{self.iteration}"
                        panorama_msg.rotation_degree = float(self.orientation_angle_between_original_images_and_stitched_images)
                        self.stitched_mask.publish(panorama_msg)
                        
                        panorama_msg = StitchedData()
                        panorama_msg.image = self.bridge.cv2_to_imgmsg(panorama_heatmap, encoding="bgra8")
                        panorama_msg.image.header.stamp = self.get_clock().now().to_msg()
                        panorama_msg.image.header.frame_id = f"HEATMAP_{self.iteration}"
                        panorama_msg.rotation_degree = float(self.orientation_angle_between_original_images_and_stitched_images)
                        self.stitched_heatmap.publish(panorama_msg)

                        cv2.imwrite(os.path.join(self.config['output_dir'], self.config['output_filename']) + str(self.iteration) + ".png", panorama)
                        cv2.imwrite(os.path.join(self.config['output_dir'], self.config['output_filename']) + str(self.iteration) + "_mask.png", panorama_mask)
                        cv2.imwrite(os.path.join(self.config['output_dir'], self.config['output_filename']) + str(self.iteration) + "_heatmap.png", panorama_heatmap)

                        self.is_stitch_ready = False
                        self.is_dir_reset = False
                    else:
                        print(f"--- Panorama EXCEEDS the bounding box. Iteration: {self.iteration}")
                        print(f"Panorama size: ({panorama_width}, {panorama_height})")
                        print(f"Bounding box size: ({bb_width}, {bb_height})")

                        cv2.imwrite(os.path.join(self.config['output_dir'], self.config['output_filename']) + str(self.iteration) + ".png", panorama)
                        cv2.imwrite(os.path.join(self.config['output_dir'], self.config['output_filename']) + str(self.iteration) + "_mask.png", panorama_mask)
                        cv2.imwrite(os.path.join(self.config['output_dir'], self.config['output_filename']) + str(self.iteration) + "_heatmap.png", panorama_heatmap)

                        self.iteration = self.iteration + 1



#                     panorama_msg = self.bridge.cv2_to_imgmsg(panorama, encoding="bgr8")
#                     panorama_msg.header.stamp = self.get_clock().now().to_msg()
#                     panorama_msg.header.frame_id = "camera_frame"
# 
#                     self.stitched_rgb.publish(panorama_msg)
# 
#                     cv2.imwrite(os.path.join(self.config['output_dir'], self.config['output_filename']) + ".png", panorama)
#                     cv2.imwrite(os.path.join(self.config['output_dir'], self.config['output_filename']) + "_mask.png", panorama_mask)
#                     cv2.imwrite(os.path.join(self.config['output_dir'], self.config['output_filename']) + "_heatmap.png", panorama_heatmap)
# 
#                     self.is_stitch_ready = False
#                     self.is_dir_reset = False

                else: 
                    self.get_logger().info("Waiting for Mission Completion")


                    if not self.is_dir_reset: 

#                         self.original_images = []
#                         self.original_sizes = []
#                         self.feats_list = []
#                         self.mask_images = []
#                         self.heatmap_images = []
#                         torch.cuda.empty_cache()

                        result = subprocess.run([SCRIPT_PATH])

                        self.is_dir_reset = True
                        self.iteration = 1

                    else:

                        time.sleep(2.0)
                        continue

        except Exception as e:

            self.get_logger().error(f"Waiting for startup failed with: {e}")

            result = subprocess.run([SCRIPT_PATH])

        finally:

            self.get_logger().info("Stitching completed")

            self.is_stitch_ready = False



    def load_config(self, config_path):
        """Load configuration from a YAML file and compile regex patterns."""
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)

        # Adjust device setting based on the string from config
        config["device"] = torch.device(config["device"] if torch.cuda.is_available() and config["device"] == "cuda" else "cpu")
        print("Configuration loaded successfully.")
        return config
    
    def apply_pose_filtering(self):
        if not self.to_be_pose_filtered:
            return

        blacklist = set(self.to_be_pose_filtered)
        before = len(self.image_data)

        self.image_data = {
            name: data
            for name, data in self.image_data.items()
            if name not in blacklist
        }

        after = len(self.image_data)
        self.get_logger().info(f"Pose filtering removed {before - after} images")


    def validate_image_data(self):
        for name, data in self.image_data.items():
            for k in ["image_path", "mask_path", "heatmap_path"]:
                if data[k] is None:
                    self.get_logger().warn(f"{name} missing {k}")

    

    def get_image_paths(self, config):
        """Get sorted image paths based on timestamps from filenames."""

        def basename_no_ext(path):
            return os.path.splitext(os.path.basename(path))[0]

        all_image_files = glob.glob(os.path.join(config["image_directory"], '*.*'))
        
        # Note: Changed to look for 'mask_directory' to distinguish from images
        # If your masks are in the same folder, ensure the glob pattern distinguishes them
        all_mask_image_files = glob.glob(os.path.join(config["mask_image_directory"], '*.*'))
        all_heatmap_image_files = glob.glob(os.path.join(config["heatmap_image_directory"], '*.*'))

        # Index by name
        image_map = {basename_no_ext(p): p for p in all_image_files}
        mask_map = {basename_no_ext(p): p for p in all_mask_image_files}
        heatmap_map = {basename_no_ext(p): p for p in all_heatmap_image_files}

        missing = 0

        for name, data in self.image_data.items():
            data["image_path"] = image_map.get(name)
            data["mask_path"] = mask_map.get(name)
            data["heatmap_path"] = heatmap_map.get(name)

            if data["image_path"] is None:
                missing += 1

        if missing > 0:
            self.get_logger().warn(f"{missing} images missing image files")

        self.get_logger().info(f"{len(self.image_data)} images indexed via dict")

    def extract_timestamp(self, filename):
        """Extract the timestamp from the filename assuming it is a number before the file extension."""
        basename = os.path.basename(filename)
        # Use regex to find the last sequence of digits before the file extension
        match = re.search(r'(\d+)(?=\.\w+$)', basename)
        if match:
            return int(match.group(1))
        return None

    def process_bounding_box(self):
        previous_gps = None
        previous_image_size = None
        previous_image_origin = None
        stitch_orientation = np.array([[0, 0, 0], [0, 0, 0]], dtype=np.float32)

        all_corners = []

        for index, (name, data) in enumerate(self.image_data.items()):
            self.get_logger().info(f"Processing bounding box for image {index}: {name}")

            image_path = data["image_path"]

            # Read image resolution
            img = cv2.imread(image_path)
            if img is None:
                self.get_logger().warn(f"Failed to read image {image_path}")
                continue

            h, w = img.shape[:2]
            current_image_size = np.array([w, h], dtype=np.float32)

            # GPS as vector
            current_gps = np.array([data["lat"], data["lon"], data["alt"]], dtype=np.float32)

            if index == 0:
                previous_gps = current_gps
                previous_image_size = current_image_size
                previous_image_origin = np.array([0, 0], dtype=np.float32)

                # Initial corners at origin
                corners = np.array([
                    [-w/2, -h/2],
                    [-w/2,  h/2],
                    [ w/2,  h/2],
                    [ w/2, -h/2]
                ], dtype=np.float32) + previous_image_origin
                all_corners.append(corners)
                continue
            
            # Equirectangular approximation
            dx = (current_gps[1] - previous_gps[1]) * np.cos(np.deg2rad(previous_gps[0])) * 2*np.pi*R_EARTH / 360.0
            dy = (current_gps[0] - previous_gps[0]) * 2*np.pi*R_EARTH / 360.0
            dz = current_gps[2] - previous_gps[2]  # altitude difference`

            self.get_logger().info(f"Image {index}'s dx: {dx}, dy: {dy}")

            # Comput pixel offset
            f_px = CALIBRATED_FOCAL_LENGTH
            x_px = dx * f_px / previous_gps[2]  # scale by previous altitude
            y_px = dy * f_px / previous_gps[2]

            # Vector math
            expected_vector = np.array([x_px, y_px]) # in pixel coordinate
            stitch_orientation[1][0] = stitch_orientation[1][0] + expected_vector[0]
            stitch_orientation[1][1] = stitch_orientation[1][1] + expected_vector[1]
            expected_scale = current_gps[2] / previous_gps[2]

            # Compute origin and scale
            current_image_origin = previous_image_origin + expected_vector
            current_image_size = previous_image_size * expected_scale
            w_resized, h_resized = current_image_size

            # Compute corners centered at origin
            current_corners = np.array([
                [-w_resized/2, -h_resized/2],
                [-w_resized/2,  h_resized/2],
                [ w_resized/2,  h_resized/2],
                [ w_resized/2, -h_resized/2]
            ], dtype=np.float32) + current_image_origin

            self.get_logger().info(f"Image {index}'s x_px: {x_px}, y_px: {y_px}")
            self.get_logger().info(f"Image {index}'s origin: {current_image_origin}")
            self.get_logger().info(f"Image {index}'s corners: {current_corners}")
            self.get_logger().info(f"Image {index}'s size: {current_image_size}")

            # Optional: rotate corners toward expected_vector
            # Uncomment if you want rotation
            # dx, dy = expected_vector[0], expected_vector[1]
            # theta = atan2(dy, dx)
            # R = np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]], dtype=np.float32)
            # current_corners = (R @ current_corners.T).T + current_image_origin

            all_corners.append(current_corners)

            # Update previous values
            previous_gps = current_gps
            previous_image_size = current_image_size
            previous_image_origin = current_image_origin

        # Combine all corners to compute overall bounding box
        all_corners_np = np.vstack(all_corners)
        min_x, min_y = all_corners_np.min(axis=0)
        max_x, max_y = all_corners_np.max(axis=0)

        self.get_logger().info(f"Bounding box: x[{min_x}, {max_x}], y[{min_y}, {max_y}]")

        return min_x, max_x, min_y, max_y, stitch_orientation #, all_corners
            

    def process_image(self, path, config):
        """Load, crop, resize, and extract features from an image."""
        image_cv = cv2.imread(path)
        print(f"Processing image: {path}")
        if image_cv is None:
            print(f"Error loading image {path}")
            return None, None

        # Crop image
        h, w = image_cv.shape[:2]
        image_cropped = image_cv[config["top_crop"]:h - config["bottom_crop"], config["left_crop"]:w - config["right_crop"]]

        # Update size after cropping
        h_cropped, w_cropped = image_cropped.shape[:2]

        # Convert cropped image to tensor
        # image_tensor = transforms.ToTensor()(image_cropped).to(config["device"]).unsqueeze(0)

        # Resize image for feature extraction
        image_resized = cv2.resize(image_cropped, (config["fixed_width"], config["fixed_height"]), interpolation=cv2.INTER_LINEAR)
        image_resized_tensor = transforms.ToTensor()(image_resized).to(config["device"]).unsqueeze(0)

        # Extract features
        if self.extractor is None:
            print('Using SuperPoint for feature extraction')
            self.extractor = SuperPoint(max_num_keypoints=2048).eval().to(config["device"])
        elif config['extractor'] == 'aliked':
            print('Using ALIKED for feature extraction')
            self.extractor = ALIKED(max_num_keypoints=2048).eval().to(config["device"])

        multi_feats = []
        rotations = [0, 90, 180, 270]

        cv2_rots = [None, cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_180, cv2.ROTATE_90_COUNTERCLOCKWISE]

        for rot_deg, cv2_rot in zip(rotations, cv2_rots):
            # Rotate image
            if cv2_rot is not None:
                img_rot = cv2.rotate(image_resized, cv2_rot)
            else:
                img_rot = image_resized

            img_tensor = transforms.ToTensor()(img_rot).to(config["device"]).unsqueeze(0)

            with torch.no_grad():
                feats = self.extractor.extract(img_tensor)

                # --- IMPORTANT: Map keypoints back to 0-degree coordinate system ---
                # Keypoints are currently in the rotated frame (W_rot, H_rot)
                # We move them back to (fixed_width, fixed_height)
                kpts = feats['keypoints'] # Tensor [N, 2]
                fw, fh = config["fixed_width"], config["fixed_height"]

                if rot_deg == 90:
                    # (x, y) -> (y, fw - x)
                    new_x = kpts[:, 1]
                    new_y = fw - kpts[:, 0]
                    kpts = torch.stack([new_x, new_y], dim=-1)
                elif rot_deg == 180:
                    # (x, y) -> (fw - x, fh - y)
                    kpts = torch.stack([fw - kpts[:, 0], fh - kpts[:, 1]], dim=-1)
                elif rot_deg == 270:
                    # (x, y) -> (fh - y, x)
                    new_x = fh - kpts[:, 1]
                    new_y = kpts[:, 0]
                    kpts = torch.stack([new_x, new_y], dim=-1)
                
                multi_feats.append(feats)

        # with torch.no_grad():
        #     feats = self.extractor.extract(image_resized_tensor)
        #     feats['keypoints'] = feats['keypoints'].unsqueeze(0)
        #     feats['descriptors'] = feats['descriptors'].unsqueeze(0)
        #     feats['scores'] = feats.get('scores', torch.ones((1, feats['keypoints'].shape[1]), device=feats['keypoints'].device))
        #     feats = rbd(feats)
            # if 'scores' not in feats:
            #     # Create a tensor of ones with the same length as keypoints
            #     num_kpts = feats['keypoints'].shape[0]
            #     feats['scores'] = torch.ones(num_kpts, device=feats['keypoints'].device)
# 
            # # Filter top 1000 points by score
            # k = min(1000, feats['keypoints'].shape[0])
            # indices = torch.topk(feats['scores'], k=k).indices
            # 
            # # 5. DYNAMIC FILTERING: Slice EVERY tensor in the dict that matches the keypoint count
            # # This prevents the "desync" error
            # for key in feats:
            #     if torch.is_tensor(feats[key]) and feats[key].shape[0] == num_kpts:
            #         feats[key] = feats[key][indices]


        # del image_resized_tensor, image_resized
        del img_tensor

#         kpts = feats['keypoints'].cpu().numpy()
# 
#         if kpts.ndim == 3:
#             kpts = kpts[0]
# 
#         
#         scale_x, scale_y = w_cropped / config["fixed_width"], h_cropped / config["fixed_height"]
#         vis_image = image_cropped.copy()
# 
#         print(f"Extracted {len(kpts)} features from {path}")
#         print(f"kpts shape: {kpts.shape}")
#         print(f"kpts : {kpts}")
# 
#         for i in range(len(kpts)):
#             # .item() ensures we have a standard Python float
#             raw_x = float(kpts[i, 0])
#             raw_y = float(kpts[i, 1])
#             
#             px = int(round(raw_x * scale_x))
#             py = int(round(raw_y * scale_y))
#             
#             cv2.circle(vis_image, (px, py), 3, (0, 0, 255), -1)
# 
#         # Save the result
#         filename = os.path.basename(path)
#         save_path = os.path.join(self.config['output_dir'], self.config['output_filename']) + f"{filename}"
#         cv2.imwrite(save_path, vis_image)
#         self.image_stitch_paths.append(save_path)

        vis_image = image_cropped.copy()
        kpts_0 = multi_feats[0]['keypoints'][0].cpu().numpy()
        scale_x, scale_y = w_cropped / config["fixed_width"], h_cropped / config["fixed_height"]

        for kp in kpts_0:
            px = int(round(kp[0] * scale_x))
            py = int(round(kp[1] * scale_y))
            cv2.circle(vis_image, (px, py), 3, (0, 0, 255), -1)

        filename = os.path.basename(path)
        save_path = os.path.join(self.config['output_dir'], self.config['output_filename']) + filename
        cv2.imwrite(save_path, vis_image)
        self.image_stitch_paths.append(save_path)

        print(f"Extracted {len(kpts_0)} features from {path}")

        # return (h_cropped, w_cropped), feats
        return (h_cropped, w_cropped), multi_feats

    def tensor_to_image(self, tensor):
        """Convert a PyTorch tensor to a NumPy image."""
        image = tensor.squeeze(0).permute(1, 2, 0).cpu().numpy()
        image = (image * 255).astype(np.uint8)
        return image



    def match_keypoints(self, feats_list, original_sizes, config):
        """Iterate through 4 rotated feature sets to find the best geometric match."""
        print(f"Matching keypoints for {len(feats_list)} images...")
        
        # Define matching network
        matcher = LightGlue(features=config['extractor']).eval().to(config["device"])

        accumulated_H = np.eye(3)
        transformations = [accumulated_H.copy()]
        h0, w0 = original_sizes[0]
        corners0 = np.array([[0, 0], [0, h0], [w0, h0], [w0, 0]], dtype=np.float32)
        all_corners = [corners0]

        for i in range(1, len(feats_list)):
            print(f"Matching pair {i-1} to {i}")
            
            # We always match against the 0-degree features of the PREVIOUS image
            feats0 = feats_list[i - 1][0] 
            
            best_H = None
            best_inliers_count = -1
            best_m_kpts1_orig = None
            best_rot_idx = 0

            # Try matching all 4 rotations of the CURRENT image
            for rot_idx in range(4):
                feats1 = feats_list[i][rot_idx]

                # 1. Match features
                matches01 = matcher({"image0": feats0, "image1": feats1})

                matches01_rbd = rbd(matches01)

                f0_rbd = rbd(feats0)
                f1_rbd = rbd(feats1)
                matches = matches01_rbd["matches0"]

                # 2. Filter valid matches
                valid = matches > -1
                if valid.sum() < 3: continue
                m_kpts0 = f0_rbd["keypoints"][valid]
                m_kpts1 = f1_rbd["keypoints"][matches[valid]]

                if len(m_kpts0) < 3:
                    continue
                

                # 3. Scale back to original image dimensions
                h_resized, w_resized = config["fixed_height"], config["fixed_width"]
                h0_orig, w0_orig = original_sizes[i - 1]
                hi_orig, wi_orig = original_sizes[i]

                m_kpts0_orig = m_kpts0.clone()
                m_kpts0_orig[:, 0] *= (w0_orig / w_resized)
                m_kpts0_orig[:, 1] *= (h0_orig / h_resized)

                m_kpts1_orig = m_kpts1.clone()
                m_kpts1_orig[:, 0] *= (wi_orig / w_resized)
                m_kpts1_orig[:, 1] *= (hi_orig / h_resized)


                # 4. Estimate transformation (RANSAC)
                m_kpts0_np = m_kpts0_orig.cpu().numpy()
                m_kpts1_np = m_kpts1_orig.cpu().numpy()

                retval = cv2.estimateAffinePartial2D(
                    m_kpts1_np, m_kpts0_np, 
                    method=cv2.RANSAC, 
                    maxIters=5000, 
                    confidence=0.999, 
                    ransacReprojThreshold=2.0
                )

                if retval is not None:
                    H_affine, inliers = retval
                    if H_affine is not None:
                        num_inliers = np.sum(inliers)
                        print(f"{num_inliers} at orientation of {rot_idx * 90}")
                        if num_inliers > best_inliers_count:
                            print(f"Updating best H with {rot_idx * 90}")
                            best_inliers_count = num_inliers
                            best_H = np.vstack([H_affine, [0, 0, 1]])
                            best_m_kpts1_orig = m_kpts1_orig # For visualization
                            best_rot_idx = rot_idx
                    else:
                        print("H_affine does not exist")
                else:
                    print("Something went wrong unpacing estimateAffinePartial2D")

            # --- Post-search: Process the winning rotation ---
            print(f"--- Debug 5 ---")
            if best_H is not None:
                # accumulated_H = accumulated_H @ best_H
                # transformations.append(accumulated_H.copy())
                
                print(f"Match found! Rotation of : {best_rot_idx * 90} (Inliers: {best_inliers_count})")

                fw, fh = config["fixed_width"], config["fixed_height"]
                R_matrix = np.eye(3)
                if best_rot_idx == 1: # 90 deg clockwise
                    # (x, y) -> (y, fw - x)
                    R_matrix = np.array([[0, 1, 0], 
                                         [-1, 0, fw], 
                                         [0, 0, 1]], dtype=np.float32)
                elif best_rot_idx == 2: # 180 deg
                    # (x, y) -> (fw - x, fh - y)
                    R_matrix = np.array([[-1, 0, fw], 
                                         [0, -1, fh], 
                                         [0, 0, 1]], dtype=np.float32)
                elif best_rot_idx == 3: # 270 deg (90 CCW)
                    # (x, y) -> (fh - y, x)
                    R_matrix = np.array([[0, -1, fh], 
                                         [1, 0, 0], 
                                         [0, 0, 1]], dtype=np.float32)

                orig_h, orig_w = original_sizes[i]
                scale_to_fixed = np.array([[fw/orig_w, 0, 0], 
                                           [0, fh/orig_h, 0], 
                                           [0, 0, 1]])
                scale_to_orig = np.array([[orig_w/fw, 0, 0], 
                                          [0, orig_h/fh, 0], 
                                          [0, 0, 1]])
                
                # The "Corrected" rotation for full-res pixels
                R_full_res = scale_to_orig @ R_matrix @ scale_to_fixed

                # Combine: New H = best_H @ R_full_res
                # This allows the warper to use the 0-degree original image
                best_H_corrected = best_H @ R_full_res

                # Chain to global accumulated matrix
                accumulated_H = accumulated_H @ best_H_corrected
                transformations.append(accumulated_H.copy())


                hi, wi = original_sizes[i]
                S = np.array([[wi/fw, 0, 0], [0, hi/fh, 0], [0, 0, 1]])
                S_inv = np.linalg.inv(S)

                # Update corners for bounding box
                corners_i = np.array([[0, 0], [0, hi], [wi, hi], [wi, 0]], dtype=np.float32)
                # Use perspectiveTransform for 3x3 matrices
                corners_i_transformed = cv2.perspectiveTransform(corners_i.reshape(-1, 1, 2), accumulated_H)
                all_corners.append(corners_i_transformed.reshape(-1, 2))

                # Visualization: Draw only the BEST matching features on the image
                image_cv = cv2.imread(self.image_stitch_paths[i])
                if image_cv is not None:
                    h_img, w_img = image_cv.shape[:2]
                    # Scale factors from original size to current visualization image size
                    # Note: if image_stitch_paths[i] is already cropped/original size, 
                    # use the ratio of image_cv size to original_sizes[i]
                    orig_h, orig_w = original_sizes[i]
                    scale_draw_x = w_img / orig_w
                    scale_draw_y = h_img / orig_h

                    for j in range(len(best_m_kpts1_orig)):
                        px = int(round(float(best_m_kpts1_orig[j, 0]) * scale_draw_x))
                        py = int(round(float(best_m_kpts1_orig[j, 1]) * scale_draw_y))
                        # Draw green dot for matches
                        cv2.circle(image_cv, (px, py), 4, (0, 255, 0), -1)

                    filename = os.path.basename(self.image_paths[i])
                    save_path = os.path.join(config['output_dir'], f"match_{best_rot_idx}_{filename}")
                    cv2.imwrite(save_path, image_cv)
            else:
                print(f"CRITICAL: No rotation could match image {i} to {i-1}")
                # Append identity or last known H to maintain list length
                transformations.append(accumulated_H.copy())

        return transformations, all_corners



    def stitch_images(self, transformations, all_corners):
        """Stitch images together into a panorama using original blending logic."""
        print("Starting stitching of images...")
        all_corners = np.vstack(all_corners)
        x_min, y_min = np.int32(all_corners.min(axis=0) - 0.5)
        x_max, y_max = np.int32(all_corners.max(axis=0) + 0.5)
        panorama_width = x_max - x_min
        panorama_height = y_max - y_min

        total_stitching = len(self.image_paths)
        stitch_count = 0

        # Compute the translation matrix
        translation = np.array([[1, 0, -x_min],
                                [0, 1, -y_min],
                                [0, 0, 1]])

        # Initialize the panorama and mask
        panorama = np.zeros((panorama_height, panorama_width, 4), dtype=np.uint8)
        panorama_mask = np.zeros((panorama_height, panorama_width, 4), dtype=np.uint8)
        panorama_heatmap = np.zeros((panorama_height, panorama_width, 4), dtype=np.uint8)
        mask_panorama = np.zeros((panorama_height, panorama_width), dtype=np.uint8)

        # Warp and blend each image using the original blending logic
        for i in range(len(self.image_paths)):
            stitch_count += 1
            print(f"Stitching image {stitch_count}/{total_stitching}")

            # 1. Load image
            rgb_image_cv = cv2.imread(self.image_paths[i])
            mask_image_cv = cv2.imread(self.mask_paths[i])
            heatmap_image_cv = cv2.imread(self.heatmap_paths[i])
            print(f"Processing image: {self.image_paths[i]}, {self.mask_paths[i]}, {self.heatmap_paths[i]}")
            if rgb_image_cv is None or mask_image_cv is None or heatmap_image_cv is None :
                print(f"Error loading image {self.image_paths[i]}, {self.mask_paths[i]}, {self.heatmap_paths[i]}")
                return None, None, None

            # 2. Crop image
            rgb_h, rgb_w = rgb_image_cv.shape[:2]
            mask_h, mask_w = mask_image_cv.shape[:2]
            heatmap_h, heatmap_w = heatmap_image_cv.shape[:2]
            rgb_image_cropped = rgb_image_cv[self.config["top_crop"]:rgb_h - self.config["bottom_crop"], self.config["left_crop"]:rgb_w - self.config["right_crop"]]
            mask_image_cropped = mask_image_cv[self.config["top_crop"]:mask_h - self.config["bottom_crop"], self.config["left_crop"]:mask_w - self.config["right_crop"]]
            heatmap_image_cropped = heatmap_image_cv[self.config["top_crop"]:heatmap_h - self.config["bottom_crop"], self.config["left_crop"]:heatmap_w - self.config["right_crop"]]

            hi, wi = rgb_image_cropped.shape[:2]

            # 5. Now perform your coordinate math
            H = transformations[i]
            H_total = translation @ H

            tx = H_total[0, 2]
            ty = H_total[1, 2]

            print(f"[Image {i}] Translation -> x: {tx:.2f}, y: {ty:.2f}")

            center = np.array([[wi / 2, hi / 2, 1.0]])
            center_pan = (H_total @ center.T).flatten()

            print(f"[Image {i}] center -> x: {center_pan[0]:.2f}, y: {center_pan[1]:.2f}")

            # Warp the image
            warped_image_i = cv2.warpPerspective(rgb_image_cropped, H_total, (panorama_width, panorama_height))
            warped_mask_image_i = cv2.warpPerspective(mask_image_cropped, H_total, (panorama_width, panorama_height))
            warped_heatmap_image_i = cv2.warpPerspective(heatmap_image_cropped, H_total, (panorama_width, panorama_height))

            warped_image_i = cv2.cvtColor(warped_image_i, cv2.COLOR_BGR2BGRA)
            warped_mask_image_i = cv2.cvtColor(warped_mask_image_i, cv2.COLOR_BGR2BGRA)
            warped_heatmap_image_i = cv2.cvtColor(warped_heatmap_image_i, cv2.COLOR_BGR2BGRA)

            # Warp the mask
            hi, wi = rgb_image_cropped.shape[:2]
            mask_i = np.ones((hi, wi), dtype=np.uint8) * 255
            warped_mask_i = cv2.warpPerspective(mask_i, H_total, (panorama_width, panorama_height))

            # Update panorama and mask_panorama with original blending logic
            mask_overlap = warped_mask_i > 0
            # panorama[mask_overlap] = warped_image_i[mask_overlap]
            # panorama_mask[mask_overlap] = warped_mask_image_i[mask_overlap]
            # panorama_heatmap[mask_overlap] = warped_heatmap_image_i[mask_overlap]

            panorama[mask_overlap, :3] = warped_image_i[mask_overlap, :3]
            panorama[mask_overlap, 3] = 255

            panorama_mask[mask_overlap, :3] = warped_mask_image_i[mask_overlap, :3]
            panorama_mask[mask_overlap, 3] = 255

            panorama_heatmap[mask_overlap, :3] = warped_heatmap_image_i[mask_overlap, :3]
            panorama_heatmap[mask_overlap, 3] = 255

            # Mask for masking panorama
            mask_panorama[mask_overlap] = warped_mask_i[mask_overlap]

            del rgb_image_cv, rgb_image_cropped, warped_image_i, mask_image_cv, mask_image_cropped, warped_mask_image_i, heatmap_image_cv, heatmap_image_cropped, warped_heatmap_image_i
            gc.collect()
            torch.cuda.empty_cache()

            print(f"Stitched image {i + 1} of {len(self.image_paths)}")

        print("Stitching completed.")
        return panorama, panorama_mask, panorama_heatmap


def main(args=None):

    rclpy.init(args=args)

    stitch = None
    executor = None


    try:

        stitch = StitchingNode()
        executor = SingleThreadedExecutor()
        executor.add_node(stitch)
        executor.spin()

    except KeyboardInterrupt:
        print("Shutting Down!")
    except Exception as e:
        print(e)
    finally:

        if stitch: stitch.destroy_node()

        if executor: executor.shutdown()

        if rclpy.ok(): rclpy.shutdown()



if __name__ == '__main__':
    main()
