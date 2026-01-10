#!/usr/bin/python3
#Nodes + QoS
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSPresetProfiles, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.context import Context


#Executors
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

#msgs
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Image
from PIL import Image as PILImage
from std_srvs.srv import Trigger
from custom_interfaces.msg import RGB

import struct

#Comms
import threading 
import socket
import sys

#Timezones
import time
from zoneinfo import ZoneInfo
from datetime import datetime, timezone

#CSVs
import csv
import io
import os

#Image msgs
import cv2
from cv_bridge import CvBridge
import numpy as np

#Transforms
from transforms3d import _gohlketransforms, euler, quaternions

#Math
import math

HOST = "None"
PORT = 6060
RECONNECT_TIMER_PERIOD = 2.0

DISCOVERY_MESSAGE_EXPECTED = b'discover_jetson_request' 
DISCOVERY_MESSAGE_RESPONSE = b'IAmTheJetsonForStitching' 
DISCOVERY_PORT = 6060
DISCOVERY_PORT_TABLET = 8060
LISTEN_TIMEOUT = 1.0


RGB_IMAGE_START_PREFIX = "RGB_IMAGE_START_JPEG"
VARI_IMAGE_START_PREFIX = "VARI_IMAGE_START_JPEG"
IMAGE_END_SUFFIX = "IMAGE_END"
IMAGE_NONE_PREFIX = "IMAGE_NONE"


MASK_RECEIVE_PREFIX = "MASK_IMAGE_START\n"
MASK_RECEIVE_SUFFIX = "MASK_IMAGE_END\n"

RGB_RECEIVE_PREFIX = "RGB_IMAGE_START\n"
RGB_RECEIVE_SUFFIX = "RGB_IMAGE_END\n"

GPS_RECEIVE_PREFIX = "GPS_START\n"
GPS_RECEIVE_SUFFIX = "GPS_END\n"


HEATMAP_RECEIVE_PREFIX = "HEATMAP_START\n"
HEATMAP_RECEIVE_SUFFIX = "HEATMAP_END\n"

RPY_RECEIVE_PREFIX = "RPY_START\n"
RPY_RECEIVE_SUFFIX = "RPY_END\n"


BUFFER_SIZE_PREFIX = "SIZE_START\n"
BUFFER_SIZE_SUFFIX = "SIZE_END\n"


HEALTH_RECEIVE_PREFIX = "HEALTH_START\n"
HEALTH_RECEIVE_SUFFIX = "HEALTH_END\n"


STITCH_SEND_PREFIX = "STITCHED_IMAGE_START\n"
STITCH_SEND_SUFFIX = "STITCHED_IMAGE_END\n"

MISSION_FINISHED = "MISSION FINISHED\n"


RGB_MAX_SIZE = 20



# ---- Helpers ---- 

global_shutdown_flag = threading.Event()




class ReceiveData(Node):

    def __init__(self):
        super().__init__('receive_data')

        self.shutdown_flag = threading.Event()

        global HOST
        global HOST_TABLET

        self.host_socket = None
        self.discovery_socket = None


        #Run on a loop
        # while (HOST == "None" and not self.shutdown_flag.is_set() and rclpy.ok()): 
        #     HOST = str(self.host_discovery_listener(DISCOVERY_PORT))
        #     if HOST == "None":
        #         # print(f"Could not find HOST on network")
        #         self.get_logger().info("Could not find HOST on network")
        #     else:
        #         self.get_logger().info("Found the client!")
        #         break

        # while (HOST_TABLET == "None" and not self.shutdown_flag.is_set() and rclpy.ok()): 
        #     HOST_TABLET = str(discover_host_udp(DISCOVERY_PORT_TABLET, DISCOVERY_MESSAGE_TABLET))
        #     if (HOST_TABLET == "None"): print(f"Could not find HOST_TABLET on network")

        #connections
        self.connected = False
        # self.connection_timer = self.create_timer(RECONNECT_TIMER_PERIOD, self.connect_to_client_callback)
        self.connected_tablet = False
        # self.connection_timer_tablet = self.create_timer(RECONNECT_TIMER_PERIOD, self.connect_to_tablet_callback)
        self.threads_running = False

        #threads list
        self.threads = []

        #init for data
        self.gps_lat = 0.0
        self.gps_long = 0.0
        self.gps_alt = 0.0
        self.gps_signal_level = 0
        self.compass_heading = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.gimbal_roll = 0.0
        self.gimbal_pitch = 0.0
        self.gimbal_yaw = 0.0
        
        self.yaw_and_gimbal_quaternion = None
        self.yaw_and_gimbal_quaternion_past = None

        self.pose_filter_triggered = False
        

        #mutex for writing to file
        self.csv_lock = threading.Lock()



        self.pst_tz = ZoneInfo("America/Vancouver")
        self.bridge = CvBridge()
        qos_settings = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=15, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        # self.publisher_ = self.create_publisher(Image, "request_segmentation", qos_profile=qos_settings)
        # self.gps_publisher_ = self.create_publisher(String,"image_gps", qos_profile=qos_settings)


        #stitched images
        self.mission_publisher_ = self.create_publisher(String, "mission_completion", qos_profile=qos_settings)
        self.mission_publisher_check_ = self.create_client(Trigger, "mission_completion_check") # , qos_profile=qos_settings)
        # self.rgb_subscriber_ = self.create_subscription(Image, "stitched_rgb", self.store_stitched_rgb, qos_profile=qos_settings)
        self.rgb_subscriber_ = self.create_subscription(RGB, "rgb_images", self.store_rgbs, qos_profile=qos_settings)
        self.mask_subscriber_ = self.create_subscription(Image, "mask_images", self.store_masks, qos_profile=qos_settings)
        self.heatmap_subscriber_ = self.create_subscription(Image, "heatmaps", self.store_heatmaps, qos_profile=qos_settings)
        self.pose_filter_publisher = self.create_publisher(String, 'pose_filter', qos_profile=qos_settings)

        self.stitched_image_to_bytes = None
        self.stitch_req = False
        self.stitch_request_in_flight = False
        self.inference_finished = False

        self.mission_msg = String()
        self.mission_msg.data = "MISSION FINISHED"

        #tracking segmented images
        self.rgb_buffer = []
        self.mask_buffer = []
        self.gps_buffer = []    
        self.heatmap_buffer = []
        self.rpy_buffer = []

        self.time_buffer = []

        self.total_time = 0
        self.total_images = 0


        self.heatmap_png_buffer = []
        self.mask_png_buffer = []
        self.rgb_png_buffer = []


        ## self.mask_names = self.create_subscription(String, 'mask_files', self.update_buffers, qos_profile=qos_settings)
        # self.rgb_images = self.create_subscription(Image, 'rgb_images', self.store_rgbs, qos_profile=qos_settings)
        # self.mask_images = self.create_subscription(Image, 'mask_images', self.store_masks, qos_profile=qos_settings)
        # self.heatmap_images = self.create_subscription(Image, 'heatmaps', self.store_heatmaps, qos_profile=qos_settings)
        ## self.crop_health = self.create_subscription(String, 'health', self.store_health_metrics, qos_profile=qos_settings)



        self.rgb_dir = "/home/chimera/ros2_ws/src/chimera_stitching/images"
        self.mask_dir = "/home/chimera/ros2_ws/src/chimera_stitching/masks"
        self.heatmap_dir = "/home/chimera/ros2_ws/src/chimera_stitching/heatmaps"


        self.buffer_ready = False
        self.cur_img_buffer = None
        self.cur_img_msg = None
        self.cur_cv_img = None



        #logging the send fields
        self.log_ready = False
        self.log_cv = threading.Condition()

        self.cur_mask_filepath = None
        self.cur_heatmap_filepath = None
        self.cur_rgb_filepath = None
        self.cur_gps_lat = None
        self.cur_gps_long = None
        self.cur_gps_alt = None
        self.cur_timestamp = None


        #sending the images to the tablet
        self.tablet_cv = threading.Condition()

        #sending health metrics to the tablet
        self.health_buffer = []

        self.last_image_time = time.monotonic()
        self.stitch_timeout_sec = 20.0




        #csv frames
        fields = ['Time', 'Original Image', 'Mask', 'Heatmap']
        csv_timestamp = datetime.now(self.pst_tz)
        file_timestamp = csv_timestamp.strftime("%Y_%m_%d_%H_%M_%S")
        self.filename = "/home/chimera/ros2_ws/src/chimera_stitching/logs/" + "log_"+ file_timestamp + ".csv"
        self.sent_log = "/home/chimera/ros2_ws/src/chimera_stitching/logs/" + "sent_" + file_timestamp + ".csv"

        sent_fields = ['Time', 'Heatmap']

        #Writing sent log fields

        with open(self.sent_log, 'w') as csvfile:
            csvwriter = csv.writer(csvfile)
            self.file_position = csvfile.tell()
            csvwriter.writerow(sent_fields)

        self.run_processing_threads()



        #Writing waterfall fields


        # with self.csv_lock:

        #     with open(self.filename, 'w') as csvfile:
        #         csvwriter = csv.writer(csvfile)
        #         self.file_position = csvfile.tell()
        #         csvwriter.writerow(fields)

    def host_discovery_listener(self, port):
        try:

            self.discovery_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.discovery_socket.bind(('', port))
            self.discovery_socket.settimeout(LISTEN_TIMEOUT)

            print(f"[*] Listening for discovery packet {DISCOVERY_MESSAGE_EXPECTED.decode('utf-8')} for {LISTEN_TIMEOUT} seconds...")

            while True:
                data, addr = self.discovery_socket.recvfrom(1024)
                client_ip, client_port = addr   

                print(f"Received: {data.decode()}")

                if data == DISCOVERY_MESSAGE_EXPECTED:
                    print(f"[+] Discovered client at {client_ip}:{client_port}")
                    self.discovery_socket.sendto(
                        DISCOVERY_MESSAGE_RESPONSE,
                        (client_ip, client_port)
                    )
                    self.discovery_socket.close()
                    return client_ip

        except socket.timeout:
            print(f"\n[-] Listen timed out after {LISTEN_TIMEOUT} seconds.")
            print("[-] No reply received.")
            return None


    def store_stitched_rgb(self,msg):
        self.get_logger().info("Received Stitched RGB")

        cv_stitched = self.bridge.imgmsg_to_cv2(msg)

        stitched_rgb_encoded = cv2.imencode('.png', cv_stitched)[1]
        self.stitched_image_to_bytes = stitched_rgb_encoded.tobytes()



    def store_health_metrics(self, msg):
        self.get_logger().info("Received health")

        with self.health_buffer_lock:
            self.health_buffer.append(msg.data.split('\n'))

    
    def on_stitch_response(self, future):
        try:
            response = future.result()
            self.stitch_req = response.success
            if self.stitch_req:
                self.mission_publisher_.publish(self.mission_msg)
                self.get_logger().info(f"Stitch request result: True")
            else:
                self.get_logger().info(f"Stitch request result: False")
        except Exception as e:
            self.get_logger().error(f"Stitch service failed: {e}")
        finally:
            self.last_image_time = time.monotonic()
            self.stitch_request_in_flight = False


    def unwrap_angle(self, curr, prev):
        diff = curr - prev
        if diff > 180:
            return curr - 360
        elif diff < -180:
            return curr + 360
        return curr
    

    def store_rgbs(self, msg):
        # self.get_logger().info(f"Received rgb. name: {msg.name}, lat: {msg.gps_latitude}, long: {msg.gps_longitude}, alt: {msg.gps_altitude}, roll: {msg.roll}, pitch: {msg.pitch}, yaw: {msg.yaw}, g_roll: {msg.gimbal_roll}, g_pitch: {msg.gimbal_pitch}, g_yaw: {msg.gimbal_yaw}")

        try:
            self.get_logger().info(f"Received rgb. name: {msg.name}")

            cv_rgb = self.bridge.imgmsg_to_cv2(msg.image)
            
            self.gps_lat = msg.gps_latitude
            self.gps_long = msg.gps_longitude
            self.gps_alt = msg.gps_altitude

            self.roll = msg.roll
            self.pitch = msg.pitch
            self.yaw = msg.yaw

            self.gimbal_roll = msg.gimbal_roll
            self.gimbal_pitch = msg.gimbal_pitch
            self.gimbal_yaw = msg.gimbal_yaw

            Quaternion_flight_yaw = _gohlketransforms.quaternion_from_euler(math.radians(self.roll), math.radians(self.pitch), math.radians(self.yaw), 'sxyz')
            Quaternion_gimbal_yaw = _gohlketransforms.quaternion_from_euler(math.radians(self.gimbal_roll), math.radians(self.gimbal_pitch), math.radians(self.gimbal_yaw), 'sxyz')
            Quaternion_flight_yaw = Quaternion_flight_yaw / np.linalg.norm(Quaternion_flight_yaw)
            Quaternion_gimbal_yaw = Quaternion_gimbal_yaw / np.linalg.norm(Quaternion_gimbal_yaw)
            self.get_logger().info(f"Quaternion_flight_yaw: {Quaternion_flight_yaw}")
            self.get_logger().info(f"Quaternion_gimbal_yaw: {Quaternion_gimbal_yaw}")
            self.yaw_and_gimbal_quaternion =  _gohlketransforms.quaternion_multiply(Quaternion_flight_yaw, Quaternion_gimbal_yaw)
            
            if self.yaw_and_gimbal_quaternion_past is not None:

                # enforce same hemisphere
                if np.dot(self.yaw_and_gimbal_quaternion, self.yaw_and_gimbal_quaternion_past) < 0:
                    self.yaw_and_gimbal_quaternion = - self.yaw_and_gimbal_quaternion

                forward_curr = quaternions.rotate_vector(np.array([0, 0, 1]), self.yaw_and_gimbal_quaternion)
                forward_past = quaternions.rotate_vector(np.array([0, 0, 1]), self.yaw_and_gimbal_quaternion_past)

                # q_rel = _gohlketransforms.quaternion_multiply(
                #     self.yaw_and_gimbal_quaternion,
                #     _gohlketransforms.quaternion_inverse(self.yaw_and_gimbal_quaternion_past)
                # )
                

                dot = np.clip(np.dot(forward_curr, forward_past), -1.0, 1.0)
                angle = math.acos(dot)

                # angle = 2 * math.acos(np.clip(abs(q_rel[0]), -1.0, 1.0))
            else:
                self.yaw_and_gimbal_quaternion_past = self.yaw_and_gimbal_quaternion.copy()
                angle = 0

            # vector_of_yaw_and_gimbal_quaternion = quaternions.rotate_vector(_gohlketransforms.unit_vector(np.array([0, 0, 1])), self.yaw_and_gimbal_quaternion, True)
            # vector_of_yaw_and_gimbal_quaternion_past = quaternions.rotate_vector(_gohlketransforms.unit_vector(np.array([0, 0, 1])), self.yaw_and_gimbal_quaternion_past, True)
            # angle_between_two_quaternions = _gohlketransforms.angle_between_vectors(vector_of_yaw_and_gimbal_quaternion, vector_of_yaw_and_gimbal_quaternion_past)

            self.get_logger().info(f"Angle between current frame {msg.name} and past frame: {angle:.4f}. roll: {msg.roll}, pitch: {msg.pitch}, yaw: {msg.yaw}, g_roll: {msg.gimbal_roll}, g_pitch: {msg.gimbal_pitch}, g_yaw: {msg.gimbal_yaw}")

            if  angle > math.pi / 6:
                self.pose_filter_triggered
                pose_filter_msg = String()
                pose_filter_msg.data = msg.name
                self.pose_filter_publisher.publish(pose_filter_msg)
                self.get_logger().info(f"---Pose filtered {msg.name}")


            cv_rgb = cv2.resize(cv_rgb, (1024, 768), interpolation=cv2.INTER_AREA)

            rgb_encoded = cv2.imencode('.png', cv_rgb)[1]
            rgb_to_bytes = rgb_encoded.tobytes()

            # with self.rgb_buffer_lock:
            #     self.rgb_buffer.append(rgb_to_bytes)


            #store rgb debug
            # timestamp = datetime.now(self.pst_tz).strftime("%Y%m%d_%H%M%S_%f")
            rgb_filepath = os.path.join(self.rgb_dir, f"{msg.name}.png")


            cv2.imwrite(rgb_filepath, cv_rgb) 
            self.last_image_time = time.monotonic()

            # heatmap_pil.save(heatmap_filepath)
            # with self.rgb_png:
            #     self.rgb_png_buffer.append(rgb_filepath)
        except Exception as e:
            self.get_logger().error(f"RGB receive failed: {e}")


    def store_heatmaps(self, msg):
        self.get_logger().info("Received heatmap")

        cv_heatmap = self.bridge.imgmsg_to_cv2(msg)

        cv_heatmap = cv2.resize(cv_heatmap, (1024, 768), interpolation=cv2.INTER_AREA)

        heatmap_encoded = cv2.imencode('.png', cv_heatmap)[1]
        heatmap_to_bytes = heatmap_encoded.tobytes()

        # with self.heatmap_buffer_lock:
        #     self.heatmap_buffer.append(heatmap_to_bytes)


        #store heatmaps debug
        # timestamp = datetime.now(self.pst_tz).strftime("%Y%m%d_%H%M%S_%f")
        heatmap_filepath = os.path.join(self.heatmap_dir, f"{msg.header.frame_id}.png")


        cv2.imwrite(heatmap_filepath, cv_heatmap) 
        self.last_image_time = time.monotonic()

        # heatmap_pil.save(heatmap_filepath)
        # with self.heatmap_png:
        #     self.heatmap_png_buffer.append(heatmap_filepath)
        



    def store_masks(self, msg):

        self.get_logger().info("Received mask image")

        cv_mask = self.bridge.imgmsg_to_cv2(msg)

        cv_mask = cv2.resize(cv_mask, (1024,768), interpolation=cv2.INTER_AREA)

        mask_encode = cv2.imencode('.png', cv_mask)[1]
        mask_to_bytes = mask_encode.tobytes()

        # with self.mask_buffer_lock:
        #     self.mask_buffer.append(mask_to_bytes)


        # timestamp = datetime.now(self.pst_tz).strftime("%Y%m%d_%H%M%S_%f")
        mask_filepath = os.path.join(self.mask_dir, f"{msg.header.frame_id}.png")

        cv2.imwrite(mask_filepath, cv_mask)
        self.last_image_time = time.monotonic()

        # mask_pil.save(mask_filepath)
        # with self.mask_png:
        #     self.mask_png_buffer.append(mask_filepath)


#     def receive_data_from_remote(self):
#         if not self.connected:
#             self.get_logger().info(f"connected: {self.connected}")
#             return
# 
#         # self.get_logger().info("Receiving data from remote server...")
#         file_obj = self.conn.makefile('rb')
# 
#         try:
#             while self.connected and not self.shutdown_flag.is_set():
#                 # Always read raw bytes first, then decode
#                 line_bytes = file_obj.readline()
# 
#                 if not line_bytes:
#                     self.get_logger().warn("Connection closed by server.")
#                     self.connected = False
# 
#                     break # Exit the loop cleanly
# 
#                 # Decode the line, stripping whitespace
#                 # line = line_bytes.decode('utf-8').strip()
# 
#                 if line_bytes.startswith(("MISSION FINISHED").encode()):
#                     line = line_bytes.decode('utf-8')
#                     self.get_logger().info("Mission Signal Received")
# 
# 
#                     if not self.stitch_req:
#                         self.stitch_req = True
# 
#                         # if self.inference_finished:
#                         #     self.mission_publisher_.publish(self.mission_msg)
#                 
#                 # else:
#                     # if line: # Avoid logging empty lines
#                         # self.get_logger().info(f"Received unhandled line: {line}")
# 
#                 # time.sleep(0.3)
#             
# 
# 
# 
#         except Exception as e:
#             self.get_logger().error(f"An unexpected error occurred in receive loop: {e}")
#         finally:
#             # Cleanup when the loop exits for any reason
# 
#             self.get_logger().warn("Receive loop finished.")
#             # self.connected_tablet = False
#             self.connected = False
#             file_obj.close()
#             if self.conn is not None:
#                 self.conn.close()
#                 self.conn = None
#             else:
#                 # Reactivate the timer to attempt reconnection
#                 # while (HOST == "None" and not self.shutdown_flag.is_set() and rclpy.ok()): 
#                 #     HOST = str(self.host_discovery_listener(DISCOVERY_PORT))
#                 #     if HOST == "None":
#                 #         # print(f"Could not find HOST on network")
#                 #         self.get_logger().info("Could not find HOST on network")
#                 #     else:
#                 #         self.get_logger().info("Found the client!")
#                 #         break
#                 # self.connection_timer = self.create_timer(RECONNECT_TIMER_PERIOD, self.connect_to_client_callback)
#                 if not self.shutdown_flag.is_set():
#                     self.threads_running = False
#                     self.connection_timer = self.create_timer(RECONNECT_TIMER_PERIOD, self.connect_to_client_callback)
#                     # self.connection_timer_tablet = self.create_timer(RECONNECT_TIMER_PERIOD, self.connect_to_tablet_callback)
#                     self.get_logger().info("Disconnected. Reconnection timer activated.")
# 
# 
#             # if self.host_socket_tablet is not None:
#             #     self.host_socket_tablet.close()
#             #     self.host_socket_tablet = None


    def stitching_trigger(self):

        self.get_logger().info("Stitching trigger thread started")

        try:
            while not self.shutdown_flag.is_set():

                # if not self.connected:
                #     continue

                # Only act if mission finished signal was received
                # if self.stitch_req:
                elapsed = time.monotonic() - self.last_image_time

                if elapsed >= self.stitch_timeout_sec:
                    self.get_logger().warn(
                        f"No images received for {elapsed:.1f}s — checking mission completion"
                    )
                    self.last_image_time = time.monotonic()

                    if not self.stitch_req and not self.stitch_request_in_flight:
                        if not self.mission_publisher_check_.service_is_ready():
                            self.get_logger().warn("Stitch service not ready")
                            continue

                        self.stitch_request_in_flight = True
                        request = Trigger.Request()
                        future = self.mission_publisher_check_.call_async(request)
                        future.add_done_callback(self.on_stitch_response)

                    # self.mission_publisher_.publish(self.mission_msg)

                    # Prevent duplicate publishes
                    self.stitch_req = False
                    self.inference_finished = True

                time.sleep(1)

        except Exception as e:
            self.get_logger().error(f"Error in stitching_trigger: {e}")
        finally:
            self.get_logger().warn("Stitching trigger thread exited")




    def logging_callback(self):


        try:

            while not self.shutdown_flag.is_set():



                with self.log_cv:


                    while not self.log_ready:

                        self.log_cv.wait()

                        if self.shutdown_flag.is_set():
                            break

                    if self.shutdown_flag.is_set():
                        break


                    self.log_ready = False

                    sent_fields = ['Time', 'GPS Latitude', 'GPS Longitude', 'GPS Altitude', 'GPS Signal Level', 'Compass Heading', 'Original Image', 'Mask', 'Heatmap']
                    update_row  = [self.cur_timestamp, self.gps_lat, self.gps_long, self.gps_alt, self.gps_signal_level, self.compass_heading, self.cur_rgb_filepath, self.cur_mask_filepath, self.cur_heatmap_filepath]

                    with open(self.sent_log, 'a') as csvfile:
                        csvwriter = csv.writer(csvfile)
                        csvwriter.writerow(update_row)

        except Exception as e:
            self.get_logger().error(f"An unexpected error occured in logging thread: {e}")
        finally:
            self.get_logger().info("Logging loop has finished")
            self.connected = False
            self.connected_tablet = False

            if self.host_socket is not None:
                self.host_socket.close()
                self.host_socket = None



    def run_processing_threads(self):

        if self.threads_running:
            return
        else:

            self.threads_running = True

            # self.receive_thread = threading.Thread(target=self.receive_data_from_remote)
            # self.receive_thread.start()

            # self.sending_thread = threading.Thread(target=self.send_data_to_remote)
            # self.sending_thread.start()

            # self.buffers_thread = threading.Thread(target=self.update_buffers)
            # self.buffers_thread.start()

            self.stitching_trigger_thread = threading.Thread(target=self.stitching_trigger)
            self.stitching_trigger_thread.start()

            self.logging_thread = threading.Thread(target=self.logging_callback)
            self.logging_thread.start()


            # self.threads.append(self.receive_thread)
            # self.threads.append(self.sending_thread)
            # self.threads.append(self.buffers_thread)
            self.threads.append(self.stitching_trigger_thread)
            self.threads.append(self.logging_thread)




#     def connect_to_client_callback(self):
# 
#         if self.connected:
#             return
# 
#         try:
#             # Create listening socket ONCE
#             self.get_logger().info(f"Listening for client on port {PORT}...")
# 
#             self.host_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#             self.host_socket.bind(("", PORT))
#             self.host_socket.listen(1)
#             self.host_socket.settimeout(1)
# 
#             conn, addr = self.host_socket.accept()
# 
#             self.conn = conn
#             self.connected = True
# 
#             self.get_logger().info(
#                 f"Client connected from {addr[0]}:{addr[1]}"
#             )
# 
#             self.run_processing_threads()
#             self.connection_timer.cancel()
# 
#         except socket.timeout:
#             print(f"\n[-] Listen timed out after {LISTEN_TIMEOUT} seconds.")
#             print("[-] Client not connected yet.")
#             pass 
# 
#         except Exception as e:
#             self.get_logger().error(f"Accept failed: {e}")
#             if hasattr(self, "tcp_socket"):
#                 self.connected = False
#                 self.host_socket.close()
#                 del self.host_socket
# 
# 
#         except ConnectionRefusedError:
#             self.get_logger().warn("Connection failed. Tablet may not be running. Retrying...")
#             # The timer will automatically try again on its next cycle.
#             # self.client_socket_tablet.close() # Close the failed socket
#             if self.host_socket_tablet is not None:
#                 self.host_socket_tablet.close()
#                 self.host_socket_tablet.close()
#         except Exception as e:
#             self.get_logger().error(f"An unexpected error occurred: {e}")
#             # if self.client_socket_tablet:
#             #     self.client_socket_tablet.close()
#             if self.host_socket_tablet is not None:
#                 self.host_socket_tablet.close()
#                 self.host_socket_tablet = None

    def stop_all_threads(self):

        self.get_logger().warn("Stopping all Threads and setting shutdown flag")

        global global_shutdown_flag

        global_shutdown_flag.set()

        self.shutdown_flag.set()

        if self.log_cv:
            self.log_cv.acquire()
            self.log_cv.notify_all()
            self.log_cv.release()

        if self.csv_lock.locked(): 
            self.get_logger().warn("Releasing csv lock")
            self.csv_lock.release()


        for t in self.threads:

            if t.is_alive():
                t.join(timeout=2.0)

                if t.is_alive():
                    self.get_logger().error(f"Thread {t.name} failed to join succesfully")
                else:
                    self.get_logger().warn(f"Thread {t.name} joined succesfully")

        self.get_logger().info("All threads joined. Ready for shutdown")








def main(args=None):

    rclpy.init(args=args)

    receive_data = None
    executor = None

    try:

        receive_data = ReceiveData()

        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(receive_data)

        executor.spin()
    except KeyboardInterrupt:
        print("shutting down")
    except Exception as e:
        print(e)
    finally:


        if receive_data: 
            receive_data.destroy_node()
            receive_data.stop_all_threads()

        if executor: executor.shutdown()

    
        if rclpy.ok(): rclpy.shutdown()
        



if __name__ == '__main__':
    main()