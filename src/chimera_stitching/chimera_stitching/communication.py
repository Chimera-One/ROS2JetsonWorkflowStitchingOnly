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
        while (HOST == "None" and not self.shutdown_flag.is_set() and rclpy.ok()): 
            HOST = str(self.host_discovery_listener(DISCOVERY_PORT))
            if HOST == "None":
                # print(f"Could not find HOST on network")
                self.get_logger().info("Could not find HOST on network")
            else:
                self.get_logger().info("Found the client!")
                break

        # while (HOST_TABLET == "None" and not self.shutdown_flag.is_set() and rclpy.ok()): 
        #     HOST_TABLET = str(discover_host_udp(DISCOVERY_PORT_TABLET, DISCOVERY_MESSAGE_TABLET))
        #     if (HOST_TABLET == "None"): print(f"Could not find HOST_TABLET on network")

        #connections
        self.connected = False
        self.connection_timer = self.create_timer(RECONNECT_TIMER_PERIOD, self.connect_to_client_callback)
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

        #mutex for writing to file
        self.csv_lock = threading.Lock()
        self.mask_png = threading.Lock()
        self.heatmap_png = threading.Lock()
        self.rgb_png = threading.Lock()


        #mutexes for buffers
        self.rgb_buffer_lock = threading.Lock()
        self.heatmap_buffer_lock = threading.Lock()
        self.mask_buffer_lock = threading.Lock()
        self.time_buffer_lock = threading.Lock()
        self.health_buffer_lock = threading.Lock()




        self.pst_tz = ZoneInfo("America/Vancouver")
        self.bridge = CvBridge()
        qos_settings = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=15, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        # self.publisher_ = self.create_publisher(Image, "request_segmentation", qos_profile=qos_settings)
        # self.gps_publisher_ = self.create_publisher(String,"image_gps", qos_profile=qos_settings)


        #stitched images
        self.mission_publisher_ = self.create_publisher(String, "mission_completion", qos_profile=qos_settings)
        # self.rgb_subscriber_ = self.create_subscription(Image, "stitched_rgb", self.store_stitched_rgb, qos_profile=qos_settings)
        self.rgb_subscriber_ = self.create_subscription(Image, "rgb_images", self.store_rgbs, qos_profile=qos_settings)
        self.mask_subscriber_ = self.create_subscription(Image, "mask_images", self.store_masks, qos_profile=qos_settings)
        self.heatmap_subscriber_ = self.create_subscription(Image, "heatmaps", self.store_heatmaps, qos_profile=qos_settings)
        self.stitched_image_to_bytes = None
        self.stitch_req = False
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


        self.buffer_cv = threading.Condition()
        self.buffer_ready = False
        self.cur_img_buffer = None
        self.cur_img_msg = None
        self.cur_cv_img = None
        self.buffer_mutex = threading.Lock()



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
        self.stitch_timeout_sec = 10.0




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

    

    def store_rgbs(self, msg):
        self.get_logger().info("Received rgb")

        cv_rgb = self.bridge.imgmsg_to_cv2(msg)

        cv_rgb = cv2.resize(cv_rgb, (1024, 768), interpolation=cv2.INTER_AREA)

        rgb_encoded = cv2.imencode('.png', cv_rgb)[1]
        rgb_to_bytes = rgb_encoded.tobytes()

        # with self.rgb_buffer_lock:
        #     self.rgb_buffer.append(rgb_to_bytes)


        #store rgb debug
        timestamp = datetime.now(self.pst_tz).strftime("%Y%m%d_%H%M%S_%f")
        rgb_filepath = os.path.join(self.rgb_dir, f"rgb_{timestamp}.png")


        cv2.imwrite(rgb_filepath, cv_rgb) 
        self.last_image_time = time.monotonic()

        # heatmap_pil.save(heatmap_filepath)
        # with self.rgb_png:
        #     self.rgb_png_buffer.append(rgb_filepath)
        


    def store_heatmaps(self, msg):
        self.get_logger().info("Received heatmap")

        cv_heatmap = self.bridge.imgmsg_to_cv2(msg)

        cv_heatmap = cv2.resize(cv_heatmap, (1024, 768), interpolation=cv2.INTER_AREA)

        heatmap_encoded = cv2.imencode('.png', cv_heatmap)[1]
        heatmap_to_bytes = heatmap_encoded.tobytes()

        # with self.heatmap_buffer_lock:
        #     self.heatmap_buffer.append(heatmap_to_bytes)


        #store heatmaps debug
        timestamp = datetime.now(self.pst_tz).strftime("%Y%m%d_%H%M%S_%f")
        heatmap_filepath = os.path.join(self.heatmap_dir, f"heatmap_{timestamp}.png")


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


        timestamp = datetime.now(self.pst_tz).strftime("%Y%m%d_%H%M%S_%f")
        mask_filepath = os.path.join(self.mask_dir, f"mask_{timestamp}.png")

        cv2.imwrite(mask_filepath, cv_mask)
        self.last_image_time = time.monotonic()

        # mask_pil.save(mask_filepath)
        # with self.mask_png:
        #     self.mask_png_buffer.append(mask_filepath)





    def update_buffers(self):

        if not self.connected:
            return

        try:

            while self.connected and not self.shutdown_flag.is_set():
                with self.buffer_cv:


                    while not self.buffer_ready or (len(self.rgb_buffer) > RGB_MAX_SIZE):
                        self.get_logger().info("waiting for producer")
                        self.buffer_cv.wait()
                        # self.get_logger().info("loop iteration done")
                        # if(len(self.rgb_buffer) == RGB_MAX_SIZE): 
                        #     self.publisher_.publish(self.cur_img_msg)
                        #     self.cur_img_msg = None

                        if self.shutdown_flag.is_set():
                            break
                    
                    if self.shutdown_flag.is_set():
                        break
                    
                    self.rgb_buffer.append(self.cur_img_buffer)
                    self.publisher_.publish(self.cur_img_msg)

                    self.get_logger().info("Published message")
                    self.gps_buffer.append((self.gps_long, self.gps_lat, self.gps_alt))

                    gps_msg = String()
                    gps_msg.data = str(self.gps_lat) + " " + str(self.gps_long) + " " + str(self.gps_alt)
                    self.gps_publisher_.publish(gps_msg)

                    self.rpy_buffer.append((self.roll, self.pitch, self.yaw))
                    # self.get_logger().info(f"RPY is: {self.roll}, {self.pitch}, {self.yaw}")

                    csv_timestamp = datetime.now(self.pst_tz)
                    time_log = csv_timestamp.strftime("%H_%M_%S_%f")
                    rgb_filepath = f"/home/chimera/ros2_ws/src/chimera_segmentation/chimera_stitching/images/rgb_img_{time_log}.jpg"
                    self.rgb_png_buffer.append(rgb_filepath)


                    cv2.imwrite(rgb_filepath, self.cur_cv_img)

                    self.buffer_ready = False

                    self.buffer_cv.notify()


                    # self.get_logger().info("Updating buffers")

                        # update_row  = [" ", " ", " ", " ", " ", " ", " ", self.total_time]

                        # with open(self.sent_log, 'a') as csvfile:
                        #     csvwriter = csv.writer(csvfile)
                        #     csvwriter.writerow(update_row)

        except Exception as e:
            self.get_logger().error(f"An unexpected error occured in update buffers thread: {e}")
        finally:
            self.get_logger().warn("Update buffer has finished")
            self.connected = False


    def receive_data_from_remote(self):
        if not self.connected:
            self.get_logger().info(f"connected: {self.connected}")
            return

        # self.get_logger().info("Receiving data from remote server...")
        file_obj = self.conn.makefile('rb')

        try:
            while self.connected and not self.shutdown_flag.is_set():
                # Always read raw bytes first, then decode
                line_bytes = file_obj.readline()

                if not line_bytes:
                    self.get_logger().warn("Connection closed by server.")
                    self.connected = False

                    break # Exit the loop cleanly

                # Decode the line, stripping whitespace
                # line = line_bytes.decode('utf-8').strip()

                if line_bytes.startswith(RGB_RECEIVE_PREFIX.encode()):
                    line = line_bytes.decode('utf-8')
                    # self.get_logger().info(f"RGB prefix found")
                elif line_bytes.startswith(MASK_RECEIVE_PREFIX.encode()):
                    line = line_bytes.decode('utf-8')
                    # self.get_logger().info(f"MASK prefix found")
                elif line_bytes.startswith(HEATMAP_RECEIVE_PREFIX.encode()):
                    line = line_bytes.decode('utf-8')
                    # self.get_logger().info(f"HEATMAP prefix found")
                elif line_bytes.startswith(IMAGE_NONE_PREFIX.encode()):
                    line = line_bytes.decode('utf-8')
                    # self.get_logger().info(f"IMAGE NONE prefix found")
                elif line_bytes.startswith(("MISSION FINISHED").encode()):
                    line = line_bytes.decode('utf-8')
                    self.get_logger().info(f"MISSION FINISHED prefix found")
                else:
                    # unrecognized line, skip decoding
                    continue
                
                # --- Process RGB Image Data ---
                if line == RGB_RECEIVE_PREFIX:
                    try:
                        # 1. Parse image size from the header
                        image_size = file_obj.readline().strip()

                        if not image_size.isdigit():
                            self.get_logger().error(f"Invalid size header: {image_size[:32]}...")
                            continue
                        image_size = int(image_size)
                        # image_size = int(file_obj.readline().decode('utf-8').strip())
                        self.get_logger().info(f"RGB Image header received. Size: {image_size} bytes.")

                        # 2. Read the exact number of bytes for the image from the buffered file object
                        image_buffer = file_obj.read(image_size)

                        if len(image_buffer) != image_size:
                            self.get_logger().error("Did not receive the full RGB image binary.")
                            continue

                        # 3. Read the "RGB_RECEIVE_SUFFIX" marker
                        end_marker_bytes = file_obj.readline()
                        end_marker = end_marker_bytes.decode('utf-8')
                        if end_marker != RGB_RECEIVE_SUFFIX:
                            self.get_logger().warn(f"Expected {RGB_RECEIVE_SUFFIX}, but got: '{end_marker}'")

                        # 4. Process and save the image
                        # Will have to remove this later since it slows down transfer significantly

                        # self.get_logger().info("Image received successfully. Processing...")



                        # self.get_logger().info("Converting buffer to msgs")





                        # data = io.BytesIO(image_buffer)
                        # csv_msg = np.load(data)
                        np_msg = np.frombuffer(image_buffer, dtype=np.uint8)
                        cv_img = cv2.imdecode(np_msg, cv2.IMREAD_COLOR)

                        # self.get_logger().info("Converting cv_img to Image msg")

                        img_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
                        img_msg.header.stamp = self.get_clock().now().to_msg()
                        img_msg.header.frame_id = "camera_frame"

                        # self.get_logger().info("published msg")

                        # cv_img = cv2.resize(cv_img, (400,400))
                        # data_encode = cv2.imencode('.png', cv_img)[1]
                        # np_encode = np.array(data_encode)
                        
                        # self.get_logger().info("Resizing the image")

                        #cv2.imwrite(rgb_filepath, cv_img)

                        # self.get_logger().info("Updating buffers")




                        # with self.buffer_cv:


                        #    while self.buffer_ready:
                        #        self.get_logger().info("waiting for consumer")
                        #        self.buffer_cv.wait()


                        #    self.cur_img_buffer = image_buffer
                        #    self.cur_img_msg = img_msg
                        #    self.cur_cv_img = cv_img

                        #    self.buffer_ready = True
                        #    self.buffer_cv.notify()

                        self.store_rgbs(img_msg)


                        # with self.time_buffer_lock:

                        #     self.time_buffer.append(time.time())

                        ##5. Append to CSV file
                        # row = [time_log, self.gps_lat, self.gps_long, self.gps_alt, self.roll, self.pitch, self.yaw, str(f"/home/chimera/ros2_ws/src/chimera_segmentation/chimera_segmentation/images/rgb_img_{timestamp}.jpg"), 0 , 0]

                        # with self.csv_lock:

                        #     with open(self.filename, 'a', newline='') as csvfile:
                        #         csvwriter = csv.writer(csvfile)
                        #         csvwriter.writerow(row)
                        


                    except (ValueError, IndexError) as e:
                        self.get_logger().error(f"Error parsing RGB image size from header '{line}': {e}")
                    except Exception as e:
                        self.get_logger().error(f"Error reading or processing RGB image: {e}")

                # --- Process MASK Image Data ---
                elif line == MASK_RECEIVE_PREFIX:
                    try:
                        # 1. Parse image size from the header
                        image_size = file_obj.readline().strip()

                        if not image_size.isdigit():
                            self.get_logger().error(f"Invalid size header: {image_size[:32]}...")
                            continue
                        image_size = int(image_size)
                        # image_size = int(file_obj.readline().decode('utf-8').strip())
                        self.get_logger().info(f"MASK Image header received. Size: {image_size} bytes.")

                        # 2. Read the exact number of bytes for the image from the buffered file object
                        image_buffer = file_obj.read(image_size)

                        if len(image_buffer) != image_size:
                            self.get_logger().error("Did not receive the full MASK image binary.")
                            continue

                        # 3. Read the "MASK_RECEIVE_SUFFIX" marker
                        end_marker_bytes = file_obj.readline()
                        end_marker = end_marker_bytes.decode('utf-8')
                        if end_marker != MASK_RECEIVE_SUFFIX:
                            self.get_logger().warn(f"Expected {MASK_RECEIVE_SUFFIX}, but got: '{end_marker}'")

                        # 4. Process and save the image
                        # Will have to remove this later since it slows down transfer significantly

                        # self.get_logger().info("Image received successfully. Processing...")



                        # self.get_logger().info("Converting buffer to msgs")





                        # data = io.BytesIO(image_buffer)
                        # csv_msg = np.load(data)
                        np_msg = np.frombuffer(image_buffer, dtype=np.uint8)
                        cv_img = cv2.imdecode(np_msg, cv2.IMREAD_COLOR)

                        # self.get_logger().info("Converting cv_img to Image msg")

                        img_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
                        img_msg.header.stamp = self.get_clock().now().to_msg()
                        img_msg.header.frame_id = "camera_frame"

                        # self.get_logger().info("published msg")

                        # cv_img = cv2.resize(cv_img, (400,400))
                        # data_encode = cv2.imencode('.png', cv_img)[1]
                        # np_encode = np.array(data_encode)
                        
                        # self.get_logger().info("Resizing the image")

                        #cv2.imwrite(rgb_filepath, cv_img)

                        # self.get_logger().info("Updating buffers")




                        # with self.buffer_cv:


                        #    while self.buffer_ready:
                        #        self.get_logger().info("waiting for consumer")
                        #        self.buffer_cv.wait()


                        #    self.cur_img_buffer = image_buffer
                        #    self.cur_img_msg = img_msg
                        #    self.cur_cv_img = cv_img

                        #    self.buffer_ready = True
                        #    self.buffer_cv.notify()


                        self.store_masks(img_msg)


                        # with self.time_buffer_lock:

                        #     self.time_buffer.append(time.time())

                        ##5. Append to CSV file
                        # row = [time_log, self.gps_lat, self.gps_long, self.gps_alt, self.roll, self.pitch, self.yaw, str(f"/home/chimera/ros2_ws/src/chimera_segmentation/chimera_segmentation/images/rgb_img_{timestamp}.jpg"), 0 , 0]

                        # with self.csv_lock:

                        #     with open(self.filename, 'a', newline='') as csvfile:
                        #         csvwriter = csv.writer(csvfile)
                        #         csvwriter.writerow(row)
                        


                    except (ValueError, IndexError) as e:
                        self.get_logger().error(f"Error parsing MASK image size from header '{line}': {e}")
                    except Exception as e:
                        self.get_logger().error(f"Error reading or processing MASK image: {e}")

                # --- Process HEATMAP Image Data ---
                elif line == HEATMAP_RECEIVE_PREFIX:
                    try:
                        # 1. Parse image size from the header
                        image_size = file_obj.readline().strip()

                        if not image_size.isdigit():
                            self.get_logger().error(f"Invalid size header: {image_size[:32]}...")
                            continue
                        image_size = int(image_size)
                        # image_size = int(file_obj.readline().decode('utf-8').strip())
                        self.get_logger().info(f"HEATMAP Image header received. Size: {image_size} bytes.")

                        # 2. Read the exact number of bytes for the image from the buffered file object
                        image_buffer = file_obj.read(image_size)

                        if len(image_buffer) != image_size:
                            self.get_logger().error("Did not receive the full HEATMAP image binary.")
                            continue

                        # 3. Read the "HEATMAP_RECEIVE_SUFFIX" marker
                        end_marker_bytes = file_obj.readline()
                        end_marker = end_marker_bytes.decode('utf-8')
                        if end_marker != HEATMAP_RECEIVE_SUFFIX:
                            self.get_logger().warn(f"Expected {HEATMAP_RECEIVE_SUFFIX}, but got: '{end_marker}'")

                        # 4. Process and save the image
                        # Will have to remove this later since it slows down transfer significantly

                        # self.get_logger().info("Image received successfully. Processing...")



                        # self.get_logger().info("Converting buffer to msgs")





                        # data = io.BytesIO(image_buffer)
                        # csv_msg = np.load(data)
                        np_msg = np.frombuffer(image_buffer, dtype=np.uint8)
                        cv_img = cv2.imdecode(np_msg, cv2.IMREAD_COLOR)

                        # self.get_logger().info("Converting cv_img to Image msg")

                        img_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
                        img_msg.header.stamp = self.get_clock().now().to_msg()
                        img_msg.header.frame_id = "camera_frame"

                        # self.get_logger().info("published msg")

                        # cv_img = cv2.resize(cv_img, (400,400))
                        # data_encode = cv2.imencode('.png', cv_img)[1]
                        # np_encode = np.array(data_encode)
                        
                        # self.get_logger().info("Resizing the image")

                        #cv2.imwrite(rgb_filepath, cv_img)

                        # self.get_logger().info("Updating buffers")




                        # with self.buffer_cv:


                        #    while self.buffer_ready:
                        #        self.get_logger().info("waiting for consumer")
                        #        self.buffer_cv.wait()


                        #    self.cur_img_buffer = image_buffer
                        #    self.cur_img_msg = img_msg
                        #    self.cur_cv_img = cv_img

                        #    self.buffer_ready = True
                        #    self.buffer_cv.notify()


                        self.store_heatmaps(img_msg)


                        # with self.time_buffer_lock:

                        #     self.time_buffer.append(time.time())

                        ##5. Append to CSV file
                        # row = [time_log, self.gps_lat, self.gps_long, self.gps_alt, self.roll, self.pitch, self.yaw, str(f"/home/chimera/ros2_ws/src/chimera_segmentation/chimera_segmentation/images/rgb_img_{timestamp}.jpg"), 0 , 0]

                        # with self.csv_lock:

                        #     with open(self.filename, 'a', newline='') as csvfile:
                        #         csvwriter = csv.writer(csvfile)
                        #         csvwriter.writerow(row)
                        


                    except (ValueError, IndexError) as e:
                        self.get_logger().error(f"Error parsing HEATMAP image size from header '{line}': {e}")
                    except Exception as e:
                        self.get_logger().error(f"Error reading or processing HEATMAP image: {e}")

                elif line == IMAGE_NONE_PREFIX:
                    self.get_logger().info("Server indicated no image was sent.")

                elif line == MISSION_FINISHED:
                    self.get_logger().info("Mission Signal Received")


                    if not self.stitch_req:
                        self.stitch_req = True

                        # if self.inference_finished:
                        #     self.mission_publisher_.publish(self.mission_msg)
                
                # else:
                    # if line: # Avoid logging empty lines
                        # self.get_logger().info(f"Received unhandled line: {line}")

                # time.sleep(0.3)
            



        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred in receive loop: {e}")
        finally:
            # Cleanup when the loop exits for any reason

            self.get_logger().warn("Receive loop finished.")
            # self.connected_tablet = False
            self.connected = False
            file_obj.close()
            if self.conn is not None:
                self.conn.close()
                self.conn = None
            else:
                # Reactivate the timer to attempt reconnection
                # while (HOST == "None" and not self.shutdown_flag.is_set() and rclpy.ok()): 
                #     HOST = str(self.host_discovery_listener(DISCOVERY_PORT))
                #     if HOST == "None":
                #         # print(f"Could not find HOST on network")
                #         self.get_logger().info("Could not find HOST on network")
                #     else:
                #         self.get_logger().info("Found the client!")
                #         break
                self.connection_timer = self.create_timer(RECONNECT_TIMER_PERIOD, self.connect_to_client_callback)


            # if self.host_socket_tablet is not None:
            #     self.host_socket_tablet.close()
            #     self.host_socket_tablet = None


    def stitching_trigger(self):

        self.get_logger().info("Stitching trigger thread started")

        try:
            while not self.shutdown_flag.is_set():

                if not self.connected or not self.stitch_req:
                    time.sleep(1)
                    continue

                # Only act if mission finished signal was received
                if self.stitch_req:
                    elapsed = time.monotonic() - self.last_image_time

                    if elapsed >= self.stitch_timeout_sec:
                        self.get_logger().warn(
                            f"No images received for {elapsed:.1f}s — triggering mission completion"
                        )

                        self.mission_publisher_.publish(self.mission_msg)

                        # Prevent duplicate publishes
                        self.stitch_req = False
                        self.inference_finished = True

                time.sleep(1)

        except Exception as e:
            self.get_logger().error(f"Error in stitching_trigger: {e}")
        finally:
            self.get_logger().warn("Stitching trigger thread exited")




    def logging_callback(self):


        if not self.connected:
            return

        try:

            while self.connected and not self.shutdown_flag.is_set():



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


            # if self.host_socket_tablet is not None:
            #     self.host_socket_tablet.close()
            #     self.host_socket_tablet = None




    def send_data_to_remote(self):

        if not self.connected or not self.connected_tablet:
            return

        try:

            while not self.shutdown_flag.is_set() and self.connected and self.connected_tablet:




                if (self.stitched_image_to_bytes is not None):

                    self.get_logger().info("Sending Stitched Image")

                    stitched_size = len(self.stitched_image_to_bytes)
                    stitched_size_str = str(stitched_size) + '\n'

                    self.host_socket_tablet.send(STITCH_SEND_PREFIX.encode('utf-8'))
                    self.host_socket_tablet.send(stitched_size_str.encode('utf-8'))
                    self.host_socket_tablet.sendall(self.stitched_image_to_bytes)
                    self.host_socket_tablet.send(STITCH_SEND_SUFFIX.encode('utf-8'))

                    self.stitched_image_to_bytes = None
                    self.stitch_req = False

                # -------- MASK -------
                if ( (len(self.rgb_buffer) < 1) | (len(self.gps_buffer) < 1) | (len(self.mask_buffer) < 1) | (len(self.heatmap_buffer) < 1) | (len(self.heatmap_png_buffer) < 1) | (len(self.mask_png_buffer) < 1) ):
                    time.sleep(0.01)
                    # self.get_logger().info("Mask buffer is empty")
                    continue

                if ( (len(self.health_buffer) < 1) ): 
                    time.sleep(0.01)
                    continue

                # start = time.time()

                with self.mask_buffer_lock:
                    mask_to_bytes = self.mask_buffer.pop(0)

                with self.heatmap_buffer_lock:
                    heatmap_to_bytes = self.heatmap_buffer.pop(0)
                    self.get_logger().info(f"heatmap buffer size is {len(self.heatmap_buffer)}")



                # if (len(self.rgb_buffer) < 1) | (len(self.gps_buffer) < 1):
                #     time.sleep(0.01)
                #     continue

                with self.buffer_cv:

                    rgb_to_bytes = self.rgb_buffer.pop(0)
                    img_gps = self.gps_buffer.pop(0)

                    rgb_buffer_length = len(self.rgb_buffer)
                    self.get_logger().info(f"RGB Buffer size is: {len(self.rgb_buffer)}")

                    if(rgb_buffer_length == 0): 
                        self.inference_finished = True

                        if self.stitch_req == True:
                            self.mission_publisher_.publish(self.mission_msg)
                            self.stitch_req = False
                    else:
                        self.inference_finished = False

                    rgb_dir_str = self.rgb_png_buffer.pop(0)

                    img_rpy = self.rpy_buffer.pop(0)
                    self.buffer_cv.notify()


                with self.health_buffer_lock:
                    health_metrics = self.health_buffer.pop(0)
                    # self.get_logger().info(f"health metrics: {health_metrics[0]}")
                    # self.get_logger().info(f"real health metrics: {health_metrics}")



                



                # Converting Mask Image to Bytes

                # with PILImage.open(mask_filename) as mask:

                #     mask_byte_buffer = io.BytesIO()
                #     mask.save(mask_byte_buffer, format="PNG")
                #     mask_to_bytes = mask_byte_buffer.getvalue()


                mask_size = len(mask_to_bytes)
                mask_size_str = str(mask_size) + "\n"

                # self.get_logger().info("Image size sending to RC: " + mask_size_str)

                # Sending START bytes, size, image then end bytes


                # ------ MASK TO RC ------ 
                self.host_socket.send(MASK_SEND_PREFIX.encode('utf-8'))
                                                                                                                                                                                                                                                                                
                self.host_socket.send(mask_size_str.encode('utf-8'))

                self.host_socket.sendall(mask_to_bytes)
                self.host_socket.send(MASK_SEND_SUFFIX.encode('utf-8'))




                # -------- RGB -------
                rgb_size = len(rgb_to_bytes)
                rgb_size_str = str(rgb_size) + "\n"



                #Sending START bytes, size, image then end bytes

                # ------ RGB TO RC ------ 
                self.host_socket.send(RGB_SEND_PREFIX.encode('utf-8'))
                self.host_socket.send(rgb_size_str.encode('utf-8'))
                self.host_socket.sendall(rgb_to_bytes)
                self.host_socket.send(RGB_SEND_SUFFIX.encode('utf-8'))



                # self.get_logger().info("Finished sending RGB Image of size sending to RC: " + rgb_size_str)

                # ------ HEATMAP ------

                heatmap_size = len(heatmap_to_bytes)
                heatmap_size_str = str(heatmap_size) + "\n"

                # ------ HEATMAP TO RC ------ 
                self.host_socket.send(HEATMAP_SEND_PREFIX.encode('utf-8'))
                self.host_socket.send(heatmap_size_str.encode('utf-8'))
                self.host_socket.sendall(heatmap_to_bytes)
                self.host_socket.send(HEATMAP_SEND_SUFFIX.encode('utf-8'))



                # self.get_logger().info("Finished sending Heatmap Image of size sending to RC: " + heatmap_size_str)


                # -------- GPS -------


                img_long = img_gps[0]
                img_lat = img_gps[1]
                img_alt = img_gps[2]

                img_long_str = str(img_long) + "\n"
                img_lat_str = str(img_lat) + "\n"
                img_alt_str = str(img_alt) + "\n"

                # self.get_logger().info(img_long_str)
                # self.get_logger().info(img_lat_str)
                # self.get_logger().info(img_alt_str)


                # img_gps_send_str = GPS_SEND_PREFIX + ": " + img_long_strs + " " + img_lat_str + " " + img_alt_str + "\n"


                # -------- RPY -------

                img_roll = img_rpy[0]
                img_pitch = img_rpy[1]
                img_yaw = img_rpy[2]


                img_roll_str = str(img_roll) + "\n"
                img_pitch_str = str(img_pitch) + "\n"
                img_yaw_str = str(img_yaw) + "\n"




                # ------ GPS TO RC ------ 
                self.host_socket.send(GPS_SEND_PREFIX.encode('utf-8'))
                self.host_socket.send(img_long_str.encode('utf-8'))
                self.host_socket.send(img_lat_str.encode('utf-8'))
                self.host_socket.send(img_alt_str.encode('utf-8'))
                self.host_socket.send(GPS_SEND_SUFFIX.encode('utf-8'))




                # ---- Buffer Size ---
                buffer_size_str = str(rgb_buffer_length) + "\n"


                # ------ BUFFER TO RC ------ 
                self.host_socket.send(BUFFER_SIZE_PREFIX.encode('utf-8'))
                self.host_socket.send(buffer_size_str.encode('utf-8'))
                self.host_socket.send(BUFFER_SIZE_SUFFIX.encode('utf-8'))


                # ------ MASK TO TABLET ------ 
                self.host_socket_tablet.send(MASK_SEND_PREFIX.encode('utf-8'))
                self.host_socket_tablet.send(mask_size_str.encode('utf-8'))

                self.host_socket_tablet.sendall(mask_to_bytes)
                self.host_socket_tablet.send(MASK_SEND_SUFFIX.encode('utf-8'))

                # ------ RGB TO TABLET ------ 
                self.host_socket_tablet.send(RGB_SEND_PREFIX.encode('utf-8'))
                self.host_socket_tablet.send(rgb_size_str.encode('utf-8'))
                self.host_socket_tablet.sendall(rgb_to_bytes)
                self.host_socket_tablet.send(RGB_SEND_SUFFIX.encode('utf-8'))
                
                # ------ HEATMAP TO TABLET ------ 
                self.host_socket_tablet.send(HEATMAP_SEND_PREFIX.encode('utf-8'))
                self.host_socket_tablet.send(heatmap_size_str.encode('utf-8'))
                self.host_socket_tablet.sendall(heatmap_to_bytes)
                self.host_socket_tablet.send(HEATMAP_SEND_SUFFIX.encode('utf-8'))

                # ------ GPS TO TABLET ------ 
                self.host_socket_tablet.send(GPS_SEND_PREFIX.encode('utf-8'))
                self.host_socket_tablet.send(img_long_str.encode('utf-8'))
                self.host_socket_tablet.send(img_lat_str.encode('utf-8'))
                self.host_socket_tablet.send(img_alt_str.encode('utf-8'))
                self.host_socket_tablet.send(GPS_SEND_SUFFIX.encode('utf-8'))

                # ------ BUFFER TO TABLET ------ 
                self.host_socket_tablet.send(BUFFER_SIZE_PREFIX.encode('utf-8'))
                self.host_socket_tablet.send(buffer_size_str.encode('utf-8'))
                self.host_socket_tablet.send(BUFFER_SIZE_SUFFIX.encode('utf-8'))



                # ------ RPY TO TABLET -----
                self.host_socket_tablet.send(RPY_SEND_PREFIX.encode('utf-8'))
                self.host_socket_tablet.send(img_roll_str.encode('utf-8'))
                self.host_socket_tablet.send(img_pitch_str.encode('utf-8'))
                self.host_socket_tablet.send(img_yaw_str.encode('utf-8'))
                self.host_socket_tablet.send(RPY_SEND_SUFFIX.encode('utf-8'))

                # ------  HEALTH TO TABLET -----
                self.host_socket_tablet.send(HEALTH_SEND_PREFIX.encode('utf-8'))
                self.host_socket_tablet.send(str(health_metrics[0]+'\n').encode('utf-8'))
                self.host_socket_tablet.send(str(health_metrics[1]+'\n').encode('utf-8'))
                self.host_socket_tablet.send(str(health_metrics[2]+'\n').encode('utf-8'))
                self.host_socket_tablet.send(HEALTH_SEND_SUFFIX.encode('utf-8'))



                csv_timestamp = datetime.now(self.pst_tz)
                time_log = csv_timestamp.strftime("%H:%M:%S:%f")

                mask_dir_str = None
                heatmap_dir_str = None

                with self.mask_png:
                    mask_dir_str = self.mask_png_buffer.pop(0)


                with self.heatmap_png:
                    heatmap_dir_str = self.heatmap_png_buffer.pop(0)

                with self.log_cv:

                    self.cur_mask_filepath = mask_dir_str
                    self.cur_heatmap_filepath = heatmap_dir_str
                    self.cur_rgb_filepath = rgb_dir_str

                    self.cur_gps_lat = img_lat
                    self.cur_gps_long = img_long
                    self.cur_gps_alt = img_alt

                    self.cur_timestamp = time_log

                    self.log_ready = True
                    self.log_cv.notify()






        except Exception as e:
            self.get_logger().error(f"An unexpected error occured in send loop: {e}")
        finally:
            self.get_logger().warn("Send loop has finished")
            self.connected = False
            self.connected_tablet = False

            if self.host_socket is not None:
                self.host_socket.close()
                self.host_socket = None


            # if self.client_socket_tablet is not None:
            #     self.client_socket_tablet.close()
            #     self.client_socket_tablet = None

            if not self.shutdown_flag.is_set():
                self.threads_running = False
                self.connection_timer = self.create_timer(RECONNECT_TIMER_PERIOD, self.connect_to_client_callback)
                # self.connection_timer_tablet = self.create_timer(RECONNECT_TIMER_PERIOD, self.connect_to_tablet_callback)
                self.get_logger().info("Disconnected. Reconnection timer activated.")


    # def send_data_to_tablet(self):

    #    if not self.connected_tablet:
    #        return

    #    try:

    #        while not self.shutdown_flag.is_set():



    #            # -------- MASK -------





    #    except Exception as e:
    #        self.get_logger().error(f"An unexpected error occured in send loop: {e}")
    #    finally:
    #        self.get_logger().info("Send loop has finished")
    #        self.connected_tablet = False


    def run_processing_threads(self):

        if self.threads_running:
            return
        else:

            self.threads_running = True

            self.receive_thread = threading.Thread(target=self.receive_data_from_remote)
            self.receive_thread.start()

            # self.sending_thread = threading.Thread(target=self.send_data_to_remote)
            # self.sending_thread.start()

            # self.buffers_thread = threading.Thread(target=self.update_buffers)
            # self.buffers_thread.start()

            self.stitching_trigger_thread = threading.Thread(target=self.stitching_trigger)
            self.stitching_trigger_thread.start()

            self.logging_thread = threading.Thread(target=self.logging_callback)
            self.logging_thread.start()


            self.threads.append(self.receive_thread)
            # self.threads.append(self.sending_thread)
            # self.threads.append(self.buffers_thread)
            self.threads.append(self.stitching_trigger_thread)
            self.threads.append(self.logging_thread)




    def connect_to_client_callback(self):

        if self.connected:
            return

        try:
            # Create listening socket ONCE
            self.get_logger().info(f"Listening for client on port {PORT}...")

            self.host_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.host_socket.bind(("", PORT))
            self.host_socket.listen(1)
            self.host_socket.settimeout(1)

            conn, addr = self.host_socket.accept()

            self.conn = conn
            self.connected = True

            self.get_logger().info(
                f"Client connected from {addr[0]}:{addr[1]}"
            )

            self.run_processing_threads()
            self.connection_timer.cancel()

        except socket.timeout:
            print(f"\n[-] Listen timed out after {LISTEN_TIMEOUT} seconds.")
            print("[-] Client not connected yet.")
            pass 

        except Exception as e:
            self.get_logger().error(f"Accept failed: {e}")
            if hasattr(self, "tcp_socket"):
                self.connected = False
                self.host_socket.close()
                del self.host_socket




    def connect_to_tablet_callback(self):
        """
        This callback is executed by the timer to attempt a connection.
        """
        # If already connected, do nothing.
        if self.connected_tablet:
            return

        self.get_logger().info(f"Attempting to connect to tablet at {HOST_TABLET}:{PORT_TABLET}...")
        try:

            # Create a new socket and attempt to connect
            self.host_socket_tablet = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.host_socket_tablet.connect((HOST_TABLET, PORT_TABLET))
            
            # --- Connection Successful---
            self.connected_tablet = True
            self.get_logger().info("Successfully connected to the tablet!")

            # --- Refresh buffers ----
            # self.rgb_buffer.clear()
            # self.rgb_png_buffer.clear()
            # self.mask_buffer.clear()
            # self.mask_png_buffer.clear()
            # self.heatmap_buffer.clear()
            # self.heatmap_png_buffer.clear()
            # self.gps_buffer.clear()
            
            # Cancel the timer so it doesn't keep trying to connect
            # self.connection_timer.cancel()
            self.connection_timer_tablet.cancel()

            if self.connected and self.connected_tablet:
                self.run_processing_threads()
            else:
                self.connection_timer = self.create_timer(RECONNECT_TIMER_PERIOD, self.connect_to_client_callback)

            # You can now start other operations, e.g., create another timer to receive data
            # self.receive_thread = threading.Thread(target=self.receive_data_from_tablet)
            # self.receive_thread.start()

            # self.tablet_sending_thread = threading.Thread(target=self.send_data_to_tablet)
            # self.tablet_sending_thread.start()


            # self.threads.append(self.tablet_sending_thread)

            # self.buffers_thread = threading.Thread(target=self.update_buffers)
            # self.buffers_thread.start()

            # self.tablet_logging_thread = threading.Thread(target=self.logging_callback)
            # self.tablet_logging_thread.start()

            # From testing tablet tends to connect slower but this is not robust



        except ConnectionRefusedError:
            self.get_logger().warn("Connection failed. Tablet may not be running. Retrying...")
            # The timer will automatically try again on its next cycle.
            # self.client_socket_tablet.close() # Close the failed socket
            if self.host_socket_tablet is not None:
                self.host_socket_tablet.close()
                self.host_socket_tablet.close()
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred: {e}")
            # if self.client_socket_tablet:
            #     self.client_socket_tablet.close()
            if self.host_socket_tablet is not None:
                self.host_socket_tablet.close()
                self.host_socket_tablet = None

    def stop_all_threads(self):

        self.get_logger().warn("Stopping all Threads and setting shutdown flag")

        global global_shutdown_flag

        global_shutdown_flag.set()

        self.shutdown_flag.set()

        if self.buffer_cv:
            self.buffer_cv.acquire()
            self.buffer_cv.notify_all()
            self.buffer_cv.release()

        if self.log_cv:
            self.log_cv.acquire()
            self.log_cv.notify_all()
            self.log_cv.release()

        if self.csv_lock.locked(): 
            self.get_logger().warn("Releasing csv lock")
            self.csv_lock.release()

        if self.mask_png.locked():
            self.get_logger().warn("Releasing mask png lock")
            self.mask_png.release()

        if self.heatmap_png.locked():
            self.get_logger().warn("Releasing heatmap png lock")
            self.heatmap_png.release()

        if self.rgb_png.locked():
            self.get_logger().warn("Releasing rgb png lock")
            self.rgb_png.release()


        if self.rgb_buffer_lock.locked():
            self.get_logger().warn("Releasing rgb buffer lock")
            self.rgb_buffer_lock.release()

        if self.heatmap_buffer_lock.locked():
            self.get_logger().warn("Releasing heatmap buffer lock")
            self.heatmap_buffer_lock.release()

        if self.mask_buffer_lock.locked():
            self.get_logger().warn("Releasing mask buffer lock")
            self.mask_buffer_lock.release()

        if self.time_buffer_lock.locked():
            self.get_logger().warn("Releasing time buffer lock")
            self.time_buffer_lock.release()

        if self.buffer_mutex.locked():
            self.buffer_mutex.release()


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