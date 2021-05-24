#!/usr/bin/env python3
from airsim import MultirotorClient, ImageType, ImageRequest, ImageResponse
from airsim import CameraInfo as SimCameraInfo
import os
import json
import numpy as np

import rclpy
from rclpy.node import Node
from tf2_ros.transform_broadcaster import TransformBroadcaster
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion, TransformStamped


class ImagePublisher(Node):
    def __init__(self, rate=60):
        super().__init__("airsim_images")

        # Create timer for calling publish at predefined rate
        self.create_timer(1/rate, self.publish)

        # AirSim variables
        self._airsim_client = MultirotorClient(ip=os.environ["WSL_HOST_IP"])
        # self._camera_name = "front_center"
        self._camera_name = "bottom_center"
        self._camera_frame_id = "realsense"
        self._vehicle_name = self.get_namespace().split("/")[1]

        # ROS Publishers
        # self._pub_ir = self.create_publisher(Image, "ir/image_raw", 1)
        self._pub_color = self.create_publisher(Image, "color/image_raw", 1)
        # self._pub_depth = self.create_publisher(Image, "depth/image_raw", 1)
        # self._pub_info_ir = self.create_publisher(CameraInfo, "ir/camera_info", 1)
        self._pub_info_color = self.create_publisher(CameraInfo, "color/camera_info", 1)
        # self._pub_depth = self.create_publisher(Image, "depth/image_raw", 1)

        # TF related variables
        self.br = TransformBroadcaster(self)

        # CV
        self.bridge = CvBridge()

        # Internal variables
        self._cam_info_msgs = {}
        
        self.get_logger().info("Initialized image publisher")

    def publish(self):
        """Publish images from AirSim to ROS"""
        responses = self._airsim_client.simGetImages([
            # uncompressed RGB array bytes
            ImageRequest(self._camera_name, ImageType.Scene, compress=False),
            # # infrared uncompressed image
            # ImageRequest(self._camera_name, ImageType.Infrared, compress=False),
            # # floating point uncompressed image
            # ImageRequest(self._camera_name, ImageType.DepthPlanner, pixels_as_float=True, compress=False),
        ], self._vehicle_name)
        color_response = responses[0]
        # ir_response = responses[1]
        # depth_response = responses[2]

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        # TODO: implement parameter for frame id, also decide on if each separate image type should have a different frame id
        #  This may mean we should load the ids via ros parameters
        header.frame_id = self._camera_frame_id

        # Handle cam info it has not been found yet
        if self._vehicle_name not in self._cam_info_msgs.keys():
            self._cam_info_msgs[self._vehicle_name] = {}
            cam_info = self._airsim_client.simGetCameraInfo(self._camera_name, self._vehicle_name)
            self.get_logger().info(f"{cam_info.proj_mat}")
            # TODO: implement multiple cameras for each lens on realsense and update this method
            self._cam_info_msgs[self._vehicle_name]["color"] = construct_info(header, cam_info, color_response.height, color_response.width)
            # self._cam_info_msgs[self._vehicle_name]["ir"] = self._cam_info_msgs[self._vehicle_name]["color"]

        image_color = construct_image(header, color_response, "bgr8")
        # image_ir = construct_image(header, ir_response, "rgb8")
        # image_depth = construct_image(header, depth_response, "rgb8")

        # TODO: use camera pose from airsim
        tfmsg = TransformStamped()
        translation = Vector3(x=0., y=0., z=0.)
        tfmsg.transform.translation = translation
        tfmsg.transform.rotation = Quaternion(x=0., y=0., z=0., w=1.)
        tfmsg.child_frame_id = self._camera_frame_id
        tf_header = Header()
        tf_header.stamp = header.stamp
        tfmsg.header = tf_header
        tfmsg.header.frame_id = "world"
        self.br.sendTransform(tfmsg)

        self._pub_color.publish(image_color)
        # self._pub_ir.publish(image_ir)
        # self._pub_depth.publish(image_depth)
        self._pub_info_color.publish(self._cam_info_msgs[self._vehicle_name]["color"])
        # self._pub_info_ir.publish(self._cam_info_msgs[self._vehicle_name]["ir"])


def construct_info(header: Header, info: SimCameraInfo, height: int, width: int) -> CameraInfo:
    msg = CameraInfo()

    Tx = 0.0  # Assumed for now since we are not using stereo
    hfov = info.fov
    print(info.proj_mat)

    # https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/issues/86
    f = width / (2 * np.tan(0.5 * hfov))
    Fx = Fy = f
    cx = width / 2
    cy = height / 2

    K = np.array([
        [Fx,  0.0, cx],
        [0.0, Fy,  cy],
        [0.0, 0.0, 1 ]
    ]).flatten()

    R = np.array([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]
    ]).flatten()

    P = np.array([
        [Fx,  0.0, cx,  Tx ],
        [0.0, Fy,  cy,  0.0],
        [0.0, 0.0, 1.0, 0.0]
    ]).flatten()

    msg.header = header
    msg.height = height
    msg.width = width
    msg.k = K
    msg.r = R
    msg.p = P
    msg.binning_x = 0
    msg.binning_y = 0

    return msg


def construct_image(header: Header, response: ImageResponse, encoding: str) -> Image:
    msg = Image()
    msg.header = header
    msg.encoding = encoding
    msg.height = response.height
    msg.width = response.width
    msg.data = response.image_data_uint8 if response.image_type != ImageType.DepthPlanner else response.image_data_float
    msg.is_bigendian = 0
    msg.step = response.width * 3
    return msg


def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)


if __name__=="__main__":
    main()
