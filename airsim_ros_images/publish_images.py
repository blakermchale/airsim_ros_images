#!/usr/bin/env python3
from airsim import MultirotorClient, ImageType, ImageRequest, ImageResponse
import os
import json

import rclpy
from rclpy.node import Node
from tf2_ros.transform_broadcaster import TransformBroadcaster

from sensor_msgs.msg import Image
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion, TransformStamped


class ImagePublisher(Node):
    def __init__(self, rate=60):
        super().__init__("airsim_images")

        # Create timer for calling publish at predefined rate
        self.create_timer(1/rate, self.publish)

        # AirSim variables
        self._airsim_client = MultirotorClient(ip=os.environ["WSL_HOST_IP"])
        self._camera_name = "bottom_center"  # front_center

        # ROS Publishers
        self._pub_ir = self.create_publisher(Image, "ir/image_raw", 1)
        self._pub_rgb = self.create_publisher(Image, "color/image_raw", 1)
        self._pub_depth = self.create_publisher(Image, "depth/image_raw", 1)

        # TF related variables
        self.br = TransformBroadcaster(self)

        self.get_logger().info("Initialized image publisher")

    def publish(self):
        """Publish images from AirSim to ROS"""
        responses = self._airsim_client.simGetImages([
            # uncompressed RGB array bytes
            ImageRequest(self._camera_name, ImageType.Scene, compress=False),
            # # floating point uncompressed image
            # ImageRequest(self._camera_name, ImageType.DepthPlanar, pixels_as_float=True, compress=False),
            # infrared uncompressed image
            ImageRequest(self._camera_name, ImageType.Infrared, compress=False),
        ])
        rgb_response = responses[0]
        ir_response = responses[1]

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        # TODO: implement parameter for frame id, also decide on if each separate image type should have a different frame id
        #  This may mean we should load the ids via ros parameters
        header.frame_id = "camera"

        image_rgb = construct_image(header, rgb_response, "rgb8")
        image_ir = construct_image(header, ir_response, "rgb8")

        # TODO: use camera pose from airsim
        tfmsg = TransformStamped()
        translation = Vector3(x=0., y=0., z=0.)
        tfmsg.transform.translation = translation
        tfmsg.transform.rotation = Quaternion(x=0., y=0., z=0., w=1.)
        tfmsg.child_frame_id = "camera"
        tf_header = Header()
        tf_header.stamp = header.stamp
        tfmsg.header = tf_header
        tfmsg.header.frame_id = "world"
        self.br.sendTransform(tfmsg)

        
        self._pub_ir.publish(image_ir)
        self._pub_rgb.publish(image_rgb)
        # self._pub_depth.publish(image_depth)


def construct_image(header: Header, response: ImageResponse, encoding: str) -> Image:
    msg = Image()
    msg.header = header
    msg.encoding = encoding
    msg.height = response.height
    msg.width = response.width
    msg.data = response.image_data_uint8
    msg.is_bigendian = 0
    msg.step = response.width * 3
    return msg


def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)


if __name__=="__main__":
    main()
