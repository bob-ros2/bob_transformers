#!/usr/bin/env python3

import sys
import os
import logging
import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from std_msgs.msg import Header
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from bob_msgs.msg import TTImage
import numpy as np
import cv2

from diffusers import StableDiffusionPipeline
from diffusers import StableDiffusionXLPipeline
import torch

class TTINode(Node):

    def __init__(self):
        super().__init__('tti')

        logging.basicConfig(
            level = (logging.DEBUG
                if self.get_logger().get_effective_level() \
                    == LoggingSeverity.DEBUG \
                else logging.INFO),
            format="[%(levelname)s] [%(asctime)s.] [%(name)s]: %(message)s",
            datefmt="%s")

        self.declare_parameters(
            namespace='',
            parameters=[
                # CompVis/stable-diffusion-v1-4
                # stabilityai/stable-diffusion-2
                # stabilityai/stable-diffusion-xl-base-1.0
                ('model_id', 'stabilityai/stable-diffusion-xl-base-1.0'),
                ('pipeline_type', 'StableDiffusionXLPipeline'),
                ('show', True),
                ('result_image', os.getenv('TTI_RESULT_IMAGE', '')),
                ('frame_rate', 1)
            ])

        self.show = self.get_parameter(
            'show').get_parameter_value().bool_value
        self.result_image = self.get_parameter(
            'result_image').get_parameter_value().string_value
        self.model_id = self.get_parameter(
            'model_id').get_parameter_value().string_value
        self.pipeline_type = self.get_parameter(
            'pipeline_type').get_parameter_value().string_value
        
        self.pub_image = self.create_publisher(
            Image, 'image_raw', 10)

        self.pub_tti_image = self.create_publisher(
            TTImage, 'tti_image', 10)

        self.timer = self.create_timer(
            1.0/self.get_parameter('frame_rate').value, 
            self.timer_callback)

        self.sub = self.create_subscription(
            String, "tti_in", self.input_callback, 10)

        self.bridge = CvBridge()
        self.image_msg = []
        self.cv_image = []

        if self.result_image:
            try:
                self.cv_image = cv2.imread(self.result_image)
                self.image_msg = self.bridge.cv2_to_imgmsg(
                    np.array(self.cv_image), 
                    encoding = "bgr8", 
                    header = Header(
                        stamp = self.get_clock().now().to_msg(), 
                        frame_id = self.get_name()))
            except: 
                self.cv_image = []
        try:

            if self.pipeline_type == 'StableDiffusionXLPipeline':
                self.pipe = StableDiffusionXLPipeline.from_pretrained(
                    self.model_id, 
                    torch_dtype=torch.float16, 
                    use_safetensors=True, 
                    variant="fp16")
            elif self.pipeline_type == 'StableDiffusionPipeline':
                self.pipe = StableDiffusionPipeline.from_pretrained(
                    self.model_id, 
                    torch_dtype=torch.float16, 
                    use_safetensors=True, 
                    variant="fp16")
            else:
                self.get_logger().error(
                    f"error: unkown pipeline_type: {self.pipeline_type}")
                sys.exit(1)

            self.pipe.to('cuda')

        except Exception as e:
            self.get_logger().error(f'exception: {e}')
            sys.exit(1)

    def timer_callback(self):
        if not len(self.cv_image): return

        if self.pub_image.get_subscription_count():
            self.pub_image.publish(self.image_msg)
            self.get_logger().debug('published image')

        if self.show and len(self.cv_image):
            cv2.imshow("TTI", self.cv_image)
            cv2.waitKey(1)

    def input_callback(self, msg):        
        self.text2image(msg.data)

    def text2image(self, prompt):
        try:

            image = self.pipe(prompt).images[0]
            
            self.result_image and image.save(self.result_image)

            # convert RGB to BGR
            np_image = np.array(image)
            self.cv_image = np_image[:, :, ::-1].copy()

            # create an image message for the publishers 
            self.image_msg = self.bridge.cv2_to_imgmsg(
                np.array(self.cv_image),
                encoding = "bgr8",
                header = Header(
                    stamp = self.get_clock().now().to_msg(), 
                    frame_id = self.get_name()))

            # publish image only
            if self.pub_image.get_subscription_count():
                self.pub_image.publish(self.image_msg)
                self.get_logger().debug('published Image')

            # publish tti image with caption text
            if self.pub_tti_image.get_subscription_count():
                tti_msg = TTImage()
                tti_msg.image = self.image_msg
                tti_msg.caption = prompt
                self.pub_tti_image.publish(tti_msg)
                self.get_logger().debug('published TTImage')

            self.result_image and self.get_logger().info(
                f"image saved: {self.result_image}")

        except Exception as e:
            self.get_logger().error(
                f"exception: {e}")


def main(args=None):
    rclpy.init(args=args)
    n = TTINode()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
