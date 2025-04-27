#!/usr/bin/env python3
#
# Copyright 2025 BobRos
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

"""This ROS Node integrates `Huggingface Transformers` capabilities into `ROS`.
"""

import os
import json
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from bob_msgs.msg import TTImage
from transformers import pipeline
from bob_transformers.base import BaseNode

class ITTNode(BaseNode):
    """Basic image to text ROS Node
    """
    def __init__(self):
        super().__init__('itt')

        # https://huggingface.co/tasks/image-to-text
        # Salesforce/blip-image-captioning-base

        # https://huggingface.co/tasks/image-text-to-text
        # llava-hf/llava-interleave-qwen-0.5b-hf

        default_model_id = 'llava-hf/llava-interleave-qwen-0.5b-hf'

        self.declare_parameters(
            namespace='',
            parameters=[

                ('model_id', os.getenv('ITT_MODEL_ID', default_model_id),
                ParameterDescriptor(description=
                'The Huggingface model_id to be used. Environment variable '
                f'ITT_MODEL_ID. Default: {default_model_id}')),

                ('max_new_tokens', int(os.getenv('ITT_MAX_NEW_TOKEN', '64')),
                ParameterDescriptor(description=
                'Max tokens to respond. Environment variable '
                'ITT_MAX_NEW_TOKEN. Default: 64')),
            ])

        self.pub = self.add_publisher(
            String, 'itt_out')
        
        if not os.getenv('LLM_LIFECYCLE_NODE', default=False):
            self.configure()

    def configure(self):
        """Handle configure for lifecycle node."""
        self.add_subscriber(
            String, "messages", 
            self.messages_callback)
        self.pipeline = pipeline(
            "image-text-to-text", 
            model=self.get_parameter(
                "model_id").value)

    def messages_callback(self, msg: String):
        """String message callback.


        `Example: Passing message array with one or more messages`::

            [
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "image", 
                            "path": "/path/to/image.png"},
                        {
                            "type": "text", 
                            "text": "What do you see on that image"},
                        ],
                }
            ]

        `Example: Passing as dict and configure max token`::

            {
                "max_new_tokens": 128,
                "messages": [
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "image", 
                                "path": "/path/to/image.png"
                            },
                            {
                                "type": "text", 
                                "text": "What do you see on that image"
                            }
                        ]
                    }
                ]
            }


        `Example: Passing as simple string <image-path> [<num-max-tokens>] <text>`::

            /path/to/image.png What do you see on that image
            /path/to/image.png 128 What do you see on that image

        Args:
            msg (String): The received topic message.

        Raises:
            Exception: raises a
        """
        try:
            self.get_logger().debug(f"{msg.data}")
            max_new_tokens = self.get_parameter(
                'max_new_tokens').value

            try:
                obj = json.loads(msg.data)
            except:
                obj = msg.data
            
            if isinstance(obj, list):
                for message in obj:
                    response = self.pipeline(
                        text=[message], 
                        max_new_tokens=max_new_tokens)
                    self.get_logger().info(
                        str(response))

            elif isinstance(obj, dict):
                if 'max_new_tokens' in obj:
                    max_new_tokens = obj['max_new_tokens']
                for message in obj['data']:
                    response = self.pipeline(
                        text=[message], 
                        max_new_tokens=max_new_tokens)
                    self.pub.publish(
                        String(data=str(response)))
                    self.get_logger().info(
                        str(response))

            elif isinstance(obj, str) and \
                len(msg.data.split()) > 1:
                if msg.data.split()[1].isdigit():
                    if len(msg.data.split()) < 3:
                        raise Exception(
                            f"To less input data: {msg.data}")
                    max_new_tokens = int(
                        msg.data.split()[1])
                    words = msg.data.split()[2:]
                else:
                    words = msg.data.split()[1:]

                message = {
                    "role": "user",
                    "content": [
                        {
                            "type": "image", 
                            "path": msg.data.split()[0]},
                        {
                            "type": "text", 
                            "text": " ".join(words)}
                    ]
                }

                response = self.pipeline(
                    text=[message], 
                    max_new_tokens=max_new_tokens)
                self.pub.publish(
                    String(data=str(response)))
                self.get_logger().info(
                    str(response))

            else:
                raise Exception(
                    f"Invalid input format: {msg.data}")

        except Exception as e:
            self.get_logger().error(
                f"messages_callback: exception: {e}")

def main(args=None):
    rclpy.init(args=args)
    n = ITTNode()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
