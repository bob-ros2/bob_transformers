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

"""Base ROS Node for package bob_transformers.
"""

import os
import json
import logging
from typing import Any
from std_msgs.msg import Header
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from bob_transformers.lnode import LNode

if int(os.getenv('TRANSFORMERS_LIFECYCLE_NODE', default='0')):
    class Base(LNode):
        """Lifecycle ROS Node"""
else:
    class Base(Node):
        """Standard ROS Node"""

class BaseNode(Base):
    def __init__(self, node_name):
        """
        Base ROS Node for all nodes from package bob_transformers.

        It can be used as standard rclpy.Node or as rclpy.lifecycle.LifecycleNode. 
        See bob_transformers.lnode.LNode for more details.
        To activate LifecycleNode set environment variable TRANSFORMERS_LIFECYCLE_NODE=1

        Args:
            node_name (str): The name of the ROS node
            enable_communication_interface (bool, optional): Enable communication. Defaults to True.
        """
        super().__init__(node_name)

        logging.basicConfig(
            level = (logging.DEBUG
                if self.get_logger().get_effective_level() \
                    == LoggingSeverity.DEBUG \
                else logging.INFO),
            format="[%(levelname)s] [%(asctime)s.] [%(name)s]: %(message)s",
            datefmt="%s")

        self.publisher = list()
        self.subscriber = list()

    def destroy(self):
        """Handle destroy for lifecycle node."""
        for sub in self.subscriber:
            self.destroy_subscription(sub)
        self.subscriber.clear()

    def add_subscriber(self, 
        class_type: Any, 
        topic: str, 
        callback, 
        queue_size: int=100):
        """Adds a subscriber.

        Args:
            class_type (Any): Message class of the topic.
            topic (str): The topic location.
            callback (function): Callback function which receives the message.
            queue_size (int, optional): Queue size. Defaults to 100.

        Returns:
            Subscriber: The created subscriber instance.
        """
        sub = self.create_subscription(
            class_type, topic, callback, queue_size)
        self.subscriber.append(sub)
        return sub
    
    def add_publisher(self, 
        class_type: Any, 
        topic: str, 
        queue_size: int=100):
        """Adds a publisher.

        Args:
            class_type (Any): Class type of any ROS topic message type.
            topic (str): The ROS topic name
            queue_size (int, optional): Queue size. Defaults to 100.

        Returns:
            Publisher: The created publisher instance.
        """
        pub = self.create_publisher(
            class_type, topic, queue_size)
        self.publisher.append(pub)
        return pub

    def jsonfy(
        self, data: Any, 
        metadata: list=None, 
        tags: list=None) -> str:
        """
        Formats a json object. 
        The keys 'stamp', 'frame_id' and 'tags' are auto filled with the node 
        name and the creation timestamp. The used data must be JSON compatible.
        The fields 'metadata' or 'tags' can be overwritten, if both are provided 
        'metadata' overwrites 'tags'.


        `Example of the produced JSON`::

            {
                "metadata": [
                    {"key": "stamp", "value": 1432343.034742467},
                    {"key": "frame_id", "value": "a frame ID"},
                    {"key": "tags", "value": ["tagname1", "another tag"]}
                    <additional key/value entries ...>
                ],
                "data": <any kind of data>
            }

        Args:
            data (Any): Data to add
            metadata (list, optional): If provided it superseeds default metadata. Defaults to None.
            tags (list, optional): If provided it overwrites default metadata['tags']. If metadata is provide this paramater has no effect. Defaults to None.

        Returns:
            str: The resulting json String
        """
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.get_name()
        return json.dumps({
            "metadata": metadata or [
                {"key": "stamp", "value": float("%d.%09d" 
                    % (header.stamp.sec, header.stamp.nanosec))},
                {"key": "frame_id", "value": header.frame_id},
                {"key": "'tags'", "value": tags or [f"{self.get_name()}"]}
            ],
            "data": data
        })
