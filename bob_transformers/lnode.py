#
# Copyright 2024 BobRos
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

from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState
from rclpy.lifecycle.node import TransitionCallbackReturn

class LNode(LifecycleNode):
    """
    Simple LifecycleNode default wrapper.
    It can be used as a replacement for the standard rclpy.Node to turn any 
    existing normal ROS Node into a lifecycle ROS node. Handles the basic 
    LifecycleNode events `configure`, `cleanup"` and  `shutdown`.
    The functions `configure` and `destroy` have to be overidden in order to 
    use it. Everything important in the rclpy.Node __init__ function need to 
    go into overidden `configure` function. In the overidden `destroy` 
    function resources, like subscribers etc,  have to be released.
    """
    def __init__(self, node_name, *, enable_communication_interface: bool = True, **kwargs):
        """
        Create a default lifecycle node.

        See rclpy.lifecycle.LifecycleNodeMixin and rclpy.node.Node()
        for the documentation of each parameter.
        """
        LifecycleNode.__init__(self, node_name, 
            enable_communication_interface=enable_communication_interface, **kwargs)

    def configure(self):
        raise NotImplementedError

    def destroy(self):
        raise NotImplementedError

    def on_configure(self, previous_state: LifecycleState):
        self.get_logger().info("on_configure")
        self.configure()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, previous_state: LifecycleState):
        self.get_logger().info("on_cleanup")
        self.destroy()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, previous_state: LifecycleState):
        self.get_logger().info("on_shutdown")
        self.destroy()
        return TransitionCallbackReturn.SUCCESS
