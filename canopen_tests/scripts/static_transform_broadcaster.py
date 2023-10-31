from typing import Optional
from typing import Union
from typing import List

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


class StaticTransformBroadcaster(Node):

    def __init__(self, node_name='transform_publisher', qos: Optional[Union[QoSProfile, int]] = None):
        super().__init__(node_name=node_name)

        if qos is None:
            qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                )
        self.pub_tf = self.create_publisher(TFMessage, "/tf_static", qos)

    def sendTransform(self, transform: Union[TransformStamped, List[TransformStamped]]) -> None:
        if not isinstance(transform, list):
            if hasattr(transform, '__iter__'):
                transform = list(transform)
            else:
                transform = [transform]
        self.pub_tf.publish(TFMessage(transforms=transform))