#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from builtin_interfaces.msg import Duration


class BodyMarker(Node):
    def __init__(self):
        super().__init__('body_marker')
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        # Publish at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_marker)

    def publish_marker(self):
        marker = Marker()
        
        # Anchor the marker to the body frame
        marker.header = Header(
            frame_id='body',
            stamp=rclpy.time.Time().to_msg()
        )
        
        marker.ns = 'body_marker'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Position at the origin of the body frame
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        # Scale (size of the sphere)
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25

        # Color: semi-transparent red
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

        # Lifetime: persists until overwritten
        marker.lifetime = Duration(sec=0, nanosec=0)

        self.publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = BodyMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
