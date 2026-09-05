#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from aruco_opencv_msgs.msg import ArucoDetection


class ArucoBridgeNode(Node):
    """Bridge node that converts aruco_opencv topics to warehouse topics."""

    def __init__(self):
        super().__init__('aruco_bridge')

        # Publishers for warehouse topics
        self.marker_detected_pub = self.create_publisher(Bool, '/warehouse/marker_detected', 10)
        self.marker_id_pub = self.create_publisher(Int32, '/warehouse/marker_id', 10)

        # Subscribe to aruco_opencv topics
        # aruco_tracker publishes to /aruco_tracker/markers
        self.create_subscription(
            ArucoDetection,
            '/aruco_tracker/markers',
            self.aruco_marker_cb,
            10
        )

        self.get_logger().info('Aruco Bridge node started')
        self.get_logger().info('Subscribing to /aruco_tracker/markers and publishing to warehouse topics')
        self.get_logger().info('Waiting for aruco_tracker to publish markers...')

    def aruco_marker_cb(self, msg):
        """Callback for aruco marker detection."""
        # Check if any markers are detected
        marker_detected = len(msg.poses) > 0
        
        # Publish marker detection status
        detected_msg = Bool()
        detected_msg.data = marker_detected
        self.marker_detected_pub.publish(detected_msg)

        # If markers detected, publish the first marker's ID
        if marker_detected:
            marker_id_msg = Int32()
            marker_id_msg.data = msg.poses[0].marker_id
            self.marker_id_pub.publish(marker_id_msg)
            
            self.get_logger().info(f'Marker detected: ID={marker_id_msg.data}')
        else:
            # Publish -1 when no marker detected
            marker_id_msg = Int32()
            marker_id_msg.data = -1
            self.marker_id_pub.publish(marker_id_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
