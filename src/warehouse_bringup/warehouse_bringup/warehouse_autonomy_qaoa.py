#!/usr/bin/env python3

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Int32, String


def yaw_to_quaternion(yaw_deg):
    yaw_rad = math.radians(yaw_deg)
    qz = math.sin(yaw_rad / 2.0)
    qw = math.cos(yaw_rad / 2.0)
    return qz, qw


class WarehouseAutonomyQAOA(Node):
    """Enhanced warehouse autonomy node that uses QAOA-optimized waypoints."""

    def __init__(self):
        super().__init__('warehouse_autonomy_qaoa')

        self.navigator = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.marker_detected = False
        self.marker_id = None
        self.marker_id_stamp = None

        # Subscriptions for vision
        self.create_subscription(Bool, '/warehouse/marker_detected', self.marker_detected_cb, 10)
        self.create_subscription(Int32, '/warehouse/marker_id', self.marker_id_cb, 10)

        # Subscription for QAOA-optimized waypoints
        self.create_subscription(String, '/warehouse/optimized_waypoints', self.optimized_waypoints_cb, 10)

        # Station definitions (fixed positions)
        self.stations = {
            0: {
                'name': 'Home Charging Dock',
                'task': 'Verify robot home dock and charging station',
                'x': -3.678,
                'y': 9.3,
                'yaw': 90.0,
                'marker_id': 0,
            },
            1: {
                'name': 'Loading Zone',
                'task': 'Verify loading area',
                'x': 1.803,
                'y': 9.3,
                'yaw': 90.0,
                'marker_id': 1,
            },
            2: {
                'name': 'Inventory Scan Station',
                'task': 'Verify shelf inventory using wall marker',
                'x': 1.801,
                'y': -9.0,
                'yaw': -90.0,
                'marker_id': 2,
            },
            3: {
                'name': 'Dispatch Zone',
                'task': 'Verify dispatch area',
                'x': -3.630,
                'y': -9.3,
                'yaw': -90.0,
                'marker_id': 3,
            },
        }

        # Default order (can be overridden by QAOA)
        self.waypoints = [
            self.stations[0],  # Charging Dock
            self.stations[1],  # Loading Zone
            self.stations[2],  # Inventory Scan
            self.stations[3],  # Dispatch Zone
            self.stations[0],  # Return to Charging Dock
        ]

        self.current_index = 0
        self.successful_tasks = 0
        self.failed_tasks = 0
        self.use_qaoa = False  # Flag to use QAOA-optimized waypoints

        self.get_logger().info('Warehouse autonomy with QAOA support started.')
        self.get_logger().info('Waiting for Nav2 action server...')

        self.navigator.wait_for_server()

        self.get_logger().info('Nav2 is ready. Starting warehouse mission in 3 seconds...')
        time.sleep(10)

        # Check if QAOA waypoints are available, otherwise use default
        if not self.use_qaoa:
            self.get_logger().info('Using default waypoint order (QAOA not yet available)')

        self.send_next_goal()

    def marker_detected_cb(self, msg):
        self.marker_detected = msg.data

    def marker_id_cb(self, msg):
        self.marker_id = msg.data
        self.marker_id_stamp = self.get_clock().now()

    def optimized_waypoints_cb(self, msg):
        """Callback for QAOA-optimized waypoints."""
        try:
            waypoints_data = json.loads(msg.data)
            self.get_logger().info(f'Received QAOA-optimized waypoints: {waypoints_data}')

            # Rebuild waypoints from optimized order
            new_waypoints = []
            for wp_data in waypoints_data:
                station_id = wp_data.get('station_id')
                if station_id is None:
                    self.get_logger().error(f'Missing station_id in waypoint data: {wp_data}')
                    return
                if station_id not in self.stations:
                    self.get_logger().error(f'Invalid station_id: {station_id}')
                    return
                new_waypoints.append(self.stations[station_id])

            # Add return to charging dock at the end if not already there
            if len(new_waypoints) > 0 and new_waypoints[-1].get('station_id', 0) != 0:
                new_waypoints.append(self.stations[0])

            self.waypoints = new_waypoints
            self.use_qaoa = True
            self.get_logger().info(f'Waypoints updated with QAOA optimization: {len(new_waypoints)} waypoints')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON decode error: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Failed to parse QAOA waypoints: {str(e)}')

    def send_next_goal(self):
        if self.current_index >= len(self.waypoints):
            self.get_logger().info('========================================')
            self.get_logger().info('WAREHOUSE MISSION COMPLETE')
            self.get_logger().info(f'Total mission steps: {len(self.waypoints)}')
            self.get_logger().info(f'Successful tasks: {self.successful_tasks}')
            self.get_logger().info(f'Failed tasks: {self.failed_tasks}')
            self.get_logger().info(f'QAOA optimization used: {self.use_qaoa}')
            self.get_logger().info('========================================')
            return

        wp = self.waypoints[self.current_index]

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()

        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = wp['x']
        pose.pose.position.y = wp['y']
        pose.pose.position.z = 0.0

        qz, qw = yaw_to_quaternion(wp['yaw'])
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal_msg.pose = pose

        self.get_logger().info('----------------------------------------')
        self.get_logger().info(f'MISSION STEP {self.current_index + 1}/{len(self.waypoints)}')
        self.get_logger().info(
            f"Navigating to {wp['name']}: x={wp['x']}, y={wp['y']}, yaw={wp['yaw']} deg"
        )

        future = self.navigator.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2.')
            self.failed_tasks += 1
            self.current_index += 1
            self.send_next_goal()
            return

        self.get_logger().info('Goal accepted by Nav2.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        wp = self.waypoints[self.current_index]
        status = future.result().status

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"Navigation FAILED for {wp['name']} (status: {status})"
            )
            self.failed_tasks += 1
            self.current_index += 1
            time.sleep(2)
            self.send_next_goal()
            return

        self.get_logger().info(f"ARRIVED: {wp['name']}")

        verified = self.verify_station(wp)

        if verified:
            self.get_logger().info(f"VISION VERIFIED: {wp['name']}")
            self.get_logger().info(f"TASK: {wp['task']}")
            time.sleep(3)
            self.get_logger().info(f"TASK COMPLETE: {wp['name']}")
            self.successful_tasks += 1
        else:
            self.get_logger().error(f"VISION VERIFICATION FAILED: {wp['name']}")
            self.failed_tasks += 1

        self.current_index += 1
        time.sleep(2)
        self.send_next_goal()

    def verify_station(self, wp):
        expected_marker = wp['marker_id']

        if expected_marker is None:
            self.get_logger().info('No visual marker required for this waypoint.')
            return True

        arrival_stamp = self.get_clock().now()
        self.get_logger().info(f'Looking for ArUco marker ID {expected_marker}...')

        start_time = time.time()
        timeout = 15.0

        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)

            if (self.marker_id == expected_marker and
                    self.marker_id_stamp is not None and
                    self.marker_id_stamp >= arrival_stamp):
                self.get_logger().info(
                    f'Correct marker detected: expected={expected_marker}, detected={self.marker_id}'
                )
                return True

        self.get_logger().error(
            f'Marker verification timeout: expected={expected_marker}, last_detected={self.marker_id}'
        )
        return False


def main(args=None):
    rclpy.init(args=args)
    node = WarehouseAutonomyQAOA()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
