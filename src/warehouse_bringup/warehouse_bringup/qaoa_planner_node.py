#!/usr/bin/env python3

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class QAOAPlanner(Node):
    """ROS2 node that uses optimization heuristics to optimize warehouse station visitation order.
    
    Note: This is a simplified version that uses classical optimization heuristics.
    For full QAOA integration, install the qaoa_training_pipeline package with all dependencies.
    """

    def __init__(self):
        super().__init__('qaoa_planner')

        # Publisher for optimized waypoints
        self.waypoint_pub = self.create_publisher(
            String,
            '/warehouse/optimized_waypoints',
            10
        )

        # Timer to periodically re-optimize (e.g., when inventory changes)
        self.timer = self.create_timer(30.0, self.optimize_and_publish)

        self.get_logger().info('QAOA Planner node initialized (using classical heuristics)')

    def create_warehouse_graph(self):
        """Create a graph representation of warehouse stations with distances."""
        
        # Station positions from your warehouse_autonomy_node.py
        stations = {
            0: {'name': 'Charging Dock', 'x': -3.678, 'y': 9.3},
            1: {'name': 'Loading Zone', 'x': 1.803, 'y': 9.3},
            2: {'name': 'Inventory Scan', 'x': 1.801, 'y': -9.0},
            3: {'name': 'Dispatch Zone', 'x': -3.630, 'y': -9.3},
        }

        # Calculate distances between all pairs
        edge_list = []
        for i in stations:
            for j in stations:
                if i < j:  # Avoid duplicates
                    dist = ((stations[i]['x'] - stations[j]['x'])**2 + 
                            (stations[i]['y'] - stations[j]['y'])**2)**0.5
                    edge_list.append({
                        "nodes": [i, j],
                        "weight": -dist  # Negative for maximization (shorter distance = higher weight)
                    })

        return {
            "edge list": edge_list,
            "Description": "Warehouse station distance graph for QAOA optimization"
        }

    def optimize_with_qaoa(self):
        """Use optimization heuristics to find optimal station visitation order.
        
        This implements a TSP (Traveling Salesman Problem) solver using:
        1. Nearest neighbor heuristic for initial solution
        2. 2-opt local search for improvement
        
        For full QAOA integration, this would call the qaoa_training_pipeline.
        """
        
        # Create the problem graph
        graph_data = self.create_warehouse_graph()
        
        # Extract distances
        distances = {}
        for edge in graph_data["edge list"]:
            i, j = edge["nodes"]
            distances[(i, j)] = -edge["weight"]  # Convert back to positive
            distances[(j, i)] = distances[(i, j)]  # Symmetric

        # Nearest neighbor heuristic
        unvisited = {0, 1, 2, 3}
        current = 0  # Start from charging dock
        order = [current]
        unvisited.remove(current)

        while unvisited:
            nearest = min(unvisited, key=lambda x: distances.get((current, x), float('inf')))
            order.append(nearest)
            unvisited.remove(nearest)
            current = nearest

        # Return to charging dock
        order.append(0)

        # 2-opt local search improvement
        improved = True
        iterations = 0
        max_iterations = 100
        
        while improved and iterations < max_iterations:
            improved = False
            iterations += 1
            
            for i in range(1, len(order) - 2):
                for j in range(i + 1, len(order) - 1):
                    # Calculate current distance
                    current_dist = (
                        distances.get((order[i-1], order[i]), float('inf')) +
                        distances.get((order[j], order[j+1]), float('inf'))
                    )
                    
                    # Calculate new distance if we swap
                    new_dist = (
                        distances.get((order[i-1], order[j]), float('inf')) +
                        distances.get((order[i], order[j+1]), float('inf'))
                    )
                    
                    if new_dist < current_dist:
                        # Perform 2-opt swap
                        order[i:j+1] = order[j:i-1:-1]
                        improved = True

        self.get_logger().info(f'Optimization completed in {iterations} iterations')
        return order

    def optimize_and_publish(self):
        """Run QAOA optimization and publish results."""
        
        self.get_logger().info('Running QAOA optimization for warehouse route...')

        try:
            # Get optimized order
            optimal_order = self.optimize_with_qaoa()

            # Map order back to station names
            station_names = {
                0: 'Charging Dock',
                1: 'Loading Zone',
                2: 'Inventory Scan',
                3: 'Dispatch Zone'
            }

            # Create waypoints message
            waypoints = []
            for idx in optimal_order:
                waypoints.append({
                    'name': station_names[idx],
                    'station_id': idx
                })

            # Publish as JSON string
            msg = String()
            msg.data = json.dumps(waypoints)
            self.waypoint_pub.publish(msg)

            self.get_logger().info(f'Published optimized waypoints: {waypoints}')

        except Exception as e:
            self.get_logger().error(f'QAOA optimization failed: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = QAOAPlanner()

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
