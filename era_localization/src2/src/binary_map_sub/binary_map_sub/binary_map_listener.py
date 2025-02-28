import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray
import numpy as np
import time
import cv2
from scipy.ndimage import rotate
from scipy.optimize import differential_evolution
from floor_map import FloorMap

def calculate_overlap(big_matrix, small_matrix, x_offset, y_offset, theta):
    rotated_small = rotate(small_matrix.astype(float), theta, reshape=True, order=0, mode='constant', cval=0)
    rotated_small = rotated_small > 0.5 
    sh, sw = rotated_small.shape
    bh, bw = big_matrix.shape
    x_offset = int(round(x_offset))
    y_offset = int(round(y_offset))
    x_start = max(0, x_offset)
    y_start = max(0, y_offset)
    x_end = min(bh, x_offset + sh)
    y_end = min(bw, y_offset + sw)
    if x_end <= x_start or y_end <= y_start:
        return 0
    sx_start = max(0, -x_offset)
    sy_start = max(0, -y_offset)
    sx_end = sx_start + (x_end - x_start)
    sy_end = sy_start + (y_end - y_start)
    big_region = big_matrix[x_start:x_end, y_start:y_end]
    small_region = rotated_small[sx_start:sx_end, sy_start:sy_end]
    return np.sum(big_region & small_region)

def optimization_wrapper(params, big, small):
    x, y, theta = params
    return -calculate_overlap(big, small, x, y, theta)

def find_optimal_alignment(big_matrix, small_matrix):
    bounds = [
        (0, big_matrix.shape[0]),
        (0, big_matrix.shape[1]),
        (0, 360)
    ]
    result = differential_evolution(
        optimization_wrapper,
        bounds,
        args=(big_matrix, small_matrix),
        strategy='best1bin',
        maxiter=50,
        popsize=15,
        tol=0.1,
        seed=42
    )
    return (
        int(round(result.x[0])),
        int(round(result.x[1])),
        result.x[2] % 360,
        -result.fun
    )

class BinaryMapSubscriber(Node):
    def __init__(self):
        super().__init__('binary_map_subscriber')
        self.subscription = self.create_subscription(
            Int8MultiArray,
            '/binary_map',
            self.binary_map_callback,
            10
        )
        floor_map_obj = FloorMap()
        floor_map_obj.configure(10.0, 1.0)  # pixels per meter and border size
        self.field_map = floor_map_obj.create_floor_map()

    def binary_map_callback(self, msg):
        if len(msg.layout.dim) < 2:
            self.get_logger().warn("Received message without dimensions!")
            return

        rows = msg.layout.dim[0].size
        cols = msg.layout.dim[1].size

        binary_map = np.array(msg.data, dtype=np.uint8).reshape((rows, cols))

        self.get_logger().info(f"Received binary map of shape: {binary_map.shape}")

        start_time = time.time()
        optimal_x, optimal_y, optimal_theta, max_overlap = find_optimal_alignment(self.field_map, binary_map)
        end_time = time.time()

        self.get_logger().info(f"Optimal Position: ({optimal_x}, {optimal_y})")
        self.get_logger().info(f"Optimal Rotation: {optimal_theta:.2f}°")
        self.get_logger().info(f"Maximum Overlap: {max_overlap}")
        self.get_logger().info(f"Alignment Computation Time: {end_time - start_time:.2f} seconds")

def main(args=None):
    rclpy.init(args=args)
    node = BinaryMapSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
