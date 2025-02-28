import numpy as np
from typing import List, Tuple, Dict
from enum import Enum
from dataclasses import dataclass
from scipy.optimize import linear_sum_assignment
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32, String
from nav_msgs.msg import Odometry
import math
from math import atan2, sqrt
from .heat_maps import HeatMapClusterer, HeatMapGenerator, RoboCupState, HeatMapVisualizer
import cv2
from .robot_sim import BOT_SIZE
import time

PASS_ALIGNMENT_THRESHOLD = 0.1   #degreee of alignment in radians

# --- Enums and Data Classes ---
class GameState(Enum):
    WE_HAVE_BALL = "we_have_ball"
    OPPONENT_HAS_BALL = "opponent_has_ball"
    LOOSE_BALL = "loose_ball"
    
class RobotRole(Enum):
    BALL_HANDLER = "ball_handler"
    ATTACKER = "attacker"
    MIDFIELDER = "midfielder"
    DEFENDER = "defender"
    GOALKEEPER = "goalkeeper"

@dataclass
class Position:
    x: float
    y: float
    z: float = 0.0

@dataclass
class Velocity:
    vx: float
    vy: float
    vz: float = 0.0

# --- Robot, Ball and Field Classes ---
class Robot:
    def __init__(self, id: int, position: Position, velocity: Velocity, role: RobotRole):
        self.id = id
        self.position = position
        self.velocity = velocity
        self.role = role
        self.target_position = Position(0.0, 0.0, 0.0)
        self.has_ball = False
        self.current_theta = 0.0
        self.target_theta = 0.0
        # Record the position where the robot first took possession of the ball.
        self.possession_position = None

class Ball:
    def __init__(self):
        self.position = Position(0.0, 0.0, 0.0)
        self.velocity = Velocity(0.0, 0.0, 0.0)
        self.possession = None  # None, 'our', or 'opponent'

class Field:
    def __init__(self):
        self.length = 22.0
        self.width = 14.0
        self.our_goal = Position(-self.length/2, 0.0)
        self.opponent_goal = Position(self.length/2, 0.0)

# --- Main Game Manager Node ---
class GameManagerROS2(Node):
    def __init__(self, state: RoboCupState, heat_generator: HeatMapGenerator, 
                 clusterer: HeatMapClusterer, visualizer: HeatMapVisualizer):
        super().__init__('decicion')
        self.field = Field()
        self.our_robots: List[Robot] = []
        self.opponent_robots: List[Robot] = []
        self.ball = Ball()
        self.game_state = GameState.LOOSE_BALL
        self.state = state
        self.heat_generator = heat_generator
        self.clusterer = clusterer
        self.visualizer = visualizer
        
        # --- Global pass-related state variables ---
        self.pass_in_progress = False      # True when a pass is underway.
        self.pass_start_time = 0           # Time at which the pass command was issued.
        self.pass_timeout = 0              # Allowed time for the pass transit.
        self.pass_receiver_id = None       # Intended receiver's robot ID.
        
        # --- New Publishers for command and pass status ---
        self.command_pub = self.create_publisher(String, 'simulation/command', 10)
        self.pass_status_pub = self.create_publisher(String, 'simulation/pass_status', 10)
        
        # Initialize robots with default values.
        self._initialize_robots()
        
        # Ball subscription.
        self.ball_sub = self.create_subscription(
            Float32MultiArray,
            'ball_data',
            self.ball_callback,
            10
        )
        
        # Robot position subscriptions.
        self.our_robot_subs = []
        self.opp_robot_subs = []
        for i in range(5):
            # Our robots.
            self.our_robot_subs.append(
                self.create_subscription(
                    Float32MultiArray,
                    f'o{i+1}_data',
                    lambda msg, idx=i: self.our_robot_callback(msg, idx),
                    10
                )
            )
            # Opponent robots.
            self.opp_robot_subs.append(
                self.create_subscription(
                    Float32MultiArray,
                    f'b{i+1}_data',
                    lambda msg, idx=i: self.opp_robot_callback(msg, idx),
                    10
                )
            )
        
        # Robot velocity subscriptions (odometry).
        self.our_robot_vel_subs = []
        self.opp_robot_vel_subs = []
        for i in range(5):
            # Our robots velocity.
            self.our_robot_vel_subs.append(
                self.create_subscription(
                    Odometry,
                    f'o{i+1}_odom',
                    lambda msg, idx=i: self.our_robot_vel_callback(msg, idx),
                    10
                )
            )
            # Opponent robots velocity.
            self.opp_robot_vel_subs.append(
                self.create_subscription(
                    Odometry,
                    f'b{i+1}_odom',
                    lambda msg, idx=i: self.opp_robot_vel_callback(msg, idx),
                    10
                )
            )
        
        # Publishers for target positions.
        self.target_pubs = []
        for i in range(5):
            self.target_pubs.append(
                self.create_publisher(
                    Float32MultiArray,
                    f'/o{i+1}/decision_target_data',
                    10
                )
            )
        
        # Timer for decision-making loop.
        self.timer = self.create_timer(0.1, self.decision_making_callback)
        self.get_logger().info('Game Manager Node initialized')

    # --- Callbacks for updating state ---
    def ball_callback(self, msg):
        """Handle ball data updates."""
        self.ball.position = Position(
            msg.data[0], msg.data[1], msg.data[2]
        )
        self.ball.velocity = Velocity(
            msg.data[3], msg.data[4], msg.data[5]
        )
        # state.ball_holder is assumed to be an integer (-1 if no one has possession).
        self.state.ball_holder = int(msg.data[6])
        self.state.ball_position = np.array([self.ball.position.x, self.ball.position.y])

    def our_robot_callback(self, msg, robot_idx):
        """Handle our robot position updates."""
        self.our_robots[robot_idx].position = Position(
            msg.data[0], msg.data[1], 0
        )
        self.our_robots[robot_idx].current_theta = msg.data[2]
        self.state.our_positions[robot_idx] = [msg.data[0], msg.data[1]]

    def opp_robot_callback(self, msg, robot_idx):
        """Handle opponent robot position updates."""
        self.opponent_robots[robot_idx].position = Position(
            msg.data[0], msg.data[1], msg.data[2]
        )
        self.state.opp_positions[robot_idx] = [msg.data[0], msg.data[1]]

    def our_robot_vel_callback(self, msg, robot_idx):
        """Handle our robot velocity updates."""
        self.our_robots[robot_idx].velocity = Velocity(
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        )
        self.state.our_velocities[robot_idx] = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y
        ]

    def opp_robot_vel_callback(self, msg, robot_idx):
        """Handle opponent robot velocity updates."""
        self.opponent_robots[robot_idx].velocity = Velocity(
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        )
        self.state.opp_velocities[robot_idx] = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y
        ]

    def publish_target_positions(self):
        """Publish target positions for all robots."""
        for i, robot in enumerate(self.our_robots):
            msg = Float32MultiArray()
            msg.data = [
                robot.target_position.x,
                robot.target_position.y,
                robot.target_theta
            ]
            self.target_pubs[i].publish(msg)

    def decision_making_callback(self):
        """Main decision-making loop triggered by timer."""
        # If a pass is in progress, do not override that with loose-ball logic.
        if not self.pass_in_progress:
            self.update_game_state()
            if self.game_state == GameState.WE_HAVE_BALL:
                self._handle_we_have_ball()
            elif self.game_state == GameState.OPPONENT_HAS_BALL:
                self._handle_opponent_has_ball()
            elif self.game_state == GameState.LOOSE_BALL:
                self._handle_loose_ball()
        else:
            self.get_logger().info("Pass in progress: skipping decision-making for ball control.")
        self.check_pass_status()
        self.publish_target_positions()

    def _initialize_robots(self):
        """Initialize robots with default positions."""
        roles = [RobotRole.GOALKEEPER, RobotRole.DEFENDER, RobotRole.MIDFIELDER, 
                 RobotRole.ATTACKER, RobotRole.ATTACKER]
        for i in range(5):
            pos = self.state.our_positions[i]
            vel = self.state.our_velocities[i]
            self.our_robots.append(
                Robot(i, Position(pos[0], pos[1], 0.0), 
                      Velocity(vel[0], vel[1], 0.0), roles[i])
            )
        for i in range(5):
            pos = self.state.opp_positions[i]
            vel = self.state.opp_velocities[i]
            self.opponent_robots.append(
                Robot(i, Position(pos[0], pos[1], 0.0),
                      Velocity(vel[0], vel[1], 0.0), RobotRole.ATTACKER)
            )

    def determine_ball_possession(self):
        """Determine which team has possession of the ball."""
        POSSESSION_THRESHOLD = 0.5  # meters
        opp_min_dist = float('inf')
        for robot in self.opponent_robots:
            dist = math.sqrt((robot.position.x - self.ball.position.x)**2 +
                             (robot.position.y - self.ball.position.y)**2)
            if dist < opp_min_dist:
                opp_min_dist = dist
        
        if self.state.ball_holder >= 0 and self.state.ball_holder < 5:
            return GameState.WE_HAVE_BALL
        elif opp_min_dist < POSSESSION_THRESHOLD:
            return GameState.OPPONENT_HAS_BALL
        else:
            return GameState.LOOSE_BALL

    def update_game_state(self):
        """Update the game state based on ball possession."""
        self.game_state = self.determine_ball_possession()
        self.get_logger().info(f'Current game state: {self.game_state}')

    def draw_assignment_lines(self, image):
        scale_factor = self.visualizer.scale  # match the scaling used in visualization
        for robot in self.our_robots:
            sy, sx = self.visualizer.get_nearest_index((robot.position.x, robot.position.y))
            ey, ex = self.visualizer.get_nearest_index((robot.target_position.x, robot.target_position.y))
            sx = int(sx * scale_factor)
            sy = int(sy * scale_factor)
            ex = int(ex * scale_factor)
            ey = int(ey * scale_factor)
            self.visualizer.draw_dotted_line(image, (sx, sy), (ex, ey), (255, 255, 255), thickness=3, gap=10)

    # --- Helper functions for goal and pass probabilities ---
    def calculate_goal_probability(self, bot: Robot) -> float:
        """Compute a simple goal probability based on distance to opponent goal."""
        goal = self.field.opponent_goal
        dx = bot.position.x - goal.x
        dy = bot.position.y - goal.y
        distance = math.sqrt(dx*dx + dy*dy)
        max_distance = self.field.length
        prob = max(0.0, 1.0 - distance / max_distance)
        return prob

    def calculate_pass_probability(self, bot: Robot, teammate: Robot) -> float:
        """Compute a simple pass probability based on distance between bot and teammate."""
        dx = bot.position.x - teammate.position.x
        dy = bot.position.y - teammate.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        prob = 1-distance/10
        if bot.position.x - teammate.position.x > 0:
            prob = 0
        return prob

    def calculate_goal_and_pass_probabilities(self) -> Tuple[float, Dict[int, float]]:
        """
        Returns a tuple: (goal_probability, {teammate_id: pass_probability, ...})
        for the current ball handler.
        """
        ball_holder_id = self.state.ball_holder
        if ball_holder_id < 0 or ball_holder_id >= len(self.our_robots):
            return 0.0, {}
        shooter = self.our_robots[ball_holder_id]
        goal_prob = self.calculate_goal_probability(shooter)
        pass_probs = {}
        for teammate in self.our_robots:
            if teammate.id != shooter.id:
                pass_probs[teammate.id] = self.calculate_pass_probability(shooter, teammate)
        return goal_prob, pass_probs

    # --- Modified "We Have Ball" logic ---
    def _handle_we_have_ball(self):
        """Handle decision making when we have the ball—with shot/pass logic."""
        self.get_logger().info("\nHandling WE_HAVE_BALL state:")
        ball_holder_id = self.state.ball_holder
        if ball_holder_id < 0 or ball_holder_id >= len(self.our_robots):
            self.get_logger().warn("Invalid ball holder ID in WE_HAVE_BALL state")
            return
        ball_handler = self.our_robots[ball_holder_id]
        
        # Record where the ball was taken if not already set.
        if ball_handler.possession_position is None:
            ball_handler.possession_position = Position(ball_handler.position.x,
                                                        ball_handler.position.y,
                                                        ball_handler.position.z)
        
        # Calculate probabilities.
        goal_prob, pass_probs = self.calculate_goal_and_pass_probabilities()
        best_pass_id = None
        best_pass_prob = 0.0
        for teammate_id, prob in pass_probs.items():
            if prob > best_pass_prob:
                best_pass_prob = prob
                best_pass_id = teammate_id
        
        # Always plan to advance toward the opponent goal.
        goal_pos = self.field.opponent_goal
        dx = goal_pos.x - ball_handler.possession_position.x
        dy = goal_pos.y - ball_handler.possession_position.y
        distance_to_goal = math.sqrt(dx*dx + dy*dy)
        max_move = 3.0  # rule: bot cannot move more than 3 meters away from possession point.
        scale = max_move / distance_to_goal if distance_to_goal > max_move else 1.0
        target_x = ball_handler.possession_position.x + dx * scale
        target_y = ball_handler.possession_position.y + dy * scale
        
        ball_handler.target_position = Position(target_x, target_y, ball_handler.position.z)
        ball_handler.target_theta = math.atan2(goal_pos.y - ball_handler.position.y,
                                               goal_pos.x - ball_handler.position.x)
        self.get_logger().info(f"Ball handler {ball_handler.id} advancing toward goal at ({target_x:.2f}, {target_y:.2f}) (goal_prob: {goal_prob:.2f})")
        
        # Decision branches:
        GOAL_PROB_THRESHOLD = 0.95
        PASS_PROB_THRESHOLD = 0.7
        
        if goal_prob > GOAL_PROB_THRESHOLD:
            # If near the target, attempt a shot.
            pos_error = math.sqrt((ball_handler.position.x - target_x)**2 + (ball_handler.position.y - target_y)**2)
            if pos_error < 0.5:
                command_msg = String()
                kick_speed = 700 # For a shot, use a relatively high speed.
                command_msg.data = f"KICK {kick_speed} 45"
                self.command_pub.publish(command_msg)
                self.get_logger().info(f"Ball handler {ball_handler.id} shooting with command: {command_msg.data}")
                
        elif best_pass_id is not None and best_pass_prob > PASS_PROB_THRESHOLD:
            # Attempt a pass if one is not already in progress.
            if not self.pass_in_progress:
                receiver = self.our_robots[best_pass_id]
                # Stop both bots by setting their target positions to their current positions.
                ball_handler.target_position = Position(ball_handler.position.x,
                                                        ball_handler.position.y,
                                                        ball_handler.position.z)
                receiver.target_position = Position(receiver.position.x,
                                                      receiver.position.y,
                                                      receiver.position.z)
                # Orient the two bots toward each other.
                angle_handler_to_receiver = math.atan2(receiver.position.y - ball_handler.position.y,
                                                       receiver.position.x - ball_handler.position.x)
                angle_receiver_to_handler = math.atan2(ball_handler.position.y - receiver.position.y,
                                                       ball_handler.position.x - receiver.position.x)
                ball_handler.target_theta = angle_handler_to_receiver
                receiver.target_theta = angle_receiver_to_handler
                self.get_logger().info(f"Initiating pass from {ball_handler.id} to {receiver.id} (pass_prob: {best_pass_prob:.2f})")
                
                status_msg = String()
                status_msg.data = f"PASS_INITIATED from {ball_handler.id} to {receiver.id}"
                self.pass_status_pub.publish(status_msg)
                # Compute the distance between ball handler and receiver.
                d = math.sqrt((ball_handler.position.x - receiver.position.x)**2 +
                              (ball_handler.position.y - receiver.position.y)**2)
                # For the pass, choose a speed proportional to the distance.
                base_speed = 400
                factor = 50
                pass_speed = base_speed + factor * d
                # Compute the expected transit time.
                self.pass_timeout = d / pass_speed if pass_speed > 0 else 2.0

                if (abs(ball_handler.current_theta - ball_handler.target_theta) < PASS_ALIGNMENT_THRESHOLD) and (abs(receiver.current_theta - receiver.target_theta) < PASS_ALIGNMENT_THRESHOLD):
                # Now it is safe to execute the pass.
                    command_msg = String()
                    command_msg.data = f"KICK {pass_speed} 0"
                    self.command_pub.publish(command_msg)
                    self.get_logger().info(f"Ball handler {ball_handler.id} passing with command: {command_msg.data}")
                    status_msg.data = f"PASS_EXECUTED from {ball_handler.id} to {receiver.id}"
                    self.pass_status_pub.publish(status_msg)
                    # Mark that a pass is now underway. (No robot has possession.)
                    self.pass_in_progress = True
                    self.pass_receiver_id = receiver.id
                    self.pass_start_time = time.time()
                else:
                    # Optionally log or wait until alignment is achieved.
                    self.get_logger().info("Waiting for proper alignment before passing...")

            else:
                self.get_logger().info("Pass already in progress, waiting for receiver to gain possession.")
        else:
            # Neither opportunity is strong—still advance toward the goal.
            self.get_logger().info(f"Ball handler {ball_handler.id} advancing as default action.")
        
        # Assign strategic positions for supporting robots.
        # Exclude the ball handler and, if a pass is underway, also exclude the intended receiver.
        excluded_ids = {ball_handler.id}
        if self.pass_in_progress:
            excluded_ids.add(self.pass_receiver_id)
        available_robots = [robot for robot in self.our_robots if robot.id not in excluded_ids]
        strategic_positions = self.generate_strategic_positions()[:len(available_robots)]
        assignments = self.assign_positions(strategic_positions, available_robots)
        for robot_id, target_pos in assignments.items():
            self.our_robots[robot_id].target_position = target_pos
            self.get_logger().info(f"Robot {robot_id} moving to support at position ({target_pos.x:.2f}, {target_pos.y:.2f})")

    def _handle_loose_ball(self):
        """Handle decision making when the ball is loose (only invoked when no pass is underway)."""
        if self.pass_in_progress:
            self.get_logger().info("Pass in progress: loose ball routine skipped.")
            return

        self.get_logger().info("\nHandling LOOSE_BALL state:")
        # Find the closest robot to the ball.
        closest_robot = min(self.our_robots, 
                            key=lambda r: np.sqrt((r.position.x - self.ball.position.x)**2 + 
                                                  (r.position.y - self.ball.position.y)**2))
        closest_robot.target_position = self.ball.position

        dx = self.ball.position.x - closest_robot.position.x
        dy = self.ball.position.y - closest_robot.position.y
        distance = sqrt(dx**2 + dy**2)
        if distance < 1e-6 or distance < BOT_SIZE / 2:
            target_x, target_y = self.ball.position.x, self.ball.position.y
        else:
            target_x = self.ball.position.x - (BOT_SIZE / 2) * (dx / distance)
            target_y = self.ball.position.y - (BOT_SIZE / 2) * (dy / distance)
        target_theta = atan2(target_y - closest_robot.position.y, target_x - closest_robot.position.x)
        closest_robot.target_theta = target_theta

        self.get_logger().info(f"Robot {closest_robot.id} is closest to ball - moving to get possession")
        self.state.ball_holder = closest_robot.id
        closest_robot.possession_position = Position(closest_robot.position.x,
                                                      closest_robot.position.y,
                                                      closest_robot.position.z)
        # Generate positions for other robots.
        available_robots = [robot for robot in self.our_robots if robot.id != closest_robot.id]
        strategic_positions = self.generate_strategic_positions()
        assignments = self.assign_positions(strategic_positions, available_robots)
        for robot_id, target_pos in assignments.items():
            self.our_robots[robot_id].target_position = target_pos
            self.get_logger().info(f"Robot {robot_id} moving to support at position ({target_pos.x:.2f}, {target_pos.y:.2f})")

    def _handle_opponent_has_ball(self):
        """Basic handling when an opponent has the ball."""
        self.get_logger().info("\nHandling OPPONENT_HAS_BALL state:")
        closest_opp = min(self.opponent_robots, 
                          key=lambda r: np.sqrt((r.position.x - self.ball.position.x)**2 + 
                                                (r.position.y - self.ball.position.y)**2))
        closest_our = min(self.our_robots, 
                          key=lambda r: np.sqrt((r.position.x - self.ball.position.x)**2 + 
                                                (r.position.y - self.ball.position.y)**2))
        blocking_pos = Position(
            closest_opp.position.x - 1.5,
            closest_opp.position.y,
            0.0
        )
        closest_our.target_position = blocking_pos
        closest_our.target_theta = math.atan2(
            self.ball.position.y - blocking_pos.y,
            self.ball.position.x - blocking_pos.x
        )
        available_robots = [robot for robot in self.our_robots 
                            if robot.id != closest_our.id and robot.role != RobotRole.GOALKEEPER]
        defensive_positions = []
        base_x = (self.field.our_goal.x + self.ball.position.x) / 2
        spread = 2.0
        for i in range(len(available_robots)):
            y_pos = self.ball.position.y + spread * (i - len(available_robots)/2)
            defensive_positions.append(Position(base_x, y_pos, 0.0))
        assignments = self.assign_positions(defensive_positions, available_robots)
        for robot_id, target_pos in assignments.items():
            self.our_robots[robot_id].target_position = target_pos
            self.our_robots[robot_id].target_theta = math.atan2(
                self.ball.position.y - target_pos.y,
                self.ball.position.x - target_pos.x
            )
            self.get_logger().info(
                f"Robot {robot_id} defending at position ({target_pos.x:.2f}, {target_pos.y:.2f})"
            )

    def generate_strategic_positions(self) -> List[Position]:
        """Generate strategic positions using heatmaps based on game state."""
        maps = []
        weights = []
        maps.extend([
            self.heat_generator.robots_repulsion_map(),
            self.heat_generator.vertical_center_attraction_map(),
            self.heat_generator.horizontal_right_attraction_map()
        ])
        weights.extend([0.6, 0.15, 0.15])
        if self.game_state == GameState.WE_HAVE_BALL:
            maps.extend([
                self.heat_generator.ideal_pass_distance_map(),
                self.heat_generator.goal_direction_map()
            ])
            weights.extend([0.2, 0.15])
        elif self.game_state == GameState.LOOSE_BALL:
            maps.extend([
                self.heat_generator.ideal_pass_distance_map(),
                self.heat_generator.goal_direction_map()
            ])
            weights.extend([0.2, 0.15])
        combined_map = self.heat_generator.combine_heat_maps(maps, weights)
        positions = self.clusterer.get_strategic_positions(combined_map)
        image = self.visualizer.get_opencv_visualization_aligned(combined_map, positions)
        self.draw_assignment_lines(image)
        image = cv2.flip(image, 0)
        cv2.imshow("heatmap", image)
        cv2.waitKey(1)
        return [Position(pos[0], pos[1]) for pos in positions]

    def assign_positions(self, strategic_positions: List[Position], robots: List[Robot]) -> Dict[int, Position]:
        """Assign robots to positions using the Hungarian algorithm while preserving robot IDs."""
        cost_matrix = np.zeros((len(robots), len(strategic_positions)))
        for i, robot in enumerate(robots):
            for j, pos in enumerate(strategic_positions):
                cost_matrix[i, j] = np.sqrt((robot.position.x - pos.x)**2 +
                                            (robot.position.y - pos.y)**2)
        row_indices, col_indices = linear_sum_assignment(cost_matrix)
        assignments = {}
        for i, j in zip(row_indices, col_indices):
            real_robot_id = robots[i].id
            assignments[real_robot_id] = strategic_positions[j]
        return assignments

    def check_pass_status(self):
        """Check whether the intended receiver has gained possession within the allowed time."""
        if self.pass_in_progress:
            # Instead of checking ball-to-robot distance, we use state.ball_holder.
            if self.state.ball_holder == self.pass_receiver_id:
                self.get_logger().info(f"Pass successful to robot {self.pass_receiver_id}")
                status_msg = String()
                status_msg.data = f"PASS_SUCCESS from pass to {self.pass_receiver_id}"
                self.pass_status_pub.publish(status_msg)
                self.pass_in_progress = False
            elif time.time() - self.pass_start_time > self.pass_timeout:
                self.get_logger().warn(f"Pass from robot {self.state.ball_holder} to robot {self.pass_receiver_id} failed")
                status_msg = String()
                status_msg.data = f"PASS_FAILED from {self.state.ball_holder} to {self.pass_receiver_id}"
                self.pass_status_pub.publish(status_msg)
                self.pass_in_progress = False

    def execute_decision_making(self):
        """Main decision-making loop (logic now invoked in decision_making_callback)."""
        pass  # Not used because decision_making_callback already wraps the logic.

def main(args=None):
    rclpy.init(args=args)
    state = RoboCupState()
    generator = HeatMapGenerator(state)
    clusterer = HeatMapClusterer(state)
    visualizer = HeatMapVisualizer(state)
    game_manager = GameManagerROS2(state, generator, clusterer, visualizer)
    try:
        rclpy.spin(game_manager)
    except KeyboardInterrupt:
        pass
    finally:
        game_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
