import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans
import cv2
import time 
import os

class RoboCupState:
    def __init__(self):
        # Field dimensions (in meters)
        self.field_length = 22.0  # Adjust based on your field size
        self.field_width = 14.0    # Adjust based on your field size
        
        # Initialize with example positions (you'll update these with real positions)
        self.our_positions = np.array([
            [-2.0, 1.0],   # Player 1 (ball holder)
            [-6.0, -1.5],  # Player 2
            [4.0, 0.0],    # Player 3
            [1.0, 5.0],    # Player 4
            [2.0, -2.0]    # Player 5
        ])
        
        self.opp_positions = np.array([
            [5.0, 5.0],    # Opponent 1
            [1.0, -1.0],   # Opponent 2
            [0.0, 1.0],    # Opponent 3
            [-1.0, -2.0],  # Opponent 4
            [-2.0, 2.0]    # Opponent 5
        ])
        
        self.our_velocities = np.array([
            [0.5, 0.0],    # Player 1 velocity
            [0.0, 0.5],    # Player 2 velocity
            [-0.5, 0.0],   # Player 3 velocity
            [0.0, -0.5],   # Player 4 velocity
            [0.3, 0.3]     # Player 5 velocity
        ])
        
        self.opp_velocities = np.array([
            [-0.3, 0.0],   # Opponent 1 velocity
            [0.0, -0.3],   # Opponent 2 velocity
            [0.3, 0.0],    # Opponent 3 velocity
            [0.0, 0.3],    # Opponent 4 velocity
            [-0.2, -0.2]   # Opponent 5 velocity
        ])
        
        self.ball_holder = 0  # Index of our robot holding the ball
        
        # Resolution for heat maps (pixels per meter)
        self.resolution = 10
        
        # Initialize grid
        self.x = np.linspace(-self.field_length/2, self.field_length/2, 
                           int(self.field_length * self.resolution))
        self.y = np.linspace(-self.field_width/2, self.field_width/2, 
                           int(self.field_width * self.resolution))
        self.X, self.Y = np.meshgrid(self.x, self.y)

class HeatMapGenerator:
    def __init__(self, state: RoboCupState):
        self.state = state
        
    def get_distance_from_point(self, point):
        """Calculate distance from each grid point to a specific point"""
        return np.sqrt((self.state.X - point[0])**2 + (self.state.Y - point[1])**2)
    
    def robots_repulsion_map(self, sigma=1.0):
        """Generate heat map where values increase away from all robots"""
        heat_map = np.zeros_like(self.state.X)
        
        # Add repulsion from our robots
        for pos in self.state.our_positions:
            distance = self.get_distance_from_point(pos)
            heat_map += 1 - np.exp(-distance**2 / (2*sigma**2))
            
        # Add repulsion from opponent robots
        for pos in self.state.opp_positions:
            distance = self.get_distance_from_point(pos)
            heat_map += 1 - np.exp(-distance**2 / (2*sigma**2))
            
        return heat_map / heat_map.max()  # Normalize
    
    def vertical_center_attraction_map(self):
        """Generate heat map with higher values near vertical center"""
        return 1 - np.abs(self.state.Y) / (self.state.field_width/2)
    
    def horizontal_right_attraction_map(self):
        # Create gradient using X coordinates
        # Normalize X coordinates to [0,1] range where:
        # leftmost = 0.0, rightmost = 1.0
        x_normalized = (self.state.X - self.state.X.min()) / (self.state.X.max() - self.state.X.min())
        return x_normalized
    
    def ball_holder_circle_map(self, radius=1.5):
        """Generate circular region around ball holder"""
        heat_map = np.zeros_like(self.state.X)
        
        # Create circles around all our robots with the ball holder having higher value
        for i, pos in enumerate(self.state.our_positions):
            distance = self.get_distance_from_point(pos)
            if i == self.state.ball_holder:
                heat_map += (distance <= radius).astype(float) * 1.0  # Full intensity for ball holder
                 
        return heat_map / heat_map.max()
    
    def ideal_pass_distance_map(self, A=1.0, r0=3.0, sigma=1.0):
        """Generate heat map based on ideal pass distance equation"""
        heat_map = np.zeros_like(self.state.X)
        
        # Calculate pass distance map from ball holder
        holder_pos = self.state.our_positions[self.state.ball_holder]
        r = self.get_distance_from_point(holder_pos)
        heat_map = A * np.exp(-(r - r0)**2 / (2*sigma**2))
            
        return heat_map / heat_map.max()
    
    def goal_direction_map(self, goal_pos=(10.2, 0.0), IGD=6.0, sigma=1.0, p=1.0):
        """
        Generate heat map based on goal probability equation:
        GoalProb = cos(α) * (p/(d*sqrt(2π))) * exp(-(dist_KG - IGD)^2 / (2σ^2))
        
        Parameters:
        - goal_pos: Position of the goal center (x, y)
        - IGD: Ideal Goal Distance
        - sigma: Standard deviation for the Gaussian distribution
        - p: Scaling parameter
        """
        holder_pos = self.state.our_positions[self.state.ball_holder]
        
        # Calculate angle component (cos(α))
        dx = self.state.X - holder_pos[0]
        dy = self.state.Y - holder_pos[1]
        
        # Calculate angles relative to ball holder
        angles = np.arctan2(dy, dx)
        goal_angle = np.arctan2(goal_pos[1] - holder_pos[1], 
                            goal_pos[0] - holder_pos[0])
        
        # Calculate cos(α) - angle difference from goal direction
        cos_alpha = np.cos(angles - goal_angle)
        
        # Calculate distance from each point to goal (dist_KG)
        dist_to_goal = np.sqrt((self.state.X - goal_pos[0])**2 + 
                            (self.state.Y - goal_pos[1])**2)
        
        # Calculate Gaussian component
        gaussian = np.exp(-(dist_to_goal - IGD)**2 / (2 * sigma**2))
        
        # Calculate normalization factor
        d = np.sqrt(dx**2 + dy**2)  # distance from ball holder to each point
        norm_factor = p / (d * np.sqrt(2 * np.pi))
        
        # Combine all components
        heat_map = cos_alpha * norm_factor * gaussian
        
        # Normalize to [0, 1] range
        heat_map = np.clip(heat_map, 0, None)  # Ensure non-negative values
        if heat_map.max() > 0:
            heat_map = heat_map / heat_map.max()
        
        return heat_map
    
    def combine_heat_maps(self, maps, weights=None):
        """Combine multiple heat maps with optional weights"""
        if weights is None:
            weights = [1.0] * len(maps)
        
        combined = np.zeros_like(self.state.X)
        for map_data, weight in zip(maps, weights):
            combined += weight * map_data
            
        return combined / combined.max()  # Normalize

class HeatMapClusterer:
    def __init__(self, state: RoboCupState):
        self.state = state
        
    def find_optimal_positions(self, heat_map, n_clusters=8):
        """Find optimal positions in the heatmap using clustering"""
        high_value_points = np.argwhere(heat_map > 0.85)
        
        if len(high_value_points) < n_clusters:
            return np.array([[0, 0]] * n_clusters)
            
        weights = np.array([heat_map[x, y] for x, y in high_value_points])
        
        kmeans = KMeans(n_clusters=n_clusters, random_state=42)
        kmeans.fit(high_value_points, sample_weight=weights)
        
        centers = kmeans.cluster_centers_
        x_coords = self.state.x[np.clip(centers[:, 1].astype(int), 0, len(self.state.x)-1)]
        y_coords = self.state.y[np.clip(centers[:, 0].astype(int), 0, len(self.state.y)-1)]
        
        return np.column_stack((x_coords, y_coords))
    
    def get_nearest_index(self, position):
        """Convert field coordinates to heatmap indices"""
        x_idx = np.argmin(np.abs(self.state.x - position[0]))
        y_idx = np.argmin(np.abs(self.state.y - position[1]))
        return y_idx, x_idx
    
    def get_strategic_positions(self, combined_map):
        """Get strategic positions based on the combined heatmap"""
        positions = self.find_optimal_positions(combined_map)
        position_values = [combined_map[self.get_nearest_index(pos)] for pos in positions]
        return positions[np.argsort(position_values)[::-1]]
        
    
    


class HeatMapVisualizer:
    def __init__(self, state: RoboCupState):
        self.state = state
        self.scale = 4

    def show_matplotlib(self, heat_map, title="Heat Map"):
        """Display heat map using matplotlib with blue-red colormap"""
        plt.figure(figsize=(10, 8))
        
        # Create custom colormap from blue to red
        colors = ['darkblue', 'blue', 'royalblue', 'white', 'red', 'darkred']
        n_bins = 100
        cmap = plt.cm.RdBu_r  # Built-in blue-red colormap
        
        # Plot heatmap
        plt.imshow(heat_map, extent=[-self.state.field_length/2, self.state.field_length/2,
                                   -self.state.field_width/2, self.state.field_width/2],
                  origin='lower', cmap=cmap)
        
        # Plot robot positions
        plt.plot(self.state.our_positions[:, 0], self.state.our_positions[:, 1], 
                'go', label='Our Team', markersize=10)
        plt.plot(self.state.opp_positions[:, 0], self.state.opp_positions[:, 1], 
                'yo', label='Opponents', markersize=10)
        
        # Highlight ball holder
        plt.plot(self.state.our_positions[self.state.ball_holder, 0],
                self.state.our_positions[self.state.ball_holder, 1],
                'ro', markersize=15, label='Ball Holder')
        
        plt.colorbar(label='Value')
        plt.title(title)
        plt.xlabel('X (meters)')
        plt.ylabel('Y (meters)')
        plt.legend()
        plt.grid(True)
        plt.show()
    
    def get_nearest_index(self, position):
        """Convert field coordinates to heatmap indices"""
        x_idx = np.argmin(np.abs(self.state.x - position[0]))
        y_idx = np.argmin(np.abs(self.state.y - position[1]))
        return y_idx, x_idx
    
    def visualize_clusters(self, heat_map, positions, title="Clustered Strategic Positions"):
        """Visualize heatmap with clustered positions"""
        plt.figure(figsize=(12, 8))
        
        # Plot heatmap
        plt.imshow(heat_map, extent=[-self.state.field_length/2, self.state.field_length/2,
                                   -self.state.field_width/2, self.state.field_width/2],
                  origin='lower', cmap='RdBu_r')
        
        # Plot existing robots
        plt.plot(self.state.our_positions[:, 0], self.state.our_positions[:, 1], 
                'go', label='Our Team', markersize=10)
        plt.plot(self.state.opp_positions[:, 0], self.state.opp_positions[:, 1], 
                'yo', label='Opponents', markersize=10)
        
        # Plot ball holder
        plt.plot(self.state.our_positions[self.state.ball_holder, 0],
                self.state.our_positions[self.state.ball_holder, 1],
                'ro', markersize=15, label='Ball Holder')
        
        # Plot cluster positions
        for i, pos in enumerate(positions):
            plt.plot(pos[0], pos[1], 'w*', markersize=15, 
                    label=f'Strategic Position {i+1}')
            plt.annotate(f'P{i+1}', (pos[0], pos[1]), 
                        xytext=(10, 10), textcoords='offset points',
                        color='white', fontsize=12)
        
        plt.colorbar(label='Heat Value')
        plt.title(title)
        plt.xlabel('X (meters)')
        plt.ylabel('Y (meters)')
        plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        plt.grid(True)
        plt.tight_layout()
        plt.show()
        
    def get_opencv_visualization(self, heat_map, positions):
        """Get OpenCV image with clusters"""
        heat_map_normalized = (heat_map * 255).astype(np.uint8)
        heat_map_color = cv2.applyColorMap(heat_map_normalized, cv2.COLORMAP_JET)
        
        # Convert heatmap coordinates to image coordinates
        for pos in positions:
            x, y = self.get_nearest_index(pos)
            cv2.drawMarker(heat_map_color, (y, x), (255, 255, 255), 
                          cv2.MARKER_STAR, 20, 2)
            
        return heat_map_color
    
    def draw_dotted_line(self, img, start_pt, end_pt, color, thickness=2, gap=10):
        """
        Draw a dotted line on img from start_pt to end_pt (both are (x, y) in image coords).
        gap sets the length of spacing between dots.
        """
        dist_x = end_pt[0] - start_pt[0]
        dist_y = end_pt[1] - start_pt[1]
        length = int((dist_x**2 + dist_y**2)**0.5)
        
        # Avoid division by zero if start and end are the same
        if length == 0:
            return
        
        step_x = dist_x / length
        step_y = dist_y / length
        
        # Plot small circles along the line
        for i in range(0, length, gap):
            cx = int(start_pt[0] + step_x * i)
            cy = int(start_pt[1] + step_y * i)
            cv2.circle(img, (cx, cy), thickness, color, -1)

    def get_opencv_visualization_aligned(self, heat_map, positions, title="Clustered Positions"):
        """
        Create an OpenCV visualization with a colormap aligned to Matplotlib's RdBu_r.
        """
        # Normalize the heatmap to [0, 1]
        heat_map_normalized = (heat_map - heat_map.min()) / (heat_map.max() - heat_map.min())

        # Apply Matplotlib colormap (RdBu_r)
        cmap = plt.cm.RdBu_r  # Matplotlib's colormap
        mapped_colors = (cmap(heat_map_normalized)[:, :, :3] * 255).astype(np.uint8)  # RGB values

        # Convert RGB to BGR for OpenCV
        heat_map_bgr = cv2.cvtColor(mapped_colors, cv2.COLOR_RGB2BGR)
        # Suppose you want it scaled by a factor of 2
        scale_factor = self.scale
        new_width = int(heat_map_bgr.shape[1] * scale_factor)
        new_height = int(heat_map_bgr.shape[0] * scale_factor)
        heat_map_bgr = cv2.resize(heat_map_bgr, (new_width, new_height), interpolation=cv2.INTER_LINEAR)

        # Example: draw our team with green circles
        for pos in self.state.our_positions:
            x_idx, y_idx = self.get_nearest_index(pos)
            # Scale indexes if you've resized the heat_map_bgr
            x_idx = int(x_idx * scale_factor)
            y_idx = int(y_idx * scale_factor)
            cv2.circle(heat_map_bgr, (y_idx, x_idx), 8, (0, 255, 0), -1)

        # Example: draw opponent team with yellow circles
        for pos in self.state.opp_positions:
            x_idx, y_idx = self.get_nearest_index(pos)
            x_idx = int(x_idx * scale_factor)
            y_idx = int(y_idx * scale_factor)
            cv2.circle(heat_map_bgr, (y_idx, x_idx), 8, (0, 255, 255), -1)
        # Overlay cluster positions

        for pos in positions:
            x_idx, y_idx = self.get_nearest_index(pos)
            cv2.drawMarker(heat_map_bgr, (int(y_idx*scale_factor), int(x_idx*scale_factor)), (255, 255, 255), cv2.MARKER_STAR, 20, 2)

        # # Optionally add title using OpenCV
        # title_pos = (10, 30)  # Top-left corner for text
        # font = cv2.FONT_HERSHEY_SIMPLEX
        # cv2.putText(heat_map_bgr, title, title_pos, font, 1, (255, 255, 255), 2, cv2.LINE_AA)
        
       
        return heat_map_bgr
    
    


# Example usage:
if __name__ == "__main__":
    # Initialize state
    state = RoboCupState()
    
    # Create generators and visualizer
    generator = HeatMapGenerator(state)
    visualizer = HeatMapVisualizer(state)
    clusterer = HeatMapClusterer(state)
    
    # Generate individual heat maps
    i_time = time.time()
    repulsion_map = generator.robots_repulsion_map()
    vertical_map = generator.vertical_center_attraction_map()
    horizontal_map = generator.horizontal_right_attraction_map()  # New map
    # ball_circle_map = generator.ball_holder_circle_map()
    pass_distance_map = generator.ideal_pass_distance_map()
    goal_map = generator.goal_direction_map()
    
    # Combine maps 
    combine_time = time.time()
    combined_map = generator.combine_heat_maps(
        [repulsion_map, vertical_map, horizontal_map, pass_distance_map, goal_map],
        weights=[0.6, 0.15, 0.15, 0.2, 0.15, 0.15]  # Adjusted weights to include horizontal map
    )
    print(f'combining time = {time.time()-combine_time}')
    positions = clusterer.get_strategic_positions(combined_map)
    print(positions)
    
  
    
    # Visualize individual maps
    visualizer.show_matplotlib(repulsion_map, "Robots Repulsion Map")
    visualizer.show_matplotlib(vertical_map, "Vertical Center Attraction Map")
    visualizer.show_matplotlib(horizontal_map, "Horizontal Right Attraction Map")
    # visualizer.show_matplotlib(ball_circle_map, "Ball Holder Circle Map")
    visualizer.show_matplotlib(pass_distance_map, "Ideal Pass Distance Map")
    visualizer.show_matplotlib(goal_map, "Goal Direction Map")
    visualizer.show_matplotlib(combined_map, "Combined Heat Map")

    visualizer.visualize_clusters(combined_map, positions)
    

    # # Example of getting OpenCV image
    # cv_image = visualizer.get_opencv_visualization_aligned(combined_map,positions)
    # # cv_image = visualizer.get_opencv_visualization_aligned(combined_map,positions)
    # # cv2.imshow('Heat Map (OpenCV)', cv_image)
    # cv_image = visualizer.get_opencv_visualization_aligned(combined_map,positions)
    
    
    