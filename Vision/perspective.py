import numpy as np
import cv2

class CoordinateEstimator:
    def __init__(self, image_width, image_height, fov_horizontal, fov_vertical, camera_height, camera_tilt=0):
        """
        Initialize the coordinate estimator
        
        Args:
            image_width (int): Width of the camera image in pixels
            image_height (int): Height of the camera image in pixels
            fov_horizontal (float): Horizontal field of view in degrees
            fov_vertical (float): Vertical field of view in degrees
            camera_height (float): Height of camera from ground in meters
            camera_tilt (float): Camera tilt angle from vertical in degrees (default 0)
        """
        self.image_width = image_width
        self.image_height = image_height
        self.fov_h = np.radians(fov_horizontal)
        self.fov_v = np.radians(fov_vertical)
        self.camera_height = camera_height
        self.camera_tilt = np.radians(camera_tilt)
        
        # Calculate focal length in pixels
        self.focal_length_x = (image_width / 2) / np.tan(self.fov_h / 2)
        self.focal_length_y = (image_height / 2) / np.tan(self.fov_v / 2)
        
        # Image center
        self.cx = image_width / 2
        self.cy = image_height / 2
        
        # Create camera matrix
        self.camera_matrix = np.array([
            [self.focal_length_x, 0, self.cx],
            [0, self.focal_length_y, self.cy],
            [0, 0, 1]
        ])
        
        # Calculate horizon line
        self.calculate_horizon_line()
        
    def calculate_horizon_line(self):
        """Calculate the horizon line in image coordinates"""
        # The horizon line is where y-component of the rotated ray becomes 0
        # This happens at the angle perpendicular to the camera's tilted normal
        horizon_angle = self.camera_tilt - np.pi/2
        self.horizon_y = self.cy - self.focal_length_y * np.tan(horizon_angle)
        
    def is_above_horizon(self, pixel_y):
        """Check if a pixel is above the horizon line"""
        return pixel_y < self.horizon_y
    
    def pixel_to_ray(self, pixel_x, pixel_y):
        
        # Convert pixel coordinates to normalized device coordinates
        x = (pixel_x - self.cx) / self.focal_length_x
        y = (pixel_y - self.cy) / self.focal_length_y
        
        # Create ray direction vector
        ray = np.array([x, y, 1.0])
        return ray / np.linalg.norm(ray)
    
    def estimate_world_position(self, pixel_x, pixel_y):
        """
        Estimate 3D world position from pixel coordinates
        Assumes the point lies on the ground plane (y = 0 in world space)
        
        Args:
            pixel_x (float): X coordinate in image space
            pixel_y (float): Y coordinate in image space
            
        Returns:
            tuple: (x, z, valid) where:
                  x, z are coordinates in world space (meters)
                  valid is False if point is above horizon or at infinity
        """
        # Check if point is above horizon
        if self.is_above_horizon(pixel_y):
            return (float('inf'), float('inf'), False)
            
        # Get ray direction in camera space
        ray = self.pixel_to_ray(pixel_x, pixel_y)
        
        # Apply camera tilt transformation
        rotation_matrix = np.array([
            [1, 0, 0],
            [0, np.cos(self.camera_tilt), -np.sin(self.camera_tilt)],
            [0, np.sin(self.camera_tilt), np.cos(self.camera_tilt)]
        ])
        ray = rotation_matrix @ ray
        
        # Check if ray is parallel to or pointing above ground plane
        if ray[1] >= 0:  # y-component determines if ray intersects ground
            return (float('inf'), float('inf'), False)
        
        # Ground plane intersection
        t = -self.camera_height / ray[1]  # Scale factor for ray
        
        # Calculate world position
        world_x = ray[0] * t
        world_z = ray[2] * t
        
        return world_x, world_z, True

    def draw_world_grid(self, image, grid_size=1.0, max_distance=10.0):
        
        result = image.copy()
        
        # Draw the horizon line
        cv2.line(result, 
                (0, int(self.horizon_y)), 
                (self.image_width, int(self.horizon_y)), 
                (255, 0, 0), 2)  # Blue horizon line
        
        # Draw longitudinal lines (parallel to camera's forward direction)
        x_range = np.arange(-max_distance, max_distance + grid_size, grid_size)
        z_range = np.arange(0, max_distance + grid_size, grid_size)
        
        for x in x_range:
            points = []
            for z in z_range:
                # Convert world coordinates to camera space
                point_cam = np.array([x, -self.camera_height, z])
                
                # Apply inverse camera tilt
                rotation_matrix = np.array([
                    [1, 0, 0],
                    [0, np.cos(-self.camera_tilt), -np.sin(-self.camera_tilt)],
                    [0, np.sin(-self.camera_tilt), np.cos(-self.camera_tilt)]
                ])
                point_cam = rotation_matrix @ point_cam
                
                # Project to image space if point is in front of camera
                if point_cam[2] > 0:
                    pixel_x = int((point_cam[0] / point_cam[2]) * self.focal_length_x + self.cx)
                    # Flip the y-coordinate for OpenCV's coordinate system
                    pixel_y = int(self.cy - (point_cam[1] / point_cam[2]) * self.focal_length_y)
                    
                    if 0 <= pixel_x < self.image_width and 0 <= pixel_y < self.image_height:
                        points.append((pixel_x, pixel_y))
            
            # Draw the longitudinal line
            if len(points) >= 2:
                for i in range(len(points) - 1):
                    cv2.line(result, points[i], points[i + 1], (0, 255, 0), 2)
        
        # Draw lateral lines (perpendicular to camera's forward direction)
        for z in z_range:
            points = []
            for x in x_range:
                # Convert world coordinates to camera space
                point_cam = np.array([x, -self.camera_height, z])
                
                # Apply inverse camera tilt
                rotation_matrix = np.array([
                    [1, 0, 0],
                    [0, np.cos(-self.camera_tilt), -np.sin(-self.camera_tilt)],
                    [0, np.sin(-self.camera_tilt), np.cos(-self.camera_tilt)]
                ])
                point_cam = rotation_matrix @ point_cam
                
                # Project to image space if point is in front of camera
                if point_cam[2] > 0:
                    pixel_x = int((point_cam[0] / point_cam[2]) * self.focal_length_x + self.cx)
                    # Flip the y-coordinate for OpenCV's coordinate system
                    pixel_y = int(self.cy - (point_cam[1] / point_cam[2]) * self.focal_length_y)
                    
                    if 0 <= pixel_x < self.image_width and 0 <= pixel_y < self.image_height:
                        points.append((pixel_x, pixel_y))
            
            # Draw the lateral line
            if len(points) >= 2:
                for i in range(len(points) - 1):
                    cv2.line(result, points[i], points[i + 1], (0, 255, 0), 2)

            # Add distance labels
            if len(points) > 0:
                mid_point = points[len(points)//2]
                cv2.putText(result, f"{z:.1f}m", 
                        (mid_point[0], mid_point[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        return result
    

estimator = CoordinateEstimator(
    image_width=1600,
    image_height=720,
    fov_horizontal=74.26,
    fov_vertical=37.63,
    camera_height=8,  # meters
    camera_tilt= 30  # degrees down from horizontal
)

# Alternative approach if you just want the coordinates:
coordinates = estimator.estimate_world_position(pixel_x=1600, pixel_y=720)
world_x, world_z = coordinates[:2]  # Take just the first two values

image = cv2.imread(r'images\basketball court.jpeg')

grid_image = estimator.draw_world_grid(image, grid_size=1.0, max_distance=30)
cv2.imshow('Grid', grid_image)
cv2.waitKey(0)