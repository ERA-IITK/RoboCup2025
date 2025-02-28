# RoboCup 2025 Submission
## Team: ERA-IITK 
## Introduction

Team ERA-IITK is an autonomous robotics team based at the Indian Institute of Technology Kanpur, India. Founded in 2018, we are dedicated to pioneering autonomous solutions for complex robotics challenges. Representing India at international competitions such as the RoboCup MSL Challenge, we continuously push the boundaries of grassroots engineering through innovative hardware and software solutions. This repository houses the codebase for all the domains described in our Team Description Paper.

## Domains

### Mechanical Design
This module covers the design and simulation of the robot's physical components:
- **Chassis & Structure:** Designed for optimal speed, strength, agility, and stability.
- **Drive Motors & Suspension:** Implementation of a holonomic drive using planetary gear motors, omni-wheels, and optimized suspension.
- **Dribbling & Kicking Mechanisms:** Custom systems for precise ball control and shot execution, including solenoid and angle control modules.
- **Goalkeeper’s Shield:** A mechanical barrier designed to protect the goal.

### Electrical Design
This domain handles the robot’s power and control circuitry:
- **Central Control PCB:** Based on an STM Nucleo-144 microcontroller, integrating sensors and actuator commands.
- **Power Distribution:** Utilizes multiple buck and boost converters powered by dual Li-ion batteries to supply various voltages.
- **Kicking Mechanism Power Circuitry:** High-power circuitry for safe and efficient solenoid operation.
- **Emergency Stop:** A dedicated circuit ensuring immediate shutdown in critical situations.

### Self-Localization
This module enables accurate positioning on the field:
- **Field-Line Extraction:** Processes camera data to highlight white field lines.
- **Pose Estimation:** Uses differential evolution to optimize the alignment of the extracted lines with a reference field map, providing a robust global pose estimate.
- **Fusion with Odometry:** Enhances accuracy by combining visual and inertial data.

### Controls and Motion Planning
Responsible for planning and executing the robot’s trajectory:
- **Path Planning:** Implements RRT* and cubic spline smoothing to generate collision-free, smooth paths.
- **Model Predictive Control (MPC):** Uses the ACADO toolkit for real-time, constraint-aware motion control.
- **Inverse Kinematics:** Converts optimized velocity commands into individual wheel angular velocities.

### Vision System
Integrates hardware and software to perceive the game environment:
- **Hardware:** Employs four wide-angle IMX 179 USB cameras connected to a Jetson Orin nano, operating at 18 fps.
- **Object Detection:** Runs Yolov11 in parallel threads to detect enemy robots, teammate bots, the ball, and goal posts.
- **Coordinate Transformation:** Maps image pixel coordinates to real-world field coordinates using a pre-calibrated transformation.
- **Occlusion Handling:** Uses a temporary storage array to compare positions and differentiate duplicate detections.

### Decision Algorithm
This module formulates game strategies by processing the current field state:
- **Game State Analysis:** Determines ball possession and evaluates the positions of enemy and teammate bots.
- **Heat Map Integration:** Merges various attraction and repulsion maps to compute optimal positions.
- **Decision Tree:** Assigns actions (such as moving towards the ball or shooting) based on the dynamic field state.
- **ROS2 Integration:** Ensures seamless communication between decision-making and other modules.
