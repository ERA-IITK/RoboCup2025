import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_dir = get_package_share_directory('my_package')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'RoboCupMSL.wbt')
    )

    robot_controllers = []
    for i in range(1, 6):  # For robots O1 to O5  and B1 to B5
        robot_description_path = os.path.join(package_dir, 'resource', f'my_robot_O{i}.urdf')
        robot_controller = WebotsController(
            robot_name=f'O{i}',
            parameters=[
                {'robot_description': robot_description_path},
            ]
        )
        robot_controllers.append(robot_controller)
        #upper one is for orange players
        #the code below is for blue players
        robot_description_path = os.path.join(package_dir, 'resource', f'my_robot_B{i}.urdf')
        robot_controller = WebotsController(
            robot_name=f'B{i}',
            parameters=[
                {'robot_description': robot_description_path},
            ]
        )
        robot_controllers.append(robot_controller)

    robot_description_path = os.path.join(package_dir, 'resource', 'ball.urdf')
    robot_controller = WebotsController(
        robot_name='new_ball',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )
    robot_controllers.append(robot_controller)

    

    return LaunchDescription([
        webots,
        *robot_controllers,  # Unpack the list of controllers
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
