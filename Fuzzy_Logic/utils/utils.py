# utils.py
import math
from controllers.goal_controller import GoalController
from controllers.obstacle_controller import ObstacleController

def calculate_angle_to_target(robot_pos, target_pos):
    dx = target_pos[0] - robot_pos[0]
    dy = target_pos[1] - robot_pos[1]
    angle_rad = math.atan2(dy, dx)
    angle_deg = math.degrees(angle_rad)

    # Normalize to [-90, 90]
    if angle_deg > 90:
        angle_deg = angle_deg - 180
    elif angle_deg < -90:
        angle_deg = angle_deg + 180

    return angle_deg


def get_obstacle_info(robot_pos, obstacles):
    min_distance = float('inf')
    min_angle = 0

    for obstacle in obstacles:
        dx = obstacle[0] - robot_pos[0]
        dy = obstacle[1] - robot_pos[1]
        distance = math.sqrt(dx * dx + dy * dy)

        if distance < min_distance:
            min_distance = distance
            min_angle = math.degrees(math.atan2(dy, dx))
            if min_angle > 90:
                min_angle = min_angle - 180
            elif min_angle < -90:
                min_angle = min_angle + 180

    return min_angle, min_distance


def compute_fuzzy_control(robot_pos, target_pos, obstacles, goal_controller, obstacle_controller):
    angle_to_goal = calculate_angle_to_target(robot_pos, target_pos)

    if obstacles:
        obstacle_angle, obstacle_distance = get_obstacle_info(robot_pos, obstacles)
        if obstacle_distance <= 0.4:  # 40 cm = 0.4 units
            return obstacle_controller.compute(obstacle_angle, obstacle_distance)

    return goal_controller.compute(angle_to_goal)
