# simulator.py
import pygame
import random
import numpy as np
import math
from controllers.goal_controller import GoalController
from controllers.obstacle_controller import ObstacleController
from utils.utils import compute_fuzzy_control


class RobotSimulator:
    def __init__(self):
        pygame.init()
        self.width = 800  # Window size (pixels)
        self.height = 800
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Robot Navigation Simulator")

        # Scale factors (convert from meters to pixels)
        self.scale = 400  # 2 meters = 800 pixels

        # Initialize controllers
        self.goal_controller = GoalController()
        self.obstacle_controller = ObstacleController()

        # Initialize positions
        self.reset_simulation()

    def reset_simulation(self):
        """Reset the simulation with new random positions"""
        self.robot_pos = [1.0, 0.05]  # Center x, bottom y + 0.05
        self.target_pos = [random.uniform(0.5, 1.5), random.uniform(1.7, 1.9)]

        # Initialize obstacles
        self.obstacles = self._generate_obstacles()
        self.obstacle_speeds = [random.uniform(0.0002, 0.0005) for _ in range(4)]
        self.obstacle_directions = [1 if random.random() > 0.5 else -1 for _ in range(4)]

    def _generate_obstacles(self):
        obstacles = []
        y_positions = np.linspace(0.7, 1.4, 4)

        for y in y_positions:
            width = random.uniform(0.3, 0.5)
            x = random.uniform(0.2, 1.8 - width)
            obstacles.append([x, y, width])

        return obstacles

    def _update_obstacles(self):
        for i in range(len(self.obstacles)):
            # Update x position
            self.obstacles[i][0] += self.obstacle_speeds[i] * self.obstacle_directions[i]

            # Reverse direction if hitting walls
            if self.obstacles[i][0] <= 0 or self.obstacles[i][0] + self.obstacles[i][2] >= 2:
                self.obstacle_directions[i] *= -1

    def check_circle_rect_collision(self, circle_center, circle_radius, rect):
        """Check if a circle and a rectangle collide."""
        # Closest point on the rectangle to the circle's center
        closest_x = max(rect.left, min(circle_center[0], rect.right))
        closest_y = max(rect.top, min(circle_center[1], rect.bottom))

        # Calculate the distance from the circle's center to this closest point
        distance_x = circle_center[0] - closest_x
        distance_y = circle_center[1] - closest_y

        # Use the Pythagorean theorem to check if the distance is less than the circle's radius
        distance_squared = (distance_x ** 2) + (distance_y ** 2)
        return distance_squared < (circle_radius ** 2)


    def _check_collision(self):
        """Check if robot has collided with any obstacle"""
        robot_radius = 0.25 * self.scale  # Adjust robot's radius with scale
        robot_center = (self.robot_pos[0] * self.scale, (2 - self.robot_pos[1]) * self.scale)

        for obs in self.obstacles:
            obs_x, obs_y, obs_width = obs

            # Create a rectangle for the obstacle with the scaled coordinates
            obs_rect = pygame.Rect(
                obs_x * self.scale,
                (2 - obs_y) * self.scale - (0.05 * self.scale),
                obs_width * self.scale,
                0.1 * self.scale
            )

            # Check for collision between the circle (robot) and the rectangle (obstacle)
            if self.check_circle_rect_collision(robot_center, robot_radius, obs_rect):
                return True
        return False


    def _check_goal_reached(self):
        """Check if robot has reached the target"""
        dx = self.robot_pos[0] - self.target_pos[0]
        dy = self.robot_pos[1] - self.target_pos[1]
        distance = math.sqrt(dx * dx + dy * dy)
        return distance < 0.15  # Goal reached if within 15cm

    def _draw(self):
        # Fill background
        self.screen.fill((255, 255, 255))

        # Draw border
        pygame.draw.rect(self.screen, (0, 0, 0), (0, 0, self.width, self.height), 2)

        # Draw robot
        robot_x = int(self.robot_pos[0] * self.scale)
        robot_y = int((2 - self.robot_pos[1]) * self.scale)  # Flip y-coordinate
        pygame.draw.circle(self.screen, (0, 255, 0), (robot_x, robot_y), int(0.25 * self.scale))

        # Draw target
        target_x = int(self.target_pos[0] * self.scale)
        target_y = int((2 - self.target_pos[1]) * self.scale)
        pygame.draw.circle(self.screen, (255, 0, 0), (target_x, target_y), int(0.05 * self.scale))

        # Draw obstacles
        for obs in self.obstacles:
            obs_x = int(obs[0] * self.scale)
            obs_y = int((2 - obs[1]) * self.scale)
            obs_width = int(obs[2] * self.scale)
            obs_height = int(0.1 * self.scale)
            pygame.draw.rect(self.screen, (0, 0, 255),
                             (obs_x, obs_y - obs_height // 2, obs_width, obs_height))

        pygame.display.flip()

    def game_over(self):
        """Display game over message and wait for the user to press 'R' to restart."""
        game_over = True
        while game_over:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    exit()
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_r:  # Нажмите 'R' для перезапуска
                        game_over = False  # Выход из состояния game over

            # Рисуем сообщение "Game Over"
            self.screen.fill((255, 255, 255))
            font = pygame.font.Font(None, 36)
            text = font.render("CHARGE, ПОКИНУТЬ ТЕЛО:)", True, (255, 0, 0))
            self.screen.blit(text, (self.width // 2 - text.get_width() // 2, self.height // 2 - text.get_height() // 2))
            pygame.display.flip()

    def run(self):
        running = True
        clock = pygame.time.Clock()

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_r:  # Reset simulation with 'R' key
                        self.reset_simulation()

            # Update obstacle positions
            self._update_obstacles()

            # Get control commands
            speed, direction = compute_fuzzy_control(
                self.robot_pos,
                self.target_pos,
                [(obs[0], obs[1]) for obs in self.obstacles],
                self.goal_controller,
                self.obstacle_controller
            )

            # preUpdate robot position based on control commands
            angle_rad = math.radians(direction)
            if (angle_rad > 0):
                self.robot_pos[0] += speed * math.cos(angle_rad) * 0.005
            else:
                self.robot_pos[0] += speed * math.cos(angle_rad) * -0.005
            self.robot_pos[1] += speed * abs(math.sin(angle_rad) * 0.005)
            print(self.robot_pos[1])

            if self._check_collision():
                print("Collision detected! Press 'R' to reset")
                self.game_over()
                self.reset_simulation()

            # Keep robot within bounds
            self.robot_pos[0] = max(0.25, min(1.75, self.robot_pos[0]))
            self.robot_pos[1] = max(0.25, min(1.75, self.robot_pos[1]))

            if self._check_goal_reached():
                print("Goal reached! Press 'R' to reset")
                self.reset_simulation()

            self._draw()
            clock.tick(80)

        pygame.quit()

def main():
    simulator = RobotSimulator()
    simulator.run()

if __name__ == "__main__":
    main()
