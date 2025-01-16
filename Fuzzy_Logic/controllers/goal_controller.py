# goal_controller.py
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl


class GoalController:
    def __init__(self):
        # Input variables - угол до цели
        self.angle_to_goal = ctrl.Antecedent(np.arange(-95, 96, 1), 'angle_to_goal')

        # Output variables - скорость и направление движения
        self.robot_speed = ctrl.Consequent(np.arange(0, 0.31, 0.01), 'robot_speed')
        self.robot_direction = ctrl.Consequent(np.arange(-95, 96, 1), 'robot_direction')

        self._define_memberships()
        self._define_rules()

        self.controller = ctrl.ControlSystem(self.rules)
        self.simulation = ctrl.ControlSystemSimulation(self.controller)

    def _define_memberships(self):
        # Более детальное разделение углов для лучшего контроля
        self.angle_to_goal['left'] = fuzz.trapmf(self.angle_to_goal.universe, [-95, -95, -40, 0])
        self.angle_to_goal['center'] = fuzz.trimf(self.angle_to_goal.universe, [-40, 0, 40])
        self.angle_to_goal['right'] = fuzz.trapmf(self.angle_to_goal.universe, [0, 40, 95, 95])

        # Более плавное управление скоростью
        self.robot_speed['slow'] = fuzz.trapmf(self.robot_speed.universe, [0, 0, 0.05, 0.12])
        self.robot_speed['fast'] = fuzz.trapmf(self.robot_speed.universe, [0.12, 0.15, 0.2, 0.2])

        # Более точное управление поворотами
        self.robot_direction['left'] = fuzz.trapmf(self.robot_direction.universe, [-95, -95, -40, 0])
        self.robot_direction['center'] = fuzz.trimf(self.robot_direction.universe, [-40, 0, 40])
        self.robot_direction['right'] = fuzz.trapmf(self.robot_direction.universe, [0, 40, 95, 95])

    def _define_rules(self):
        self.rules = [
            # Правила для случая, когда цель находится прямо впереди
            ctrl.Rule(self.angle_to_goal['center'],
                      [self.robot_direction['center'], self.robot_speed['fast']]),

            # Правила в случае, когда цель справа или слева от робота
            ctrl.Rule(self.angle_to_goal['right'],
                      [self.robot_direction['right'], self.robot_speed['fast']]),
            ctrl.Rule(self.angle_to_goal['left'],
                      [self.robot_direction['left'], self.robot_speed['fast']]),
        ]

    def compute(self, angle):
        try:
            self.simulation.input['angle_to_goal'] = angle
            self.simulation.compute()
            return (self.simulation.output['robot_speed'],
                    self.simulation.output['robot_direction'])
        except:
            return 15, angle
