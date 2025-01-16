# obstacle_controller.py
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

class ObstacleController:
    def __init__(self):
        # Input variables
        self.angle_to_obstacle = ctrl.Antecedent(np.arange(-90, 91, 1), 'angle_to_obstacle')
        self.distance_to_obstacle = ctrl.Antecedent(np.arange(0, 0.4, 0.01), 'distance_to_obstacle')

        # Output variables
        self.robot_speed = ctrl.Consequent(np.arange(0, 0.2, 0.01), 'robot_speed')
        self.robot_direction = ctrl.Consequent(np.arange(-90, 91, 1), 'robot_direction')

        self._define_memberships()
        self._define_rules()

        self.controller = ctrl.ControlSystem(self.rules)
        self.simulation = ctrl.ControlSystemSimulation(self.controller)

    def _define_memberships(self):
        # Более детальные функции принадлежности для угла до препятствия
        self.angle_to_obstacle['very_left'] = fuzz.trapmf(self.angle_to_obstacle.universe, [-90, -90, -75, -60])
        self.angle_to_obstacle['left'] = fuzz.trimf(self.angle_to_obstacle.universe, [-75, -45, -15])
        self.angle_to_obstacle['slightly_left'] = fuzz.trimf(self.angle_to_obstacle.universe, [-30, -15, 0])
        self.angle_to_obstacle['center'] = fuzz.trimf(self.angle_to_obstacle.universe, [-15, 0, 15])
        self.angle_to_obstacle['slightly_right'] = fuzz.trimf(self.angle_to_obstacle.universe, [0, 15, 30])
        self.angle_to_obstacle['right'] = fuzz.trimf(self.angle_to_obstacle.universe, [15, 45, 75])
        self.angle_to_obstacle['very_right'] = fuzz.trapmf(self.angle_to_obstacle.universe, [60, 75, 90, 90])

        # Более детальные функции принадлежности для расстояния
        self.distance_to_obstacle['extremely_close'] = fuzz.trimf(self.distance_to_obstacle.universe, [0, 0, 0.15])
        self.distance_to_obstacle['very_close'] = fuzz.trimf(self.distance_to_obstacle.universe, [0.10, 0.20, 0.30])
        self.distance_to_obstacle['close'] = fuzz.trimf(self.distance_to_obstacle.universe, [0.25, 0.35, 0.45])
        self.distance_to_obstacle['medium'] = fuzz.trimf(self.distance_to_obstacle.universe, [0.40, 0.50, 0.60])
        self.distance_to_obstacle['far'] = fuzz.trimf(self.distance_to_obstacle.universe, [0.55, 0.65, 75])
        self.distance_to_obstacle['very_far'] = fuzz.trimf(self.distance_to_obstacle.universe, [70, 80, 80])

        # Более детальное управление скоростью
        self.robot_speed['stop'] = fuzz.trimf(self.robot_speed.universe, [0, 0, 0.05])
        self.robot_speed['very_slow'] = fuzz.trimf(self.robot_speed.universe, [0, 0.05, 0.10])
        self.robot_speed['slow'] = fuzz.trimf(self.robot_speed.universe, [0.05, 0.10, 0.15])
        self.robot_speed['medium'] = fuzz.trimf(self.robot_speed.universe, [0.10, 0.15, 0.20])
        self.robot_speed['fast'] = fuzz.trimf(self.robot_speed.universe, [0.15, 0.25, 0.30])

        # Более детальное управление поворотами
        self.robot_direction['sharp_left'] = fuzz.trapmf(self.robot_direction.universe, [-90, -90, -75, -60])
        self.robot_direction['left'] = fuzz.trimf(self.robot_direction.universe, [-75, -45, -15])
        self.robot_direction['slight_left'] = fuzz.trimf(self.robot_direction.universe, [-30, -15, 0])
        self.robot_direction['center'] = fuzz.trimf(self.robot_direction.universe, [-15, 0, 15])
        self.robot_direction['slight_right'] = fuzz.trimf(self.robot_direction.universe, [0, 15, 30])
        self.robot_direction['right'] = fuzz.trimf(self.robot_direction.universe, [15, 45, 75])
        self.robot_direction['sharp_right'] = fuzz.trapmf(self.robot_direction.universe, [60, 75, 90, 90])

    def _define_rules(self):
        self.rules = [
            # # Правила для экстремально близких препятствий - экстренное уклонение
            # ctrl.Rule(self.distance_to_obstacle['extremely_close'] & self.angle_to_obstacle['center'],
            #          [self.robot_direction['sharp_left'], self.robot_speed['stop']]),
            # ctrl.Rule(self.distance_to_obstacle['extremely_close'] & self.angle_to_obstacle['slightly_left'],
            #          [self.robot_direction['sharp_right'], self.robot_speed['stop']]),
            # ctrl.Rule(self.distance_to_obstacle['extremely_close'] & self.angle_to_obstacle['slightly_right'],
            #          [self.robot_direction['sharp_left'], self.robot_speed['stop']]),
            #
            # # Правила для очень близких препятствий - сильное уклонение
            # ctrl.Rule(self.distance_to_obstacle['very_close'] & self.angle_to_obstacle['center'],
            #          [self.robot_direction['left'], self.robot_speed['very_slow']]),
            # ctrl.Rule(self.distance_to_obstacle['very_close'] & self.angle_to_obstacle['slightly_left'],
            #          [self.robot_direction['right'], self.robot_speed['very_slow']]),
            # ctrl.Rule(self.distance_to_obstacle['very_close'] & self.angle_to_obstacle['slightly_right'],
            #          [self.robot_direction['left'], self.robot_speed['very_slow']]),
            #
            # # Правила для близких препятствий - умеренное уклонение
            # ctrl.Rule(self.distance_to_obstacle['close'] & self.angle_to_obstacle['center'],
            #          [self.robot_direction['slight_left'], self.robot_speed['slow']]),
            # ctrl.Rule(self.distance_to_obstacle['close'] & self.angle_to_obstacle['left'],
            #          [self.robot_direction['slight_right'], self.robot_speed['slow']]),
            # ctrl.Rule(self.distance_to_obstacle['close'] & self.angle_to_obstacle['right'],
            #          [self.robot_direction['slight_left'], self.robot_speed['slow']]),
            #
            # # Правила для препятствий на среднем расстоянии - легкое уклонение
            # ctrl.Rule(self.distance_to_obstacle['medium'] & self.angle_to_obstacle['center'],
            #          [self.robot_direction['slight_left'], self.robot_speed['medium']]),
            # ctrl.Rule(self.distance_to_obstacle['medium'] & self.angle_to_obstacle['slightly_left'],
            #          [self.robot_direction['slight_right'], self.robot_speed['medium']]),
            # ctrl.Rule(self.distance_to_obstacle['medium'] & self.angle_to_obstacle['slightly_right'],
            #          [self.robot_direction['slight_left'], self.robot_speed['medium']]),
            #
            # # Правила для дальних препятствий - нормальное движение
            # ctrl.Rule(self.distance_to_obstacle['far'],
            #          [self.robot_direction['center'], self.robot_speed['fast']]),
            # ctrl.Rule(self.distance_to_obstacle['very_far'],
            #          [self.robot_direction['center'], self.robot_speed['fast']])
        ]

    def compute(self, angle, distance):
        try:
            self.simulation.input['angle_to_obstacle'] = angle
            self.simulation.input['distance_to_obstacle'] = distance * 100
            self.simulation.compute()
            return (self.simulation.output['robot_speed'],
                   self.simulation.output['robot_direction'])
        except:
            return 5, -angle
