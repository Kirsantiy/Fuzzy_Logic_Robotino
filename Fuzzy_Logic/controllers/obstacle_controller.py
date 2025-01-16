import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
# obstacle_controller.py
class ObstacleController:
    def __init__(self):
        # Input variables
        self.angle_to_obstacle = ctrl.Antecedent(np.arange(-90, 91, 1), 'Angle to obstacle')
        self.distance_to_obstacle = ctrl.Antecedent(np.arange(0, 0.41, 0.01), 'Distance to obstacle')

        # Output variables
        self.robot_speed = ctrl.Consequent(np.arange(0, 0.31, 0.01), 'Robot speed')
        self.robot_direction = ctrl.Consequent(np.arange(-90, 91, 1), 'Robot direction')

        self._define_memberships()
        self._define_rules()

        self.controller = ctrl.ControlSystem(self.rules)
        self.simulation = ctrl.ControlSystemSimulation(self.controller)

    def _define_memberships(self):
        # Angle memberships
        self.angle_to_obstacle['Left'] = fuzz.trapmf(self.angle_to_obstacle.universe, [-90, -90, -40, 0])
        self.angle_to_obstacle['Front'] = fuzz.trimf(self.angle_to_obstacle.universe, [-40, 0, 40])
        self.angle_to_obstacle['Right'] = fuzz.trapmf(self.angle_to_obstacle.universe, [0, 40, 90, 90])

        # Distance memberships
        self.distance_to_obstacle['Near'] = fuzz.trapmf(self.distance_to_obstacle.universe, [0, 0, 0.20, 0.35])
        self.distance_to_obstacle['Far'] = fuzz.trapmf(self.distance_to_obstacle.universe, [0.20, 0.35, 0.40, 0.40])

        # Speed memberships
        self.robot_speed['Slow'] = fuzz.trapmf(self.robot_speed.universe, [0, 0, 0.10, 0.25])
        self.robot_speed['Medium'] = fuzz.trimf(self.robot_speed.universe, [0.10, 0.20, 0.25])
        self.robot_speed['Fast'] = fuzz.trapmf(self.robot_speed.universe, [0.20, 0.25, 0.30, 0.30])

        # Direction memberships
        self.robot_direction['Left'] = fuzz.trapmf(self.robot_direction.universe, [-90, -90, -40, 0])
        self.robot_direction['Front'] = fuzz.trimf(self.robot_direction.universe, [-40, 0, 40])
        self.robot_direction['Right'] = fuzz.trapmf(self.robot_direction.universe, [0, 40, 90, 90])

    def _define_rules(self):
        self.rules = [
            ctrl.Rule(self.distance_to_obstacle['Near'] & self.angle_to_obstacle['Front'],
                      [self.robot_direction['Left'], self.robot_speed['Slow']]),
            ctrl.Rule(self.distance_to_obstacle['Near'] & self.angle_to_obstacle['Left'],
                      [self.robot_direction['Right'], self.robot_speed['Slow']]),
            ctrl.Rule(self.distance_to_obstacle['Near'] & self.angle_to_obstacle['Right'],
                      [self.robot_direction['Left'], self.robot_speed['Slow']]),
            ctrl.Rule(self.distance_to_obstacle['Far'],
                      self.robot_speed['Medium'])
        ]

    def compute(self, angle, distance):
        self.simulation.input['Angle to obstacle'] = angle
        self.simulation.input['Distance to obstacle'] = distance
        self.simulation.compute()
        return (self.simulation.output['Robot speed'],
                self.simulation.output['Robot direction'])
