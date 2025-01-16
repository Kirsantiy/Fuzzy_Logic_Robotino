# goal_controller.py
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl


class GoalController:
    def __init__(self):
        # Input variables
        self.angle_to_goal = ctrl.Antecedent(np.arange(-90, 91, 1), 'Angle to goal')

        # Output variables
        self.robot_speed = ctrl.Consequent(np.arange(0, 0.31, 0.01), 'Robot speed')
        self.robot_direction = ctrl.Consequent(np.arange(-90, 91, 1), 'Robot direction')

        self._define_memberships()
        self._define_rules()

        self.controller = ctrl.ControlSystem(self.rules)
        self.simulation = ctrl.ControlSystemSimulation(self.controller)

    def _define_memberships(self):
        # Angle to goal memberships
        self.angle_to_goal['Left'] = fuzz.trapmf(self.angle_to_goal.universe, [-90, -90, -40, 0])
        self.angle_to_goal['Front'] = fuzz.trimf(self.angle_to_goal.universe, [-40, 0, 40])
        self.angle_to_goal['Right'] = fuzz.trapmf(self.angle_to_goal.universe, [0, 40, 90, 90])

        # Speed memberships
        self.robot_speed['Slow'] = fuzz.trapmf(self.robot_speed.universe, [0, 0, 0.10, 0.25])
        self.robot_speed['Fast'] = fuzz.trapmf(self.robot_speed.universe, [0.10, 0.25, 0.30, 0.30])

        # Direction memberships
        self.robot_direction['Left'] = fuzz.trapmf(self.robot_direction.universe, [-90, -90, -40, 0])
        self.robot_direction['Front'] = fuzz.trimf(self.robot_direction.universe, [-40, 0, 40])
        self.robot_direction['Right'] = fuzz.trapmf(self.robot_direction.universe, [0, 40, 90, 90])

    def _define_rules(self):
        self.rules = [
            ctrl.Rule(self.angle_to_goal['Left'],
                      [self.robot_direction['Left'], self.robot_speed['Fast']]),
            ctrl.Rule(self.angle_to_goal['Front'],
                      [self.robot_direction['Front'], self.robot_speed['Fast']]),
            ctrl.Rule(self.angle_to_goal['Right'],
                      [self.robot_direction['Right'], self.robot_speed['Fast']])
        ]

    def compute(self, angle):
        self.simulation.input['Angle to goal'] = angle
        self.simulation.compute()
        return (self.simulation.output['Robot speed'],
                self.simulation.output['Robot direction'])
