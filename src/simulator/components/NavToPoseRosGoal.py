from simulator.typehints.component_types import Component


class NavToPoseRosGoal(Component):
    """
    Indicates that the entity has a ros goal handle of a Navigate to pose goal
    """
    def __init__(self):
        self.goal_handle = None

    def clean_goal(self):
        self.goal_handle = None

    def __str__(self):
        return "NavToPose"