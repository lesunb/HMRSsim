from simulator.typehints.component_types import Component


class NavToPoseRosGoal(Component):
    """
    Indicates that the entity has a ROS goal handle of a NavigateToPose goal.
    """

    def __init__(self):
        self.goal_handle = None

    def clean_goal(self):
        self.goal_handle = None

    def __str__(self):
        return "NavToPose"