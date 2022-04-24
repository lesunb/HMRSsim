from simulator.typehints.component_types import Component


class NavToPoseRosGoal(Component):
    """
    Indicates that the entity has a ROS goal handle of a NavigateToPose goal.
    """

    def __init__(self):
        self.goal_handle = None
        self.x = None
        self.y = None

    def __str__(self):
        return "NavToPose"