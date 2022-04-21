from simulator.components.Position import Position
from simulator.components.Velocity import Velocity
from simulator.components.NavToPoseRosGoal import NavToPoseRosGoal
from simulator.typehints.ros_types import RosActionServer
from simulator.typehints.component_types import EVENT, GotoPosPayload, GotoPoiPayload, GotoPosEventTag, GotoPoiEventTag, EndOfPathTag

import logging

from typing import List

from nav2_msgs.action import NavigateToPose

from rclpy.action.server import ServerGoalHandle

class Nav2System(RosActionServer):
    """
    This system controls the navigation interface with ROS.
    For example, if a robot receives a goal through ROS to move to a point,
    this module will take this goal and translate it to a robot in HMRSim.
    """

    def __init__(self, **kwargs):
        super().__init__()
        self.logger = logging.getLogger(__name__)
        self.event_store = kwargs.get('event_store', None)
        self.exit_event = kwargs.get('exit_event', None)
        self.world = kwargs.get('world', None)
        self.destiny = None

    def process(self):
        for ent, (vel, pos, ros_goal) in self.world.get_components(Velocity, Position, NavToPoseRosGoal):
            if ros_goal.goal_handle is not None and ros_goal.goal_handle.is_active:
                # An EndOfPathTag indicates that a robot arrived
                end_event = self.event_store.get(lambda e: self.has_type_and_ent(e) and e.type == EndOfPathTag and e.ent == ent)
                if end_event is not None:
                    self.logger.info(f"The robot {ent} arrived at destination.")
                    ros_goal.goal_handle.execute()
                    ros_goal.clean_goal()
    
    def has_type_and_ent(self, ev):
        return hasattr(ev, "type") and hasattr(ev, "ent")

    def handle_accepted_goal(self, goal_handle: ServerGoalHandle):
        """
        This is a callback to be executed after a goal has just been accepted through ROS.
        """

        pose = goal_handle.request.pose.pose
        self.logger.info('Goal received: %s, %s' % (pose.position.x, pose.position.y))
        if not self.event_store:
            self.logger.warn('Could not find event store')
            return

        x = str(pose.position.x)
        y = str(pose.position.y)
        destiny = [x, y]

        for ent, (vel, pos, ros_goal) in self.world.get_components(Velocity, Position, NavToPoseRosGoal):
            if ros_goal.goal_handle is not None:
                self.logger.info("There is already a goal running")
                break
            ros_goal.goal_handle = goal_handle
            self.go_to(ent, destiny)

    def send_result(self, goal_handle: ServerGoalHandle):
        """
        For HMRSim, this is a callback that should be called after an entity has arrived
        in the destiny.
        """

        goal_handle.succeed()
        result = NavigateToPose.Result()
        return result

    def go_to(self, ent, args: List[str]):
        """
        This is gonna add a new event to move an entity towards it's destination.
        """

        if len(args) == 1: # it's a POI
            payload = GotoPoiPayload(ent, args[0])
            new_event = EVENT(GotoPoiEventTag, payload)
        elif len(args) == 2:
            payload = GotoPosPayload(ent, [float(args[0]), float(args[1])])
            new_event = EVENT(GotoPosEventTag, payload)
        else:
            raise Exception('GO instruction failed. Go <poi> OR Go <x> <y>')
        if new_event:
            self.event_store.put(new_event)

    def get_handle_accepted_goal_callback(self):
        return self.handle_accepted_goal

    def get_result_callback(self):
        return self.send_result
    
    def get_service_type(self):
        return NavigateToPose

    def get_name(self):
        return 'navigate_to_pose'