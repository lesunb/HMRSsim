from simulator.components.Position import Position
from simulator.components.Velocity import Velocity
from simulator.components.Path import Path
from simulator.components.NavToPoseRosGoal import NavToPoseRosGoal
from simulator.typehints.ros_types import RosActionServer
from simulator.typehints.component_types import EVENT, GotoPosPayload, GotoPoiPayload, GotoPosEventTag, GotoPoiEventTag, EndOfPathTag
from simulator.typehints.dict_types import SystemArgs

import logging

from typing import List

from nav2_msgs.action import NavigateToPose

from rclpy.action import CancelResponse
from rclpy.action.server import ServerGoalHandle

import math

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

    def end_path_event_listener(self, kwargs: SystemArgs):
        """
        This method waits for an event that indicates that an entity arrived at destiny is triggered.
        When a ROS entity arrives at destination, a result is sent to the client.
        """
        while True:
            # An EndOfPathTag indicates that a robot arrived
            end_event = yield self.event_store.get(lambda e: e.type == EndOfPathTag)

            for ent, (vel, pos, ros_goal) in self.world.get_components(Velocity, Position, NavToPoseRosGoal):
                if end_event.payload.ent == ent and ros_goal.goal_handle is not None:
                    self.logger.info(f"The robot {ent} arrived at destination.")
                    ros_goal.goal_handle.execute()
                    ros_goal.goal_handle = None
                    break

    def process(self):
        """
        This is made to be executed in each time a ROS controling system is executed.
        This method will send feedback to the client with te states of the robots.
        """

        for ent, (vel, pos, ros_goal) in self.world.get_components(Velocity, Position, NavToPoseRosGoal):
            if ros_goal.goal_handle is not None and ros_goal.goal_handle.is_active:
                if pos.changed:
                    # Publishing feedback
                    feedback = NavigateToPose.Feedback()
                    feedback.current_pose.pose.position.x = pos.x
                    feedback.current_pose.pose.position.y = pos.y
                    feedback.distance_remaining = math.dist([pos.x, pos.y], [ros_goal.x, ros_goal.y])
                    ros_goal.goal_handle.publish_feedback(feedback)
    
    def has_type_payload(self, ev):
        return hasattr(ev, "type") and hasattr(ev, "payload")

    def handle_accepted_goal(self, goal_handle: ServerGoalHandle):
        """
        This is a callback to be executed after a goal had just been accepted through ROS.
        """

        pose = goal_handle.request.pose.pose
        self.logger.info('Goal received: %s, %s' % (pose.position.x, pose.position.y))
        if not self.event_store:
            self.logger.warn('Could not find event store')
            return

        for ent, (vel, pos, ros_goal) in self.world.get_components(Velocity, Position, NavToPoseRosGoal):
            if ros_goal.goal_handle is not None:
                self.logger.info("There is already a goal running")
                break
            ros_goal.goal_handle = goal_handle
            ros_goal.x = pose.position.x
            ros_goal.y = pose.position.y
            self.go_to(ent, [str(pose.position.x), str(pose.position.y)])

    def send_result(self, goal_handle: ServerGoalHandle):
        """
        For HMRSim, this is a callback that should be called after an entity had arrived
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

    def cancel(self, goal_handle: ServerGoalHandle):
        """
        This is gonna be executed when a cancelation is requested.
        """
        self.logger.info("Cancel requested...")
        for ent, (vel, pos, ros_goal) in self.world.get_components(Velocity, Position, NavToPoseRosGoal):
            if ros_goal.goal_handle == goal_handle:
                vel.x = 0
                vel.y = 0
                vel.alpha = 0
                self.world.remove_component(ent, Path)
                ros_goal.goal_handle = None
                self.logger.info("Cancelation accepted")
                return CancelResponse.ACCEPT
        return CancelResponse.REJECT

    def get_cancel_callback(self):
        return self.cancel

    def get_handle_accepted_goal_callback(self):
        return self.handle_accepted_goal

    def get_result_callback(self):
        return self.send_result
    
    def get_service_type(self):
        return NavigateToPose

    def get_name(self):
        return 'navigate_to_pose'
