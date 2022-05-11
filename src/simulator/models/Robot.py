from simulator import primitives as primitives
from simulator.components.Velocity import Velocity
from simulator.components.NavToPoseRosGoal import NavToPoseRosGoal

from typing import Tuple, List
from simulator.typehints.component_types import Component

from simulator.models.Shape import from_object as shape_model_object

MODEL = 'robot'

def from_object(el, line_width=10) -> Tuple[List[Component], dict]:
    options = el.attrib
    components, style = shape_model_object(el, line_width)
    
    if options['type'] == 'robot':
        ros_goal_comp = NavToPoseRosGoal()
        if "name" in options:
            ros_goal_comp.name = options["name"]
        components.append(Velocity(x=0, y=0))
        components.append(ros_goal_comp)
    return components, options