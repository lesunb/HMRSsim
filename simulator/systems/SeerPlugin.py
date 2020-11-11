import threading
import logging
import json
from queue import Queue
from typing import Callable

from simpy import Environment
from esper import World

from components.Skeleton import Skeleton
from components.Position import Position

message_buffer = Queue()


def default_consumer():
    logger = logging.getLogger(__name__ + '.consumer')
    while True:
        message = message_buffer.get()  # Blocking function
        logger.info(message)
        message_buffer.task_done()


def init(consumer: Callable, scan_interval: float):
    # Init consumer thread
    threading.Thread(target=consumer, daemon=True).start()
    # The producer thread

    def process(kwargs):
        event_store = kwargs.get('EVENT_STORE', None)
        world: World = kwargs.get('WORLD', None)
        env: Environment = kwargs.get('ENV', None)
        if event_store is None:
            raise Exception("Can't find eventStore")
        elif env is None:
            raise Exception("Can't find env")

        # Puts information about the simulation as the first message in the queue
        # window name, width and height
        simulation_skeleton = world.component_for_entity(1, Skeleton)
        size = json.loads(simulation_skeleton.style)
        base = {
            "timestamp": -1,
            "window_name": simulation_skeleton.id,
            "dimensions": size
        }
        message_buffer.put(base)
        # Scan simulation situation every scan_interval seconds and report
        while True:
            yield env.timeout(scan_interval)

            new_message = {
                "timestamp": env.now
            }
            for ent, (skeleton, position) in world.get_components(Skeleton, Position):
                # TODO: keep map of entities sent on the last time?
                # So we can see if a new one is found
                # Or if any of them is missing (was deleted)
                if ent == 1:  # Entity 1 is the entire model
                    continue
                if not position.changed:
                    continue

                data = {
                    'value': skeleton.value,
                    'x': position.x,
                    'y': position.y,
                    'width': position.w,
                    'height': position.h,
                    'style': skeleton.style
                }

                new_message[skeleton.id] = data
            # Add message to queue
            message_buffer.put(new_message)

    return process
