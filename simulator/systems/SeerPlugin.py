import threading
import logging
import json
from queue import Queue
from typing import Callable, List
from simulator.typehints.dict_types import SystemArgs

from simpy import Environment
from esper import World

from simulator.components.Skeleton import Skeleton
from simulator.components.Position import Position

message_buffer = Queue()


def consumer_manager(consumers: List[Callable], also_log: bool):
    logger = logging.getLogger(__name__ + '.consumer')
    logging.addLevelName(25, 'SEER')
    logger.setLevel('SEER')
    while True:
        message, msg_idx = message_buffer.get()  # Blocking function
        if also_log:
            logger.log(25, message)
        for c in consumers:
            c(message, msg_idx)
        message_buffer.task_done()
        if 'theEnd' in message:
            break
    logger.log(25, f'Exiting consumer manager')
    return


def init(consumers: List[Callable], scan_interval: float, also_log=False):
    # Init consumer thread
    thread = threading.Thread(target=consumer_manager, args=[consumers, also_log])
    thread.start()

    # The producer thread

    def process(kwargs: SystemArgs):
        event_store = kwargs.get('EVENT_STORE', None)
        world: World = kwargs.get('WORLD', None)
        env: Environment = kwargs.get('ENV', None)
        msg_idx = 0
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
        message_buffer.put((base, msg_idx))
        msg_idx += 1
        # Scan simulation situation every scan_interval seconds and report
        last_round: dict = {}
        # Local ref most used functions
        get_components = world.get_components
        sleep = env.timeout
        while True:

            new_message = {
                "timestamp": round(float(env.now), 3)
            }
            for ent, (skeleton, position) in get_components(Skeleton, Position):
                if ent == 1:  # Entity 1 is the entire model
                    continue
                elif last_round.get(ent, (0, None))[0] != 0 and not position.changed and not skeleton.changed:
                    last_round[ent] = (2, skeleton.id)
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
                last_round[ent] = (2, skeleton.id)
                position.changed = False
                skeleton.changed = False
            # Check for deleted entities
            deleted = []
            for k, v in last_round.items():
                if v[0] == 2:
                    last_round[k] = (1, v[1])
                elif v[0] == 1:
                    deleted.append(v[1])
                    last_round[k] = (0, v[1])
            new_message['deleted'] = deleted
            # Add message to queue
            message_buffer.put((new_message, msg_idx))
            msg_idx += 1
            yield sleep(scan_interval)

    def clean():
        message_buffer.put(({"theEnd": True}, -1))
        logging.getLogger(__name__).debug(f'Executing Seer cleanup function')
        thread.join()

    return process, clean
