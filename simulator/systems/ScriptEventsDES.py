from typing import NamedTuple, List, Tuple, Callable
from typehints.dict_types import SystemArgs

from esper import World
from simpy import FilterStore, Environment
import logging

from typehints.component_types import EVENT, ERROR
from systems.PathProcessor import EndOfPathTag

import components.Script as scriptComponent


ExecutePayload = NamedTuple('ExecuteScriptInstructionPayload', [('ent', int)])
ExecuteInstructionTag = 'ExecuteInstruction'
ExtraInstruction = Tuple[str, Callable[[int, List[str], scriptComponent.Script, FilterStore], scriptComponent.States]]


def init(extra_instructions: List[ExtraInstruction], watch_list: List[str]):
    logger = logging.getLogger(__name__)
    instruction_set = {t[0]: t[1] for t in extra_instructions}
    watchlist = [ExecuteInstructionTag, EndOfPathTag] + watch_list
    logger.debug(f'My instruction set: {instruction_set}')
    logger.debug(f'My Watchlist: {watchlist}')

    def process(kwargs: SystemArgs):
        # Init
        __event_store = kwargs.get('EVENT_STORE', None)
        __world: World = kwargs.get('WORLD', None)
        env: Environment = kwargs.get('ENV', None)
        if __event_store is None:
            raise Exception("Can't find eventStore")
        # On the first run, we put all the scripts in the world in the event queue
        for ent, Script in __world.get_component(scriptComponent.Script):
            payload = ExecutePayload(ent=ent)
            new_event = EVENT(ExecuteInstructionTag, payload)
            __event_store.put(new_event)

        # Now we keep checking for pending events and executing them
        while True:
            ev = yield __event_store.get(lambda e: e.type in watchlist or type(e) == ERROR)
            payload = ev.payload
            if type(ev) == ERROR:
                # Here we handle errors that occurred in the processing of some entity's script
                script = __world.component_for_entity(ev.ent, scriptComponent.Script)
                error_handlers = script.error_handlers
                handler = error_handlers.get(ev.type, None)
                if handler is None:
                    handler = error_handlers.get(script.default_error_tag, None)
                if handler is not None:
                    handler(ev.payload, kwargs)
                else:
                    logger.error(f'No handler for error {ERROR} was found')
                    script.logs.append(f'Received error {ERROR}, no handler was found.')
            else:
                try:
                    script = __world.component_for_entity(payload.ent, scriptComponent.Script)
                except KeyError:
                    logger.warning(f'[ENV TIME {env.now}] Got event {ev.type} for an entity without Script')
                    continue
                if ev.type == ExecuteInstructionTag:
                    if script.state != scriptComponent.States.READY:
                        logger.warning(f'Request to execute script not ready')
                        script.logs.append(
                            f'[{env.now}] Request to execute. Script not in READY state. (Curr state is {script.state})'
                        )
                    i_type, *args = script.instructions[script.curr_instruction].split(' ')
                    next_state: scriptComponent.States
                    if i_type in instruction_set:
                        next_state = instruction_set[i_type](payload.ent, args, script, __event_store)
                        script.logs.append(f'[{env.now}] Execute instruction {i_type}. In state {next_state}')
                    else:
                        logger.error(f'Unknown instruction {i_type}')
                        script.logs.append(f'[{env.now}] Executing instruction {i_type} Failed. Unknown instruction.')
                        next_state = scriptComponent.States.READY
                    if next_state == scriptComponent.States.READY:
                        payload = ExecutePayload(payload.ent)
                        new_event = EVENT(ExecuteInstructionTag, payload)
                        __event_store.put(new_event)
                else:
                    ent = payload.ent
                    logger.debug(f'[{env.now}] Got event {ev.type} for ent {ent} - {ev.payload}')
                    if ev.type not in script.expecting:
                        logger.warning(f'Was not expecting {ev.type}')
                    else:
                        script.expecting.remove(ev.type)
                        if not script.expecting:
                            r = unblockEntity(script)
                            if r == scriptComponent.States.READY:
                                payload = ExecutePayload(ent=ent)
                                new_event = EVENT(ExecuteInstructionTag, payload)
                                __event_store.put(new_event)

    return process


def unblockEntity(script: scriptComponent.Script) -> scriptComponent.States:
    script.curr_instruction += 1
    if script.curr_instruction == len(script.instructions):
        script.state = scriptComponent.States.DONE
    else:
        script.state = scriptComponent.States.READY
    return script.state
