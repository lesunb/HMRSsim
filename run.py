import pyglet
import sys

from systems.MovementProcessor import MovementProcessor
from systems.CollisionProcessor import CollisionProcessor
from systems.RenderProcessor import RenderProcessor
from systems.PathProcessor import PathProcessor

import systems.GotoDESProcessor as gotoProcessor
import systems.MapDESProcessor as mapProcessor
import systems.StopCollisionDESProcessor as StopCollision
import systems.EnergyConsumptionDESProcessor as energySystem
import systems.ManageObjects as ObjectManager
import systems.ClawDESProcessor as ClawProcessor
from systems.ScriptEventsDES import init

extra_instructions = [
    (gotoProcessor.GotoInstructionId, gotoProcessor.goInstruction)
]
ScriptProcessor = init(extra_instructions)

from main import Simulator, EVENT

# Create a simulation with config
simulator = Simulator(sys.argv[1])
# Some simulator objects
width, height = simulator.window_dimensions
window = simulator.window
eventStore = simulator.KWARGS['EVENT_STORE']
exitEvent = simulator.EXIT_EVENT
env = simulator.ENV

# Defines and initializes esper.Processor for the simulation
normal_processors = [
    MovementProcessor(minx=0, miny=0, maxx=width, maxy=height),
    CollisionProcessor(),
    RenderProcessor(),
    PathProcessor()
]
# Defines DES processors
des_processors = [
    ClawProcessor.process,
    ObjectManager.process,
    StopCollision.process,
    gotoProcessor.process,
    mapProcessor.process,
    energySystem.process,
    ScriptProcessor
]
# Add processors to the simulation, according to processor type
for p in normal_processors:
    simulator.add_system(p)
for p in des_processors:
    simulator.add_DES_system(p)

# pyglet related stuff
KEYS = pyglet.window.key
GOTO = False
MAP = False
buff = []
@window.event
def on_text(text):
    if GOTO and text != 'G':
        buff.append(text)
    elif MAP:
        buff.append(text)

@window.event
def on_key_press(key, mod):
    global GOTO
    global MAP
    if not GOTO and mod & KEYS.MOD_SHIFT and key == KEYS.G:
        print(f"Goto key pressed. Usage: ^G ent-poi ENTER")
        GOTO = True
    if not MAP and mod & KEYS.MOD_SHIFT and key == KEYS.M:
        print(f"Map key pressed. Usage: ^M ent key")
        MAP = True
    if key == KEYS.P and not MAP and not GOTO:
        print('Removing')
        payload = ClawProcessor.GRAB_ClawPayload(ClawProcessor.ClawOps.GRAB, 'medicine', 2)
        event = EVENT(ClawProcessor.ClawTag, payload)
        eventStore.put(event)
    if key == KEYS.D and not MAP and not GOTO:
        print('Re-creating')
        payload = ClawProcessor.GRAB_ClawPayload(ClawProcessor.ClawOps.DROP, 'medicine', 2)
        event = EVENT(ClawProcessor.ClawTag, payload)

        eventStore.put(event)
    if key == KEYS.ENTER or key == KEYS.RETURN:
        if GOTO:
            ent, poi = "".join(buff).split('-')
            print(f"End of Goto sequence. Taking entity {ent} to point {poi}")
            payload = gotoProcessor.GotoPayload(int(ent), int(poi))
            new_event = EVENT(gotoProcessor.GotoEventTag, payload)
            eventStore.put(new_event)
        elif MAP:
            ent, key = "".join(buff[1:]).split(' ')
            print(f"End of Map sequence. Taking entity {ent} to path {key}")
            payload = mapProcessor.MapPayload(int(ent), key)
            new_event = EVENT(mapProcessor.MapEventTag, payload)
            eventStore.put(new_event)
        buff.clear()
        GOTO = False
        MAP = False


@window.event
def on_draw():
    # Clear the window to background color
    window.clear()
    # Draw the batch of Renderables:
    simulator.batch.draw()


@window.event
def on_close():
    global EXIT
    print(f'Exiting from window')
    EXIT = False
    exitEvent.succeed()


if __name__ == "__main__":
    # NOTE!  schedule_interval will automatically pass a "delta time" argument
    #        to world.process, so you must make sure that your Processor classes
    #        account for this. See the example Processors above.
    simulator.run()
