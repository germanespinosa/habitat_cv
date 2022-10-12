import math
from math import sin, cos, pi, atan2, asin
from time import sleep
from cellworld import World, Display, Location, Agent_markers, Step, Timer, Cell_map, Coordinates
from cellworld_tracking import TrackingClient
from tcp_messages import MessageClient, Message
from json_cpp import JsonObject
from cellworld_controller_service import ControllerClient

# Globals
current_prey_destination = None
controller_state = None

class AgentData:
    """
    Stores information about the agents
    """
    def __init__(self, agent_name: str):
        self.is_valid = None
        self.step = Step()                      # step object - step is a class in experiment.py
        self.step.agent_name = agent_name
        self.move_state = None
        self.move_done = False

def on_step(step):
    """
    Updates steps and predator behavior
    """
    time_out = 1.0
    if step.agent_name == "predator":
        tick_robot.is_valid = Timer(time_out)
        tick_robot.step = step

def get_location(x, y):
    '''
    Converts x and y coordinates to locations
    :param x: (int)  x coordinate
    :param y: (int) y coordinate
    :return: location
    '''
    return world.cells[map[Coordinates(x, y)]].location

def on_click(event):
    """
    Updates desired robot destination
    :param event: click on figure
    """

    global current_prey_destination
    if event.button == 1:
        location = Location(event.xdata, event.ydata)
        cell_id = world.cells.find(location)
        destination_cell = world.cells[cell_id]
        current_prey_destination = destination_cell.location
        controller.set_destination(destination_cell.location)
        display.circle(current_prey_destination, 0.01, "orange")
        print(location)
        controller_timer.reset()

def on_keypress(event):
    """
    Sets up keyboard intervention
    """
    global running
    global current_predator_destination
    global controller_timer
    global destination_list
    global controller_state

    if event.key == "p": # To pause auto robot
        print("pause")
        controller.pause()
        controller_state = 0
    if event.key == "r":
        print("resume")
        controller.resume()
        controller_state = 1




# World Setup
occlusions = "00_00"
world = World.get_from_parameters_names("hexagonal", "canonical", occlusions)
display = Display(world, fig_size=(9.0*.75, 8.0*.75), animated=True)
map = Cell_map(world.configuration.cell_coordinates)

# Agent Setup
tick_robot = AgentData("predator") # robot with 3 leds recognized as predator by tracker

# Controller Setup
controller_timer = Timer(3.0)
controller = ControllerClient()
if not controller.connect("127.0.0.1", 4590):
    print("failed to connect to the controller")
    exit(1)
controller.set_request_time_out(10000)
controller.subscribe()
controller.on_step = on_step

# Click Input Setup
cid1 = display.fig.canvas.mpl_connect('button_press_event', on_click)
cid_keypress = display.fig.canvas.mpl_connect('key_press_event', on_keypress)

current_prey_destination = get_location(-5,7)
display.circle(current_prey_destination, 0.01, "cyan")

#current_predator_destination = tick_robot.step.location  # initial predator destination

running = True
while running:
    if not controller_timer:
        print()
        controller.set_destination(current_prey_destination)
        controller_timer.reset()

    if tick_robot.is_valid:
        display.agent(step=tick_robot.step, color="blue", size= 15)
    else:
        display.agent(step=tick_robot.step, color="grey", size= 15)

    display.update()
    sleep(0.1)

# TODO: add pause feature