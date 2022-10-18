import math
from math import sin, cos, pi, atan2, asin
from time import sleep
from cellworld import World, Display, Location, Agent_markers, Step, Timer, Cell_map, Coordinates, Cell_group, Cell_group_builder
from cellworld_tracking import TrackingClient
from tcp_messages import MessageClient, Message
from json_cpp import JsonObject
from cellworld_controller_service import ControllerClient

# Globals
display = None
current_prey_destination = None
controller_state = None
possible_destinations = Cell_group()

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


def load_world():
    global display
    global world
    global possible_destinations

    occlusion = Cell_group_builder.get_from_name("hexagonal", occlusions + ".occlusions")
    possible_destinations = world.create_cell_group(Cell_group_builder.get_from_name("hexagonal", occlusions + ".predator_destinations"))
    world.set_occlusions(occlusion)
    display = Display(world, fig_size=(9.0*.75, 8.0*.75), animated=True)


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


def to_degrees(angle_rads):
    return angle_rads * 180/math.pi


def get_correction_location(current_location, get_rotation=False):
    '''
    Checks surrounding cells and provides new robot location
    '''
    current_id = world.cells.find(current_location)
    current_coordinate = world.cells[current_id].coordinates

    # holds move info
    moves_dict = {'0': [Coordinates(2,0), 90.0],
                  '1': [Coordinates(1,-1), 150.0],
                  '2': [Coordinates(-1,-1), 210.0],
                  '3': [Coordinates(-2,0), 270.0],
                  '4': [Coordinates(-1,1), 330.0],
                  '5': [Coordinates(1,1), 30.0]}

    # new coordinate closest open location
    move_check = ['2', '1', '0', '3', '4', '5']

    # find first open cell from move_check list
    for move in move_check:
        new_coordinate = current_coordinate + moves_dict[move][0]
        new_id = world.cells[map[new_coordinate]].id
        if world.cells[new_id].occluded == False:
            break

    if get_rotation:
        return moves_dict[move][1]

    return new_coordinate

def get_correction_rotation(location, destination, set_rotation_num):
    if set_rotation_num == 1:
        rotation = to_degrees(location.atan(destination))

    else:
        rotation = get_correction_location(destination, True)

    return rotation




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
    if event.key == "f":
        print("setting rotation to 0")
        controller.set_rotation(0)

    if event.key == "c":
        print("correcting robot position")
        current_robot_location = tick_robot.step.location
        # 1. get new coordinate
        predator_correction = get_correction_location(current_robot_location)
        # 2. get rotation 1 - rotation required to translate to new location
        rotation1 = get_correction_rotation(current_robot_location, current_predator_destination, 1)
        controller.set_rotation(rotation1)
        # 3. set destination
        controller.set_coordinate(predator_correction)
        # 4. get rotation 2 - rotation to prep for next move will be standard move rotation

        """
        #rotation2 = get_correction_rotation(0, current_predator_destination, 2)
        #controller.set_rotation(rotation2) # make it either 150 or 210 or ... based on position and where the occlusions are
        """




# World Setup
occlusions = "00_00"
# occlusions = "21_05"
world = World.get_from_parameters_names("hexagonal", "canonical", occlusions)
map = Cell_map(world.configuration.cell_coordinates)
load_world()

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