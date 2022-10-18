import math
from math import sin, cos, pi, atan2, asin
from time import sleep
from cellworld import *
from json_cpp import JsonObject
from cellworld_controller_service import ControllerClient
from cellworld_experiment_service import ExperimentClient
from random import choice, choices

# Globals
display = None
current_predator_destination = None
controller_state = None
possible_destinations = Cell_group()

episode_in_progress = False
experiment_log_folder = "/research/logsV2"
current_experiment_name = ""

pheromone_charge = .25
pheromone_decay = 1.0
pheromone_max = 50

possible_destinations = Cell_group()
possible_destinations_weights = []
spawn_locations = Cell_group()
spawn_locations_weights = []

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


def on_experiment_started(experiment):
    """
    To start experiment right click on map
    """
    print("Experiment started:", experiment)
    experiments[experiment.experiment_name] = experiment.copy()


def on_episode_finished(m):
    global episode_in_progress, current_predator_destination, inertia_buffer, display

    last_trajectory = Experiment.get_from_file(experiment_log_folder + "/" + current_experiment_name + "_experiment.json").episodes[-1].trajectories.get_agent_trajectory("prey")
    for step in last_trajectory:
        cell_id = world.cells[world.cells.find(step.location)].id
        for index, pd in enumerate(possible_destinations):
            if pd.id == cell_id:
                possible_destinations_weights[index] = min(possible_destinations_weights[index] + pheromone_charge, pheromone_max)

        for index, pd in enumerate(spawn_locations):
            if pd.id == cell_id:
                spawn_locations_weights[index] = min(spawn_locations_weights[index] + pheromone_charge, pheromone_max)

    cmap=plt.cm.Reds([w / max(spawn_locations_weights) for w in spawn_locations_weights])
    for i, sl in enumerate(spawn_locations):
        display.cell(cell=sl, color=cmap[i])

    controller.resume()
    controller.set_behavior(0)
    inertia_buffer = 1
    episode_in_progress = False
    current_predator_destination = choices(spawn_locations, weights=spawn_locations_weights)[0].location
    controller.set_destination(current_predator_destination)     # set destination
    destination_list.append(current_predator_destination)
    if controller_timer != 1: # no idea why the timer would be an integer but whatevs
        controller_timer.reset()                                     # reset controller timer
    display.circle(current_predator_destination, 0.01, "red")


def on_capture( frame:int ):
    global inertia_buffer
    controller.set_behavior(0)
    inertia_buffer = 1
    print ("PREY CAPTURED")


def on_episode_started(experiment_name):
    print("hi")
    global display, episode_in_progress, current_experiment_name
    current_experiment_name = experiment_name
    print("New Episode: ", experiment_name)
    print("Occlusions: ", experiments[experiment_name].world.occlusions)

def on_prey_entered_arena():
    global episode_in_progress
    episode_in_progress = True


def load_world():
    global display, world, possible_destinations
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

    global current_predator_destination
    if event.button == 1:
        location = Location(event.xdata, event.ydata)
        cell_id = world.cells.find(location)
        destination_cell = world.cells[cell_id]
        current_predator_destination = destination_cell.location
        controller.set_destination(destination_cell.location)
        display.circle(current_predator_destination, 0.01, "orange")
        print(location)
        controller_timer.reset()
    else:
        print("starting experiment")
        exp = experiment_service.start_experiment(                  # call start experiment
            prefix="PREFIX",
            suffix="SUFFIX",
            occlusions=occlusions,
            world_implementation="canonical",
            world_configuration="hexagonal",
            subject_name="SUBJECT",
            duration=10)
        print("Experiment Name: ", exp.experiment_name)
        r = experiment_service.start_episode(exp.experiment_name)   # call start episode
        print(f"R: {r}")

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
        predator_correction_coordinate = get_correction_location(current_robot_location)
        # 2. get rotation 1 - rotation required to translate to new location
        rotation1 = get_correction_rotation(current_robot_location, current_predator_destination, 1)
        controller.set_rotation(rotation1)
        # 3. set destination
        controller.set_coordinate(predator_correction_coordinate)
        # 4. get rotation 2 - rotation to prep for next move will be standard move rotation
        rotation2 = get_correction_rotation(0, current_predator_destination, 2)
        controller.set_rotation(rotation2) # make it either 150 or 210 or ... based on position and where the occlusions are




# World Setup
occlusions = "00_00"
# occlusions = "21_05"
world = World.get_from_parameters_names("hexagonal", "canonical", occlusions)
map = Cell_map(world.configuration.cell_coordinates)
load_world()

# Agent Setup
tick_robot = AgentData("predator") # robot with 3 leds recognized as predator by tracker

# Experiment Setup
experiment_service = ExperimentClient()
experiment_service.on_experiment_started = on_experiment_started
experiment_service.on_episode_started = on_episode_started
experiment_service.on_prey_entered_arena = on_prey_entered_arena
experiment_service.on_episode_finished = on_episode_finished
experiment_service.on_capture = on_capture
if not experiment_service.connect("127.0.0.1"):
    print("Failed to connect to experiment service")
    exit(1)
experiment_service.set_request_time_out(5000)
experiment_service.subscribe()                  # having issues subscribing to exp service
experiments = {}

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


# current_predator_destination = get_location(-5,7)
current_predator_destination = tick_robot.step.location  # initial predator destination
display.circle(current_predator_destination, 0.01, "cyan")

running = True
while running:
    if not controller_timer:
        print()
        controller.set_destination(current_predator_destination)
        controller_timer.reset()

    if tick_robot.is_valid:
        display.agent(step=tick_robot.step, color="blue", size= 15)
    else:
        display.agent(step=tick_robot.step, color="grey", size= 15)

    display.update()
    sleep(0.1)

# TODO: add pause feature