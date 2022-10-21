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
    # print("Experiment started:", experiment)
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
    # print ("PREY CAPTURED")


def on_episode_started(experiment_name):
    global display, episode_in_progress, current_experiment_name
    current_experiment_name = experiment_name
    # print("New Episode: ", experiment_name)
    # print("Occlusions: ", experiments[experiment_name].world.occlusions)

def on_prey_entered_arena():
    global episode_in_progress
    episode_in_progress = True


def load_world():
    global display, world, possible_destinations
    occlusion = Cell_group_builder.get_from_name("robot", occlusions + ".occlusions")
    #possible_destinations = world.create_cell_group(Cell_group_builder.get_from_name("robot", occlusions + ".predator_destinations"))
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
    return world.cells[map[Coordinates(x, y)]].location

def get_location2(coordinate):
    return world.cells[map[coordinate]].location


def to_degrees(angle_rads):
    ang = angle_rads * 180/math.pi
    if ang < 0:
        ang = 360 + ang
    return ang

def unnormalize(ang):
    if ang < 0:
        ang = 360 + ang
    return ang



def get_idl(location):
    return world.cells.find(location)


def get_idc(coordinate):
    return world.cells[map[coordinate]].id


def get_coordinate(id):
    return world.cells[id].coordinates


def get_correction(current_location, current_rotation):
    """
    Find the correction cell
    It must: 1. exist, 2. be unoccluded, 3. be closest relative to current robot orientation

    Input
    : location - current robot location
    : rotation - current robot angle
    """
    moves = [Coordinates(2,0), Coordinates(1,-1), Coordinates(-1,-1), Coordinates(-2,0), Coordinates(-1,1), Coordinates(1,1)]
    move_angles = [90.0, 150.0, 210.0, 270.0, 330.0, 30.0]

    current_id = get_idl(current_location)
    current_coordinate = get_coordinate(current_id)

    possible_moves = [] # indices of possible moves
    possible_coordinates = []

    # find moves that exist and are not occluded
    for move in moves:
        new_coordinate = current_coordinate + move
        if new_coordinate in coord_list:
            new_id = get_idc(new_coordinate)

            if not world.cells[new_id].occluded:
                possible_moves.append(moves.index(move))
                possible_coordinates.append(new_coordinate)

    new_coordinate = (min(possible_coordinates, key= lambda x:abs(to_degrees(current_location.atan(world.cells[map[x]].location)) - current_rotation)))
    rotation1 = to_degrees(current_location.atan(get_location2(new_coordinate)))

    selected_move = possible_moves[possible_coordinates.index(new_coordinate)]
    rotation2 = move_angles[selected_move]

    return new_coordinate, rotation1, rotation2



def on_click(event):
    """
    Updates desired robot destination
    :param event: click on figure
    """

    global current_predator_destination
    if event.button == 1 and controller_state:
        location = Location(event.xdata, event.ydata)
        cell_id = world.cells.find(location)
        destination_cell = world.cells[cell_id]
        current_predator_destination = destination_cell.location
        controller.set_destination(destination_cell.location)
        display.circle(current_predator_destination, 0.01, "orange")
        controller_timer.reset()
    else:
        # print("starting experiment")
        exp = experiment_service.start_experiment(                  # call start experiment
            prefix="PREFIX",
            suffix="SUFFIX",
            occlusions=occlusions,
            world_implementation="canonical",
            world_configuration="hexagonal",
            subject_name="SUBJECT",
            duration=10)
        # print("Experiment Name: ", exp.experiment_name)
        r = experiment_service.start_episode(exp.experiment_name)   # call start episode
        # print(f"R: {r}")

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
    if event.key == "z":
        id = get_idl(tick_robot.step.location)
        print(get_coordinate(id))

    if event.key == "c":
        controller.pause()
        controller_state = 0
        # maybe puase controller here
        print("correcting robot position")
        current_robot_location = tick_robot.step.location
        current_predator_rotation = unnormalize(tick_robot.step.rotation)
        correction_coordinate, rotation1, rotation2 = get_correction(current_robot_location,current_predator_rotation)
        current_predator_destination = get_location2(correction_coordinate)

        controller.set_rotation(rotation1)
        controller.set_coordinate(correction_coordinate)
        controller.set_rotation(rotation2) # make it either 150 or 210 or ... based on position and where the occlusions are

        display.circle(current_robot_location, 0.005, "blue")
        display.circle(current_predator_destination, 0.005, "red")
        display.update()

        controller.resume()
        controller_state = 1




# World Setup
# occlusions = "00_00"
occlusions = "21_05"
world = World.get_from_parameters_names("robot", "canonical", occlusions)
map = Cell_map(world.configuration.cell_coordinates)
load_world()
coord_list = world.cells.get('coordinates')

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
controller_state = 1
while running:
    # if not controller_timer and controller_state == 1:
    #     # print("SENDING IT AGAIN!")
    #     controller.set_destination(current_predator_destination)
    #     controller_timer.reset()

    if tick_robot.is_valid:
        display.agent(step=tick_robot.step, color="blue", size= 15)
    else:
        display.agent(step=tick_robot.step, color="grey", size= 15)

    display.update()
    sleep(0.1)

# TODO: add pause feature