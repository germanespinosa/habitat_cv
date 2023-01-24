from cellworld import *
from json_cpp import *
import sys

class Frame_info(JsonObject):
    def __init__(self):
        self.frame_number = int(0)
        self.mouse = bool()
        self.robot = bool()
        self.puff = bool()
        self.mouse_location = Location()
        JsonObject.__init__(self)


class Frames_info(JsonList):
    def __init__(self):
        JsonList.__init__(self, list_type=Frame_info)


class Complete_frame(JsonObject):

    def __init__(self):
        self.index = int()
        self.frame = int()
        self.time_stamp = float()
        self.mouse_location = Location()
        self.mouse_rotation = float()
        self.mouse_data = str()
        self.mouse = bool()
        self.robot_location = Location()
        self.robot_rotation = float()
        self.robot_data = str()
        self.robot = bool()
        self.puff = bool(False)
        JsonObject.__init__(self)

class Episode_frames(JsonList):

    def __int__(self):
        self.__init__(self, list_type=Complete_frame)

    def to_trajectories(self):
        t = Trajectories()
        for i in self:
            if i.robot:
                robot_step = Step()
                robot_step.frame = i.frame
                robot_step.time_stamp = i.time_stamp
                robot_step.data = i.robot_data
                robot_step.location = i.robot_location
                robot_step.rotation = i.robot_rotation
                robot_step.agent_name = "predator"
                t.append(robot_step)
            if i.mouse:
                mouse_step = Step()
                mouse_step.frame = i.frame
                mouse_step.time_stamp = i.time_stamp
                mouse_step.data = i.mouse_data
                mouse_step.location = i.mouse_location
                mouse_step.rotation = i.mouse_rotation
                mouse_step.agent_name = "prey"
                t.append(mouse_step)
        return t

def get_next_step(steps, last_time_stamp):
    for step in steps:
        if step.time_stamp > last_time_stamp:
            return step
    return None


def get_episode_frames(steps: Trajectories) -> Episode_frames:
    prey_steps = steps.get_agent_trajectory("prey")
    pre_video = steps.get_agent_trajectory("predator").where("time_stamp", prey_steps[0].time_stamp, "<")
    predator_steps = steps.get_agent_trajectory("predator").where("time_stamp", prey_steps[0].time_stamp, ">=")
    episode_frames = Episode_frames()
    last_time_stamp = 0
    last_frame = -1
    for pi, prey_step in enumerate(prey_steps):
        prey_time_stamp = prey_step.time_stamp
        predator_step = get_next_step(predator_steps, last_time_stamp)
        while(predator_step and predator_step.time_stamp < prey_time_stamp):
            predator_only_frame = Complete_frame()
            predator_only_frame.frame = predator_step.frame
            predator_only_frame.time_stamp = predator_step.time_stamp
            predator_only_frame.mouse = False
            predator_only_frame.robot = True
            predator_only_frame.robot_data = predator_step.data
            predator_only_frame.robot_location = predator_step.location
            predator_only_frame.robot_rotation = predator_step.rotation
            episode_frames.append(predator_only_frame)
            last_time_stamp = predator_step.time_stamp
            last_frame = predator_step.frame
            predator_step = get_next_step(predator_steps, last_time_stamp)

        # for f in range(last_frame, prey_step.frame - 1):
        #     no_detection_frame = Complete_frame()
        #     no_detection_frame.frame = f
        #     no_detection_frame.mouse = False
        #     no_detection_frame.robot = False
        #     episode_frames.append(no_detection_frame)

        complete_frame = Complete_frame()
        complete_frame.frame = prey_step.frame
        complete_frame.time_stamp = prey_step.time_stamp
        complete_frame.mouse = True
        complete_frame.mouse_location = prey_step.location
        complete_frame.mouse_rotation = prey_step.rotation
        complete_frame.mouse_data = prey_step.data
        if predator_step \
                and predator_step.time_stamp == prey_step.time_stamp \
                and predator_step.frame == prey_step.frame :
            complete_frame.robot = True
            complete_frame.robot_location = predator_step.location
            complete_frame.robot_rotation = predator_step.rotation
            complete_frame.robot_data = predator_step.data
        else:
            complete_frame.robot = False
        if prey_step.data == "puff":
            complete_frame.puff = True
        episode_frames.append(complete_frame)
        last_frame = prey_step.frame
        last_time_stamp = prey_step.time_stamp
    return episode_frames

episode = Episode.load_from_file(sys.argv[1])

new_trajectories = Trajectories()
is_good = False
for i, s in enumerate(episode.trajectories):
    if s.time_stamp < 1 or s.agent_name == "prey":
        is_good = True
    if is_good:
        new_trajectories.append(s)

episode.trajectories = new_trajectories

frames = Frames_info().load_from_file(sys.argv[2])

episode_frames = get_episode_frames(episode.trajectories)
# print(len(episode_frames))
# print(len(episode_frames.where("mouse", False)))
# print(len(episode_frames.where("robot", False)))
#
# print("puffs: ", frames.where("puff", True).get("puff"))

new_episode_frames = Episode_frames()
last_i = -1
max_difference = 0
mismatches = 0

# from matplotlib import pyplot as plt
# plt.plot([1 if x.robot else 0 for x in episode_frames])
# # plt.plot([1 if x.mouse else 0 for x in episode_frames])
# # plt.plot([1 if x.puff else 0 for x in episode_frames])
# plt.show()

for i, f in enumerate(frames):
    new_complete_frame = Complete_frame()
    new_complete_frame.frame = f.frame_number
    if f.mouse or f.robot:
        last_i += 1
        if last_i >= len(episode_frames):
            continue
        episode_frame = episode_frames[last_i]
        d = f.mouse_location.dist(episode_frame.mouse_location)
        if d > max_difference:
            max_difference = d
        new_complete_frame.time_stamp = episode_frame.time_stamp
        new_complete_frame.mouse_data = episode_frame.mouse_data
        new_complete_frame.mouse_rotation = episode_frame.mouse_rotation
        new_complete_frame.mouse_location = episode_frame.mouse_location
        new_complete_frame.robot_data = episode_frame.robot_data
        new_complete_frame.robot_rotation = episode_frame.robot_rotation
        new_complete_frame.robot_location = episode_frame.robot_location
        if f.mouse != episode_frame.mouse or f.robot != episode_frame.robot:
            mismatches +=1
        new_complete_frame.robot = episode_frame.robot
        new_complete_frame.mouse = episode_frame.mouse
    new_episode_frames.append(new_complete_frame)

episode.trajectories = new_episode_frames.to_trajectories()
if max_difference > .01 or mismatches > 0:
    print("WARNING!!! file:", sys.argv[1], file=sys.stderr)
    print("max_difference:", max_difference, file=sys.stderr)
    print("mismatches:", mismatches, file=sys.stderr)
episode.save(sys.argv[3])
