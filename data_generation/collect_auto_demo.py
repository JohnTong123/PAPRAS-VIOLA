"""
Modified from robosuite example scripts.
A script to collect a batch of human demonstrations that can be used
to generate a learning curriculum (see `demo_learning_curriculum.py`).

The demonstrations can be played back using the `playback_demonstrations_from_pkl.py`
script.
"""
import os
import shutil
import time
import argparse
import datetime
import h5py
from glob import glob
import numpy as np
import json

import robosuite as suite
from robosuite import load_controller_config
from robosuite.wrappers import DataCollectionWrapper, VisualizationWrapper
from robosuite.utils.input_utils import input2action


import init_path

from envs import *
from pid import basicPID

import cv2
from robots.papras import PAPRAS

import math

import robosuite.utils.transform_utils as T
from robosuite.devices import *
from robosuite.models.robots import *
from robosuite.robots import *




# def input2actionmod(device, robot, active_arm="right", env_configuration=None):
    
    
#     state = device.get_controller_state()
#     # Note: Devices output rotation with x and z flipped to account for robots starting with gripper facing down
#     #       Also note that the outputted rotation is an absolute rotation, while outputted dpos is delta pos
#     #       Raw delta rotations from neutral user input is captured in raw_drotation (roll, pitch, yaw)
#     dpos, rotation, raw_drotation, grasp, reset = (
#         state["dpos"],
#         state["rotation"],
#         state["raw_drotation"],
#         state["grasp"],
#         state["reset"],
#     )

#     # If we're resetting, immediately return None
#     if reset:
#         return None, None

#     # Get controller reference
#     controller = robot.controller if not isinstance(robot, Bimanual) else robot.controller[active_arm]
#     gripper_dof = robot.gripper.dof if not isinstance(robot, Bimanual) else robot.gripper[active_arm].dof

#     # First process the raw drotation
#     drotation = raw_drotation[[1, 0, 2]]
#     print(controller.name)
#     if controller.name == "IK_POSE":
#         # If this is panda, want to swap x and y axis
#         if isinstance(robot.robot_model, Panda):
#             drotation = drotation[[1, 0, 2]]
#         else:
#             # Flip x
#             drotation[0] = -drotation[0]
#         # Scale rotation for teleoperation (tuned for IK)
#         drotation *= 10
#         dpos *= 5
#         # relative rotation of desired from current eef orientation
#         # map to quat
#         drotation = T.mat2quat(T.euler2mat(drotation))

#         # If we're using a non-forward facing configuration, need to adjust relative position / orientation
#         if env_configuration == "single-arm-opposed":
#             # Swap x and y for pos and flip x,y signs for ori
#             dpos = dpos[[1, 0, 2]]
#             drotation[0] = -drotation[0]
#             drotation[1] = -drotation[1]
#             if active_arm == "left":
#                 # x pos needs to be flipped
#                 dpos[0] = -dpos[0]
#             else:
#                 # y pos needs to be flipped
#                 dpos[1] = -dpos[1]

#         # Lastly, map to axis angle form
#         drotation = T.quat2axisangle(drotation)

#     elif controller.name == "OSC_POSE":
#         # Flip z
#         drotation[2] = -drotation[2]
#         # Scale rotation for teleoperation (tuned for OSC) -- gains tuned for each device
#         drotation = drotation * 1.5 if isinstance(device, Keyboard) else drotation * 50
#         dpos = dpos * 75 if isinstance(device, Keyboard) else dpos * 125
#     elif controller.name == "OSC_POSITION":
#         dpos = dpos * 75 if isinstance(device, Keyboard) else dpos * 125
#     else:
#         # No other controllers currently supported
#         print("Error: Unsupported controller specified -- Robot must have either an IK or OSC-based controller!")

#     # map 0 to -1 (open) and map 1 to 1 (closed)
#     grasp = 1 if grasp else -1

#     # Create action based on action space of individual robot
#     if controller.name == "OSC_POSITION":
#         action = np.concatenate([dpos, [grasp] * gripper_dof])
#     else:
#         action = np.concatenate([dpos, drotation, [grasp] * gripper_dof])

#     # Return the action and grasp
    # return action, grasp




def get_keyboard_target_pos(current):
    target_pos = current.copy()
    print("\nCurrent position: ", target_pos)
    print("Enter new target position (x, y, z, gripper) or type 'q' to quit:")

    
    
    user_input = input("Target position (x y z 1,0 for gripper): ").strip()
    if user_input.lower() == 'q':
        return None  # Signal to exit

    # Parse new position
    if len(user_input.split()) == 3:
        x,y,z =  map(float, user_input.split())
        # print(x,y,z, 'inputs')
        # g = 0.0
        g = 0
    else:
        x, y, z, g = map(float, user_input.split())
    target_pos = np.array([x, y, z, int(g)])
    print(target_pos,'targ')
    return target_pos
  




def collect_human_trajectory(env, device, arm, env_configuration, remove_directory=[], use_discrete_actions=False, num_discrete_actions=11):
    """
    Use the device (keyboard or SpaceNav 3D mouse) to collect a demonstration.
    The rollout trajectory is saved to files in npz format.
    Modify the DataCollectionWrapper wrapper to add new fields or change data formats.

    Args:
        env (MujocoEnv): environment to control
        device (Device): to receive controls from the device
        arms (str): which arm to control (eg bimanual) 'right' or 'left'
        env_configuration (str): specified environment configuration
    """

    env.reset()
    # ID = 2 always corresponds to agentview
    env.render()

    is_first = True

    task_completion_hold_count = -1  # counter to collect 10 timesteps after reaching goal
    device.start_control()

    # Loop until we get a reset from the input or the task completes
    saving = True
    count = 0

    # print(env._get_observations().keys())
    # print(env._get_observations()['robot0_eef_pos'])
    # print(env.robots[0].controller.keys())
    # print('ahhh')
    # print(env.model[58]
    # print(env.data.body_xpos[58])
    # print(env.get_obj())
    # quit()
    grasp = 1 if device.get_controller_state()["grasp"] else -1
    target_pos = env._get_observations()['robot0_eef_pos'].copy()
    target_pos = np.append(target_pos,grasp)
    current = target_pos.copy()
    # print(device.get_controller_state().keys())
    # print(target_pos)
    # print('')
    # print(env._get_observations()['robot0_gripper_qpos'])
    # prin

    pid = basicPID()

    ct = -1
    t = env.get_obj()
    spoon_pos = (t[1][0],t[1][1])
    pot_pos = (t[0][0],t[0][1])
    # print(env.objects[1].__dict__, "ahhh")
    print(t, 'ttt')
    # print(env._get_observations()['skillet_to_robot0_eef_pos'],"skill")
    # print(env._get_observations()['fake-spoon_to_robot0_eef_pos'],"spo")
    def generate_octagon_points(center, radius, n, z, gripper):
        x0, y0 = center
        points = []
        for i in range(8):
            theta = math.pi / 4 * i
            x = x0 + radius * math.cos(theta)
            y = y0 + radius * math.sin(theta)
            points.append(np.array([x, y,z,gripper]))
        return [i for i in points]*n
    stirring_points = generate_octagon_points(pot_pos, 0.05,3, 1.05 ,1)
    # print(len(stirring_points),"sz")
    target_seq = [np.array([0, 0 ,1.2, 0]), np.array([0, -0.08 ,1.2, 0]), np.array([0, -0.15 ,1.1, 0]), np.array([0, -0.18 ,1.1, 0]), 
                  np.array([spoon_pos[0], spoon_pos[1] ,1.02, 0]), np.array([spoon_pos[0], spoon_pos[1] ,1.02, 1]), np.array([spoon_pos[0], spoon_pos[1] ,1.02, 1]),np.array([spoon_pos[0], spoon_pos[1],1.02, 1]),np.array([spoon_pos[0], spoon_pos[1],1.02, 1]),
                  np.array([spoon_pos[0], spoon_pos[1] ,1.02, 1]), np.array([spoon_pos[0], spoon_pos[1] ,1.1, 1]),np.array([0, 0 ,1.2, 1]),np.array([0, 0.08 ,1.2, 1]),
                  np.array([0, 0.15 ,1.1, 1]), np.array([0, 0.18 ,1.1, 1]),np.array([pot_pos[0], pot_pos[1] ,1.1, 1]) ,np.array([pot_pos[0], pot_pos[1] ,1.05, 1])]
    target_seq = target_seq + stirring_points + [None]
# 0 -0.08 1.2
# 0 -0.15 1.1
# 0 -0.18 1.1
# 0 -0.2 1.05
# 0 -0.2 1.02 1]
    while True:
        count += 1
        # Set active robot
        current = env._get_observations()['robot0_eef_pos'].copy()
        grasp = target_pos[3]
        current = np.append(current, grasp)
        active_robot = env.robots[0] if env_configuration == "bimanual" else env.robots[arm == "left"]
        # print(env_configuration)
        # print(device)
        # print(arm)  
        # Get the newest action
        # action, grasp = input2action(
        #     device=device,
        #     robot=active_robot,
        #     active_arm=arm,
        #     env_configuration=env_configuration
        # )
        # print(action,grasp)
        # If action is none, then this a reset so we should break
        # if action is None:
            # print("Break")
            
            # saving = False
            # break
        # if use_discrete_actions:
            # action = get_discretized_actions(action, num_discrete_actions)
            # action = env.get_discrete_action()

        # Run environment step
        # print(action)
        # print('aaaaa')
        # print(current)
        # print(target_pos)
        if ((target_pos - current )**2).mean() < 0.00001:
            
            # target_pos = get_keyboard_target_pos(target_pos)
            ct+=1
            target_pos = target_seq[ct]
            pid.reset()
            # print(pid.accumulated_error)
        # action = 
        

        # print(current)
        # print(target_pos)
        if target_pos is None:
            print("Ending trajectory collection.")
            break  # Exit loop if user quits
        # action = np.clip(1e5*(target_pos - current),-3,3)
        action = pid.update(current,target_pos)
        print(action)
        action[3] = target_pos[3]
    
        # target_pos = new_target_pos
        # Generate action to move to target position
        # target_orientation = env._get_observations()['robot0_eef_pos']  # Keep current orientation
        # target_pose = {"position": target_pos, "orientation": target_orientation}
        # grasp = 1 if device.get_controller_state()["grasp"] else -1
        # action = np.clip(new_target_pos - target_pos)

        env.step(action)
        env.render()

        # Also break if we complete the task
        if task_completion_hold_count == 0:
            # saving = True
            # print('bruv')

            break
        

        # state machine to check for having a success for 10 consecutive timesteps
        if env._check_success():
            if task_completion_hold_count > 0:
                task_completion_hold_count -= 1  # latched state, decrement count
            else:
                task_completion_hold_count = 10  # reset count on first success timestep
        else:
            task_completion_hold_count = -1  # null the counter if there's no success

    print(count)
    # cleanup for end of data collection episodes
    if not saving:
        remove_directory.append(env.ep_directory.split('/')[-1])
    env.close()
    return saving
    # return True

def gather_demonstrations_as_hdf5(directory, out_dir, env_info, args, remove_directory=[]):
    """
    Gathers the demonstrations saved in @directory into a
    single hdf5 file.

    The strucure of the hdf5 file is as follows.

    data (group)
        date (attribute) - date of collection
        time (attribute) - time of collection
        repository_version (attribute) - repository version used during collection
        env (attribute) - environment name on which demos were collected

        demo1 (group) - every demonstration has a group
            model_file (attribute) - model xml string for demonstration
            states (dataset) - flattened mujoco states
            actions (dataset) - actions applied during demonstration

        demo2 (group)
        ...

    Args:
        directory (str): Path to the directory containing raw demonstrations.
        out_dir (str): Path to where to store the hdf5 file. 
        env_info (str): JSON-encoded string containing environment information,
            including controller and robot info
    """

    hdf5_path = os.path.join(out_dir, "demo.hdf5")
    print(hdf5_path)
    f = h5py.File(hdf5_path, "w")

    # store some metadata in the attributes of one group
    grp = f.create_group("data")

    num_eps = 0
    env_name = None  # will get populated at some point

    for ep_directory in os.listdir(directory):
        # print(ep_directory)
        if ep_directory in remove_directory:
            print("Skipping")
            continue
        state_paths = os.path.join(directory, ep_directory, "state_*.npz")
        states = []
        actions = []

        for state_file in sorted(glob(state_paths)):
            dic = np.load(state_file, allow_pickle=True)
            env_name = str(dic["env"])

            states.extend(dic["states"])
            for ai in dic["action_infos"]:
                actions.append(ai["actions"])
                
        if len(states) == 0:
            continue

        # Delete the first actions and the last state. This is because when the DataCollector wrapper
        # recorded the states and actions, the states were recorded AFTER playing that action.
        del states[-1]
        assert len(states) == len(actions)

        num_eps += 1
        ep_data_grp = grp.create_group("demo_{}".format(num_eps))

        # store model xml as an attribute
        xml_path = os.path.join(directory, ep_directory, "model.xml")
        with open(xml_path, "r") as f:
            xml_str = f.read()
        ep_data_grp.attrs["model_file"] = xml_str
        # ep_data_grp.attrs["pick_id"] = env.pick_id
        # ep_data_grp.attrs["place_id"] = env.place_id

        # write datasets for states and actions
        ep_data_grp.create_dataset("states", data=np.array(states))
        ep_data_grp.create_dataset("actions", data=np.array(actions))

    # write dataset attributes (metadata)
    now = datetime.datetime.now()
    grp.attrs["date"] = "{}-{}-{}".format(now.month, now.day, now.year)
    grp.attrs["time"] = "{}:{}:{}".format(now.hour, now.minute, now.second)
    grp.attrs["repository_version"] = suite.__version__
    grp.attrs["env"] = env_name
    grp.attrs["env_info"] = env_info
    grp.attrs["use_discrete_actions"] = args.use_discrete_actions
    grp.attrs["num_discrete_actions"] = args.num_discrete_actions

    f.close()


if __name__ == "__main__":
    # Arguments
    # print('yay')
    # print(TASK_MAPPING)
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--directory",
        type=str,
        default="demonstration_data",
    )
    parser.add_argument("--environment", type=str, default="tool-use")
    parser.add_argument("--robots", nargs="+", type=str, default="Panda", help="Which robot(s) to use in the env")
    parser.add_argument("--config", type=str, default="single-arm-opposed",
                        help="Specified environment configuration if necessary")
    parser.add_argument("--arm", type=str, default="right", help="Which arm to control (eg bimanual) 'right' or 'left'")
    parser.add_argument("--camera", type=str, default="agentview", help="Which camera to use for collecting demos")
    parser.add_argument("--controller", type=str, default="OSC_POSITION",
                        help="Choice of controller. Can be 'IK_POSE' or 'OSC_POSE'")
    parser.add_argument("--device", type=str, default="spacemouse")
    parser.add_argument("--pos-sensitivity", type=float, default=1.0, help="How much to scale position user inputs")
    parser.add_argument("--rot-sensitivity", type=float, default=1.0, help="How much to scale rotation user inputs")
    parser.add_argument("--num-demonstration", type=int, default=100, help="How much to scale rotation user inputs")
    parser.add_argument("--task-id", type=int, default=-1)
    
    parser.add_argument("--use-discrete-actions", action="store_true")
    parser.add_argument("--num-discrete-actions", type=int, default=11)
    
    
    args = parser.parse_args()
    
    assert(args.environment in TASK_MAPPING.keys())

    # Get controller config
    controller_config = load_controller_config(default_controller=args.controller)

    # Create argument configuration
    config = {
        "robots": args.robots,
        "controller_configs": controller_config,
    }

    # Check if we're using a multi-armed environment and use env_configuration argument if so
    if "TwoArm" in args.environment:
        config["env_configuration"] = args.config

    # Create environment

    # config["task_id"] = args.task_id

    env = TASK_MAPPING[args.environment](
        exp_name="normal",
        **config,
        has_renderer=True,
        has_offscreen_renderer=False,
        render_camera=args.camera,
        ignore_done=True,
        use_camera_obs=False,
        reward_shaping=True,
        control_freq=20,
    )

    # Wrap this with visualization wrapper
    env = VisualizationWrapper(env)

    # Grab reference to controller config and convert it to json-encoded string
    env_info = json.dumps(config)

    # wrap the environment with data collection wrapper
    tmp_directory = "demonstration_data/{}_tmp/{}".format(args.environment, str(time.time()).replace(".", "_"))
    env = DataCollectionWrapper(env, tmp_directory)

    # initialize device
    if args.device == "keyboard":
        from robosuite.devices import Keyboard

        device = Keyboard(pos_sensitivity=args.pos_sensitivity, rot_sensitivity=args.rot_sensitivity)
        env.viewer.add_keypress_callback(device.on_press)

        # env.viewer.add_keypress_callback("any", device.on_press)
        # env.viewer.add_keyup_callback("any", device.on_release)
        # env.viewer.add_keyrepeat_callback("any", device.on_press)
    elif args.device == "spacemouse":
        from robosuite.devices import SpaceMouse

        device = SpaceMouse(9583, 50734, pos_sensitivity=args.pos_sensitivity, rot_sensitivity=args.rot_sensitivity)
        # device = SpaceMouse(9583, 50770, pos_sensitivity=args.pos_sensitivity, rot_sensitivity=args.rot_sensitivity)
        
    else:
        raise Exception(
            "Invalid device choice: choose either 'keyboard' or 'spacemouse'."
        )

    # make a new timestamped directory
    t1, t2 = str(time.time()).split(".")
    if args.task_id != -1:
        new_dir = os.path.join(args.directory, "{}_task_{}_{}_{}".format(args.environment, args.task_id, t1, t2))
    else:
        new_dir = os.path.join(args.directory, "{}_{}_{}".format(args.environment, t1, t2))        
    os.makedirs(new_dir)

    # collect demonstrations

    remove_directory = []
    i = 0
    while i < args.num_demonstration:
        print(i)
        # print("hi")
        saving = collect_human_trajectory(env, device, args.arm, args.config, remove_directory, args.use_discrete_actions, num_discrete_actions=args.num_discrete_actions)
        # print(saving)
        if saving:
            gather_demonstrations_as_hdf5(tmp_directory, new_dir, env_info, args, remove_directory)
            i += 1
            print('yay')
