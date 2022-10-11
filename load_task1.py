import numpy as np
from PIL import Image
import numpy as np
import habitat_sim
from habitat_sim.utils.common import d3_40_colors_rgb
import cv2
import math

# This is the scene we are going to load.
# support a variety of mesh formats, such as .glb, .gltf, .obj, .ply
### put your scene path ###
test_scene = "replica_v1/apartment_0/habitat/mesh_semantic.ply"
sim_settings = {
    "scene": test_scene,  # Scene path
    "default_agent": 0,  # Index of the default agent
    "sensor_height": 1.5,  # Height of sensors in meters, relative to the agent
    "width": 512,  # Spatial resolution of the observations
    "height": 512,
    # sensor pitch (x rotation in rads), lin: front view is 0
    "sensor_pitch": 0,
}

BEV_sim_settings = {
    "scene": test_scene,  # Scene path
    "default_agent": 0,  # Index of the default agent
    "sensor_height": 2.3,  # Height of sensors in meters, relative to the agent
    "width": 512,  # Spatial resolution of the observations
    "height": 512,
    # sensor pitch (x rotation in rads), lin: BEV view is pi*-0.5
    "sensor_pitch": math.pi*-0.5,
}

# This function generates a config for the simulator.
# It contains two parts:
# one for the simulator backend
# one for the agent, where you can attach a bunch of sensors


def transform_rgb_bgr(image):
    return image[:, :, [2, 1, 0]]


def transform_depth(image):
    depth_img = (image / 10 * 255).astype(np.uint8)
    return depth_img


def transform_semantic(semantic_obs):
    semantic_img = Image.new(
        "P", (semantic_obs.shape[1], semantic_obs.shape[0]))
    semantic_img.putpalette(d3_40_colors_rgb.flatten())
    semantic_img.putdata((semantic_obs.flatten() % 40).astype(np.uint8))
    semantic_img = semantic_img.convert("RGB")
    semantic_img = cv2.cvtColor(np.asarray(semantic_img), cv2.COLOR_RGB2BGR)
    return semantic_img


def make_simple_cfg(settings):
    # simulator backend
    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.scene_id = settings["scene"]
    # agent
    agent_cfg = habitat_sim.agent.AgentConfiguration()

    # In the 1st example, we attach only one sensor,
    # a RGB visual sensor, to the agent
    rgb_sensor_spec = habitat_sim.CameraSensorSpec()
    rgb_sensor_spec.uuid = "color_sensor"
    rgb_sensor_spec.sensor_type = habitat_sim.SensorType.COLOR
    rgb_sensor_spec.resolution = [settings["height"], settings["width"]]
    rgb_sensor_spec.position = [0.0, settings["sensor_height"], 0.0]
    rgb_sensor_spec.orientation = [
        settings["sensor_pitch"],
        0.0,
        0.0,
    ]
    rgb_sensor_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE

    # depth snesor
    depth_sensor_spec = habitat_sim.CameraSensorSpec()
    depth_sensor_spec.uuid = "depth_sensor"
    depth_sensor_spec.sensor_type = habitat_sim.SensorType.DEPTH
    depth_sensor_spec.resolution = [settings["height"], settings["width"]]
    depth_sensor_spec.position = [0.0, settings["sensor_height"], 0.0]
    depth_sensor_spec.orientation = [
        settings["sensor_pitch"],
        0.0,
        0.0,
    ]
    depth_sensor_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE

    # semantic snesor
    semantic_sensor_spec = habitat_sim.CameraSensorSpec()
    semantic_sensor_spec.uuid = "semantic_sensor"
    semantic_sensor_spec.sensor_type = habitat_sim.SensorType.SEMANTIC
    semantic_sensor_spec.resolution = [settings["height"], settings["width"]]
    semantic_sensor_spec.position = [0.0, settings["sensor_height"], 0.0]
    semantic_sensor_spec.orientation = [
        settings["sensor_pitch"],
        0.0,
        0.0,
    ]
    semantic_sensor_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE

    agent_cfg.sensor_specifications = [
        rgb_sensor_spec, depth_sensor_spec, semantic_sensor_spec]

    return habitat_sim.Configuration(sim_cfg, [agent_cfg])


cfg = make_simple_cfg(sim_settings)
sim = habitat_sim.Simulator(cfg)

# by lin
BEV_cfg = make_simple_cfg(BEV_sim_settings)
BEV_sim = habitat_sim.Simulator(BEV_cfg)

# initialize an agent
agent = sim.initialize_agent(sim_settings["default_agent"])
BEV_agent = BEV_sim.initialize_agent(BEV_sim_settings["default_agent"])

# Set agent state
agent_state = habitat_sim.AgentState()
agent_state.position = np.array([0.0, 0.0, 0.0])  # agent in world space
agent.set_state(agent_state)

BEV_agent.set_state(agent_state)

# obtain the default, discrete actions that an agent can perform
# default action space contains 3 actions: move_forward, turn_left, and turn_right
action_names = list(
    cfg.agents[sim_settings["default_agent"]].action_space.keys())
print("Discrete action space: ", action_names)


FORWARD_KEY = "w"
LEFT_KEY = "a"
RIGHT_KEY = "d"
FINISH = "f"
SAVE_KEY = "s"  # by lin
print("#############################")
print("use keyboard to control the agent")
print(" w for go forward  ")
print(" a for turn left  ")
print(" d for trun right  ")
print(" a for save bev and front img in task1")
print(" o for omit all rgb, depth, GT data in task2")
print(" f for finish and quit the program")
print("#############################")


def navigateAndSee(action=""):
    if action in action_names:
        observations = sim.step(action)
        BEV_observations = BEV_sim.step(action)
        #print("action: ", action)

        '''
        show front view
        '''
        rgb_img = transform_rgb_bgr(observations["color_sensor"])
        depth_img = transform_depth(observations["depth_sensor"])
        semantic_img = transform_semantic(observations["semantic_sensor"])
        cv2.imshow("RGB", rgb_img)
        cv2.imshow("depth", depth_img)
        cv2.imshow("semantic", semantic_img)
        agent_state = agent.get_state()
        sensor_state = agent_state.sensor_states['color_sensor']
        # print("camera pose: x y z rw rx ry rz")
        # print(sensor_state.position[0],sensor_state.position[1],sensor_state.position[2],  sensor_state.rotation.w, sensor_state.rotation.x, sensor_state.rotation.y, sensor_state.rotation.z)

        '''
        show BEV view
        '''
        BEV_rgb_img = transform_rgb_bgr(BEV_observations["color_sensor"])
        BEV_depth_img = transform_depth(BEV_observations["depth_sensor"])
        BEV_semantic_img = transform_semantic(
            BEV_observations["semantic_sensor"])
        cv2.imshow("BEV_RGB", BEV_rgb_img)
        cv2.imshow("BEV_depth", BEV_depth_img)
        cv2.imshow("BEV_semantic", BEV_semantic_img)
        BEV_agent_state = BEV_agent.get_state()
        BEV_sensor_state = BEV_agent_state.sensor_states['color_sensor']
        # print("BEV camera pose: x y z rw rx ry rz")
        # print(BEV_sensor_state.position[0],BEV_sensor_state.position[1],BEV_sensor_state.position[2],  BEV_sensor_state.rotation.w, BEV_sensor_state.rotation.x, BEV_sensor_state.rotation.y, BEV_sensor_state.rotation.z)
        return rgb_img, depth_img, semantic_img, BEV_rgb_img, BEV_depth_img, BEV_semantic_img


action = "move_forward"
rgb_IMG, depth_IMG, semantic_IMG, BEV_rgb_IMG, BEV_depth_IMG, BEV_semantic_IMG = navigateAndSee(
    action)

while True:
    keystroke = cv2.waitKey(0)
    if keystroke == ord(FORWARD_KEY):
        action = "move_forward"
        rgb_IMG, depth_IMG, semantic_IMG, BEV_rgb_IMG, BEV_depth_IMG, BEV_semantic_IMG = navigateAndSee(
            action)
        print("action: FORWARD")
    elif keystroke == ord(LEFT_KEY):
        action = "turn_left"
        rgb_IMG, depth_IMG, semantic_IMG, BEV_rgb_IMG, BEV_depth_IMG, BEV_semantic_IMG = navigateAndSee(
            action)
        print("action: LEFT")
    elif keystroke == ord(RIGHT_KEY):
        action = "turn_right"
        rgb_IMG, depth_IMG, semantic_IMG, BEV_rgb_IMG, BEV_depth_IMG, BEV_semantic_IMG = navigateAndSee(
            action)
        print("action: RIGHT")
    elif keystroke == ord(SAVE_KEY):
        cv2.imwrite('task1_data/front_view_path.png', rgb_IMG)
        cv2.imwrite('task1_data/top_view_path.png', BEV_rgb_IMG)
        print("action: SAVE IMG")
    elif keystroke == ord(FINISH):
        print("action: FINISH")
        break
    else:
        print("INVALID KEY")
        continue
