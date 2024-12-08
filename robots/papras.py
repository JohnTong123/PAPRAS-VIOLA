import numpy as np

from robosuite.models.robots.robot_model import register_robot
import os
from robosuite.models.robots.manipulators.manipulator_model import ManipulatorModel
from robosuite.utils.mjcf_utils import xml_path_completion

def generate_x_range():
    return  np.random.random() * 0.6 - 0.1
def generate_y_range():
    return  np.random.random() * 0.4 - 0.2

# x_range = generate_x_range()
x_range = 0
y_range = 0 
# y_range = generate_y_range()


class PAPRAS(ManipulatorModel):
    """
    UR5e is a sleek and elegant new robot createfrom .robo_global import x_range,y_range,generate_x_range,generate_y_range
d by Universal Robots

    Args:
        idn (int or str): Number or some other unique identification string for this robot instance
    """

    def __init__(self, idn=0):

        xml_file = os.path.join(os.path.abspath(os.path.dirname(__file__)), "models/robot.xml")
        super().__init__(xml_path_completion(xml_file), idn=idn)

    # @property
    # def _eef_name(self):
    #     return "robot1/end_link"

    @property
    def default_mount(self):
        return "RethinkMount"

    @property
    def default_gripper(self):
        return "Robotiq85Gripper"

    @property
    def default_controller_config(self):
        return "default_ur5e"

    @property
    def init_qpos(self):
        return np.array([0, 0.0, 0.0, 0.0, 0.0, 3.14147456])

    @property
    def base_xpos_offset(self):
        """
        Base position offset with a 90-degree rotation applied along the Z-axis.
        """
        # Original offset before rotation
        x_range = 0
        y_range = 0
        z_range = 0

        # Base rotation in quaternion form (90 degrees about Z-axis)
        # quaternion_90_z = [0, 0, np.sin(np.pi / 4), np.cos(np.pi / 4)]

        # Offset position after rotating 90 degrees around Z-axis
        return {
            "bins": ((-0.5, -0.1, 0)),
            "empty": ((-0.6, 0, 0)),
            "table": lambda table_length: ((x_range - table_length / 2, y_range, z_range)),
        }


    @property
    def top_offset(self):
        
        return np.array((0, 0, 1.0))

    @property
    def _horizontal_radius(self):
        return 0.5

    @property
    def arm_type(self):
        return "single"


register_robot( PAPRAS)

from robosuite.robots import ROBOT_CLASS_MAPPING, SingleArm
ROBOT_CLASS_MAPPING.update({"PAPRAS": SingleArm})
ROBOT_CLASS_MAPPING["PAPRAS"] = SingleArm


