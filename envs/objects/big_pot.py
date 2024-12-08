import numpy as np
from robosuite.models.objects import CompositeObject
from robosuite.utils.mjcf_utils import array_to_string, CustomMaterial, add_to_dict
import robosuite.utils.transform_utils as T
import numpy as np

class LargeKitchenPotObject(CompositeObject):
    def __init__(
            self,
            name,
            radius=0.1,  # Outer radius of the pot
            height=0.2,
            wall_thickness=0.005,
            density=1000,
            use_texture=True):

        self._name = name
        self.radius = radius
        self.height = height
        self.wall_thickness = wall_thickness
        self.use_texture = use_texture

        base_args = {
            "total_size": self.radius * 2,
            "name": self.name,
            "locations_relative_to_center": True,
            "obj_types": "all",
        }
        obj_args = {}

        geom_mat = "steel_scratched_mat"
        geom_frictions = (0.005, 0.005, 0.0001)
        solref = (0.02, 1.)
        
        # Calculate the side length of the octagon
        side_length = 2 * radius * np.sin(np.pi / 8)
        
        # Calculate the inner radius (distance from center to middle of each side)
        inner_radius = radius * np.cos(np.pi / 8)
        
        # Create octagonal sides
        for i in range(8):
            angle = np.pi / 4 * i
            x = inner_radius * np.cos(angle)
            y = inner_radius * np.sin(angle)
            
            add_to_dict(
                dic=obj_args,
                geom_types="box",
                geom_locations=(x, y, self.height/2),
                geom_quats=T.convert_quat(T.axisangle2quat(np.array([0, 0, angle])), to="wxyz"),
                geom_sizes=np.array([wall_thickness, side_length/2, self.height/2]),
                geom_names=f"octagon_side_{i}",
                geom_rgbas=None,
                geom_materials=geom_mat,
                geom_frictions=geom_frictions,
                solref=solref,
                density=density
            )

        # Create octagonal bottom
        bottom_radius = inner_radius - wall_thickness/2
        bottom_side_length = 2 * bottom_radius * np.sin(np.pi / 8)
        
        for i in range(8):
            angle = np.pi / 4 * i
            x = bottom_radius * np.cos(angle)
            y = bottom_radius * np.sin(angle)
            
            add_to_dict(
                dic=obj_args,
                geom_types="box",
                geom_locations=(x, y, wall_thickness/2),
                geom_quats=T.convert_quat(T.axisangle2quat(np.array([0, 0, angle])), to="wxyz"),
                geom_sizes=np.array([wall_thickness, bottom_side_length/2, wall_thickness/2]),
                geom_names=f"octagon_bottom_{i}",
                geom_rgbas=None,
                geom_materials=geom_mat,
                geom_frictions=geom_frictions,
                solref=solref,
                density=density
            )

        # Handles (unchanged)
        handle_radius = 0.015
        handle_width = self.radius * 0.8
        handle_length = self.wall_thickness * 4
        handle_height = self.height * 0.9

        

        obj_args.update(base_args)
        obj_args["joints"] = [{"type": "free", "damping":"0.0005"}]

        super().__init__(**obj_args)

        tex_attrib = {
            "type": "cube",
        }
        mat_attrib = {
            "texrepeat": "3 3",
            "specular": "0.4",
            "shininess": "0.1",
        }
        steel_scratched_material = CustomMaterial(
            texture="SteelScratched",
            tex_name="steel_scratched_tex",
            mat_name="steel_scratched_mat",
            tex_attrib=tex_attrib,
            mat_attrib=mat_attrib,
        )
 
        self.append_material(steel_scratched_material)

    @property
    def bottom_offset(self):
        return np.array([0, 0, -self.height/2])

    @property
    def top_offset(self):
        return np.array([0, 0, self.height/2])
        
    @property
    def horizontal_radius(self):
        return self.radius