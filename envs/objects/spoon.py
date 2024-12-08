import numpy as np
from robosuite.models.objects import CompositeObject
from robosuite.utils.mjcf_utils import array_to_string, CustomMaterial, add_to_dict
import robosuite.utils.transform_utils as T

class SpoonObject(CompositeObject):
    def __init__(
            self,
            name,
            spoon_size=(0.30, 0.06, 0.025),  # Length, width, thickness
            density=700,  # Density of wood
            use_texture=True):

        self._name = name
        self.total_length, self.max_width, self.thickness = spoon_size
        self.use_texture = use_texture

        base_args = {
            "total_size": self.total_length / 2.0,
            "name": self.name,
            "locations_relative_to_center": True,
            "obj_types": "all",
        }
        obj_args = {}

        geom_mat = "wood_mat"
        geom_frictions = (0.005, 0.005, 0.0001)
        solref = (0.02, 1.)

        # Define spoon parts
        handle_length = self.total_length * 0.6
        handle_width = self.max_width * 0.4
        neck_length = self.total_length * 0.25
        neck_width = self.max_width * 0.4
        bowl_length = self.total_length * 0.2
        bowl_width = self.max_width
        bowl_depth = self.thickness * 1.5

        # Handle
        add_to_dict(
            dic=obj_args,
            geom_types="box",
            geom_locations=(-self.total_length/2 + handle_length/2, 0, 0),
            geom_quats=T.convert_quat(T.axisangle2quat(np.array([0, 0, 0])), to="wxyz"),
            geom_sizes=np.array([handle_length/2, handle_width/2, self.thickness/2]),
            geom_names="handle",
            geom_rgbas=None,
            geom_materials=geom_mat,
            geom_frictions=geom_frictions,
            solref=solref,
            density=density
        )

        # Neck (connection between handle and bowl)
        add_to_dict(
            dic=obj_args,
            geom_types="box",
            geom_locations=(-self.total_length/2 + handle_length + neck_length/2, 0, 0),
            geom_quats=T.convert_quat(T.axisangle2quat(np.array([0, 0, 0])), to="wxyz"),
            geom_sizes=np.array([neck_length/2, neck_width/2, self.thickness/2]),
            geom_names="neck",
            geom_rgbas=None,
            geom_materials=geom_mat,
            geom_frictions=geom_frictions,
            solref=solref,
            density=density
        )

        # Bowl
        add_to_dict(
            dic=obj_args,
            geom_types="ellipsoid",
            geom_locations=(self.total_length/2 - bowl_length/2, 0, bowl_depth/4),
            geom_quats=T.convert_quat(T.axisangle2quat(np.array([np.pi/12, 0, 0])), to="wxyz"),
            geom_sizes=np.array([bowl_length/2, bowl_width/2, bowl_depth/2]),
            geom_names="bowl",
            geom_rgbas=None,
            geom_materials=geom_mat,
            geom_frictions=geom_frictions,
            solref=solref,
            density=density
        )

        # Sites
        site_attrs = []
        for site_name, site_pos in [
            ("bottom", [0, 0, -self.thickness]),
            ("top", [0, 0, self.thickness]),
            ("horizontal", [self.total_length / 2, 0, 0])
        ]:
            site = self.get_site_attrib_template()
            site.update({
                "name": site_name,
                "pos": array_to_string(np.array(site_pos)),
                "size": "0.005",
                "rgba": "0 0 0 0"
            })
            site_attrs.append(site)

        obj_args.update(base_args)
        obj_args["sites"] = site_attrs
        obj_args["joints"] = [{"type": "free", "damping":"0.0005"}]

        super().__init__(**obj_args)

        # Add wood texture
        tex_attrib = {
            "type": "cube",
        }
        mat_attrib = {
            "texrepeat": "3 3",
            "specular": "0.4",
            "shininess": "0.1",
        }
        wood_material = CustomMaterial(
            texture="WoodLight",
            tex_name="wood_light_tex",
            mat_name="wood_mat",
            tex_attrib=tex_attrib,
            mat_attrib=mat_attrib,
        )
        self.append_material(wood_material)

    @property
    def bottom_offset(self):
        return np.array([0, 0, -self.thickness/2])

    @property
    def top_offset(self):
        return np.array([0, 0, self.thickness/2])
        
    @property
    def horizontal_radius(self):
        return self.total_length / 2