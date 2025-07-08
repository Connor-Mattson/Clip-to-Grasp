"""
A wrapper for the basic methods of the pybullet simulation.
"""

import pybullet as p
import pybullet_data
from src.sim.model_obj import ModelObj

class BulletSim:
    def __init__(self):
        self.client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # load URDFs
        self.plane_id = p.loadURDF("plane.urdf")
        self.objects = []
        self.object_ids = []

    def extend_search_path(self, path):
        p.setAdditionalSearchPath(path)

    def set_gravity(self, grav_x, grav_y, grav_z):
        p.setGravity(grav_x, grav_y, grav_z)

    def add_object(self, obj: ModelObj):
        new_obj_id = p.loadURDF(
            obj.path, 
            obj.position, 
            obj.orientation,
            globalScaling=obj.scale
        )
        self.objects.append(obj)
        self.object_ids.append(new_obj_id)
        obj.oid = new_obj_id
        return new_obj_id

    def get_object_aabb(self, obj_id):
        aabb_min, aabb_max = p.getAABB(obj_id)
        return aabb_min, aabb_max

    def register_objects(self, objs: list[ModelObj]):
        for obj in objs:
            self.add_object(obj)

    def close(self):
        p.disconnect()

    def enable_auxiliary_camera(self):
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 1)

