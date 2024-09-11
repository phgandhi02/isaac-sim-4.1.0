# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from omni.isaac.core.articulations.articulation import Articulation
from omni.isaac.core.articulations.articulation_view import ArticulationView
from omni.isaac.core.materials.deformable_material import DeformableMaterial
from omni.isaac.core.materials.deformable_material_view import DeformableMaterialView
from omni.isaac.core.materials.particle_material import ParticleMaterial
from omni.isaac.core.materials.particle_material_view import ParticleMaterialView
from omni.isaac.core.prims.base_sensor import BaseSensor
from omni.isaac.core.prims.geometry_prim import GeometryPrim
from omni.isaac.core.prims.geometry_prim_view import GeometryPrimView
from omni.isaac.core.prims.rigid_contact_view import RigidContactView
from omni.isaac.core.prims.rigid_prim import RigidPrim
from omni.isaac.core.prims.rigid_prim_view import RigidPrimView
from omni.isaac.core.prims.soft.cloth_prim import ClothPrim
from omni.isaac.core.prims.soft.cloth_prim_view import ClothPrimView
from omni.isaac.core.prims.soft.deformable_prim import DeformablePrim
from omni.isaac.core.prims.soft.deformable_prim_view import DeformablePrimView
from omni.isaac.core.prims.soft.particle_system import ParticleSystem
from omni.isaac.core.prims.soft.particle_system_view import ParticleSystemView
from omni.isaac.core.prims.xform_prim import XFormPrim
from omni.isaac.core.prims.xform_prim_view import XFormPrimView
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.robots.robot_view import RobotView


class SceneRegistry(object):
    """Class to keep track of the different types of objects added to the scene

    Example:

    .. code-block:: python

        >>> from omni.isaac.core.scenes import SceneRegistry
        >>>
        >>> scene_registry = SceneRegistry()
        >>> scene_registry
        <omni.isaac.core.scenes.scene_registry.SceneRegistry object at 0x...>
    """

    def __init__(self) -> None:
        self._rigid_objects = dict()
        self._geometry_objects = dict()
        self._articulated_systems = dict()
        self._robots = dict()
        self._xforms = dict()
        self._sensors = dict()
        self._xform_prim_views = dict()
        self._deformable_prims = dict()
        self._deformable_prim_views = dict()
        self._deformable_materials = dict()
        self._deformable_material_views = dict()
        self._cloth_prims = dict()
        self._cloth_prim_views = dict()
        self._particle_systems = dict()
        self._particle_system_views = dict()
        self._particle_materials = dict()
        self._particle_material_views = dict()
        self._geometry_prim_views = dict()
        self._rigid_prim_views = dict()
        self._rigid_contact_views = dict()
        self._articulated_views = dict()
        self._robot_views = dict()

        self._all_object_dicts = [
            self._rigid_objects,
            self._geometry_objects,
            self._articulated_systems,
            self._robots,
            self._xforms,
            self._sensors,
            self._xform_prim_views,
            self._deformable_prims,
            self._deformable_prim_views,
            self._deformable_materials,
            self._deformable_material_views,
            self._cloth_prims,
            self._cloth_prim_views,
            self._particle_systems,
            self._particle_system_views,
            self._particle_materials,
            self._particle_material_views,
            self._geometry_prim_views,
            self._rigid_prim_views,
            self._rigid_contact_views,
            self._articulated_views,
            self._robot_views,
        ]
        return

    @property
    def articulated_systems(self) -> dict:
        """Registered ``Articulation`` objects"""
        return self._articulated_systems

    @property
    def rigid_objects(self) -> dict:
        """Registered ``RigidPrim`` objects"""
        return self._rigid_objects

    @property
    def rigid_prim_views(self) -> dict:
        """Registered ``RigidPrimView`` objects"""
        return self._rigid_prim_views

    @property
    def rigid_contact_views(self) -> dict:
        """Registered ``RigidContactView`` objects"""
        return self._rigid_contact_views

    @property
    def geometry_prim_views(self) -> dict:
        """Registered ``GeometryPrimView`` objects"""
        return self._geometry_prim_views

    @property
    def articulated_views(self) -> dict:
        """Registered ``ArticulationView`` objects"""
        return self._articulated_views

    @property
    def robot_views(self) -> dict:
        """Registered ``RobotView`` objects"""
        return self._robot_views

    @property
    def robots(self) -> dict:
        """Registered ``Robot`` objects"""
        return self._robots

    @property
    def xforms(self) -> dict:
        """Registered ``XFormPrim`` objects"""
        return self._xforms

    @property
    def sensors(self) -> dict:
        """Registered ``BaseSensor`` (and derived) objects"""
        return self._sensors

    @property
    def xform_prim_views(self) -> dict:
        """Registered ``XFormPrimView`` objects"""
        return self._xform_prim_views

    @property
    def deformable_prims(self) -> dict:
        """Registered ``DeformablePrim`` objects"""
        return self._deformable_prims

    @property
    def deformable_prim_views(self) -> dict:
        """Registered ``DeformablePrimView`` objects"""
        return self._deformable_prim_views

    @property
    def deformable_materials(self) -> dict:
        """Registered ``DeformableMaterial`` objects"""
        return self._deformable_materials

    @property
    def deformable_material_views(self) -> dict:
        """Registered ``DeformableMaterialView`` objects"""
        return self._deformable_material_views

    @property
    def cloth_prims(self) -> dict:
        """Registered ``ClothPrim`` objects"""
        return self._cloth_prims

    @property
    def cloth_prim_views(self) -> dict:
        """Registered ``ClothPrimView`` objects"""
        return self._cloth_prim_views

    @property
    def particle_systems(self) -> dict:
        """Registered ``ParticleSystem`` objects"""
        return self._particle_systems

    @property
    def particle_system_views(self) -> dict:
        """Registered ``ParticleSystemView`` objects"""
        return self._particle_system_views

    @property
    def particle_materials(self) -> dict:
        """Registered ``ParticleMaterial`` objects"""
        return self._particle_materials

    @property
    def particle_material_views(self) -> dict:
        """Registered ``particle_material_view`` objects"""
        return self._particle_material_views

    # TODO: add if name exists check uniqueness
    def add_rigid_object(self, name: str, rigid_object: RigidPrim) -> None:
        """Register a ``RigidPrim`` (or subclass) object

        Args:
            name (str): object name
            rigid_object (RigidPrim): object
        """
        self._rigid_objects[name] = rigid_object
        return

    def add_rigid_prim_view(self, name: str, rigid_prim_view: RigidPrimView) -> None:
        """Register a ``RigidPrimView`` (or subclass) object

        Args:
            name (str): object name
            rigid_prim_view (RigidPrimView): object
        """
        self._rigid_prim_views[name] = rigid_prim_view
        return

    def add_rigid_contact_view(self, name: str, rigid_contact_view: RigidContactView) -> None:
        """Register a ``RigidContactView`` (or subclass) object

        Args:
            name (str): object name
            rigid_contact_view (RigidContactView): object
        """
        self._rigid_contact_views[name] = rigid_contact_view
        return

    def add_articulated_system(self, name: str, articulated_system: Articulation) -> None:
        """Register a ``Articulation`` (or subclass) object

        Args:
            name (str): object name
            articulated_system (Articulation): object
        """
        self._articulated_systems[name] = articulated_system
        return

    def add_articulated_view(self, name: str, articulated_view: ArticulationView) -> None:
        """Register a ``ArticulationView`` (or subclass) object

        Args:
            name (str): object name
            articulated_view (ArticulationView): object
        """
        self._articulated_views[name] = articulated_view
        return

    def add_geometry_object(self, name: str, geometry_object: GeometryPrim) -> None:
        """Register a ``GeometryPrim`` (or subclass) object

        Args:
            name (str): object name
            geometry_object (GeometryPrim): object
        """
        self._geometry_objects[name] = geometry_object
        return

    def add_geometry_prim_view(self, name: str, geometry_prim_view: GeometryPrimView) -> None:
        """Register a ``GeometryPrimView`` (or subclass) object

        Args:
            name (str): object name
            geometry_prim_view (GeometryPrim): object
        """
        self._geometry_prim_views[name] = geometry_prim_view
        return

    def add_robot(self, name: str, robot: Robot) -> None:
        """Register a ``Robot`` (or subclass) object

        Args:
            name (str): object name
            robot (Robot): object
        """
        self._robots[name] = robot
        return

    def add_robot_view(self, name: str, robot_view: RobotView) -> None:
        """Register a ``RobotView`` (or subclass) object

        Args:
            name (str): object name
            robot_view (RobotView): object
        """
        self._robot_views[name] = robot_view
        return

    def add_xform_view(self, name: str, xform_prim_view: XFormPrimView) -> None:
        """Register a ``XFormPrimView`` (or subclass) object

        Args:
            name (str): object name
            xform_prim_view (XFormPrimView): object
        """
        self._xform_prim_views[name] = xform_prim_view
        return

    def add_deformable(self, name: str, deformable: DeformablePrim) -> None:
        """Register a ``DeformablePrim`` (or subclass) object

        Args:
            name (str): object name
            deformable (DeformablePrim): object
        """
        self._deformable_prims[name] = deformable
        return

    def add_deformable_view(self, name: str, deformable_prim_view: DeformablePrimView) -> None:
        """Register a ``DeformablePrimView`` (or subclass) object

        Args:
            name (str): object name
            deformable_prim_view (DeformablePrimView): object
        """
        self._deformable_prim_views[name] = deformable_prim_view
        return

    def add_deformable_material(self, name: str, deformable_material: DeformableMaterial) -> None:
        """Register a ``DeformableMaterial`` (or subclass) object

        Args:
            name (str): object name
            deformable_material (DeformableMaterial): object
        """
        self._deformable_materials[name] = deformable_material
        return

    def add_deformable_material_view(self, name: str, deformable_material_view: DeformableMaterialView) -> None:
        """Register a ``DeformableMaterialView`` (or subclass) object

        Args:
            name (str): object name
            deformable_material_view (DeformableMaterialView): object
        """
        self._deformable_material_views[name] = deformable_material_view
        return

    def add_cloth(self, name: str, cloth: ClothPrim) -> None:
        """Register a ``ClothPrim`` (or subclass) object

        Args:
            name (str): object name
            cloth (ClothPrim): object
        """
        self._cloth_prims[name] = cloth
        return

    def add_cloth_view(self, name: str, cloth_prim_view: ClothPrimView) -> None:
        """Register a ``ClothPrimView`` (or subclass) object

        Args:
            name (str): object name
            cloth_prim_view (ClothPrimView): object
        """
        self._cloth_prim_views[name] = cloth_prim_view
        return

    def add_particle_system(self, name: str, particle_system: ParticleSystem) -> None:
        """Register a ``ParticleSystem`` (or subclass) object

        Args:
            name (str): object name
            particle_system (ParticleSystemView): object
        """
        self._particle_systems[name] = particle_system
        return

    def add_particle_system_view(self, name: str, particle_system_view: ParticleSystemView) -> None:
        """Register a ``ParticleSystemView`` (or subclass) object

        Args:
            name (str): object name
            particle_system_view (ParticleSystemView): object
        """
        self._particle_system_views[name] = particle_system_view
        return

    def add_particle_material(self, name: str, particle_material: ParticleMaterial) -> None:
        """Register a ``ParticleMaterial`` (or subclass) object

        Args:
            name (str): object name
            particle_material (ParticleMaterial): object
        """
        self._particle_materials[name] = particle_material
        return

    def add_particle_material_view(self, name: str, particle_material_view: ParticleMaterialView) -> None:
        """Register a ``ParticleMaterialView`` (or subclass) object

        Args:
            name (str): object name
            particle_material_view (ParticleMaterialView): object
        """
        self._particle_material_views[name] = particle_material_view
        return

    def add_xform(self, name: str, xform: XFormPrim) -> None:
        """Register a ``XFormPrim`` (or subclass) object

        Args:
            name (str): object name
            robot (Robot): object
        """
        self._xforms[name] = xform
        return

    def add_sensor(self, name: str, sensor: BaseSensor) -> None:
        """Register a ``BaseSensor`` (or subclass) object

        Args:
            name (str): object name
            sensor (BaseSensor): object
        """
        self._sensors[name] = sensor

        return

    def name_exists(self, name: str) -> bool:
        """Check if an object exists in the registry by its name

        Args:
            name (str): object name

        Returns:
            bool: whether the object is registered or not

        Example:

        .. code-block:: python

            >>> # given a registered ground plane named 'default_ground_plane'
            >>> scene_registry.name_exists("default_ground_plane")
            True
        """
        for object_dict in self._all_object_dicts:
            if name in object_dict:
                return True
        return False

    def remove_object(self, name: str) -> None:
        """Remove and object from the registry

        .. note::

            This method will only remove the object from the internal registry.
            The wrapped object will not be removed from the USD stage

        Args:
            name (str): object name

        Raises:
            Exception: If the name doesn't exist in the registry

        Example:

        .. code-block:: python

            >>> # given a registered ground plane named 'default_ground_plane'
            >>> scene_registry.remove_object("default_ground_plane")
        """
        for object_dict in self._all_object_dicts:
            if name in object_dict:
                del object_dict[name]
                return
        raise Exception("Cannot remove object {} from the scene since it doesn't exist".format(name))

    def get_object(self, name: str) -> XFormPrim:
        """Get a registered object by its name if exists otherwise None

        Args:
            name (str): object name

        Returns:
            XFormPrim: the object if it exists otherwise None

        Example:

        .. code-block:: python

            >>> # given a registered ground plane named 'default_ground_plane'
            >>> scene_registry.get_object("default_ground_plane")
            <omni.isaac.core.objects.ground_plane.GroundPlane object at 0x...>
        """
        for object_dict in self._all_object_dicts:
            if name in object_dict:
                return object_dict[name]
        return None
