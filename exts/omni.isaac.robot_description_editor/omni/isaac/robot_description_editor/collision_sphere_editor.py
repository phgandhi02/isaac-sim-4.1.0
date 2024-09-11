# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from collections import OrderedDict

import carb
import lula
import numpy as np
import yaml
from omni.isaac.core.materials import PreviewSurface
from omni.isaac.core.objects.sphere import VisualSphere
from omni.isaac.core.utils.prims import delete_prim, is_prim_path_valid
from omni.isaac.core.utils.string import find_unique_string_name
from pxr import Sdf


class CollisionSphereEditor:
    def __init__(self):
        self.path_2_spheres = {}
        self.path_2_sphere_serial_copy = {}

        self._operations = []

        self._redo = []

        self.filter = ""
        self.filter_in_sphere_color = np.array([37, 207, 188]) / 255  # All spheres that match filter
        self.filter_out_sphere_color = np.array([207.0, 184.0, 37.0]) / 255  # All Spheres that don't match filter

        self._preview_color = np.array([1.0, 0, 0])
        self._preview_spheres = []

        self._sphere_path_generators = {}

        self._lula_path = "/World/LulaRobotDescriptionEditor"

        self._filter_in_surface_prim_path = self._lula_path + "/selected_link_sphere_material"
        self._filter_out_surface_prim_path = self._lula_path + "/base_sphere_material"
        self._filter_in_surface = None
        self._filter_out_surface = None

        self._preview_sphere_surface_prim_path = self._lula_path + "/preview_sphere_material"
        self._preview_sphere_surface = None

    def _ensure_preview_sphere_visual_material(self):
        if not is_prim_path_valid(self._preview_sphere_surface_prim_path):
            self._preview_sphere_surface = PreviewSurface(
                self._preview_sphere_surface_prim_path, color=np.array([1.0, 0, 0])
            )
        elif self._preview_sphere_surface is None or not self._preview_sphere_surface.prim.IsValid():
            delete_prim(self._preview_sphere_surface_prim_path)

            self._preview_sphere_surface = PreviewSurface(
                self._preview_sphere_surface_prim_path, color=np.array([1.0, 0, 0])
            )

    def _ensure_collision_sphere_visual_material(self):
        """Make sure that the visual materials for sphere visualization have been created and are valid"""
        if not is_prim_path_valid(self._filter_in_surface_prim_path):
            self._filter_in_surface = PreviewSurface(
                prim_path=self._filter_in_surface_prim_path, color=self.filter_in_sphere_color
            )
        elif self._filter_in_surface is None or not self._filter_in_surface.prim.IsValid():
            delete_prim(self._filter_in_surface_prim_path)
            self._filter_in_surface = PreviewSurface(
                prim_path=self._filter_in_surface_prim_path, color=self.filter_in_sphere_color
            )

        if not is_prim_path_valid(self._filter_out_surface_prim_path):
            self._filter_out_surface = PreviewSurface(
                prim_path=self._filter_out_surface_prim_path, color=self.filter_out_sphere_color
            )
        elif self._filter_out_surface is None or not self._filter_out_surface.prim.IsValid():
            delete_prim(self._filter_out_surface_prim_path)
            self._filter_out_surface = PreviewSurface(
                prim_path=self._filter_out_surface_prim_path, color=self.filter_out_sphere_color
            )

    @staticmethod
    def _path_generator(path: str):
        """Get a generator that incrementally adds integers to `path` forever"""
        count = 1
        while True:
            yield f"{path}_{count}"
            count += 1

    def _get_unused_collision_sphere_path(self, link_path: str):
        sphere_base_path = self._get_collision_sphere_base_path(link_path)

        if sphere_base_path not in self._sphere_path_generators:
            self._sphere_path_generators[sphere_base_path] = self._path_generator(sphere_base_path)

        sphere_path_generator = self._sphere_path_generators[sphere_base_path]
        sphere_path = next(sphere_path_generator)
        while is_prim_path_valid(sphere_path):
            sphere_path = next(sphere_path_generator)
        return sphere_path

    def clear_spheres(self, store_op=True):
        self._sphere_path_generators = {}
        sphere_paths = list(self.path_2_spheres.keys())
        if len(sphere_paths) == 0:
            return

        if store_op:
            self.copy_all_sphere_data()
            deleted_spheres = ["DEL"]
            for sphere_path in sphere_paths:
                deleted_spheres.append(self.path_2_sphere_serial_copy[sphere_path])
            self._operations.append(deleted_spheres)

        for sphere_path in sphere_paths:
            self.delete_sphere(sphere_path)

    def clear_link_spheres(self, link_path, store_op=True):
        if self._get_collision_sphere_base_path(link_path) in self._sphere_path_generators:
            del self._sphere_path_generators[self._get_collision_sphere_base_path(link_path)]
        path_len = len(link_path)

        to_delete = []
        if store_op:
            self.copy_all_sphere_data()
        deleted_spheres = ["DEL"]
        for p in self.path_2_spheres.keys():
            if is_prim_path_valid(p) and p[:path_len] == link_path:
                deleted_spheres.append(self.path_2_sphere_serial_copy[p])
                to_delete.append(p)

        if store_op:
            self._operations.append(deleted_spheres)
        for s in to_delete:
            self.delete_sphere(s)

    def delete_sphere(self, sphere_path):
        if is_prim_path_valid(sphere_path):
            delete_prim(sphere_path)

        if sphere_path in self.path_2_spheres:
            del self.path_2_spheres[sphere_path]

    def set_sphere_colors(self, filter, color_in=None, color_out=None):
        self._ensure_collision_sphere_visual_material()

        if color_in is not None:
            self.filter_in_sphere_color = color_in
            self._filter_in_surface.set_color(color_in)

        if color_out is not None:
            self.filter_out_sphere_color = color_out
            self._filter_out_surface.set_color(color_out)
        self.filter = filter

        with Sdf.ChangeBlock():
            for sphere_path in self.path_2_spheres.keys():
                self.set_sphere_color(sphere_path, False)

    def set_sphere_color(self, sphere_path, ensure_visual_material=True):
        if ensure_visual_material:
            self._ensure_collision_sphere_visual_material()
        if not is_prim_path_valid(sphere_path):
            return
        sphere = self.path_2_spheres[sphere_path]
        if sphere_path[: len(self.filter)] == self.filter:
            sphere.apply_visual_material(self._filter_in_surface)
        else:
            sphere.apply_visual_material(self._filter_out_surface)

    def copy_all_sphere_data(self):
        self._ensure_collision_sphere_visual_material()
        sphere_paths = list(self.path_2_spheres.keys())
        deleted_spheres = ["DEL"]
        for sphere_path in sphere_paths:
            if is_prim_path_valid(sphere_path):
                sphere = self.path_2_spheres[sphere_path]
                color = sphere.get_applied_visual_material().get_color()
                self.path_2_sphere_serial_copy[sphere_path] = {
                    "sphere_path": sphere_path,
                    "center": sphere.get_local_pose()[0],
                    "radius": sphere.get_radius(),
                    "color": color,
                }
            else:
                if sphere_path in self.path_2_sphere_serial_copy:
                    deleted_spheres.append(self.path_2_sphere_serial_copy[sphere_path])
                self.delete_sphere(sphere_path)
        if len(deleted_spheres) > 1:
            self._operations.append(deleted_spheres)

    def undo(self):
        if len(self._operations) == 0:
            return

        last_op = self._operations.pop()
        op_type = last_op[0]
        op = last_op[1:]

        if op_type == "ADD":
            redo = ["ADD"]
            for sphere_path in op:
                if is_prim_path_valid(sphere_path):
                    sphere = self.path_2_spheres[sphere_path]
                    redo.append(
                        {
                            "sphere_path": sphere_path,
                            "center": sphere.get_local_pose()[0],
                            "radius": sphere.get_radius(),
                        }
                    )
                self.delete_sphere(sphere_path)
            self._redo.append(redo)

        elif op_type == "DEL":
            redo = ["DEL"]
            for d in op:
                sphere = VisualSphere(
                    d["sphere_path"],
                    translation=d["center"],
                    radius=d["radius"],
                    visual_material=self._filter_in_surface,
                )
                self.path_2_spheres[d["sphere_path"]] = sphere
                self.set_sphere_color(d["sphere_path"])
                redo.append(d)
            self._redo.append(redo)

        elif op_type == "SCALE":
            redo = ["SCALE"]
            for d in op:
                path = d["sphere_path"]
                rad = d["radius"]
                factor = d["factor"]
                if path in self.path_2_spheres and is_prim_path_valid(path):
                    sphere = self.path_2_spheres[path]
                    sphere.set_radius(rad)
                    redo.append({"sphere_path": path, "radius": factor * rad})
            self._redo.append(redo)

    def redo(self):
        if len(self._redo) == 0:
            return

        last_redo = self._redo.pop()
        op_type = last_redo[0]
        op = last_redo[1:]

        if op_type == "ADD":
            added_spheres = ["ADD"]
            for d in op:
                sphere = VisualSphere(
                    d["sphere_path"],
                    translation=d["center"],
                    radius=d["radius"],
                    visual_material=self._filter_in_surface,
                )
                self.path_2_spheres[d["sphere_path"]] = sphere
                self.set_sphere_color(d["sphere_path"])
                added_spheres.append(d["sphere_path"])
            self._operations.append(added_spheres)

        elif op_type == "DEL":
            deleted_spheres = ["DEL"]
            for d in op:
                self.delete_sphere(d["sphere_path"])
                deleted_spheres.append(d)
            self._operations.append(deleted_spheres)

        elif op_type == "SCALE":
            for d in op:
                path = d["sphere_path"]
                rad = d["radius"]
                if path in self.path_2_spheres and is_prim_path_valid(path):
                    sphere = self.path_2_spheres[path]
                    sphere.set_radius(rad)

    def generate_spheres(self, link_path, points, face_inds, vert_cts, num_spheres, radius_offset, is_preview):
        if not is_preview and self._preview_spheres:
            # If preview spheres exist, change them to permanent spheres
            added_sphere_paths = ["ADD"]
            for preview_sphere in self._preview_spheres:
                if not preview_sphere.prim.IsValid():
                    continue
                sphere_path = self.add_sphere(
                    link_path, preview_sphere.get_local_pose()[0], preview_sphere.get_radius(), store_op=False
                )
                added_sphere_paths.append(sphere_path)
            self._operations.append(added_sphere_paths)
            self.clear_preview()
            return

        unique_vert_cts = np.unique(vert_cts)
        if len(unique_vert_cts) != 1 or unique_vert_cts[0] != 3:
            self.clear_preview()
            carb.log_error(
                "Cannot generate collsision spheres for mesh because the specified mesh is not composed of triangles."
            )
            return

        num_points = face_inds.shape[0]

        generator = lula.create_collision_sphere_generator(points, face_inds.reshape((num_points // 3, 3)))
        result = generator.generate_spheres(num_spheres, radius_offset)
        if is_preview:
            self.clear_preview()
            self._ensure_preview_sphere_visual_material()
            preview_sphere_path_generator = self._path_generator(self._get_collision_sphere_preview_path(link_path))
            for lula_sphere in result:
                sphere_path = next(preview_sphere_path_generator)
                self._preview_spheres.append(
                    VisualSphere(
                        sphere_path,
                        color=self._preview_color,
                        translation=lula_sphere.center,
                        radius=lula_sphere.radius,
                        visual_material=self._preview_sphere_surface,
                    )
                )
        else:
            added_sphere_paths = ["ADD"]
            for lula_sphere in result:
                sphere_path = self.add_sphere(link_path, lula_sphere.center, lula_sphere.radius, store_op=False)
                added_sphere_paths.append(sphere_path)
            self._operations.append(added_sphere_paths)
            self.clear_preview()

    def clear_preview(self):
        self._preview_path_generator = {}

        for sphere in self._preview_spheres:
            self.delete_sphere(sphere.prim_path)
        self._preview_spheres = []

    def add_sphere(self, link_path, center, radius, store_op=True):
        if not is_prim_path_valid(link_path):
            carb.log_warn("Attempted to add sphere nested under non-existent path")

        if link_path[-1] == "/":
            link_path = link_path[:-1]

        self._redo = []
        sphere_path = self._get_unused_collision_sphere_path(link_path)

        self._ensure_collision_sphere_visual_material()
        if sphere_path[: len(self.filter)] == self.filter:
            visual_material = self._filter_in_surface
        else:
            visual_material = self._filter_out_surface

        sphere = VisualSphere(sphere_path, translation=center, radius=radius, visual_material=visual_material)

        self.path_2_spheres[sphere.prim_path] = sphere

        if store_op:
            self._operations.append(["ADD", sphere.prim_path])

        return sphere_path

    def _get_sphere_list_from_xrdf_geometries(self, parsed_file, geometry_group_name) -> dict:
        spheres = {}
        if "geometry" not in parsed_file:
            carb.log_warn("No geometry groups specified under 'geometry' in XRDF file.  No spheres will be imported.")
            return spheres
        elif geometry_group_name not in parsed_file["geometry"]:
            carb.log_warn("Collision geometry group could not be found under 'geometry'.  No spheres will be imported.")
            return spheres

        geometry_groups = parsed_file["geometry"]
        imported_group = geometry_groups[geometry_group_name]

        if "spheres" in imported_group:
            spheres = imported_group["spheres"]

        from collections import deque

        handled_groups = set([geometry_group_name])
        clones = deque()
        if "clone" in imported_group:
            for clone_group_name in imported_group["clone"]:
                clones.append(clone_group_name)

        while len(clones) > 0:
            clone_group_name = clones.popleft()
            if clone_group_name in handled_groups:
                continue
            handled_groups.add(clone_group_name)
            if clone_group_name in geometry_groups:
                clone_group = geometry_groups[clone_group_name]
            else:
                continue
            if "spheres" in clone_group:
                for key in clone_group["spheres"]:
                    if key in spheres:
                        spheres[key].extend(clone_group["spheres"][key])
                    else:
                        spheres[key] = clone_group["spheres"][key]
            if "clone" in clone_group:
                clones.extend(clone_group["clone"])

        return spheres

    def load_xrdf_spheres(self, robot_prim_path, parsed_file: dict):
        self.clear_spheres(store_op=False)

        self._redo = []
        self._operations = []

        if "collision" not in parsed_file:
            carb.log_warn("No collision group specified in XRDF file. No spheres will be imported")
            return
        elif "geometry" not in parsed_file["collision"]:
            carb.log_warn("No geometry group specified under 'collision' in XRDF file. No spheres will be imported")
            return

        geometry_group_name = parsed_file["collision"]["geometry"]

        if (
            "self_collision" in parsed_file
            and "geometry" in parsed_file["self_collision"]
            and parsed_file["self_collision"]["geometry"]
        ) != geometry_group_name:
            carb.log_warn(
                "Specifying a 'self_collision' geometry group that is not the same as "
                + "the 'collision' geometry group is not supported by this importer. "
                + "The 'self_collision' group will be ignored."
            )

        buffer_distances = parsed_file["collision"].get("buffer_distance", {})

        sphere_dict = self._get_sphere_list_from_xrdf_geometries(parsed_file, geometry_group_name)
        if len(sphere_dict.keys()) == 0:
            return

        added_sphere_paths = ["ADD"]
        for key, val in sphere_dict.items():
            link_path = robot_prim_path + "/" + key
            if is_prim_path_valid(link_path):
                for sphere in val:
                    center = np.array(sphere["center"])
                    radius = sphere["radius"]
                    sphere_path = self.add_sphere(link_path, center, radius, store_op=False)
                    added_sphere_paths.append(sphere_path)
            else:
                carb.log_warn("Could not place sphere from xrdf at path: {}".format(link_path))

        self._operations.append(added_sphere_paths)

        for k, v in buffer_distances.items():
            link_path = robot_prim_path + "/" + k
            for p in self.path_2_spheres.keys():
                if is_prim_path_valid(p) and p[: len(link_path)] == link_path:
                    sphere = self.path_2_spheres[p]
                    rad = sphere.get_radius()
                    sphere.set_radius(rad + v)

    def load_spheres(self, robot_prim_path, robot_description_file_path):
        self.clear_spheres(store_op=False)

        self._redo = []
        self._operations = []

        with open(robot_description_file_path, "r") as stream:
            try:
                parsed_file = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        sphere_list = parsed_file["collision_spheres"]

        if sphere_list is None:
            return

        robot_path = robot_prim_path

        added_sphere_paths = ["ADD"]

        for sphere_dict in sphere_list:
            for key, val in sphere_dict.items():
                link_path = robot_path + "/" + key
                if is_prim_path_valid(link_path):
                    for sphere in val:
                        center = np.array(sphere["center"])
                        radius = sphere["radius"]
                        sphere_path = self.add_sphere(link_path, center, radius, store_op=False)
                        added_sphere_paths.append(sphere_path)
                else:
                    carb.log_warn("Could not place sphere from robot description at path: {}".format(link_path))

        self._operations.append(added_sphere_paths)

    def interpolate_spheres(self, path1, path2, num_spheres):
        if not is_prim_path_valid(path1):
            carb.log_warn("{} is not a valid Prim path to a sphere".format(path1))
            return
        elif not is_prim_path_valid(path2):
            carb.log_warn("{} is not a valid Prim path to a sphere".format(path2))
            return

        link_path = self._get_link_path(path1)
        if self._get_link_path(path2) != link_path:
            carb.log_warn(
                "Prim paths {} and {} are not nested under the same link.  They cannot be interpolated.".format(
                    path1, path2
                )
            )

        epsilon = 1e-12

        sphere_1 = self.path_2_spheres[path1]
        sphere_2 = self.path_2_spheres[path2]

        rad_1 = max(sphere_1.get_radius(), epsilon)
        rad_2 = max(sphere_2.get_radius(), epsilon)

        t1 = sphere_1.get_local_pose()[0]
        t2 = sphere_2.get_local_pose()[0]

        d = t2 - t1

        # Assign radii for interpolated spheres according to a geometric sequence.
        rads = np.geomspace(rad_1, rad_2, num=num_spheres + 2)

        # Position spheres so that they're tangent to an enclosing cone.
        #
        # When the two endpoint spheres have the same radius, the interpolated spheres
        # will be evenly spaced.  The general expression below gives this result when
        # the equal-radius limit is taken analytically, but for the purpose of numerical
        # evaluation we must handle this case separately to avoid division by zero.
        if abs((rad_1 - rad_2) / (rad_1 + rad_2)) < epsilon:
            relative_offsets = np.linspace(0.0, 1.0, num=num_spheres + 2)
        else:
            relative_offsets = (rads - rad_1) / (rad_2 - rad_1)

        added_sphere_paths = ["ADD"]
        for i in range(1, num_spheres + 1):
            sphere_path = self.add_sphere(link_path, t1 + relative_offsets[i] * d, rads[i], store_op=False)
            added_sphere_paths.append(sphere_path)
        self._operations.append(added_sphere_paths)

    def scale_spheres(self, path, factor):
        scaled_spheres = ["SCALE"]
        path_len = len(path)

        for p in self.path_2_spheres.keys():
            if is_prim_path_valid(p) and p[:path_len] == path:
                sphere = self.path_2_spheres[p]
                rad = sphere.get_radius()
                sphere.set_radius(factor * rad)
                scaled_spheres.append({"sphere_path": p, "radius": rad, "factor": factor})
        self._operations.append(scaled_spheres)

    def get_sphere_names_by_link(self, link_path):
        sphere_names = []
        for sphere_path in self.path_2_spheres.keys():
            sphere_link_path = self._get_link_path(sphere_path)
            if sphere_link_path == link_path:
                sphere_names.append(sphere_path[len(link_path) :])

        return sphere_names

    # Used for XRDF files
    def write_spheres_to_dict(self, robot_prim_path, link_to_spheres):
        for sphere in self.path_2_spheres.values():
            prim_path = sphere.prim_path
            if is_prim_path_valid(prim_path):
                if prim_path[: len(robot_prim_path)] != robot_prim_path:
                    carb.log_warn(
                        "Not writing sphere at path {} to file because it is not nested under the robot Articulation".format(
                            prim_path
                        )
                    )
                    continue
                link_name = prim_path[len(robot_prim_path) + 1 : prim_path.rfind("/")]
                link_spheres = link_to_spheres.get(link_name, [])
                sphere_pose = self._round_list_floats(sphere.get_local_pose()[0])
                link_spheres.append({"center": sphere_pose, "radius": sphere.get_radius()})
                link_to_spheres[link_name] = link_spheres

    # Used for Robot Description Files
    def save_spheres(self, robot_prim_path, f):
        link_to_spheres = OrderedDict()
        for sphere in self.path_2_spheres.values():
            prim_path = sphere.prim_path
            if is_prim_path_valid(prim_path):
                if prim_path[: len(robot_prim_path)] != robot_prim_path:
                    carb.log_warn(
                        "Not writing sphere at path {} to file because it is not nested under the robot Articulation".format(
                            prim_path
                        )
                    )
                    continue
                link_name = prim_path[len(robot_prim_path) + 1 : prim_path.rfind("/")]
                link_spheres = link_to_spheres.get(link_name, [])
                sphere_pose = self._round_list_floats(sphere.get_local_pose()[0])
                link_spheres.append({"center": sphere_pose, "radius": round(sphere.get_radius(), 5)})
                link_to_spheres[link_name] = link_spheres

        f.write("collision_spheres:\n")
        for link_name, sphere_list in link_to_spheres.items():
            f.write("  - {}:\n".format(link_name))
            for sphere in sphere_list:
                f.write('    - "center": {}\n'.format(sphere["center"]))
                f.write('      "radius": {}\n'.format(sphere["radius"]))

    def on_shutdown(self):
        self.clear_spheres(store_op=False)
        self.clear_preview()
        if is_prim_path_valid(self._lula_path):
            delete_prim(self._lula_path)

    def _get_collision_sphere_base_path(self, link_path):
        return link_path + "/collision_sphere"

    def _get_collision_sphere_preview_path(self, link_path):
        return link_path + "/preview_sphere"

    def _round_list_floats(self, l, decimals=3):
        r = []
        for f in l:
            r.append(round(f, decimals))
        return r

    def _get_link_path(self, sphere_path):
        # Remove last element of Prim path to sphere

        slash_ind = sphere_path.rfind("/")

        link_path = sphere_path[:slash_ind]
        return link_path
