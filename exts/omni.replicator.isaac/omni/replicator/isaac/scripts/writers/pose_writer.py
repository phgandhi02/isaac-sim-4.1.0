# Copyright (c) 2024, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from functools import partial

import numpy as np
from omni.replicator.core import AnnotatorRegistry, BackendDispatch, Writer, WriterRegistry
from omni.replicator.core.scripts.functional import write_image, write_json
from omni.replicator.isaac.scripts.utils import calculate_truncation_ratio_simple
from PIL import Image, ImageDraw
from pxr import Gf

__version__ = "0.0.1"


class PoseWriter(Writer):
    """Pose Writer

    Args:
        output_dir:
            Output directory string that indicates the directory to save the results.
        use_subfolders:
            If True, the writer will create subfolders for each render product, otherwise all data is saved in the same folder.
        visibility_threshold:
            Objects with visibility below this threshold will be skipped.  Default: ``0.0`` (fully occluded)
        skip_empty_frames:
            If True, the writer will skip frames that do not have visible objects.
        write_debug_images:
            If True, the writer will include rgb images overlaid with the projected 3d bounding boxes.
        frame_padding:
            Pad the frame number with leading zeroes.  Default: ``4``
        format:
            Specifies which format the data will be outputted as. Default: ``None`` (will write most of the available data)
    """

    RGB_ANNOT_NAME = "rgb"
    BB3D_ANNOT_NAME = "bounding_box_3d_fast"
    CAM_PARAMS_ANNOT_NAME = "camera_params"
    SUPPORTED_FORMATS = set(["dope", "centerpose"])
    CUBOID_KEYPOINTS_ORDER = [
        "Center",
        "LDB",
        "LDF",
        "LUB",
        "LUF",
        "RDB",
        "RDF",
        "RUB",
        "RUF",
    ]
    CUBOID_KEYPOINT_COLORS = [
        "white",
        "red",
        "green",
        "blue",
        "yellow",
        "cyan",
        "magenta",
        "orange",
        "purple",
    ]
    CUBOID_EDGE_COLORS = {"front": "red", "back": "blue", "connecting": "green"}

    def __init__(
        self,
        output_dir: str,
        use_subfolders: bool = False,
        visibility_threshold: float = 0.0,
        skip_empty_frames: bool = True,
        write_debug_images: bool = False,
        frame_padding: int = 6,
        format: str = None,
        use_s3: bool = False,
        s3_bucket: str = None,
        s3_endpoint_url: str = None,
        s3_region: str = None,
    ):
        self.version = __version__

        if not use_s3:
            self.backend = BackendDispatch(output_dir=output_dir)
        else:
            self.backend = BackendDispatch(
                key_prefix=output_dir, bucket=s3_bucket, endpoint_url=s3_endpoint_url, region=s3_region
            )

        self._use_subfolders = use_subfolders
        self._visibility_threshold = visibility_threshold
        self._skip_empty_frames = skip_empty_frames
        self._write_debug_images = write_debug_images
        self._frame_padding = frame_padding
        self._frame_id = 0
        if format is not None and format.lower() not in self.SUPPORTED_FORMATS:
            raise ValueError(f"Unsupported format: {format}. Supported formats: {self.SUPPORTED_FORMATS}")
        else:
            self._format = format

        # Handle multiple render products scenario (e.g. single render product:'rgb', multiple render products: 'rgb-{rp_name}')
        self._render_product_names = []
        self._multiple_render_products = False

        # Store processed data to be written every frame in the selected format
        self._frame_data = {}

        # Store debug related data to write overlay images (projected cuboid, local frame axes, etc.)
        self._debug_frame_data = {}

        # For more details: https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator/annotators_details.html
        self.annotators = []
        self.annotators.append(AnnotatorRegistry.get_annotator(self.RGB_ANNOT_NAME))
        self.annotators.append(AnnotatorRegistry.get_annotator(self.BB3D_ANNOT_NAME))
        self.annotators.append(AnnotatorRegistry.get_annotator(self.CAM_PARAMS_ANNOT_NAME))

    # Abstract method from Writer to access the annotator data and write to disk
    def write(self, data: dict):
        # In case of multiple render products annotator names are suffixed with the render product name:
        # (e.g. 'rgb' -> 'rgb-{rp_name}')
        for rp_name in self._render_product_names:
            # Process the frame data of the current render product
            num_objs = self._process_frame_data(data, rp_name)

            # Early exist if empty frames should not be written
            if self._skip_empty_frames and num_objs == 0:
                continue

            # Create subfolder name if data should be separated for each render product
            rp_subfolder = f"{rp_name}/" if self._multiple_render_products and self._use_subfolders else ""

            # Write frame data to disk
            self._write_frame_data(data, rp_name, rp_subfolder)
            if self._write_debug_images:
                self._write_debug_data(data, rp_name, rp_subfolder)

            # If render products are NOT separated into subfolders increment the frame id after processing each render product
            if not self._use_subfolders:
                self._frame_id += 1

        # If render products are separated into subfolders increment the frame id after processing all render products
        if self._use_subfolders:
            self._frame_id += 1

    # Note that the frame id can be incremented for each render product write (if use_subfolders is False) or for each step (if use_subfolders is True)
    def get_current_frame_id(self):
        return self._frame_id

    # Process the render product data and store it in the selected format, return the number of objects in the frame
    def _process_frame_data(self, data: dict, render_product_name: str) -> int:
        # Store the frame data for writing to disk
        self._frame_data = {}

        # Get and process the camera parameters annotator data in the selected format
        camera_params_annot_name = (
            f"{self.CAM_PARAMS_ANNOT_NAME}-{render_product_name}"
            if self._multiple_render_products
            else self.CAM_PARAMS_ANNOT_NAME
        )
        camera_params = data[camera_params_annot_name]

        # Get and process the bounding box 3d annotator data in the selected format
        bb3d_annot_name = (
            f"{self.BB3D_ANNOT_NAME}-{render_product_name}" if self._multiple_render_products else self.BB3D_ANNOT_NAME
        )
        bb3d_data = data[bb3d_annot_name]["data"]
        bb3d_info = data[bb3d_annot_name]["info"]
        objs_data = self._process_bounding_boxes(bb3d_data, bb3d_info, camera_params)

        # Early exist if empty frames should be skipped and there are no visible objects in the frame
        if self._skip_empty_frames and len(objs_data) == 0:
            return 0

        # Store the camera information in the
        self._frame_data["camera_data"] = self._process_camera_parameters(camera_params)

        # Store the predefined order of the cuboid keypoints
        self._frame_data["keypoint_order"] = self.CUBOID_KEYPOINTS_ORDER

        # Add the objects data to the frame entries
        self._frame_data["objects"] = objs_data

        return len(objs_data)

    # Process the bounding box annotator data (extract objects label, location, rotation, visibility, etc.)
    def _process_bounding_boxes(self, bb3d_data, bb3d_info, camera_params) -> list:
        # Map the ids to class names from the bbox annotator "idToLabels" data
        # ('idToLabels': {0: {'class': 'cube'}, 1: {'class': 'sphere'}} -> {0: 'cube', 1: 'sphere'})
        id_to_labels = {k: v["class"] for k, v in bb3d_info["idToLabels"].items()}

        if self._write_debug_images:
            self._debug_frame_data["world_frame_transforms"] = []
            self._debug_frame_data["projected_keypoints"] = []
            self._debug_frame_data["size_local"] = []
            self._debug_frame_data["center_local"] = []
        # Iterate the bounding box data and extract the object informations
        objs = []
        for i, bbox in enumerate(bb3d_data):
            obj = {}
            # `occlusionRatio` represents (visible pixels / total pixels) where `0.0` is fully visible and `1.0` is fully occluded
            # NOTE: `obj_visibility` is inverted to match the format where `0.0` is fully occluded and `1.0`` is fully visible
            obj_visibility = 1.0 - abs(float(bbox["occlusionRatio"]))

            # Early exit if visibility is below the given threshold
            if obj_visibility <= self._visibility_threshold:
                continue

            obj["label"] = id_to_labels[bbox["semanticId"]]
            obj["prim_path"] = bb3d_info["primPaths"][i]
            obj["visibility"] = round(obj_visibility, 3)

            # Local space to to world transform (row-major)
            local_to_world_tf = bbox["transform"]

            if self._format is None:
                obj["local_to_world_transform"] = local_to_world_tf.tolist()
                # Extract world frame location (last row) and rotation matrix (3x3) from the row-major transform matrix
                location_world_frame = local_to_world_tf[3, :3]
                obj["location_world_frame"] = location_world_frame.tolist()
                rotation_matrix_world_frame = local_to_world_tf[:3, :3]
                obj["rotation_matrix_world_frame"] = rotation_matrix_world_frame.tolist()

                # Get the world frame quaternion using Gf.Transform (row-major)
                local_to_world_tf_gf = Gf.Transform()
                local_to_world_tf_gf.SetMatrix(Gf.Matrix4d(local_to_world_tf.tolist()))
                quat_world_frame_gf = local_to_world_tf_gf.GetRotation().GetQuat()
                obj["quat_wxyz_world_frame"] = [quat_world_frame_gf.GetReal()] + list(
                    quat_world_frame_gf.GetImaginary()
                )
            if self._write_debug_images:
                self._debug_frame_data["world_frame_transforms"].append(local_to_world_tf)

            # World to camera transform (row-major) (transform a point from world coordinate to camera coordinate)
            world_to_camera_tf = camera_params["cameraViewTransform"].reshape(4, 4)
            # Object world space to camera frame transform (row-major matrix multiplication)
            obj_to_camera_tf = world_to_camera_tf @ local_to_world_tf
            # Extract camera frame location (last row) and rotation matrix (3x3) from the row-major transform matrix
            location_camera_frame = obj_to_camera_tf[3, :3]
            if self._format is None:
                obj["location_camera_frame"] = location_camera_frame.tolist()
            elif self._format == "centerpose" or self._format == "dope":
                obj["location"] = location_camera_frame.tolist()
            if self._format is None:
                rotation_matrix_camera_frame = obj_to_camera_tf[:3, :3]
                obj["rotation_matrix_camera_frame"] = rotation_matrix_camera_frame.tolist()
            # Get the camera frame quaternion using Gf.Transform (row-major)
            obj_to_camera_tf_gf = Gf.Transform()
            obj_to_camera_tf_gf.SetMatrix(Gf.Matrix4d(obj_to_camera_tf.tolist()))
            quat_camera_frame_gf = obj_to_camera_tf_gf.GetRotation().GetQuat()
            if self._format is None:
                obj["quat_wxyz_camera_frame"] = [quat_camera_frame_gf.GetReal()] + list(
                    quat_camera_frame_gf.GetImaginary()
                )
            elif self._format == "centerpose" or self._format == "dope":
                obj["quaternion_xyzw"] = list(quat_camera_frame_gf.GetImaginary()) + [quat_camera_frame_gf.GetReal()]

            # Size of the object before scale (NOTE: scale is not applied yet to objects in local frame)
            min_local = np.array([bbox["x_min"], bbox["y_min"], bbox["z_min"], 1])
            max_local = np.array([bbox["x_max"], bbox["y_max"], bbox["z_max"], 1])
            size_local = np.abs(max_local - min_local)[:3].tolist()
            center_local = min_local + (max_local - min_local) / 2
            if self._write_debug_images:
                self._debug_frame_data["size_local"].append(size_local)
                self._debug_frame_data["center_local"].append(center_local[:3].tolist())

            # Cuboid keypoints in local frame
            keypoints_local = {
                "Center": center_local,
                "LDB": np.array([bbox["x_min"], bbox["y_min"], bbox["z_min"], 1]),  # Left-Down-Back
                "LDF": np.array([bbox["x_min"], bbox["y_min"], bbox["z_max"], 1]),  # Left-Down-Front
                "LUB": np.array([bbox["x_min"], bbox["y_max"], bbox["z_min"], 1]),  # Left-Up-Back
                "LUF": np.array([bbox["x_min"], bbox["y_max"], bbox["z_max"], 1]),  # Left-Up-Front
                "RDB": np.array([bbox["x_max"], bbox["y_min"], bbox["z_min"], 1]),  # Right-Down-Back
                "RDF": np.array([bbox["x_max"], bbox["y_min"], bbox["z_max"], 1]),  # Right-Down-Front
                "RUB": np.array([bbox["x_max"], bbox["y_max"], bbox["z_min"], 1]),  # Right-Up-Back
                "RUF": np.array([bbox["x_max"], bbox["y_max"], bbox["z_max"], 1]),  # Right-Up-Front
            }

            # Calculate the (scaled) size of the object from its world bounds (NOTE: scale is applied through the transform)
            min_world = min_local @ local_to_world_tf
            max_world = max_local @ local_to_world_tf
            size_world = np.abs(max_world - min_world)[:3].tolist()
            if self._format is None:
                obj["size"] = size_world
            elif self._format == "centerpose":
                obj["scale"] = size_world

            # Transform the cuboid keypoints from local to world frame in the given order
            keypoints_world_ordered = [keypoints_local[k] @ local_to_world_tf for k in self.CUBOID_KEYPOINTS_ORDER]
            if self._format is None:
                obj["cuboid_keypoints_world_frame"] = [point[:3].tolist() for point in keypoints_world_ordered]
            # Transform the cuboid keypoints from world to camera frame
            keypoints_camera_ordered = [point @ world_to_camera_tf for point in keypoints_world_ordered]
            if self._format is None:
                obj["cuboid_keypoints_camera_frame"] = [point[:3].tolist() for point in keypoints_camera_ordered]
            elif self._format == "centerpose":
                obj["keypoints_3d"] = [point[:3].tolist() for point in keypoints_camera_ordered]
            # Get the camera projection matrix and screen size to project the cuboid keypoints to screen space
            cam_projection_tf = camera_params["cameraProjection"].reshape((4, 4))
            screen_size = camera_params["renderProductResolution"]
            keypoints_projected_ordered = [
                self._project_camera_point_to_screen(point, cam_projection_tf, screen_size)
                for point in keypoints_camera_ordered
            ]
            if self._format is None:
                obj["cuboid_keypoints_projected"] = keypoints_projected_ordered
            elif self._format == "centerpose" or self._format == "dope":
                obj["projected_cuboid"] = keypoints_projected_ordered
            if self._write_debug_images:
                self._debug_frame_data["projected_keypoints"].append(keypoints_projected_ordered)

            obj["truncation_ratio"] = calculate_truncation_ratio_simple(
                keypoints_projected_ordered, screen_size[0], screen_size[1]
            )

            objs.append(obj)

        return objs

    # Get the camera parameters from the annotator data
    def _process_camera_parameters(self, camera_params) -> dict:
        camera_data = {}
        if self._format is None:
            camera_data["aperture"] = camera_params["cameraAperture"].tolist()
            camera_data["aperture_offset"] = camera_params["cameraApertureOffset"].tolist()
            camera_data["focal_length"] = float(camera_params["cameraFocalLength"])
            camera_data["resolution"] = camera_params["renderProductResolution"].tolist()
            camera_data["meters_per_scene_unit"] = float(camera_params["metersPerSceneUnit"])

        # OV only supports square pixels, so the pixel size is the same in both x and y directions
        # https://docs.omniverse.nvidia.com/materials-and-rendering/latest/cameras.html#cameras
        pixel_size = camera_params["cameraAperture"][0] / camera_params["renderProductResolution"][0]
        camera_data["intrinsics"] = {
            "fx": camera_params["cameraFocalLength"] / pixel_size,
            "fy": camera_params["cameraFocalLength"] / pixel_size,
            "cx": camera_params["renderProductResolution"][0] / 2.0 + camera_params["cameraApertureOffset"][0],
            "cy": camera_params["renderProductResolution"][1] / 2.0 + camera_params["cameraApertureOffset"][1],
        }
        camera_data["camera_view_matrix"] = np.round(camera_params["cameraViewTransform"], 5).reshape(4, 4).tolist()
        camera_data["camera_projection_matrix"] = np.round(camera_params["cameraProjection"], 5).reshape(4, 4).tolist()

        if self._format == "centerpose":
            camera_data["width"] = camera_params["renderProductResolution"].tolist()[0]
            camera_data["height"] = camera_params["renderProductResolution"].tolist()[1]

        # Debug data needed for the overlay projections
        if self._write_debug_data:
            self._debug_frame_data["camera_projection_matrix"] = camera_params["cameraProjection"].reshape(4, 4)
            self._debug_frame_data["camera_view_matrix"] = camera_params["cameraViewTransform"].reshape(4, 4)
            self._debug_frame_data["resolution"] = camera_params["renderProductResolution"]

        return camera_data

    # Write the processed data to disk
    def _write_frame_data(self, data: dict, render_product_name: str, render_product_subfolder: str = ""):
        # Write frame data to as a JSON file
        file_path_json = f"{render_product_subfolder}{self._frame_id:0{self._frame_padding}}.json"
        self.backend.schedule(write_json, path=file_path_json, data=self._frame_data, indent=2)

        # Get RGB data from annotator
        rgb_annot_name = (
            f"{self.RGB_ANNOT_NAME}-{render_product_name}" if self._multiple_render_products else self.RGB_ANNOT_NAME
        )
        rgb_data = data[rgb_annot_name]

        # Write image to disk
        rgb_file_path = f"{render_product_subfolder}{self._frame_id:0{self._frame_padding}}.png"
        self.backend.schedule(write_image, path=rgb_file_path, data=rgb_data)

    # Write overlay debug data to disk
    def _write_debug_data(self, data: dict, render_product_name: str, render_product_subfolder: str = ""):
        # Get RGB data to overlay with the debug information
        rgb_annot_name = (
            f"{self.RGB_ANNOT_NAME}-{render_product_name}" if self._multiple_render_products else self.RGB_ANNOT_NAME
        )
        rgb_data = data[rgb_annot_name]

        # Create overlay image from the RGB data
        rgb_img = Image.fromarray(rgb_data)
        draw = ImageDraw.Draw(rgb_img)

        # Draw the projected cuboid and its edges
        for keypoints in self._debug_frame_data["projected_keypoints"]:
            self._draw_projected_keypoints(draw, keypoints)

        # Get the stored camera parameters for debug purposes
        camera_projection_matrix = self._debug_frame_data["camera_projection_matrix"]
        camera_view_matrix = self._debug_frame_data["camera_view_matrix"]
        screen_size = self._debug_frame_data["resolution"]

        # Draw objects local frame axes
        for i, tf in enumerate(self._debug_frame_data["world_frame_transforms"]):
            size = self._debug_frame_data["size_local"][i]
            center = self._debug_frame_data["center_local"][i]
            self._draw_local_frame_axes(
                draw,
                tf,
                camera_view_matrix,
                camera_projection_matrix,
                screen_size,
                size_local=size,
                origin_local=center,
            )

        # Overlay the world frame axes on the bottom left part of the RGB image
        self._draw_world_frame_axes_bottom_left(draw, camera_view_matrix, camera_projection_matrix, screen_size)

        file_path = f"{render_product_subfolder}{self._frame_id:0{self._frame_padding}}_overlay.png"
        self.backend.schedule(write_image, path=file_path, data=np.asarray(rgb_img))

    # Transform a 3D point from world coordinates to camera coordinates
    def _world_point_to_camera_point(self, world_point, view_matrix):
        # Convert the 3D point to homogeneous coordinates (if not already in that form)
        point_homogeneous = np.array(world_point) if len(world_point) == 4 else np.array([*world_point, 1.0])

        # Transform to camera frame (row-major representation where the translation vector is on the left side of the multiplication)
        point_camera = point_homogeneous @ view_matrix

        return point_camera

    # Project a 3D point from camera coordinates to 2D screen coordinates
    def _project_camera_point_to_screen(self, camera_point, projection_matrix, screen_size):
        # Apply the projection matrix to project to screen coordinates
        point_screen = camera_point @ projection_matrix

        # Normalize to NDC (Normalized Device Coordinates) by dividing x, y, z, by w: (x, y, z, w) -> (x/w, y/w, z/w, 1)
        point_screen_normalized = point_screen / point_screen[3]

        # Map NDC to screen coordinates. Adjust x and y for screen dimensions, flipping y to match screen's coordinate system.
        x = (point_screen_normalized[0] + 1) * screen_size[0] / 2
        y = (1 - point_screen_normalized[1]) * screen_size[1] / 2

        return round(x), round(y)

    # Project a 3D point from world coordinates to 2D screen coordinates
    def _project_world_point_to_screen(self, world_point, view_matrix, projection_matrix, screen_size):
        point_camera = self._world_point_to_camera_point(world_point, view_matrix)
        return self._project_camera_point_to_screen(point_camera, projection_matrix, screen_size)

    # Projects the local frame axes of the object to the screen
    def _draw_local_frame_axes(
        self,
        draw,
        local_to_world_transform,
        camera_view_matrix,
        camera_projection_matrix,
        screen_size,
        size_local=[1, 1, 1],
        origin_local=[0, 0, 0],
        axes_length_perc=0.25,
    ):
        # The length of the local axes is a percentage of the mean size of the object in local frame (before any scaling)
        local_axes_length = np.mean(size_local) * axes_length_perc

        # Define the end points of the local coordinate system axes include the local center of the object bounds
        origin_local = np.array([origin_local[0], origin_local[1], origin_local[2], 1])
        x_axis_end_point_local = np.array([local_axes_length + origin_local[0], origin_local[1], origin_local[2], 1])
        y_axis_end_point_local = np.array([origin_local[0], local_axes_length + origin_local[1], origin_local[2], 1])
        z_axis_end_point_local = np.array([origin_local[0], origin_local[1], local_axes_length + origin_local[2], 1])

        # Transform local end points to world frame using row-major matrix multiplication (translation on the left side)
        origin_world = origin_local @ local_to_world_transform
        x_axis_end_point_world = x_axis_end_point_local @ local_to_world_transform
        y_axis_end_point_world = y_axis_end_point_local @ local_to_world_transform
        z_axis_end_point_world = z_axis_end_point_local @ local_to_world_transform

        # Define a partial helper function to project 3D world points to 2D screen points
        project_to_screen = partial(
            self._project_world_point_to_screen,
            view_matrix=camera_view_matrix,
            projection_matrix=camera_projection_matrix,
            screen_size=screen_size,
        )

        # Project the origin and axes end points from 3D world coordinates to 2D screen coordinates
        origin_2d = project_to_screen(origin_world)
        x_axis_end_2d = project_to_screen(x_axis_end_point_world)
        y_axis_end_2d = project_to_screen(y_axis_end_point_world)
        z_axis_end_2d = project_to_screen(z_axis_end_point_world)

        # Draw the 3D axes on the 2D screen using lines with appropriate colors for each axis
        draw.line([origin_2d, x_axis_end_2d], fill="red", width=2)  # X-axis in red
        draw.line([origin_2d, y_axis_end_2d], fill="green", width=2)  # Y-axis in green
        draw.line([origin_2d, z_axis_end_2d], fill="blue", width=2)  # Z-axis in blue

    # Draws the world frame axes at the bottom left corner of the image.
    def _draw_world_frame_axes_bottom_left(
        self, draw, camera_view_matrix, camera_projection_matrix, screen_size, axes_scale=0.03, margin_percentage=0.03
    ):
        # Set a world location for the axes origin (1 unit in front of the camera) where -Z is the camera's forward direction
        camera_to_world_matrix = np.linalg.inv(camera_view_matrix)
        point_in_camera_space = np.array([0, 0, -1, 1])

        # Create the axes in world (1 unit in front of the camera) with the given axes size
        origin_world = point_in_camera_space @ camera_to_world_matrix
        x_axis_end_point_world = np.array([axes_scale + origin_world[0], origin_world[1], origin_world[2], 1])
        y_axis_end_point_world = np.array([origin_world[0], axes_scale + origin_world[1], origin_world[2], 1])
        z_axis_end_point_world = np.array([origin_world[0], origin_world[1], axes_scale + origin_world[2], 1])

        # Create a partial function with fixed camera parameters
        project_to_screen = partial(
            self._project_world_point_to_screen,
            view_matrix=camera_view_matrix,
            projection_matrix=camera_projection_matrix,
            screen_size=screen_size,
        )

        # Project the origin and axes end points into 2D screen coordinates
        origin_2d = project_to_screen(origin_world)
        x_axis_end_2d = project_to_screen(x_axis_end_point_world)
        y_axis_end_2d = project_to_screen(y_axis_end_point_world)
        z_axis_end_2d = project_to_screen(z_axis_end_point_world)

        # Calculate offset margin (a percentage of the screen size) to ensure axes are not on the edge of the screen
        margin = int(margin_percentage * min(screen_size))
        offset_x = margin - min(origin_2d[0], x_axis_end_2d[0], y_axis_end_2d[0], z_axis_end_2d[0])
        offset_y = screen_size[1] - margin - max(origin_2d[1], x_axis_end_2d[1], y_axis_end_2d[1], z_axis_end_2d[1])

        # Apply the offset to the projected points
        origin_2d = (origin_2d[0] + offset_x, origin_2d[1] + offset_y)
        x_axis_end_2d = (x_axis_end_2d[0] + offset_x, x_axis_end_2d[1] + offset_y)
        y_axis_end_2d = (y_axis_end_2d[0] + offset_x, y_axis_end_2d[1] + offset_y)
        z_axis_end_2d = (z_axis_end_2d[0] + offset_x, z_axis_end_2d[1] + offset_y)

        # Draw the axes with the specified colors
        draw.line([origin_2d, x_axis_end_2d], fill="red", width=2)  # X-axis in red
        draw.line([origin_2d, y_axis_end_2d], fill="green", width=2)  # Y-axis in green
        draw.line([origin_2d, z_axis_end_2d], fill="blue", width=2)  # Z-axis in blue

    # Draw the projected cuboid and its edges
    def _draw_projected_keypoints(self, draw, keypoints, point_size=4, edge_size=2):
        # Draw the projected cuboid keypoint vertices in the specified colors
        for i, point in enumerate(keypoints):
            draw.ellipse(
                (point[0] - point_size, point[1] - point_size, point[0] + point_size, point[1] + point_size),
                fill=self.CUBOID_KEYPOINT_COLORS[i],
            )

        # Draw the edges of the projected cuboid with specified colors for each set
        edges = {
            "front": [(1, 2), (2, 4), (4, 3), (3, 1)],  # Front face
            "back": [(5, 6), (6, 8), (8, 7), (7, 5)],  # Back face
            "connecting": [(1, 5), (2, 6), (3, 7), (4, 8)],  # Connecting edges
        }
        for edge_type, edge_list in edges.items():
            for start, end in edge_list:
                draw.line(keypoints[start] + keypoints[end], fill=self.CUBOID_EDGE_COLORS[edge_type], width=edge_size)

    # Override to cache the render product names
    def attach(self, render_products, trigger="omni.replicator.core.OgnOnFrame"):
        super().attach(render_products, trigger)
        self._cache_render_product_names(render_products)

    # Override to clear the writer state
    def detach(self):
        super().detach()
        self._reset_writer_state()

    # Save the render product names for easier data access in the write function
    def _cache_render_product_names(self, render_products):
        if not isinstance(render_products, list):
            render_products = [render_products]
        for rp in render_products:
            rp_name = rp.path.split("/Render/")[-1]
            self._render_product_names.append(rp_name)
        # Check if there are multiple render products, this is used to suffix the annotator names for data access
        self._multiple_render_products = len(self._render_product_names) > 1

    # Reset the writer state
    def _reset_writer_state(self):
        self._render_product_names = []
        self._frame_id = 0
        self._multiple_render_products = False


WriterRegistry.register(PoseWriter)
# Adding to default writers for Replicator telemetry tracking
WriterRegistry._default_writers.append("PoseWriter") if "PoseWriter" not in WriterRegistry._default_writers else None
