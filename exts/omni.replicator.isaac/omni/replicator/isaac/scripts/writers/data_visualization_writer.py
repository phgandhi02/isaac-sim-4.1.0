# Copyright (c) 2023-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import carb
import numpy as np
from omni.replicator.core import AnnotatorRegistry, BackendDispatch, Writer, WriterRegistry
from PIL import Image, ImageDraw

__version__ = "0.0.2"


class DataVisualizationWriter(Writer):
    """Data Visualization Writer

    This writer can be used to visualize various annotator data.

    Supported annotators:
    - bounding_box_2d_tight
    - bounding_box_2d_loose
    - bounding_box_3d

    Supported backgrounds:
    - rgb
    - normals

    Args:
        output_dir (str):
            Output directory for the data visualization files forwarded to the backend writer.
        bounding_box_2d_tight (bool, optional):
            If True, 2D tight bounding boxes will be drawn on the selected background (transparent by default).
            Defaults to False.
        bounding_box_2d_tight_params (dict, optional):
            Parameters for the 2D tight bounding box annotator. Defaults to None.
        bounding_box_2d_loose (bool, optional):
            If True, 2D loose bounding boxes will be drawn on the selected background (transparent by default).
            Defaults to False.
        bounding_box_2d_loose_params (dict, optional):
            Parameters for the 2D loose bounding box annotator. Defaults to None.
        bounding_box_3d (bool, optional):
            If True, 3D bounding boxes will be drawn on the selected background (transparent by default). Defaults to False.
        bounding_box_3d_params (dict, optional):
            Parameters for the 3D bounding box annotator. Defaults to None.
        frame_padding (int, optional):
            Number of digits used for the frame number in the file name. Defaults to 4.

    """

    BB_2D_TIGHT = "bounding_box_2d_tight_fast"
    BB_2D_LOOSE = "bounding_box_2d_loose_fast"
    BB_3D = "bounding_box_3d_fast"
    SUPPORTED_BACKGROUNDS = ["rgb", "normals"]

    def __init__(
        self,
        output_dir: str,
        bounding_box_2d_tight: bool = False,
        bounding_box_2d_tight_params: dict = None,
        bounding_box_2d_loose: bool = False,
        bounding_box_2d_loose_params: dict = None,
        bounding_box_3d: bool = False,
        bounding_box_3d_params: dict = None,
        frame_padding: int = 4,
    ):
        self.version = __version__
        self._output_dir = output_dir
        self.backend = BackendDispatch({"paths": {"out_dir": output_dir}})

        self._frame_id = 0
        self._frame_padding = frame_padding

        self._render_product_names = []
        self.annotators = []
        self._annotator_params = {}
        valid_backgrounds = set()

        # Add the enabled annotators to the writer, store its parameters, and verify if a valid background type is given
        if bounding_box_2d_tight:
            self.annotators.append(AnnotatorRegistry.get_annotator(self.BB_2D_TIGHT))
            if bounding_box_2d_tight_params is not None:
                self._annotator_params[self.BB_2D_TIGHT] = bounding_box_2d_tight_params
                if (background := bounding_box_2d_tight_params.get("background")) and self._is_valid_background(
                    background
                ):
                    valid_backgrounds.add(background)
            else:
                self._annotator_params[self.BB_2D_TIGHT] = {}

        if bounding_box_2d_loose:
            self.annotators.append(AnnotatorRegistry.get_annotator(self.BB_2D_LOOSE))
            if bounding_box_2d_loose_params is not None:
                self._annotator_params[self.BB_2D_LOOSE] = bounding_box_2d_loose_params
                if (background := bounding_box_2d_loose_params.get("background")) and self._is_valid_background(
                    background
                ):
                    valid_backgrounds.add(background)
            else:
                self._annotator_params[self.BB_2D_LOOSE] = {}

        if bounding_box_3d:
            self.annotators.append(AnnotatorRegistry.get_annotator(self.BB_3D))
            # The 'camera params' annotator contains the camera data needed for the 3D bounding box screen projection
            self.annotators.append(AnnotatorRegistry.get_annotator("camera_params"))
            if bounding_box_3d_params is not None:
                self._annotator_params[self.BB_3D] = bounding_box_3d_params
                if (background := bounding_box_3d_params.get("background")) and self._is_valid_background(background):
                    valid_backgrounds.add(background)
            else:
                self._annotator_params[self.BB_3D] = {}

        # Add the valid background annotators to the writer
        for background in valid_backgrounds:
            self.annotators.append(AnnotatorRegistry.get_annotator(background))

    def write(self, data: dict):
        # Cache render product names
        if len(self._render_product_names) == 0:
            self._save_render_product_names(data)

        # In case of multiple render products:
        # * annotator names are suffixed with the render product name
        # * data is separated into subfolders using the render product names
        multiple_render_products = len(self._render_product_names) > 1
        for rp_name in self._render_product_names:
            rp_subfolder = "" if not multiple_render_products else f"{rp_name}/"

            if self.BB_2D_TIGHT in self._annotator_params:
                annot_name = self.BB_2D_TIGHT if not multiple_render_products else f"{self.BB_2D_TIGHT}-{rp_name}"
                write_params = self._annotator_params.get(self.BB_2D_TIGHT, {})
                self._write_bounding_box_overlay(data, annot_name, rp_name, rp_subfolder, write_params)

            if self.BB_2D_LOOSE in self._annotator_params:
                annot_name = self.BB_2D_LOOSE if not multiple_render_products else f"{self.BB_2D_LOOSE}-{rp_name}"
                write_params = self._annotator_params.get(self.BB_2D_LOOSE, {})
                self._write_bounding_box_overlay(data, annot_name, rp_name, rp_subfolder, write_params)

            if self.BB_3D in self._annotator_params:
                annot_name = self.BB_3D if not multiple_render_products else f"{self.BB_3D}-{rp_name}"
                write_params = self._annotator_params.get(self.BB_3D, {})
                self._write_bounding_box_overlay(data, annot_name, rp_name, rp_subfolder, write_params)

        self._frame_id += 1

    def _write_bounding_box_overlay(
        self,
        data: dict,
        bb_annot_name: str,
        render_product_name: str,
        render_product_subfolder: str,
        write_params: dict,
    ):
        # Check the bbox data type
        bbox_type = ""
        if bb_annot_name.startswith(self.BB_2D_TIGHT):
            bbox_type = "2d_tight"
        elif bb_annot_name.startswith(self.BB_2D_LOOSE):
            bbox_type = "2d_loose"
        elif bb_annot_name.startswith(self.BB_3D):
            bbox_type = "3d"

        # Get the background type on which the bounding boxes will be drawn
        img = self._get_background_image(data, render_product_name, write_params)

        # Draw the bounding boxes on the selected background image
        self._draw_bounding_boxes(img, data, bb_annot_name, render_product_name, bbox_type, write_params)

        # Save the image
        file_path = f"{render_product_subfolder}bounding_box_{bbox_type}_{self._frame_id:0{self._frame_padding}}.png"
        self.backend.write_image(file_path, np.asarray(img))

    def _get_background_image(self, data: dict, render_product_name: str, write_params: dict) -> Image:
        # Check the background type for the given annotator
        if "background" in write_params:
            background_type = write_params["background"]
            multiple_render_products = len(self._render_product_names) > 1
            annot_name = background_type if not multiple_render_products else f"{background_type}-{render_product_name}"

            if background_type == "rgb":
                annot_data = data[annot_name]
                return Image.fromarray(annot_data)

            if background_type == "normals":
                annot_data = data[annot_name]
                colored_data = ((annot_data * 0.5 + 0.5) * 255).astype(np.uint8)
                return Image.fromarray(colored_data)

        # If no  background is chosen use a transparent image as default
        resolution = tuple(data[f"rp_{render_product_name}"]["resolution"])
        return Image.new("RGBA", resolution, (0, 0, 0, 0))

    def _draw_bounding_boxes(
        self,
        background_img: Image,
        data: dict,
        bb_annot_name: str,
        render_product_name: str,
        bbox_type: str,
        write_params: dict,
    ):
        # Draw the bounding boxes on the selected background image
        draw = ImageDraw.Draw(background_img)

        if bbox_type == "2d_tight" or bbox_type == "2d_loose":
            self._draw_2d_bounding_boxes(draw, data, bb_annot_name, write_params)

        if bbox_type == "3d":
            self._draw_3d_bounding_boxes(draw, data, bb_annot_name, render_product_name, write_params)

    def _draw_2d_bounding_boxes(self, draw: ImageDraw, data: dict, bb_annot_name: str, write_params: dict):
        # Get the 2d bbox data from the annotator
        bb_annot_data = data[bb_annot_name]["data"]

        # Get the recangle draw parameters
        fill_color = None if "fill" not in write_params else write_params["fill"]
        rectangle_color = "green" if "outline" not in write_params else write_params["outline"]
        rectangle_width = 1 if "width" not in write_params else write_params["width"]

        # Iterate the bounding boxes and draw the rectangles
        for bbox_data in bb_annot_data:
            # ('semanticId', '<u4'), ('x_min', '<i4'), ('y_min', '<i4'), ('x_max', '<i4'), ('y_max', '<i4'), ('occlusionRatio', '<f4')
            x_min, y_min, x_max, y_max = bbox_data[1], bbox_data[2], bbox_data[3], bbox_data[4]
            draw.rectangle(
                [x_min, y_min, x_max, y_max], fill=fill_color, outline=rectangle_color, width=rectangle_width
            )

    def _draw_3d_bounding_boxes(
        self, draw: ImageDraw, data: dict, bb_annot_name: str, render_product_name: str, write_params: dict
    ):
        # Get the 3d bbox data from the annotator
        annot_data = data[bb_annot_name]["data"]

        # Access the camera parameters
        multiple_render_products = len(self._render_product_names) > 1
        camera_params_annot_name = (
            "camera_params" if not multiple_render_products else f"camera_params-{render_product_name}"
        )

        # Transpose is needed for the row-column-major conversion
        cam_view_transform = data[camera_params_annot_name]["cameraViewTransform"].reshape((4, 4))
        cam_view_transform = cam_view_transform.T
        cam_projection_transform = data[camera_params_annot_name]["cameraProjection"].reshape((4, 4))
        cam_projection_transform = cam_projection_transform.T

        # The resolution is used to map the Normalized Device Coordinates (NDC) to screen space
        screen_width, screen_height = data[camera_params_annot_name]["renderProductResolution"]

        # Get the line draw parameters
        line_color = "green" if "fill" not in write_params else write_params["fill"]
        line_width = 1 if "width" not in write_params else write_params["width"]

        # Iterate the bounding boxes and draw the edges
        for bbox_data in annot_data:
            # ('semanticId', '<u4'), ('x_min', '<f4'), ('y_min', '<f4'), ('z_min', '<f4'), ('x_max', '<f4'), ('y_max', '<f4'), ('z_max', '<f4'), ('transform', '<f4', (4, 4)), ('occlusionRatio', '<f4')
            # Bounding box points in local coordinate system
            x_min, y_min, z_min, x_max, y_max, z_max = (
                bbox_data[1],
                bbox_data[2],
                bbox_data[3],
                bbox_data[4],
                bbox_data[5],
                bbox_data[6],
            )

            # Transformation matrix from local to world coordinate system
            local_to_world_transform = bbox_data[7]
            local_to_world_transform = local_to_world_transform.T

            # Calculate all 8 vertices of the bounding box in local space
            vertices_local = [
                np.array([x_min, y_min, z_min, 1]),
                np.array([x_min, y_min, z_max, 1]),
                np.array([x_min, y_max, z_min, 1]),
                np.array([x_min, y_max, z_max, 1]),
                np.array([x_max, y_min, z_min, 1]),
                np.array([x_max, y_min, z_max, 1]),
                np.array([x_max, y_max, z_min, 1]),
                np.array([x_max, y_max, z_max, 1]),
            ]

            # Transform vertices to world, camera, and screen space
            vertices_screen = []
            for vertex in vertices_local:
                # Transform to world space
                world_homogeneous = np.dot(local_to_world_transform, vertex)
                # Transform to camera space
                camera_homogeneous = np.dot(cam_view_transform, world_homogeneous)
                # Projection transformation
                clip_space = np.dot(cam_projection_transform, camera_homogeneous)
                # Normalize Device Coordinates (NDC)
                ndc = clip_space[:3] / clip_space[3]
                # Map NDC to screen space
                screen_point = ((ndc[0] + 1) * screen_width / 2, (1 - ndc[1]) * screen_height / 2)
                vertices_screen.append(screen_point)

            # Draw the bounding box edges
            draw.line([vertices_screen[0], vertices_screen[1]], fill=line_color, width=line_width)
            draw.line([vertices_screen[0], vertices_screen[2]], fill=line_color, width=line_width)
            draw.line([vertices_screen[0], vertices_screen[4]], fill=line_color, width=line_width)
            draw.line([vertices_screen[1], vertices_screen[3]], fill=line_color, width=line_width)
            draw.line([vertices_screen[1], vertices_screen[5]], fill=line_color, width=line_width)
            draw.line([vertices_screen[2], vertices_screen[3]], fill=line_color, width=line_width)
            draw.line([vertices_screen[2], vertices_screen[6]], fill=line_color, width=line_width)
            draw.line([vertices_screen[3], vertices_screen[7]], fill=line_color, width=line_width)
            draw.line([vertices_screen[4], vertices_screen[5]], fill=line_color, width=line_width)
            draw.line([vertices_screen[4], vertices_screen[6]], fill=line_color, width=line_width)
            draw.line([vertices_screen[5], vertices_screen[7]], fill=line_color, width=line_width)
            draw.line([vertices_screen[6], vertices_screen[7]], fill=line_color, width=line_width)

    def _save_render_product_names(self, data: dict):
        for k in data.keys():
            if k.startswith("rp_"):
                self._render_product_names.append(k[3:])

    def _is_valid_background(self, background: str) -> bool:
        if background in self.SUPPORTED_BACKGROUNDS:
            return True
        else:
            carb.log_warn(
                f"Background '{background}' is not supported, please choose from the supported types: {self.SUPPORTED_BACKGROUNDS}, default transparent image will be used instead.."
            )
            return False

    def detach(self):
        self._render_product_names = []
        self._frame_id = 0
        return super().detach()


WriterRegistry.register(DataVisualizationWriter)
# Adding to default writers for Replicator telemetry tracking
WriterRegistry._default_writers.append(
    "DataVisualizationWriter"
) if "DataVisualizationWriter" not in WriterRegistry._default_writers else None
