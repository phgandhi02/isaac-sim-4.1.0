# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

simulation_app = SimulationApp()

# Expected delay values (first frame is expected to be different than the following frames)
EXPECTED_FIRST_DELAY = 2
EXPECTED_FOLLOWING_DELAYS = 1

import argparse

import carb.settings
import numpy as np
import omni.graph.core as og
import omni.replicator.core as rep
import omni.timeline
from omni.isaac.core.utils.semantics import add_update_semantics, remove_all_semantics
from omni.isaac.nucleus import get_assets_root_path
from pxr import UsdGeom

parser = argparse.ArgumentParser()
parser.add_argument(
    "--waitidle",
    required=False,
    help="Set `hydraEngine/waitIdle` and `renderer/waitIdle` to True at runtime",
    action="store_true",
)
parser.add_argument("--env-url", default=None, help="Path to a custom environment url, default None")
parser.add_argument(
    "--num-additional-render-products",
    type=int,
    default=0,
    help="Number of additional render products to create to increase the rendering load",
)
args, unknown = parser.parse_known_args()


# Create a new empty stage or load a custom environment
env_url = args.env_url
if env_url is not None:
    env_path = env_url if env_url.startswith("omniverse://") else get_assets_root_path() + env_url
    print(f"[FrameDelay] Loading custom stage from path: {env_path}")
    omni.usd.get_context().open_stage(env_path)
else:
    print(f"[FrameDelay] Creating a new emtpy stage")
    omni.usd.get_context().new_stage()
stage = omni.usd.get_context().get_stage()

# Clear any previous semantic data in the stage (we only want to track the cube location at index 0)
for prim in stage.Traverse():
    remove_all_semantics(prim, recursive=True)

# Create additional render products to increase the rendering load
additional_render_products = []
for i in range(args.num_additional_render_products):
    rp = rep.create.render_product("/OmniverseKit_Persp", (1280, 720), name=f"AdditionalRenderProduct_{i}")
    additional_render_products.append(rp)
if additional_render_products:
    print(
        f"[FrameDelay] Created {len(additional_render_products)} additional render products: {additional_render_products}"
    )

# NOTE: Capture on play needs to stay True (default value) to get annotator data every update
rep.orchestrator.set_capture_on_play(True)

# Set `hydraEngine/waitIdle` and `renderer/waitIdle` to True during runtime (ideally set from the command line)
if args.waitidle:
    print(f"[FrameDelay] Setting `hydraEngine/waitIdle` and `renderer/waitIdle` to True")
    carb.settings.get_settings().set("/app/hydraEngine/waitIdle", True)
    carb.settings.get_settings().set("/app/renderer/waitIdle", True)

cube = stage.DefinePrim(f"/World/Cube", "Cube")
add_update_semantics(cube, "my_cube")
UsdGeom.Xformable(cube).AddTranslateOp().Set((0, 0, 0))
print(f"[FrameDelay] Cube's initial location set to: {cube.GetAttribute('xformOp:translate').Get()}")


# Create a camera and render product to look at the cube from a top view, use a 3d bbox annotator to track the cube bb location
camera = rep.create.camera(position=(0, 0, 10), look_at=(0, 0, 0))
render_product = rep.create.render_product(camera, (512, 512))
bbox3d_annot = rep.annotators.get("bounding_box_3d")
bbox3d_annot.attach(render_product)
bbox_data = bbox3d_annot.get_data()

# Access the current frame number from the post process dispatcher or the ReferenceTime annotator
dispatcher = og.get_node_by_path("/Render/PostProcess/SDGPipeline/PostProcessDispatcher")
frame_no = dispatcher.get_attribute("outputs:referenceTimeNumerator").get()
# ref_time_annot = rep.annotators.get("ReferenceTime")
# ref_time_annot.attach(render_product)
# frame_no = ref_time_annot.get_data()["referenceTimeNumerator"]

# Frame number warmup, stop once the frame number data is available
print(f"[FrameDelay] Starting warmup until the frame number is available")
for i in range(10):
    simulation_app.update()
    frame_no = dispatcher.get_attribute("outputs:referenceTimeNumerator").get()
    print(f"\tstep={i}; frame_no={frame_no};")
    if frame_no > 0:
        print(f"\t\t Warmup finished (frame_no={frame_no} > 0).")
        break

# Timeline needs to run to get annotator data
print(f"[FrameDelay] Starting timeline")
timeline = omni.timeline.get_timeline_interface()
timeline.play()

# Number of cube mobements to test
num_movements = 20
step_size = 1 / num_movements
cube_locations = [(0, 0, step_size * i) for i in range(num_movements)]
num_delay_frames = []

for i, loc in enumerate(cube_locations):
    # Set the cube to a new location in stage
    print(f"[FrameDelay] location idx={i}; new cube location={loc}")
    cube.GetAttribute("xformOp:translate").Set(loc)
    target_height = loc[2]

    # Update the app until the annotator bounding box height matches the target height
    for j in range(10):
        simulation_app.update()
        frame_no = dispatcher.get_attribute("outputs:referenceTimeNumerator").get()
        bbox_data = bbox3d_annot.get_data()["data"]

        # The first frames the annotator data might still be empty
        if len(bbox_data) > 0:
            # Get the height location of the cube's bounding box from the annotator data
            transform = bbox_data[0]["transform"]
            annot_height = transform[3][2]
            print(
                f"\tstep={j}; frame_no={frame_no}; annot_height={annot_height:.4f}; target_height={target_height:.4f};"
            )
            if np.isclose(annot_height, target_height, atol=0.001):
                # NOTE: one app update is needed to generate annotator data, hence a delay counts as num app updates - 1
                print(f"\t\t Annotator data in sync with stage after a delay of {j} frames.")
                num_delay_frames.append(j)
                break
        else:
            f"\tstep={j}; frame_no={frame_no}; no annotator data yet."

# Stats
print(f"[FrameDelay] Stats (num app updates to sync, from writing the USD attributes until annotator access):")
print(f"\tnum_delay_frames: {num_delay_frames}")
print(f"\tmin num_delay_frames: {min(num_delay_frames)}")
print(f"\tmax num_delay_frames: {max(num_delay_frames)}")
print(f"\tmean num_delay_frames: {np.mean(num_delay_frames)}")

# Check the first delay
if num_delay_frames[0] != EXPECTED_FIRST_DELAY:
    raise ValueError(f"The first delay is {num_delay_frames[0]}, from the expected: {EXPECTED_FIRST_DELAY}.")

# Check the following delays
for i, x in enumerate(num_delay_frames[1:], start=1):
    if x != EXPECTED_FOLLOWING_DELAYS:
        raise ValueError(
            f"The delay at index {i} is {x}, all non-first delays are extected to be: {EXPECTED_FOLLOWING_DELAYS}."
        )

simulation_app.close()
