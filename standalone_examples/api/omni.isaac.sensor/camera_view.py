# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import os

import numpy as np
import omni.replicator.core as rep
import torch
from omni.isaac.core import World
from omni.isaac.core.objects import VisualCuboid
from omni.isaac.sensor import CameraView
from PIL import Image

my_world = World(stage_units_in_meters=1.0)

for i in range(2):
    my_world.scene.add(
        VisualCuboid(
            prim_path=f"/new_cube_{i}",
            name=f"cube_{i}",
            position=np.array([0, i * 0.5, 0.2]),
            scale=np.array([0.1, 0.1, 0.1]),
            size=1.0,
            color=np.array([0, 0, 255]),
        )
    )

camera_01 = rep.create.camera(position=(1, 1, 1), look_at=(0, 0, 0))
camera_02 = rep.create.camera(position=(0, 1, 1), look_at=(0, 0, 0))
camera_03 = rep.create.camera(position=(1, 1, 0), look_at=(0, 0, 0))
camera_04 = rep.create.camera(position=(1, 0, 1), look_at=(0, 0, 0))

my_world.scene.add_default_ground_plane()
my_world.reset()

camera_view = CameraView(
    name="camera_prim_view", prim_paths_expr="/Replicator/Camera_Xform*/Camera", output_annotators=["rgb", "depth"]
)
# Run an app update to make sure the camera view has data
simulation_app.update()

# Pre-allocate memory for using the out argument (float32)
rgb_np_tiled_out = np.zeros((*camera_view.tiled_resolution, 3), dtype=np.float32)
rgb_tiled_torch_out = torch.zeros((*camera_view.tiled_resolution, 3), device="cuda", dtype=torch.float32)
batched_rgb_shape = (len(camera_view.prims), *camera_view.camera_resolution, 3)
rgb_batched_out = torch.zeros(batched_rgb_shape, device="cuda", dtype=torch.float32)

depth_np_tiled_out = np.zeros(camera_view.tiled_resolution, dtype=np.float32)
depth_tiled_torch_out = torch.zeros(camera_view.tiled_resolution, device="cuda", dtype=torch.float32)
depth_batched_shape = (len(camera_view.prims), *camera_view.camera_resolution, 1)
depth_batched_out = torch.zeros(depth_batched_shape, device="cuda", dtype=torch.float32)

out_dir = "output_camera_view"
os.makedirs(out_dir, exist_ok=True)
os.makedirs(f"{out_dir}/tiled", exist_ok=True)
os.makedirs(f"{out_dir}/batched", exist_ok=True)

for i in range(2):
    print(f" ** Step {i} ** ")
    my_world.step(render=True)

    #### RGB
    ## numpy
    rgb_tiled_np = camera_view.get_rgb_tiled(device="cpu")
    print(f"rgb_tiled_np.shape: {rgb_tiled_np.shape}, type: {type(rgb_tiled_np)}, dtype: {rgb_tiled_np.dtype}")
    rgb_tiled_np_uint8 = (rgb_tiled_np * 255).astype(np.uint8)
    print(
        f"rgb_tiled_uint8.shape: {rgb_tiled_np_uint8.shape}, type: {type(rgb_tiled_np_uint8)}, dtype: {rgb_tiled_np_uint8.dtype}"
    )
    rgb_tiled_img = Image.fromarray(rgb_tiled_np_uint8)
    rgb_tiled_img.save(f"{out_dir}/tiled/{str(i).zfill(3)}_rgb_tiled_np.png")
    # out argument
    camera_view.get_rgb_tiled(out=rgb_np_tiled_out, device="cpu")
    print(
        f"rgb_np_tiled_out.shape: {rgb_np_tiled_out.shape}, type: {type(rgb_np_tiled_out)}, dtype: {rgb_np_tiled_out.dtype}"
    )
    rgb_np_tiled_out_uint8 = (rgb_np_tiled_out * 255).astype(np.uint8)
    print(
        f"rgb_np_tiled_out_uint8.shape: {rgb_np_tiled_out_uint8.shape}, type: {type(rgb_np_tiled_out_uint8)}, dtype: {rgb_np_tiled_out_uint8.dtype}"
    )
    rgb_np_tiled_out_img = Image.fromarray(rgb_np_tiled_out_uint8)
    rgb_np_tiled_out_img.save(f"{out_dir}/tiled/{str(i).zfill(3)}_rgb_tiled_np_out.png")

    ## torch
    rgb_tiled_torch = camera_view.get_rgb_tiled(device="cuda")
    print(
        f"rgb_tiled_torch.shape: {rgb_tiled_torch.shape}, type: {type(rgb_tiled_torch)}, dtype: {rgb_tiled_torch.dtype}"
    )
    rgb_tiled_torch_uint8 = (rgb_tiled_torch * 255).to(dtype=torch.uint8)
    print(
        f"rgb_tiled_torch_uint8.shape: {rgb_tiled_torch_uint8.shape}, type: {type(rgb_tiled_torch_uint8)}, dtype: {rgb_tiled_torch_uint8.dtype}"
    )
    rgb_tiled_torch_img = Image.fromarray(rgb_tiled_torch_uint8.cpu().numpy())
    rgb_tiled_torch_img.save(f"{out_dir}/tiled/{str(i).zfill(3)}_rgb_tiled_torch.png")
    # out argument
    camera_view.get_rgb_tiled(out=rgb_tiled_torch_out, device="cuda")
    print(
        f"rgb_tiled_torch_out.shape: {rgb_tiled_torch_out.shape}, type: {type(rgb_tiled_torch_out)}, dtype: {rgb_tiled_torch_out.dtype}"
    )
    rgb_tiled_torch_out_uint8 = (rgb_tiled_torch_out * 255).to(dtype=torch.uint8)
    print(
        f"rgb_tiled_torch_out_uint8.shape: {rgb_tiled_torch_out_uint8.shape}, type: {type(rgb_tiled_torch_out_uint8)}, dtype: {rgb_tiled_torch_out_uint8.dtype}"
    )
    rgb_tiled_torch_out_img = Image.fromarray(rgb_tiled_torch_out_uint8.cpu().numpy())
    rgb_tiled_torch_out_img.save(f"{out_dir}/tiled/{str(i).zfill(3)}_rgb_tiled_torch_out.png")

    ## batched
    rgb_batched = camera_view.get_rgb()
    print(f"rgb_batched.shape: {rgb_batched.shape}, type: {type(rgb_batched)}, dtype: {rgb_batched.dtype}")
    for camera_id in range(rgb_batched.shape[0]):
        rgb_batched_uint8 = (rgb_batched[camera_id] * 255).to(dtype=torch.uint8)
        print(
            f"camera_id={camera_id}: rgb_batched_uint8.shape: {rgb_batched_uint8.shape}, type: {type(rgb_batched_uint8)}, dtype: {rgb_batched_uint8.dtype}"
        )
        rgb_batched_img = Image.fromarray(rgb_batched_uint8.cpu().numpy())
        rgb_batched_img.save(f"{out_dir}/batched/{str(i).zfill(3)}_rgb_batched_{camera_id}.png")
    # out argument
    camera_view.get_rgb(out=rgb_batched_out)
    print(
        f"rgb_batched_out.shape: {rgb_batched_out.shape}, type: {type(rgb_batched_out)}, dtype: {rgb_batched_out.dtype}"
    )
    for camera_id in range(rgb_batched_out.shape[0]):
        rgb_batched_out_uint8 = (rgb_batched_out[camera_id] * 255).to(dtype=torch.uint8)
        print(
            f"camera_id={camera_id}: rgb_batched_out_uint8.shape: {rgb_batched_out_uint8.shape}, type: {type(rgb_batched_out_uint8)}, dtype: {rgb_batched_out_uint8.dtype}"
        )
        rgb_batched_out_img = Image.fromarray(rgb_batched_out_uint8.cpu().numpy())
        rgb_batched_out_img.save(f"{out_dir}/batched/{str(i).zfill(3)}_rgb_batched_out_{camera_id}.png")

    #### Depth
    ## numpy
    depth_tiled_np = camera_view.get_depth_tiled(device="cpu")
    print(f"depth_tiled_np.shape: {depth_tiled_np.shape}, type: {type(depth_tiled_np)}, dtype: {depth_tiled_np.dtype}")
    depth_tiled_np_uint8 = (depth_tiled_np * 255).astype(np.uint8)
    depth_tiled_img = Image.fromarray(depth_tiled_np_uint8, mode="L")
    depth_tiled_img.save(f"{out_dir}/tiled/{str(i).zfill(3)}_depth_tiled_np.png")
    # out argument
    camera_view.get_depth_tiled(out=depth_np_tiled_out, device="cpu")
    print(
        f"depth_np_tiled_out.shape: {depth_np_tiled_out.shape}, type: {type(depth_np_tiled_out)}, dtype: {depth_np_tiled_out.dtype}"
    )
    depth_np_tiled_out_uint8 = (depth_np_tiled_out * 255).astype(np.uint8)
    depth_np_tiled_out_img = Image.fromarray(depth_np_tiled_out_uint8, mode="L")
    depth_np_tiled_out_img.save(f"{out_dir}/tiled/{str(i).zfill(3)}_depth_tiled_np_out.png")

    ## torch
    depth_tiled_torch = camera_view.get_depth_tiled(device="cuda")
    print(
        f"depth_tiled_torch.shape: {depth_tiled_torch.shape}, type: {type(depth_tiled_torch)}, dtype: {depth_tiled_torch.dtype}"
    )
    depth_tiled_torch_uint8 = (depth_tiled_torch * 255).to(dtype=torch.uint8)
    depth_tiled_torch_img = Image.fromarray(depth_tiled_torch_uint8.cpu().numpy(), mode="L")
    depth_tiled_torch_img.save(f"{out_dir}/tiled/{str(i).zfill(3)}_depth_tiled_torch.png")
    # out argument
    camera_view.get_depth_tiled(out=depth_tiled_torch_out, device="cuda")
    print(
        f"depth_tiled_torch_out.shape: {depth_tiled_torch_out.shape}, type: {type(depth_tiled_torch_out)}, dtype: {depth_tiled_torch_out.dtype}"
    )
    depth_tiled_torch_out_uint8 = (depth_tiled_torch_out * 255).to(dtype=torch.uint8)
    depth_tiled_torch_out_img = Image.fromarray(depth_tiled_torch_out_uint8.cpu().numpy(), mode="L")
    depth_tiled_torch_out_img.save(f"{out_dir}/tiled/{str(i).zfill(3)}_depth_tiled_torch_out.png")

    ## batched
    depth_batched = camera_view.get_depth()
    print(f"depth_batched.shape: {depth_batched.shape}, type: {type(depth_batched)}, dtype: {depth_batched.dtype}")
    for camera_id in range(depth_batched.shape[0]):
        depth_batched_uint8 = (depth_batched[camera_id] * 255).squeeze().to(dtype=torch.uint8)
        depth_batched_img = Image.fromarray(depth_batched_uint8.cpu().numpy(), mode="L")
        depth_batched_img.save(f"{out_dir}/batched/{str(i).zfill(3)}_depth_batched_{camera_id}.png")
    # out argument
    camera_view.get_depth(out=depth_batched_out)
    print(
        f"depth_batched_out.shape: {depth_batched_out.shape}, type: {type(depth_batched_out)}, dtype: {depth_batched_out.dtype}"
    )
    for camera_id in range(depth_batched_out.shape[0]):
        depth_batched_out_uint8 = (depth_batched_out[camera_id] * 255).squeeze().to(dtype=torch.uint8)
        depth_batched_out_img = Image.fromarray(depth_batched_out_uint8.cpu().numpy(), mode="L")
        depth_batched_out_img.save(f"{out_dir}/batched/{str(i).zfill(3)}_depth_batched_out_{camera_id}.png")

    # API
    raw_data_cpu = camera_view.get_data(device="cpu")
    print(f"len(raw_data_cpu): {len(raw_data_cpu)}, type: {type(raw_data_cpu)}, dtype: {raw_data_cpu.dtype}")
    raw_data_cuda = camera_view.get_data(device="cuda")
    print(f"len(raw_data_cuda): {len(raw_data_cuda)}, type: {type(raw_data_cuda)}, dtype: {raw_data_cuda.dtype}")
    print(f"camera_view.get_local_poses(camera_axes='ros'): {camera_view.get_local_poses(camera_axes='ros')}")
    print(f"camera_view.get_local_poses(camera_axes='usd'): {camera_view.get_local_poses(camera_axes='usd')}")
    print(f"camera_view.get_local_poses(camera_axes='world'): {camera_view.get_local_poses(camera_axes='world')}")
    print(f"camera_view.get_world_poses(): {camera_view.get_world_poses()}")

    print(f"camera_view.get_focal_lengths(): {camera_view.get_focal_lengths()}")
    print(f"camera_view.get_focus_distances(): {camera_view.get_focus_distances()}")
    print(f"camera_view.get_lens_apertures(): {camera_view.get_lens_apertures()}")
    print(f"camera_view.get_horizontal_apertures(): {camera_view.get_horizontal_apertures()}")
    print(f"camera_view.get_vertical_apertures(): {camera_view.get_vertical_apertures()}")
    print(f"camera_view.get_projection_types(): {camera_view.get_projection_types()}")
    print(f"camera_view.get_projection_modes(): {camera_view.get_projection_modes()}")
    print(f"camera_view.get_stereo_roles(): {camera_view.get_stereo_roles()}")
    print(f"camera_view.get_shutter_properties(): {camera_view.get_shutter_properties()}")

    print(
        f"camera_view.set_shutter_properties(): {camera_view.set_shutter_properties(camera_view.get_shutter_properties())}"
    )

    print(f"camera_view.get_focus_distances(): {camera_view.get_focus_distances()}")

    simulation_app.update()

simulation_app.close()
