# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import os
import unittest
from pathlib import Path

import carb
import numpy as np
import omni.kit
import omni.replicator.core as rep
import omni.usd
import torch
from omni.isaac.core.utils.stage import create_new_stage_async
from omni.replicator.isaac.scripts.writers.pytorch_listener import PytorchListener
from PIL import Image


class TestOgnCustomReplicatorRandomizer(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        await omni.kit.app.get_app().next_update_async()
        await create_new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self):
        await omni.kit.app.get_app().next_update_async()
        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()

    async def test_custom_randomizer(self):
        import math
        import random
        from itertools import chain

        import omni.replicator.core as rep
        import omni.usd
        from pxr import UsdGeom

        # Generate a random 3D point on the surface of a sphere of a given radius.
        def random_point_on_sphere(radius):
            # Generate a random direction by spherical coordinates (phi, theta)
            phi = random.uniform(0, 2 * math.pi)
            # Sample costheta to ensure uniform distribution of points on the sphere (surface is proportional to sin(theta))
            costheta = random.uniform(-1, 1)
            theta = math.acos(costheta)

            # Convert from spherical to Cartesian coordinates
            x = radius * math.sin(theta) * math.cos(phi)
            y = radius * math.sin(theta) * math.sin(phi)
            z = radius * math.cos(theta)

            return x, y, z

        # Generate a random 3D point within a sphere of a given radius, ensuring a uniform distribution throughout the volume.
        def random_point_in_sphere(radius):
            # Generate a random direction by spherical coordinates (phi, theta)
            phi = random.uniform(0, 2 * math.pi)
            # Sample costheta to ensure uniform distribution of points on the sphere (surface is proportional to sin(theta))
            costheta = random.uniform(-1, 1)
            theta = math.acos(costheta)

            # Scale the radius uniformly within the sphere, applying the cube root to a random value
            # to account for volume's cubic growth with radius (r^3), ensuring spatial uniformity.
            r = radius * (random.random() ** (1 / 3))

            # Convert from spherical to Cartesian coordinates
            x = r * math.sin(theta) * math.cos(phi)
            y = r * math.sin(theta) * math.sin(phi)
            z = r * math.cos(theta)

            return x, y, z

        # Generate a random 3D point between two spheres, ensuring a uniform distribution throughout the volume.
        def random_point_between_spheres(radius1, radius2):
            # Ensure radius1 < radius2
            if radius1 > radius2:
                radius1, radius2 = radius2, radius1

            # Generate a random direction by spherical coordinates (phi, theta)
            phi = random.uniform(0, 2 * math.pi)
            # Sample costheta to ensure uniform distribution of points on the sphere (surface is proportional to sin(theta))
            costheta = random.uniform(-1, 1)
            theta = math.acos(costheta)

            # Uniformly distribute points between two spheres by weighting the radius to match volume growth (r^3),
            # ensuring spatial uniformity by taking the cube root of a value between the radii cubed.
            r = (random.uniform(radius1**3, radius2**3)) ** (1 / 3.0)

            # Convert from spherical to Cartesian coordinates
            x = r * math.sin(theta) * math.cos(phi)
            y = r * math.sin(theta) * math.sin(phi)
            z = r * math.cos(theta)

            return x, y, z

        stage = omni.usd.get_context().get_stage()
        prim_count = 500
        prim_scale = 0.1
        rad_in = 0.5
        rad_on = 1.5
        rad_bet1 = 2.5
        rad_bet2 = 3.5

        # Create the default prims
        on_sphere_prims = [stage.DefinePrim(f"/World/sphere_{i}", "Sphere") for i in range(prim_count)]
        in_sphere_prims = [stage.DefinePrim(f"/World/cube_{i}", "Cube") for i in range(prim_count)]
        between_spheres_prims = [stage.DefinePrim(f"/World/cylinder_{i}", "Cylinder") for i in range(prim_count)]

        # Add xformOps and scale to the prims
        for prim in chain(on_sphere_prims, in_sphere_prims, between_spheres_prims):
            if not prim.HasAttribute("xformOp:translate"):
                UsdGeom.Xformable(prim).AddTranslateOp()
            if not prim.HasAttribute("xformOp:scale"):
                UsdGeom.Xformable(prim).AddScaleOp()
            if not prim.HasAttribute("xformOp:rotateXYZ"):
                UsdGeom.Xformable(prim).AddRotateXYZOp()
            prim.GetAttribute("xformOp:scale").Set((prim_scale, prim_scale, prim_scale))

        # Randomize the prims
        for _ in range(10):
            for in_sphere_prim in in_sphere_prims:
                rand_rot = (random.uniform(0, 360), random.uniform(0, 360), random.uniform(0, 360))
                in_sphere_prim.GetAttribute("xformOp:rotateXYZ").Set(rand_rot)
                rand_loc = random_point_in_sphere(rad_in)
                in_sphere_prim.GetAttribute("xformOp:translate").Set(rand_loc)

            for on_sphere_prim in on_sphere_prims:
                rand_rot = (random.uniform(0, 360), random.uniform(0, 360), random.uniform(0, 360))
                on_sphere_prim.GetAttribute("xformOp:rotateXYZ").Set(rand_rot)
                rand_loc = random_point_on_sphere(rad_on)
                on_sphere_prim.GetAttribute("xformOp:translate").Set(rand_loc)

            for between_spheres_prim in between_spheres_prims:
                rand_rot = (random.uniform(0, 360), random.uniform(0, 360), random.uniform(0, 360))
                between_spheres_prim.GetAttribute("xformOp:rotateXYZ").Set(rand_rot)
                rand_loc = random_point_between_spheres(rad_bet1, rad_bet2)
                between_spheres_prim.GetAttribute("xformOp:translate").Set(rand_loc)

    async def test_custom_replicator_randomizer_graph(self):
        import omni.replicator.core as rep
        from omni.replicator.core.scripts.utils import ReplicatorItem, ReplicatorWrapper, create_node, set_target_prims

        @ReplicatorWrapper
        def on_sphere(
            radius: float = 1.0,
            input_prims: ReplicatorItem | list[str] | None = None,
        ) -> ReplicatorItem:

            node = create_node("omni.replicator.isaac.OgnSampleOnSphere", radius=radius)
            if input_prims:
                set_target_prims(node, "inputs:prims", input_prims)
            return node

        @ReplicatorWrapper
        def in_sphere(
            radius: float = 1.0,
            input_prims: ReplicatorItem | list[str] | None = None,
        ) -> ReplicatorItem:

            node = create_node("omni.replicator.isaac.OgnSampleInSphere", radius=radius)
            if input_prims:
                set_target_prims(node, "inputs:prims", input_prims)
            return node

        @ReplicatorWrapper
        def between_spheres(
            radius1: float = 0.5,
            radius2: float = 1.0,
            input_prims: ReplicatorItem | list[str] | None = None,
        ) -> ReplicatorItem:

            node = create_node("omni.replicator.isaac.OgnSampleBetweenSpheres", radius1=radius1, radius2=radius2)
            if input_prims:
                set_target_prims(node, "inputs:prims", input_prims)
            return node

        prim_count = 50
        prim_scale = 0.1
        rad_in = 0.5
        rad_on = 1.5
        rad_bet1 = 2.5
        rad_bet2 = 3.5

        # Create the default prims
        sphere = rep.create.sphere(count=prim_count, scale=prim_scale)
        cube = rep.create.cube(count=prim_count, scale=prim_scale)
        cylinder = rep.create.cylinder(count=prim_count, scale=prim_scale)

        # Create the randomization graph
        with rep.trigger.on_frame():
            with sphere:
                rep.randomizer.rotation()
                in_sphere(rad_in)

            with cube:
                rep.randomizer.rotation()
                on_sphere(rad_on)

            with cylinder:
                rep.randomizer.rotation()
                between_spheres(rad_bet1, rad_bet2)

        # Preview and call the graph
        await rep.orchestrator.preview_async()
        for _ in range(10):
            await rep.orchestrator.step_async()
