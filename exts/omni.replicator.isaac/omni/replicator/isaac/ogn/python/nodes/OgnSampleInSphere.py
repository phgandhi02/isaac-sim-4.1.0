# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


import numpy as np
import omni.graph.core as og
import omni.usd
from pxr import Sdf, UsdGeom


class OgnSampleInSphere:
    @staticmethod
    def compute(db) -> bool:
        prim_paths = db.inputs.prims
        if len(prim_paths) == 0:
            db.outputs.execOut = og.ExecutionAttributeState.DISABLED
            return False

        stage = omni.usd.get_context().get_stage()
        prims = [stage.GetPrimAtPath(str(path)) for path in prim_paths]

        radius = db.inputs.radius

        try:
            for prim in prims:
                if not UsdGeom.Xformable(prim):
                    prim_type = prim.GetTypeName()
                    raise ValueError(
                        f"Expected prim at {prim.GetPath()} to be an Xformable prim but got type {prim_type}"
                    )
                if not prim.HasAttribute("xformOp:translate"):
                    UsdGeom.Xformable(prim).AddTranslateOp()
            if radius <= 0:
                raise ValueError(f"Radius must be positive, got {radius}")

        except Exception as error:
            db.log_error(str(error))
            db.outputs.execOut = og.ExecutionAttributeState.DISABLED
            return False

        samples = []
        for _ in range(len(prims)):
            # Generate a random direction by spherical coordinates (phi, theta)
            phi = np.random.uniform(0, 2 * np.pi)
            # Sample costheta to ensure uniform distribution of points on the sphere (surface is proportional to sin(theta))
            costheta = np.random.uniform(-1, 1)
            theta = np.arccos(costheta)

            # Scale the radius uniformly within the sphere, applying the cube root to a random value
            # to account for volume's cubic growth with radius (r^3), ensuring spatial uniformity.
            r = radius * (np.random.random() ** (1 / 3))

            # Convert from spherical to Cartesian coordinates
            x = r * np.sin(theta) * np.cos(phi)
            y = r * np.sin(theta) * np.sin(phi)
            z = r * np.cos(theta)

            samples.append((x, y, z))

        with Sdf.ChangeBlock():
            for prim, sample in zip(prims, samples):
                prim.GetAttribute("xformOp:translate").Set(sample)

        db.outputs.execOut = og.ExecutionAttributeState.ENABLED
        return True
