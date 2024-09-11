# Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


import omni
import omni.replicator.core as rep
from omni.isaac.core.utils.prims import set_targets


def set_target_prims(primPath: str, targetPrimPaths: list, inputName: str = "inputs:targetPrim"):
    stage = omni.usd.get_context().get_stage()
    try:
        set_targets(stage.GetPrimAtPath(primPath), inputName, targetPrimPaths)
    except Exception as e:
        print(e, primPath)


def register_node_writer_with_telemetry(*args, **kwargs):
    rep.writers.register_node_writer(*args, **kwargs)
    # Register writer for Replicator telemetry tracking
    if kwargs["name"] not in rep.WriterRegistry._default_writers:
        rep.WriterRegistry._default_writers.append(kwargs["name"])


def register_annotator_from_node_with_telemetry(*args, **kwargs):
    rep.AnnotatorRegistry.register_annotator_from_node(*args, **kwargs)
    # Register annotator for Replicator telemetry tracking
    if kwargs["name"] not in rep.AnnotatorRegistry._default_annotators:
        rep.AnnotatorRegistry._default_annotators.append(kwargs["name"])
