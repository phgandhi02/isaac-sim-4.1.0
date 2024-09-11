# Copyright (c) 2024, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

# isort: off
# fmt: off

import omni.structuredlog
import json
import omni.log

# The JSON schema for these events
# The $ref statements for these events have been expanded because python's
# standard library json module can't expand $ref statements.
# If you want to distribute the jsonref package, you can use $ref statements.
schema = """
{
    "generated": "This was generated from omni.physx.structuredlog.schema.",
    "anyOf": [
        {
            "$ref": "#/definitions/events/com.nvidia.kit.physx.log_omni_physx_zerog_activated"
        },
        {
            "$ref": "#/definitions/events/com.nvidia.kit.physx.log_omni_physx_stage_apis"
        }
    ],
    "$schema": "http://json-schema.org/draft-07/schema#",
    "schemaMeta": {
        "clientName": "omni.physx",
        "schemaVersion": "1.0",
        "eventPrefix": "com.nvidia.kit.physx",
        "definitionVersion": "1.0",
        "description": "omni.physx schema to track and improve the product."
    },
    "definitions": {
        "events": {
            "com.nvidia.kit.physx.log_omni_physx_zerog_activated": {
                "eventMeta": {
                    "service": "telemetry",
                    "privacy": {
                        "category": "usage",
                        "description": "This information is crucial to track omni.physx.zerogravity usage amongst customers."
                    },
                    "omniverseFlags": []
                },
                "type": "object",
                "additionalProperties": false,
                "properties": {},
                "description": "Tracks whether omni.physx.zerogravity is activated (i.e. zg mode is entered)."
            },
            "com.nvidia.kit.physx.log_omni_physx_stage_apis": {
                "eventMeta": {
                    "service": "telemetry",
                    "privacy": {
                        "category": "usage",
                        "description": "This information is crucial to track omni.physx API usage amongst customers."
                    },
                    "omniverseFlags": []
                },
                "type": "object",
                "additionalProperties": false,
                "required": [
                    "apis_found_on_prims"
                ],
                "properties": {
                    "apis_found_on_prims": {
                        "type": "string",
                        "description": "the physics APIs found in the USD stage at stage load",
                        "examples": [
                            "PhysicsArticulationRootAPI;PhysxVehicleContextAPI",
                            "PhysxDeformableBodyAPI;PhysxParticleSystem;PhysicsCollisionAPI;PhysicsRevoluteJointAPI",
                            "PhysicsDistanceJoint"
                        ]
                    }
                },
                "description": "Tracks which omni.physx physics APIs are found at stage load through FSD."
            }
        }
    },
    "description": "omni.physx schema to track and improve the product."
}
"""

# the telemetry events dictionary we can use to send events
events = None
try:
    schema = json.loads(schema)
    events = omni.structuredlog.register_schema(schema)
except Exception as e:
    omni.log.error("failed to register the schema: " + str(type(e)) + " " + str(e))

# These are wrappers for the send functions that you can call to send telemetry events
def log_omni_physx_zerog_activated_send_event():
    """
    Helper function to send the com.nvidia.kit.physx.log_omni_physx_zerog_activated event.
    Tracks whether omni.physx.zerogravity is activated (i.e. zg mode
    is entered).
    Returns: no return value.
    """
    if events is None:
        return

    try:
        omni.structuredlog.send_event(events["log_omni_physx_zerog_activated"], {})
    except Exception as e:
        omni.log.error("failed to send telemetry event log_omni_physx_zerog_activated " + str(type(e)) + " " + str(e))


def log_omni_physx_stage_apis_send_event(apis_found_on_prims):
    """
    Helper function to send the com.nvidia.kit.physx.log_omni_physx_stage_apis event.
    Tracks which omni.physx physics APIs are found at stage load
    through FSD.
    Args:
        apis_found_on_prims: the physics APIs found in the USD stage at stage load
    Returns: no return value.
    """
    if events is None:
        return

    try:
        omni.structuredlog.send_event(events["log_omni_physx_stage_apis"], {"apis_found_on_prims": apis_found_on_prims})
    except Exception as e:
        omni.log.error("failed to send telemetry event log_omni_physx_stage_apis " + str(type(e)) + " " + str(e))
