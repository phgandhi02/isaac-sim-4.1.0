# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import ctypes

import carb
from omni.syntheticdata._syntheticdata import acquire_syntheticdata_interface


def object_id_to_prim_path(object_id):
    """Given an ObjectId get a Prim Path

    Args:
        object_id (int): object id, like from a RTX sensor return

    Returns:
        prim path string
    """
    return acquire_syntheticdata_interface().get_uri_from_instance_segmentation_id(int(object_id))


class OgnIsaacPrintRTXSensorInfo:
    """
    Print raw RTX sensor data to console. Example of using omni.sensors Python bindings in OmniGraph node.
    """

    @staticmethod
    def compute(db) -> bool:
        """read a pointer and print data from it assuming it is Rtx"""
        if not db.inputs.dataPtr:
            carb.log_warn("invalid data input to OgnIsaacPrintRTXSensorInfo")
            return True

        import omni.sensors.nv.common.bindings._common as gmo

        PyCapsule_Destructor = ctypes.CFUNCTYPE(None, ctypes.py_object)
        PyCapsule_New = ctypes.pythonapi.PyCapsule_New
        PyCapsule_New.restype = ctypes.py_object
        PyCapsule_New.argtypes = (ctypes.c_void_p, ctypes.c_char_p, PyCapsule_Destructor)
        gmo_data = gmo.getModelOutputFromBuffer(
            PyCapsule_New(ctypes.cast(db.inputs.dataPtr, ctypes.c_void_p), None, PyCapsule_Destructor(0))
        )

        print("-------------------- NEW FRAME ------------------------------------------")
        print("-------------------- gmo:")
        print(f"frameId:     {gmo_data.frameId}")
        print(f"timestampNs: {gmo_data.timestampNs}")
        print(f"numElements: {gmo_data.numElements}")
        print(f"auxType: {gmo_data.auxType}")
        print(f"Return 0:")
        print(f"    timeOffsetNs: {gmo_data.timeOffSetNs[0]}")
        print(f"    azimuth:      {gmo_data.x[0]}")
        print(f"    elevation:    {gmo_data.y[0]}")
        print(f"    range:        {gmo_data.z[0]}")
        print(f"    intensity:    {gmo_data.scalar[0]}")
        print(f"Return {gmo_data.numElements - 1}:")
        print(f"    timeOffsetNs: {gmo_data.timeOffSetNs[gmo_data.numElements - 1]}")
        print(f"    azimuth:      {gmo_data.x[gmo_data.numElements - 1]}")
        print(f"    elevation:    {gmo_data.y[gmo_data.numElements - 1]}")
        print(f"    range:        {gmo_data.z[gmo_data.numElements - 1]}")
        print(f"    intensity:    {gmo_data.scalar[gmo_data.numElements - 1]}")

        # NOTE: Material mapping only valid for Lidar data currently
        if gmo_data.auxType == gmo.AuxType.LIDAR:
            print(f"Prim <-> Material mapping:")
            material_mapping = {}
            for i in range(gmo_data.numElements):
                objId = gmo_data.objId[i]
                matId = gmo_data.matId[i]
                if gmo_data.z[i] > 0.0 and gmo_data.scalar[i] > 0.0:
                    if objId not in material_mapping:
                        material_mapping[objId] = matId

            for obj in material_mapping:
                prim_path = object_id_to_prim_path(obj)
                print(f"objectId {obj} with prim path {prim_path} has material ID {material_mapping[obj]}.")
        return True
