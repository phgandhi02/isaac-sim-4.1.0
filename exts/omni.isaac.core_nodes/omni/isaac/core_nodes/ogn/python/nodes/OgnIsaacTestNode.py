# Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


class OgnIsaacTestNode:
    """
    Test Isaac Sim Node
    """

    @staticmethod
    def compute(db) -> bool:
        """Dummy Compute Function"""
        # copy input to output
        db.outputs.output = db.inputs.input
        return True
