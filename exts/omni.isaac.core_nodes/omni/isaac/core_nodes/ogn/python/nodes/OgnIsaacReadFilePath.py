# Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from os.path import exists

import omni


class OgnIsaacReadFilePath:
    """
    look for file at path given, and return its contents
    """

    @staticmethod
    def compute(db) -> bool:

        # Empty input:
        db.outputs.fileContents = ""
        if len(db.inputs.path) == 0:
            db.log_warn("Empty input path, returning empty string.")
            return False
        elif not exists(db.inputs.path):
            db.log_warn(f"Could not find file at {db.inputs.path}, returning empty string.")
            return False
        else:
            with open(db.inputs.path, "r") as f:
                db.outputs.fileContents = f.read()
