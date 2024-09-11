# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

"""Python module for numpy<->ocs2 compatibility."""

# python
import os

# Conveniences to other module directories via relative paths
OCS2_EXT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../"))
"""Path to the extension source directory."""

OCS2_DATA_DIR = os.path.join(OCS2_EXT_DIR, "data")
"""Path to the extension data directory."""
