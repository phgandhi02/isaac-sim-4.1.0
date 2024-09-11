# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import gzip
import os
import shutil

from . import utils

logger = utils.set_up_logging(__name__)


async def convert_chrome_to_tracy(json_path: str, tracy_path: str) -> str:
    """Convert CPU trace JSON file to Tracy format."""
    from omni.kit.profiler.tracy import tracy

    # Convert JSON to Tracy.
    logger.info(f"Converting {json_path} to {tracy_path}")
    tracy.convert_json_to_tracy(json_path, tracy_path)

    # If there was a problem with tracy, just return the original file
    if not os.path.exists(tracy_path):
        logger.warning(f"Unable to create tracy file for {json_path}...")
        return json_path

    # Compress file.
    with open(tracy_path, "rb") as f_in:
        with gzip.open(tracy_path + ".gz", "wb") as f_out:
            shutil.copyfileobj(f_in, f_out)

    return tracy_path + ".gz"
