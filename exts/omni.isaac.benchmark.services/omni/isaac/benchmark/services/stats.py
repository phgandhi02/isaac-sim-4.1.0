# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from typing import Dict


class OmniStats:
    def get_stats(self) -> Dict:
        scopes_dict = {}
        for scope in self._scopes:
            stats = self._stats_if.get_stats(scope["scopeId"])
            stats_dict = {}
            for x in stats:
                name = x["name"].replace(" - ", "_")
                name = name.replace(" ", "_")
                stats_dict[name] = x["value"]
            scopes_dict[scope["name"]] = stats_dict
        return scopes_dict
