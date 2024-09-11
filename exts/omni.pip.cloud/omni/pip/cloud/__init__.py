# Copyright (c) 2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


import omni.ext


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id):
        # Force reload of newer typing extensions provided by this extensions prebundle
        from importlib import reload

        import typing_extensions

        reload(typing_extensions)
        pass

    def on_shutdown(self):
        pass
