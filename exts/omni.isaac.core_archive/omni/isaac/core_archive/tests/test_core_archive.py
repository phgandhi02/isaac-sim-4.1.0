# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.kit.test


class TestPipArchive(omni.kit.test.AsyncTestCase):
    # import all packages to make sure dependencies were not missed
    async def test_import_all(self):
        import construct
        import cycler
        import gunicorn
        import jinja2
        import kiwisolver
        import llvmlite
        import markupsafe
        import matplotlib
        import nest_asyncio
        import numba
        import nvsmi
        import osqp
        import packaging
        import pint
        import plotly
        import pyparsing
        import pyperclip
        import qdldl
        import quaternion
        import selenium
        import tornado
        import webbot

        self.assertIsNotNone(quaternion)
        self.assertIsNotNone(numba)
        self.assertIsNotNone(webbot)
        self.assertIsNotNone(selenium)
        self.assertIsNotNone(construct)
        self.assertIsNotNone(llvmlite)
        self.assertIsNotNone(nest_asyncio)
        self.assertIsNotNone(jinja2)
        self.assertIsNotNone(markupsafe)
        self.assertIsNotNone(matplotlib)
        self.assertIsNotNone(pyparsing)
        self.assertIsNotNone(cycler)
        self.assertIsNotNone(kiwisolver)
        self.assertIsNotNone(pint)
        self.assertIsNotNone(packaging)
        self.assertIsNotNone(gunicorn)
        self.assertIsNotNone(osqp)
        self.assertIsNotNone(qdldl)
        self.assertIsNotNone(nvsmi)
        self.assertIsNotNone(tornado)
        self.assertIsNotNone(plotly)
        self.assertIsNotNone(pyperclip)
