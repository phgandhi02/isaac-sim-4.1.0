# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add support for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import torch

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from omni.isaac.core.utils.semantics import add_update_semantics, remove_all_semantics
from pxr import Semantics


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestSemantics(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        pass

    async def tearDown(self):
        await omni.kit.app.get_app().next_update_async()
        pass

    async def test_add_update_semantics(self):
        stage = omni.usd.get_context().get_stage()
        prim = stage.DefinePrim("/test", "Xform")
        # make sure we can apply semantics to a new prim
        add_update_semantics(prim, "test_data")
        semantic_api = Semantics.SemanticsAPI.Get(prim, "Semantics")
        self.assertIsNotNone(semantic_api)

        type_attr = semantic_api.GetSemanticTypeAttr()
        data_attr = semantic_api.GetSemanticDataAttr()
        self.assertEqual(type_attr.Get(), "class")
        self.assertEqual(data_attr.Get(), "test_data")

        # make sure we can update existing semantics
        add_update_semantics(prim, "new_test_data")
        self.assertEqual(type_attr.Get(), "class")
        self.assertEqual(data_attr.Get(), "new_test_data")

        # add a second semantic label, first should remain valid
        add_update_semantics(prim, "another_data", "another_class", "1")
        # first should remain valid
        semantic_api = Semantics.SemanticsAPI.Get(prim, "Semantics")
        type_attr = semantic_api.GetSemanticTypeAttr()
        data_attr = semantic_api.GetSemanticDataAttr()
        self.assertIsNotNone(semantic_api)
        self.assertEqual(type_attr.Get(), "class")
        self.assertEqual(data_attr.Get(), "new_test_data")
        # second should exist
        semantic_api_1 = Semantics.SemanticsAPI.Get(prim, "Semantics1")
        self.assertIsNotNone(semantic_api_1)
        type_attr_1 = semantic_api_1.GetSemanticTypeAttr()
        data_attr_1 = semantic_api_1.GetSemanticDataAttr()
        self.assertEqual(type_attr_1.Get(), "another_class")
        self.assertEqual(data_attr_1.Get(), "another_data")

        # we should be able to update this new semantic label
        add_update_semantics(prim, "test_data", "another_class", "1")

        self.assertEqual(type_attr_1.Get(), "another_class")
        self.assertEqual(data_attr_1.Get(), "test_data")

        # should not error with None input
        add_update_semantics(prim, None, None)

        pass

    async def test_remove_semantics(self):
        stage = omni.usd.get_context().get_stage()
        prim = stage.DefinePrim("/test", "Xform")
        nested_prim = stage.DefinePrim("/test/test", "Xform")
        # make sure we can apply semantics to a new prim
        add_update_semantics(prim, "test_data_a")
        add_update_semantics(prim, "test_data_b", suffix="_a")
        semantic_api = Semantics.SemanticsAPI.Get(prim, "Semantics")
        self.assertIsNotNone(semantic_api)
        # Check the first semantic
        type_attr = semantic_api.GetSemanticTypeAttr()
        data_attr = semantic_api.GetSemanticDataAttr()
        self.assertEqual(type_attr.Get(), "class")
        self.assertEqual(data_attr.Get(), "test_data_a")
        # Add semantics to nested
        add_update_semantics(nested_prim, "test_data_nested")
        remove_all_semantics(prim)
        # there should be no semantic properties left
        for prop in prim.GetProperties():
            is_semantic = Semantics.SemanticsAPI.IsSemanticsAPIPath(prop.GetPath())
            self.assertFalse(is_semantic)
        # nested should not be touched and have one semantic property
        is_semantic = Semantics.SemanticsAPI.IsSemanticsAPIPath("/test/test.semantic:Semantics:params:semanticData")
        self.assertTrue(is_semantic)

        add_update_semantics(prim, "test_data_a")
        add_update_semantics(prim, "test_data_b", suffix="_a")
        remove_all_semantics(prim, recursive=True)
        # neither prim should have semantics now
        for prop in prim.GetProperties():
            is_semantic = Semantics.SemanticsAPI.IsSemanticsAPIPath(prop.GetPath())
            self.assertFalse(is_semantic)
        for nested_prim in prim.GetProperties():
            is_semantic = Semantics.SemanticsAPI.IsSemanticsAPIPath(prop.GetPath())
            self.assertFalse(is_semantic)
