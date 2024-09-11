# Copyright (c) 2018-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import json
import unittest

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestImportWizard(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        await omni.kit.app.get_app().next_update_async()
        pass

    # After running each test
    async def tearDown(self):
        await omni.kit.app.get_app().next_update_async()
        pass

    # Run for 60 frames and make sure there were no errors loading
    async def test_wizard_wrapper(self):
        for frame in range(60):
            await omni.kit.app.get_app().next_update_async()
        pass

    # test links in the wizard to make sure they all still exist
    # TODO: should pass once docs are public
    @unittest.expectedFailure
    async def test_docs_links(self):
        from urllib.error import URLError
        from urllib.request import urlopen

        def is_website_online(url):
            try:
                res = urlopen(url, timeout=1.0)
                print(f"testing link url", url)
                return res.status == 200
            except URLError:
                print("URL failed", url)
                return False

        def extract_key(data_dict, key, value_array=[]):
            if isinstance(data_dict, dict):
                for k, v in data_dict.items():
                    if k == key:
                        if v != "":
                            value_array.append(v)
                    extract_key(v, key, value_array)
            return value_array

        EXTENSION_FOLDER_PATH = omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)
        with open(EXTENSION_FOLDER_PATH + "/data/pipeline.json", "r") as file:
            data = json.load(file)

        links_all = []
        links_all = extract_key(data, "Documentation Link", links_all)
        links_all = extract_key(data, "API Link", links_all)
        links_all = extract_key(data, "Examples Link", links_all)
        links_unique = set(links_all)
        for url in links_unique:
            self.assertTrue(is_website_online(url))

    # test the pipeline.json file to make sure it is valid
    async def test_pipeline_json(self):
        EXTENSION_FOLDER_PATH = omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)
        with open(EXTENSION_FOLDER_PATH + "/data/available_tools.json", "r") as file:
            available_tools_file = json.load(file)
        available_tools = available_tools_file["Available Tools"]
        with open(EXTENSION_FOLDER_PATH + "/data/pipeline.json", "r") as file:
            pipeline_data = json.load(file)
        pipeline_entries = list(pipeline_data.keys())

        # make sure all the tools in the available_tools list has an entry in pipeline.json and vice versa
        for tool in available_tools:
            print(f"checking tool {tool}")
            self.assertTrue(tool in pipeline_entries)
        for tool in pipeline_entries:
            print(f"checking tool {tool}")
            self.assertTrue(tool in available_tools)

        # make sure all the entries in pipeline.json has all the fields required
        for tool in pipeline_entries:
            self.assertTrue("App Name" in pipeline_data[tool])
            self.assertTrue("Extension" in pipeline_data[tool])
            self.assertTrue("Window" in pipeline_data[tool])
            self.assertTrue("Summary" in pipeline_data[tool])
            self.assertTrue("Basic Instructions" in pipeline_data[tool])
            self.assertTrue("Advanced Instructions" in pipeline_data[tool])
            self.assertTrue("Menu" in pipeline_data[tool])
            self.assertTrue("Resources" in pipeline_data[tool])
            self.assertTrue("Documentation Link" in pipeline_data[tool]["Resources"])

        # TODO: if there's a list of extensions names and extension folder names, then check that App NAme and Extension are all valid as well

    async def test_questionnaire_json(self):
        EXTENSION_FOLDER_PATH = omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)
        with open(EXTENSION_FOLDER_PATH + "/data/questionnaire.json", "r") as file:
            questionnaire_data = json.load(file)

        with open(EXTENSION_FOLDER_PATH + "/data/available_tools.json", "r") as file:
            available_tools_file = json.load(file)
        available_tools = available_tools_file["Available Tools"]

        # get all the tools listed in the questionnaire, make sure they are in available_tools.json
        tool_list = []
        for qa_keys, qa_values in questionnaire_data.items():
            for a_keys, a_answers in qa_values["a_options"].items():
                if a_answers["tool"] != "":
                    tool_list.append(a_answers["tool"])
        for tool in tool_list:
            print(f"checking tool {tool}")
            self.assertTrue(tool in available_tools)

        # there is one and only one beginning
        qa_keys = list(questionnaire_data.keys())
        self.assertTrue("begin" in qa_keys)
        self.assertEqual(qa_keys.count("begin"), 1)

        # make sure all the question keys in "next_qa" are keys in the questionnaire
        for qa_keys, qa_values in questionnaire_data.items():
            for a_keys, a_answers in qa_values["a_options"].items():
                if a_answers["next_qa"] != "end":
                    self.assertTrue(a_answers["next_qa"] in questionnaire_data.keys())
