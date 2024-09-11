# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import os
import shutil
from datetime import datetime
from pathlib import Path

import omni.kit.app


class TemplateGenerator:
    def __init__(self):
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_extension_id_by_module("omni.isaac.extension_templates")
        self._extension_path = ext_manager.get_extension_path(ext_id)

    def _write_string_to_file(self, file_path, file_string):
        Path(os.path.dirname(file_path)).mkdir(parents=True, exist_ok=True)

        f = open(file_path, "w+")
        f.write(file_string)
        f.close()

    def _copy_directory_contents(self, source_dir, target_dir):
        new_paths = []

        for file_name in os.listdir(source_dir):
            source = os.path.join(source_dir, file_name)
            target = os.path.join(target_dir, file_name)

            if os.path.isfile(source):
                Path(os.path.dirname(target)).mkdir(parents=True, exist_ok=True)
                shutil.copy(source, target)
                new_paths.append(target)
            elif os.path.isdir(source):
                new_paths.extend(
                    self._copy_directory_contents(source, os.path.join(target_dir, os.path.basename(source)))
                )

        return new_paths

    def _replace_keywords(self, replace_dict, file_paths):
        for file_path in file_paths:
            if file_path[-4:] == ".png":
                continue
            template = open(file_path, "r")
            file_string = template.read()

            for k, v in replace_dict.items():
                file_string = file_string.replace(k, v)

            template.close()

            self._write_string_to_file(file_path, file_string)

    def _write_common_data(self, file_path, extension_title, extension_description):
        source_dir = os.path.join(self._extension_path, "template_source_files", "common")
        new_paths = self._copy_directory_contents(source_dir, file_path)

        python_package_name = self._get_python_package_name(extension_title)

        replace_keywords = {
            "{EXTENSION_TITLE}": extension_title,
            "{EXTENSION_DESCRIPTION}": extension_description,
            "{CURRENT_DATE}": datetime.now().strftime("%Y-%m-%d"),
            "{EXTENSION_REPOSITORY}": "",
            "{PYTHON_PACKAGE_NAME}": python_package_name,
        }
        self._replace_keywords(replace_keywords, new_paths)

    def _get_python_package_name(self, extension_name):
        # Convert all special characters in extension_name to underscores to make a valid python package name
        package_name = ""
        for c in extension_name:
            if c.isalnum():
                package_name += c
            else:
                package_name += "_"

        # Add the tag _python to the end of the path to make it more likely to be a unique module name
        package_name += "_python"

        return package_name

    def generate_configuration_tooling_template(self, file_path, extension_title, extension_description):
        self._write_common_data(file_path, extension_title, extension_description)

        python_package_name = self._get_python_package_name(extension_title)

        source_dir = os.path.join(self._extension_path, "template_source_files", "configuration_tooling_workflow")
        target_dir = os.path.join(file_path, python_package_name)

        new_paths = self._copy_directory_contents(source_dir, target_dir)

        replace_keywords = {
            "{EXTENSION_TITLE}": '"' + extension_title + '"',
            "{EXTENSION_DESCRIPTION}": '"' + extension_description + '"',
        }

        self._replace_keywords(replace_keywords, [os.path.join(target_dir, "global_variables.py")])

    def generate_loaded_scenario_template(self, file_path, extension_title, extension_description):
        self._write_common_data(file_path, extension_title, extension_description)

        python_package_name = self._get_python_package_name(extension_title)

        source_dir = os.path.join(self._extension_path, "template_source_files", "loaded_scenario_workflow")
        target_dir = os.path.join(file_path, python_package_name)

        new_paths = self._copy_directory_contents(source_dir, target_dir)

        replace_keywords = {
            "{EXTENSION_TITLE}": '"' + extension_title + '"',
            "{EXTENSION_DESCRIPTION}": '"' + extension_description + '"',
        }

        self._replace_keywords(replace_keywords, [os.path.join(target_dir, "global_variables.py")])

    def generate_scripting_template(self, file_path, extension_title, extension_description):
        self._write_common_data(file_path, extension_title, extension_description)

        python_package_name = self._get_python_package_name(extension_title)

        source_dir = os.path.join(self._extension_path, "template_source_files", "scripting_workflow")
        target_dir = os.path.join(file_path, python_package_name)

        new_paths = self._copy_directory_contents(source_dir, target_dir)

        replace_keywords = {
            "{EXTENSION_TITLE}": '"' + extension_title + '"',
            "{EXTENSION_DESCRIPTION}": '"' + extension_description + '"',
        }

        self._replace_keywords(replace_keywords, [os.path.join(target_dir, "global_variables.py")])

    def generate_component_library_template(self, file_path, extension_title, extension_description):
        self._write_common_data(file_path, extension_title, extension_description)

        python_package_name = self._get_python_package_name(extension_title)

        source_dir = os.path.join(self._extension_path, "template_source_files", "ui_component_library")
        target_dir = os.path.join(file_path, python_package_name)

        new_paths = self._copy_directory_contents(source_dir, target_dir)

        replace_keywords = {
            "{EXTENSION_TITLE}": '"' + extension_title + '"',
            "{EXTENSION_DESCRIPTION}": '"' + extension_description + '"',
        }

        self._replace_keywords(replace_keywords, [os.path.join(target_dir, "global_variables.py")])
