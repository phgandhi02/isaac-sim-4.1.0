import os
import omni.kit.test
import omni.graph.core as og
import omni.graph.core.tests as ogts
from omni.graph.core.tests.omnigraph_test_utils import _TestGraphAndNode
from omni.graph.core.tests.omnigraph_test_utils import _test_clear_scene
from omni.graph.core.tests.omnigraph_test_utils import _test_setup_scene
from omni.graph.core.tests.omnigraph_test_utils import _test_verify_scene


class TestOgn(ogts.OmniGraphTestCase):

    async def test_data_access(self):
        from omni.physxcct.ogn.OgnControlsSettingsDatabase import OgnControlsSettingsDatabase
        test_file_name = "OgnControlsSettingsTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):  # pragma: no cover
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_physx_cct_OgnControlsSettings")
        database = OgnControlsSettingsDatabase(test_node)
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 2)

        def _attr_error(attribute: og.Attribute, usd_test: bool) -> str:  # pragma no cover
            test_type = "USD Load" if usd_test else "Database Access"
            return f"{node_type_name} {test_type} Test - {attribute.get_name()} value error"


        self.assertTrue(test_node.get_attribute_exists("inputs:backward"))
        attribute = test_node.get_attribute("inputs:backward")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.backward
        database.inputs.backward = db_value
        expected_value = "S"
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:down"))
        attribute = test_node.get_attribute("inputs:down")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.down
        database.inputs.down = db_value
        expected_value = "Q"
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:forward"))
        attribute = test_node.get_attribute("inputs:forward")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.forward
        database.inputs.forward = db_value
        expected_value = "W"
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:gamepadSensitivity"))
        attribute = test_node.get_attribute("inputs:gamepadSensitivity")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.gamepadSensitivity
        database.inputs.gamepadSensitivity = db_value
        expected_value = 25
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:left"))
        attribute = test_node.get_attribute("inputs:left")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.left
        database.inputs.left = db_value
        expected_value = "A"
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:mouseSensitivity"))
        attribute = test_node.get_attribute("inputs:mouseSensitivity")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.mouseSensitivity
        database.inputs.mouseSensitivity = db_value
        expected_value = 25
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:right"))
        attribute = test_node.get_attribute("inputs:right")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.right
        database.inputs.right = db_value
        expected_value = "D"
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:up"))
        attribute = test_node.get_attribute("inputs:up")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.up
        database.inputs.up = db_value
        expected_value = "E"
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("outputs_control_settings"))
        attribute = test_node.get_attribute("outputs_control_settings")
        self.assertTrue(attribute.is_valid())
        db_value = database.outputs.control_settings
        temp_setting = database.inputs._setting_locked
        database.inputs._testing_sample_value = True
        database.inputs._setting_locked = temp_setting
        self.assertTrue(database.inputs._testing_sample_value)
