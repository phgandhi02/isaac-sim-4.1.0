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
        from omni.physxcct.ogn.OgnCharacterControllerDatabase import OgnCharacterControllerDatabase
        test_file_name = "OgnCharacterControllerTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):  # pragma: no cover
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_physx_cct_OgnCharacterController")
        database = OgnCharacterControllerDatabase(test_node)
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 1)

        def _attr_error(attribute: og.Attribute, usd_test: bool) -> str:  # pragma no cover
            test_type = "USD Load" if usd_test else "Database Access"
            return f"{node_type_name} {test_type} Test - {attribute.get_name()} value error"


        self.assertTrue(test_node.get_attribute_exists("inputs:activate"))
        attribute = test_node.get_attribute("inputs:activate")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.activate
        database.inputs.activate = db_value

        self.assertTrue(test_node.get_attribute_exists("inputs:capsulePath"))
        attribute = test_node.get_attribute("inputs:capsulePath")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.capsulePath
        database.inputs.capsulePath = db_value
        expected_value = ""
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:deactivate"))
        attribute = test_node.get_attribute("inputs:deactivate")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.deactivate
        database.inputs.deactivate = db_value

        self.assertTrue(test_node.get_attribute_exists("inputs:gravity"))
        attribute = test_node.get_attribute("inputs:gravity")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.gravity
        database.inputs.gravity = db_value
        expected_value = True
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:setupControls"))
        attribute = test_node.get_attribute("inputs:setupControls")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.setupControls
        database.inputs.setupControls = db_value
        expected_value = "Auto"
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:speed"))
        attribute = test_node.get_attribute("inputs:speed")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.speed
        database.inputs.speed = db_value
        expected_value = 500
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("outputs:done"))
        attribute = test_node.get_attribute("outputs:done")
        self.assertTrue(attribute.is_valid())
        db_value = database.outputs.done
        database.outputs.done = db_value
        temp_setting = database.inputs._setting_locked
        database.inputs._testing_sample_value = True
        database.outputs._testing_sample_value = True
        database.inputs._setting_locked = temp_setting
        self.assertTrue(database.inputs._testing_sample_value)
        self.assertTrue(database.outputs._testing_sample_value)
