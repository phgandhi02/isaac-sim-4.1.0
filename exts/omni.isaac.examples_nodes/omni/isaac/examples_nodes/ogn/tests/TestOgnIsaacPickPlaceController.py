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
        from omni.isaac.examples_nodes.ogn.OgnIsaacPickPlaceControllerDatabase import OgnIsaacPickPlaceControllerDatabase
        test_file_name = "OgnIsaacPickPlaceControllerTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):  # pragma: no cover
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_isaac_examples_nodes_IsaacPickPlaceController")
        database = OgnIsaacPickPlaceControllerDatabase(test_node)
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 1)

        def _attr_error(attribute: og.Attribute, usd_test: bool) -> str:  # pragma no cover
            test_type = "USD Load" if usd_test else "Database Access"
            return f"{node_type_name} {test_type} Test - {attribute.get_name()} value error"


        self.assertTrue(test_node.get_attribute_exists("inputs:endEffectorOffset"))
        attribute = test_node.get_attribute("inputs:endEffectorOffset")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.endEffectorOffset
        database.inputs.endEffectorOffset = db_value
        expected_value = [0.0, 0.0, 0.0]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:eventsDT"))
        attribute = test_node.get_attribute("inputs:eventsDT")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.eventsDT
        expected_value = [0.01, 0.01, 1.0, 0.01, 0.01, 0.01, 0.01, 0.05, 0.01, 0.08]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:execIn"))
        attribute = test_node.get_attribute("inputs:execIn")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.execIn
        database.inputs.execIn = db_value

        self.assertTrue(test_node.get_attribute_exists("inputs:pickingPosition"))
        attribute = test_node.get_attribute("inputs:pickingPosition")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.pickingPosition
        database.inputs.pickingPosition = db_value
        expected_value = [0.25, 0.25, 0.0]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:placingPosition"))
        attribute = test_node.get_attribute("inputs:placingPosition")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.placingPosition
        database.inputs.placingPosition = db_value
        expected_value = [0.25, -0.25, 0.0]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:robotModel"))
        attribute = test_node.get_attribute("inputs:robotModel")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.robotModel
        database.inputs.robotModel = db_value
        expected_value = ""
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:robotPrimPath"))
        attribute = test_node.get_attribute("inputs:robotPrimPath")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.robotPrimPath
        database.inputs.robotPrimPath = db_value
        expected_value = ""
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:usePath"))
        attribute = test_node.get_attribute("inputs:usePath")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.usePath
        database.inputs.usePath = db_value
        expected_value = False
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))
        temp_setting = database.inputs._setting_locked
        database.inputs._testing_sample_value = True
        database.inputs._setting_locked = temp_setting
        self.assertTrue(database.inputs._testing_sample_value)
