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
        from omni.isaac.wheeled_robots.ogn.OgnHolonomicControllerDatabase import OgnHolonomicControllerDatabase
        test_file_name = "OgnHolonomicControllerTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):  # pragma: no cover
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_isaac_wheeled_robots_HolonomicController")
        database = OgnHolonomicControllerDatabase(test_node)
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 2)

        def _attr_error(attribute: og.Attribute, usd_test: bool) -> str:  # pragma no cover
            test_type = "USD Load" if usd_test else "Database Access"
            return f"{node_type_name} {test_type} Test - {attribute.get_name()} value error"


        self.assertTrue(test_node.get_attribute_exists("inputs:angularGain"))
        attribute = test_node.get_attribute("inputs:angularGain")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.angularGain
        database.inputs.angularGain = db_value
        expected_value = 1
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:execIn"))
        attribute = test_node.get_attribute("inputs:execIn")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.execIn
        database.inputs.execIn = db_value

        self.assertTrue(test_node.get_attribute_exists("inputs:inputVelocity"))
        attribute = test_node.get_attribute("inputs:inputVelocity")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.inputVelocity
        database.inputs.inputVelocity = db_value
        expected_value = [0.0, 0.0, 0.0]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:linearGain"))
        attribute = test_node.get_attribute("inputs:linearGain")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.linearGain
        database.inputs.linearGain = db_value
        expected_value = 1
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:mecanumAngles"))
        attribute = test_node.get_attribute("inputs:mecanumAngles")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.mecanumAngles
        expected_value = []
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:upAxis"))
        attribute = test_node.get_attribute("inputs:upAxis")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.upAxis
        database.inputs.upAxis = db_value
        expected_value = [0.0, 0.0, 1.0]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:wheelAxis"))
        attribute = test_node.get_attribute("inputs:wheelAxis")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.wheelAxis
        database.inputs.wheelAxis = db_value
        expected_value = [1.0, 0.0, 0.0]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:wheelOrientations"))
        attribute = test_node.get_attribute("inputs:wheelOrientations")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.wheelOrientations
        expected_value = []
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:wheelPositions"))
        attribute = test_node.get_attribute("inputs:wheelPositions")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.wheelPositions
        expected_value = []
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:wheelRadius"))
        attribute = test_node.get_attribute("inputs:wheelRadius")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.wheelRadius
        expected_value = []
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("outputs:jointVelocityCommand"))
        attribute = test_node.get_attribute("outputs:jointVelocityCommand")
        self.assertTrue(attribute.is_valid())
        db_value = database.outputs.jointVelocityCommand
        temp_setting = database.inputs._setting_locked
        database.inputs._testing_sample_value = True
        database.inputs._setting_locked = temp_setting
        self.assertTrue(database.inputs._testing_sample_value)
