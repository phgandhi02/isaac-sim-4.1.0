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
        from omni.isaac.wheeled_robots.ogn.OgnStanleyControlPIDDatabase import OgnStanleyControlPIDDatabase
        test_file_name = "OgnStanleyControlPIDTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):  # pragma: no cover
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_isaac_wheeled_robots_StanleyControlPID")
        database = OgnStanleyControlPIDDatabase(test_node)
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 1)

        def _attr_error(attribute: og.Attribute, usd_test: bool) -> str:  # pragma no cover
            test_type = "USD Load" if usd_test else "Database Access"
            return f"{node_type_name} {test_type} Test - {attribute.get_name()} value error"


        self.assertTrue(test_node.get_attribute_exists("inputs:currentOrientation"))
        attribute = test_node.get_attribute("inputs:currentOrientation")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.currentOrientation
        database.inputs.currentOrientation = db_value
        expected_value = [0.0, 0.0, 0.0, 0.0]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:currentPosition"))
        attribute = test_node.get_attribute("inputs:currentPosition")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.currentPosition
        database.inputs.currentPosition = db_value
        expected_value = [0.0, 0.0, 0.0]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:currentSpeed"))
        attribute = test_node.get_attribute("inputs:currentSpeed")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.currentSpeed
        database.inputs.currentSpeed = db_value
        expected_value = [0.0, 0.0, 0.0]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:drawPath"))
        attribute = test_node.get_attribute("inputs:drawPath")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.drawPath
        database.inputs.drawPath = db_value
        expected_value = False
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:execIn"))
        attribute = test_node.get_attribute("inputs:execIn")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.execIn
        database.inputs.execIn = db_value

        self.assertTrue(test_node.get_attribute_exists("inputs:gains"))
        attribute = test_node.get_attribute("inputs:gains")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.gains
        database.inputs.gains = db_value
        expected_value = [0.5, 0.1, 0.0872665]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:maxVelocity"))
        attribute = test_node.get_attribute("inputs:maxVelocity")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.maxVelocity
        database.inputs.maxVelocity = db_value
        expected_value = 1.5
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:pathArrays"))
        attribute = test_node.get_attribute("inputs:pathArrays")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.pathArrays
        expected_value = []
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:reachedGoal"))
        attribute = test_node.get_attribute("inputs:reachedGoal")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.reachedGoal
        expected_value = [False, False]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:step"))
        attribute = test_node.get_attribute("inputs:step")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.step
        database.inputs.step = db_value
        expected_value = 0.16666666667
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:target"))
        attribute = test_node.get_attribute("inputs:target")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.target
        database.inputs.target = db_value
        expected_value = [0, 0, 0]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:targetChanged"))
        attribute = test_node.get_attribute("inputs:targetChanged")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.targetChanged
        database.inputs.targetChanged = db_value
        expected_value = False
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:thresholds"))
        attribute = test_node.get_attribute("inputs:thresholds")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.thresholds
        database.inputs.thresholds = db_value
        expected_value = [0.1, 0.1]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:wheelBase"))
        attribute = test_node.get_attribute("inputs:wheelBase")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.wheelBase
        database.inputs.wheelBase = db_value
        expected_value = 0.4132
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("outputs:angularVelocity"))
        attribute = test_node.get_attribute("outputs:angularVelocity")
        self.assertTrue(attribute.is_valid())
        db_value = database.outputs.angularVelocity
        database.outputs.angularVelocity = db_value

        self.assertTrue(test_node.get_attribute_exists("outputs:execOut"))
        attribute = test_node.get_attribute("outputs:execOut")
        self.assertTrue(attribute.is_valid())
        db_value = database.outputs.execOut
        database.outputs.execOut = db_value

        self.assertTrue(test_node.get_attribute_exists("outputs:linearVelocity"))
        attribute = test_node.get_attribute("outputs:linearVelocity")
        self.assertTrue(attribute.is_valid())
        db_value = database.outputs.linearVelocity
        database.outputs.linearVelocity = db_value
        temp_setting = database.inputs._setting_locked
        database.inputs._testing_sample_value = True
        database.outputs._testing_sample_value = True
        database.inputs._setting_locked = temp_setting
        self.assertTrue(database.inputs._testing_sample_value)
        self.assertTrue(database.outputs._testing_sample_value)
