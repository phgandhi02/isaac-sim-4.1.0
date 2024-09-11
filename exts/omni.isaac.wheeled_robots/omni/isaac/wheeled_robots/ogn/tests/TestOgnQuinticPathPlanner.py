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
        from omni.isaac.wheeled_robots.ogn.OgnQuinticPathPlannerDatabase import OgnQuinticPathPlannerDatabase
        test_file_name = "OgnQuinticPathPlannerTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):  # pragma: no cover
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_isaac_wheeled_robots_QuinticPathPlanner")
        database = OgnQuinticPathPlannerDatabase(test_node)
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

        self.assertTrue(test_node.get_attribute_exists("inputs:execIn"))
        attribute = test_node.get_attribute("inputs:execIn")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.execIn
        database.inputs.execIn = db_value

        self.assertTrue(test_node.get_attribute_exists("inputs:goalAccel"))
        attribute = test_node.get_attribute("inputs:goalAccel")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.goalAccel
        database.inputs.goalAccel = db_value
        expected_value = 0.02
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:goalVelocity"))
        attribute = test_node.get_attribute("inputs:goalVelocity")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.goalVelocity
        database.inputs.goalVelocity = db_value
        expected_value = 0.5
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:initialAccel"))
        attribute = test_node.get_attribute("inputs:initialAccel")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.initialAccel
        database.inputs.initialAccel = db_value
        expected_value = 0.02
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:initialVelocity"))
        attribute = test_node.get_attribute("inputs:initialVelocity")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.initialVelocity
        database.inputs.initialVelocity = db_value
        expected_value = 0.5
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:maxAccel"))
        attribute = test_node.get_attribute("inputs:maxAccel")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.maxAccel
        database.inputs.maxAccel = db_value
        expected_value = 1.5
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:maxJerk"))
        attribute = test_node.get_attribute("inputs:maxJerk")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.maxJerk
        database.inputs.maxJerk = db_value
        expected_value = 0.3
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

        self.assertTrue(test_node.get_attribute_exists("inputs:targetOrientation"))
        attribute = test_node.get_attribute("inputs:targetOrientation")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.targetOrientation
        database.inputs.targetOrientation = db_value
        expected_value = [0.0, 0.0, 0.0, 0.0]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:targetPosition"))
        attribute = test_node.get_attribute("inputs:targetPosition")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.targetPosition
        database.inputs.targetPosition = db_value
        expected_value = [0.0, 0.0, 0.0]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("outputs:execOut"))
        attribute = test_node.get_attribute("outputs:execOut")
        self.assertTrue(attribute.is_valid())
        db_value = database.outputs.execOut
        database.outputs.execOut = db_value

        self.assertTrue(test_node.get_attribute_exists("outputs:pathArrays"))
        attribute = test_node.get_attribute("outputs:pathArrays")
        self.assertTrue(attribute.is_valid())
        db_value = database.outputs.pathArrays

        self.assertTrue(test_node.get_attribute_exists("outputs:target"))
        attribute = test_node.get_attribute("outputs:target")
        self.assertTrue(attribute.is_valid())
        db_value = database.outputs.target
        database.outputs.target = db_value

        self.assertTrue(test_node.get_attribute_exists("outputs:targetChanged"))
        attribute = test_node.get_attribute("outputs:targetChanged")
        self.assertTrue(attribute.is_valid())
        db_value = database.outputs.targetChanged
        database.outputs.targetChanged = db_value
        temp_setting = database.inputs._setting_locked
        database.inputs._testing_sample_value = True
        database.outputs._testing_sample_value = True
        database.inputs._setting_locked = temp_setting
        self.assertTrue(database.inputs._testing_sample_value)
        self.assertTrue(database.outputs._testing_sample_value)
