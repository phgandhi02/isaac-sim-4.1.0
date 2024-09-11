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
        from omni.isaac.ros2_bridge.ogn.OgnROS2QoSProfileDatabase import OgnROS2QoSProfileDatabase
        test_file_name = "OgnROS2QoSProfileTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):  # pragma: no cover
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_isaac_ros2_bridge_ROS2QoSProfile")
        database = OgnROS2QoSProfileDatabase(test_node)
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 1)

        def _attr_error(attribute: og.Attribute, usd_test: bool) -> str:  # pragma no cover
            test_type = "USD Load" if usd_test else "Database Access"
            return f"{node_type_name} {test_type} Test - {attribute.get_name()} value error"


        self.assertTrue(test_node.get_attribute_exists("inputs:createProfile"))
        attribute = test_node.get_attribute("inputs:createProfile")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.createProfile
        database.inputs.createProfile = db_value
        expected_value = "Default for publishers/subscribers"
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:deadline"))
        attribute = test_node.get_attribute("inputs:deadline")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.deadline
        database.inputs.deadline = db_value
        expected_value = 0.0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:depth"))
        attribute = test_node.get_attribute("inputs:depth")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.depth
        database.inputs.depth = db_value
        expected_value = 10
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:durability"))
        attribute = test_node.get_attribute("inputs:durability")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.durability
        database.inputs.durability = db_value
        expected_value = "volatile"
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:history"))
        attribute = test_node.get_attribute("inputs:history")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.history
        database.inputs.history = db_value
        expected_value = "keepLast"
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:leaseDuration"))
        attribute = test_node.get_attribute("inputs:leaseDuration")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.leaseDuration
        database.inputs.leaseDuration = db_value
        expected_value = 0.0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:lifespan"))
        attribute = test_node.get_attribute("inputs:lifespan")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.lifespan
        database.inputs.lifespan = db_value
        expected_value = 0.0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:liveliness"))
        attribute = test_node.get_attribute("inputs:liveliness")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.liveliness
        database.inputs.liveliness = db_value
        expected_value = "systemDefault"
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:reliability"))
        attribute = test_node.get_attribute("inputs:reliability")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.reliability
        database.inputs.reliability = db_value
        expected_value = "reliable"
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("outputs:qosProfile"))
        attribute = test_node.get_attribute("outputs:qosProfile")
        self.assertTrue(attribute.is_valid())
        db_value = database.outputs.qosProfile
        database.outputs.qosProfile = db_value
        temp_setting = database.inputs._setting_locked
        database.inputs._testing_sample_value = True
        database.outputs._testing_sample_value = True
        database.inputs._setting_locked = temp_setting
        self.assertTrue(database.inputs._testing_sample_value)
        self.assertTrue(database.outputs._testing_sample_value)
