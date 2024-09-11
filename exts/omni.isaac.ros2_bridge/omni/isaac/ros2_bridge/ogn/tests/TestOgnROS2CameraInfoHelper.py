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
        from omni.isaac.ros2_bridge.ogn.OgnROS2CameraInfoHelperDatabase import OgnROS2CameraInfoHelperDatabase
        test_file_name = "OgnROS2CameraInfoHelperTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):  # pragma: no cover
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_isaac_ros2_bridge_ROS2CameraInfoHelper")
        database = OgnROS2CameraInfoHelperDatabase(test_node)
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 1)

        def _attr_error(attribute: og.Attribute, usd_test: bool) -> str:  # pragma no cover
            test_type = "USD Load" if usd_test else "Database Access"
            return f"{node_type_name} {test_type} Test - {attribute.get_name()} value error"


        self.assertTrue(test_node.get_attribute_exists("inputs:context"))
        attribute = test_node.get_attribute("inputs:context")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.context
        database.inputs.context = db_value
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:enabled"))
        attribute = test_node.get_attribute("inputs:enabled")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.enabled
        database.inputs.enabled = db_value
        expected_value = True
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:execIn"))
        attribute = test_node.get_attribute("inputs:execIn")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.execIn
        database.inputs.execIn = db_value

        self.assertTrue(test_node.get_attribute_exists("inputs:frameId"))
        attribute = test_node.get_attribute("inputs:frameId")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.frameId
        database.inputs.frameId = db_value
        expected_value = "sim_camera"
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:frameSkipCount"))
        attribute = test_node.get_attribute("inputs:frameSkipCount")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.frameSkipCount
        database.inputs.frameSkipCount = db_value
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:nodeNamespace"))
        attribute = test_node.get_attribute("inputs:nodeNamespace")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.nodeNamespace
        database.inputs.nodeNamespace = db_value
        expected_value = ""
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:qosProfile"))
        attribute = test_node.get_attribute("inputs:qosProfile")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.qosProfile
        database.inputs.qosProfile = db_value
        expected_value = ""
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:queueSize"))
        attribute = test_node.get_attribute("inputs:queueSize")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.queueSize
        database.inputs.queueSize = db_value
        expected_value = 10
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:renderProductPath"))
        attribute = test_node.get_attribute("inputs:renderProductPath")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.renderProductPath
        database.inputs.renderProductPath = db_value
        expected_value = ""
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:resetSimulationTimeOnStop"))
        attribute = test_node.get_attribute("inputs:resetSimulationTimeOnStop")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.resetSimulationTimeOnStop
        database.inputs.resetSimulationTimeOnStop = db_value
        expected_value = False
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:topicName"))
        attribute = test_node.get_attribute("inputs:topicName")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.topicName
        database.inputs.topicName = db_value
        expected_value = "camera_info"
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:useSystemTime"))
        attribute = test_node.get_attribute("inputs:useSystemTime")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.useSystemTime
        database.inputs.useSystemTime = db_value
        expected_value = False
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))
        temp_setting = database.inputs._setting_locked
        database.inputs._testing_sample_value = True
        database.inputs._setting_locked = temp_setting
        self.assertTrue(database.inputs._testing_sample_value)
