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
        from omni.isaac.core_nodes.ogn.OgnIsaacArticulationStateDatabase import OgnIsaacArticulationStateDatabase
        test_file_name = "OgnIsaacArticulationStateTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):  # pragma: no cover
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_isaac_core_nodes_IsaacArticulationState")
        database = OgnIsaacArticulationStateDatabase(test_node)
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 1)

        def _attr_error(attribute: og.Attribute, usd_test: bool) -> str:  # pragma no cover
            test_type = "USD Load" if usd_test else "Database Access"
            return f"{node_type_name} {test_type} Test - {attribute.get_name()} value error"


        self.assertTrue(test_node.get_attribute_exists("inputs:execIn"))
        attribute = test_node.get_attribute("inputs:execIn")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.execIn
        database.inputs.execIn = db_value

        self.assertTrue(test_node.get_attribute_exists("inputs:jointIndices"))
        attribute = test_node.get_attribute("inputs:jointIndices")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.jointIndices
        expected_value = []
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:jointNames"))
        attribute = test_node.get_attribute("inputs:jointNames")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.jointNames
        expected_value = []
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:robotPath"))
        attribute = test_node.get_attribute("inputs:robotPath")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.robotPath
        database.inputs.robotPath = db_value
        expected_value = ""
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("outputs:jointNames"))
        attribute = test_node.get_attribute("outputs:jointNames")
        self.assertTrue(attribute.is_valid())
        db_value = database.outputs.jointNames

        self.assertTrue(test_node.get_attribute_exists("outputs:jointPositions"))
        attribute = test_node.get_attribute("outputs:jointPositions")
        self.assertTrue(attribute.is_valid())
        db_value = database.outputs.jointPositions

        self.assertTrue(test_node.get_attribute_exists("outputs:jointVelocities"))
        attribute = test_node.get_attribute("outputs:jointVelocities")
        self.assertTrue(attribute.is_valid())
        db_value = database.outputs.jointVelocities

        self.assertTrue(test_node.get_attribute_exists("outputs:measuredJointEfforts"))
        attribute = test_node.get_attribute("outputs:measuredJointEfforts")
        self.assertTrue(attribute.is_valid())
        db_value = database.outputs.measuredJointEfforts

        self.assertTrue(test_node.get_attribute_exists("outputs:measuredJointForces"))
        attribute = test_node.get_attribute("outputs:measuredJointForces")
        self.assertTrue(attribute.is_valid())
        db_value = database.outputs.measuredJointForces

        self.assertTrue(test_node.get_attribute_exists("outputs:measuredJointTorques"))
        attribute = test_node.get_attribute("outputs:measuredJointTorques")
        self.assertTrue(attribute.is_valid())
        db_value = database.outputs.measuredJointTorques
        temp_setting = database.inputs._setting_locked
        database.inputs._testing_sample_value = True
        database.inputs._setting_locked = temp_setting
        self.assertTrue(database.inputs._testing_sample_value)
