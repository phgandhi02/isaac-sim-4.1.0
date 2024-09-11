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
        from omni.physxcct.ogn.OgnSpawnCapsuleDatabase import OgnSpawnCapsuleDatabase
        test_file_name = "OgnSpawnCapsuleTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):  # pragma: no cover
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_physx_cct_OgnSpawnCapsule")
        database = OgnSpawnCapsuleDatabase(test_node)
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 1)

        def _attr_error(attribute: og.Attribute, usd_test: bool) -> str:  # pragma no cover
            test_type = "USD Load" if usd_test else "Database Access"
            return f"{node_type_name} {test_type} Test - {attribute.get_name()} value error"


        self.assertTrue(test_node.get_attribute_exists("inputs:capsuleHeight"))
        attribute = test_node.get_attribute("inputs:capsuleHeight")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.capsuleHeight
        database.inputs.capsuleHeight = db_value
        expected_value = 100
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:capsulePos"))
        attribute = test_node.get_attribute("inputs:capsulePos")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.capsulePos
        database.inputs.capsulePos = db_value
        expected_value = [0, 0, 0]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:capsuleRadius"))
        attribute = test_node.get_attribute("inputs:capsuleRadius")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.capsuleRadius
        database.inputs.capsuleRadius = db_value
        expected_value = 50
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:spawn"))
        attribute = test_node.get_attribute("inputs:spawn")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.spawn
        database.inputs.spawn = db_value

        self.assertTrue(test_node.get_attribute_exists("outputs:done"))
        attribute = test_node.get_attribute("outputs:done")
        self.assertTrue(attribute.is_valid())
        db_value = database.outputs.done
        database.outputs.done = db_value

        self.assertTrue(test_node.get_attribute_exists("outputs:path"))
        attribute = test_node.get_attribute("outputs:path")
        self.assertTrue(attribute.is_valid())
        db_value = database.outputs.path
        database.outputs.path = db_value
        temp_setting = database.inputs._setting_locked
        database.inputs._testing_sample_value = True
        database.outputs._testing_sample_value = True
        database.inputs._setting_locked = temp_setting
        self.assertTrue(database.inputs._testing_sample_value)
        self.assertTrue(database.outputs._testing_sample_value)
