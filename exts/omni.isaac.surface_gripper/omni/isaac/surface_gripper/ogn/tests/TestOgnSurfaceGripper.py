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
        from omni.isaac.surface_gripper.ogn.OgnSurfaceGripperDatabase import OgnSurfaceGripperDatabase
        test_file_name = "OgnSurfaceGripperTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):  # pragma: no cover
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_isaac_surface_gripper_SurfaceGripper")
        database = OgnSurfaceGripperDatabase(test_node)
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 1)

        def _attr_error(attribute: og.Attribute, usd_test: bool) -> str:  # pragma no cover
            test_type = "USD Load" if usd_test else "Database Access"
            return f"{node_type_name} {test_type} Test - {attribute.get_name()} value error"


        self.assertTrue(test_node.get_attribute_exists("inputs:BendAngle"))
        attribute = test_node.get_attribute("inputs:BendAngle")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.BendAngle
        database.inputs.BendAngle = db_value
        expected_value = 7.5
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:Close"))
        attribute = test_node.get_attribute("inputs:Close")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.Close
        database.inputs.Close = db_value

        self.assertTrue(test_node.get_attribute_exists("inputs:Damping"))
        attribute = test_node.get_attribute("inputs:Damping")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.Damping
        database.inputs.Damping = db_value
        expected_value = 1000.0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:Delta"))
        attribute = test_node.get_attribute("inputs:Delta")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.Delta
        database.inputs.Delta = db_value
        expected_value = 0.0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:DisableGravity"))
        attribute = test_node.get_attribute("inputs:DisableGravity")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.DisableGravity
        database.inputs.DisableGravity = db_value
        expected_value = True
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:ForceLimit"))
        attribute = test_node.get_attribute("inputs:ForceLimit")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.ForceLimit
        database.inputs.ForceLimit = db_value
        expected_value = 1000000.0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:GripPosition"))
        attribute = test_node.get_attribute("inputs:GripPosition")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.GripPosition

        self.assertTrue(test_node.get_attribute_exists("inputs:GripThreshold"))
        attribute = test_node.get_attribute("inputs:GripThreshold")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.GripThreshold
        database.inputs.GripThreshold = db_value
        expected_value = 0.01
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:Open"))
        attribute = test_node.get_attribute("inputs:Open")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.Open
        database.inputs.Open = db_value

        self.assertTrue(test_node.get_attribute_exists("inputs:ParentRigidBody"))
        attribute = test_node.get_attribute("inputs:ParentRigidBody")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.ParentRigidBody

        self.assertTrue(test_node.get_attribute_exists("inputs:RetryClose"))
        attribute = test_node.get_attribute("inputs:RetryClose")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.RetryClose
        database.inputs.RetryClose = db_value
        expected_value = False
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:Stiffness"))
        attribute = test_node.get_attribute("inputs:Stiffness")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.Stiffness
        database.inputs.Stiffness = db_value
        expected_value = 10000.0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:TorqueLimit"))
        attribute = test_node.get_attribute("inputs:TorqueLimit")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.TorqueLimit
        database.inputs.TorqueLimit = db_value
        expected_value = 1000000.0
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

        self.assertTrue(test_node.get_attribute_exists("inputs:onStep"))
        attribute = test_node.get_attribute("inputs:onStep")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.onStep
        database.inputs.onStep = db_value

        self.assertTrue(test_node.get_attribute_exists("outputs:Closed"))
        attribute = test_node.get_attribute("outputs:Closed")
        self.assertTrue(attribute.is_valid())
        db_value = database.outputs.Closed
        database.outputs.Closed = db_value

        self.assertTrue(test_node.get_attribute_exists("outputs:GripBroken"))
        attribute = test_node.get_attribute("outputs:GripBroken")
        self.assertTrue(attribute.is_valid())
        db_value = database.outputs.GripBroken
        database.outputs.GripBroken = db_value
        temp_setting = database.inputs._setting_locked
        database.inputs._testing_sample_value = True
        database.outputs._testing_sample_value = True
        database.inputs._setting_locked = temp_setting
        self.assertTrue(database.inputs._testing_sample_value)
        self.assertTrue(database.outputs._testing_sample_value)
