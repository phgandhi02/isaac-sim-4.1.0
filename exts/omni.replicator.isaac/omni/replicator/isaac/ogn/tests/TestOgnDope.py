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
        from omni.replicator.isaac.ogn.OgnDopeDatabase import OgnDopeDatabase
        test_file_name = "OgnDopeTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):  # pragma: no cover
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_replicator_isaac_Dope")
        database = OgnDopeDatabase(test_node)
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 1)

        def _attr_error(attribute: og.Attribute, usd_test: bool) -> str:  # pragma no cover
            test_type = "USD Load" if usd_test else "Database Access"
            return f"{node_type_name} {test_type} Test - {attribute.get_name()} value error"


        self.assertTrue(test_node.get_attribute_exists("inputs:boundingBox3d"))
        attribute = test_node.get_attribute("inputs:boundingBox3d")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.boundingBox3d
        expected_value = []
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:cameraFisheyeMaxFOV"))
        attribute = test_node.get_attribute("inputs:cameraFisheyeMaxFOV")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.cameraFisheyeMaxFOV
        database.inputs.cameraFisheyeMaxFOV = db_value
        expected_value = 0.0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:cameraFisheyeNominalHeight"))
        attribute = test_node.get_attribute("inputs:cameraFisheyeNominalHeight")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.cameraFisheyeNominalHeight
        database.inputs.cameraFisheyeNominalHeight = db_value
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:cameraFisheyeNominalWidth"))
        attribute = test_node.get_attribute("inputs:cameraFisheyeNominalWidth")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.cameraFisheyeNominalWidth
        database.inputs.cameraFisheyeNominalWidth = db_value
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:cameraFisheyeOpticalCentre"))
        attribute = test_node.get_attribute("inputs:cameraFisheyeOpticalCentre")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.cameraFisheyeOpticalCentre
        database.inputs.cameraFisheyeOpticalCentre = db_value
        expected_value = [0.0, 0.0]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:cameraFisheyePolynomial"))
        attribute = test_node.get_attribute("inputs:cameraFisheyePolynomial")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.cameraFisheyePolynomial
        expected_value = []
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:cameraModel"))
        attribute = test_node.get_attribute("inputs:cameraModel")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.cameraModel
        database.inputs.cameraModel = db_value
        expected_value = ""
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:cameraNearFar"))
        attribute = test_node.get_attribute("inputs:cameraNearFar")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.cameraNearFar
        database.inputs.cameraNearFar = db_value
        expected_value = [0.0, 0.0]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:cameraProjection"))
        attribute = test_node.get_attribute("inputs:cameraProjection")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.cameraProjection
        database.inputs.cameraProjection = db_value
        expected_value = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:cameraRotation"))
        attribute = test_node.get_attribute("inputs:cameraRotation")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.cameraRotation
        database.inputs.cameraRotation = db_value
        expected_value = [0.0, 0.0, 0.0]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:cameraViewTransform"))
        attribute = test_node.get_attribute("inputs:cameraViewTransform")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.cameraViewTransform
        database.inputs.cameraViewTransform = db_value
        expected_value = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:exec"))
        attribute = test_node.get_attribute("inputs:exec")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.exec
        database.inputs.exec = db_value

        self.assertTrue(test_node.get_attribute_exists("inputs:focalLength"))
        attribute = test_node.get_attribute("inputs:focalLength")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.focalLength
        database.inputs.focalLength = db_value
        expected_value = 0.0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:height"))
        attribute = test_node.get_attribute("inputs:height")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.height
        database.inputs.height = db_value
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:horizontalAperture"))
        attribute = test_node.get_attribute("inputs:horizontalAperture")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.horizontalAperture
        database.inputs.horizontalAperture = db_value
        expected_value = 0.0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:occlusion"))
        attribute = test_node.get_attribute("inputs:occlusion")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.occlusion
        expected_value = []
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:sdIMInstanceSemanticMap"))
        attribute = test_node.get_attribute("inputs:sdIMInstanceSemanticMap")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.sdIMInstanceSemanticMap
        expected_value = []
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:sdIMMaxSemanticHierarchyDepth"))
        attribute = test_node.get_attribute("inputs:sdIMMaxSemanticHierarchyDepth")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.sdIMMaxSemanticHierarchyDepth
        database.inputs.sdIMMaxSemanticHierarchyDepth = db_value
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:sdIMMinSemanticIndex"))
        attribute = test_node.get_attribute("inputs:sdIMMinSemanticIndex")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.sdIMMinSemanticIndex
        database.inputs.sdIMMinSemanticIndex = db_value
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:sdIMNumSemanticTokens"))
        attribute = test_node.get_attribute("inputs:sdIMNumSemanticTokens")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.sdIMNumSemanticTokens
        database.inputs.sdIMNumSemanticTokens = db_value
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:sdIMNumSemantics"))
        attribute = test_node.get_attribute("inputs:sdIMNumSemantics")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.sdIMNumSemantics
        database.inputs.sdIMNumSemantics = db_value
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:sdIMSemanticTokenMap"))
        attribute = test_node.get_attribute("inputs:sdIMSemanticTokenMap")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.sdIMSemanticTokenMap
        expected_value = []
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:semanticTypes"))
        attribute = test_node.get_attribute("inputs:semanticTypes")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.semanticTypes
        expected_value = ['class']
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:width"))
        attribute = test_node.get_attribute("inputs:width")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.width
        database.inputs.width = db_value
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("outputs:bufferSize"))
        attribute = test_node.get_attribute("outputs:bufferSize")
        self.assertTrue(attribute.is_valid())
        db_value = database.outputs.bufferSize
        database.outputs.bufferSize = db_value

        self.assertTrue(test_node.get_attribute_exists("outputs:data"))
        attribute = test_node.get_attribute("outputs:data")
        self.assertTrue(attribute.is_valid())
        db_value = database.outputs.data

        self.assertTrue(test_node.get_attribute_exists("outputs:exec"))
        attribute = test_node.get_attribute("outputs:exec")
        self.assertTrue(attribute.is_valid())
        db_value = database.outputs.exec
        database.outputs.exec = db_value

        self.assertTrue(test_node.get_attribute_exists("outputs:height"))
        attribute = test_node.get_attribute("outputs:height")
        self.assertTrue(attribute.is_valid())
        db_value = database.outputs.height
        database.outputs.height = db_value

        self.assertTrue(test_node.get_attribute_exists("outputs:idToLabels"))
        attribute = test_node.get_attribute("outputs:idToLabels")
        self.assertTrue(attribute.is_valid())
        db_value = database.outputs.idToLabels
        database.outputs.idToLabels = db_value

        self.assertTrue(test_node.get_attribute_exists("outputs:width"))
        attribute = test_node.get_attribute("outputs:width")
        self.assertTrue(attribute.is_valid())
        db_value = database.outputs.width
        database.outputs.width = db_value
        temp_setting = database.inputs._setting_locked
        database.inputs._testing_sample_value = True
        database.outputs._testing_sample_value = True
        database.inputs._setting_locked = temp_setting
        self.assertTrue(database.inputs._testing_sample_value)
        self.assertTrue(database.outputs._testing_sample_value)
