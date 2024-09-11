import omni.ext
import omni.physx
import carb

from omni.physxforcefields.bindings import _physxForceFields

from omni.kit.commands import execute
from omni.physx.scripts import utils

from pxr import Usd, UsdGeom, ForceFieldSchema

# to make the tests appear under Test Runner
utils.safe_import_tests("omni.physxforcefields.scripts", "tests")

from . import commands

DEMO_SCENES = "omni.physxforcefields.scripts.samples"


class PhysxForceFieldsExtension(omni.ext.IExt):
    def __init__(self):
        self._instance_name = "ForceField"
        super().__init__()
        return

    def find_largest_instance(self, prim):
        instance_number = 0

        for schema in prim.GetAppliedSchemas():
            instance_name = schema.find(":" + self._instance_name)

            if instance_name >= 0:
                number = int(schema[instance_name + 11:])

                if number > instance_number:
                    instance_number = number

        return instance_number

    def schema_present(self, prim):
        for schema in prim.GetAppliedSchemas():
            instance_name = schema.find("PhysxForceField")

            if instance_name >= 0:
                return True

        return False

    def remove_all_components(self, prim, component):
        for schema in prim.GetAppliedSchemas():
            instance = schema.find(component)

            if instance >= 0:
                instance_position = schema.find(":")

                if instance_position >= 0:
                    instance_name = schema[instance_position + 1:]
                    self.remove_force_field_api(prim, component, instance_name)

    def add_force_field_command(self, usd_prim, component):

        instance_number = self.find_largest_instance(usd_prim) + 1

        # Instance names cannot contain spaces
        instance_name = self._instance_name + str(instance_number)

        if component == "PhysxForceFieldDragAPI":
            execute("ApplyAPISchema", api=ForceFieldSchema.PhysxForceFieldDragAPI, prim=usd_prim,
                    api_prefix="physxForceField", multiple_api_token=instance_name)
        elif component == "PhysxForceFieldLinearAPI":
            execute("ApplyAPISchema", api=ForceFieldSchema.PhysxForceFieldLinearAPI, prim=usd_prim,
                    api_prefix="physxForceField", multiple_api_token=instance_name)
        elif component == "PhysxForceFieldNoiseAPI":
            execute("ApplyAPISchema", api=ForceFieldSchema.PhysxForceFieldNoiseAPI, prim=usd_prim,
                    api_prefix="physxForceField", multiple_api_token=instance_name)
        elif component == "PhysxForceFieldPlanarAPI":
            execute("ApplyAPISchema", api=ForceFieldSchema.PhysxForceFieldPlanarAPI, prim=usd_prim,
                    api_prefix="physxForceField", multiple_api_token=instance_name)
        elif component == "PhysxForceFieldSphericalAPI":
            execute("ApplyAPISchema", api=ForceFieldSchema.PhysxForceFieldSphericalAPI, prim=usd_prim,
                    api_prefix="physxForceField", multiple_api_token=instance_name)
        elif component == "PhysxForceFieldConicalAPI":
            execute("ApplyAPISchema", api=ForceFieldSchema.PhysxForceFieldConicalAPI, prim=usd_prim,
                    api_prefix="physxForceField", multiple_api_token=instance_name)
        elif component == "PhysxForceFieldWindAPI":
            execute("ApplyAPISchema", api=ForceFieldSchema.PhysxForceFieldWindAPI, prim=usd_prim,
                    api_prefix="physxForceField", multiple_api_token=instance_name)
        elif component == "PhysxForceFieldSpinAPI":
            execute("ApplyAPISchema", api=ForceFieldSchema.PhysxForceFieldSpinAPI, prim=usd_prim,
                    api_prefix="physxForceField", multiple_api_token=instance_name)
        elif component == "PhysxForceFieldRingAPI":
            execute("ApplyAPISchema", api=ForceFieldSchema.PhysxForceFieldRingAPI, prim=usd_prim,
                    api_prefix="physxForceField", multiple_api_token=instance_name)

        usdContext = omni.usd.get_context()
        stage = usdContext.get_stage()

        api_type_name = Usd.SchemaRegistry().GetSchemaTypeName(Usd.CollectionAPI)
        instances = utils.get_schema_instances(usd_prim, api_type_name)

        if ForceFieldSchema.Tokens.forceFieldBodies not in instances:
            collectionAPI = Usd.CollectionAPI.Apply(usd_prim, ForceFieldSchema.Tokens.forceFieldBodies)
            collectionAPI.CreateIncludesRel().AddTarget(stage.GetDefaultPrim().GetPath())
            collectionAPI.CreateIncludeRootAttr().Set(True)

    def remove_force_field_api(self, usd_prim, component, multiple_instance_name):
        if component == "PhysxForceFieldDragAPI":
            execute("UnapplyAPISchema", api=ForceFieldSchema.PhysxForceFieldDragAPI, prim=usd_prim,
                    api_prefix="physxForceField", multiple_api_token=multiple_instance_name)
        elif component == "PhysxForceFieldLinearAPI":
            execute("UnapplyAPISchema", api=ForceFieldSchema.PhysxForceFieldLinearAPI, prim=usd_prim,
                    api_prefix="physxForceField", multiple_api_token=multiple_instance_name)
        elif component == "PhysxForceFieldNoiseAPI":
            execute("UnapplyAPISchema", api=ForceFieldSchema.PhysxForceFieldNoiseAPI, prim=usd_prim,
                    api_prefix="physxForceField", multiple_api_token=multiple_instance_name)
        elif component == "PhysxForceFieldPlanarAPI":
            execute("UnapplyAPISchema", api=ForceFieldSchema.PhysxForceFieldPlanarAPI, prim=usd_prim,
                    api_prefix="physxForceField", multiple_api_token=multiple_instance_name)
        elif component == "PhysxForceFieldSphericalAPI":
            execute("UnapplyAPISchema", api=ForceFieldSchema.PhysxForceFieldSphericalAPI, prim=usd_prim,
                    api_prefix="physxForceField", multiple_api_token=multiple_instance_name)
        elif component == "PhysxForceFieldConicalAPI":
            execute("UnapplyAPISchema", api=ForceFieldSchema.PhysxForceFieldConicalAPI, prim=usd_prim,
                    api_prefix="physxForceField", multiple_api_token=multiple_instance_name)
        elif component == "PhysxForceFieldWindAPI":
            execute("UnapplyAPISchema", api=ForceFieldSchema.PhysxForceFieldWindAPI, prim=usd_prim,
                    api_prefix="physxForceField", multiple_api_token=multiple_instance_name)
        elif component == "PhysxForceFieldSpinAPI":
            execute("UnapplyAPISchema", api=ForceFieldSchema.PhysxForceFieldSpinAPI, prim=usd_prim,
                    api_prefix="physxForceField", multiple_api_token=multiple_instance_name)
        elif component == "PhysxForceFieldRingAPI":
            execute("UnapplyAPISchema", api=ForceFieldSchema.PhysxForceFieldRingAPI, prim=usd_prim,
                    api_prefix="physxForceField", multiple_api_token=multiple_instance_name)

    def remove_force_field_command(self, usd_prim, component, multiple_instance_name):
        if multiple_instance_name:
            self.remove_force_field_api(usd_prim, component, multiple_instance_name)
        else:
            self.remove_all_components(usd_prim, component)

        if not self.schema_present(usd_prim):
            usd_prim.RemoveAPI(Usd.CollectionAPI, ForceFieldSchema.Tokens.forceFieldBodies)
            utils.removeMultipleAPISchemaProperties(Usd.CollectionAPI, usd_prim, "collection", ForceFieldSchema.Tokens.forceFieldBodies)

    def on_startup(self):
        self._physxForceFieldsInterface = None
        self._physxInterface = None
        self._physxUnitTestInterface = None
        self._physxSceneQueryInterface = None
        self._physxSimInterface = None
        self._physxForceFieldsInterface = _physxForceFields.acquire_physx_force_fields_interface()
        
        try:
            from . import tests            
            tests.setPhysxForceFieldsInterface(self._physxForceFieldsInterface)

            self._physxInterface = omni.physx.get_physx_interface()
            tests.setPhysxInterface(self._physxInterface)
            self._physxUnitTestInterface = omni.physx.get_physxunittests_interface()
            tests.setPhysxUnitTestInterface(self._physxUnitTestInterface)
            self._physxSceneQueryInterface = omni.physx.get_physx_scene_query_interface()
            tests.setPhysxSceneQueryInterface(self._physxSceneQueryInterface)
            self._physxSimInterface = omni.physx.get_physx_simulation_interface()
            tests.setPhysxSimInterface(self._physxSimInterface)
        except ImportError:
            pass

        try:
            import omni.kit.window.property as propertyWindow
            import omni.kit.property.physx as kitProperties
            from omni.kit.property.physx.widgets import SingleInstanceWidget

            import omni.physxdemos as demo


            demo.register(DEMO_SCENES)

            baseProperties = ['physxForceField:enabled', 
                            'physxForceField:surfaceSampleDensity', 'physxForceField:surfaceAreaScaleEnabled', 
                            'physxForceField:position', 'physxForceField:range']

            kitProperties.register_schema(
                ForceFieldSchema.PhysxForceFieldDragAPI,
                baseProperties + ['physxForceFieldDrag:minimumSpeed', 'physxForceFieldDrag:linear', 
                                'physxForceFieldDrag:square'],
                kitProperties.Component(
                    lambda p: False,
                    lambda _: True,
                    "PhysxForceFieldDragAPI", "Drag Force Field", ForceFieldSchema.PhysxForceFieldDragAPI,
                    self.add_force_field_command, self.remove_force_field_command
                ),
                prefix="physxForceField"
            )

            kitProperties.register_schema(
                ForceFieldSchema.PhysxForceFieldLinearAPI,
                baseProperties + ['physxForceFieldLinear:direction', 'physxForceFieldLinear:constant', 
                                'physxForceFieldLinear:linear', 'physxForceFieldLinear:inverseSquare'],
                kitProperties.Component(
                    lambda p: False,
                    lambda _: True,
                    "PhysxForceFieldLinearAPI", "Linear Force Field", ForceFieldSchema.PhysxForceFieldLinearAPI,
                    self.add_force_field_command, self.remove_force_field_command
                ),
                prefix="physxForceField"
            )

            kitProperties.register_schema(
                ForceFieldSchema.PhysxForceFieldNoiseAPI,
                baseProperties + ['physxForceFieldNoise:drag', 'physxForceFieldNoise:amplitude', 
                                'physxForceFieldNoise:frequency'],
                kitProperties.Component(
                    lambda p: False,
                    lambda _: True,
                    "PhysxForceFieldNoiseAPI", "Noise Force Field", ForceFieldSchema.PhysxForceFieldNoiseAPI,
                    self.add_force_field_command, self.remove_force_field_command
                ),
                prefix="physxForceField"
            )

            kitProperties.register_schema(
                ForceFieldSchema.PhysxForceFieldPlanarAPI,
                baseProperties + ['physxForceFieldPlanar:normal', 'physxForceFieldPlanar:constant', 
                                'physxForceFieldPlanar:linear', 'physxForceFieldPlanar:inverseSquare'],
                kitProperties.Component(
                    lambda p: False,
                    lambda _: True,
                    "PhysxForceFieldPlanarAPI", "Planar Force Field", ForceFieldSchema.PhysxForceFieldPlanarAPI,
                    self.add_force_field_command, self.remove_force_field_command
                ),
                prefix="physxForceField"
            )

            kitProperties.register_schema(
                ForceFieldSchema.PhysxForceFieldSphericalAPI,
                baseProperties + ['physxForceFieldSpherical:constant', 'physxForceFieldSpherical:linear', 
                                'physxForceFieldSpherical:inverseSquare'],
                kitProperties.Component(
                    lambda p: False,
                    lambda _: True,
                    "PhysxForceFieldSphericalAPI", "Spherical Force Field", ForceFieldSchema.PhysxForceFieldSphericalAPI,
                    self.add_force_field_command, self.remove_force_field_command
                ),
                prefix="physxForceField"
            )

            kitProperties.register_schema(
                ForceFieldSchema.PhysxForceFieldConicalAPI,
                baseProperties + ['physxForceFieldSpherical:angle', 'physxForceFieldSpherical:direction', 
                'physxForceFieldSpherical:constant', 'physxForceFieldSpherical:linear', 
                'physxForceFieldSpherical:inverseSquare'],
                kitProperties.Component(
                    lambda p: False,
                    lambda _: True,
                    "PhysxForceFieldConicalAPI", "Conical Force Field", ForceFieldSchema.PhysxForceFieldConicalAPI,
                    self.add_force_field_command, self.remove_force_field_command
                ),
                prefix="physxForceField"
            )

            kitProperties.register_schema(
                ForceFieldSchema.PhysxForceFieldWindAPI,
                baseProperties + ['physxForceFieldWind:drag', 'physxForceFieldWind:averageSpeed', 
                                'physxForceFieldWind:averageDirection', 'physxForceFieldWind:speedVariation', 
                                'physxForceFieldWind:speedVariationFrequency', 'physxForceFieldWind:directionVariation', 
                                'physxForceFieldWind:directionVariationFrequency'],
                kitProperties.Component(
                    lambda p: False,
                    lambda _: True,
                    "PhysxForceFieldWindAPI", "Wind Force Field", ForceFieldSchema.PhysxForceFieldWindAPI,
                    self.add_force_field_command, self.remove_force_field_command
                ),
                prefix="physxForceField"
            )

            kitProperties.register_schema(
                ForceFieldSchema.PhysxForceFieldSpinAPI,
                baseProperties + ['physxForceFieldSpin:normalAxis', 'physxForceFieldSpin:radius', 
                                'physxForceFieldSpin:constant', 'physxForceFieldSpin:linear', 
                                'physxForceFieldSpin:inverseSquare'],
                kitProperties.Component(
                    lambda p: False,
                    lambda _: True,
                    "PhysxForceFieldSpinAPI", "Spin Force Field", ForceFieldSchema.PhysxForceFieldSpinAPI,
                    self.add_force_field_command, self.remove_force_field_command
                ),
                prefix="physxForceField"
            )

            kitProperties.register_schema(
                ForceFieldSchema.PhysxForceFieldRingAPI,
                baseProperties + ['physxForceFieldRing:normalAxis', 'physxForceFieldRing:radius', 
                                'physxForceFieldRing:constant', 'physxForceFieldRing:linear', 
                                'physxForceFieldRing:inverseSquare', 'physxForceFieldRing:spinConstant', 
                                'physxForceFieldRing:spinLinear', 'physxForceFieldRing:spinInverseSquare'],
                kitProperties.Component(
                    lambda p: False,
                    lambda _: True,
                    "PhysxForceFieldRingAPI", "Ring Force Field", ForceFieldSchema.PhysxForceFieldRingAPI,
                    self.add_force_field_command, self.remove_force_field_command
                ),
                prefix="physxForceField"
            )

            self._collection_widget = SingleInstanceWidget(
                "Force Field Bodies",
                Usd.CollectionAPI,
                ForceFieldSchema.Tokens.forceFieldBodies,
                {}
            )
            self._collection_widget_name = "physx_collection_forcefield"
            propertyWindow.get_window().register_widget(kitProperties.Manager.scheme, self._collection_widget_name, self._collection_widget)
            
        except ImportError:
            pass

    def on_shutdown(self):
        if self._physxForceFieldsInterface is not None:
            _physxForceFields.release_physx_force_fields_interface(self._physxForceFieldsInterface)
            _physxForceFields.release_physx_force_fields_interface_scripting(self._physxForceFieldsInterface) # OM-60917
            self._physxForceFieldsInterface = None

        try:
            from . import tests
            
            tests.clearPhysxSimInterface()
            self._physxSimInterface = None

            tests.clearPhysxUnitTestInterface()
            self._physxUnitTestInterface = None

            tests.clearPhysxInterface()
            self._physxInterface = None

            tests.clearPhysxSceneQueryInterface()
            self._physxSceneQueryInterface = None

        except (NameError, ModuleNotFoundError):
            pass

        try:
            demo.unregister(DEMO_SCENES)
       
            kitProperties.unregister_schema(ForceFieldSchema.PhysxForceFieldDragAPI)
            kitProperties.unregister_schema(ForceFieldSchema.PhysxForceFieldLinearAPI)
            kitProperties.unregister_schema(ForceFieldSchema.PhysxForceFieldNoiseAPI)
            kitProperties.unregister_schema(ForceFieldSchema.PhysxForceFieldPlanarAPI)
            kitProperties.unregister_schema(ForceFieldSchema.PhysxForceFieldSphericalAPI)
            kitProperties.unregister_schema(ForceFieldSchema.PhysxForceFieldConicalAPI)
            kitProperties.unregister_schema(ForceFieldSchema.PhysxForceFieldWindAPI)
            kitProperties.unregister_schema(ForceFieldSchema.PhysxForceFieldSpinAPI)
            kitProperties.unregister_schema(ForceFieldSchema.PhysxForceFieldRingAPI)

            propertyWindow.get_window().unregister_widget(kitProperties.Manager.scheme, self._collection_widget_name)
            self._collection_widget = None
            
        except (NameError, ModuleNotFoundError):
            pass