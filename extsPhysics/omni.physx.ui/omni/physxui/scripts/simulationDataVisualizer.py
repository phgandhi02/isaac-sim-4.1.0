from enum import IntEnum, auto
import carb.input
import omni.appwindow
from omni import ui
import omni.usd
from omni.ui import color as cl
from omni.physx.scripts import propertyQueryRigidBody
from .physicsViewportShared import *
from pxr import Usd, Gf, UsdGeom, UsdPhysics, PhysxSchema, Sdf, Tf
import collections
import math
from .utils import register_stage_update_node

USE_USDRT = True

if USE_USDRT:
    import usdrt

class PhysicsDataTrace():
    def __init__(self, datapoint_num_components, datapoints_num = 200, datapoint_interval = 0.05, min_max_damping = 0.001) -> None:
        self._datapoint_interval = datapoint_interval
        self._trace_interval_modifier = 1.0 / self._datapoint_interval
        self._datapoints_num = datapoints_num
        self._datapoint_num_components = datapoint_num_components
        self._min_max_damping_factor = 1.0 - min_max_damping

        self._input_accumulator = None
        self._input_accumulator_time_elapsed = None
        self._trace_datapoints = None
        self._trace_min_value = None
        self._trace_max_value = None
        self.reset()

    def reset(self):
        self._input_accumulator = None
        self._input_accumulator_time_elapsed = 0.0
        self._trace_datapoints = []
        for _ in range(self._datapoint_num_components):
            self._trace_datapoints.append(collections.deque([0.0] * self._datapoints_num, maxlen=self._datapoints_num))
        self._trace_max_value = [-math.inf] * self._datapoint_num_components
        self._trace_min_value = [math.inf] * self._datapoint_num_components

    # Overridable function for data types that may need further processing (e.g. quaternions to Eulers).
    def append_data(self, data):
        for n in range(self._datapoint_num_components):
            self._trace_datapoints[n].append(data[n])
            self._trace_max_value[n] = max(self._trace_max_value[n] * self._min_max_damping_factor, data[n])
            self._trace_min_value[n] = min(self._trace_min_value[n] * self._min_max_damping_factor, data[n])

    # Adds a sample to the accumulator. Returns true if a new datapoint was added.
    # Note: the number of elements in the input data does not need to match the datapoint length, 
    # but the length must be consistent throughout all add_sample calls.
    def add_sample(self, delta_time, data) -> bool:

        if self._input_accumulator is None:
            self._input_accumulator = [0.0] * len(data)

        self._input_accumulator_time_elapsed += delta_time

        if self._input_accumulator_time_elapsed >= self._datapoint_interval:
            # Time we've exceeded the interval.
            excess_time = self._input_accumulator_time_elapsed - self._datapoint_interval

            for n in range(len(self._input_accumulator)):
                # Add the incoming data for the time that was within the interval.
                self._input_accumulator[n] += (delta_time - excess_time) * data[n]
                # Divide by interval to get average for the timespan.
                self._input_accumulator[n] /= self._datapoint_interval

            self.append_data(self._input_accumulator)

            # Set the accumulator to the excess time.
            self._input_accumulator_time_elapsed = excess_time

            for n in range(len(self._input_accumulator)):
                self._input_accumulator[n] = excess_time * data[n]

            return True
        else:
            for n in range(len(self._input_accumulator)):
                self._input_accumulator[n] += delta_time * data[n]

            return False

    def get_data(self, index=-1):
        if index == -1:
            indices = []
            for n in range(self._datapoint_num_components):
                indices.append(self._trace_datapoints[n])
            return indices
        else:
            return self._trace_datapoints[index]

    def get_min(self, index = -1):
        if index == -1:
            value_min = math.inf
            for n in range(self._datapoint_num_components):
                value_min = min(value_min, self._trace_min_value[n])
            return value_min
        else:
            return self._trace_min_value[index]

    def get_max(self, index = -1):
        if index == -1:
            value_max = -math.inf
            for n in range(self._datapoint_num_components):
                value_max = max(value_max, self._trace_max_value[n])
            return value_max
        else:
            return self._trace_max_value[index]

    def get_datapoint_num_components(self):
        return self._datapoint_num_components

class PhysicsDataTraceEulersFromQuaternion(PhysicsDataTrace):
    def __init__(self) -> None:
        super().__init__(3)

    def append_data(self, data):
        rotation = Gf.Rotation(Gf.Quaternion(data[3], Gf.Vec3d(data[0], data[1], data[2])))
        data = [*rotation.Decompose(Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis())]
        return super().append_data(data)

class PhysicsDataTraceVector(PhysicsDataTrace):
    def __init__(self) -> None:
        super().__init__(3)

class SimulationDataVisualizerWindow(ui.Window):
    SETTINGS_UI_WINDOW_OPACITY = "/persistent/app/viewport/ui/background/opacity"

    class DataProperties():
        def __init__(self, name, components = 3, component_names = None, component_colors = None):
            self._values = [None] * components
            self._ui_values = [None] * components
            self._name = name

            if component_names is None:
                self._component_names = ["X", "Y", "Z", "W"][0:components] if components > 1 else None
            else:
                self._component_names = component_names

            if component_colors is None:
                self._component_colors = SimulationDataVisualizerWindow.PROPERTY_COMPONENT_COLORS[0:components] if components > 1 else None
            else:
                self._component_colors = component_colors


        def build_ui(self):
            with ui.HStack():
                ui.Label(self._name, width=SimulationDataVisualizerWindow.PROPERTY_NAME_WIDTH, style=SimulationDataVisualizerWindow.PROPERTY_NAME_STYLE)
                if len(self._values) == 1:
                    with ui.ZStack():
                        ui.Rectangle(style={"background_color": SimulationDataVisualizerWindow.PROPERTY_VALUE_BACKGROUND_COLOR, "border_color": 0x0,"border_width": 0,"border_radius": 2})
                        with ui.HStack():
                            self._ui_values[0] = ui.Label("-", alignment=ui.Alignment.RIGHT, style=SimulationDataVisualizerWindow.PROPERTY_VALUE_STYLE)
                            ui.Spacer(width=4)
                else:
                    RECT_WIDTH = 12
                    BORDER_RADIUS = 2
                    SPACING = 4

                    for index in range(len(self._values)):
                        if(index != 0):
                            ui.Spacer(width=SPACING)
                        with ui.ZStack():
                            ui.Rectangle(style={"background_color": SimulationDataVisualizerWindow.PROPERTY_VALUE_BACKGROUND_COLOR, "border_color": 0x0,"border_width": 0,"border_radius": BORDER_RADIUS})
                            with ui.ZStack(width=RECT_WIDTH * len(self._component_names[index]) + BORDER_RADIUS * 2):
                                ui.Rectangle(style={"background_color": self._component_colors[index], "border_color": 0x0,"border_width": 0,"border_radius": BORDER_RADIUS})
                                ui.Label(self._component_names[index], alignment=ui.Alignment.CENTER, width=RECT_WIDTH * len(self._component_names[index]) + BORDER_RADIUS * 2, style=SimulationDataVisualizerWindow.PROPERTY_NAME_STYLE)
                            with ui.HStack():
                                ui.Spacer(width=RECT_WIDTH * len(self._component_names[index]) + BORDER_RADIUS)
                                with ui.ZStack():
                                    ui.Rectangle(style={"background_color": SimulationDataVisualizerWindow.PROPERTY_VALUE_BACKGROUND_COLOR})
                                    with ui.HStack():
                                        self._ui_values[index] = ui.Label("-", alignment=ui.Alignment.RIGHT, style=SimulationDataVisualizerWindow.PROPERTY_VALUE_STYLE)
                                ui.Spacer(width=SPACING)

        def set_values(self, values):
            self._values = values
            for index in range(len(self._ui_values)):
                if self._ui_values[index] is not None:
                    if self._values[index] is None:
                        self._ui_values[index].text = "-"
                    else:
                        self._ui_values[index].text = (float_to_string(self._values[index]) if isinstance(self._values[index], float) else str(self._values[index]))
        
        def get_values(self):
            return self._values

    class DataProperty(DataProperties):
        def __init__(self, name):
            super().__init__(name, 1)

        def set_value(self, value):
            super().set_values([value])
        
        def get_value(self):
            return super().get_values()[0]

    class DataPropertyEulersFromQuaternion(DataProperties):
        def __init__(self, name):
            super().__init__(name, 3)

        def set_values(self, values):
            if len(values) == 4:
                rotation = Gf.Rotation(Gf.Quaternion(values[3], Gf.Vec3d(values[0], values[1], values[2])))
                values = [*rotation.Decompose(Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis())]
            super().set_values(values)
        
        def get_values(self):
            return super().get_values()

    class DataPlot():
        def __init__(self, data_property, data_trace : PhysicsDataTrace = None):
            self._ui_label_min = None
            self._ui_label_max = None
            self._ui_plots = []
            self._ui_baseline = None
            self._data_property = data_property
            if data_trace is not None:
                self._data_trace = data_trace
            else:
                self._data_trace = PhysicsDataTrace(len(self._data_property.get_values()))

        def build_ui(self, colors = None):
            self._data_property.build_ui()
            with ui.HStack(height=100):
                with ui.ZStack():
                    if colors is None:
                        colors = self._data_property._component_colors
                        if colors is None:
                            if self._data_trace.get_datapoint_num_components() == 1:
                                colors = SimulationDataVisualizerWindow.PROPERTY_COMPONENT_COLORS[1:2]
                            else:
                                colors = SimulationDataVisualizerWindow.PROPERTY_COMPONENT_COLORS[0:self._data_trace.get_datapoint_num_components()]
        
                    ui.Rectangle(style={"background_color": SimulationDataVisualizerWindow.PLOT_BACKGROUND_COLOR})
                    data_plots = []
                    for value in range(self._data_trace.get_datapoint_num_components()):
                        data_plots.append(ui.Plot(
                            ui.Type.LINE,
                            -1,
                            1,
                            *self._data_trace.get_data(value),
                            width=ui.Percent(100),
                            height=100,
                            style={"color": colors[value], "background_color": 0x00000000},
                        ))
                    self._ui_plots = data_plots

                    self._ui_baseline = ui.Plot(
                            ui.Type.LINE,
                            0,
                            1,
                            0.0, 0.0,
                            width=ui.Percent(100),
                            height=100,
                            style={"color": 0x44ffffff, "background_color": 0x00000000},
                        )
                    self._ui_label_max = ui.Label("1.0", style={"font_size": 12.0, "margin": 3.0}, width=0, alignment=ui.Alignment.LEFT_TOP)
                    self._ui_label_min = ui.Label("-1.0", style={"font_size": 12.0, "margin": 3.0}, width=0, alignment=ui.Alignment.LEFT_BOTTOM)
                    # This is to block the default plot tooltips, since they do not work well with multiple graphs overlapping.
                    with ui.VStack(content_clipping=True):
                        ui.Rectangle(style={"background_color": 0x000000}, width=ui.Percent(100), height=100)
                    
                    self.refresh_plot()

        def _refresh_limits(self):

            def adjust_limit(limit: float):
                range_factor = math.pow(10.0, math.floor(math.log10(math.fabs(limit))))
                return (-1.0 if limit < 0 else 1.0) * math.ceil(math.fabs(limit) / range_factor) * range_factor

            data_min = self._data_trace.get_min()

            if isinf(data_min):
                data_min = -1.0
            if data_min != 0.0:
                data_min = adjust_limit(data_min)

            data_max = self._data_trace.get_max()

            if isinf(-data_max):
                data_max = data_min + 2.0
            if data_max != 0.0:
                data_max = adjust_limit(data_max)

            # Set offset of min and max to always match distance from baseline. This makes the graphs easier to read but could be made optional.
            if data_min < 0.0:
                data_max = max(data_max, -data_min)
                data_min = min(-data_max, data_min)
            else:
                data_min = 0.0

            self._ui_label_min.text = float_to_string(data_min)
            self._ui_label_max.text = float_to_string(data_max)

            return data_min, data_max
        
        def reset_data(self):
            self._data_trace.reset()
            self.refresh_plot()

        def refresh_plot(self):
            if self._ui_label_min is None:
                return

            data_min, data_max = self._refresh_limits()

            for n in range(self._data_trace.get_datapoint_num_components()):
                self._ui_plots[n].scale_min = data_min
                self._ui_plots[n].scale_max = data_max
                self._ui_plots[n].set_data(*self._data_trace.get_data(n))

            self._ui_baseline.scale_min = data_min
            self._ui_baseline.scale_max = data_max

        def set_values(self, values):
            self._data_property.set_values(values)

        def add_sample(self, time_delta, values):
            if time_delta > 0.0 and self._data_trace.add_sample(time_delta, values):
                self.refresh_plot()

            self._data_property.set_values(values)

    class DataPlotEulersFromQuaternion(DataPlot):
        def __init__(self, data_property, data_trace: PhysicsDataTrace = None):
            if data_trace is None:
                data_trace = PhysicsDataTraceEulersFromQuaternion()
            super().__init__(data_property, data_trace)

        def _refresh_limits(self):

            self._ui_label_min.text = "-180°"
            self._ui_label_max.text = "+180°"

            return -180.0, 180.0

    class Monitor():
        def __init__(self, window):
            self._window = window

        def get_stage(self):
            return self._window._stage
        
        def get_prim(self):
            return self._window._prim

        def get_prim_path(self):
            return self._window._prim_path

        def get_usdrt_prim(self):
            return self._window._usdrt_prim

        @staticmethod
        def get_is_prim_valid_target(prim : Usd.Prim) -> bool:
            return (prim is not None)

        def refresh_simulation_data(self, delta_time = 0.0):
            pass

        def refresh_data(self):
            self.refresh_simulation_data()

        def build_collapsable_section(self, name, build_func):
            with ui.CollapsableFrame(name, height=0, style={"background_color": SimulationDataVisualizerWindow.FRAME_COLOR}):
                with ui.HStack():
                    with ui.VStack(spacing=SimulationDataVisualizerWindow.LINE_SPACING):
                        build_func()

        def build_section(self, build_func):
            with ui.Frame(height=0, style={"background_color": SimulationDataVisualizerWindow.FRAME_COLOR}):
                with ui.HStack():
                    with ui.VStack(spacing=SimulationDataVisualizerWindow.LINE_SPACING):
                        build_func()

        def build_ui(self):
            pass

    """
    class MonitorTemplate(Monitor):
        def __init__(self, window):
            super().__init__(window)

        def refresh_simulation_data(self, delta_time = 0.0):
            super().refresh_simulation_data(delta_time)

        def refresh_data(self):
            super().refresh_data()

        def build_ui(self):
            super().build_ui()

        @staticmethod
        def get_is_prim_valid_target(prim : Usd.Prim) -> bool:
            return super(__class__, __class__).get_is_prim_valid_target(prim)
    """

    class MonitorPrimBasic(Monitor):
        def __init__(self, window):
            super().__init__(window)
            self._property_path = SimulationDataVisualizerWindow.DataProperty("Path")

        def refresh_data(self):
            super().refresh_data()
            self._property_path.set_value(self.get_prim_path() if (self.get_prim() is not None and self.get_prim().IsValid()) else None)

        def build_ui(self):
            super().build_ui()
            self.build_section(self._property_path.build_ui)

        @staticmethod
        def get_is_prim_valid_target(prim : Usd.Prim) -> bool:
            return True

    class MonitorXformable(Monitor):
        def __init__(self, window):
            super().__init__(window)

            world_position_prop = SimulationDataVisualizerWindow.DataProperties("Position", 3)
            self._data_trace_world_position = SimulationDataVisualizerWindow.DataPlot(world_position_prop)

            world_orientation_prop = SimulationDataVisualizerWindow.DataPropertyEulersFromQuaternion("Orientation")
            self._data_trace_world_orientation = SimulationDataVisualizerWindow.DataPlotEulersFromQuaternion(world_orientation_prop)

            self._prim_xformable = None
            self._usdrt_xformable_prim = None
            self._usdrt_xformable_prim_attrib_world_position = None
            self._usdrt_xformable_prim_attrib_world_orientation = None

            self._data_trace_prim = None

        def refresh_simulation_data(self, delta_time = 0.0):
            super().refresh_simulation_data(delta_time)

            if self._usdrt_xformable_prim is not None and self._usdrt_xformable_prim.HasWorldXform():
                position = self._usdrt_xformable_prim_attrib_world_position.Get()
                quaternion = self._usdrt_xformable_prim_attrib_world_orientation.Get()
            elif self._prim_xformable is not None:
                prim_xform_world = self._prim_xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                position = prim_xform_world.ExtractTranslation()
                rotation = prim_xform_world.RemoveScaleShear().GetOrthonormalized().ExtractRotation()
                quaternion = rotation.GetQuaternion()
            else:
                return

            orientation = [*quaternion.GetImaginary()] + [quaternion.GetReal()]
            position = [*position]
            self._data_trace_world_position.add_sample(delta_time, position)
            self._data_trace_world_orientation.add_sample(delta_time, orientation)
            
        def refresh_data(self):

            if self._data_trace_prim != self.get_prim():
                self._data_trace_prim = self.get_prim()
                self._data_trace_world_position.reset_data()
                self._data_trace_world_position.reset_data()

            self._prim_xformable = UsdGeom.Xformable(self.get_prim()) if self.get_prim() is not None else None

            if self.get_usdrt_prim() is None:
                self._usdrt_xformable_prim = None
                self._usdrt_xformable_prim_attrib_world_position = None
                self._usdrt_xformable_prim_attrib_world_orientation = None
            else:
                self._usdrt_xformable_prim = usdrt.Rt.Xformable(self.get_usdrt_prim())
                self._usdrt_xformable_prim_attrib_world_position = self._usdrt_xformable_prim.GetWorldPositionAttr()
                self._usdrt_xformable_prim_attrib_world_orientation = self._usdrt_xformable_prim.GetWorldOrientationAttr()

            super().refresh_data()

        def _build_world_pose_section(self):
            self._data_trace_world_position.build_ui()
            self._data_trace_world_orientation.build_ui()

        def build_ui(self):
            super().build_ui()
            self.build_collapsable_section("World Pose", self._build_world_pose_section)

        @staticmethod
        def get_is_prim_valid_target(prim : Usd.Prim) -> bool:
            return super(__class__, __class__).get_is_prim_valid_target(prim) and prim.IsA(UsdGeom.Xformable)

    class MonitorRigidBody(Monitor):
        def __init__(self, window):
            super().__init__(window)

            self._prim_rigid_body_api = None
            self._prim_rigid_body_linear_velocity = None
            self._prim_rigid_body_angular_velocity = None
            self._usdrt_prim_rigid_body = None
            self._usdrt_prim_rigid_body_linear_velocity = None
            self._usdrt_prim_rigid_body_angular_velocity = None

            self._rigid_body_property_query_manager = None
            self._refresh_data_on_query_complete = False

            linear_velocity_prop = SimulationDataVisualizerWindow.DataProperties("Linear", 4, ["X", "Y", "Z", "M"])
            self._data_trace_linear_velocity = SimulationDataVisualizerWindow.DataPlot(linear_velocity_prop)

            angular_velocity_prop = SimulationDataVisualizerWindow.DataProperties("Angular", 4, ["X", "Y", "Z", "M"])
            self._data_trace_angular_velocity = SimulationDataVisualizerWindow.DataPlot(angular_velocity_prop)

            self._total_mass_prop = SimulationDataVisualizerWindow.DataProperty("Total mass")
            self._center_of_mass_prop = SimulationDataVisualizerWindow.DataProperties("Center of mass", 3)
            self._principal_axes_prop = SimulationDataVisualizerWindow.DataProperties("Principal Axes", 3)
            self._diagonal_inertia_prop = SimulationDataVisualizerWindow.DataProperties("Diagonal inertia", 3)

            self._data_trace_prim = None

        def refresh_simulation_data(self, delta_time = 0.0):
            super().refresh_simulation_data(delta_time)

            linear_velocity = None
            angular_velocity = None
            if self._usdrt_prim_rigid_body is not None:
                if self._usdrt_prim_rigid_body_linear_velocity is not None:
                    linear_velocity = self._usdrt_prim_rigid_body_linear_velocity.Get()
                if self._usdrt_prim_rigid_body_angular_velocity is not None:
                    angular_velocity = self._usdrt_prim_rigid_body_angular_velocity.Get()
            if linear_velocity is None and self._prim_rigid_body_linear_velocity is not None:
                linear_velocity = self._prim_rigid_body_linear_velocity.Get()
            if angular_velocity is None and self._prim_rigid_body_angular_velocity:
                angular_velocity = self._prim_rigid_body_angular_velocity.Get()

            if linear_velocity is not None:
                self._data_trace_linear_velocity.add_sample(delta_time, [*linear_velocity] + [Gf.Vec3d(*linear_velocity).GetLength()])
            if angular_velocity is not None:
                self._data_trace_angular_velocity.add_sample(delta_time, [*angular_velocity] + [Gf.Vec3d(*angular_velocity).GetLength()])

        def refresh_data(self):
            self._prim_rigid_body_api = None
            self._prim_rigid_body_linear_velocity = None
            self._prim_rigid_body_angular_velocity = None
            self._usdrt_prim_rigid_body = None
            self._usdrt_prim_rigid_body_linear_velocity = None
            self._usdrt_prim_rigid_body_angular_velocity = None

            if self.get_prim() is None:
                super().refresh_data()
                return
            
            if self._data_trace_prim != self.get_prim():
                self._data_trace_prim = self.get_prim()
                self._data_trace_linear_velocity.reset_data()
                self._data_trace_angular_velocity.reset_data()

            self._prim_rigid_body_api = UsdPhysics.RigidBodyAPI(self.get_prim())
            self._prim_rigid_body_linear_velocity = self._prim_rigid_body_api.GetVelocityAttr()
            self._prim_rigid_body_angular_velocity = self._prim_rigid_body_api.GetAngularVelocityAttr()
            if self.get_usdrt_prim() is not None:
                self._usdrt_prim_rigid_body = usdrt.UsdPhysics.RigidBodyAPI(self.get_usdrt_prim())
                if self._usdrt_prim_rigid_body is not None:
                    self._usdrt_prim_rigid_body_linear_velocity = self._usdrt_prim_rigid_body.GetVelocityAttr()
                    self._usdrt_prim_rigid_body_angular_velocity = self._usdrt_prim_rigid_body.GetAngularVelocityAttr()

            class QueryManagerObjectInfo(propertyQueryRigidBody.QueryManager):
                def __init__(self, parent):
                    super().__init__()
                    self._parent = parent
                
                def on_query_finished(self):
                    if(self.get_query_status() == propertyQueryRigidBody.Query.Status.COMPLETE and
                        self._parent._refresh_data_on_query_complete):
                        self._parent.refresh_data()
            
            if self._rigid_body_property_query_manager is None:
                self._rigid_body_property_query_manager = QueryManagerObjectInfo(self)

            if self._rigid_body_property_query_manager.get_query_status() == propertyQueryRigidBody.Query.Status.UNSET:
                # Fetch mass properties from PhysX.
                self._rigid_body_property_query_manager.submit_query(self.get_prim())

            if self._rigid_body_property_query_manager.get_query_status() == propertyQueryRigidBody.Query.Status.IN_PROGRESS:
                self._total_mass_prop.set_value(None)
                self._center_of_mass_prop.set_values(None)
                self._principal_axes_prop.set_values(None)
                self._diagonal_inertia_prop.set_values(None)
                self._refresh_data_on_query_complete = True
            else:
                self._refresh_data_on_query_complete = False
                results = self._rigid_body_property_query_manager.get_query_result()

                world_rotation_eulers = [*Gf.Rotation(results.principal_axes).Decompose(Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis())]

                self._total_mass_prop.set_value(results.mass)
                self._center_of_mass_prop.set_values(results.center_of_mass)
                self._principal_axes_prop.set_values(world_rotation_eulers)
                self._diagonal_inertia_prop.set_values(results.diagonal_inertia)

            super().refresh_data()

        def _build_velocity_section(self):
            self._data_trace_linear_velocity.build_ui()
            self._data_trace_angular_velocity.build_ui()

        def _build_mass_section(self):
            self._total_mass_prop.build_ui()
            self._center_of_mass_prop.build_ui()
            self._principal_axes_prop.build_ui()
            self._diagonal_inertia_prop.build_ui()

        def build_ui(self):
            super().build_ui()
            self.build_collapsable_section("Velocity", self._build_velocity_section)
            self.build_collapsable_section("Mass/inertia", self._build_mass_section)

        @staticmethod
        def get_is_prim_valid_target(prim : Usd.Prim) -> bool:
            return super(__class__, __class__).get_is_prim_valid_target(prim) and prim.HasAPI(UsdPhysics.RigidBodyAPI)


    class MonitorResiduals(Monitor):
        def __init__(self, window):
            super().__init__(window)

            self._residual_api = None
            self._residual_attribute_position_rms = None
            self._residual_attribute_position_max = None
            self._residual_attribute_velocity_rms = None
            self._residual_attribute_velocity_max = None
            self._data_trace_residual_position_rms = SimulationDataVisualizerWindow.DataPlot(
                SimulationDataVisualizerWindow.DataProperty("Position RMS"))
            self._data_trace_residual_position_max = SimulationDataVisualizerWindow.DataPlot(
                SimulationDataVisualizerWindow.DataProperty("Position Max"))

            self._data_trace_residual_velocity_rms = SimulationDataVisualizerWindow.DataPlot(
                SimulationDataVisualizerWindow.DataProperty("Velocity RMS"))
            self._data_trace_residual_velocity_max = SimulationDataVisualizerWindow.DataPlot(
                SimulationDataVisualizerWindow.DataProperty("Velocity Max"))
            
            self._data_trace_prim = None

        def refresh_simulation_data(self, delta_time = 0.0):
            super().refresh_simulation_data(delta_time)
            if self._residual_api is not None:
                self._data_trace_residual_position_rms.add_sample(delta_time, [self._residual_attribute_position_rms.Get()])
                self._data_trace_residual_position_max.add_sample(delta_time, [self._residual_attribute_position_max.Get()])

                self._data_trace_residual_velocity_rms.add_sample(delta_time, [self._residual_attribute_velocity_rms.Get()])
                self._data_trace_residual_velocity_max.add_sample(delta_time, [self._residual_attribute_velocity_max.Get()])

        def refresh_data(self):
            if self.get_prim() is None:
                self._residual_api = None
                self._residual_attribute_position_rms = None
                self._residual_attribute_position_max = None
                self._residual_attribute_velocity_rms = None
                self._residual_attribute_velocity_max = None
                self._data_trace_residual_position_rms.reset_data()
                self._data_trace_residual_position_max.reset_data()
                self._data_trace_residual_velocity_rms.reset_data()
                self._data_trace_residual_velocity_max.reset_data()
                super().refresh_data()
                return
            

            if self._data_trace_prim != self.get_prim():
                self._data_trace_prim = self.get_prim()
                self._data_trace_residual_position_rms.reset_data()
                self._data_trace_residual_position_max.reset_data()
                self._data_trace_residual_velocity_rms.reset_data()
                self._data_trace_residual_velocity_max.reset_data()

            self._residual_api = PhysxSchema.PhysxResidualReportingAPI(self.get_prim())
            self._residual_attribute_position_rms = self._residual_api.GetPhysxResidualReportingRmsResidualPositionIterationAttr()
            self._residual_attribute_position_max = self._residual_api.GetPhysxResidualReportingMaxResidualPositionIterationAttr()
            self._residual_attribute_velocity_rms = self._residual_api.GetPhysxResidualReportingRmsResidualVelocityIterationAttr()
            self._residual_attribute_velocity_max = self._residual_api.GetPhysxResidualReportingMaxResidualVelocityIterationAttr()
            super().refresh_data()

        def _build_residual_section(self):
            self._data_trace_residual_position_rms.build_ui()
            self._data_trace_residual_position_max.build_ui()
            self._data_trace_residual_velocity_rms.build_ui()
            self._data_trace_residual_velocity_max.build_ui()

        def build_ui(self):
            super().build_ui()
            self.build_collapsable_section("Residuals", self._build_residual_section)

        @staticmethod
        def get_is_prim_valid_target(prim : Usd.Prim) -> bool:
            return super(__class__, __class__).get_is_prim_valid_target(prim) and prim.HasAPI(PhysxSchema.PhysxResidualReportingAPI) and not prim.IsA(UsdPhysics.Joint)

    class MonitorResidualsJoint(Monitor):
        def __init__(self, window):
            super().__init__(window)

            self._residual_api = None
            self._residual_attribute_position = None
            self._residual_attribute_velocity = None
            residual_position_prop = SimulationDataVisualizerWindow.DataProperty("Position") # , 2, ["RMS", "Max"], colors)
            self._data_trace_residual_position = SimulationDataVisualizerWindow.DataPlot(residual_position_prop)
            residual_velocity_prop = SimulationDataVisualizerWindow.DataProperty("Velocity") # , 2, ["RMS", "Max"], colors)
            self._data_trace_residual_velocity = SimulationDataVisualizerWindow.DataPlot(residual_velocity_prop)

            self._data_trace_prim = None

        def refresh_simulation_data(self, delta_time = 0.0):
            super().refresh_simulation_data(delta_time)
            if self._residual_api is not None:
                residual_position = [self._residual_attribute_position.Get()]
                residual_velocity = [self._residual_attribute_velocity.Get()]
                self._data_trace_residual_position.add_sample(delta_time, residual_position)
                self._data_trace_residual_velocity.add_sample(delta_time, residual_velocity)
        
        def refresh_data(self):
            if self.get_prim() is None:
                self._residual_api = None
                self._residual_attribute_position = None
                self._residual_attribute_velocity = None
                self._data_trace_residual_position.reset_data()
                self._data_trace_residual_velocity.reset_data()
                super().refresh_data()
                return
            
            if self._data_trace_prim != self.get_prim():
                self._data_trace_prim = self.get_prim()
                self._data_trace_residual_position.reset_data()
                self._data_trace_residual_velocity.reset_data()

            self._residual_api = PhysxSchema.PhysxResidualReportingAPI(self.get_prim())
            self._residual_attribute_position = self._residual_api.GetPhysxResidualReportingRmsResidualPositionIterationAttr()
            self._residual_attribute_velocity = self._residual_api.GetPhysxResidualReportingRmsResidualVelocityIterationAttr()
            super().refresh_data()

        def _build_residual_section(self):
            self._data_trace_residual_position.build_ui()
            self._data_trace_residual_velocity.build_ui()

        def build_ui(self):
            super().build_ui()
            self.build_collapsable_section("Residual", self._build_residual_section)

        @staticmethod
        def get_is_prim_valid_target(prim : Usd.Prim) -> bool:
            return super(__class__, __class__).get_is_prim_valid_target(prim) and prim.HasAPI(PhysxSchema.PhysxResidualReportingAPI) and prim.IsA(UsdPhysics.Joint)

    PROPERTY_COMPONENT_COLORS = [cl("#AA5555"), cl("#71A376"), cl("#4F7DA0"), cl("AAAA55"), cl("AA55AA"), cl("55AAAA")]
    WINDOW_COLOR = cl("#454545")
    FRAME_COLOR = cl("#323434")
    LINE_SPACING = 2
    PROPERTY_NAME_STYLE = {"margin": 2}
    PROPERTY_NAME_WIDTH = 100
    PROPERTY_VALUE_BACKGROUND_COLOR = cl("#1F2123")
    PROPERTY_VALUE_STYLE = {"margin": 2, "color": cl("#868E8F")}
    PLOT_BACKGROUND_COLOR = cl("#222222")

    DEFAULT_WINDOW_WIDTH = 400

    def __init__(self):
        super().__init__(
            "Simulation Data Visualizer",
            visible=False,
            flags=ui.WINDOW_FLAGS_NO_FOCUS_ON_APPEARING
        )

        self.frame.set_build_fn(self.build)

        self._settings = carb.settings.get_settings()
        self._background_opacity_setting = self._settings.subscribe_to_node_change_events(
            self.SETTINGS_UI_WINDOW_OPACITY, self._on_setting_changed
        )

        self._rigid_body_property_query_manager = None
        self._refresh_text_on_query_complete = False

        self._selection = None

        self._usd_context = omni.usd.get_context()
        self._stage_update_node = None
        self._stage = None
        self._stage_event_sub = None
        self._physx_step_sub = None
        self._simulation_time_delta = 0.0
        self._usd_object_changed_listener = None

        self._data_monitors = []

        self._prim_path = None
        self._prim = None

        self._usdrt_stage = None
        self._usdrt_prim = None

        opacity = self._settings.get_as_float(self.SETTINGS_UI_WINDOW_OPACITY)
        SimulationDataVisualizerWindow.set_background_opacity(opacity)

    def _on_setting_changed(self, item, event_type):
        opacity = self._settings.get_as_float(self.SETTINGS_UI_WINDOW_OPACITY)
        SimulationDataVisualizerWindow.set_background_opacity(opacity)
        self.frame.rebuild()
    
    @staticmethod
    def set_background_opacity(opacity):
        def set_color_opacity(color : int, opacity : float):
            return color % (0x01000000) + min(255, int(opacity * 256.0)) * 0x01000000
        
        SimulationDataVisualizerWindow.WINDOW_COLOR = set_color_opacity(SimulationDataVisualizerWindow.WINDOW_COLOR, opacity)
        SimulationDataVisualizerWindow.FRAME_COLOR = set_color_opacity(SimulationDataVisualizerWindow.FRAME_COLOR, opacity)
        SimulationDataVisualizerWindow.PROPERTY_VALUE_BACKGROUND_COLOR = set_color_opacity(SimulationDataVisualizerWindow.PROPERTY_VALUE_BACKGROUND_COLOR, opacity)
        SimulationDataVisualizerWindow.PLOT_BACKGROUND_COLOR = set_color_opacity(SimulationDataVisualizerWindow.PLOT_BACKGROUND_COLOR, opacity)

        for n in range(len(SimulationDataVisualizerWindow.PROPERTY_COMPONENT_COLORS)):
            SimulationDataVisualizerWindow.PROPERTY_COMPONENT_COLORS[n] = set_color_opacity(SimulationDataVisualizerWindow.PROPERTY_COMPONENT_COLORS[n], opacity)


    def _refresh_monitor_data(self):
        prim_monitor_types = self.get_prim_data_monitor_types(self._prim)
        refresh_data = True
        if len(prim_monitor_types) != len(self._data_monitors):
            self._data_monitors.clear()
            self.frame.rebuild()
            refresh_data = False
        else:
            for monitor, prim_monitor_type in zip(self._data_monitors, prim_monitor_types):
                if type(monitor) != prim_monitor_type:
                    refresh_data = False
                    self._data_monitors.clear()
                    self.frame.rebuild()
                    break

        if refresh_data:
            for monitor in self._data_monitors:
                monitor.refresh_data()

    def _refresh_monitor_simulation_data(self, time_delta):
        for monitor in self._data_monitors:
            monitor.refresh_simulation_data(time_delta)

    def _refresh_target(self):
        self._selection = self._usd_context.get_selection()
        selected_prim_path = None
        if self._selection is not None and self._stage is not None:
            selected_prim_paths = self._selection.get_selected_prim_paths()
            if len(selected_prim_paths) > 0:
                # Always use the most recently (last) selected prim.
                selected_prim_path = selected_prim_paths[len(selected_prim_paths) - 1]
            else:
                selected_prim_path = None

        self._set_target_path(selected_prim_path)

    def _set_target_path(self, prim_path):
        if prim_path == self._prim_path:
            return

        self._prim_path = prim_path

        if prim_path is not None:
            self._prim = self._stage.GetPrimAtPath(prim_path)

            if self._physx_step_sub is None:
                physx_interface = omni.physx.get_physx_interface()
                if physx_interface is not None:
                    self._physx_step_sub = physx_interface.subscribe_physics_on_step_events(self._on_simulation_step, False, 0 )
                    self._simulation_time_delta = 0.0
                else:
                    carb.log_error("Failed to retrieve Physx interface.")
            
            if self._stage_update_node is None:
                self.stage_update_node = register_stage_update_node("simDataVisualization", on_update_fn=self._on_stage_update, priority=12)

            if self._usdrt_stage is not None:
                self._usdrt_prim = self._usdrt_stage.GetPrimAtPath(str(prim_path))
            else:
                self._usdrt_prim = None
        else:
            self._prim = None
            self._physx_step_sub = None
            self._stage_update_node = None
            self._usdrt_prim = None

        self._refresh_monitor_data()

    def _on_simulation_step(self, delta_time):
        self._simulation_time_delta += delta_time
    
    def _on_stage_update(self, current_time, delta_time):

        if self._simulation_time_delta <= 0.0:
            return

        self._refresh_monitor_simulation_data(self._simulation_time_delta)

        self._simulation_time_delta = 0.0

    def _attach_stage(self):
        self._stage = self._usd_context.get_stage()
        if self._stage is not None:
            if USE_USDRT:
                self._usdrt_stage = usdrt.Usd.Stage.Attach(self._usd_context.get_stage_id())
                if self._usdrt_stage is None:
                    carb.log_error("Failed to attach USDRT to stage.")
            self._attach_usd_change_listener()
        self._refresh_target()
 
    def _detach_stage(self):
        self._physx_step_sub = None
        self._stage_update_node = None
        self._stage = None
        self._usdrt_stage = None
        self._set_target_path(None)
        self._revoke_usd_change_listener()

    def _attach_usd_change_listener(self):
        if self._usd_object_changed_listener is None and self._stage is not None:
            self._usd_object_changed_listener = Tf.Notice.Register(
                    Usd.Notice.ObjectsChanged, self._on_usd_objects_changed,
                    self._stage)

    def _revoke_usd_change_listener(self):
        if self._usd_object_changed_listener is not None:
            self._usd_object_changed_listener.Revoke()
            self._usd_object_changed_listener = None

    def _on_usd_objects_changed(self, notice, stage):
        if self._prim is None:
            return
        refresh_data = False
        for changed_path in (notice.GetChangedInfoOnlyPaths() + notice.GetResyncedPaths()):
            if(self._stage.GetPrimAtPath(changed_path.GetPrimPath()) == self._prim):
                refresh_data = True

        if refresh_data:
            self._refresh_monitor_data()

    def _on_stage_event(self, event):
        if event.type is int(omni.usd.StageEventType.SELECTION_CHANGED):
            self._refresh_target()
        elif event.type is int(omni.usd.StageEventType.OPENED):
            self._attach_stage()
        elif event.type is int(omni.usd.StageEventType.CLOSING):
            self._detach_stage()
        elif event.type is int(omni.usd.StageEventType.SIMULATION_START_PLAY):
            # During simulation we only listen for specific attributes that are then queried each frame.
            self._revoke_usd_change_listener()
        elif event.type is int(omni.usd.StageEventType.SIMULATION_STOP_PLAY):
            self._attach_usd_change_listener()

    def set_pos(self, x, y):
        self.position_x, self.position_y = x, y

    def show(self):
        if not self.docked:
            self.width = self.DEFAULT_WINDOW_WIDTH
            self.height = -1

        if self._stage_event_sub is None:
            self._stage_event_sub = self._usd_context.get_stage_event_stream().create_subscription_to_pop(self._on_stage_event)

        self.visible = True

        self._attach_stage()

    def hide(self):
        self.visible = False
        self._stage_event_sub = None
        self._detach_stage()

    def get_prim_data_monitor_types(self, prim) -> list[Monitor]:
        monitor_types = []

        if __class__.MonitorPrimBasic.get_is_prim_valid_target(prim):
            monitor_types.append(__class__.MonitorPrimBasic)

        if __class__.MonitorXformable.get_is_prim_valid_target(prim):
            monitor_types.append(__class__.MonitorXformable)

        if __class__.MonitorRigidBody.get_is_prim_valid_target(prim):
            monitor_types.append(__class__.MonitorRigidBody)

        if __class__.MonitorResiduals.get_is_prim_valid_target(prim):
            monitor_types.append(__class__.MonitorResiduals)

        if __class__.MonitorResidualsJoint.get_is_prim_valid_target(prim):
            monitor_types.append(__class__.MonitorResidualsJoint)

        return monitor_types
    
    def build(self):
       
        if not self.docked:
            self.width = self.DEFAULT_WINDOW_WIDTH
            self.height = -1

        self.frame.set_style({"Window": {"background_color": self.WINDOW_COLOR, "border_color": 0x0, "border_width": 0, "border_radius": 5}})

        with self.frame:
            with ui.VStack(height=0, spacing=5):
                self._data_monitors.clear()
                monitor_types = self.get_prim_data_monitor_types(self._prim)

                for monitor_type in monitor_types:
                    self._data_monitors.append(monitor_type(self))

                for monitor in self._data_monitors:
                    monitor.build_ui()
                    monitor.refresh_data()

    def destroy(self):
        self._detach_stage()
        super().destroy()
