from typing import Callable
import omni.usd
import carb
import usdrt


display_dialog: bool = True  # static property
omni_ui_loaded: bool = True  # static property

try:
    from omni.ui import color as cl
    from omni import ui
except:    
    omni_ui_loaded = False
    pass


class FSDCheckDialog:
    FABRIC_SCENE_DELEGATE_SETTING = "/app/useFabricSceneDelegate"
    SPACER_WIDTH: int = 10
    SPACER_HEIGHT: int = 10
    WARNING_COLOR = None
    HI_COLOR = None

    @classmethod
    def is_fsd_enabled(cls) -> bool:
        return carb.settings.get_settings().get_as_bool(cls.FABRIC_SCENE_DELEGATE_SETTING)

    @classmethod
    def is_physics_present(cls) -> bool:
        # check if there is some physics, currently lets check just rigid bodies
        usdrt_stage = usdrt.Usd.Stage.Attach(omni.usd.get_context().get_stage_id())
        prim_paths = usdrt_stage.GetPrimsWithAppliedAPIName("PhysicsRigidBodyAPI")
        return len(prim_paths) != 0

    def __init__(self, title="FSD Physics Check"):
        self._window = ui.Window(
            title,
            visible=False,
            width=0,
            height=0,
            auto_resize=True,
            dockPreference=ui.DockPreference.DISABLED
        )
        self._window.flags = (
            ui.WINDOW_FLAGS_NO_COLLAPSE
            | ui.WINDOW_FLAGS_NO_SCROLLBAR
            | ui.WINDOW_FLAGS_MODAL
            | ui.WINDOW_FLAGS_NO_SAVED_SETTINGS
        )
        self.WARNING_COLOR = cl.yellow
        self.HI_COLOR = cl.white

        def do_not_show_changed(model):
            global display_dialog
            display_dialog = not model.as_bool

        def build_line(label: str, indent: int = self.SPACER_WIDTH, style=None):
            with ui.HStack():
                ui.Spacer(width=indent)
                label = ui.Label(label, width=0)
                if style:
                    label.set_style(style)
                ui.Spacer(width=indent)  # also add outer spacer
            return label

        def build_section(name: str, build_func: Callable[[], None]):
            def collapsed_changed(collapsed: bool):
                pass  # TODO: do we want to collapse other frames when expanded?
            ui.Spacer(height=3)
            col_frame = ui.CollapsableFrame(name, height=0, collapsed=True, collapsed_changed_fn=collapsed_changed)
            with col_frame:
                with ui.HStack():
                    ui.Spacer(width=self.SPACER_WIDTH * 2)
                    with ui.VStack():
                        build_func()
            return col_frame

        def build_disable_fsd_permanently_section():
            build_line("Execute your application with additional cmd line parameter:")
            build_line("--/app/useFabricSceneDelegate=false", self.SPACER_WIDTH * 2, style={"color": self.HI_COLOR})
            ui.Spacer(height=self.SPACER_HEIGHT)
            build_line("- OR -")
            ui.Spacer(height=self.SPACER_HEIGHT)
            build_line("Go to your application .kit file and add to your [settings] section line:")
            build_line("app.useFabricSceneDelegate = false", self.SPACER_WIDTH * 2, style={"color": self.HI_COLOR})

        def build_how_to_section():
            with ui.VStack(height=0, spacing=0):
                build_section(
                    "How to disable Fabric Scene Delegate for this session only",
                    lambda: build_line("Go to Edit->Preferences->Rendering and uncheck 'Enable Fabric delegate', then reload your stage.")
                )
                build_section(
                    "How to disable Fabric Scene Delegate for subsequent application runs",
                    build_disable_fsd_permanently_section
                )

        def build_do_not_show_checkbox():
            tooltip = "Stop showing this window on Play button press."
            with ui.HStack():
                ui.Spacer(width=self.SPACER_WIDTH)
                ui.CheckBox(width=20, tooltip=tooltip).model.add_value_changed_fn(do_not_show_changed)
                ui.Spacer(width=3)
                ui.Label("Do not display this window again when the Play button is pressed subsequently", tooltip=tooltip)

        def build_ok_button():
            with ui.HStack(height=0):
                ui.Spacer()
                ui.Button("OK", width=80, height=0).set_clicked_fn(self.hide)
                ui.Spacer()

        with self._window.frame:
            with ui.VStack(height=0, spacing=self.SPACER_HEIGHT):
                ui.Spacer(height=self.SPACER_HEIGHT)
                build_line(
                    "Fabric Scene Delegate does not support physics simulation - physics will be disabled!",
                    style={"color": self.WARNING_COLOR}
                )
                build_line("For physics to work please turn off Fabric Scene Delegate in render settings.")
                ui.Spacer(height=self.SPACER_HEIGHT)
                build_how_to_section()
                ui.Spacer(height=self.SPACER_HEIGHT)
                ui.Line(width=ui.Fraction(1))
                build_do_not_show_checkbox()
                build_ok_button()

    def show(self):
        global display_dialog
        self._window.visible = display_dialog

    def hide(self):
        self._window.visible = False


class StageUpdateFSDCheck():
    def __init__(self):
        self._stage_update_iface = None
        self._usd_context = None
        self._timeline_events = None
        self._timeline_event_sub = None
        self._detached = False

    def startup(self, stage_update_iface):
        self._stage_update_iface = stage_update_iface
        self._usd_context = omni.usd.get_context("")

        if omni_ui_loaded:
            try:
                self._timeline_events = omni.timeline.get_timeline_interface().get_timeline_event_stream()
                self._detached = False

                def on_timeline_event(e):
                    if e.type == int(omni.timeline.TimelineEventType.PLAY):
                        # check
                        if (
                            FSDCheckDialog.is_fsd_enabled()
                            and self._stage_update_iface.is_node_attached()
                            and FSDCheckDialog.is_physics_present()
                        ):
                            carb.log_error("Fabric Scene Delegate (FSD) does not support physics simulation, please disable FSD in render settings. Physics will be disabled.")
                            FSDCheckDialog().show()
                            self._detached = True
                            self._stage_update_iface.detach_node()
                    if e.type == int(omni.timeline.TimelineEventType.STOP):
                        if self._detached:
                            self._detached = False
                            self._stage_update_iface.attach_node()
                self._timeline_event_sub = self._timeline_events.create_subscription_to_pop(on_timeline_event)
            except:
                pass

    def shutdown(self):
        self._stage_update_iface = None
        self._usd_context = None
        self._timeline_events = None
        self._timeline_event_sub = None
        self._detached = False
