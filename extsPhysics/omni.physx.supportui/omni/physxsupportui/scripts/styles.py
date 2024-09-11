import omni.ui as ui


class Styles:
    BUTTON_SIZE_H = 34
    BUTTON_SIZE_V = BUTTON_SIZE_H
    BUTTON_STYLE = {
        "Button.Label": {"alignment": ui.Alignment.CENTER_BOTTOM, "font_size": 12},
        "Button": {"background_color": 0x0, "margin": 1, "padding": 2},
        "Button:hovered": {"background_color": 0xFF373737},
        "Button:checked": {"background_color": 0xFF23211F}
    }
    SEPARATOR_COLOR = 0xFF6F6F6F
    DISABLED_COLOR = 0xFF555555
