import omni.ui


DEFAULT_SPACING_H = 10
DEFAULT_SPACING_V = 5
DEFAULT_MARGIN_TOOLTIP = 5
DEFAULT_WINDOW_PADDING_X = 10
DEFAULT_WINDOW_PADDING_Y = 10
DEFAULT_WINDOW_SPACING_V = 10

STYLE_REFERENCE = {
    "CollapsableFrame": {
        "margin": 0,
        "padding": 10,
        "border_width": 1,
        "border_color": 0xFF555555
    },

    "Button": {
        "padding": 5
    }
}


def get_selected_string_and_index(model):
    selectedIndex = model.get_item_value_model().as_int
    selectedItem = model.get_item_children()[selectedIndex]
    selectedString = model.get_item_value_model(selectedItem).as_string
    return (selectedString, selectedIndex)


#
# note: Could discuss whether to move some of this to the pyhsx.ui extension and add a dependency
#       or even whether to move the vehicle stuff into pyhsx.ui where possible.
#

def create_group_layout(name):
    with omni.ui.CollapsableFrame(name, height = 0):
        vStack = omni.ui.VStack(spacing = DEFAULT_SPACING_V)
        return vStack


def create_wrapped_label(name, tooltipText = "", width = 0, tooltipWidth = 0):
    label = omni.ui.Label(name, word_wrap = True, width = width)

    # tooltip could be set in omni.ui.Label directly but the text will have no padding
    # which does look somewhat crappy. Thus, creating custom tooltip widget.
    if (tooltipText):
        def create_tooltip():
            omni.ui.Label(tooltipText, word_wrap = True, width = tooltipWidth,
                style = {"margin": DEFAULT_MARGIN_TOOLTIP})  # "padding" did not work

        label.set_tooltip_fn(create_tooltip)

    return label


def _get_width_from_width_list_max_2(widthList):
    width0 = 0
    width1 = 0
    widthListSize = len(widthList)
    if (widthListSize == 1):
        width0 = widthList[0]
        width1 = widthList[0]
    elif (widthListSize > 1):
        width0 = widthList[0]
        width1 = widthList[1]

    return (width0, width1)


def create_checkbox_with_label(layout, value, onChangeFn=None,
    labelText="", tooltipText="", widthList=[], tooltipWidth = 0):

    with layout:
        with omni.ui.HStack(height = 0, spacing = DEFAULT_SPACING_H):
            (widthLabel, widthCheckbox) = _get_width_from_width_list_max_2(widthList)

            label = create_wrapped_label(labelText, tooltipText, width = widthLabel, tooltipWidth = tooltipWidth)

            checkbox = omni.ui.CheckBox(width = widthCheckbox)
            checkbox.model.set_value(value)

            onChangeFnHandle = None
            if (onChangeFn is not None):
                onChangeFnHandle = checkbox.model.add_value_changed_fn(onChangeFn)

            return (checkbox, label, onChangeFnHandle)


def create_float_drag_with_label(layout, value, minValue, maxValue, step, onChangeFn=None,
    labelText="", tooltipText="", widthList=[], tooltipWidth = 0):

    with layout:
        with omni.ui.HStack(height = 0, spacing = DEFAULT_SPACING_H):
            (widthLabel, widthDrag) = _get_width_from_width_list_max_2(widthList)

            label = create_wrapped_label(labelText, tooltipText, width = widthLabel, tooltipWidth = tooltipWidth)

            drag = omni.ui.FloatDrag(min = minValue, max = maxValue, step = step, width = widthDrag)
            drag.model.set_value(value)

            onChangeFnHandle = None
            if (onChangeFn is not None):
                onChangeFnHandle = drag.model.add_value_changed_fn(onChangeFn)

            return (drag, label, onChangeFnHandle)


def create_int_drag_with_label(layout, value, minValue, maxValue, step, onChangeFn=None,
    labelText="", tooltipText="", widthList=[], tooltipWidth = 0):

    with layout:
        with omni.ui.HStack(height = 0, spacing = DEFAULT_SPACING_H):
            (widthLabel, widthDrag) = _get_width_from_width_list_max_2(widthList)

            label = create_wrapped_label(labelText, tooltipText, width = widthLabel, tooltipWidth = tooltipWidth)

            drag = omni.ui.IntDrag(min = minValue, max = maxValue, step = step, width = widthDrag)
            drag.model.set_value(value)

            onChangeFnHandle = None
            if (onChangeFn is not None):
                onChangeFnHandle = drag.model.add_value_changed_fn(onChangeFn)

            return (drag, label, onChangeFnHandle)


def create_combo_box_with_label(layout, entryList=[], selectedIndex=0, onChangeFn=None, 
    labelText="", tooltipText="", widthList=[], tooltipWidth = 0):

    with layout:
        with omni.ui.HStack(height = 0, spacing = DEFAULT_SPACING_H):

            (widthLabel, widthComboBox) = _get_width_from_width_list_max_2(widthList)

            label = create_wrapped_label(labelText, tooltipText, width = widthLabel, tooltipWidth = tooltipWidth)

            comboBox = omni.ui.ComboBox(selectedIndex, *entryList, width = widthComboBox)  # unpack list into arguments

            onChangeFnHandle = None
            if (onChangeFn is not None):
                onChangeFnHandle = comboBox.model.add_item_changed_fn(onChangeFn)

            return (comboBox, label, onChangeFnHandle)


def create_button_with_label(layout, onClickedFn=None, buttonText="", labelText="", tooltipText="",
    widthList=[], tooltipWidth = 0):

    with layout:
        with omni.ui.HStack(height = 0, spacing = DEFAULT_SPACING_H):

            (widthLabel, widthButton) = _get_width_from_width_list_max_2(widthList)

            label = create_wrapped_label(labelText, tooltipText, width = widthLabel, tooltipWidth = tooltipWidth)

            button = omni.ui.Button(buttonText, width = widthButton, clicked_fn = onClickedFn)

            return (button, label)
