from omni import ui

LABEL_WIDTH = 200
LABEL_HEIGHT = 18
INDENT = '    '

def make_label(name, indent_level=0):
    with ui.HStack():
        ui.Label(f'{indent_level * INDENT}{name}', name="label", word_wrap=True, width=LABEL_WIDTH, height=LABEL_HEIGHT)

def make_label_value(name, value, indent_level=0):
    with ui.HStack():
        ui.Label(f'{indent_level * INDENT}{name}', name="label", word_wrap=True, width=LABEL_WIDTH, height=LABEL_HEIGHT)
        ui.StringField(name="models").model.set_value(str(value) if value != 3.4028234663852885981170418348452e+38 else 'PX_MAX_F32')

def make_label_button(name, value, callback, indent_level=0):
    with ui.HStack():
        ui.Label(f'{indent_level * INDENT}{name}', name="label", word_wrap=True, width=LABEL_WIDTH, height=LABEL_HEIGHT)
        ui.Button(text=str(value), width=LABEL_WIDTH, height=LABEL_HEIGHT, clicked_fn=callback)
