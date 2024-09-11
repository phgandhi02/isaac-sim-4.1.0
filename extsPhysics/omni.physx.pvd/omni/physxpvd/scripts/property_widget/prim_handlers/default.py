from collections.abc import Iterable
from pxr import Usd
from functools import partial

from omni.physxpvd.scripts.property_widget.ui_helpers import (
    make_label,
    make_label_value,
    make_label_button,
)

from omni.physxpvd.scripts.property_widget.usd_helpers import (
    get_class_name,
    get_time,
    convert_from_camel_case,
    select_by_object_handle,
    OMNI_PVD_ATTRIBUTE_DATA_PREFIX,
    OMNI_PVD_ATTRIBUTE_HANDLE_PREFIX,
)

def prim_handler(prim: Usd.Prim) -> bool:
    if not prim.HasAttribute("omni:pvdi:class"):
        return False
    make_label(get_class_name(prim))

    for attribute in prim.GetAttributes():
        name = attribute.GetName()
        if name.startswith(OMNI_PVD_ATTRIBUTE_DATA_PREFIX):
            make_label_value(convert_from_camel_case(name, OMNI_PVD_ATTRIBUTE_DATA_PREFIX), attribute.Get(get_time()))
        if name.startswith(OMNI_PVD_ATTRIBUTE_HANDLE_PREFIX):
            value = attribute.Get(get_time())

            display_name = convert_from_camel_case(name, OMNI_PVD_ATTRIBUTE_HANDLE_PREFIX)
            
            if isinstance(value, Iterable):
                target_handles = value
                make_label(f'{display_name} (Ref List)')
                for i, target_handle in enumerate(target_handles):
                    if (i>10):
                        make_label(f'    List is truncated...')
                        break;
                    make_label_button(f'{display_name} [{i}]', 'Select...', partial(select_by_object_handle, target_handle), 1)
            else:
                target_handle = value
                make_label_button(f'{display_name} (Ref)', 'Select...', partial(select_by_object_handle, target_handle))

    return True
