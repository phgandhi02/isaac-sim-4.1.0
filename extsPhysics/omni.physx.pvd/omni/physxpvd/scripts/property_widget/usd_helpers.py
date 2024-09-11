from re import sub
from omni.usd import get_context
from omni.timeline import get_timeline_interface

OMNI_PVD_ATTRIBUTE_DATA_PREFIX = 'omni:pvd:'
OMNI_PVD_ATTRIBUTE_HANDLE_PREFIX = 'omni:pvdh:'
OMNI_PVD_METADATA_PREFIX = 'omni:pvdi:'

def get_time():
    timeline = get_timeline_interface()
    return timeline.get_current_time() * timeline.get_time_codes_per_seconds()

def select_by_object_handle(target_handle):
    #print("doing the select")
    for prim in get_context().get_stage().TraverseAll():
        traversing_handle = get_object_handle(prim)
        if not traversing_handle:
            continue
        if int(traversing_handle) != int(target_handle):
            continue
        # should not use the visibility attribute but an activity attribute instead
        #visibility_attribute = prim.GetAttribute('visibility')
        #if not visibility_attribute:
        #    continue
        #if visibility_attribute.Get(get_time()) != 'inherited':
        #    continue
        #print("le found the object, selecting it")
        get_context().get_selection().set_selected_prim_paths([str(prim.GetPath())], True)
        return

def get_value_with_prefix(prim, name, prefix):
    name_with_prefix = prefix + name
    if not prim.HasAttribute(name_with_prefix):
        return None
    return prim.GetAttribute(name_with_prefix).Get(get_time())

def get_value(prim, name):
    return get_value_with_prefix(prim, name, OMNI_PVD_ATTRIBUTE_DATA_PREFIX)

def get_class_name(prim):
    return get_value_with_prefix(prim, 'class', OMNI_PVD_METADATA_PREFIX)

def get_object_handle(prim):
    return get_value_with_prefix(prim, 'handle', OMNI_PVD_METADATA_PREFIX)

def convert_from_camel_case(name, prefix):
    name = name[len(prefix):]
    words = sub('([A-Z][a-z]+)', r' \1', sub('([A-Z]+)', r' \1', name)).split()
    return ' '.join([word[0].upper() + word[1:] for word in words])
