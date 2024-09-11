import omni.kit.commands
import omni.kit.undo
import omni.physxdemos as demo
import omni.usd
import os
import os.path
import asyncio
# from threading import Timer


# helper to load a usd file
def _open_test_usd_file(stage, filename):
    schema_folder = os.path.abspath(os.path.normpath(os.path.join(__file__, "../../../../../data/usd/assets")))
    schema_folder = schema_folder.replace("\\", "/") + "/"
    full_filename = schema_folder + filename + ".usd"
    # print(full_filename)
    exists = os.path.exists(full_filename)
    assert(exists)
    context = omni.usd.get_context()
    context.open_stage(full_filename)

