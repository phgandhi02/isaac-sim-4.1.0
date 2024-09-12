import os
from pathlib import Path

from isaacsim import SimulationApp

kit = SimulationApp()

import carb

for _ in range(10):
    kit.update()

# get the current output path and check if the file exists
pvd_output_dir = carb.settings.get_settings().get_as_string("/persistent/physics/omniPvdOvdRecordingDirectory")

print("omniPvdOvdRecordingDirectory: ", pvd_output_dir)
my_file = Path(os.path.join(pvd_output_dir, "tmp.ovd"))
assert my_file.is_file()

kit.close()
