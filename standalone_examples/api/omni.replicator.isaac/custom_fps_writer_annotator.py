# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import os

import carb.settings
import omni.kit.app
import omni.replicator.core as rep
import omni.timeline
import omni.usd

# NOTE: To avoid FPS delta misses make sure the sensor framerate is divisible by the timeline framerate
STAGE_FPS = 60.0
SENSOR_FPS = 10.0
SENSOR_DT = 1.0 / SENSOR_FPS


def run_custom_fps_example(num_frames=10):
    # Create a new stage
    omni.usd.get_context().new_stage()

    # Disable capture on play (data will only be accessed at custom times)
    carb.settings.get_settings().set("/omni/replicator/captureOnPlay", False)

    # Set the timeline parameters
    timeline = omni.timeline.get_timeline_interface()
    timeline.set_looping(False)
    timeline.set_current_time(0.0)
    timeline.set_end_time(10)
    timeline.set_time_codes_per_second(STAGE_FPS)
    timeline.play()
    timeline.commit()

    # Create a light and a semantically annoated cube
    rep.create.light()
    rep.create.cube(semantics=[("class", "cube")])

    # Create a render product and disable it (it will re-enabled when data is needed)
    rp = rep.create.render_product("/OmniverseKit_Persp", (512, 512), name="rp")
    rp.hydra_texture.set_updates_enabled(False)

    # Create a writer and an annotator as different ways to access the data
    out_dir_rgb = os.getcwd() + "/_out_writer_fps_rgb"
    print(f"Writer data will be written to: {out_dir_rgb}")
    writer_rgb = rep.WriterRegistry.get("BasicWriter")
    writer_rgb.initialize(output_dir=out_dir_rgb, rgb=True)
    # NOTE: 'trigger=None' is needed to make sure the writer is only triggered at the custom schedule times
    writer_rgb.attach(rp, trigger=None)
    annot_depth = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
    annot_depth.attach(rp)

    # Run the simulation for the given number of frames and access the data at the desired framerates
    previous_time = timeline.get_current_time()
    elapsed_time = 0.0
    for i in range(num_frames):
        current_time = timeline.get_current_time()
        elapsed_time += current_time - previous_time
        print(f"[{i}] current_time={current_time:.4f}, elapsed_time={elapsed_time:.4f}/{SENSOR_DT:.4f};")

        # Check if enough time has passed to trigger the sensor
        if elapsed_time >= SENSOR_DT:
            # Reset the elapsed time with the difference to the optimal trigger time (when the timeline fps is not divisible by the sensor framerate)
            elapsed_time = elapsed_time - SENSOR_DT

            # Enable render products for data access
            rp.hydra_texture.set_updates_enabled(True)

            # Write will be scheduled at the next step call
            writer_rgb.schedule_write()

            # Step needs to be called after scheduling the write
            rep.orchestrator.step(delta_time=0.0)

            # After step, the annotator data is available and in sync with the stage
            annot_data = annot_depth.get_data()
            print(f"\tWriter triggered and annotator data shape={annot_data.shape};")

            # Disable render products to avoid unnecessary rendering
            rp.hydra_texture.set_updates_enabled(False)

            # Restart the timeline if it has been paused by the replicator step function
            if not timeline.is_playing():
                timeline.play()

        previous_time = current_time
        # Advance the app (timeline) by one frame
        simulation_app.update()

    # Make sure the writer finishes writing the data to disk
    rep.orchestrator.wait_until_complete()


# Run the example for a given number of frames
run_custom_fps_example(num_frames=50)

# Close the application
simulation_app.close()
