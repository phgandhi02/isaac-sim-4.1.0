# omni.isaac.benchmarks

## Usage

To enable this extension, go to the Extension Manager menu and enable omni.isaac.benchmarks extension.

## How to Read the Perflab Reports

### Test Name

Indicates the name of the test. Most tests are run with multiple parameters, so the name of the test identifies the test and the parameters used.

For the camera tests, the parameters are:

- number of cameras (1, 2, 4, or 8)
- camera resolution (1280x720 or 1920x1080)

For the physx lidar tests, the parameter is the number of lidars (1, 5, 10, 25, 50, or 100).

For the real time factor test, there is only one test and no parameters.

For the carter robot tests, the parameters are:

- number of robots (1, 5, 10, 25, or 50)
- what sensors are enabled on the robots (lidar, camera, both, or none)
Note: if there are any sensors enabled, there is no test for 25 or 50 robots, as the number of render products is too high.

For the o3dyn robot tests, the parameters are:

- number of robots (1, 5, 10, 25, or 50)

For the ros camera tests, the parameters are:

- number of cameras (1, 2, 4, or 8)
- camera resolution (1280x720 or 1920x1080)

For the rtx lidar tests, the parameter is the number of lidars (1, 2, 4, or 8).

For the scene_generation tests, the parameters are:

- number of assets (100 or 1000)
- asset spawn location:
  - origin - no additional transform attributes are applied
  - random - scattered in a predefined volume with a fixed random seed for repeatability
- prim type:
  - xform - base prim without visuals, spawn location is kept at origin since it does not influence rendering (lights/shadows)
  - mesh - spawned either in origin (no additional transform) or randomized in a volume (with or without additional lights)
- api: 
  - usd - direct call to USD's DefinePrim + AddReference
  - isaac - using a helper function that first checks if a prim with the same name already exists in stage
- number of lights (0 or 8) - used as additional parameters for mesh assets spawned in a scattered area

For the sdg_generation tests, the parameters are:

- number of cameras (1, 2, or 4)
- camera resolution (1280x720, 128x128, 256x256, or 512x512) - if the camera resolution is not 1280x720, there is only 1 camera
- annotators (rgb or all) - please see <https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator/annotators_details.html> for more information on annotators

### Segment

Tests will typically be broken up into segments, like Loading and Benchmark (loading all assets and then running the benchmark), each with their own measured metrics. Some tests, like scene_generation, have more segments. See:

For the camera, carter robots, o3dyn robots, physx lidars, rtx lidars, ros camera, and real time factor tests, the segments are:

- Loading: Load all assets and initialize the simulation.
- Benchmark: Run the simulation for a fixed amount of time.

For the scene_generation test, the segments are:

- Spawn: Load all assets using Python code (reference and load each asset individually).
- Play: Run the simulation for a fixed amount of time.
- Load: open saved stage of the spawned assets as a usda file

For the sdg_generation test, the segments are:

- Loading: Load all assets and initialize the simulation.
- Baseline: Run the simulation for a fixed amount of time, with just the stage & assets, and no additional features.
- Baseline_SDG: Run the simulation for a fixed amount of time, but with render products (cameras) and the BasicWriter loaded. The render products are not actually being written in this segment, but the BasicWriter is loaded and initialized.
- Benchmark: Run the simulation for a fixed amount of time, with render products (cameras) and the BasicWriter loaded, and the render products being written to disk.
- Baseline_Cleanup: Run the simulation for a fixed amount of time, but with the writer now detached. The render products are loaded but not being written.

### Hardware

- Indicates whether the GPU or Render Thread data is shown
  - Render Thread refers to the CPU, which is primarily used to measure the total time to do an app update (presumably, one frame)
  - GPU data is measuring only the time taken to render that frame on the GPU
  - The difference of these two times is any CPU overhead that was not overlapped by the GPU
- The GPU data is typically the most important, as it is more likely to be the bottleneck when simulating robots, sensors, and/or other render products
