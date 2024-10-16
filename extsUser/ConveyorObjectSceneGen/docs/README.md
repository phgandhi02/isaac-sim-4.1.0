## ConveyorObjectSceneGenExt

### Description: 
This extension is used to create a scene with a conveyor, object, and background. It will load a robotic manipulator and spawn in an asset of choice in grid fashion with random positions and orientations across the conveyor belt. 

### Topic Names:
1. /isaac_joint_commands - topic to receive the joint states of the manipulator in the simulation.
2. /isaac_joint_states - topic to publish the joint states of the manipulator in the simulation.
3. /rgb - topic publishing rbg images from the camera attached to the end-effector of the manipulator.
4. /depth - topic publishing depth images from the camera attached to the end-effector of the manipulator.

### Data:
This folder includes any of the necessary assets that come with the extension. By default the extension comes with a mesh asset of a mold of teeth. This serves as an example asset that can be generated along the conveyor belt. It also includes a conveyor asset, warehouse scene asset, and manipulator asset. 

### Usage
To enable this extension, run Isaac Sim with the flags --ext-folder {path_to_ext_folder} --enable {ext_directory_name}

### Setup:
1. Open Isaac Sim (optional: when the app selector menu opens type `--/physics/cudaDevice=0` in the extra-args field to use CUDA) and click start in app selector menu. This may take a min. Please ignore the warning that pops us and click wait.
2. Click the window button the topmost ribbon menu. Then click the extensions option to open the extensions menu.
3. Type `ConveyorObjectSceneGenExt` in the search bar and it will show the under the Nvidia menu. If not check the Third party menu. Enable the extension by click the toggle button.
4. Close the extensions window. In the topmost ribbon menu, the extension name will appear as an option.
5. Click the `ConveyorObjectSceneGenExt` ribbon and select the option underneath named `ConveyorObjectSceneGenExt`. This will open another window with a load and reset button.
6. Click the load button and wait for the scene to generate.
7. Click the run button to start the simulation.
Note: The parameters for the camera may need to be changed based on user preference.