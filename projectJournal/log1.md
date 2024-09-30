# Project Log 1
This is the first entry in the project journal which is my attempt to document the process about which I learn and test
things in Isaac sim and my implementations within it. 

The benefit of using extensions is that you get the hot-reloading feature which means you don't have to restart the entire simulation and only the necessary modules with restart so that you can run.

Also, to use the gpu do the following:
1. launch isaac sim from the omniverse launcher
2. type in --/physics/cudaDevice=0 (or another number depending on the index.).
    - This should ensure that your code runs on the gpu which will make running it a lot faster, especially if you have parallel instances running. 

I found a good video on implementing an extension to generate scene with multiple robots:
- [Isaac Sim/Robotics Weekly Livestream: From Single to Multi Robot Environment](https://www.youtube.com/watch?v=DYWHN9Aym_Y&list=PL3jK4xNnlCVdi9t6KQq6Z-cwhiVXlJv2p&index=1).

Pretty much you just add a reference and then use the world.scene.add(prim, ... , etc.) to modify the attributes of the 
prim. This is how omniverse and Isaac offers an easier interface to the OpenUSD SW. You just put all the objects in the
_setup_scene() function within the [ui_builder.py](../extsUser/syntheticDataGen/omni_isaac_syntheticDataGen_python/ui_builder.py). 

[scenario.py](../extsUser/syntheticDataGen/omni_isaac_syntheticDataGen_python/scenario.py) is used for handling any of the logic at higher level ie. used the ui_builder.py to construct the scene and create attributes to self. Then call those attributes in the scenario and perform logic to run during the sim run.