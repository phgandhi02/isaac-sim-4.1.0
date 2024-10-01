# Project Log 6

I have just figured out how to use git to make new branches off branches and so now I am trying to clean up the source control structure so that the extensions are in the extension branch which has several sub-branches for extensions that I am developing. This means that can keep the development separate and push when they are working. It took me a while to figure out but I think I've got it now. I've also realized that I've been tunnel-visioning hard and so I've been spinning out into circles which is why I haven't made much progress. I need to just get a working demo together, hopefully, today. If I don't then it's not a big deal but I would like to do that so then I can relax a bit.

I need to get something together so that Andrew can apply what he is learning in ROS towards something. I am thinking about doing the simulation in Isaac Sim though it might take time to develop so I am worried that I should just start with Gazebo and then transition to Isaac Sim. That might be the best option although we will see later this weekend.

## Plan for Today:
- Go to class
- Tasking for today:
    - Create a scene with teeth on a conveyor, a UR robot, and get the footage from the camera and publish to a ROS2 topic. Save the images.
        1. Use existing script to load in teeth with random positions and orientations in grid fashion. Refer to example Scene.
        1. load in scene with ur5e manipulator, conveyor belt, and stand. Use existing usd file to do this.
        1. create a camera_to_ROS_topic action graph using the Isaac Sim workflow to publish images to ROS2. Add to base scene usd file.
        1. create action graph to enable read and write joint states from/to UR5e in Isaac Sim. Add to base scene usd file.
        1. Use MoveIt2 to generate joint states for UR5e based on positions I set.
        1. Use Rviz2 to change the position of the UR5e which MoveIt2 will solve. 
        1. Save the images with whatever method is easiest.