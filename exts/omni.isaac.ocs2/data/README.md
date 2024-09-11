# OCS2 Mobile Manipulator

The `ocs2_mobile_manipulator` package supports various robotic arms and wheel-based mobile manipulators. The system model is determined by parsing the URDF and the task file.

You can find the complete information over: https://github.com/leggedrobotics/ocs2_robotic_assets/blob/main/docs/MobileManipulatorUrdfAssets.md

Over here, we summarize the steps involved in creating the URDF used for the examples.

## Franka Panda

* In the `src` directory of your catkin workspace, clone the official repository of the [Franka Panda](https://www.franka.de/):

```bash
git clone git@github.com:frankaemika/franka_ros.git
```

* Build the necessary packages and source the workspace:

```bash
catkin build franka_description ocs2_robotic_assets

source devel/setup.bash
```

* Convert the xacro file to urdf format:

```bash
rosrun xacro xacro -o $(rospack find ocs2_robotic_assets)/resources/mobile_manipulator/franka/urdf/panda.urdf $(rospack find franka_description)/robots/panda_arm.urdf.xacro hand:=true
```

* Copy all meshes from `franka_description` to `ocs2_robotic_assets/resources` directory:

```bash
cp -r $(rospack find franka_description)/meshes $(rospack find ocs2_robotic_assets)/resources/mobile_manipulator/franka/meshes
```

* Replace the meshes locations in the robot's urdf

```bash
sed -i 's+franka_description+ocs2_robotic_assets/resources/mobile_manipulator/franka+g' $(rospack find ocs2_robotic_assets)/resources/mobile_manipulator/franka/urdf/panda.urdf
```

* Add a dummy link "root" to the URDF (KDL prefers the root of the tree to have an empty-link):

```xml
  ...
  <!-- Root link -->
  <link name="root"/>
  <joint name="root_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="root"/>
    <child link="panda_link0"/>
  </joint>
  <!-- Robot Arm -->
  ...
```
