# Changelog
## [2.31.4] - 2024-07-17
### Added
- added a joint state subscriber test where the jointNames are connected

## [2.31.3] - 2024-07-12
### Removed
- IsaacReadSimulationTime removed from RTXLidarROS2PublishFlatScan, timestamp now comes from
  OgnIsaacComputeRTXLidarFlatScan.

## [2.31.2] - 2024-07-09
### Changed
- Updates default (undistorted) values for CameraInfo distortion_model, d fields

## [2.31.1] - 2024-07-08
### Changed
- Use a per instance variable for publish_multithreading_disabled

## [2.31.0] - 2024-07-05
### Changed
- use register_*_with_telemetry functions to simplify registration code
- use OgnIsaacPassthroughImagePtr node for depth
- switch to using ptr for depth data to avoid extra cpu copy

## [2.30.0] - 2024-07-03
### Added
- OgnIsaacROS2CameraInfoHelper node to generate CameraInfo message with correct rectification parameters for stereo cameras
- camera_info_utils.py module contains utility functions for generating CameraInfo message
### Changed
- OgnIsaacROS2CameraHelper:sensor_type == camera_info deprecated in favor of OgnIsaacROS2CameraInfoHelper. This node will use camera_info_utils.py module until the functionality is fully removed in a future release.

## [2.29.0] - 2024-06-27
### Changed
- Unify ROS 2 backend implementations

## [2.28.1] - 2024-06-25
### Fixed
- JointState Subscriber fills in nans when receiving empty arrays for command modes that are not used
### Added
- jointstate subscriber test for mixed controls

## [2.28.0] - 2024-06-17
### Added
- Setting publish_multithreading_disabled for toggling ROS2 publishers to publish without multithreading

## [2.27.0] - 2024-06-14
### Changed
- Moved and renamed Navigation example in the menu to under Isaac Examples -> ROS2 -> Navigation -> Carter Navigation

## [2.26.5] - 2024-06-11
### Fixed
- RTX laser_scan topic name in rtx_lidar standalone example

## [2.26.4] - 2024-05-29
### Fixed
- Crash on stage close if twistsubscriber was not run for a single frame

## [2.26.3] - 2024-05-23
### Fixed
- Extra warnings when nitros bridge message type was not found

## [2.26.2] - 2024-05-23
### Fixed
- Fix references to non-existing node in the MoveIt example

## [2.26.1] - 2024-05-21
### Added
- Resetting output of the Twist Subscriber to zeros upon stopping simulation.

## [2.26.0] - 2024-05-20
### Changed
- Foxy support is deprecated

## [2.25.3] - 2024-05-20
### Added
- RTX camera and lidar omnigraph shortcuts uses RunOneSimulationFrame

## [2.25.2] - 2024-05-17
### Changed
- default subscriber topic in ROS2 jointstate omnigraph shortcut uses /joint_command and not /cmd

## [2.25.1] - 2024-05-16
### Changed
- Fixed Nova Carter multiple robot navigation and Carter Stereo standalone examples from crashing

## [2.25.0] - 2024-05-16
### Added
- Isaac ROS NITROS Bridge integration for publishing images on Linux

## [2.24.1] - 2024-05-15
### Fixed
- Bug with memcpy when using texture memory

## [2.24.0] - 2024-05-09
### Changed
- OgnROS2PublishImage does gpu->cpu copy on a separate cuda stream to improve performance

## [2.23.8] - 2024-05-08
### Fixed
- OgnROS2PublishLaserScan angle inputs now provided in degrees rather than mixed degrees/radians

## [2.23.7] - 2024-05-08
### Fixed
- Automated ROS2 bridge tests on Windows

## [2.23.6] - 2024-05-04
### Fixed
- Rclpy loading on Windows

## [2.23.5] - 2024-05-01
### Fixed
- ROS2 bridge Service printing unnecessary warnings

## [2.23.4] - 2024-04-30
### Fixed
- ROS2 bridge check failing on humble due to missing libraries

## [2.23.3] - 2024-04-26
### Fixed
- Error when service server and client nodes started with blank message definition

## [2.23.2] - 2024-04-26
### Fixed
- ROS2 bridge check failing on humble due to missing libraries

## [2.23.1] - 2024-04-25
### Fixed
- Group ROS2 Subscriber input attributes to match the ROS2 Publisher OGN style

## [2.23.0] - 2024-04-24
### Changed
- Stereo offset input in ROS Camera Info and Helper nodes to accept camera baseline in meters used to compute Tx, Ty.

## [2.22.4] - 2024-04-20
### Fixed
- Loading Foxy internal libraries

## [2.22.3] - 2024-04-19
### Fixed
- Registration for system time writers with telemetry tracking

## [2.22.2] - 2024-04-18
### Fixed
- Update IStageUpdate usage to fix deprecation error

### Added
- Telemetry for writers and annotators

## [2.22.1] - 2024-04-18
### Fixed
- Removed unnecessary imports from a unit test

## [2.22.0] - 2024-04-17
### Added
- Generic Omnigraph ROS client and server nodes

## [2.21.0] - 2024-04-17
### Added
- Implemented static TF publishing in OgnROS2PublishTransformTree

## [2.20.0] - 2024-04-16
### Added
- New QoSProfile node to create any preset or custom qos profile to use with ROS2 Omnigraph nodes
- QoS Profile inputs to all ROS2 Publisher, Subscriber, Service Omnigraph nodes

## [2.19.4] - 2024-04-16
### Fixed
- Prevent ROS2 Service backend from printing unnecessary errors while waiting for service request

## [2.19.3] - 2024-04-15
### Added
- Unit test for jointstate publisher and subscriber

### Fixed
- Fix ROS2 Service Prim service availability when resetting the simulation

## [2.19.2] - 2024-04-15
### Fixed
- Fixed flaky ROS2 rtx lidar unit test

## [2.19.1] - 2024-04-15
### Fixed
- Memory leak when ROS 2 libs were not loaded properly

### Added
- A button to open documentation links for OmniGraph shortcuts

## [2.19.0] - 2024-04-11
### Added
- Added OmniGraph shortcuts for sample ROS2 generic publisher node
- Add frameSkipCount option to ROS2  RTX Lidar and Camera Helper Nodes to modify the rate of lidar and point cloud publishing

## [2.18.0] - 2024-04-09
### Added
- Added quality of service support for ROS2 bridge subscribers and publishers

## [2.17.0] - 2024-04-04
### Added
- Option to use system timestamp for ROS2 RTX Lidar and ROS2 Camera Helper Nodes

### Changed
- Updated ROS2 Camera and RTX Lidar unit tests for publishing using system timestamp

## [2.16.0] - 2024-04-04
### Added
- Generic ROS2 Publisher node
- Isaac Sim now supports CycloneDDS (Linux ROS2 Humble only)

## [2.15.1] - 2024-04-03
### Fixed
- Depth not publishing after stop -> play

## [2.15.0] - 2024-04-03
### Changed
- Improve dynamic message API and add support for JSON data

## [2.14.1] - 2024-04-02
### Fixed
- Error on startup due to missing libraries is now a warning, prevents errors on startup for users that don't want to use ROS2 by default.

## [2.14.0] - 2024-04-02
### Added
- ROS2 Service Prim node to list prims and their attributes, as well as read and write a specific attribute

## [2.13.3] - 2024-04-02
### Changed
- used IsaacCreateRenderProduct for the camera OG shortcuts
- default to enable semanticlabels for some camera topics

## [2.13.3] - 2024-04-02
### Changed
- Improved error logging in ROS2 bridge

## [2.13.2] - 2024-03-26
### Fixed
- Fix dynamic message typesupport and introspection for actions

## [2.13.1] - 2024-03-20
### Added
- OG shortcuts to RTX lidar, odometry, tf pub
- node namespace options for all the shortcuts
- popup notifications instead of print statements for errors in OG shortcuts

## [2.13.0] - 2024-03-19
### Added
- New folder under python/scripts/ for omnigraph shortcuts
- Menu shortcuts to generate ROS Omnigraphs
- Shortcuts for ROS clock, jointstate publisher and subscriber, and camera

## [2.12.1] - 2024-03-18
### Changed
- Updated the stdoutFailPatterns.exclude to ignore articulation related error

## [2.12.0] - 2024-03-15
### Added
- Added ROS2SubscribeTransformTree omnigraph node
- New folder under python/scripts/ for omnigraph shortcuts
- Menu shortcuts to generate ROS Omnigraphs
- Shortcuts for ROS clock, jointstate publisher and subscriber, and camera

## [2.11.0] - 2024-03-13
### Added
- ROS2 service
- Dynamic ROS2 message for services and actions

## [2.10.2] - 2024-03-12
### Added
- Added differential base unit test for nova carter

## [2.10.1] - 2024-03-12
### Fixed
- Fixed logging in ROS2 bridge backend
- Multi GPU support

## [2.10.0] - 2024-03-11
### Changed
- Improved Image publisher performance
- Generic ROS2 Subscriber node

## [2.9.3] - 2024-03-05
### Fixed
- RGB being published twice per frame

## [2.9.2] - 2024-03-04
### Changed
- Updated omnigraph nodes to use per instance state instead of internal state

## [2.9.1] - 2024-02-29
### Fixed
- Fixed edge case crash in ROS2 subscribers

## [2.9.0] - 2024-02-29
### Added
- Carb setting publish_without_verification for toggling ROS2 publishers to publish regardless of subscription count on a topic

## [2.8.4] - 2024-02-28
### Added
- Added Show Debug View flag to RTX Lidar Helper


## [2.8.3] - 2024-02-24
### Changed
- Update source code to run a build clean of warnings

## [2.8.2] - 2024-02-07
### Changed
- Updated path to the nucleus extension

## [2.8.1] - 2024-02-02
### Changed
- Added profile zones for the image publisher

## [2.8.0] - 2024-01-23
### Changed
- ROS2 Bridge no longer requires the vision_msgs for activation. However, if you would like to use Isaac Sim publishers dependant on vision_msgs, simply install it on the system or use isaac sim internal libraries.

## [2.7.1] - 2024-01-19
### Fixed
- Missing DLLs for local windows usage
- empty semantics test

## [2.7.0] - 2024-01-19
### Changed
- Renamed "Isaac Examples > ROS" menu to "Isaac Examples > ROS2"
## [2.6.2] - 2024-01-18
### Changed
- Changed get_assets_root_path to get_assets_root_path_async for the unit tests

## [2.6.1] - 2024-01-03
### Changed
- Updated samples and unit tests with newer OgnIsaacArticulationController node

## [2.6.0] - 2024-01-02
### Added
- Publishers now check to see topic subscription count before filling data and publishing.

## [2.5.2] - 2023-12-20
### Added
- Lidar and pointcloud tests with replicator enabled

## [2.5.1] - 2023-12-08
### Fixed
- Removed conflicting TF Publishers from standalone ROS2 Moveit tutorial

## [2.5.0] - 2023-11-28
### Added
- ROS2 Ackermann Subscriber and Publisher nodes using non-default ROS2 AckermannDriveStamped message type.

## [2.4.0] - 2023-11-27
### Added
- Option to publish a full scan from RTX lidar
- APIs to dynamically load ROS2 libraries at runtime depending on what messages are being created. Will only be used for select non-default ROS 2 messages.

## [2.3.10] - 2023-11-20
### Fixed
- Queue Size (QOS depth) settings now enforced when creating subscriber nodes.
- Added unit tests for variable queue sizes in subscribers

## [2.3.9] - 2023-10-17
### Fixed
- Added fix for foxy backend to match humble backend

## [2.3.8] - 2023-10-13
### Fixed
- CameraInfo publisher now includes identity Rectification matrix

## [2.3.7] - 2023-10-11
### Changed
- Disable extension if rclpy cannot load

## [2.3.6] - 2023-10-09
### Fixed
- Joint State Subscriber supports input state messages with only one mode of drive specified
- camera noise example errors

## [2.3.5] - 2023-10-09
### Fixed
- Moveit Tutorial had incorrect OG input name

## [2.3.4] - 2023-10-07
### Changed
- Set ROS2 Context node to read ROS_DOMAIN_ID from env vars by default

## [2.3.3] - 2023-10-05
### Fixed
- Bug with  ROS2 windows loading internal libs

## [2.3.2] - 2023-10-04
### Added
- Tests for JointState Publisher
### Fixed
- Carter 1 Unit tests
- JointState Publisher sign correction when a joint's parents are in reversed order

## [2.3.1] - 2023-10-03
### Fixed
- Windows message for setting PATH
## [2.3.0] - 2023-09-27
### Added
- Added enable flag for the lidar and camera helper
- Improved description for the frame ID for the camera and lidar helper
- Add menu item for Isaac ROS VSLAM tutorial
## [2.2.3] - 2023-09-27

### Changed
- Made several warnings into print and log statements

### Fixed
- rclpy not working in script editor

## [2.2.2] - 2023-09-25
### Fixed
- Transform tree node can accept multiple target prims as input

## [2.2.1] - 2023-09-22
### Fixed
- missing ros2 libs when running using internal libs
- windows startup issues
- rclpy not working on windows
- clock subscriber not properly getting time

## [2.2.0] - 2023-09-20
### Changed
- Load internal ROS2 libs when ROS2 is not sourced
- Print warning telling user how to source ROS2 to use internal libs with rclpy


## [2.1.0] - 2023-09-18
### Fixed
- error when shutting down extension
- properly handle case where ROS2 bridge carb plugin did not startup correctly

### Changed
- load system rclpy, fallback onto internal rclpy if its not found

## [2.0.0] - 2023-09-06
### Changed
- Re-written to use rclc APIs
- User must source their local ROS2 workspace in the terminal before starting isaac sim
- Foxy and humble are supported
- The ROS_DISTRO env variable is used to determine what ROS backend to use
- The separate Humble bridge extension has been removed

## [1.13.3] - 2023-08-25
### Changed
- added stdout fail pattern for the expected no prim found edge case for the ogn test
- Changed test_camera test to expect identical fx and fy as vertical aperture is computed from horizontal aperture.

## [1.13.3] - 2023-08-25
### Fixed
- Add default values for World transform attributes when fetching fabric in computeWorldXformNoCache function used in processAllFrames
## [1.13.2] - 2023-08-18
### Fixed
- Pixel are square in 105.1 Vertical aperture is not used, and based off of horizontal

## [1.13.1] - 2023-08-10
### Changed
- Changed omnigraph targetPrim types from bundle to target
## [1.13.0] - 2023-08-03
### Changed
- Image and PCL publishers to use ptrs instead of arrays when possible to reduce memory copies

## [1.12.4] - 2023-07-05
### Fixed
- Joint efforts not being applied in joint state subscriber

## [1.12.3] - 2023-06-12
### Changed
- Update to kit 105.1
- rename synthetic data writer templates to match rendervar names

## [1.12.2] - 2023-03-13
### Fixed
- Incorrect 2D, 3D bbox data
- Bug with RTX Lidar not publishing flatscan correctly
- invalid json string if semantic message was empty


## [1.12.1] - 2023-03-09
### Changed
- Modified Omnigraph for Moveit example to match tutorials provided by Moveit2
### Fixed
- Negative bbox sizes

## [1.12.0] - 2023-02-28
### Fixed
- Issue where image based messages always used increasing simulation time
### Added
- resetSimulationTimeOnStop to Camera Helper node.

## [1.11.2] - 2023-02-21
### Fixed
- Camera and RTX lidar helper settings are updated on stop/play

## [1.11.1] - 2023-02-01
### Fixed
- Changed semanticId in PublishBbox2d and 3d from int to string.

## [1.11.0] - 2023-01-31
### Added
- Fabric support

## [1.10.5] - 2023-01-25
### Fixed
- remove un-needed cpp ogn files from extension

## [1.10.5] - 2023-01-21
### Fixed
- TF_Tree fix when multiple objects shared the same prim name using the isaac:nameOverride attribute

## [1.10.4] - 2023-01-20
### Fixed
- Tests use a fixed QOS profile to improve determinism
## [1.10.3] - 2023-01-09
### Fixed
- Crash when using an old context handle

## [1.10.2] - 2023-01-06
### Fixed
- onclick_fn warning when creating UI

## [1.10.1] - 2022-12-12

### Fixed
- Errors when switching between ROS2 and ROS2 humble bridges

## [1.10.0] - 2022-12-10

### Changed
- Switch publishing nodes to use replicator writer backend
## [1.9.0] - 2022-11-21

### Added
- rtx_lidar_helper Node

## [1.8.0] - 2022-11-17

### Added
- node template for rtx_radar

### Changed
- changed node template name for rtx_lidar

## [1.7.0] - 2022-11-14
### Changed
- Deprecated viewport input for camera helper
- Added renderProductPath input for camera helper

## [1.6.3] - 2022-10-25
### Fixed
- RTX Lidar Transform Tree Publisher

## [1.6.2] - 2022-09-13
### Fixed
- Test failures
- Warning when setting semantic class input

## [1.6.1] - 2022-09-07
### Fixed
- Fixes for kit 103.5

## [1.6.0] - 2022-08-30

### Changed
- Remove direct legacy viewport calls
## [1.5.0] - 2022-08-15

### Added
- ROS2Context node has a useDomainIDEnvVar flag that can be set to true so that the ROS_DOMAIN_ID variable is used

## [1.4.1] - 2022-08-09

### Removed
- Unused carb settings
### Fixed
- Activating image publishers should not cause a crash anymore
- File watcher patterns for extension

## [1.4.0] - 2022-07-22

### Added
- omni.syntheticdata template to publish RTX lidar point cloud

## [1.3.0] - 2022-07-21

### Changed
- Removed articulation control from OgnROS2SubscribeJointState
- Added JointState message outputs to OgnROS2SubscribeJointState allowing users to connect outputs to articulation controller core_node

## [1.2.3] - 2022-07-15

### Changed
- OgnROS2PublishSemanticLabels to accept string data type
- Timestamp data now appended to JSON msg output

## [1.2.2] - 2022-07-13

### Changed
- Improved image publisher perf

## [1.2.1] - 2022-07-06

### Fixed
- Quaternion input descriptions

## [1.2.0] - 2022-06-21

### Added
- ROS2 IMU publisher node

## [1.1.2] - 2022-05-31

### Fixed
- Removed Teleport sample from "Isaac Examples" Menu

## [1.1.1] - 2022-05-26

### Fixed
- Crash when switching extension on/off and simulating

## [1.1.0] - 2022-05-20

### Added
- ROS2 examples in "Isaac Examples" Menu

## [1.0.0] - 2022-05-18

### Changed
- Fully switched to OG ROS2 bridge nodes

## [0.8.3] - 2022-05-18

### Changed
- Added 32SC1 image type option to OgnROS2PublishImage

### Fixed
- Corrected stereoOffset input in OgnROS2CameraHelper

## [0.8.2] - 2022-05-03

### Changed
- Output data types to vectord and quatd in ROS2 Odometry, Raw TF publisher nodes and Twist subscriber node
- Added dropdown menu and validation for encoding input in ROS2 image publisher node

## [0.8.1] - 2022-05-02

### Changed
- Output data type from float to pointf in ROS2 point cloud publisher node

## [0.8.0] - 2022-04-29

### Added
- ROS2 Image publisher node

## [0.7.0] - 2022-04-28

### Added
- ROS2 Camera Info publisher node

## [0.6.1] - 2022-04-27

### Changed
- ROS2 point cloud publisher node to read generic point cloud buffer
- ROS2 laser scan publisher node to read generic lidar data buffers

## [0.6.0] - 2022-04-22

### Added
- OG ROS2 Odometry publisher node
- OG ROS2 Twist subscriber node
- OG ROS2 Transform Tree publisher node
- OG ROS2 Raw Transform Tree publisher node
- OG ROS2 Joint State Publisher node
- OG ROS2 Joint State Subscriber node

### Fixed
- OG ROS2 pub/sub clock nodes now able to namespace topic names

## [0.5.0] - 2022-04-20

### Added
- OG ROS2 LaserScan publisher node
- OG ROS2 PointCloud2 publisher node for lidar
- Utility method, addTopicPrefix for namespacing ROS2 topics

## [0.4.0] - 2022-04-13

### Added
- ROS2 Context
- ROS2 pub/sub clock

## [0.3.0] - 2022-03-18

### Added
- Added ROS2 topic name validation

## [0.2.7] - 2022-03-16

### Changed
- Replaced find_nucleus_server() with get_assets_root_path()

## [0.2.6] - 2022-03-11

### Changed
- Removed Cyclone DDS to allow defualt FastRTPS DDS to run instead

## [0.2.5] - 2022-03-11

### Fixed
- Issue with lidar init on first frame

## [0.2.4] - 2022-02-25

### Fixed
- Issue with camera init on first frame

### Changed
- ROS2 Bridge to initialize rclcpp in onResume and shutdown rclcpp in onStop

## [0.2.3] - 2022-02-17

### Fixed
- Crash when changing a camera parameter when stopped

### Changed
- Switch depth to new eDistanceToPlane sensor

## [0.2.2] - 2022-02-11

### Fixed
- laserScan publisher in RosLidar to be able to synchronize with Lidar sensor after any live user changes to USD properties
- pointCloud publisher using seperate caching variables from laserScan to prevent accidental overwriting

## [0.2.1] - 2022-02-08

### Fixed
- TF Tree publisher parent frame to include filter for articulation objects, separately from rigid body.

## [0.2.0] - 2022-02-07

### Added
- odometryEnabled setting to toggle both Odometry and TF publishers in differential base components

## [0.1.1] - 2021-12-08

### Fixed
- odometry frame matches robot's starting frame, not the world frame.
- horizontal and vertical aperture use camera prim values instead of computing vertical aperture
- lidar components publish point cloud data as PCL2 messages instead of PCL
- lidar PCL2 messages only contain points that hit
- lidar publisher publishes a full scan for point cloud data

### Added
- usePhysicsStepSimTime setting and use_physics_step_sim_time to use physics step events to update simulation time

## [0.1.0] - 2021-04-23

### Added
- Initial version of ROS2 Bridge
