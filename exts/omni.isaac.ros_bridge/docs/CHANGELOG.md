# Changelog

## [1.20.1] - 2024-07-12
### Removed
- IsaacReadSimulationTime removed from RTXLidarROS2PublishFlatScan, timestamp now comes from
  OgnIsaacComputeRTXLidarFlatScan.

## [1.20.0] - 2024-07-05
### Changed
- use register_*_with_telemetry functions to simplify registration code
- use OgnIsaacPassthroughImagePtr node for depth
- switch to using ptr for depth data to avoid extra cpu copy

## [1.19.4] - 2024-06-10
### Fixed
- Unit tests

## [1.19.3] - 2024-06-03
### Fixed
- Crash on twist subscriber node release if it wasn't run at least one frame

## [1.19.2] - 2024-05-21
### Added
- Resetting output of the Twist Subscriber to zeros upon stopping simulation.

## [1.19.1] - 2024-05-19
### Changed
- Extension is deprecated

## [1.19.0] - 2024-05-10
### Changed
- Updated all ros_comm packages to v1.16.0

## [1.18.2] - 2024-05-07
### Fixed
- OgnROS1PublishLaserScan angle inputs now provided in degrees rather than mixed degrees/radians

## [1.18.1] - 2024-05-04
### Fixed
- Roscore not resetting correctly
- Unit tests

## [1.18.0] - 2024-04-24
### Changed
- Stereo offset input in ROS Camera Info and Helper nodes to accept camera baseline in meters used to compute Tx, Ty.

## [1.17.3] - 2024-04-19
### Fixed
- Registration for system time writers with telemetry tracking

## [1.17.2] - 2024-04-18
### Added
- Telemetry for writers and annotators

## [1.17.1] - 2024-04-12
### Added
- Unit test for jointstate publisher and subscriber

## [1.17.0] - 2024-04-11
### Added
- Add frameSkipCount option to RTX Lidar and Camera Helper Nodes to modify the rate of lidar and point cloud publishing

## [1.16.0] - 2024-04-04
### Added
- Option to use system timestamp for RTX Lidar and Camera Helper Nodes

### Changed
- Updated Camera and RTX Lidar unit tests for publishing using system timestamp

## [1.15.9] - 2024-03-12
### Fixed
- Multi GPU support

## [1.15.8] - 2024-03-05
### Fixed
- RGB being published twice per frame

## [1.15.7] - 2024-03-04
### Changed
- Updated omnigraph nodes to use per instance state instead of internal state

## [1.15.6] - 2024-02-28
### Added
- Added Show Debug View flag to RTX Lidar Helper

## [1.15.5] - 2024-02-24
### Changed
- Update source code to run a build clean of warnings

## [1.15.4] - 2024-02-02
### Changed
- Updated path to the nucleus extension

## [1.15.3] - 2024-01-19
### Fixed
- Empty semantics unit test

### Changed
- Changed get_assets_root_path to get_assets_root_path_async for the unit tests

## [1.15.2] - 2024-01-03
### Changed
- Updated samples and unit tests with newer OgnIsaacArticulationController node

## [1.15.1] - 2023-12-20
### Added
- Lidar and pointcloud tests with replicator enabled

## [1.15.0] - 2023-11-28
### Added
- ROS1 Ackermann Subscriber and Publisher nodes
- Option to publish a full scan from RTX lidar

## [1.14.12] - 2023-10-20
### Fixed
- Logging levels for roscore helper

## [1.14.11] - 2023-10-13
### Fixed
- CameraInfo publisher now includes identity Rectification matrix

## [1.14.10] - 2023-10-09
### Fixed
- Moveit Tutorial had incorrect OG input name

## [1.14.9] - 2023-10-04

### Added
- Tests for JointState Publisher
### Fixed
- Carter 1 Unit tests
- JointState Publisher sign correction when a joint's parents are in reversed order


## [1.14.8] - 2023-09-29
### Added
- Removed context field from OgnROS1RtxLidarHelper OG node

## [1.14.7] - 2023-09-26
### Added
- Added enable flag for the lidar and camera helper
- Improved description for the frame ID for the camera and lidar helper

## [1.14.6] - 2023-09-25
### Fixed
- Transform tree node can accept multiple target prims as input

## [1.14.5] - 2023-08-26
### Fixed
- CUDA error in PCL publisher
- Semantic publisher not correctly publishing bbox, semantics, instances

## [1.14.4] - 2023-08-25
### Changed
- Added standard out fail pattern for the expected no prim found edge cases for the ogn test
- Changed test_camera test to expect identical fx and fy as vertical aperture is computed from horizontal aperture.

## [1.14.3] - 2023-08-25
### Fixed
- Add default values for World transform attributes when fetching fabric in computeWorldXformNoCache function used in processAllFrames
## [1.14.2] - 2023-08-18
### Fixed
- Pixel are square in 105.1 Vertical aperture is not used, and based off of horizontal

## [1.14.1] - 2023-08-15
### Changed
- Changed targetPrim datatype from bundle to target in omnigraph

## [1.14.0] - 2023-08-11
### Changed
- Image and pcl nodes support gpu/cpu ptr data

## [1.13.0] - 2023-08-03
### Changed
- Image and PCL publishers to use ptrs instead of arrays when possible to reduce memory copies

## [1.12.0] - 2023-07-07
### Added
- Sync and async versions of checking for rosmaster
### Changed
- renamed wait_for_rosmaster to wait_for_rosmaster_async

## [1.11.4] - 2023-06-12
### Changed
- Update to kit 105.1
- add lib locations to PYTHONPATH
- rename synthetic data writer templates to match rendervar names

## [1.11.3] - 2023-03-30
### Fixed
- Bug in Joint Subscriber when using effort commands

## [1.11.2] - 2023-03-13
### Fixed
- Incorrect 2D, 3D bbox data
- Bug with RTX Lidar not publishing flatscan correctly
- invalid json string if semantic message was empty

## [1.11.1] - 2023-03-09
### Fixed
- Negative bbox sizes

## [1.11.0] - 2023-02-28
### Fixed
- Issue where image based messages always used increasing simulation time
### Added
- resetSimulationTimeOnStop to Camera Helper node.

## [1.10.0] - 2023-01-31
### Added
- Fabric support

## [1.9.2] - 2023-01-21
### Fixed
- TF_Tree fix when multiple objects shared the same prim name using the isaac:nameOverride attribute
- remove un-needed cpp ogn files from extension
## [1.9.1] - 2023-01-06
### Fixed
- onclick_fn warning when creating UI

## [1.9.0] - 2022-12-10

### Changed
- Switch publishing nodes to use replicator writer backend


## [1.8.1] - 2022-12-02

### Fixed
- Revert erroneous removal of omni.graph.scriptnode extensions and modules.

## [1.8.0] - 2022-11-21

### Added
- rtx_lidar_helper Node

## [1.7.0] - 2022-11-17

### Added
- node template for rtx_radar

### Changed
- changed node template name for rtx_lidar

## [1.6.0] - 2022-11-14
### Changed
- Deprecated viewport input for camera helper
- Added renderProductPath input for camera helper

## [1.5.3] - 2022-10-25
### Fixed
- RTX Lidar Transform Tree Publsiher

## [1.5.2] - 2022-09-13
### Fixed
- Test failures
- Warning when setting semantic class input

## [1.5.1] - 2022-09-07
### Fixed
- Fixes for kit 103.5

## [1.5.0] - 2022-08-30

### Changed
- Remove direct legacy viewport calls

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
- Removed articulation control from OgnROS1SubscribeJointState
- Added JointState message outputs to OgnROS1SubscribeJointState allowing users to connect outputs to articulation controller core_node

## [1.2.3] - 2022-07-15

### Changed
- OgnROS1PublishSemanticLabels to accept string data type
- Timestamp data now appended to JSON msg output

## [1.2.2] - 2022-07-13

### Changed
- Improved image publisher perf

## [1.2.1] - 2022-07-06

### Fixed
- Quaternion input descriptions

## [1.2.0] - 2022-06-21

### Added
- ROS IMU publisher node

## [1.1.2] - 2022-06-02

### Fixed
- shutdown function call in roscore.py

## [1.1.1] - 2022-05-27

### Fixed
- Updated OgnROS1ServiceTeleport to check for invalid prim to avoid crash

## [1.1.0] - 2022-05-20

### Added
- ROS examples in "Isaac Examples" Menu

## [1.0.0] - 2022-05-18

### Changed
- Fully switched to OG ROS bridge nodes

## [0.10.1] - 2022-05-18

### Fixed
- Corrected stereoOffset input in OgnROS1CameraHelper

## [0.10.0] - 2022-05-15

### Changed
- Add bbox2d,3d,instance, segmentation, camera_info to camera helper
- renamed sensor to type for camera helper
- added 32int output to image publisher

## [0.9.0] - 2022-05-12

### Changed
- Added camera helper node
- Added 2d bbox, 3d bbox, semantics nodes

## [0.8.3] - 2022-05-03

### Changed
- Output data types to vectord and quatd in ROS Odometry, Raw TF publisher nodes and Twist subscriber node
- Added dropdown menu and validation for encoding input in ROS image publisher node

## [0.8.2] - 2022-05-02

### Changed
- Output data type from float to pointf in ROS point cloud publisher node

## [0.8.1] - 2022-04-29

### Changed
- Added frameId input to ROS Image publisher node
- Included row length calculation for image buffer in ROS Image publisher node

## [0.8.0] - 2022-04-28

### Added
- ROS Camera Info publisher node

## [0.7.3] - 2022-04-27

### Changed
- ROS point cloud publisher node to read generic point cloud buffer

## [0.7.2] - 2022-04-25

### Changed
- ROS laserscan publisher node to accept data from Isaac Read Lidar Beams core node

### Fixed
- ROS image publisher node name

## [0.7.1] - 2022-04-22

### Fixed
- ROS Raw TF publisher node description and warning message

## [0.7.0] - 2022-04-19

### Added
- ROS image publisher node

## [0.6.1] - 2022-04-14

### Fixed
- Normalized robotFront vector when calculating odometry
- Cross-product used to find robot's y-component of linear velocity in OG ROS Odometry publisher node

## [0.6.0] - 2022-04-12

### Added
- OG ROS teleport service
- OG Isaac Read Odometry node
- OG ROS Raw Transform Tree publisher node

### Changed
- Removed bundle input and added odometry related vector inputs to OG ROS Odometry publisher node
- Updated UI Names for ROS OG nodes

## [0.5.0] - 2022-04-08

### Added
- OG ROS Odometry publisher with odom->chassis frame TF publish node

## [0.4.0] - 2022-04-07

### Added
- OG ROS Twist subscriber node

### Changed
- Updated nodeNamespace descriptions for all ROS OG nodes

## [0.3.0] - 2022-04-05

### Added
- OG ROS Transform Tree publisher node
- Utility method, addFramePrefix for namespacing frameIds
- OG ROS Joint State Publisher node
- OG ROS Joint State Subscriber node

### Changed
- ROS OG nodes to use nodeNamespace as input

## [0.2.9] - 2022-03-31

### Added
- OG ROS PointCloud2 publisher node for lidar

## [0.2.8] - 2022-03-23

### Changed
- Default topic name for OG clock nodes

### Added
- OG ROS laserscan publisher node

## [0.2.7] - 2022-03-19

### Changed
- Add OG nodes for clock topic

## [0.2.6] - 2022-03-16

### Changed
- Replaced find_nucleus_server() with get_assets_root_path()

## [0.2.5] - 2022-03-11

### Fixed
- Issue with lidar init on first frame

## [0.2.4] - 2022-03-02

### Fixed
- Issue with camera init on first frame

## [0.2.3] - 2022-02-17

### Fixed
- Crash when changing a camera parameter when stopped

### Changed
- Switch depth to new eDistanceToPlane sensor
- Enable OmniGraph

## [0.2.2] - 2022-02-11

### Fixed
- laserScan publisher to be able to synchronize with Lidar sensor after any live user changes to USD properties
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
- Initial version of ROS Bridge
