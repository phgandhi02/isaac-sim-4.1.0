# Changelog

## [1.0.1] - 2024-03-07
### Changed
- Removed the usage of the deprecated dynamic_control extension 

## [1.0.0] - 2024-02-09
### Changed
- Moved menu items to omni.isaac.menu

## [0.7.2] - 2024-02-02
### Changed
- Updated path to the nucleus extension

## [0.7.1] - 2024-01-18
### Changed
- Changed get_assets_root_path to get_assets_root_path_async for the unit tests

## [0.7.0] - 2024-01-11
### Changed
- Moved forklift asset under new Robots > Forklift section
- Renamed forklift asset to Forklift B.

### Added
- New Forklift C asset option under Robots > Forklift

## [0.6.0] - 2023-11-29
### Changed
- Moved End effectors out of Robots menu again
- Removed Factory Franka

## [0.5.4] - 2023-11-20
### Changed
- Moved all End effectors under Robots section

### Fixed
- April tag menu

### Fixed
- Loading Robotiq hand into a path that doesn't start with a number

## [0.5.3] - 2023-10-06
### Added
- Carter V2.4 asset path, removed Carter V2 and V2.3
- Renamed Carter V2.4 Nova Carter

## [0.5.2] - 2023-10-04
### Fixed
- Broken asset paths

## [0.5.1] - 2023-09-13
### Fixed
- Updated path of Kuka KR210 robot to match update to Nucleus Server
- Updated name of Kuka KR210 robot in Create menu.

## [0.5.0] - 2023-09-06
### Changed
- updated Create > Isaac > Robots menu

## [0.4.0] - 2023-08-17
### Added
- getCameraPrimFromRenderProduct function

## [0.3.0] - 2023-08-01
### Added
- Added new robots to Create -> Isaac -> Robots menu

## [0.2.5] - 2023-06-12
### Changed
- Update to kit 105.1, omni.usd.utils renamed omni.usd

## [0.2.4] - 2023-01-06
### Fixed
- onclick_fn warning when creating UI

## [0.2.3] - 2022-11-03
### Fixed
- Default cameras for office, hospital, simple room start at reasonable defaults now

## [0.2.2] - 2022-10-27
### Fixed
- Revamped Create -> Isaac menu

## [0.2.1] - 2022-09-07
### Fixed
- Fixes for kit 103.5

## [0.2.0] - 2022-09-01

### Changed
- removed legacy viewport calls
## [0.1.11] - 2022-03-16

### Changed
- Replaced find_nucleus_server() with get_assets_root_path()

## [0.1.10] - 2021-12-01

### Removed
- isaac.nucleus.default setting moved to omni.isaac.core

## [0.1.9] - 2021-10-21

### Removed
- lookat_to_quat to omni.isaac.core.utils.rotations
- get_intrinsics_matrix, backproject_depth, project_depth_to_worldspace to omni.isaac.core.utils.viewports
- set_up_z_axis to omni.isaac.core.utils.stage.set_stage_up_axis
- omni.isaac.demo specific math utils
- test_utils.py
- create_background
- quat_to_euler_angles, use omni.isaac.core.utils.rotations.quat_to_euler_angles

## [0.1.8] - 2021-08-13

### Added
- find_nucleus_server_async with timeout

## [0.1.7] - 2021-07-31

### Removed
- Surface Gripper is its own extension

## [0.1.6] - 2021-07-30

### Removed
- Physics Inspector is its own extension
- Physics Utilities is its own extension
- Merge Mesh is its own extension
- Debug Draw is its own extension

## [0.1.5] - 2021-07-12

### Added
- New UI for Surface Gripper

## [0.1.4] - 2021-05-24

### Added
- add physics utils extension
- add create menu

## [0.1.3] - 2021-02-17

### Added
- update to python 3.7
- update to omni.kit.uiapp

## [0.1.2] - 2021-01-04

### Added
- Fix nucleus utils for new content window

## [0.1.1] - 2020-12-11

### Added
- Add unit tests to extension

## [0.1.0] - 2020-12-03

### Added
- Initial version of Isaac Sim Utils Extension
