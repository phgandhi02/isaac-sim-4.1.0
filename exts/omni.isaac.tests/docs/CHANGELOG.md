# Changelog

## [1.18.0] - 2024-07-10
### Removed
- Deprecated omni.isaac.dofbot and removed its usage.

## [1.17.0] - 2024-05-23
### Added
- Tests for the Leatherback robot.
### Fixed
- O3dyn Tests

## [1.16.6] - 2024-05-11
### Changed
- Renamed Transporter to iw.hub

## [1.16.5] - 2024-04-10
### Added
- Tests for the Create 3 robot.

## [1.16.4] - 2024-03-20
### Changed
- Updated nvblox tests to match the sample

## [1.16.3] - 2024-03-14
### Changed
- Update tolerances of O3dyn tests

## [1.16.2] - 2024-03-07
### Changed
- Removed the usage of the deprecated dynamic_control extension 

## [1.16.1] - 2024-02-02
### Changed
- Updated path to the nucleus extension

## [1.16.0] - 2024-02-01
### Added
- Added NvBlox unit test

## [1.15.3] - 2024-01-25
### Added
- Added O3Dyn robot rotate unit tests

### Changed
- Changed get_assets_root_path to get_assets_root_path_async for the unit tests

## [1.15.2] - 2024-01-03
### Changed
- Updated unit tests with newer OgnIsaacArticulationController node

## [1.15.1] 2023-12-06
### Fixed
- Modified jetbot unit tests to not go above its max speed

## [1.15.0] 2023-11-30
### Added
- Added unit tests for O3Dyn Robot.
- Update unit test to use get_assets_root_path_async

## [1.14.4] 2023-11-29
### Added
- Added End Effectors menu items back into menu tests

## [1.14.3] 2023-11-27
### Added
- Added unit tests for checking the sensor and environment menu items

## [1.14.2] 2023-11-27
### Fixed
- updated tests to renamed nova carter asset 

## [1.14.1] 2023-11-21
### Added
- Added writers to the 'test_randomizer_snippets.py' isaac replicator randomizers 

## [1.14.0] 2023-11-20
### Added
- Added automatic loading test for assets under "Robots"
- Added test for apriltag menu

## [1.13.1] 2023-10-11
### Changed
- Modified target angular velocity for the spin test as the old values are above the speed limit

## [1.13.0] 2023-10-05
### Changed
- Changed carter and drive goal carter unit tests to use the NOVA carter (V2.4) asset

## [1.12.0] 2023-08-28
### Added
- test_forklift_articulations.py for testing drive & lift of Forklift C model

## [1.11.1] 2023-08-29
### Fixed
- test_randomizer_snippets.py to use await instead of ensure_future

## [1.11.0] 2023-08-24
### Added
- test_randomizer_snippets.py containing replicator alternative randomizer examples from the docs 
- Missing utility snippet from the docs

## [1.10.4] - 2023-08-21
### Fixed
- Fixed drive_goal_carter_v2 tests to use the correct timecode setting
### Changed
- Changed test accel, test spin, and test circle thresholds for the carter v1, v2 and jetbot

## [1.10.3] - 2023-08-09
### Changed
- Changed unit tests as the prim types of omnigraph nodes have been changed from bundle to target

## [1.10.2] - 2023-08-09
### Fixed
- Added timecode to stage for test_stage_up_axis and test_stage_units
- Fixed articulation_drives_opposite.usd file errors

## [1.10.1] - 2023-06-12
### Changed
- Update to kit 105.1, omni.usd.utils renamed omni.usd

## [1.10.0] 2023-02-24
### Added
- Joint effort test, with pure usd joints and articulation joints

## [1.9.0] 2023-02-05
### Changed
- Refactor and fix broken tests

## [1.8.0] 2023-01-20
### Changed
- Combined and cleaned up tests

## [1.7.0] 2023-01-11
### Removed
- Moved multi-cam utility snippet to standalone_examples/omni.isaac.snippets/

## [1.6.0] 2023-01-03
### Added
- omni.anim.people extension startup test

## [1.5.0] 2022-12-10
### Changed
- use set_target_prims from core nodes

## [1.4.0] 2022-11-18
### Added
- cuopt example extension startup test

## [1.3.1] 2022-10-27
### Fixed
- Fixed broken mobile robot tests

## [1.3.0] 2022-10-19
### Added
- Added Test Cases that cause segfaults in the current build of Isaac Sim

## [1.2.0] - 2022-10-05 
### Added
- Added Test Case to check if Sim will freeze when opening a USD stage 100 times

## [1.1.1] - 2022-09-07
### Fixed
- Fixes for kit 103.5

## [1.1.0] - 2022-07-26

### Added
- Differential base robot testcases

## [1.0.0] - 2022-07-12

### Added
- Initial Version
