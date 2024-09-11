# Changelog
## [1.5.0] - 2024-05-23
### Fixed
- Fixed bug where single robot checkbox in UI workflow had no effect when True.

## [1.4.0] - 2024-05-17
### Fixed
- Fixed bug with AUTO_CREATE_FRAME miscomputing local poses for rigid bodies.
- Removed option to AUTO_CREATE_FRAME for Articulations as it causes the attachment to fail.
- Fixed bug with computing fixed joint transforms to non-root robot links.
- Fixed bug where collision masking fails when base Articulation has an Articulation root nested under the top-level path.
- Fixed bug where JointStateAPIs were being overwritten when nesting prims on STOP and then assembling on the first frame of PLAY.  The fix is to simply set the values back to zero at the right moment.
### Added
- Added test cases for different placements of Articulation Roots.  This creates a matrix of possible configurations to support between two robots being attached.
### Changed
- Robot Assembler moves the Articulation Root of the attach robot to the top-level prim in the hierarchy while it is attached, and reverses this on detach.
- Stabilized result of set_fixed_joint_transform() by teleporting the robot to where physics will think it should go.

## [1.3.0] - 2024-05-14
### Fixed
- Fixed bug where attach frames for a robot can include its fixed joint (which causes a failed assembly) when the Articulation Root is on the fixed joint.
- Fixed bug where Articulation selection function can be called with an invalid prim path and cause a harmless (but visible) error.

## [1.2.0] - 2024-04-29
### Fixed
- Fixed bug where robot assembler could not list frames for assets that have Articulation roots on a link in the Articulation.

## [1.1.6] - 2024-02-02
### Changed
- Updated path to the nucleus extension

## [1.1.5] - 2024-01-18
### Changed
- Changed get_assets_root_path to get_assets_root_path_async for the unit tests

## [1.1.4] - 2023-12-11
### Fixed
- Fixed breaking test cases that loaded local USD assets with references to robots on Nucleus

## [1.1.3] - 2023-11-13
### Fixed
- Updated documentation link

## [1.1.2] - 2023-08-11
### Added
- Added index.rst for Sphinx autodoc

## [1.1.1] - 2023-08-08
### Fixed
- Switch from ui.Window to ScrollingWindow wrapper for extension because scrolling was broken

## [1.1.0] - 2023-07-12
### Added
- Added Test Cases for Robot Assembler python API

## [1.0.0] - 2023-07-06

### Added
- Initial version of Robot Assembler Extension
