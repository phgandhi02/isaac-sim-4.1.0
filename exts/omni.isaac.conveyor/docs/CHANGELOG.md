# Changelog
## [0.4.0] - 2024-06-13

### Changed
- Updated Physics API to use SurfaceVelocity instead of rigid Body deprecated modified Velocities.
- Added Velocity Variable in omnigraph to control the conveyor speed

### Removed
- UI elements that didn't move over
- Forcing Kinematics Body on Conveyor Creation

# Changelog
## [0.3.14] - 2024-05-06
### Removed
- Unused config file

# Changelog
## [0.3.13] - 2023-11-01
### Fixed
- Error when Node starts for the first time if it doesn't have a Texture translate attribute.

## [0.3.12] - 2023-08-28
### Changed
- added standard out fail pattern for the expected no prim found edge case for the ogn test

## [0.3.11] - 2023-08-10
### Changed
- Changed conveyor prim node input type from bundle to target
## [0.3.10] - 2023-06-13
### Changed
- Update to kit 105.1, build system update, omni.usd.utils -> omni.usd, usdPhysics -> pxr/usd/usdPhysics

## [0.3.9] - 2023-02-15
### Fixed
- UI regression where selected track card was not updated correctly

## [0.3.8] - 2023-02-14
### Changed
- Update asset path

## [0.3.7] - 2023-02-09
### Fixed
- Tests Use Fabric to get physx updates
- Error message when generating UI with image URL and destroing element before image finished loading
- Create Conveyor own hidden scope instead of using /Render

## [0.3.6] - 2023-01-25
### Fixed
- remove un-needed cpp ogn files from extension

## [0.3.5] - 2023-01-06
### Fixed
- onclick_fn warning when creating UI

## [0.3.4] - 2022-12-02
### Fixed
- Adaptive scale for assets not in the same meters per unit than open stage
## [0.3.3] - 2022-12-01
### Fixed
- CreateConveyorBelt command documentation update
## [0.3.2] - 2022-12-01
### Fixed
- CreateConveyorBelt command .do() only returns the created prim and not a tuple

## [0.3.1] - 2022-11-29
### Fixed
- OG Node wouldn't update if the node direction changed
- Assets Reorganization

### Added
- Sorting Assets

## [0.3.0] - 2022-11-22
### Added
- Digital Twin Library Conveyor Authoring tool
- Support for curved conveyors
### Changed
- Default path for creating conveyor command is now at prim parent instead of rigid body prim.

## [0.2.1] - 2022-09-07
### Fixed
- Fixes for kit 103.5

## [0.2.0] - 2022-07-22

### Changed
- Convert node to cpp backend
- Conveyor node renamed to IsaacConveyor

### Added
- Simplified creating multiple conveyors, multiple prims can be selected on creation using menu

## [0.1.1] - 2022-05-10

### Changed
- Change Tests to use meters as distance unit

## [0.1.0] - 2022-03-30

- First Version
