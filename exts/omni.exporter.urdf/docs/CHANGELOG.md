# Changelog

## [0.3.0] - 2024-04-26

### Fixed

- Fix cylinder radii check
- Fix checking for duplicate prim names
- Fix issue with joint limits set to inf for revolute joints, set to continuous joint instead
- Fix cylinder scaling issue

### Added

- Replace absolute path and uri path options with mesh path prefix option
- Parse velocity and effort limits from USD and include in URDF
- Add UsdGeom.Cylinder to error message as one of the valid geom prim types
- Add hack to not include camera mesh files when exporting from isaac sim extension
- Add mesh file path char length check
- Add NodeType.SENSOR and functionality to include sensor frames in the URDF


## [0.2.0] - 2023-12-01

### Fixed

- Scaling bug when geometry prim is a child of Xform prim with scaling
- Cylinder scaling issue
- Joint frame alignment when child link transform is not identity
- Exporting revolute joints with inf limits, export as continuous joints now
- Too many root links
- Velocity and effort limits always being set to zero
- Large camera meshes added erroneously to URDF
- Terminal output labeled as error even when export is successful

### Added

- Exporting sensor prim frames to URDF
- Ability to set mesh path prefix
- Optionally setting output to directory (URDF file name automatically set to match USD file name)


## [0.1.1] - 2023-09-19

### Fixed

- Add missing dependencies and remove unused code


## [0.1.0] - 2023-07-27

### Added

- Initial version of Isaac Sim URDF exporter
