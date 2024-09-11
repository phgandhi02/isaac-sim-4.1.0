# Changelog
## [2.5.0] - 2024-07-08
### Fixed
- Fixed behavior of Robot Description Editor for assets with nested link paths.
- Fixed bug with toggling robot visibility causing an error on STOP.

## [2.4.0] - 2024-05-01
### Fixed
- Fixed Articulation DropDown population and corresponding extension behavior.

## [2.3.2] - 2024-04-29
### Changed
- Update error handling behavior to attempt to parse XRDF files with unsupported format versions after giving a warning.

## [2.3.1] - 2024-04-15
### Changed
- Update text to say cuMotion instead of Curobo and update Information Panel docs.

## [2.3.0] - 2024-03-27
### Added
- Add support for importing and exporting Curobo XRDF files
- Add maximum acceleration and jerk properties to Command Panel

## [2.2.1] - 2024-03-15
### Fixed
- Fixed logic around selecting Articulation on STOP/PLAY given new behavior in Core get_prim_object_type() function.

## [2.2.0] - 2024-02-06
### Changed
- Improved sphere creation time by reusing `VisualMaterial`s and rewriting sphere prim path generation.

## [2.1.3] - 2023-11-13
### Fixed
- Updated documentation link

## [2.1.2] - 2023-08-08
### Fixed
- Switch from ui.Window to ScrollingWindow wrapper for extension because scrolling was broken

## [2.1.1] - 2023-01-06
### Fixed
- onclick_fn warning when creating UI

## [2.1.0] - 2022-11-18

### Fixed

- Catch bug where error was thrown on saving with no Active Joint selected Now a better error is thrown that tells you what you need to fix.

### Changed

- Command Panel now completely expands upon selecting robot.

## [2.0.0] - 2022-11-17

### Changed

- Rebranded extension as "Robot Description Editor"
- Can now load from / export to entire Lula Robot Description files. 

## [1.1.0] - 2022-11-15

### Added

- Added ability to generate collision spheres on a per-link basis

## [1.0.0] - 2022-09-12

### Changed

- Modified structure of extension to organize all features around a pre-specified robot link to eliminate the need to type in correct prim paths and simplify the user experience.

### Added

- Ability to toggle visiblility on selected robot link or the robot as a whole
- Selected link has its own color for nested spheres

## [0.2.0] - 2022-09-02

### Changed

- Enhanced collision sphere interpolation feature to generate spheres with radii following a geometric sequence, positioned so as to produce a smooth conical frustum in the limit of infinite spheres.

## [0.1.1] - 2022-09-02

### Removed

- Removed unused dependencies
- Removed from load function: automatic bookmark to file path outside of Isaac Sim directories.

## [0.1.0] - 2022-08-15

### Added

- Initial version of Collision Sphere Editor Extension
