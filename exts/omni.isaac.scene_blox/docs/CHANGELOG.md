# Changelog

## [0.1.2] - 2024-02-02

### Changed
- Updated path to the nucleus extension

## [0.1.1] - 2023-03-20

### Changed

- Updates all modules to use newly-added global RNG, enabling replicability by setting seed.
### Added

- omni.isaac.scene_blox.grid_utils.config - Maintains global RNG.
## [0.1.0] - 2023-01-17

### Changed

- Moved to omni.isaac.scene_blox

## [0.0.5] - 2022-08-01

### Fixed

- bug where app would hang on exit
- error message on stage close

## [0.0.4] - 2022-08-01

### Changed

- Assets path may now be out of the /Isaac folder

### Added

- If a variant name is "*", a variant value is selected at random for all variants

## [0.0.3] - 2022-06-02

### Changed

- Removed livesync option which was unstable and not useful

## [0.0.2] - 2022-05-20

### Fixed

- Issue on prim path formatting on Windows (was using OS-style separator instead of /)

### Changed

- Adding a try - except on main file to warn users to install module with pip before trying to use it
