# Changelog

## [0.13.2] - 2024-05-01
### Fixed
- update for set_phase api change

## [0.13.1] - 2024-03-19
### Fixed
- include for benchmark services


## [0.13.0] - 2024-03-18

### Added
- isaac.startup.create_new_stage carb setting to enable/disable stage creation on app startup, enabled by default


## [0.12.3] - 2024-03-07

### Fixed
- Crash after disabling omni.activity extensions to improve performance

## [0.12.2] - 2024-02-29

### Added
- Benchmark metadata
### Changed
- Updated benchmark set_phase() call to correctly record startup time after removing deprecated API
### Removed
- Deprecated benchmark stop_runtime() call

## [0.12.1] - 2024-02-08

### Removed
- Deprecated replicator composer menu item


## [0.12.0] - 2024-01-30

### Changed
- Measures startup time if omni.isaac.benchmark.services is loaded.

## [0.11.2] - 2023-09-21

### Removed
- Remove hotkeys for Help menu

## [0.11.1] - 2023-09-06

### Changed
- Deprecation label for Replicator/Composer menu item

## [0.11.0] - 2023-09-06

### Changed
- Added Replicator YAML menu item

## [0.10.0] - 2023-08-08

### Changed
- Update Help menu

## [0.9.0] - 2023-07-05

### Changed
- Removed settings that were already set in .kit file

## [0.8.2] - 2023-06-07

### Fixed
- Issue where fullscreen and hide UI didn't work

## [0.8.1] - 2023-02-14

### Changed
- Update forums link

## [0.8.0] - 2023-02-08

### Changed
- set omnigraph default settings on app start

## [0.7.0] - 2023-01-23

### Changed
- Removed unused code
- Enabled tests
- Added setting to enable ros bridge on startup

## [0.6.2] - 2022-12-05

### Changed
- Push Help menu to the end

## [0.6.1] - 2022-12-01

### Fixed
- Disable App Selector auto start when running from Help menu (OM-74496)

## [0.6.0] - 2022-12-01

### Changed
- Added replicator menu layout

## [0.5.0] - 2022-11-22

### Changed
- Updated menu icons

## [0.4.1] - 2022-11-08

### Fixed
- Duplicate help menus using same F1 key, switched isaac sim docs to F3

## [0.4.0] - 2022-10-06

### Changed
- Updated menu structure to match create

## [0.3.1] - 2022-09-01

### Changed
- Removed unused call to viewport legacy

## [0.3.0] - 2022-08-15

### Added
- Force PhysX reset on stop to be true

## [0.2.4] - 2022-07-25

### Added
- Increase hang detection timeout (OM-55578)

## [0.2.3] - 2022-05-31

### Changed
- Use carb.tokens to get kit exe path (OM-48462)

## [0.2.2] - 2022-05-17

### Fixed
- Fixed hot reloading

## [0.2.1] - 2022-05-12

### Added
- Use omni.isaac.version.get_version()

## [0.2.0] - 2022-04-29

### Added
- Added Isaac Sim App Selector to the Help Menu.

## [0.1.4] - 2022-03-16

### Added
- Added Isaac Sim full version tag in log file.

## [0.1.3] - 2022-01-24

### Removed
- Moved Nucleus Check to its own extension (OM-43459)

## [0.1.2] - 2021-12-15

### Changed
- Added feature to detect and update downloaded Isaac Sim assets on Nucleus (OM-41819)

## [0.1.1] - 2021-12-01

### Fixed
- Fixed nucleusCheck running during post-install and warm-up (OM-41545)

### Changed
- Update settings to use camel case

## [0.1.0] - 2021-10-22

### Added
- Added first version of setup.
