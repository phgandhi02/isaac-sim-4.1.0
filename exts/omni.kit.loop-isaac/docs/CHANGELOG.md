# Changelog

## [1.2.0] - 2024-01-08
### Changed
- Moved header files to extension

## [1.1.0] - 2023-10-24
### Changed
- Update to latest loop runner code from kit-sdk and set default settings

## [1.0.2] - 2023-06-12
### Changed
- Update to kit 105.1, pybind11 header locations

## [1.0.1] - 2023-05-19
### Fixed
- Crash on exit due to threading race condition

## [1.0.0] - 2022-09-29
### Changed
-   set_runner_dt to set_manual_step_size
-   setting the dt does not enable manual mode, need to call set_manual_mode(True)
### Added
-   set_manual_mode to enable/disable manual dt during runtime. 


## [0.1.0] - 2021-07-09
### Added
-   Initial Isaac RunLoop runner implementation
