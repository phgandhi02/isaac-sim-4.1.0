# Changelog

## [3.0.1] - 2024-04-25

### Changed

- Upgraded Lula from release 0.10.0 to 0.10.1.  This fixes a bug in the global planner interface introduced in 0.10.0 that caused a fatal error to be logged when a path couldn't be found.

## [3.0.0] - 2024-04-15

### Changed

- Upgraded Lula from release 0.9.1 to 0.10.0.  This new version includes a much-improved collision sphere generator and enhancements to the task-space global planner (JT-RRT), including support for full-pose targets and improved performance.  The global planner enhancements entail some parameter changes that break backward compatibility, as does a generalization of the kinematics API to support cases where only a subset of c-space coordinates have acceleration and/or jerk limits.

## [2.1.0] - 2023-08-22

### Changed

- Upgraded Lula from release 0.9.0 to release 0.9.1.  This adds support for prefixing Lula log messages with a user-provided string and also improves behavior of the IK solver for revolute joints that have a large span between their upper and lower joint limits (i.e., larger than 2 pi).
- Added "[Lula]" prefix to all Lula log messages.

## [2.0.0] - 2023-08-06

### Changed

- Upgraded Lula from release 0.8.2 to release 0.9.0.  This new version includes a major overhaul of inverse kinematics (IK), improving performance and robustness via many performance optimizations and the addition of a BFGS solver used to refine the solution produced by the existing cyclic coordinate descent (CCD) solver.  The names of certain parameters in the `CyclicCoordDescentIkConfig` struct have changed, breaking backward compatibility.  This update also includes the fix for a bug that could affect CCD convergence and the addition of "mimic joint" support throughout.

## [1.4.1] - 2023-04-27

### Fixed

- Added a workaround for a DLL loading issue under Windows that manifested when using the new "FastImporter" in Omniverse Kit 105.

## [1.4.0] - 2023-02-23

### Changed

- Upgraded Lula from release 0.8.1 to release 0.8.2.  This fixes a bug in the trajectory generator's task-space path conversion that could result in suboptimal interpolation of orientations.  In addition, a new option was added to the trajectory generator allowing user specification of time values at waypoints.

## [1.3.1] - 2022-11-30

### Changed

- Upgraded Lula from release 0.8.0 to release 0.8.1, fixing a couple minor bugs and adding a `__version__` string to the Lula python module.

## [1.3.0] - 2022-11-16

### Changed

- Upgraded Lula from release 0.7.1 to 0.8.0.  Among other improvements, this adds a flexible trajectory generator and collision sphere generator.  In addition, the robot description file format has been simplified, with "root_link" and "cspace_to_urdf_rules" now optional and "composite_task_spaces" and "subtree_root_link" removed/ignored.

## [1.2.1] - 2022-10-09

### Fixed

- Issue where linux version of extension was being loaded on windows

## [1.2.0] - 2022-10-06

### Changed

- Changed default log level to WARNING

## [1.1.0] - 2022-04-16

### Changed

- Removed wheel from extension, provide installed wheel as part of extension instead. This removes the need for runtime installation.

## [1.0.0] - 2022-01-13

### Changed

- Updated Lula from release 0.7.0 to 0.7.1.  This fixes a bug in Lula's kinematics that had the potential to cause a segfault for certain robots.

## [0.1.0] - 2021-09-20

### Added

- Initial version of Isaac Sim Lula Extension
