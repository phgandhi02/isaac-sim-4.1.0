# Changelog
## [1.7.0] - 2024-03-14
### Fixed
- Fixed code for handling Articulation selection in Configuration Tooling Template. The issue arose from a change in behavior in Core get_prim_object_type()

## [1.6.0] - 2024-02-28
### Fixed
- Fixed code for populating drop-down menu with Articulations in Configuration Tooling Template. This issue arose with Kit behavior changes on stage and timeline events.

## [1.5.2] - 2023-12-02
### Changed
- Updated path to the nucleus extension

## [1.5.1] - 2023-11-13
### Fixed
- Updated documentation link

## [1.5.0] - 2023-10-27
### Added
- Add Scripting Template to demonstrate programming script-like behavior through a standalone UI extension.

### Changed
- Modifies extension.py to massively reduce code duplication.

## [1.4.1] - 2023-08-24
### Fixed
- Fixed bug in Template generator on Windows where user could specify a file path ending in "/".

## [1.4.0] - 2023-08-15
### Fixed
- Fixed bug in Loaded Scenario Template where XFormPrim no longer accepts an Sdf.Path

## [1.3.1] - 2023-08-08
### Fixed
- Switch from ui.Window to ScrollingWindow wrapper for extensions because scrolling was broken

## [1.3.0] - 2023-06-26
### Fixed
- Fixed bug where Configuration Tooling Template could cause a crash in Kit 105.1
- The crash was caused by attempting to initialize Articulation on all stage events. The fix is to subscribe specifically to the ASSETS_LOADED stage event.
- Subscriptions to STOP/PLAY timeline events in the Configuration Tooling Template have been removed, as they didn't acheive anything when running Kit 105.1

## [1.2.0] - 2023-05-03
### Added
- Added limitations to special characters that are considered acceptible in an extension title
- Added XYPlot to "UI Component Library Template"

### Fixed
- Extension templates now use the extension name to generate python module paths to prevent clashing python modules. Previously, all modules were named "scripts" and you could not enable multiple extension templates at once.

## [1.1.0] - 2023-03-17
### Added
- Added "Status Frame" to Extension Template Generator to give feedback to the user to guide their usage. Feedback includes verification that their templates were generated properly.

## [1.0.0] - 2023-02-28
### Changed
- Updated Templates along with breaking changes to omni.isaac.ui replacing UIFrameWrapper with CollapsableFrame instance of UIElementWrapper
- Updated Configuration Tooling Template to use rebuild() function of a CollapsableFrame instead of pre-allocating 100 invisible frames.

### Added
- Added "UI Component Library Template" to show the usage of each UIElementWrapper

## [0.3.1] - 2023-02-17
### Fixed
- Fixed UI bug in Template Generator that allowed unnamed templates to be created

## [0.3.0] - 2023-02-15
### Added
- Added "Configuration Tooling Template" which allows the user to interact with a loaded robot Articulation through the UI.

### Removed
- Removed "Async Scenario Template" since it doesn't stand out well from "Loaded Scenario Template"

## [0.2.0] - 2023-02-13
### Added
- Added "Loaded Scenario Template" which has a load and reset button that are connected to Core.

## [0.1.3] - 2023-02-01
### Fixed
- Async Scenario Template "Run Scenario" button resets properly

## [0.1.2] - 2023-01-10
### Fixed
- Async Scenario Template now disables buttons until an Articulation is selected

## [0.1.1] - 2023-01-06
### Fixed
- onclick_fn warning when creating UI

## [0.1.0] - 12-09-2022

### Added

- Initial version of Extension Template Generator
